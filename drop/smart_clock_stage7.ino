// =====================================================================
//  Smart Clock v2  -  STAGE 7: reminders, rain warning, breathing guide
//  (plus stage 6 messages and stage 5 updates over WiFi / health reports)
//  Board: Seeed XIAO ESP32-C6        Serial Monitor: 115200 baud
//  Keep these three files together in the sketch folder:
//    smart_clock_stage7.ino   partitions.csv   clock_support.h
//  partitions.csv gives the clock two 1.9 MB program slots: it downloads a new version into the
//  spare slot, checks it, then switches. If the new version misbehaves it switches back by itself.
//  (Tools > Partition Scheme can stay on "Huge APP" - the partitions.csv file overrides it.)
//
//  Libraries (Arduino IDE > Tools > Manage Libraries...):
//    - GxEPD2       by Jean-Marc Zingg   (say YES to "install all" -> adds Adafruit GFX)
//    - ArduinoJson  by Benoit Blanchon   (version 7)
//
//  SETTINGS PAGE: hold a hand over the sensor window for 5 seconds -> the screen shows a
//  QR code. Point the phone camera at it and tap the link (works on Android and iPhone).
//  Also shown for 1 minute after start-up, and at  http://smartclock.local  on iPhones/computers.
//  Everything is on that page: LED effects, colour, brightness, gestures,
//  night mode, setting the hands, screen rotation, F/C, zip code.
//
//  The Serial Monitor commands from stage 2 still work too:
//    s  status    h  hold/run hands    c 10:08:30  hands show this time
//    ] [ } {  nudge 1 s / 1 min    t  speed test    speed 50    ip  settings address
//    r  rotate screen   u  update screen   f  F/C   w  forget WiFi   + / -  pulse
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <time.h>
#include <ArduinoJson.h>
#include <algorithm>
#include <GxEPD2_BW.h>
#include <ESPmDNS.h>
#include "qrcode.h"          // QR code encoder that comes with the ESP32 board package
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#include <mbedtls/sha256.h>
#include "clock_support.h"   // trusted root certificates for GitHub and ntfy.sh + the update safety hook
#if __has_include("build_info.h")
#include "build_info.h"      // written by the GitHub build: FW_BUILD and OTA_BASE_DEFAULT
#endif
#ifndef FW_BUILD
#define FW_BUILD 0           // 0 = built on a computer in Arduino IDE
#endif
#ifndef OTA_BASE_DEFAULT
#define OTA_BASE_DEFAULT ""  // the GitHub Pages address; can also be typed in on the Setup tab
#endif
#define FW_SERIES 7
struct Reminder { char label[44]; uint8_t h, m, days; uint16_t year; uint8_t mon, mday; bool on; uint32_t lastFire; };   // days: bit 0 = Sunday; 0 = one date
struct ClockMsg { String text; uint32_t at; bool seen; bool hearted; };   // a message shown on the clock
struct NtfyMsg { String title, body, tags; int prio; };   // a message for the phone (ntfy)

// fonts (bitmaps are at the bottom of this file)
extern const GFXfont BigTime, Label27, Small13;
void drawQR(const char *title, const char *payload, const char *line1, const char *line2, bool full);
bool stage5Command(const String &cmd);
bool msgGesture(int g);
bool msgLedOverride();
void renderHeartbeat(uint32_t t);
extern volatile uint32_t heartFlashUntil;
extern volatile bool remoteClear;
int stage7Override();
void renderStage7(int ov, uint32_t t);
bool stage7Gesture(int g);
void parseRainSoon(JsonDocument &doc);
void startBreathing();
extern bool rainSoon;
extern int rainHour;
void addMessage(const String &text, uint32_t at);
String cleanText(const String &in);
void pollMessages();
void msgLoop();
void drawMsgScreen(bool full);
const char *conditionText(int c);

// ---------------- pins (from the main board) ----------------
const int PIN_EPD_CS = D3, PIN_EPD_DC = D0, PIN_EPD_RST = D6, PIN_EPD_CLK = D8, PIN_EPD_DIN = D10;
const int PIN_COIL_A = D1, PIN_COIL_B = D2;
const int PIN_LED_GATE = D9, PIN_LED_DATA = D7;

// Waveshare 1.54" V2 (SSD1681), BUSY not connected (-1)
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, -1));

Preferences prefs;
WebServer server(80);
DNSServer dns;

// ---------------- settings (saved in flash) ----------------
String wifiSsid, wifiPass, zipCode, placeName;
float lat = 0, lon = 0;
bool useF = true;
uint8_t rotation = 1;
volatile uint32_t pulseMs = 11;

// ---------------- live data ----------------
struct Weather {
  bool ok = false;
  float temp = 0, hi = 0, lo = 0;
  int rain = 0, code = 0;
  bool isDay = true;
  long utcOffset = 0;
} wx;
float roomC = NAN, roomRH = NAN;
bool setupMode = false;
uint32_t lastWeather = 0, lastDraw = 0, drawCount = 0;
String lastDrawnKey;

// =====================================================================
//  HANDS
//  The movement only knows "one more step": 16 steps = 1 second of hand
//  movement. handPos counts steps around the 12-hour dial (0 = 12:00:00),
//  so the clock always knows what time the hands show. A task compares
//  that with the real time 1000 times a second and:
//    - steps exactly in time when they match (16 per second, smooth sweep)
//    - steps fast (fastMs per step) when the hands are behind
//    - waits, or laps forward, whichever is quicker, when they are ahead
//  The position is kept in RTC memory (survives resets and reflashing) and
//  saved to flash once a minute (survives power loss).
// =====================================================================
const int32_t CYCLE = 12L * 3600 * 16;        // steps in 12 hours = 691200
volatile int32_t handPos = 0;                 // what the hands show, in steps
volatile int32_t posRequest = -1;             // set handPos from the main loop (applied by the task)
volatile bool handsHold = false, lowBattery = false, tzReady = false;
volatile uint16_t fastMs = 50;                // ms per step while catching up
volatile int testRequest = 0;                 // 1 = run the speed test
RTC_NOINIT_ATTR uint32_t rtcMagic, rtcPos, rtcCheck;

int32_t wrapPos(int32_t p) { p %= CYCLE; return p < 0 ? p + CYCLE : p; }

// where the hands should be right now (false if the clock does not know the time yet)
bool targetPos(int32_t &target) {
  if (!tzReady) return false;
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1700000000) return false;    // clock not set yet
  struct tm t;
  localtime_r(&tv.tv_sec, &t);
  int32_t secs = (t.tm_hour % 12) * 3600 + t.tm_min * 60 + t.tm_sec;
  target = secs * 16 + tv.tv_usec / 62500;
  return true;
}

void oneStep(uint32_t pulse) {
  int pin = (handPos & 1) ? PIN_COIL_A : PIN_COIL_B;    // direction alternates with every step
  digitalWrite(pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(pulse));
  digitalWrite(pin, LOW);
  handPos = wrapPos(handPos + 1);
  rtcPos = handPos; rtcCheck = handPos ^ 0xA5A5A5A5; rtcMagic = 0xC10C2026;
}

void speedTest() {
  const uint16_t periods[] = {50, 40, 31, 25, 20, 16, 13};
  Serial.println("\nSPEED TEST: the second hand does one full turn at each speed.");
  Serial.println("Watch where it starts. After each turn it pauses 5 s: did it land EXACTLY where it started?");
  vTaskDelay(pdMS_TO_TICKS(3000));
  for (uint16_t p : periods) {
    Serial.printf("  %2u ms per step (%.1fx normal speed)... ", p, 62.5f / p);
    uint32_t pulse = min<uint32_t>((uint32_t)pulseMs, p - 4);
    TickType_t last = xTaskGetTickCount();
    for (int i = 0; i < 960; i++) { oneStep(pulse); vTaskDelayUntil(&last, pdMS_TO_TICKS(p)); esp_task_wdt_reset(); }
    Serial.println("done - same spot?");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  Serial.println("Test finished. Type  speed NN  with the fastest NN that landed exactly (e.g. speed 25).");
  Serial.println("The test moved the hands about 7 minutes ahead, so they will now pause ~7 minutes to let the real time catch up.");
  Serial.println("If any speed missed, the hands are now off by a little: fix with  c HH:MM:SS  or  ] [ } {");
}

void handsTask(void *) {
  uint32_t lastStep = 0, lastFree = millis();
  esp_task_wdt_add(nullptr);                     // if this task ever froze, the watchdog restarts the clock
  for (;;) {
    esp_task_wdt_reset();
    if (posRequest >= 0) { handPos = wrapPos(posRequest); posRequest = -1; }
    if (testRequest) { speedTest(); testRequest = 0; }
    uint32_t now = millis();
    int32_t target;
    if (handsHold || lowBattery) {
      lastFree = now;
    } else if (!targetPos(target)) {
      // time not known yet: just run like a normal clock (16 steps per second)
      if (now - lastFree >= 62 + ((handPos & 1) ? 1 : 0)) { lastFree = now; oneStep(pulseMs); lastStep = now; }
    } else {
      int32_t d = target - handPos;                    // steps the hands are behind (+) or ahead (-)
      if (d > CYCLE / 2) d -= CYCLE;
      if (d < -CYCLE / 2) d += CYCLE;
      bool go = false;
      uint32_t gap = pulseMs + 4;                     // never faster than this
      if (d > 0) {
        go = true;
        if (d > 2) gap = max<uint32_t>(gap, (uint32_t)fastMs);  // catching up
      } else if (d < 0) {
        // ahead: waiting takes -d/16 seconds; lapping forward takes (CYCLE+d) steps at fastMs,
        // gaining (1 - fastMs/62.5) of a step per step. Pick the quicker one.
        float waitS = -d / 16.0f;
        float gainPerStep = 1.0f - fastMs / 62.5f;
        float lapS = gainPerStep > 0 ? (CYCLE + d) / gainPerStep * fastMs / 1000.0f : 1e9f;
        if (lapS < waitS) { go = true; gap = max<uint32_t>(gap, (uint32_t)fastMs); }
      }
      if (go && now - lastStep >= gap) {
        lastStep = now;
        oneStep(gap > fastMs ? pulseMs : min<uint32_t>((uint32_t)pulseMs, gap - 4));
      }
      lastFree = now;
    }
    vTaskDelay(1);
  }
}

String handTimeText(int32_t p) {
  int32_t s = p / 16;
  char b[24];
  snprintf(b, sizeof b, "%d:%02d:%02d", (s / 3600) == 0 ? 12 : s / 3600, (s / 60) % 60, s % 60);
  return b;
}

// =====================================================================
//  INA226: battery voltage and current (minus = charging)
// =====================================================================
int16_t inaRead(uint8_t reg) {
  Wire.beginTransmission(0x40);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(0x40, 2) < 2) return 0;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
float batteryVolts() { return (uint16_t)inaRead(0x02) * 0.00125f; }
float batteryMilliamps() { return inaRead(0x01) * 0.0025f / 0.1f; }

// =====================================================================
//  LIGHT + GESTURE SETTINGS (saved together as one block in flash)
// =====================================================================
enum Action : uint8_t { A_NONE, A_TOGGLE, A_NEXT, A_PREV, A_BRIGHTER, A_DIMMER, A_WEATHER, A_QR, A_BREATHE, A_COUNT };
const char *ACTION_NAMES[A_COUNT] = {"Do nothing", "LEDs on / off", "Next effect", "Previous effect", "Brighter", "Dimmer", "Show weather", "Show settings QR code", "Breathing guide"};
enum Gesture : uint8_t { G_LEFT, G_RIGHT, G_UP, G_DOWN, G_TAP, G_HOLD, G_HOLD5, G_DOUBLE, G_COUNT };
const char *GESTURE_NAMES[G_COUNT] = {"Swipe left", "Swipe right", "Swipe up", "Swipe down", "Tap", "Hold 2 s", "Hold 5 s", "Double tap"};
const char *GESTURE_SHORT[G_COUNT] = {"LEFT", "RIGHT", "UP", "DOWN", "TAP", "HOLD 2s", "HOLD 5s", "DOUBLE TAP"};
enum Effect : uint8_t { E_GLOW, E_BREATHE, E_RAINBOW, E_HANDS, E_COMET, E_TWINKLE,
                        E_FERRIS, E_ORBIT, E_OCEAN, E_AURORA, E_CANDLE, E_WEATHERC, E_COUNT };
const char *EFFECT_NAMES[E_COUNT] = {"Warm glow", "Breathing", "Slow rainbow", "Hour + minute markers", "Spinning comet", "Twinkle",
                                     "Ferris wheel", "Second-hand orbit", "Ocean", "Aurora", "Candle", "Weather colours"};
const uint16_t ALL_EFFECTS = (1 << E_COUNT) - 1;

// SAFETY: the ring runs from the battery side. 500 mA is well inside what the battery,
// its protection circuit, the INA226 shunt and the JST plugs can carry, keeps the LEDs
// cool in the printed groove, and can't be raised from the settings page.
const uint16_t MAX_SAFE_MA = 500;

struct LightConfig {
  uint8_t version = 6;
  uint16_t ledCount = 121;
  uint8_t ledStartHour = 6;        // clock position (1..12) where the first LED sits
  bool ledClockwise = true;        // strip runs clockwise from the first LED
  uint16_t maxmA = 400;            // current limit for the whole ring (never above MAX_SAFE_MA)
  uint8_t effect = E_GLOW;
  uint16_t effectMask = ALL_EFFECTS; // which effects "next / previous" cycles through
  uint32_t color = 0xFF8C3C;       // warm amber
  uint8_t brightness = 110;        // 10..255
  uint16_t onSeconds = 30;         // how long the ring stays on (max 1 hour)
  bool nightOn = false;
  uint8_t nightFrom = 22, nightTo = 7;
  uint8_t gestureAction[G_COUNT] = {A_PREV, A_NEXT, A_BRIGHTER, A_DIMMER, A_TOGGLE, A_WEATHER, A_QR, A_NEXT};
  bool wakeOnAny = true;           // any gesture turns the LEDs on when they are off
  uint8_t sensorRotate = 0;        // 0..3 = 0/90/180/270 degrees
  bool sensorMirror = false;
  uint8_t sensitivity = 2;         // 1 low, 2 medium, 3 high
  bool hourChime = true;           // sparkle round the ring on the hour (only while the light is on)
} cfg;

struct LightConfigV5 {      // layout used by the first stage 4b
  uint8_t version = 5; uint16_t ledCount; uint8_t ledStartHour; bool ledClockwise; uint16_t maxmA;
  uint8_t effect; uint8_t effectMask; uint32_t color; uint8_t brightness; uint16_t onSeconds;
  bool nightOn; uint8_t nightFrom, nightTo; uint8_t gestureAction[G_COUNT]; bool wakeOnAny;
  uint8_t sensorRotate; bool sensorMirror; uint8_t sensitivity;
};

struct LightConfigV4 {      // layout used by stage 3b / 4, so saved choices carry over
  uint8_t version; uint16_t ledCount; uint8_t ledStartHour; bool ledClockwise; uint16_t maxmA;
  uint8_t effect, effectMask; uint32_t color; uint8_t brightness; uint16_t onSeconds;
  bool nightOn; uint8_t nightFrom, nightTo; uint8_t gestureAction[7]; bool wakeOnAny;
  uint8_t sensorRotate; bool sensorMirror; uint8_t sensitivity;
};
void loadLightConfig() {
  prefs.begin("clock", true);
  size_t len = prefs.getBytesLength("light");
  if (len == sizeof(LightConfig)) {
    LightConfig tmp;
    prefs.getBytes("light", &tmp, sizeof tmp);
    if (tmp.version == 6) cfg = tmp;
  } else if (len == sizeof(LightConfigV5)) {
    LightConfigV5 o;
    prefs.getBytes("light", &o, sizeof o);
    if (o.version == 5) {
      cfg.ledCount = o.ledCount; cfg.ledStartHour = o.ledStartHour; cfg.ledClockwise = o.ledClockwise; cfg.maxmA = o.maxmA;
      cfg.effect = o.effect; cfg.color = o.color; cfg.brightness = o.brightness; cfg.onSeconds = o.onSeconds;
      cfg.effectMask = o.effectMask | (ALL_EFFECTS & ~0x3F);      // keep their choices, new effects switched on
      cfg.nightOn = o.nightOn; cfg.nightFrom = o.nightFrom; cfg.nightTo = o.nightTo; cfg.wakeOnAny = o.wakeOnAny;
      cfg.sensorRotate = o.sensorRotate; cfg.sensorMirror = o.sensorMirror; cfg.sensitivity = o.sensitivity;
      for (int i = 0; i < G_COUNT; i++) cfg.gestureAction[i] = o.gestureAction[i];
    }
  } else if (len == sizeof(LightConfigV4)) {
    LightConfigV4 o;
    prefs.getBytes("light", &o, sizeof o);
    if (o.version == 4) {
      cfg.ledCount = o.ledCount; cfg.ledStartHour = o.ledStartHour; cfg.ledClockwise = o.ledClockwise; cfg.maxmA = o.maxmA;
      cfg.effect = o.effect; cfg.effectMask = o.effectMask | (ALL_EFFECTS & ~0x3F); cfg.color = o.color; cfg.brightness = o.brightness; cfg.onSeconds = o.onSeconds;
      cfg.nightOn = o.nightOn; cfg.nightFrom = o.nightFrom; cfg.nightTo = o.nightTo; cfg.wakeOnAny = o.wakeOnAny;
      cfg.sensorRotate = o.sensorRotate; cfg.sensorMirror = o.sensorMirror; cfg.sensitivity = o.sensitivity;
      for (int i = 0; i < 7; i++) cfg.gestureAction[i] = o.gestureAction[i];
      cfg.gestureAction[G_TAP] = A_TOGGLE; cfg.gestureAction[G_DOUBLE] = A_NEXT;   // sensible taps for the 9930
    }
  }
  prefs.end();
  cfg.ledCount = constrain(cfg.ledCount, 1, 300);
  cfg.maxmA = constrain(cfg.maxmA, 100, MAX_SAFE_MA);
  if (cfg.onSeconds == 0 || cfg.onSeconds > 3600) cfg.onSeconds = 3600;
  if (cfg.effect >= E_COUNT) cfg.effect = E_GLOW;
  cfg.effectMask &= ALL_EFFECTS; if (!cfg.effectMask) cfg.effectMask = 1;
}
void saveLightConfig() { prefs.begin("clock", false); prefs.putBytes("light", &cfg, sizeof cfg); prefs.end(); }

// =====================================================================
//  LED RING (WS2812B, 121 LEDs at 144/m) - its own task, 50 frames/s
//  The strip's ground is switched by the MOSFET on D9, so when the LEDs
//  are off they use no power at all. Data on D7 goes high-impedance then,
//  so the strip can't be powered through the data line.
// =====================================================================
const int MAX_LEDS = 300;
uint8_t px[MAX_LEDS * 3];                      // R,G,B per LED
rmt_data_t rmtBuf[MAX_LEDS * 24];
volatile bool ledsWanted = false;              // should the ring be on
volatile uint32_t ledsLastActivity = 0;
volatile bool ledsLocate = false;              // "show where LED 1 is" test
volatile bool ledBatteryRest = false;          // battery getting low: LEDs off so the hands keep running
bool ledsPowered = false;
float ledLevel = 0;                            // fade level 0..1

bool ledsAllowed(String *why = nullptr) {
  if (lowBattery || ledBatteryRest) { if (why) *why = "battery low - plug in USB"; return false; }
  if (cfg.nightOn) {
    struct tm t;
    if (getLocalTime(&t, 0)) {
      int h = t.tm_hour;
      bool night = cfg.nightFrom > cfg.nightTo ? (h >= cfg.nightFrom || h < cfg.nightTo) : (h >= cfg.nightFrom && h < cfg.nightTo);
      if (night) { if (why) *why = "night mode"; return false; }
    }
  }
  return true;
}

void ledsPowerOn() {
  if (ledsPowered) return;
  digitalWrite(PIN_LED_GATE, HIGH);
  delay(5);
  rmtInit(PIN_LED_DATA, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);   // 0.1 us ticks
  ledsPowered = true;
}
void ledsSend(float scale);
void ledsPowerOff() {
  if (!ledsPowered) return;
  memset(px, 0, sizeof px);
  ledsSend(0);
  delay(2);
  rmtDeinit(PIN_LED_DATA);
  pinMode(PIN_LED_DATA, INPUT);
  digitalWrite(PIN_LED_GATE, LOW);
  ledsPowered = false;
}

// send px[] to the strip, scaled by brightness, fade and the current limit
void ledsSend(float scale) {
  int n = cfg.ledCount;
  uint32_t sum = 0;
  for (int i = 0; i < n * 3; i++) sum += px[i];
  // about 15 mA per colour channel at full, plus ~0.5 mA per LED just for being powered
  float estmA = sum / 255.0f * 15.0f * scale;
  float budget = min(cfg.maxmA, MAX_SAFE_MA) - n * 0.5f;
  if (estmA > budget && estmA > 0) scale *= budget / estmA;
  int k = 0;
  for (int i = 0; i < n; i++) {
    uint8_t g = px[i * 3 + 1] * scale, r = px[i * 3] * scale, b = px[i * 3 + 2] * scale;
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;      // WS2812B wants green, red, blue
    for (int bit = 23; bit >= 0; bit--) {
      bool one = grb & (1UL << bit);
      rmtBuf[k].level0 = 1; rmtBuf[k].duration0 = one ? 8 : 4;     // 0.8 / 0.4 us high
      rmtBuf[k].level1 = 0; rmtBuf[k].duration1 = one ? 5 : 9;     // 0.5 / 0.9 us low
      k++;
    }
  }
  rmtWrite(PIN_LED_DATA, rmtBuf, k, 100);
}

void setPx(int i, uint8_t r, uint8_t g, uint8_t b) {
  int n = cfg.ledCount;
  i %= n; if (i < 0) i += n;
  px[i * 3] = r; px[i * 3 + 1] = g; px[i * 3 + 2] = b;
}
void addPx(int i, uint8_t r, uint8_t g, uint8_t b, float f) {
  int n = cfg.ledCount;
  i %= n; if (i < 0) i += n;
  px[i * 3] = min(255, px[i * 3] + (int)(r * f));
  px[i * 3 + 1] = min(255, px[i * 3 + 1] + (int)(g * f));
  px[i * 3 + 2] = min(255, px[i * 3 + 2] + (int)(b * f));
}
void hsv(uint8_t h, uint8_t &r, uint8_t &g, uint8_t &b) {   // full saturation/value colour wheel
  uint8_t region = h / 43, rem = (h - region * 43) * 6;
  uint8_t q = 255 - rem, t = rem;
  switch (region) {
    case 0: r = 255; g = t; b = 0; break;
    case 1: r = q; g = 255; b = 0; break;
    case 2: r = 0; g = 255; b = t; break;
    case 3: r = 0; g = q; b = 255; break;
    case 4: r = t; g = 0; b = 255; break;
    default: r = 255; g = 0; b = q; break;
  }
}
// LED index (as a float) for a clock angle in degrees (0 = 12 o'clock, clockwise)
float ledAt(float deg) {
  float startDeg = (cfg.ledStartHour % 12) * 30.0f;
  float rel = cfg.ledClockwise ? deg - startDeg : startDeg - deg;
  rel = fmodf(rel + 720.0f, 360.0f);
  return rel / 360.0f * cfg.ledCount;
}
void blob(float centre, float width, uint8_t r, uint8_t g, uint8_t b) {   // soft spot, smooth between LEDs
  for (int k = -(int)width - 1; k <= (int)width + 1; k++) {
    int i = (int)floorf(centre) + k;
    float d = fabsf(i - centre) / width;
    if (d < 1) addPx(i, r, g, b, (1 - d) * (1 - d));
  }
}

void renderEffect(uint32_t t) {
  int n = cfg.ledCount;
  uint8_t cr = cfg.color >> 16, cg = cfg.color >> 8, cb = cfg.color;
  if (ledsLocate) {                     // first LED red, the next 5 green: shows where the strip starts and which way it runs
    memset(px, 0, sizeof px);
    setPx(0, 255, 0, 0);
    for (int i = 1; i <= 5; i++) setPx(i, 0, 255 / i, 0);
    return;
  }
  switch (cfg.effect) {
    case E_GLOW:
      for (int i = 0; i < n; i++) setPx(i, cr, cg, cb);
      break;
    case E_BREATHE: {
      float f = 0.25f + 0.75f * (0.5f + 0.5f * sinf(t * 2 * PI / 5000.0f));
      for (int i = 0; i < n; i++) setPx(i, cr * f, cg * f, cb * f);
      break;
    }
    case E_RAINBOW:
      for (int i = 0; i < n; i++) { uint8_t r, g, b; hsv((i * 256 / n + t / 40) & 255, r, g, b); setPx(i, r, g, b); }
      break;
    case E_HANDS: {
      memset(px, 0, sizeof px);
      for (int i = 0; i < n; i++) setPx(i, cr / 16, cg / 16, cb / 16);       // faint ring
      for (int h = 0; h < 12; h++) blob(ledAt(h * 30), 1.2f, cr / 5, cg / 5, cb / 5);   // hour ticks
      struct timeval tv; gettimeofday(&tv, nullptr);
      struct tm lt; localtime_r(&tv.tv_sec, &lt);
      float s = lt.tm_sec + tv.tv_usec / 1e6f, m = lt.tm_min + s / 60, h = (lt.tm_hour % 12) + m / 60;
      blob(ledAt(h * 30), 4.5f, cr, cg, cb);                 // hour: wide, in your colour
      blob(ledAt(m * 6), 2.5f, 255, 255, 255);              // minute: white
      blob(ledAt(s * 6), 1.2f, 255, 40, 20);                // second: small red dot
      break;
    }
    case E_COMET: {
      for (int i = 0; i < n * 3; i++) px[i] = 0;
      float head = fmodf(t / 3500.0f, 1.0f) * n;
      int tail = n / 3;
      for (int k = 0; k < tail; k++) {
        float f = 1.0f - (float)k / tail; f *= f;
        addPx((int)head - (cfg.ledClockwise ? k : -k), cr, cg, cb, f);
      }
      break;
    }
    case E_TWINKLE:
      for (int i = 0; i < n * 3; i++) px[i] = px[i] * 0.94f;
      for (int k = 0; k < 2; k++) if (random(100) < 35) {
        int i = random(n);
        if (random(4) == 0) setPx(i, 255, 240, 220); else setPx(i, cr, cg, cb);
      }
      break;
    case E_FERRIS: {                       // 12 coloured gondola lights turning slowly, like a fairground wheel at night
      static const uint8_t GC[6][3] = {{255, 40, 30}, {255, 190, 20}, {40, 120, 255}, {40, 220, 90}, {255, 70, 180}, {255, 120, 20}};
      memset(px, 0, sizeof px);
      for (int i = 0; i < n; i++) setPx(i, 22, 12, 4);                       // warm rim
      float turn = fmodf(t / 60000.0f, 1.0f) * 360.0f;                       // one turn a minute
      for (int g = 0; g < 12; g++) {
        const uint8_t *c = GC[g % 6];
        float tw = 0.75f + 0.25f * sinf(t / 180.0f + g * 1.7f);              // bulbs shimmer a little
        blob(ledAt(g * 30 + turn), 3.0f, c[0] * tw, c[1] * tw, c[2] * tw);
        blob(ledAt(g * 30 + 15 + turn), 0.9f, 90, 70, 40);                   // small white bulbs between
      }
      break;
    }
    case E_ORBIT: {                        // a glowing dot that follows the real second hand, with a fading tail
      memset(px, 0, sizeof px);
      for (int i = 0; i < n; i++) setPx(i, cr / 20, cg / 20, cb / 20);
      float a = (handPos % 960) * 0.375f;                                    // where the second hand actually is
      for (int k = 0; k < 24; k++) {
        float f = 1.0f - k / 24.0f; f *= f;
        blob(ledAt(a - k * 2.0f), 1.3f, cr * f, cg * f, cb * f);
      }
      blob(ledAt(a), 1.6f, 255, 245, 230);
      break;
    }
    case E_OCEAN: {                        // blue-teal waves rolling round the ring, with a little foam on the crests
      for (int i = 0; i < n; i++) {
        float x = (float)i / n * 2 * PI;
        float w = 0.5f + 0.25f * sinf(3 * x - t / 1400.0f) + 0.25f * sinf(5 * x + t / 2300.0f);
        float foam = max(0.0f, w - 0.85f) * 4.0f;
        setPx(i, 10 + 60 * foam, 30 + 170 * w * w + 60 * foam, 120 + 90 * w + 50 * foam);
      }
      break;
    }
    case E_AURORA: {                       // slow drifting greens, teals and purples
      for (int i = 0; i < n; i++) {
        float x = (float)i / n * 2 * PI;
        float h = 0.5f + 0.5f * sinf(2 * x + t / 5200.0f) * cosf(x - t / 7700.0f);        // 0 green .. 1 purple
        float v = 0.35f + 0.65f * powf(0.5f + 0.5f * sinf(3 * x - t / 3100.0f + sinf(x + t / 9000.0f)), 2);
        uint8_t r = (h > 0.6f ? (h - 0.6f) * 2.5f * 170 : 0), g = 230 * (1 - h) + 20, b = 60 + 170 * h;
        setPx(i, r * v, g * v, b * v);
      }
      break;
    }
    case E_CANDLE: {                       // warm flicker, each part of the ring breathing on its own
      static float fl[MAX_LEDS];
      static bool init = false;
      if (!init) { for (int i = 0; i < MAX_LEDS; i++) fl[i] = 0.8f; init = true; }
      for (int i = 0; i < n; i++) {
        float target = 0.55f + 0.45f * (random(1000) / 1000.0f);
        if (random(400) == 0) target = 0.25f;                                // the odd dip, like a draught
        fl[i] += (target - fl[i]) * 0.12f;
      }
      for (int i = 0; i < n; i++) {
        float f = (fl[(i + n - 1) % n] + 2 * fl[i] + fl[(i + 1) % n]) / 4;  // neighbours flicker together
        setPx(i, 255 * f, (70 + 50 * f) * f, 8 * f);
      }
      break;
    }
    case E_WEATHERC: {                     // colour follows the outside temperature; rain adds falling drops
      uint8_t r = 255, g = 140, b = 40;                                      // warm amber until the weather arrives
      if (wx.ok) {
        float f = useF ? wx.temp : wx.temp * 9 / 5 + 32;
        static const float T[5] = {20, 40, 55, 80, 95};
        static const uint8_t C[5][3] = {{60, 110, 255}, {120, 200, 255}, {255, 190, 60}, {255, 110, 20}, {255, 40, 10}};
        if (f <= T[0]) { r = C[0][0]; g = C[0][1]; b = C[0][2]; }
        else if (f >= T[4]) { r = C[4][0]; g = C[4][1]; b = C[4][2]; }
        else for (int k = 0; k < 4; k++) if (f < T[k + 1]) {
          float u = (f - T[k]) / (T[k + 1] - T[k]);
          r = C[k][0] + (C[k + 1][0] - C[k][0]) * u; g = C[k][1] + (C[k + 1][1] - C[k][1]) * u; b = C[k][2] + (C[k + 1][2] - C[k][2]) * u;
          break;
        }
      }
      float br = 0.8f + 0.2f * sinf(t * 2 * PI / 6000.0f);
      for (int i = 0; i < n; i++) setPx(i, r * br, g * br, b * br);
      bool rain = wx.ok && ((wx.code >= 51 && wx.code <= 67) || (wx.code >= 80 && wx.code <= 82) || wx.code >= 95);
      if (rain) for (int d = 0; d < 4; d++) {                                // drops run down both sides, 12 -> 6
        float ph = fmodf(t / 2600.0f + d * 0.25f, 1.0f) * 180.0f;
        blob(ledAt(ph), 1.4f, 140, 190, 255);
        blob(ledAt(360 - ph), 1.4f, 140, 190, 255);
      }
      break;
    }
  }
  // hour chime: one sparkling sweep round the ring on the hour, over whatever effect is showing
  static uint32_t chimeAt = 0;
  static int lastHour = -1;
  if (cfg.hourChime && tzReady && time(nullptr) > 1700000000) {
    struct tm lt; time_t now = time(nullptr); localtime_r(&now, &lt);
    if (lastHour >= 0 && lt.tm_hour != lastHour && lt.tm_min == 0) chimeAt = t;
    lastHour = lt.tm_hour;
  }
  if (chimeAt && t - chimeAt < 3000) {
    float u = (t - chimeAt) / 2500.0f;
    if (u <= 1) {
      float a = u * 360.0f;
      blob(ledAt(a), 2.5f, 255, 240, 200);
      for (int k = 0; k < 3; k++) addPx((int)ledAt(a - random(60)), 255, 220, 160, 0.6f);
    }
  }
}

void ledTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    bool hb = !ledsLocate && msgLedOverride();                     // new-message heartbeat or "heart sent" burst
    bool flash = millis() < heartFlashUntil;
    int ov = ledsLocate ? 0 : stage7Override();                   // breathing / reminder / rain shimmer
    bool batOk = !(lowBattery || ledBatteryRest);
    bool want = ((ledsWanted || ledsLocate || hb) && ledsAllowed()) || (ov && batOk && (ov != 1 || ledsAllowed()));
    if (ledsWanted && !ledsLocate && millis() - ledsLastActivity > min<uint32_t>(cfg.onSeconds, 3600) * 1000UL) ledsWanted = false;
    float target = want ? 1.0f : 0.0f;
    if (target > ledLevel) ledLevel = min(1.0f, ledLevel + 0.04f);      // 0.5 s fade in
    else if (target < ledLevel) ledLevel = max(0.0f, ledLevel - 0.025f); // 0.8 s fade out
    if (ledLevel <= 0) { ledsPowerOff(); continue; }
    ledsPowerOn();
    bool hbDraw = hb && (flash || !ledsWanted);                    // her own light choice wins over the heartbeat
    if (ov) { renderStage7(ov, millis()); ledsSend(ledLevel * (ov == 2 ? 0.7f : 0.5f)); continue; }
    if (hbDraw) renderHeartbeat(millis()); else renderEffect(millis());
    ledsSend(ledLevel * (ledsLocate ? 0.5f : hbDraw ? (flash ? 0.6f : 0.45f) : cfg.brightness / 255.0f));
  }
}

// =====================================================================
//  GESTURES
//  Works with either sensor chip, detected at start-up:
//   - APDS-9930 (ID 0x39, what this clock has): distance only ->
//       tap, double tap, hold 2 s, hold 5 s
//   - APDS-9960 (ID 0xAB, a possible later upgrade): adds swipes
// =====================================================================
const uint8_t APDS = 0x39;
bool apdsOK = false;
bool apdsIs9960 = false;
volatile int lastGesture = -1;
volatile uint32_t lastGestureAt = 0;
volatile uint16_t lastProximity = 0;
volatile uint16_t proxBase = 0;          // background reading with no hand (window reflection)
volatile bool handNear = false;
uint8_t ledDrive9930 = 0;                // 0 = 100 mA ... 3 = 12.5 mA

// The sensor window sits at 6 o'clock, under the hands. The second hand (and its tail)
// sweeps over it twice a minute, and the minute/hour hands park over it for a while.
// So the 9930 keeps a "background map": what the sensor reads at each second-hand
// position with no person there. A gesture has to rise above that map, not just above
// one fixed number. Anything that stays in front for more than 8 s (a parked minute or
// hour hand) is absorbed into the background instead of counting as a 5-second hold.
const int BG_BINS = 120;                 // half-second bins around the dial
float bgMap[BG_BINS];
uint16_t bgSeen[BG_BINS];
uint32_t binTouch[BG_BINS], binPrevTouch[BG_BINS];
float bgOff = 0;                         // extra background from a parked minute/hour hand
bool bgReady = false;                    // first full sweep learned
uint32_t bgStart = 0;
volatile uint16_t proxRef = 0;           // current "no hand" level (shown on the settings page)
const uint32_t OBSTRUCT_MS = 8000;
int bgBin() { return (int)((handPos % 960) * BG_BINS / 960); }
float angDist(float a, float b) { float d = fabsf(fmodf(a - b + 540.0f, 360.0f) - 180.0f); return d; }
bool secondHandZone() {                 // second hand (or its tail) within 45 degrees of the sensor
  float sec = (handPos % 960) * 0.375f;
  return angDist(sec, 180) < 45 || angDist(sec, 0) < 45;
}
bool handOverSensor() {                  // any clock hand near 6 o'clock right now?
  float sec = (handPos % 960) * 0.375f, mn = (handPos % 57600) * (360.0f / 57600), hr = handPos * (360.0f / CYCLE);
  return angDist(sec, 180) < 30 || angDist(sec, 0) < 30 || angDist(mn, 180) < 25 || angDist(hr, 180) < 25;
}

bool apdsWrite(uint8_t reg, uint8_t v) { Wire.beginTransmission(APDS); Wire.write(reg); Wire.write(v); return Wire.endTransmission() == 0; }
int apdsRead(uint8_t reg) {
  Wire.beginTransmission(APDS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(APDS, (uint8_t)1) < 1) return -1;
  return Wire.read();
}
// APDS-9930 registers are addressed with the command bit 0x80 set, e.g. PDATA (0x18) -> 0x98
int prox9930() { int lo = apdsRead(0x98), hi = apdsRead(0x99); return (lo < 0 || hi < 0) ? -1 : (hi << 8) | lo; }

uint16_t marginFor() {
  if (apdsIs9960) return cfg.sensitivity >= 3 ? 12 : cfg.sensitivity == 2 ? 25 : 45;
  return cfg.sensitivity >= 3 ? 70 : cfg.sensitivity == 2 ? 150 : 300;     // 10-bit scale (0..1023)
}

void configure9930() {
  apdsWrite(0x80, 0x00);                         // ENABLE off
  apdsWrite(0x81, 0xF6);                         // ATIME ~27 ms (keeps distance readings quick)
  apdsWrite(0x82, 0xFF);                         // PTIME 2.7 ms
  apdsWrite(0x83, 0xFF);                         // WTIME
  apdsWrite(0x8E, 8);                            // 8 LED pulses
  apdsWrite(0x8F, (ledDrive9930 << 6) | 0x20);   // LED drive, CH1 photodiode (required), gains 1x
  apdsWrite(0x9E, 0x00);                         // no offset
  apdsWrite(0x80, 0x07);                         // power + light + distance
}

// measure the "no hand" background; for the 9930 also turn the LED down if the window reflects too much
void calibrateBaseline() {
  if (apdsIs9960) {
    apdsWrite(0x80, 0x00);
    apdsWrite(0x8E, 0x89); apdsWrite(0x8F, 0x0A); apdsWrite(0x90, 0x31);
    apdsWrite(0x80, 0x05);
    delay(60);
    int maxv = 0;
    for (int i = 0; i < 12; i++) { int p = apdsRead(0x9C); if (p > maxv) maxv = p; delay(15); }
    proxBase = maxv;
    return;
  }
  if (handOverSensor()) {                       // a clock hand is over the window: don't let it fool the calibration
    ledDrive9930 = 0; configure9930(); delay(120);
    proxBase = 60;
    Serial.println("Gesture sensor: a clock hand is over the window, using the standard setting");
  } else {
    for (ledDrive9930 = 0; ledDrive9930 < 4; ledDrive9930++) {
      configure9930();
      delay(120);
      int maxv = 0;
      for (int i = 0; i < 15; i++) { int p = prox9930(); if (p > maxv) maxv = p; delay(35); }
      proxBase = maxv;
      if (maxv < 550) break;                     // enough room above the background for a hand
    }
    if (ledDrive9930 > 3) { ledDrive9930 = 0; proxBase = 60; }   // something odd in front: use the standard setting
    configure9930();
  }
  for (int i = 0; i < BG_BINS; i++) { bgMap[i] = proxBase; bgSeen[i] = 0; binTouch[i] = binPrevTouch[i] = 0; }
  bgOff = 0; bgReady = false; bgStart = millis();
}

void apdsConfigure() {
  calibrateBaseline();
  uint16_t margin = marginFor();
  Serial.printf("Gesture sensor (%s): background %u, trigger at %u\n", apdsIs9960 ? "APDS-9960" : "APDS-9930", proxBase, proxBase + margin);
  if (!apdsIs9960) return;
  uint8_t enter = min(250, proxBase + margin);
  apdsWrite(0x80, 0x00);
  apdsWrite(0x8E, 0x89); apdsWrite(0x8F, 0x0A); apdsWrite(0x90, 0x31);
  apdsWrite(0xA0, enter);
  apdsWrite(0xA1, max<int>(proxBase + 4, enter - margin / 2));
  apdsWrite(0xA2, 0x40); apdsWrite(0xA3, 0x41);
  apdsWrite(0xA4, 0); apdsWrite(0xA5, 0); apdsWrite(0xA7, 0); apdsWrite(0xA9, 0);
  apdsWrite(0xA6, 0x89); apdsWrite(0xAA, 0x00); apdsWrite(0xAB, 0x00);
  apdsWrite(0x80, 0x47);
}

bool apdsBegin() {
  int id = apdsRead(0x92);                       // same address on both chips
  if (id < 0) { Serial.println("Gesture sensor not found"); return false; }
  apdsIs9960 = (id == 0xAB || id == 0xA8 || id == 0x9C);
  Serial.printf("Gesture sensor ID 0x%02X -> %s\n", id, apdsIs9960 ? "APDS-9960 (swipes + taps)" : "APDS-9930 (taps and holds)");
  apdsConfigure();
  return true;
}

// which gestures this sensor can do (shown on the settings page)
bool gestureSupported(int g) { return apdsIs9960 || g >= G_TAP; }

// ---- APDS-9930: taps and holds from the distance reading
// The background map is indexed by where the clock thinks the second hand is. Bins where the map is
// high are where a clock hand covers the window: nothing can be seen there, so events that happen
// mostly there are ignored. If something triggers at the same second-hand position one minute after
// the last time, it is a clock hand the map hadn't learned yet: it gets learned and ignored.
uint16_t binNearMax[BG_BINS];
bool binTouched[BG_BINS];
float mapBase = 0;                       // the clock's own "nothing there" level, from the map (most of the dial is clear)
void updateMapBase() {
  float tmp[BG_BINS];
  memcpy(tmp, bgMap, sizeof tmp);
  std::nth_element(tmp, tmp + BG_BINS / 3, tmp + BG_BINS);   // a third of the way up: clear dial, not the hand passes
  mapBase = tmp[BG_BINS / 3];
}
// a parked clock hand covers the window so completely there's no room left to see a person
bool sensorBlocked() { return apdsOK && !apdsIs9960 && bgReady && mapBase + bgOff + marginFor() > 1023; }
bool binHigh(int b) { b = (b + BG_BINS) % BG_BINS; return bgMap[b] > mapBase + marginFor() * 0.7f; }
int poll9930() {
  static uint32_t t0 = 0, lastNearEnd = 0, nearN = 0, nearHigh = 0;
  int p = prox9930();
  if (p < 0) return -1;
  lastProximity = p;
  uint32_t now = millis();
  int b = bgBin();
  // Each half-second of the second hand's lap is learned on its own the first time the hand goes
  // through it (at any speed - also while the hands are catching up or after they were paused).
  // Until a spot is learned, nothing that happens there counts as a gesture.
  if (bgSeen[b] < 20) {
    bgMap[b] += (p - bgMap[b]) * 0.3f;
    bgSeen[b]++;
    proxRef = (uint16_t)bgMap[b];
    if (handNear) { handNear = false; lastNearEnd = now; }          // a gesture can't straddle an unlearned spot
    if (!bgReady) {
      bool all = true;
      for (int i = 0; i < BG_BINS; i++) if (bgSeen[i] < 20) { all = false; break; }
      if (all) { bgReady = true; updateMapBase(); Serial.printf("Gesture sensor: learned the hands (clear-dial level %.0f)\n", mapBase); }
    }
    return -1;
  }
  if (!bgReady && now - bgStart > 62000) { bgReady = true; updateMapBase(); }   // hands paused: the learned part is enough
  static uint32_t lastBase = 0;
  if (!handNear && now - lastBase > 10000) { lastBase = now; updateMapBase(); }
  float ref = max(bgMap[b], max(bgMap[(b + 1) % BG_BINS], bgMap[(b + BG_BINS - 1) % BG_BINS])) + bgOff;
  proxRef = (uint16_t)min(ref, 1023.0f);
  uint16_t margin = marginFor();
  bool hb = binHigh(b) || binHigh(b + 1) || binHigh(b - 1);
  if (!handNear) {
    if (p > ref + margin) {
      handNear = true; t0 = now; nearN = nearHigh = 0;
      memset(binTouched, 0, sizeof binTouched);
    } else {
      if (now - lastNearEnd > 1500 || hb) {
        if (!hb && p < ref - margin / 3 && bgOff > 0) bgOff = max(0.0f, bgOff - max(2.0f, bgOff * 0.05f));  // parked hand moved away
        else if (hb || p - bgOff <= bgMap[b] + margin / 2) bgMap[b] += ((p - bgOff) - bgMap[b]) * 0.08f; // keep learning
      }
      return -1;
    }
  }
  nearN++; if (hb) { nearHigh++; bgMap[b] += ((p - bgOff) - bgMap[b]) * 0.08f; }
  if (!binTouched[b]) { binTouched[b] = true; binPrevTouch[b] = binTouch[b]; binTouch[b] = now; binNearMax[b] = 0; }
  binNearMax[b] = max<int>(binNearMax[b], (int)(p - bgOff));
  uint32_t dur = now - t0;
  if (dur > OBSTRUCT_MS) {                         // still there after 8 s: a parked clock hand, not a person
    bgOff += p - ref;
    handNear = false; lastNearEnd = now;
    Serial.printf("[gesture] something parked in front (reading %d) - ignored\n", p);
    return -1;
  }
  if (p < ref + margin / 2) {
    handNear = false; lastNearEnd = now;
    bool repeat = false;
    for (int i = 0; i < BG_BINS; i++)
      if (binTouched[i] && binPrevTouch[i] && binTouch[i] - binPrevTouch[i] >= 40000 && binTouch[i] - binPrevTouch[i] <= 130000) repeat = true;
    if (repeat) {
      for (int i = 0; i < BG_BINS; i++) if (binTouched[i]) bgMap[i] = max(bgMap[i], (float)binNearMax[i]);
      Serial.printf("[gesture] same spot as last lap - that's a clock hand (second %d), learned and ignored\n", b / 2);
      return -1;
    }
    if (nearHigh * 2 > nearN) { Serial.printf("[gesture] a clock hand is over the window (second %d) - ignored\n", b / 2); return -1; }
    if (dur >= 4500) return G_HOLD5;
    if (dur >= 1500) return G_HOLD;
    if (dur >= 40 && dur < 800) return G_TAP;
  }
  return -1;
}

// ---- APDS-9960: the gesture engine (swipes), plus taps and holds
int orientGesture(int g) {
  if (g > G_DOWN) return g;
  if (cfg.sensorMirror) { if (g == G_LEFT) g = G_RIGHT; else if (g == G_RIGHT) g = G_LEFT; }
  static const uint8_t cw[4] = {G_UP, G_DOWN, G_RIGHT, G_LEFT};
  for (int k = 0; k < cfg.sensorRotate; k++) g = cw[g];
  return g;
}
int poll9960() {
  static bool active = false;
  static uint32_t startMs = 0;
  static int fu, fd, fl, fr, lu, ld, ll, lr;
  static bool haveFirst = false;
  int gconf4 = apdsRead(0xAB);
  if (gconf4 < 0) return -1;
  bool inGesture = gconf4 & 0x01;
  int prox = apdsRead(0x9C);
  if (prox >= 0) lastProximity = prox;
  int level = apdsRead(0xAE);
  if (level > 0) {
    if (!active) { active = true; startMs = millis(); haveFirst = false; }
    level = min(level, 32);
    uint8_t buf[128];
    int got = 0;
    while (got < level * 4) {
      int chunk = min(level * 4 - got, 32);
      Wire.beginTransmission(APDS); Wire.write(0xFC); Wire.endTransmission(false);
      int r = Wire.requestFrom(APDS, (uint8_t)chunk);
      if (r <= 0) break;
      for (int i = 0; i < r; i++) buf[got + i] = Wire.read();
      got += r;
    }
    for (int i = 0; i + 3 < got; i += 4) {
      int u = buf[i], d = buf[i + 1], l = buf[i + 2], r = buf[i + 3];
      if (u > 10 && d > 10 && l > 10 && r > 10) {
        if (!haveFirst) { fu = u; fd = d; fl = l; fr = r; haveFirst = true; }
        lu = u; ld = d; ll = l; lr = r;
      }
    }
  }
  handNear = active;
  if (active && !inGesture && level <= 0) {
    active = false; handNear = false;
    uint32_t dur = millis() - startMs;
    if (dur > 4500) return G_HOLD5;
    if (dur > 1500) return G_HOLD;
    if (!haveFirst) return -1;
    int udFirst = (fu - fd) * 100 / max(1, fu + fd), lrFirst = (fl - fr) * 100 / max(1, fl + fr);
    int udLast = (lu - ld) * 100 / max(1, lu + ld), lrLast = (ll - lr) * 100 / max(1, ll + lr);
    int dUD = udLast - udFirst, dLR = lrLast - lrFirst;
    if (abs(dUD) < 13 && abs(dLR) < 13) return G_TAP;
    if (abs(dUD) > abs(dLR)) return orientGesture(dUD > 0 ? G_DOWN : G_UP);
    return orientGesture(dLR > 0 ? G_RIGHT : G_LEFT);
  }
  return -1;
}

// call every ~15 ms. Turns two quick taps into a double tap.
// After a tap ends, the clock waits DOUBLE_GAP_MS for the hand to come back.
// If it comes back in time, that second visit decides: a quick one = double tap,
// a long one = hold. If it doesn't come back, the single tap is reported.
const uint32_t DOUBLE_GAP_MS = 900;
int pollGesture() {
  static bool tapPending = false;
  static bool secondVisit = false;
  static uint32_t tapAt = 0, secondAt = 0;
  if (!apdsOK) return -1;
  bool wasNear = handNear;
  int g = apdsIs9960 ? poll9960() : poll9930();
  uint32_t now = millis();
  // hand came back while a tap is waiting -> this is the second visit
  if (tapPending && !wasNear && handNear && now - tapAt < DOUBLE_GAP_MS) {
    secondVisit = true; secondAt = now;
    Serial.printf("[gesture] hand back after %lu ms\n", (unsigned long)(now - tapAt));
  }
  if (g == G_TAP) {
    if (tapPending && secondVisit) { tapPending = secondVisit = false; return G_DOUBLE; }
    tapPending = true; secondVisit = false; tapAt = now;
    return -1;
  }
  if (g >= 0) { tapPending = secondVisit = false; return g; }
  if (tapPending && secondVisit && !handNear) {          // second visit was neither tap nor hold
    tapPending = secondVisit = false;
    if (now - secondAt >= 150) return G_DOUBLE;           // 0.8-1.5 s visit: still clearly two touches
    return G_TAP;                                          // a tiny flicker: treat as the single tap
  }
  if (tapPending && !secondVisit && !handNear && now - tapAt >= DOUBLE_GAP_MS) {
    tapPending = false; return G_TAP;
  }
  return -1;
}

// =====================================================================
//  SHT41 (indoor temperature / humidity)
// =====================================================================
bool readSHT41(float &tC, float &rh) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(0x44);
    Wire.write(0xFD);
    if (Wire.endTransmission() != 0) { delay(20); continue; }
    delay(12);
    if (Wire.requestFrom(0x44, 6) < 6) { delay(20); continue; }
    uint8_t d[6];
    for (int i = 0; i < 6; i++) d[i] = Wire.read();
    tC = -45.0f + 175.0f * ((d[0] << 8) | d[1]) / 65535.0f;
    rh = constrain(-6.0f + 125.0f * ((d[3] << 8) | d[4]) / 65535.0f, 0.0f, 100.0f);
    return true;
  }
  return false;
}

// =====================================================================
//  Settings
// =====================================================================
void loadSettings() {
  prefs.begin("clock", true);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  zipCode = prefs.getString("zip", "");
  placeName = prefs.getString("place", "");
  lat = prefs.getFloat("lat", 0);
  lon = prefs.getFloat("lon", 0);
  useF = prefs.getBool("useF", true);
  rotation = prefs.getUChar("rot", 1);
  pulseMs = prefs.getUChar("pulse", 11);
  fastMs = prefs.getUShort("fast", 50);
  if (prefs.isKey("utcoff")) { wx.utcOffset = prefs.getLong("utcoff", 0); tzReady = true; }
  prefs.end();
  loadLightConfig();
}
void saveSetting(const char *key, const String &v) { prefs.begin("clock", false); prefs.putString(key, v); prefs.end(); }

// =====================================================================
//  Internet: zip code -> place + lat/lon, then weather (+ time zone)
// =====================================================================
SemaphoreHandle_t &tlsLockRef();
bool httpGetJson(const String &url, JsonDocument &doc) {
  bool locked = false;
  if (tlsLockRef()) { if (xSemaphoreTake(tlsLockRef(), pdMS_TO_TICKS(20000)) != pdTRUE) return false; locked = true; }
  struct Unlock { bool on; ~Unlock() { if (on) xSemaphoreGive(tlsLockRef()); } } unlock{locked};
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  bool ok = false;
  if (code == 200) ok = !deserializeJson(doc, http.getString());
  else Serial.printf("HTTP %d for %s\n", code, url.c_str());
  http.end();
  return ok;
}

bool lookupZip() {
  JsonDocument doc;
  if (!httpGetJson("https://api.zippopotam.us/us/" + zipCode, doc)) return false;
  JsonObject p = doc["places"][0];
  if (p.isNull()) return false;
  lat = atof(p["latitude"] | "0");
  lon = atof(p["longitude"] | "0");
  placeName = String((const char *)(p["place name"] | "")) + ", " + String((const char *)(p["state abbreviation"] | ""));
  prefs.begin("clock", false);
  prefs.putFloat("lat", lat); prefs.putFloat("lon", lon); prefs.putString("place", placeName);
  prefs.end();
  Serial.printf("Zip %s = %s (%.3f, %.3f)\n", zipCode.c_str(), placeName.c_str(), lat, lon);
  return true;
}

bool fetchWeather() {
  if (lat == 0 && lon == 0 && !lookupZip()) return false;
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(lat, 4) + "&longitude=" + String(lon, 4) +
               "&current=temperature_2m,weather_code,is_day"
               "&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max"
               "&hourly=precipitation_probability&forecast_hours=4"
               "&timezone=auto&forecast_days=1&temperature_unit=" + String(useF ? "fahrenheit" : "celsius");
  JsonDocument doc;
  if (!httpGetJson(url, doc)) return false;
  wx.temp = doc["current"]["temperature_2m"] | 0.0f;
  wx.code = doc["current"]["weather_code"] | 0;
  wx.isDay = (doc["current"]["is_day"] | 1) == 1;
  wx.hi = doc["daily"]["temperature_2m_max"][0] | 0.0f;
  wx.lo = doc["daily"]["temperature_2m_min"][0] | 0.0f;
  wx.rain = doc["daily"]["precipitation_probability_max"][0] | 0;
  parseRainSoon(doc);
  long off = doc["utc_offset_seconds"] | 0L;
  if (off != wx.utcOffset || !wx.ok) {               // time zone / daylight saving change
    wx.utcOffset = off;
    configTime(off, 0, "pool.ntp.org", "time.nist.gov");
    prefs.begin("clock", false); prefs.putLong("utcoff", off); prefs.end();
    tzReady = true;
  }
  wx.ok = true;
  Serial.printf("Weather: %.0f, code %d, hi %.0f lo %.0f rain %d%%, UTC offset %ld h\n",
                wx.temp, wx.code, wx.hi, wx.lo, wx.rain, off / 3600);
  return true;
}

// =====================================================================
//  Drawing (fonts are in the font_*.h tabs; '`' prints a degree sign)
// =====================================================================
int textWidth(const char *s, const GFXfont *f) {
  int16_t x1, y1; uint16_t w, h;
  display.setFont(f);
  display.getTextBounds(s, 0, 100, &x1, &y1, &w, &h);
  return w + x1;
}
void centerText(const char *s, int cx, int baseline, const GFXfont *f) {
  display.setFont(f);
  display.setCursor(cx - textWidth(s, f) / 2, baseline);
  display.print(s);
}

void drawMessage(const char *l1, const char *l2, const char *l3, const char *l4) {
  display.setRotation(rotation);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    centerText(l1, 100, 48, &Label27);
    centerText(l2, 100, 100, &Small13);
    centerText(l3, 100, 130, &Small13);
    centerText(l4, 100, 160, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// Date on top, the time as big as it fits, inside temperature + humidity at the bottom
void drawMain(bool full) {
  struct tm t;
  bool haveTime = getLocalTime(&t, 50);
  char date[24] = "", hm[8] = "--:--", temp[12] = "--", hum[8] = "--";
  if (haveTime) {
    strftime(date, sizeof date, "%a, %b %e", &t);
    int h = t.tm_hour % 12; if (h == 0) h = 12;
    snprintf(hm, sizeof hm, "%d:%02d", h, t.tm_min);
  }
  if (!isnan(roomC)) {
    snprintf(temp, sizeof temp, "%.0f`%c", useF ? roomC * 9 / 5 + 32 : roomC, useF ? 'F' : 'C');
    snprintf(hum, sizeof hum, "%.0f%%", roomRH);
  }
  display.setRotation(rotation);
  if (full) display.setFullWindow(); else display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    centerText(date, 100, 30, &Label27);
    display.fillRect(14, 42, 172, 2, GxEPD_BLACK);
    centerText(hm, 100, 128, &BigTime);
    display.fillRect(14, 142, 172, 2, GxEPD_BLACK);
    centerText("INSIDE", 52, 162, &Small13);
    if (rainSoon) {                                    // rain warning takes the humidity corner
      char rh[10] = "SOON";
      if (rainHour >= 0) { int h12 = rainHour % 12; if (h12 == 0) h12 = 12; snprintf(rh, sizeof rh, "%d %s", h12, rainHour < 12 ? "AM" : "PM"); }
      strlcpy(hum, rh, sizeof hum);
      centerText("RAIN AT", 150, 162, &Small13);
    } else
    centerText("HUMIDITY", 150, 162, &Small13);
    centerText(temp, 52, 192, &Label27);
    centerText(hum, 150, 192, &Label27);
    display.fillRect(100, 152, 1, 42, GxEPD_BLACK);
  } while (display.nextPage());
  display.hibernate();
}

// =====================================================================
//  WiFi setup portal
// =====================================================================
String networkOptions;
const char PAGE_HEAD[] PROGMEM = "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>Smart Clock setup</title><style>body{font-family:sans-serif;max-width:420px;margin:24px auto;padding:0 16px;color:#2b1d12;background:#f6efe6}"
  "h1{font-size:22px}label{display:block;margin-top:14px;font-weight:600}input,select{width:100%;padding:10px;font-size:16px;margin-top:4px;box-sizing:border-box}"
  "button{margin-top:20px;width:100%;padding:12px;font-size:17px;background:#5a3a22;color:#fff;border:0;border-radius:6px}</style></head><body>";

void handleRoot() {
  String p = FPSTR(PAGE_HEAD);
  p += "<h1>Smart Clock setup</h1><form method=post action=/save>"
       "<label>WiFi network</label><select name=ssid>" + networkOptions + "</select>"
       "<label>...or type it</label><input name=ssid2 placeholder='only if not in the list'>"
       "<label>WiFi password</label><input name=pass type=password>"
       "<label>Zip code</label><input name=zip inputmode=numeric maxlength=5 value='" + zipCode + "'>"
       "<label>Temperature</label><select name=units><option value=F" + String(useF ? " selected" : "") + ">Fahrenheit</option>"
       "<option value=C" + String(useF ? "" : " selected") + ">Celsius</option></select>"
       "<button>Save</button></form></body></html>";
  server.send(200, "text/html", p);
}
void handleSave() {
  String ssid = server.arg("ssid2").length() ? server.arg("ssid2") : server.arg("ssid");
  prefs.begin("clock", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", server.arg("pass"));
  if (server.arg("zip") != zipCode) { prefs.putString("zip", server.arg("zip")); prefs.putFloat("lat", 0); prefs.putFloat("lon", 0); }
  prefs.putBool("useF", server.arg("units") != "C");
  prefs.end();
  String p = FPSTR(PAGE_HEAD);
  p += "<h1>Saved</h1><p>The clock is restarting and joining <b>" + ssid + "</b>. You can close this page.</p></body></html>";
  server.send(200, "text/html", p);
  delay(1500);
  ESP.restart();
}

void startSetupPortal() {
  setupMode = true;
  Serial.println("Starting WiFi setup: join 'SmartClock-Setup'");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP_STA);
  int n = WiFi.scanNetworks();
  networkOptions = "";
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() && networkOptions.indexOf(">" + s + "<") < 0)
      networkOptions += "<option" + String(s == wifiSsid ? " selected" : "") + ">" + s + "</option>";
  }
  WiFi.softAP("SmartClock-Setup");
  delay(200);
  dns.start(53, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound([] { server.sendHeader("Location", "http://192.168.4.1/"); server.send(302, "text/plain", ""); });
  server.begin();
  drawQR("WiFi setup", "WIFI:T:nopass;S:SmartClock-Setup;;", "Scan to join SmartClock-Setup,", "then open 192.168.4.1", true);
}

bool connectWifi() {
  if (wifiSsid.isEmpty()) return false;
  Serial.printf("Connecting to %s", wifiSsid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " failed");
  return WiFi.status() == WL_CONNECTED;
}

// =====================================================================
bool forceFullNext = false;
extern bool weatherShowing;
extern volatile uint32_t gestureCount;

// save the hand position to flash (at most once a minute unless forced)
void saveHands(bool force) {
  static int32_t lastSaved = -1;
  static uint32_t lastTime = 0;
  if (!force && (millis() - lastTime < 60000 || handPos == lastSaved)) return;
  prefs.begin("clock", false); prefs.putInt("hand", handPos); prefs.end();
  lastSaved = handPos; lastTime = millis();
}

void updateAll(bool forceFull) {
  float t, h;
  if (readSHT41(t, h)) { roomC = t; roomRH = h; }
  // battery guard: if USB is unplugged and the battery runs low, park the hands (position saved) until power returns
  float v = batteryVolts(), mA = batteryMilliamps();
  if (!lowBattery && v > 2.5f && v < 3.45f && mA > 5) {
    lowBattery = true; saveHands(true);
    Serial.printf("Battery low (%.2f V): hands parked\n", v);
    drawMessage("Battery low", "Plug in the USB cable.", "The hands are parked", "and will catch up.");
  } else if (lowBattery && (v > 3.75f || mA < -5)) {
    lowBattery = false; forceFullNext = true;
    Serial.println("Power back: hands catching up");
  }
  if (lowBattery) { lastDraw = millis(); return; }
  if (forceFullNext) { forceFull = true; forceFullNext = false; }
  // the weather call also tells us the time zone and daylight saving offset
  if (WiFi.status() == WL_CONNECTED && (lastWeather == 0 || millis() - lastWeather > 30UL * 60 * 1000 || forceFull)) {
    if (fetchWeather()) lastWeather = millis();
  }
  if (weatherShowing) { lastDraw = millis(); return; }
  struct tm tm; char key[64];
  bool ht = getLocalTime(&tm, 50);
  snprintf(key, sizeof key, "%d|%d|%d|%d|%.0f|%.0f|%d|%d", rainSoon ? rainHour + 100 : 0, ht ? tm.tm_yday : -1, ht ? tm.tm_hour : -1, ht ? tm.tm_min : -1,
           isnan(roomC) ? -99.0f : (useF ? roomC * 9 / 5 + 32 : roomC), roomRH, rotation, useF);
  if (forceFull || lastDrawnKey != key) {
    // a full (flashing) refresh once an hour clears e-paper ghosting; every other minute is a quiet partial refresh
    static int lastFullHour = -1;
    bool full = forceFull || (ht && tm.tm_hour != lastFullHour);
    if (full && ht) lastFullHour = tm.tm_hour;
    drawMain(full);
    lastDrawnKey = key;
  }
  lastDraw = millis();
}


void printStatus() {
  int32_t target;
  bool known = targetPos(target);
  Serial.printf("Hands show %s", handTimeText(handPos).c_str());
  if (known) {
    int32_t d = target - handPos;
    if (d > CYCLE / 2) d -= CYCLE;
    if (d < -CYCLE / 2) d += CYCLE;
    Serial.printf(", real time %s, ", handTimeText(target).c_str());
    if (abs(d) <= 2) Serial.print("in step");
    else Serial.printf("%s by %.1f s", d > 0 ? "behind" : "ahead", abs(d) / 16.0f);
  } else Serial.print(", real time not known yet");
  Serial.printf("%s. Catch-up speed %u ms/step, pulse %u ms\n", handsHold ? " [HELD]" : "", fastMs, (unsigned)pulseMs);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd == "s") printStatus();
  else if (cmd == "h") { handsHold = !handsHold; Serial.println(handsHold ? "Hands held still. Type h again to let them run." : "Hands running."); }
  else if (cmd.startsWith("c ") || cmd.startsWith("C ")) {
    int hh = 0, mm = 0, ss = 0;
    if (sscanf(cmd.c_str() + 2, "%d:%d:%d", &hh, &mm, &ss) >= 2 && mm < 60 && ss < 60) {
      posRequest = ((hh % 12) * 3600 + mm * 60 + ss) * 16;
      delay(5);
      handsHold = false;
      saveHands(true);
      Serial.printf("OK: hands show %d:%02d:%02d. Catching up to the real time...\n", hh, mm, ss);
      delay(50); printStatus();
    } else Serial.println("Type it like:  c 10:08:30");
  }
  else if (cmd == "]" || cmd == "[" || cmd == "}" || cmd == "{") {
    int32_t step = (cmd == "]" || cmd == "[") ? 16 : 960;            // 1 second or 1 minute
    bool behind = (cmd == "]" || cmd == "}");
    posRequest = wrapPos(handPos + (behind ? -step : step));
    delay(5); saveHands(true);
    Serial.printf("Noted: hands %s by %s\n", behind ? "behind" : "ahead", step == 16 ? "1 second" : "1 minute");
  }
  else if (cmd == "t") testRequest = 1;
  else if (cmd == "ip") Serial.printf("Settings page: http://smartclock.local  or  http://%s\n", WiFi.localIP().toString().c_str());
  else if (cmd.startsWith("speed")) {
    int v = cmd.substring(5).toInt();
    if (v >= 10 && v <= 62) { fastMs = v; prefs.begin("clock", false); prefs.putUShort("fast", v); prefs.end(); Serial.printf("Catch-up speed: %d ms per step\n", v); }
    else Serial.println("Use a number from 10 to 62, e.g.  speed 25");
  }
  else if (cmd == "r") {
    rotation = (rotation + 1) % 4;
    prefs.begin("clock", false); prefs.putUChar("rot", rotation); prefs.end();
    Serial.printf("Screen rotation now %d\n", rotation);
    if (setupMode) drawQR("WiFi setup", "WIFI:T:nopass;S:SmartClock-Setup;;", "Scan to join SmartClock-Setup,", "then open 192.168.4.1", true);
    else updateAll(true);
  }
  else if (cmd == "u") updateAll(true);
  else if (cmd == "f") {
    useF = !useF;
    prefs.begin("clock", false); prefs.putBool("useF", useF); prefs.end();
    Serial.println(useF ? "Fahrenheit" : "Celsius");
    updateAll(true);
  }
  else if (cmd == "w") {
    prefs.begin("clock", false); prefs.remove("ssid"); prefs.remove("pass"); prefs.end();
    Serial.println("WiFi forgotten, restarting into setup");
    saveHands(true); delay(300); ESP.restart();
  }
  else if (cmd == "+" || cmd == "-") {
    if (cmd == "+" && pulseMs < 30) pulseMs++;
    if (cmd == "-" && pulseMs > 3) pulseMs--;
    prefs.begin("clock", false); prefs.putUChar("pulse", pulseMs); prefs.end();
    Serial.printf("Pulse now %u ms\n", (unsigned)pulseMs);
  }
  else if (stage5Command(cmd)) {}
  else Serial.println("Commands: s  h  c HH:MM:SS  ] [ } {  t  speed NN  r u f w + -   v (version)  update  restart  ntfy");
}

// =====================================================================
//  WEATHER SCREEN (shown for 20 s by a gesture)
// =====================================================================
void icCloud(int x, int y, int s) {
  display.fillCircle(x - s / 4, y, s / 4, GxEPD_BLACK);
  display.fillCircle(x + s / 5, y - s / 8, s / 3, GxEPD_BLACK);
  display.fillRoundRect(x - s / 2, y, s, s / 4, s / 8, GxEPD_BLACK);
  display.fillCircle(x - s / 4, y, s / 4 - 3, GxEPD_WHITE);
  display.fillCircle(x + s / 5, y - s / 8, s / 3 - 3, GxEPD_WHITE);
  display.fillRoundRect(x - s / 2 + 3, y + 1, s - 6, s / 4 - 4, s / 8 - 2, GxEPD_WHITE);
  display.fillRect(x - s / 4, y - 2, s / 2, s / 4 - 2, GxEPD_WHITE);
}
void icSun(int x, int y, int r) {
  display.fillCircle(x, y, r, GxEPD_BLACK);
  for (int a = 0; a < 360; a += 45) {
    float c = cos(a * DEG_TO_RAD), s = sin(a * DEG_TO_RAD);
    for (int t = -1; t <= 1; t++)
      display.drawLine(x + c * (r + 4) + t * s, y + s * (r + 4) - t * c, x + c * (r + 10) + t * s, y + s * (r + 10) - t * c, GxEPD_BLACK);
  }
}
void icMoon(int x, int y, int r) { display.fillCircle(x, y, r, GxEPD_BLACK); display.fillCircle(x + r / 2, y - r / 3, r - 2, GxEPD_WHITE); }
void icDrops(int x, int y, int s, bool snow) {
  for (int i = -1; i <= 1; i++) {
    int dx = x + i * s / 3, dy = y + (i == 0 ? 4 : 0);
    if (snow) { display.drawLine(dx - 4, dy, dx + 4, dy + 8, GxEPD_BLACK); display.drawLine(dx + 4, dy, dx - 4, dy + 8, GxEPD_BLACK); display.drawLine(dx, dy - 1, dx, dy + 9, GxEPD_BLACK); }
    else for (int t = 0; t < 2; t++) display.drawLine(dx + t, dy, dx - 4 + t, dy + 10, GxEPD_BLACK);
  }
}
void icBolt(int x, int y) {
  display.fillTriangle(x + 2, y, x - 6, y + 12, x + 1, y + 12, GxEPD_BLACK);
  display.fillTriangle(x - 1, y + 10, x + 6, y + 10, x - 3, y + 22, GxEPD_BLACK);
}
void drawIcon(int code, bool day, int x, int y) {
  if (code == 0) { day ? icSun(x, y, 15) : icMoon(x, y, 17); return; }
  if (code <= 2) { day ? icSun(x + 12, y - 12, 10) : icMoon(x + 12, y - 12, 11); icCloud(x - 3, y + 6, 46); return; }
  if (code == 3) { icCloud(x, y, 52); return; }
  if (code == 45 || code == 48) { for (int i = 0; i < 4; i++) display.fillRoundRect(x - 24 + (i & 1) * 6, y - 14 + i * 9, 44, 3, 1, GxEPD_BLACK); return; }
  icCloud(x, y - 8, 50);
  if (code >= 95) { icBolt(x, y + 6); return; }
  icDrops(x, y + 10, 34, (code >= 71 && code <= 77) || code == 85 || code == 86);
}
const char *conditionText(int c) {
  if (c == 0) return "Clear";
  if (c == 1) return "Mostly clear";
  if (c == 2) return "Partly cloudy";
  if (c == 3) return "Cloudy";
  if (c == 45 || c == 48) return "Fog";
  if (c >= 51 && c <= 57) return "Drizzle";
  if (c >= 61 && c <= 67) return "Rain";
  if (c >= 71 && c <= 77) return "Snow";
  if (c >= 80 && c <= 82) return "Showers";
  if (c == 85 || c == 86) return "Snow showers";
  if (c >= 95) return "Storms";
  return "";
}
void drawWeather() {
  char big[8], hilo[24], rain[24];
  snprintf(big, sizeof big, "%.0f", wx.temp);
  snprintf(hilo, sizeof hilo, "H %.0f`  L %.0f`", wx.hi, wx.lo);
  snprintf(rain, sizeof rain, "Rain %d%%", wx.rain);
  display.setRotation(rotation);
  display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    if (!wx.ok) { centerText("No weather yet", 100, 105, &Label27); continue; }
    centerText(placeName.length() ? placeName.c_str() : "Weather", 100, 20, &Small13);
    drawIcon(wx.code, wx.isDay, 50, 68);
    int w = textWidth(big, &BigTime);
    int x = 145 - w / 2;
    display.setFont(&BigTime); display.setCursor(x, 104); display.print(big);
    display.drawCircle(x + w + 7, 44, 6, GxEPD_BLACK); display.drawCircle(x + w + 7, 44, 5, GxEPD_BLACK); display.drawCircle(x + w + 7, 44, 4, GxEPD_BLACK);
    centerText(conditionText(wx.code), 100, 140, &Label27);
    display.fillRect(14, 152, 172, 2, GxEPD_BLACK);
    centerText(hilo, 100, 176, &Small13);
    centerText(rain, 100, 196, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// =====================================================================
//  QR CODE SCREEN: point a phone camera at it to open the settings page
//  (or, in WiFi setup mode, to join the clock's setup network)
// =====================================================================
static uint8_t qrGrid[57][57];
static int qrSize = 0;
static void qrCopy(esp_qrcode_handle_t q) {
  qrSize = min(esp_qrcode_get_size(q), 57);
  for (int y = 0; y < qrSize; y++) for (int x = 0; x < qrSize; x++) qrGrid[y][x] = esp_qrcode_get_module(q, x, y);
}
void drawQR(const char *title, const char *payload, const char *line1, const char *line2, bool full) {
  esp_qrcode_config_t qc = ESP_QRCODE_CONFIG_DEFAULT();
  qc.display_func = qrCopy;
  qc.max_qrcode_version = 10;
  qc.qrcode_ecc_level = ESP_QRCODE_ECC_MED;
  qrSize = 0;
  esp_qrcode_generate(&qc, payload);
  display.setRotation(rotation);
  if (full) display.setFullWindow(); else display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    centerText(title, 100, 20, &Small13);
    if (qrSize) {
      int m = max(2, min(5, 136 / qrSize));          // module size in pixels
      int w = qrSize * m, x0 = 100 - w / 2, y0 = 28 + (136 - w) / 2;
      for (int y = 0; y < qrSize; y++) for (int x = 0; x < qrSize; x++)
        if (qrGrid[y][x]) display.fillRect(x0 + x * m, y0 + y * m, m, m, GxEPD_BLACK);
    }
    centerText(line1, 100, 180, &Small13);
    centerText(line2, 100, 197, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// =====================================================================
//  ACTIONS (what a gesture or a button on the settings page does)
// =====================================================================
uint32_t weatherUntil = 0, qrUntil = 0;
bool weatherShowing = false;      // true while any overlay (weather or QR) is on the screen
uint8_t overlayShown = 0;         // 1 = weather, 2 = QR

void nextEffect(int dir) {
  for (int k = 0; k < E_COUNT; k++) {
    cfg.effect = (cfg.effect + dir + E_COUNT) % E_COUNT;
    if (cfg.effectMask & (1 << cfg.effect)) break;
  }
}
void doAction(int a, bool fromGesture) {
  bool wasOn = ledsWanted;
  if (fromGesture && cfg.wakeOnAny && !wasOn && a != A_WEATHER && a != A_QR && a != A_NONE && a != A_TOGGLE && a != A_BREATHE) {
    ledsWanted = true; ledsLastActivity = millis();      // first gesture just wakes the ring
    return;
  }
  switch (a) {
    case A_TOGGLE: ledsWanted = !ledsWanted; break;
    case A_NEXT: nextEffect(1); ledsWanted = true; break;
    case A_PREV: nextEffect(-1); ledsWanted = true; break;
    case A_BRIGHTER: cfg.brightness = min(255, cfg.brightness + 40); ledsWanted = true; break;
    case A_DIMMER: cfg.brightness = max(10, cfg.brightness - 40); ledsWanted = true; break;
    case A_WEATHER: weatherUntil = millis() + 20000; qrUntil = 0; break;
    case A_QR: qrUntil = millis() + 60000; weatherUntil = 0; break;
    case A_BREATHE: startBreathing(); break;
    default: break;
  }
  ledsLastActivity = millis();
  if (a == A_NEXT || a == A_PREV || a == A_BRIGHTER || a == A_DIMMER) saveLightConfig();
}
void onGesture(int g) {
  lastGesture = g; lastGestureAt = millis(); gestureCount++;
  if (stage7Gesture(g)) { Serial.printf("Gesture: %s -> reminder / breathing\n", GESTURE_SHORT[g]); return; }
  if (msgGesture(g)) { Serial.printf("Gesture: %s -> message\n", GESTURE_SHORT[g]); return; }
  int a = cfg.gestureAction[g];
  Serial.printf("Gesture: %s -> %s\n", GESTURE_SHORT[g], ACTION_NAMES[a]);
  if (cfg.wakeOnAny && !ledsWanted && a == A_NONE) { ledsWanted = true; ledsLastActivity = millis(); return; }
  doAction(a, true);
}

// =====================================================================
//  SETTINGS PAGE  (open it from the QR code on the clock screen)
//  The page itself is in SETTINGS_PAGE at the bottom of this file. It reads
//  everything from /state and saves each change straight away with /set.
// =====================================================================
extern const char SETTINGS_PAGE[] PROGMEM;
const char *GESTURE_FRIENDLY[G_COUNT] = {"Swipe left", "Swipe right", "Swipe up", "Swipe down", "Tap", "Hold 2 seconds", "Hold 5 seconds", "Double tap"};
volatile uint32_t gestureCount = 0;

String fmtDuration(int32_t steps) {
  int32_t s = abs(steps) / 16;
  char b[32];
  if (s >= 3600) snprintf(b, sizeof b, "%ld h %ld min", (long)(s / 3600), (long)((s / 60) % 60));
  else if (s >= 60) snprintf(b, sizeof b, "%ld min %ld s", (long)(s / 60), (long)(s % 60));
  else snprintf(b, sizeof b, "%ld s", (long)s);
  return b;
}

// =====================================================================
//  STAGE 5: updates over WiFi, health reports, remote restart
//
//  Updates: GitHub builds the firmware and publishes it on GitHub Pages:
//     <OTA base>/firmware/manifest.json  ->  {"build":12,"bin":"firmware/app-12.bin","size":..,"sha256":".."}
//  The clock checks it once at start-up and every night at about 3 AM, and installs a newer build
//  then (or straight away when asked from the Setup tab or with the ntfy word "update").
//  A new version has 5 minutes to prove itself; if it crashes or freezes before that, the clock
//  goes back to the previous version on its own and reports it.
//
//  Health and remote control use ntfy.sh (free push notifications, no account):
//     <topic>      the clock posts here: health report every 6 h, updates, restarts, problems
//     <topic>-cmd  send one of these words to it:  restart   update   status   help
//  The topic name works like a password: it is random and shown on the Setup tab.
// =====================================================================
String otaBase, ntfyTopic;
bool autoUpdate = true, healthOn = true;
volatile bool restartPending = false;
uint32_t restartAt = 0;
volatile bool wantUpdate = false;              // check and install now (button / ntfy)
volatile int otaState = 0;                     // 0 idle, 1 checking, 2 downloading, 3 restarting
volatile int otaPct = 0;
String otaStatus = "Not checked yet";
int latestBuild = -1;
float battV = NAN;                             // filled in by the main loop (I2C is only used from there)
bool markedValid = false;
Preferences prefs5;                            // the network task's own handle on the settings store
SemaphoreHandle_t tlsLock = nullptr;           // one HTTPS connection at a time (they need a lot of memory)
SemaphoreHandle_t &tlsLockRef() { return tlsLock; }

String fwVersion(int b = FW_BUILD) { return b > 0 ? String(FW_SERIES) + "." + String(b) : String(FW_SERIES) + ".local"; }
bool timeKnown() { return time(nullptr) > 1700000000; }
String uptimeText() {
  uint32_t m = millis() / 60000;
  if (m < 60) return String(m) + " min";
  if (m < 48 * 60) return String(m / 60) + " h " + String(m % 60) + " min";
  return String(m / 1440) + " d " + String((m % 1440) / 60) + " h";
}
const char *resetText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW: return "restart";
    case ESP_RST_PANIC: return "a crash";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "a freeze (watchdog)";
    case ESP_RST_BROWNOUT: return "a power dip";
    case ESP_RST_DEEPSLEEP: return "sleep";
    default: return "unknown";
  }
}
bool badReset(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT || r == ESP_RST_BROWNOUT;
}

void loadStage5Settings() {
  prefs.begin("clock", false);
  otaBase = prefs.getString("otaurl", OTA_BASE_DEFAULT);
  ntfyTopic = prefs.getString("ntfy", "");
  if (ntfyTopic.isEmpty()) {                   // first start: make a random, hard-to-guess topic name
    const char *abc = "abcdefghijkmnpqrstuvwxyz23456789";
    ntfyTopic = "clock-";
    for (int i = 0; i < 14; i++) ntfyTopic += abc[esp_random() % 32];
    prefs.putString("ntfy", ntfyTopic);
  }
  autoUpdate = prefs.getBool("autoupd", true);
  healthOn = prefs.getBool("health", true);
  prefs.end();
  otaBase.trim();
  while (otaBase.endsWith("/")) otaBase.remove(otaBase.length() - 1);
}

// ---------- HTTPS helpers (certificates checked against Mozilla's trusted list) ----------
void secureClient(WiFiClientSecure &c) { c.setCACertBundle(CA_BUNDLE, sizeof(CA_BUNDLE)); }

// ---------- ntfy: outgoing messages go through a small queue, sent by the network task ----------
const int NQ = 6;
NtfyMsg nq[NQ];
int nqHead = 0, nqCount = 0;
portMUX_TYPE nqMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t nqLock = nullptr;
void notify(const String &title, const String &body, const String &tags = "", int prio = 3) {
  if (!nqLock) return;
  xSemaphoreTake(nqLock, portMAX_DELAY);
  if (nqCount == NQ) { nqHead = (nqHead + 1) % NQ; nqCount--; }      // full: drop the oldest
  nq[(nqHead + nqCount) % NQ] = {title, body, tags, prio};
  nqCount++;
  xSemaphoreGive(nqLock);
}
bool sendNtfy(const NtfyMsg &m) {
  if (ntfyTopic.isEmpty() || WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure c; secureClient(c);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(c, "https://ntfy.sh/" + ntfyTopic)) return false;
  http.addHeader("Title", m.title);                     // plain text titles (emoji go in Tags)
  if (m.tags.length()) http.addHeader("Tags", m.tags);
  http.addHeader("Priority", String(m.prio));
  int code = http.POST(m.body);
  http.end();
  if (code != 200) Serial.printf("ntfy: HTTP %d\n", code);
  return code == 200;
}

String statusLine() {
  String s = "Software " + fwVersion() + " | up " + uptimeText();
  if (WiFi.status() == WL_CONNECTED) s += " | WiFi " + String(WiFi.RSSI()) + " dBm";
  if (!isnan(battV) && battV > 2.5f) s += " | battery " + String(battV, 2) + " V";
  if (!isnan(roomC)) s += " | inside " + String(useF ? roomC * 9 / 5 + 32 : roomC, 0) + (useF ? "F" : "C");
  s += " | hands " + handTimeText(handPos);
  if (handsHold) s += " (paused)";
  if (lowBattery) s += " | LOW BATTERY";
  return s;
}

// ---------- commands sent to <topic>-cmd ----------
String lastCmdId;
void handleRemote(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  Serial.printf("ntfy command: %s\n", cmd.c_str());
  if (cmd == "restart" || cmd == "reboot") {
    notify("Restarting", "Restart asked for from ntfy. Back in about 30 seconds.", "arrows_counterclockwise", 3);
    restartPending = true; restartAt = millis() + 8000;     // give the queue time to send the reply
  } else if (cmd == "update") {
    notify("Checking for an update", "Asked from ntfy.", "mag", 2);
    wantUpdate = true;
  } else if (cmd == "clear") {
    remoteClear = true;
  } else if (cmd == "status" || cmd == "ping") {
    notify("Clock status", statusLine(), "white_check_mark", 3);
  } else {
    notify("Commands", "Send one word to " + ntfyTopic + "-cmd:\nrestart  - restart the clock\nupdate  - install new software now\nstatus  - this report\nclear  - stop the heartbeat and clear messages from the screen", "information_source", 2);
  }
}
void pollCommands() {
  if (ntfyTopic.isEmpty() || !timeKnown()) return;
  String since = lastCmdId.length() ? lastCmdId : String((long)time(nullptr) - 60);
  WiFiClientSecure c; secureClient(c);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(c, "https://ntfy.sh/" + ntfyTopic + "-cmd/json?poll=1&since=" + since)) return;
  int code = http.GET();
  String body = code == 200 ? http.getString() : "";
  http.end();
  if (code != 200) { Serial.printf("ntfy poll: HTTP %d\n", code); return; }
  int from = 0;
  while (from < (int)body.length()) {
    int nl = body.indexOf('\n', from);
    if (nl < 0) nl = body.length();
    String line = body.substring(from, nl);
    from = nl + 1;
    JsonDocument d;
    if (deserializeJson(d, line) || d["event"] != "message") continue;
    String id = d["id"] | "";
    if (id.isEmpty() || id == lastCmdId) continue;
    lastCmdId = id;
    prefs5.begin("clock", false); prefs5.putString("ncmd", lastCmdId); prefs5.end();   // remembered so a restart isn't repeated
    handleRemote(d["message"] | "");
  }
}

// ---------- updates ----------
bool fetchManifest(int &build, String &binUrl, int &size, String &sha) {
  if (otaBase.isEmpty()) { otaStatus = "No update address set yet"; return false; }
  WiFiClientSecure c; secureClient(c);
  HTTPClient http;
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(c, otaBase + "/firmware/manifest.json?t=" + String(millis()))) return false;
  int code = http.GET();
  String body = code == 200 ? http.getString() : "";
  http.end();
  if (code != 200) { otaStatus = "Couldn't reach the update page (HTTP " + String(code) + ")"; return false; }
  JsonDocument d;
  if (deserializeJson(d, body)) { otaStatus = "The update page looked wrong"; return false; }
  build = d["build"] | 0;
  String bin = d["bin"] | "";
  binUrl = bin.startsWith("http") ? bin : otaBase + "/" + bin;
  size = d["size"] | 0;
  sha = d["sha256"] | "";
  sha.toLowerCase();
  return build > 0 && size > 0 && sha.length() == 64;
}

bool installUpdate(int build, const String &url, int size, const String &sha) {
  if (size > 0x1E0000) { otaStatus = "The new version is too big for this clock"; return false; }
  otaState = 2; otaPct = 0; otaStatus = "Downloading " + fwVersion(build) + "...";
  Serial.printf("Update: downloading %s (%d bytes)\n", url.c_str(), size);
  WiFiClientSecure c; secureClient(c);
  HTTPClient http;
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(c, url)) { otaState = 0; otaStatus = "Download failed to start"; return false; }
  int code = http.GET();
  if (code != 200) { http.end(); otaState = 0; otaStatus = "Download failed (HTTP " + String(code) + ")"; return false; }
  if (!Update.begin(size, U_FLASH)) { http.end(); otaState = 0; otaStatus = "Couldn't make room for the update"; return false; }
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  WiFiClient *st = http.getStreamPtr();
  static uint8_t buf[2048];
  int got = 0;
  uint32_t lastData = millis();
  bool ok = true;
  while (got < size) {
    esp_task_wdt_reset();
    int avail = st->available();
    if (avail <= 0) {
      if (!http.connected() || millis() - lastData > 20000) { ok = false; break; }
      delay(5); continue;
    }
    int n = st->readBytes(buf, min(avail, (int)sizeof buf));
    if (n <= 0) continue;
    lastData = millis();
    mbedtls_sha256_update(&ctx, buf, n);
    if (Update.write(buf, n) != (size_t)n) { ok = false; break; }
    got += n;
    otaPct = (int)((int64_t)got * 100 / size);
  }
  http.end();
  uint8_t digest[32];
  mbedtls_sha256_finish(&ctx, digest);
  mbedtls_sha256_free(&ctx);
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02x", digest[i]);
  if (!ok || got != size) { Update.abort(); otaState = 0; otaStatus = "Download was interrupted - will try again later"; return false; }
  if (sha != hex) { Update.abort(); otaState = 0; otaStatus = "Download was damaged (checksum) - not installed"; Serial.printf("sha256 mismatch: %s\n", hex); return false; }
  if (!Update.end(true)) { otaState = 0; otaStatus = "Install failed: " + String(Update.errorString()); return false; }
  prefs5.begin("clock", false); prefs5.putInt("pending", build); prefs5.end();
  otaState = 3; otaStatus = "Installed " + fwVersion(build) + " - restarting";
  NtfyMsg m = {"Installing update", "Software " + fwVersion() + " -> " + fwVersion(build) + ". Restarting now.", "arrow_up", 2};
  sendNtfy(m);
  return true;
}

// check (and maybe install). install: 0 = only look, 1 = install if newer, 2 = install even a build that failed before
void checkForUpdate(int install) {
  otaState = 1;
  int build = 0, size = 0; String url, sha;
  bool ok = fetchManifest(build, url, size, sha);
  otaState = 0;
  if (!ok) { Serial.println("Update check: " + otaStatus); return; }
  latestBuild = build;
  prefs5.begin("clock", true); int bad = prefs5.getInt("badbuild", 0); prefs5.end();
  if (build <= FW_BUILD) { otaStatus = "Up to date (" + fwVersion() + ")"; Serial.println("Update check: up to date"); return; }
  if (build == bad && install < 2) { otaStatus = fwVersion(build) + " didn't work on this clock last time - skipped"; return; }
  otaStatus = fwVersion(build) + " is available";
  if (!install) return;
  if (!isnan(battV) && battV > 2.5f && battV < 3.7f) { otaStatus = fwVersion(build) + " is available - waiting for more battery / USB power"; return; }
  if (installUpdate(build, url, size, sha)) {
    saveHands(true);
    restartPending = true; restartAt = millis() + 1500;
  }
}

// ---------- start-up report, health reports, the network task ----------
void bootReport() {
  esp_reset_reason_t r = esp_reset_reason();
  prefs5.begin("clock", false);
  int pending = prefs5.getInt("pending", 0);
  if (pending && pending != FW_BUILD) { prefs5.putInt("badbuild", pending); prefs5.remove("pending"); }   // we're back on the old one
  prefs5.end();                                  // (if it's us, "pending" stays until the 5 healthy minutes are up)
  if (pending && pending != FW_BUILD) {
    notify("Update rolled back", "Software " + fwVersion(pending) + " didn't start properly, so the clock went back to " + fwVersion() +
           " by itself. It won't try that version again on its own.", "warning", 4);
  } else if (pending) {
    notify("Updated", "Now running software " + fwVersion() + ". Checking it for 5 minutes before keeping it.", "tada", 3);
  } else if (badReset(r)) {
    notify("Clock restarted after " + String(resetText(r)), statusLine(), "warning", 4);
  } else {
    notify("Clock started", "Started after " + String(resetText(r)) + ". " + statusLine(), "alarm_clock", 2);
  }
}

void netTask(void *) {
  esp_task_wdt_add(nullptr);
  bool reported = false;
  uint32_t lastPoll = 0, lastBeat = 0, bootChecked = 0;
  int lastNightly = -1;
  uint32_t nightMin = 0;
  for (int i = 0; i < (int)ntfyTopic.length(); i++) nightMin = nightMin * 31 + ntfyTopic[i];
  nightMin %= 50;                                // each clock picks its own minute between 3:00 and 3:50
  for (;;) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (WiFi.status() != WL_CONNECTED || setupMode) continue;
    uint32_t now = millis();
    xSemaphoreTake(tlsLock, portMAX_DELAY);
    // 1. anything waiting to be sent
    while (nqCount > 0) {
      xSemaphoreTake(nqLock, portMAX_DELAY);
      NtfyMsg m = nq[nqHead];
      xSemaphoreGive(nqLock);
      if (!healthOn) { m.title = ""; }
      bool sent = m.title.isEmpty() || sendNtfy(m);
      if (!sent) break;                          // try again next time round
      xSemaphoreTake(nqLock, portMAX_DELAY);
      nqHead = (nqHead + 1) % NQ; nqCount--;
      xSemaphoreGive(nqLock);
      esp_task_wdt_reset();
    }
    if (timeKnown()) {
      if (!reported) { reported = true; bootReport(); }
      // 2. commands, once a minute
      if (now - lastPoll > 60000 || lastPoll == 0) { lastPoll = now; pollCommands(); }
      static uint32_t lastMsgPoll = 0;
      if (now - lastMsgPoll > 30000 || lastMsgPoll == 0) { lastMsgPoll = now; pollMessages(); }
      // 3. health report every 6 hours (quiet: no sound on the phone)
      if (healthOn && (lastBeat == 0 ? now > 180000 : now - lastBeat > 6UL * 3600 * 1000)) {
        lastBeat = now;
        NtfyMsg m = {"Clock OK", statusLine(), "heartbeat", 1};
        sendNtfy(m);
      }
      // 4. updates: look once after start-up, install at night, or straight away when asked
      if (!bootChecked && now > 120000) { bootChecked = now; checkForUpdate(0); }
      struct tm lt; time_t tt = time(nullptr); localtime_r(&tt, &lt);
      if (autoUpdate && lt.tm_hour == 3 && lt.tm_min == (int)nightMin && lastNightly != lt.tm_yday) {
        lastNightly = lt.tm_yday; checkForUpdate(1);
      }
    }
    if (wantUpdate) { wantUpdate = false; checkForUpdate(2); }
    xSemaphoreGive(tlsLock);
  }
}

// (verifyRollbackLater() is in clock_support.h: it tells Arduino not to approve a freshly installed
//  version at start-up - stage5Loop approves it after 5 healthy minutes, otherwise the clock goes back)

void stage5Begin() {
  tlsLock = xSemaphoreCreateMutex();
  nqLock = xSemaphoreCreateMutex();
  loadStage5Settings();
  prefs.begin("clock", true); lastCmdId = prefs.getString("ncmd", ""); prefs.end();
  Serial.printf("Software %s   updates from: %s\n", fwVersion().c_str(), otaBase.isEmpty() ? "(not set)" : otaBase.c_str());
  Serial.printf("Health reports: https://ntfy.sh/%s    commands: %s-cmd\n", ntfyTopic.c_str(), ntfyTopic.c_str());
  // watchdog: if the main loop or the hands freeze for 60 s, restart (the hands keep their place)
  esp_task_wdt_config_t wdt = {.timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = true};
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
}
bool stage5Command(const String &cmd) {         // Serial Monitor commands for stage 5
  if (cmd == "v") { Serial.println(statusLine()); Serial.println("Updates: " + otaStatus + "   address: " + (otaBase.isEmpty() ? String("(not set)") : otaBase)); }
  else if (cmd == "update") { wantUpdate = true; Serial.println("Checking for an update..."); }
  else if (cmd.startsWith("msg ")) { addMessage(cleanText(cmd.substring(4)), timeKnown() ? (uint32_t)time(nullptr) : 0); Serial.println("Test message added"); }
  else if (cmd == "restart") { restartPending = true; restartAt = millis() + 500; }
  else if (cmd == "ntfy") { Serial.printf("Health reports: https://ntfy.sh/%s\nCommands go to: %s-cmd\n", ntfyTopic.c_str(), ntfyTopic.c_str()); notify("Test from the clock", statusLine(), "wave", 3); }
  else if (cmd.startsWith("ota ")) { String v = cmd.substring(4); v.trim(); while (v.endsWith("/")) v.remove(v.length() - 1); otaBase = v; prefs.begin("clock", false); prefs.putString("otaurl", v); prefs.end(); Serial.println("Update address: " + v); }
  else return false;
  return true;
}
bool loopWdt = false;
void stage5Start() {                            // after WiFi is up
  esp_task_wdt_add(nullptr);                    // the main loop
  loopWdt = true;
  xTaskCreate(netTask, "net", 12288, nullptr, 1, nullptr);
}
void stage5Loop() {
  if (loopWdt) esp_task_wdt_reset();
  if (!markedValid && millis() > 5UL * 60 * 1000) {
    markedValid = true;
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
      esp_ota_mark_app_valid_cancel_rollback();
      prefs.begin("clock", false); prefs.remove("pending"); prefs.end();
      notify("Update kept", "Software " + fwVersion() + " has run well for 5 minutes, so the clock is keeping it.", "white_check_mark", 2);
    } else {
      esp_ota_mark_app_valid_cancel_rollback();
      prefs.begin("clock", false); if (prefs.getInt("pending", 0) == FW_BUILD) prefs.remove("pending"); prefs.end();
    }
  }
  if (restartPending && (int32_t)(millis() - restartAt) >= 0) {
    Serial.println("Restarting...");
    saveHands(true);
    ledsWanted = false;
    delay(300);
    ESP.restart();
  }
}

// =====================================================================
//  STAGE 6: "thinking of you" messages
//  Send a message to <topic>-msg in the ntfy app. The clock shows it on its little screen, glows a
//  soft heartbeat until Amanda notices, and tells you on <topic> when it was delivered and seen.
//  While a message is on the screen:  tap = send a heart back,  tap twice = next message / back
//  to the clock. Any gesture at all counts as "seen". Quiet hours are respected: no glow at night,
//  and the heartbeat starts again in the morning and when the room lights come on.
// =====================================================================
const char *RECIPIENT = "Amanda";
const int MSG_MAX = 5;
const uint32_t HEARTBEAT_MS = 30UL * 60 * 1000;        // glow for 30 minutes after a message arrives
const uint32_t SEEN_KEEP_MS = 60UL * 1000;             // a seen message stays up 1 minute after the last wave, then the clock comes back
ClockMsg msgs[MSG_MAX];
int msgCount = 0;                                      // oldest first
int msgShow = -1;                                      // message on the screen, -1 = none
uint32_t msgTouchAt = 0;                               // last gesture on the message screen
uint32_t msgNoteUntil = 0;                             // "Heart sent" line shown until
volatile bool msgDirty = false, msgSaveNeeded = false;
volatile uint32_t hbUntil = 0;                         // heartbeat glow runs until (millis)
volatile bool remoteClear = false;                     // "clear" sent to <topic>-cmd
volatile uint32_t heartFlashUntil = 0;                 // pink "heart sent" burst until (millis)
volatile bool hbActive = false;                        // read by the LED task
bool hbGlowOn = true;                                  // setting: heartbeat glow for new messages
uint32_t lastMsgTime = 0;                              // newest ntfy message time handled (unix)
String recentMsgIds;
SemaphoreHandle_t msgLock = nullptr;

int unseenCount() { int n = 0; for (int i = 0; i < msgCount; i++) if (!msgs[i].seen) n++; return n; }

// the screen fonts only have plain letters: turn curly quotes, dashes and hearts into ones they have
String cleanText(const String &in) {
  String o;
  for (int i = 0; i < (int)in.length(); i++) {
    uint8_t c = in[i];
    if (c == '\n' || c == '\r' || c == '\t') { o += ' '; continue; }
    if (c == '`') { o += '\''; continue; }                    // ` draws a degree sign in these fonts
    if (c < 0x80) { if (c >= 0x20) o += (char)c; continue; }
    int len = c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : c >= 0xC0 ? 2 : 1;
    String seq = in.substring(i, i + len);
    i += len - 1;
    if (seq == "\xE2\x80\x9C" || seq == "\xE2\x80\x9D") o += '"';
    else if (seq == "\xE2\x80\x98" || seq == "\xE2\x80\x99") o += '\'';
    else if (seq == "\xE2\x80\x93" || seq == "\xE2\x80\x94") o += '-';
    else if (seq == "\xE2\x80\xA6") o += "...";
    else if (seq == "\xE2\x9D\xA4" || seq == "\xE2\x99\xA5" || seq == "\xF0\x9F\x92\x95" || seq == "\xF0\x9F\x92\x96" ||
             seq == "\xF0\x9F\x92\x97" || seq == "\xF0\x9F\x98\x8D" || seq == "\xF0\x9F\xA5\xB0" || seq == "\xF0\x9F\x98\x98") o += "<3";
    else if (seq == "\xC2\xB0") o += '`';                     // the degree sign
    // anything else (most emoji) is left out
  }
  o.trim();
  while (o.indexOf("  ") >= 0) o.replace("  ", " ");
  if (o.length() > 160) o = o.substring(0, 157) + "...";
  return o;
}

String shortQuote(const String &t) { return t.length() > 40 ? "\"" + t.substring(0, 37) + "...\"" : "\"" + t + "\""; }

void loadMsgs() {
  prefs.begin("clock", true);
  String j = prefs.getString("msgs", "");
  lastMsgTime = prefs.getUInt("msgtime", 0);
  hbGlowOn = prefs.getBool("hbglow", true);
  prefs.end();
  JsonDocument d;
  if (j.length() && !deserializeJson(d, j)) {
    for (JsonObject o : d.as<JsonArray>()) {
      if (msgCount >= MSG_MAX) break;
      msgs[msgCount++] = {String((const char *)(o["t"] | "")), o["a"] | 0u, (o["s"] | 0) != 0, (o["h"] | 0) != 0};
    }
  }
  if (unseenCount()) hbUntil = millis() + HEARTBEAT_MS;      // still waiting after a restart: glow again
}
void saveMsgs() {                                             // main loop only (it owns "prefs")
  JsonDocument d;
  JsonArray a = d.to<JsonArray>();
  xSemaphoreTake(msgLock, portMAX_DELAY);
  for (int i = 0; i < msgCount; i++) { JsonObject o = a.add<JsonObject>(); o["t"] = msgs[i].text; o["a"] = msgs[i].at; o["s"] = msgs[i].seen ? 1 : 0; o["h"] = msgs[i].hearted ? 1 : 0; }
  uint32_t lt = lastMsgTime;
  xSemaphoreGive(msgLock);
  String j; serializeJson(d, j);
  prefs.begin("clock", false); prefs.putString("msgs", j); prefs.putUInt("msgtime", lt); prefs.end();
}

void addMessage(const String &text, uint32_t at) {
  xSemaphoreTake(msgLock, portMAX_DELAY);
  if (msgCount == MSG_MAX) {                                  // full: drop the oldest seen one, else the oldest
    int drop = 0;
    for (int i = 0; i < msgCount; i++) if (msgs[i].seen) { drop = i; break; }
    for (int i = drop; i < msgCount - 1; i++) msgs[i] = msgs[i + 1];
    msgCount--;
    if (msgShow == drop) msgShow = -1; else if (msgShow > drop) msgShow--;
  }
  msgs[msgCount++] = {text, at, false, false};
  xSemaphoreGive(msgLock);
  if (hbGlowOn) hbUntil = millis() + HEARTBEAT_MS;
  msgDirty = true; msgSaveNeeded = true;
}

// network task: new messages from <topic>-msg (checked every 30 s)
void pollMessages() {
  if (ntfyTopic.isEmpty() || !timeKnown()) return;
  uint32_t since = lastMsgTime ? lastMsgTime + 1 : (uint32_t)time(nullptr) - 300;
  WiFiClientSecure c; secureClient(c);
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(c, "https://ntfy.sh/" + ntfyTopic + "-msg/json?poll=1&since=" + String(since))) return;
  int code = http.GET();
  String body = code == 200 ? http.getString() : "";
  http.end();
  if (code != 200) { Serial.printf("ntfy messages: HTTP %d\n", code); return; }
  int from = 0;
  while (from < (int)body.length()) {
    int nl = body.indexOf('\n', from);
    if (nl < 0) nl = body.length();
    String line = body.substring(from, nl);
    from = nl + 1;
    JsonDocument d;
    if (deserializeJson(d, line) || d["event"] != "message") continue;
    String id = d["id"] | "";
    if (id.isEmpty() || recentMsgIds.indexOf(id) >= 0) continue;
    recentMsgIds += id + ","; if (recentMsgIds.length() > 200) recentMsgIds.remove(0, recentMsgIds.indexOf(',', 100) + 1);
    uint32_t t = d["time"] | 0u;
    if (t > lastMsgTime) lastMsgTime = t;
    String text = cleanText(String((const char *)(d["message"] | "")));
    String title = cleanText(String((const char *)(d["title"] | "")));
    if (title.length()) text = title + ": " + text;
    if (text.isEmpty()) continue;
    Serial.printf("Message: %s\n", text.c_str());
    addMessage(text, t);
    notify("Delivered", "On " + String(RECIPIENT) + "'s clock now: " + shortQuote(text), "incoming_envelope", 2);
  }
}

void markSeen(int i, const char *how) {
  if (i < 0 || i >= msgCount || msgs[i].seen) return;
  msgs[i].seen = true;
  msgSaveNeeded = true; msgDirty = true;
  notify("Seen", String(RECIPIENT) + " saw " + shortQuote(msgs[i].text) + how, "eyes", 3);
  if (!unseenCount()) hbUntil = 0;
}
void sendHeart(int i) {
  if (i < 0 || i >= msgCount) return;
  heartFlashUntil = millis() + 2500;
  msgNoteUntil = millis() + 8000;
  msgDirty = true;
  if (msgs[i].hearted) return;                                // one heart per message reaches the phone
  msgs[i].hearted = true; msgSaveNeeded = true;
  notify(String(RECIPIENT) + " sent a heart back", "In reply to " + shortQuote(msgs[i].text), "heart", 4);
}

// which message belongs on the screen right now
void msgPick() {
  xSemaphoreTake(msgLock, portMAX_DELAY);
  int want = -1;
  if (msgShow >= 0 && msgShow < msgCount && (!msgs[msgShow].seen || millis() - msgTouchAt < SEEN_KEEP_MS)) want = msgShow;
  if (want < 0) for (int i = 0; i < msgCount; i++) if (!msgs[i].seen) { want = i; break; }
  if (want != msgShow) { msgShow = want; msgDirty = true; }
  xSemaphoreGive(msgLock);
}

// gestures while a message is on the screen. true = handled here
bool msgGesture(int g) {
  if (msgShow < 0) return false;
  int i = msgShow;
  bool firstLook = !msgs[i].seen;
  msgTouchAt = millis();
  if (firstLook) markSeen(i, "");
  if (g == G_TAP) { sendHeart(i); return true; }
  if (g == G_DOUBLE) {                                        // next unseen message, or back to the clock
    int next = -1;
    for (int k = 0; k < msgCount; k++) if (!msgs[k].seen) { next = k; break; }
    msgShow = next; msgTouchAt = millis(); msgDirty = true;
    return true;
  }
  return firstLook;                                           // first wave just says "seen"; after that, normal
}

// the message screen (200 x 200)
void wrapLines(const String &text, const GFXfont *f, int maxW, String *lines, int &n, int maxLines) {
  n = 0;
  String cur, word;
  String t = text + " ";
  for (int i = 0; i < (int)t.length(); i++) {
    char ch = t[i];
    if (ch != ' ') { word += ch; continue; }
    if (word.isEmpty()) continue;
    String trial = cur.isEmpty() ? word : cur + " " + word;
    if (textWidth(trial.c_str(), f) <= maxW) cur = trial;
    else {
      if (cur.length()) { if (n < maxLines) lines[n] = cur; n++; }
      cur = word;
      while (textWidth(cur.c_str(), f) > maxW && cur.length() > 1) {   // a very long word: break it
        int k = cur.length() - 1;
        while (k > 1 && textWidth(cur.substring(0, k).c_str(), f) > maxW) k--;
        if (n < maxLines) lines[n] = cur.substring(0, k); n++;
        cur = cur.substring(k);
      }
    }
    word = "";
  }
  if (cur.length()) { if (n < maxLines) lines[n] = cur; n++; }
}
void drawHeart(int x, int y, int s) {                        // a small filled heart, centre top at (x, y)
  display.fillCircle(x - s / 2, y, s / 2, GxEPD_BLACK);
  display.fillCircle(x + s / 2, y, s / 2, GxEPD_BLACK);
  display.fillTriangle(x - s, y + 1, x + s, y + 1, x, y + s + s / 2, GxEPD_BLACK);
}
void drawMsgScreen(bool full) {
  xSemaphoreTake(msgLock, portMAX_DELAY);
  if (msgShow < 0 || msgShow >= msgCount) { xSemaphoreGive(msgLock); return; }
  ClockMsg m = msgs[msgShow];
  int unseen = unseenCount();
  xSemaphoreGive(msgLock);
  struct tm t; char hm[12] = "";
  if (getLocalTime(&t, 20)) { int h = t.tm_hour % 12; if (h == 0) h = 12; snprintf(hm, sizeof hm, "%d:%02d", h, t.tm_min); }
  String lines[9]; int n = 0;
  const GFXfont *f = &Label27;
  int lh = 30;
  wrapLines(m.text, f, 184, lines, n, 9);
  if (n > 4) { f = &Small13; lh = 17; wrapLines(m.text, f, 188, lines, n, 9); }
  if (n > 8) { n = 8; lines[7] += "..."; }
  display.setRotation(rotation);
  if (full) display.setFullWindow(); else display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    drawHeart(16, 14, 8);
    display.setFont(&Small13);
    display.setCursor(30, 22);
    String head = !m.seen ? (unseen > 1 ? String(unseen) + " new messages" : String("New message")) : String("Message");
    display.print(head);
    int tw = textWidth(hm, &Small13);
    display.setCursor(196 - tw, 22); display.print(hm);
    display.fillRect(4, 32, 192, 2, GxEPD_BLACK);
    int top = 42, bottom = 168;
    int blockH = n * lh;
    int y = top + max(0, (bottom - top - blockH) / 2) + lh - 6;
    for (int k = 0; k < n; k++) { centerText(lines[k].c_str(), 100, y, f); y += lh; }
    display.fillRect(4, 172, 192, 1, GxEPD_BLACK);
    const char *foot = millis() < msgNoteUntil ? "Heart sent  <3" : !m.seen ? "Wave to say you saw it" : "Tap: send a heart   2 taps: close";
    centerText(foot, 100, 192, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// heartbeat on the ring: a soft "lub-dub" every 2.4 s, or a pink burst when a heart is sent
bool msgLedOverride() { return hbActive || millis() < heartFlashUntil; }
void renderHeartbeat(uint32_t t) {
  int n = cfg.ledCount;
  if (millis() < heartFlashUntil) {                         // heart sent: a quick pink bloom from 12 o'clock
    float u = 1.0f - (heartFlashUntil - millis()) / 2500.0f;
    memset(px, 0, sizeof px);
    for (int i = 0; i < n; i++) setPx(i, 255 * (1 - u) + 40 * u, 30, 90 * (1 - u) + 20 * u);
    blob(ledAt(u * 360), 3.0f, 255, 200, 220);
    blob(ledAt(-u * 360), 3.0f, 255, 200, 220);
    return;
  }
  float ph = (t % 2400) / 1000.0f;
  auto pulse = [](float x, float c, float w) { float d = (x - c) / w; return expf(-d * d); };
  float f = 0.08f + pulse(ph, 0.25f, 0.09f) + 0.6f * pulse(ph, 0.55f, 0.09f);
  if (f > 1) f = 1;
  for (int i = 0; i < n; i++) setPx(i, 255 * f, 28 * f, 60 * f);
}

// ambient light from the gesture sensor (9930): notices the room lights coming on
int ambientLevel() { int lo = apdsRead(0x94), hi = apdsRead(0x95); return (lo < 0 || hi < 0) ? -1 : (hi << 8) | lo; }
void msgLoop() {
  msgPick();
  if (msgSaveNeeded) { msgSaveNeeded = false; saveMsgs(); }
  bool allowed = ledsAllowed();
  static bool wasAllowed = true;
  if (allowed && !wasAllowed && unseenCount() && hbGlowOn) hbUntil = millis() + HEARTBEAT_MS;    // morning: glow again
  wasAllowed = allowed;
  // room lights coming on (only judged while our own ring is dark)
  static uint32_t lastAmb = 0, darkSince = 0, ringDarkSince = 0;
  static float amb = -1;
  if (ledLevel > 0) ringDarkSince = millis();
  if (apdsOK && !apdsIs9960 && millis() - lastAmb > 2000 && millis() - ringDarkSince > 5000) {
    lastAmb = millis();
    int a = ambientLevel();
    if (a >= 0) {
      if (amb < 0) amb = a;
      bool dark = amb < 40;
      if (dark && darkSince == 0) darkSince = millis();
      if (!dark) darkSince = 0;
      if (darkSince && millis() - darkSince > 120000 && a > max(150.0f, amb * 4) && unseenCount() && hbGlowOn) {
        Serial.println("Room lights came on: showing the heartbeat again");
        hbUntil = millis() + 10UL * 60 * 1000;
        darkSince = 0;
      }
      amb = amb * 0.8f + a * 0.2f;
    }
  }
  // while a clock hand covers the window she can't wave back, so the heartbeat waits and the
  // time it waited is added on afterwards
  static uint32_t lastTick = millis();
  uint32_t nowMs = millis();
  bool blocked = sensorBlocked();
  if (blocked && hbUntil > nowMs) hbUntil += nowMs - lastTick;
  lastTick = nowMs;
  if (remoteClear) {                                          // sender cleared it: no "Seen" receipts
    remoteClear = false;
    xSemaphoreTake(msgLock, portMAX_DELAY);
    for (int i = 0; i < msgCount; i++) msgs[i].seen = true;
    xSemaphoreGive(msgLock);
    hbUntil = 0; msgShow = -1; msgDirty = true; msgSaveNeeded = true;
    notify("Cleared", "Messages on the clock were cleared from ntfy.", "broom", 2);
  }
  hbActive = hbGlowOn && unseenCount() > 0 && nowMs < hbUntil && !blocked;
}

// =====================================================================
//  STAGE 7: reminders, rain warning, breathing guide
//  Reminders: set on the settings page (Clock tab). When one is due the screen shows it and the ring
//  glows gold, even in quiet hours (she asked for it). Wave = done, tap twice = snooze 10 minutes.
//  Rain warning: if rain is likely in the next 3 hours, the screen's corner says when, and the ring
//  shimmers blue once (not at night).
//  Breathing guide: about a minute and a half of slow breathing led by the ring:
//  in 4 s, hold 4 s, out 6 s, six times. Any wave stops it.
// =====================================================================
const int REM_MAX = 10;
Reminder rems[REM_MAX];
int remCount = 0;
int remActive = -1;                 // reminder on screen now
uint32_t remStart = 0, remSnoozeAt = 0;
int remSnoozed = -1;
volatile bool remDirty = false;
const uint32_t REM_LIGHT_MS = 5UL * 60 * 1000, REM_SHOW_MS = 30UL * 60 * 1000;

bool rainWarnOn = true;
bool rainSoon = false;
int rainHour = -1;                  // local hour it's likely to start
uint32_t rainShimmerUntil = 0;

volatile uint32_t breatheStart = 0; // 0 = not running
const int BR_IN = 4000, BR_HOLD = 4000, BR_OUT = 6000, BR_CYCLES = 6;
const int BR_CYCLE = BR_IN + BR_HOLD + BR_OUT;
volatile bool brDirty = false;

String daysText(uint8_t d) {
  if (d == 0x7F) return "Every day";
  if (d == 0x3E) return "Weekdays";
  if (d == 0x41) return "Weekends";
  const char *n[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  String s;
  for (int i = 0; i < 7; i++) if (d & (1 << i)) { if (s.length()) s += " "; s += n[i]; }
  return s;
}
String hm12(int h, int m) { char b[12]; int hh = h % 12; if (hh == 0) hh = 12; snprintf(b, sizeof b, "%d:%02d %s", hh, m, h < 12 ? "AM" : "PM"); return b; }

void loadStage7() {
  prefs.begin("clock", true);
  String j = prefs.getString("rem", "");
  rainWarnOn = prefs.getBool("rainwarn", true);
  prefs.end();
  JsonDocument d;
  remCount = 0;
  if (j.length() && !deserializeJson(d, j))
    for (JsonObject o : d.as<JsonArray>()) {
      if (remCount >= REM_MAX) break;
      Reminder &r = rems[remCount++];
      memset(&r, 0, sizeof r);
      strlcpy(r.label, o["l"] | "Reminder", sizeof r.label);
      r.h = o["h"] | 8; r.m = o["m"] | 0; r.days = o["d"] | 0;
      r.year = o["y"] | 0; r.mon = o["mo"] | 0; r.mday = o["md"] | 0;
      r.on = (o["on"] | 1) != 0; r.lastFire = o["lf"] | 0u;
    }
}
void saveReminders() {
  JsonDocument d;
  JsonArray a = d.to<JsonArray>();
  for (int i = 0; i < remCount; i++) {
    Reminder &r = rems[i];
    JsonObject o = a.add<JsonObject>();
    o["l"] = r.label; o["h"] = r.h; o["m"] = r.m; o["d"] = r.days; o["y"] = r.year; o["mo"] = r.mon; o["md"] = r.mday; o["on"] = r.on ? 1 : 0; o["lf"] = r.lastFire;
  }
  String j; serializeJson(d, j);
  prefs.begin("clock", false); prefs.putString("rem", j); prefs.end();
}

void fireReminder(int i) {
  remActive = i; remStart = millis(); remDirty = true;
  Serial.printf("Reminder: %s\n", rems[i].label);
}
void reminderLoop() {
  struct tm t;
  if (!getLocalTime(&t, 0)) return;
  static int lastMin = -1;
  if (t.tm_min != lastMin) {
    lastMin = t.tm_min;
    uint32_t today = (t.tm_year + 1900) * 1000 + t.tm_yday;
    bool changed = false;
    for (int i = 0; i < remCount; i++) {
      Reminder &r = rems[i];
      if (!r.on || r.h != t.tm_hour || r.m != t.tm_min || r.lastFire == today) continue;
      bool due = r.days ? (r.days & (1 << t.tm_wday)) != 0
                        : (r.year == t.tm_year + 1900 && r.mon == t.tm_mon + 1 && r.mday == t.tm_mday);
      if (!due) continue;
      r.lastFire = today;
      if (!r.days) r.on = false;                               // a one-off is done once it has rung
      changed = true;
      fireReminder(i);
    }
    if (changed) saveReminders();
  }
  if (remSnoozed >= 0 && (int32_t)(millis() - remSnoozeAt) >= 0) { int i = remSnoozed; remSnoozed = -1; fireReminder(i); }
  if (remActive >= 0 && millis() - remStart > REM_SHOW_MS) { remActive = -1; remDirty = true; }
}
bool reminderShowing() { return remActive >= 0 && remActive < remCount; }

void drawBell(int x, int y) {
  display.fillCircle(x, y + 6, 7, GxEPD_BLACK);
  display.fillRect(x - 7, y + 6, 15, 7, GxEPD_BLACK);
  display.fillRect(x - 9, y + 12, 19, 3, GxEPD_BLACK);
  display.fillCircle(x, y + 17, 2, GxEPD_BLACK);
}
void drawReminder(bool full) {
  if (!reminderShowing()) return;
  Reminder r = rems[remActive];
  struct tm t; char hm[12] = "";
  if (getLocalTime(&t, 20)) { int h = t.tm_hour % 12; if (h == 0) h = 12; snprintf(hm, sizeof hm, "%d:%02d", h, t.tm_min); }
  String lines[9]; int n = 0;
  const GFXfont *f = &Label27; int lh = 30;
  wrapLines(String(r.label), f, 184, lines, n, 9);
  if (n > 4) { f = &Small13; lh = 17; wrapLines(String(r.label), f, 188, lines, n, 9); }
  if (n > 8) n = 8;
  display.setRotation(rotation);
  if (full) display.setFullWindow(); else display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    drawBell(16, 6);
    display.setFont(&Small13); display.setCursor(30, 22); display.print("Reminder");
    display.setCursor(196 - textWidth(hm, &Small13), 22); display.print(hm);
    display.fillRect(4, 32, 192, 2, GxEPD_BLACK);
    int top = 42, bottom = 168;
    int y = top + max(0, (bottom - top - n * lh) / 2) + lh - 6;
    for (int k = 0; k < n; k++) { centerText(lines[k].c_str(), 100, y, f); y += lh; }
    display.fillRect(4, 172, 192, 1, GxEPD_BLACK);
    centerText("Wave: done   2 taps: snooze", 100, 192, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// ---------- rain warning (filled in by fetchWeather) ----------
void parseRainSoon(JsonDocument &doc) {
  // hourly data starts at the current hour (forecast_hours=4): look at now and the next 3 hours
  bool was = rainSoon;
  rainSoon = false; rainHour = -1;
  bool rainingNow = (wx.code >= 51 && wx.code <= 67) || (wx.code >= 80 && wx.code <= 82) || wx.code >= 95;
  JsonArray tms = doc["hourly"]["time"], pp = doc["hourly"]["precipitation_probability"];
  for (size_t i = 0; i < pp.size() && i < tms.size() && i < 4; i++) {
    if ((int)(pp[i] | 0) >= 60) {
      String ts = tms[i] | "";                                 // e.g. "2026-10-13T16:00" (local time)
      rainSoon = true; rainHour = ts.length() >= 13 ? ts.substring(11, 13).toInt() : -1;
      break;
    }
  }
  if (rainingNow || !rainWarnOn) rainSoon = false;            // already raining, or switched off
  if (rainSoon && !was && ledsAllowed()) rainShimmerUntil = millis() + 8000;
}

// ---------- breathing guide ----------
bool breathing() {
  if (!breatheStart) return false;
  if (millis() - breatheStart >= (uint32_t)BR_CYCLE * BR_CYCLES) { breatheStart = 0; brDirty = true; return false; }
  return true;
}
void startBreathing() { breatheStart = millis(); if (!breatheStart) breatheStart = 1; brDirty = true; }
// phase: 0 in, 1 hold, 2 out; frac 0..1 through it; cycle 0..5
void breathPhase(uint32_t t, int &phase, float &frac, int &cycle) {
  uint32_t e = t - breatheStart;
  cycle = e / BR_CYCLE;
  int in = e % BR_CYCLE;
  if (in < BR_IN) { phase = 0; frac = in / (float)BR_IN; }
  else if (in < BR_IN + BR_HOLD) { phase = 1; frac = (in - BR_IN) / (float)BR_HOLD; }
  else { phase = 2; frac = (in - BR_IN - BR_HOLD) / (float)BR_OUT; }
}
void drawBreathe(bool full) {
  int phase, cycle; float frac;
  breathPhase(millis(), phase, frac, cycle);
  const char *word = phase == 0 ? "Breathe in" : phase == 1 ? "Hold" : "Breathe out";
  char sub[24]; snprintf(sub, sizeof sub, "%d of %d", cycle + 1, BR_CYCLES);
  display.setRotation(rotation);
  if (full) display.setFullWindow(); else display.setPartialWindow(0, 0, display.width(), display.height());
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    centerText(word, 100, 100, &Label27);
    centerText(sub, 100, 130, &Small13);
    centerText("Wave to stop", 100, 192, &Small13);
  } while (display.nextPage());
  display.hibernate();
}

// ---------- the ring ----------
// 0 nothing, 1 rain shimmer, 2 reminder, 3 breathing
int stage7Override() {
  if (millis() < heartFlashUntil) return 0;
  if (breathing()) return 3;
  if (reminderShowing() && millis() - remStart < REM_LIGHT_MS) return 2;
  if (millis() < rainShimmerUntil) return 1;
  return 0;
}
void renderStage7(int ov, uint32_t t) {
  int n = cfg.ledCount;
  memset(px, 0, sizeof px);
  if (ov == 3) {                                             // breathing: light rises up both sides from 6 to 12
    int phase, cycle; float frac;
    breathPhase(t, phase, frac, cycle);
    float level = phase == 0 ? 0.5f - 0.5f * cosf(frac * PI) : phase == 1 ? 1.0f : 0.5f + 0.5f * cosf(frac * PI);
    float reach = level * 180.0f;                            // degrees up from 6 o'clock on each side
    for (int i = 0; i < n; i++) {
      // angle of LED i from 6 o'clock (0..180 either way)
      float deg = cfg.ledClockwise ? i * 360.0f / n + (cfg.ledStartHour % 12) * 30.0f : (cfg.ledStartHour % 12) * 30.0f - i * 360.0f / n;
      float fromSix = angDist(deg, 180);
      float f = fromSix <= reach ? 1.0f : max(0.0f, 1.0f - (fromSix - reach) / 12.0f);
      float g = 0.12f + 0.88f * f;
      setPx(i, 40 * g, 170 * g, 160 * g);
    }
  } else if (ov == 2) {                                      // reminder: warm gold sweep that pulses
    float s = fmodf(t / 1600.0f, 1.0f) * 360.0f;
    float base = 0.25f + 0.15f * sinf(t / 300.0f);
    for (int i = 0; i < n; i++) setPx(i, 255 * base, 150 * base, 20 * base);
    blob(ledAt(s), 6.0f, 255, 190, 60);
    blob(ledAt(s + 180), 6.0f, 255, 190, 60);
  } else if (ov == 1) {                                      // rain soon: blue drops down both sides
    for (int i = 0; i < n; i++) setPx(i, 0, 10, 30);
    for (int d = 0; d < 4; d++) {
      float ph = fmodf(t / 1800.0f + d * 0.25f, 1.0f) * 180.0f;
      blob(ledAt(ph), 1.8f, 120, 180, 255);
      blob(ledAt(360 - ph), 1.8f, 120, 180, 255);
    }
  }
}

// ---------- gestures ----------
bool stage7Gesture(int g) {
  if (breathing()) { breatheStart = 0; brDirty = true; return true; }       // any wave stops the breathing guide
  if (reminderShowing()) {
    if (g == G_DOUBLE) { remSnoozed = remActive; remSnoozeAt = millis() + 10UL * 60 * 1000; }
    remActive = -1; remDirty = true;
    return true;
  }
  return false;
}

// ---------- settings page: /rem?op=add|del|toggle ----------
void handleReminders() {
  String op = server.arg("op"), msg = "Done";
  if (op == "add") {
    if (remCount >= REM_MAX) msg = "That's the most reminders the clock can hold - delete one first";
    else {
      Reminder r; memset(&r, 0, sizeof r);
      String label = cleanText(server.arg("label"));
      if (label.isEmpty()) label = "Reminder";
      strlcpy(r.label, label.c_str(), sizeof r.label);
      int hh = 0, mm = 0;
      sscanf(server.arg("time").c_str(), "%d:%d", &hh, &mm);
      r.h = constrain(hh, 0, 23); r.m = constrain(mm, 0, 59);
      r.days = server.arg("days").toInt() & 0x7F;
      if (!r.days) {
        int y = 0, mo = 0, md = 0;
        if (sscanf(server.arg("date").c_str(), "%d-%d-%d", &y, &mo, &md) != 3) { server.send(200, "text/plain", "Please pick a date"); return; }
        r.year = y; r.mon = mo; r.mday = md;
      }
      r.on = true;
      rems[remCount++] = r;
      saveReminders();
      msg = "Reminder saved";
    }
  } else if (op == "del" || op == "toggle") {
    int i = server.arg("i").toInt();
    if (i >= 0 && i < remCount) {
      if (op == "del") {
        for (int k = i; k < remCount - 1; k++) rems[k] = rems[k + 1];
        remCount--;
        if (remActive == i) remActive = -1; else if (remActive > i) remActive--;
        if (remSnoozed == i) remSnoozed = -1; else if (remSnoozed > i) remSnoozed--;
        msg = "Reminder deleted";
      } else { rems[i].on = !rems[i].on; rems[i].lastFire = 0; msg = rems[i].on ? "Reminder on" : "Reminder off"; }
      saveReminders();
    }
  } else if (op == "test") { if (remCount) fireReminder(remCount - 1); msg = "Showing it on the clock now"; }
  server.send(200, "text/plain", msg);
}

void handlePage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", SETTINGS_PAGE);
}

void handleState() {
  JsonDocument j;
  JsonObject s = j["s"].to<JsonObject>();
  s["on"] = ledsWanted ? 1 : 0;
  s["effect"] = cfg.effect;
  char col[8]; snprintf(col, sizeof col, "#%06lx", (unsigned long)cfg.color); s["color"] = col;
  s["bright"] = cfg.brightness;
  s["onsec"] = cfg.onSeconds;
  s["night"] = cfg.nightOn ? 1 : 0; s["nfrom"] = cfg.nightFrom; s["nto"] = cfg.nightTo;
  JsonArray g = s["g"].to<JsonArray>();
  for (int i = 0; i < G_COUNT; i++) g.add(cfg.gestureAction[i]);
  s["wake"] = cfg.wakeOnAny ? 1 : 0;
  s["srot"] = cfg.sensorRotate; s["smir"] = cfg.sensorMirror ? 1 : 0; s["sens"] = cfg.sensitivity;
  s["nled"] = cfg.ledCount; s["maxma"] = cfg.maxmA; s["lstart"] = cfg.ledStartHour % 12; s["ldir"] = cfg.ledClockwise ? 1 : 0;
  s["em"] = cfg.effectMask; s["chime"] = cfg.hourChime ? 1 : 0; s["fast"] = (int)fastMs; s["pulse"] = (int)pulseMs;
  s["units"] = useF ? "F" : "C"; s["zip"] = zipCode;
  s["hbglow"] = hbGlowOn ? 1 : 0;
  s["rainwarn"] = rainWarnOn ? 1 : 0;
  JsonArray ra = s["rem"].to<JsonArray>();
  for (int i = 0; i < remCount; i++) {
    JsonObject o = ra.add<JsonObject>();
    o["l"] = rems[i].label; o["on"] = rems[i].on ? 1 : 0;
    o["when"] = hm12(rems[i].h, rems[i].m) + ", " + (rems[i].days ? daysText(rems[i].days) : String(rems[i].mon) + "/" + String(rems[i].mday) + "/" + String(rems[i].year));
  }
  s["autoupd"] = autoUpdate ? 1 : 0; s["health"] = healthOn ? 1 : 0; s["otaurl"] = otaBase; s["ntfy"] = ntfyTopic;

  JsonObject l = j["live"].to<JsonObject>();
  struct tm t;
  if (getLocalTime(&t, 0)) {
    char b[24];
    int h = t.tm_hour % 12; if (h == 0) h = 12;
    snprintf(b, sizeof b, "%d:%02d %s", h, t.tm_min, t.tm_hour < 12 ? "AM" : "PM"); l["time"] = b;
    strftime(b, sizeof b, "%a, %b %e", &t); l["date"] = b;
  }
  if (!isnan(roomC)) {
    char b[32]; snprintf(b, sizeof b, "%.0f\xC2\xB0%s \xC2\xB7 %.0f%%", useF ? roomC * 9 / 5 + 32 : roomC, useF ? "F" : "C", roomRH);
    l["room"] = b;
  }
  if (wx.ok) {
    String cond = conditionText(wx.code); cond.toLowerCase();
    char b[48]; snprintf(b, sizeof b, "%.0f\xC2\xB0%s, %s", wx.temp, useF ? "F" : "C", cond.c_str());
    l["outside"] = b;
  }
  l["place"] = placeName;
  if (lastGesture >= 0) {
    l["gesture"] = GESTURE_FRIENDLY[lastGesture];
    l["gestureAgo"] = (millis() - lastGestureAt) / 1000;
    l["gestureId"] = gestureCount;
  }
  l["prox"] = lastProximity;
  l["proxBase"] = apdsIs9960 ? proxBase : proxRef;
  l["gready"] = apdsIs9960 || bgReady;
  l["sensor"] = !apdsOK ? "none" : apdsIs9960 ? "9960" : "9930";
  JsonArray gs = l["gestures"].to<JsonArray>();
  for (int i = 0; i < G_COUNT; i++) if (apdsOK && gestureSupported(i)) gs.add(i);
  // hands
  l["handsShow"] = handTimeText(handPos);
  int32_t target;
  if (handsHold) { l["handsState"] = "held"; l["handsChip"] = "Paused"; l["handsNote"] = "The hands are paused. Use \"Hands showing the wrong time?\" to set them and start them again."; }
  else if (!targetPos(target)) { l["handsState"] = "wait"; l["handsChip"] = "Getting the time"; l["handsNote"] = "Waiting for the time from the internet."; }
  else {
    int32_t d = target - handPos;
    if (d > CYCLE / 2) d -= CYCLE;
    if (d < -CYCLE / 2) d += CYCLE;
    if (abs(d) <= 2) { l["handsState"] = "ok"; l["handsChip"] = "On time"; l["handsNote"] = "The hands are showing the right time."; }
    else if (d > 0) { l["handsState"] = "behind"; l["handsChip"] = "Catching up"; l["handsNote"] = "The hands are " + fmtDuration(d) + " behind and moving a little faster until they're right."; }
    else { l["handsState"] = "ahead"; l["handsChip"] = "Waiting"; l["handsNote"] = "The hands are " + fmtDuration(d) + " ahead. They'll stay still until the real time catches up."; }
  }
  String why;
  if (ledsWanted && !ledsAllowed(&why)) l["ledsBlocked"] = why;
  JsonArray ma = l["msgs"].to<JsonArray>();
  xSemaphoreTake(msgLock, portMAX_DELAY);
  for (int i = msgCount - 1; i >= 0; i--) {
    JsonObject o = ma.add<JsonObject>();
    o["t"] = msgs[i].text; o["seen"] = msgs[i].seen; o["heart"] = msgs[i].hearted;
    long ago = timeKnown() && msgs[i].at ? (long)time(nullptr) - (long)msgs[i].at : -1;
    o["ago"] = ago < 0 ? String("") : ago < 90 ? String("just now") : ago < 5400 ? String(ago / 60) + " min ago" : ago < 129600 ? String(ago / 3600) + " h ago" : String(ago / 86400) + " days ago";
  }
  xSemaphoreGive(msgLock);
  l["amb"] = apdsOK && !apdsIs9960 ? ambientLevel() : -1;
  l["gblock"] = sensorBlocked();
  if (rainSoon) {
    int h12 = rainHour % 12; if (h12 == 0) h12 = 12;
    l["rain"] = rainHour >= 0 ? "Rain likely around " + String(h12) + (rainHour < 12 ? " AM" : " PM") : String("Rain likely soon");
  }
  l["ver"] = fwVersion();
  l["upd"] = otaState == 1 ? String("Checking...") : otaState == 2 ? "Downloading " + String(otaPct) + "%" : otaState == 3 ? String("Restarting...") : otaStatus;
  l["uptime"] = uptimeText();
  l["wifi"] = WiFi.SSID();
  l["ip"] = WiFi.localIP().toString();
  String out;
  serializeJson(j, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleSet() {
  String k = server.arg("k"), v = server.arg("v");
  int n = v.toInt();
  String msg = "Saved";
  bool light = true;      // saves the light settings block at the end
  if (k == "on") { ledsWanted = n != 0; ledsLastActivity = millis(); light = false; msg = ledsWanted ? "Light on" : "Light off"; }
  else if (k == "effect") { cfg.effect = constrain(n, 0, E_COUNT - 1); ledsWanted = true; ledsLastActivity = millis(); }
  else if (k == "color") { cfg.color = strtoul(v.c_str() + (v[0] == '#' ? 1 : 0), nullptr, 16); ledsWanted = true; ledsLastActivity = millis(); }
  else if (k == "bright") { cfg.brightness = constrain(n, 10, 255); ledsWanted = true; ledsLastActivity = millis(); }
  else if (k == "onsec") cfg.onSeconds = constrain(n, 10, 3600);
  else if (k == "night") cfg.nightOn = n != 0;
  else if (k == "nfrom") cfg.nightFrom = constrain(n, 0, 23);
  else if (k == "nto") cfg.nightTo = constrain(n, 0, 23);
  else if (k.length() == 2 && k[0] == 'g' && isdigit(k[1]) && k[1] - '0' < G_COUNT) cfg.gestureAction[k[1] - '0'] = constrain(n, 0, A_COUNT - 1);
  else if (k == "wake") cfg.wakeOnAny = n != 0;
  else if (k == "srot") cfg.sensorRotate = constrain(n, 0, 3);
  else if (k == "smir") cfg.sensorMirror = n != 0;
  else if (k == "sens") { cfg.sensitivity = constrain(n, 1, 3); if (apdsOK) apdsConfigure(); msg = "Sensitivity set - keep your hand away for a second"; }
  else if (k == "nled") cfg.ledCount = constrain(n, 1, MAX_LEDS);
  else if (k == "maxma") cfg.maxmA = constrain(n, 100, MAX_SAFE_MA);
  else if (k == "lstart") cfg.ledStartHour = constrain(n, 0, 11);
  else if (k == "ldir") cfg.ledClockwise = n != 0;
  else if (k == "em") cfg.effectMask = (n & ALL_EFFECTS) ? (n & ALL_EFFECTS) : 1;
  else if (k == "chime") cfg.hourChime = n != 0;
  else {
    light = false;
    prefs.begin("clock", false);
    if (k == "fast" && n >= 10 && n <= 62) { fastMs = n; prefs.putUShort("fast", n); }
    else if (k == "pulse" && n >= 3 && n <= 30) { pulseMs = n; prefs.putUChar("pulse", n); }
    else if (k == "rainwarn") { rainWarnOn = n != 0; prefs.putBool("rainwarn", rainWarnOn); if (!rainWarnOn) rainSoon = false; lastDrawnKey = ""; }
    else if (k == "hbglow") { hbGlowOn = n != 0; prefs.putBool("hbglow", hbGlowOn); if (!hbGlowOn) hbUntil = 0; }
    else if (k == "autoupd") { autoUpdate = n != 0; prefs.putBool("autoupd", autoUpdate); }
    else if (k == "health") { healthOn = n != 0; prefs.putBool("health", healthOn); }
    else if (k == "otaurl") {
      v.trim(); while (v.endsWith("/")) v.remove(v.length() - 1);
      if (v.isEmpty() || v.startsWith("https://")) { otaBase = v; prefs.putString("otaurl", v); msg = v.isEmpty() ? "Update address cleared" : "Update address saved"; }
      else msg = "The address should start with https://";
    }
    else if (k == "ntfy") {
      v.trim();
      bool okName = v.length() >= 6 && v.length() <= 60;
      for (char ch : v) if (!isalnum(ch) && ch != '-' && ch != '_') okName = false;
      if (okName) { ntfyTopic = v; prefs.putString("ntfy", v); prefs.remove("ncmd"); lastCmdId = ""; msg = "Health topic saved"; }
      else msg = "Use 6-60 letters, numbers, - or _";
    }
    else if (k == "units") { useF = v != "C"; prefs.putBool("useF", useF); lastWeather = 0; forceFullNext = true; }
    else if (k == "zip") {
      v.trim();
      if (v.length() == 5) { zipCode = v; prefs.putString("zip", v); lat = lon = 0; placeName = ""; lastWeather = 0; msg = "Location saved - weather updates in a minute"; }
      else msg = "Please type a 5-digit zip code";
    }
    else msg = "Unknown setting";
    prefs.end();
  }
  if (light) saveLightConfig();
  server.send(200, "text/plain", msg);
}

void handleDo() {
  String a = server.arg("a"), v = server.arg("v"), msg;
  if (a == "leds") { doAction(A_TOGGLE, false); msg = ledsWanted ? "Light on" : "Light off"; }
  else if (a == "weather") { doAction(A_WEATHER, false); msg = "Weather on the screen for 20 s"; }
  else if (a == "qr") { doAction(A_QR, false); msg = "QR code on the screen for 1 minute"; }
  else if (a == "hold") { handsHold = !handsHold; msg = handsHold ? "Hands paused" : "Hands running"; }
  else if (a == "pause") { handsHold = true; msg = "Hands paused"; }
  else if (a == "sethands") {
    int hh = 0, mm = 0, ss = 0;
    if (sscanf(v.c_str(), "%d:%d:%d", &hh, &mm, &ss) >= 2 && hh >= 1 && hh <= 12 && mm < 60 && ss < 60) {
      posRequest = ((hh % 12) * 3600 + mm * 60 + ss) * 16;
      delay(5); handsHold = false; saveHands(true);
      msg = "Hands set - catching up now";
    } else msg = "That time didn't look right";
  }
  else if (a == "nudge") { posRequest = wrapPos(handPos + v.toInt()); delay(5); saveHands(true); msg = "Done"; }
  else if (a == "speedtest") { testRequest = 1; msg = "Speed test running"; }
  else if (a == "rotate") {
    rotation = (rotation + 1) % 4;
    prefs.begin("clock", false); prefs.putUChar("rot", rotation); prefs.end();
    server.send(200, "text/plain", "Screen turned");
    forceFullNext = true; lastDrawnKey = "";
    return;
  }
  else if (a == "locate") {
    ledsLocate = true; msg = "First LED is red";
    xTaskCreate([](void *) { vTaskDelay(pdMS_TO_TICKS(10000)); ledsLocate = false; vTaskDelete(nullptr); }, "loc", 2048, nullptr, 1, nullptr);
  }
  else if (a == "restart") {
    server.send(200, "text/plain", "Restarting - the page will come back in about 30 seconds");
    notify("Restarting", "Restart pressed on the settings page.", "arrows_counterclockwise", 2);
    restartPending = true; restartAt = millis() + 4000;
    return;
  }
  else if (a == "update") { wantUpdate = true; msg = "Checking for an update..."; }
  else if (a == "breathe") { startBreathing(); msg = "Breathing guide starting on the clock"; }
  else if (a == "msgseen") { for (int i = 0; i < msgCount; i++) markSeen(i, " (on the settings page)"); msg = "Marked as seen"; }
  else if (a == "msgheart") {
    int i = msgCount - 1;
    if (i >= 0) { markSeen(i, " (on the settings page)"); sendHeart(i); msg = "Heart sent"; } else msg = "No messages yet";
  }
  else if (a == "msgclear") {
    xSemaphoreTake(msgLock, portMAX_DELAY); msgCount = 0; msgShow = -1; xSemaphoreGive(msgLock);
    hbUntil = 0; msgSaveNeeded = true; msgDirty = true; msg = "Messages cleared";
  }
  else if (a == "testmsg") { notify("Test from the clock", statusLine(), "wave", 3); msg = "Test message sent to " + ntfyTopic; }
  else if (a == "forgetwifi") {
    server.send(200, "text/plain", "Restarting into WiFi setup");
    prefs.begin("clock", false); prefs.remove("ssid"); prefs.remove("pass"); prefs.end();
    saveHands(true); delay(500); ESP.restart();
  }
  server.send(200, "text/plain", msg);
}

void startSettingsServer() {
  server.on("/", HTTP_GET, handlePage);
  server.on("/state", HTTP_GET, handleState);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/do", HTTP_GET, handleDo);
  server.on("/rem", HTTP_GET, handleReminders);
  server.onNotFound([] { server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); });
  server.begin();
  if (MDNS.begin("smartclock")) MDNS.addService("http", "tcp", 80);
  Serial.printf("Settings page: http://%s  (or http://smartclock.local on iPhone / computers)\n", WiFi.localIP().toString().c_str());
}

void setup() {
  // LED strip OFF and coil idle before anything else
  pinMode(PIN_LED_GATE, OUTPUT); digitalWrite(PIN_LED_GATE, LOW);
  pinMode(PIN_LED_DATA, INPUT);            // high-impedance while the strip is unpowered
  pinMode(PIN_COIL_A, OUTPUT); digitalWrite(PIN_COIL_A, LOW);
  pinMode(PIN_COIL_B, OUTPUT); digitalWrite(PIN_COIL_B, LOW);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(500);
  Serial.println("\nSmart Clock v2 - stage 7");
  loadSettings();
  stage5Begin();
  msgLock = xSemaphoreCreateMutex();
  loadMsgs();
  loadStage7();

  // where are the hands? RTC memory (exact, survives resets) or flash (last minute-save)
  if (rtcMagic == 0xC10C2026 && (rtcPos ^ 0xA5A5A5A5) == rtcCheck && rtcPos < (uint32_t)CYCLE) {
    handPos = rtcPos;
    Serial.printf("Hands at %s (from memory)\n", handTimeText(handPos).c_str());
  } else {
    prefs.begin("clock", true);
    bool known = prefs.isKey("hand");
    handPos = wrapPos(prefs.getInt("hand", 0));
    prefs.end();
    if (known) Serial.printf("Hands at %s (from flash - if wrong, set with  c HH:MM:SS)\n", handTimeText(handPos).c_str());
    else {
      handsHold = true;      // never set before: keep still until we are told what they show
      Serial.println("Hand position not set yet. Hands are held. Type  c HH:MM:SS  with the time they show.");
    }
  }
  Wire.begin();
  xTaskCreate(handsTask, "hands", 4096, nullptr, 5, nullptr);
  apdsOK = apdsBegin();
  xTaskCreate(ledTask, "leds", 4096, nullptr, 3, nullptr);

  SPI.begin(PIN_EPD_CLK, -1, PIN_EPD_DIN, -1);           // no MISO: D9 is the LED switch
  display.epd2.selectSPI(SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
  display.init(0, true, 10, false);

  if (!connectWifi()) { startSetupPortal(); return; }
  startSettingsServer();
  stage5Start();
  qrUntil = millis() + 60000;          // settings QR code for the first minute (drawn by loop)
  configTime(wx.utcOffset, 0, "pool.ntp.org", "time.nist.gov");   // last known time zone until the weather call confirms it
  updateAll(true);
}

void loop() {
  stage5Loop();
  if (setupMode) {
    dns.processNextRequest();
    server.handleClient();
    static uint32_t since = millis();
    if (millis() - since > 10UL * 60 * 1000 && !wifiSsid.isEmpty()) ESP.restart();   // after 10 min, try the saved WiFi again
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      static uint32_t lastTry = 0;
      if (millis() - lastTry > 60000) { lastTry = millis(); WiFi.reconnect(); }
    }
    server.handleClient();

    // gestures
    static uint32_t lastPoll = 0;
    if (millis() - lastPoll >= 15) {
      lastPoll = millis();
      int g = pollGesture();
      if (g >= 0) onGesture(g);
    }

    // updating: say so on the screen (the hands keep their place and catch up afterwards)
    static int otaShown = 0;
    if (otaState >= 2 && otaShown != otaState) {
      otaShown = otaState;
      drawMessage("Updating", "New clock software is", "being installed. Please", "keep it plugged in.");
    }
    if (otaState >= 2) { delay(20); return; }
    if (otaShown && otaState == 0) { otaShown = 0; lastDrawnKey = ""; forceFullNext = true; updateAll(false); }   // install didn't happen: back to the clock

    // weather screen for 20 s, then back to the clock screen
    msgLoop();
    reminderLoop();
    // what the screen shows: QR code > weather > breathing guide > reminder > message > the clock
    uint8_t want = millis() < qrUntil ? 2 : millis() < weatherUntil ? 1 : breathing() ? 5 : reminderShowing() ? 4 : msgShow >= 0 ? 3 : 0;
    static int ovMin = -1, brPhase = -1;
    struct tm mt; int curMin = getLocalTime(&mt, 0) ? mt.tm_min : -1;
    int ph = -1;
    if (want == 5) { int c; float f; breathPhase(millis(), ph, f, c); ph += c * 3; }
    auto drawOv = [&](uint8_t w, bool full) {
      msgDirty = remDirty = brDirty = false; ovMin = curMin; brPhase = ph;
      if (w == 3) drawMsgScreen(full); else if (w == 4) drawReminder(full); else drawBreathe(full);
    };
    if (want >= 3 && overlayShown == want) {
      bool dirty = want == 3 ? (msgDirty || curMin != ovMin) : want == 4 ? (remDirty || curMin != ovMin) : (ph != brPhase);
      if (dirty) drawOv(want, false);
    }
    if (want != overlayShown) {
      overlayShown = want;
      weatherShowing = want != 0;
      if (want >= 3) drawOv(want, true);
      else if (want == 1) drawWeather();
      else if (want == 2) {
        String url = "http://" + WiFi.localIP().toString() + "/";
        drawQR("Clock settings", url.c_str(), "Point your phone camera here", WiFi.localIP().toString().c_str(), false);
      } else { lastDrawnKey = ""; updateAll(false); }
    }

    // battery reserve for the LEDs: if the battery is running down, stop the LEDs first so the hands keep going
    static uint32_t lastBattCheck = 0;
    if (millis() - lastBattCheck > 10000) {
      lastBattCheck = millis();
      float v = batteryVolts(), mA = batteryMilliamps();
      battV = v;
      if (!ledBatteryRest && v > 2.5f && v < 3.65f && mA > 20) { ledBatteryRest = true; Serial.printf("Battery %.2f V: LEDs resting\n", v); }
      else if (ledBatteryRest && (v > 3.85f || mA < -20)) { ledBatteryRest = false; Serial.println("Battery OK: LEDs allowed again"); }
    }

    static int lastMin = -1;
    struct tm now;
    if (getLocalTime(&now, 0) && now.tm_min != lastMin) { lastMin = now.tm_min; updateAll(false); }
    else if (millis() - lastDraw > 60UL * 1000) updateAll(false);
  }

  saveHands(false);

  // Serial commands: a line ends with Enter, or after a short pause (works with any line-ending setting)
  static String line;
  static uint32_t lastChar = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (line.length()) { handleCommand(line); line = ""; } }
    else { line += ch; lastChar = millis(); }
  }
  if (line.length() && millis() - lastChar > 400) { handleCommand(line); line = ""; }
  delay(2);
}


// =====================================================================
//  Font bitmaps (made from Inter Display with make_gfx_font.py)
// =====================================================================
// BigTime: generated from InterDisplay-Bold.otf at 92px by make_gfx_font.py
const uint8_t BigTimeBitmaps[] PROGMEM = {
  0x1F,0xFC,0x0F,0xFE,0x07,0xFF,0x03,0xFF,0x81,0xFF,0x80,0xFF,0xC0,0x7F,0xE0,0x3F,
  0xF0,0x1F,0xF8,0x0F,0xFC,0x07,0xFE,0x03,0xFF,0x01,0xFF,0x80,0xFF,0xC0,0x7F,0xE0,
  0x3F,0xF0,0x1F,0xF8,0x07,0xFC,0x03,0xFE,0x01,0xFF,0x00,0xFF,0x80,0x7F,0xC0,0x3F,
  0xE0,0x1F,0xF0,0x0F,0xF8,0x07,0xFC,0x03,0xFE,0x01,0xFF,0x00,0xFF,0x80,0x7F,0xC0,
  0x3F,0xE0,0x1F,0xF0,0x0F,0xF8,0x07,0xFC,0x03,0xFE,0x01,0xFF,0x00,0xFF,0x80,0x7F,
  0xC0,0x3F,0xE0,0x1F,0xF0,0x0F,0xF8,0x07,0xFC,0x03,0xFE,0x01,0xFF,0x00,0xFF,0x80,
  0x7F,0xC0,0x3F,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x1F,0x00,0x1F,0xC0,0x1F,0xF0,0x0F,0xF8,0x0F,0xFE,0x07,0xFF,0x03,0xFF,0x81,
  0xFF,0xC0,0xFF,0xE0,0x7F,0xF0,0x3F,0xF0,0x0F,0xF8,0x07,0xFC,0x01,0xFC,0x00,0x78,
  0x00,0x07,0xFC,0x1F,0xF0,0x03,0xFE,0x0F,0xF8,0x01,0xFF,0x07,0xFC,0x00,0xFF,0x83,
  0xFE,0x00,0x7F,0xC1,0xFF,0x00,0x3F,0xE0,0xFF,0x80,0x1F,0xF0,0x7F,0xC0,0x0F,0xF8,
  0x3F,0xE0,0x07,0xFC,0x1F,0xF0,0x03,0xFE,0x0F,0xF8,0x01,0xFF,0x03,0xFC,0x00,0xFF,
  0x81,0xFE,0x00,0x7F,0xC0,0xFF,0x00,0x3F,0xE0,0x7F,0x80,0x1F,0xF0,0x3F,0xC0,0x07,
  0xF8,0x1F,0xE0,0x03,0xFC,0x0F,0xF0,0x01,0xFE,0x07,0xF8,0x00,0xFF,0x03,0xFC,0x00,
  0x7F,0x01,0xFE,0x00,0x3F,0x80,0xFF,0x00,0x1F,0xC0,0x7F,0x80,0x0F,0xE0,0x3F,0xC0,
  0x07,0xF0,0x1F,0xE0,0x03,0xF8,0x0F,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xFC,0x00,0xFF,0x00,0x00,0x1F,0xE0,0x07,
  0xF8,0x00,0x00,0xFE,0x00,0x3F,0x80,0x00,0x07,0xF0,0x01,0xFC,0x00,0x00,0x3F,0x80,
  0x1F,0xE0,0x00,0x03,0xFC,0x00,0xFF,0x00,0x00,0x1F,0xE0,0x07,0xF8,0x00,0x00,0xFF,
  0x00,0x3F,0xC0,0x00,0x07,0xF8,0x01,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xE0,0x00,0x01,
  0xFC,0x00,0x7F,0x00,0x00,0x0F,0xE0,0x03,0xF8,0x00,0x00,0xFF,0x00,0x3F,0xC0,0x00,
  0x07,0xF8,0x01,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xF0,0x00,0x01,0xFE,0x00,0x7F,0x80,
  0x00,0x0F,0xF0,0x03,0xFC,0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0xC1,0xFF,0xFF,0xFF,0xFF,
  0xFC,0x0F,0xFF,0xFF,0xFF,0xFF,0xE0,0x7F,0xFF,0xFF,0xFF,0xFF,0x07,0xFF,0xFF,0xFF,
  0xFF,0xF8,0x3F,0xFF,0xFF,0xFF,0xFF,0xC1,0xFF,0xFF,0xFF,0xFF,0xFE,0x0F,0xFF,0xFF,
  0xFF,0xFF,0xF0,0x7F,0xFF,0xFF,0xFF,0xFF,0x83,0xFF,0xFF,0xFF,0xFF,0xF8,0x00,0x3F,
  0x80,0x0F,0xE0,0x00,0x01,0xFC,0x00,0xFF,0x00,0x00,0x1F,0xE0,0x07,0xF8,0x00,0x00,
  0xFF,0x00,0x3F,0xC0,0x00,0x07,0xF8,0x01,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xF0,0x00,
  0x01,0xFE,0x00,0x7F,0x00,0x00,0x0F,0xE0,0x03,0xF8,0x00,0x00,0x7F,0x00,0x1F,0xC0,
  0x00,0x07,0xF8,0x01,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xF0,0x00,0x01,0xFE,0x00,0x7F,
  0x80,0x00,0x0F,0xF0,0x03,0xFC,0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0xC1,0xFF,0xFF,0xFF,
  0xFF,0xFC,0x0F,0xFF,0xFF,0xFF,0xFF,0xE0,0x7F,0xFF,0xFF,0xFF,0xFF,0x07,0xFF,0xFF,
  0xFF,0xFF,0xF8,0x3F,0xFF,0xFF,0xFF,0xFF,0xC1,0xFF,0xFF,0xFF,0xFF,0xFE,0x0F,0xFF,
  0xFF,0xFF,0xFF,0xF0,0x7F,0xFF,0xFF,0xFF,0xFF,0x83,0xFF,0xFF,0xFF,0xFF,0xF8,0x00,
  0x3F,0x80,0x0F,0xE0,0x00,0x01,0xFC,0x00,0x7F,0x00,0x00,0x0F,0xE0,0x07,0xF8,0x00,
  0x00,0xFF,0x00,0x3F,0xC0,0x00,0x07,0xF8,0x01,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xF0,
  0x00,0x01,0xFE,0x00,0x7F,0x80,0x00,0x0F,0xF0,0x03,0xF8,0x00,0x00,0x7F,0x00,0x1F,
  0xC0,0x00,0x03,0xF8,0x00,0xFE,0x00,0x00,0x3F,0xC0,0x0F,0xF0,0x00,0x01,0xFE,0x00,
  0x7F,0x80,0x00,0x0F,0xF0,0x03,0xFC,0x00,0x00,0x7F,0x80,0x1F,0xE0,0x00,0x03,0xFC,
  0x00,0xFF,0x00,0x00,0x1F,0xE0,0x07,0xF0,0x00,0x00,0xFE,0x00,0x3F,0x80,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,
  0x00,0x00,0x03,0xF0,0x00,0x00,0x00,0x00,0x0F,0xC0,0x00,0x00,0x00,0x00,0x3F,0x00,
  0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x03,0xF0,0x00,0x00,0x00,0x00,0x0F,
  0xC0,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x00,
  0x7F,0xFF,0x80,0x00,0x00,0x07,0xFF,0xFF,0x80,0x00,0x00,0x7F,0xFF,0xFF,0x80,0x00,
  0x03,0xFF,0xFF,0xFF,0x00,0x00,0x1F,0xFF,0xFF,0xFE,0x00,0x00,0xFF,0xFF,0xFF,0xFC,
  0x00,0x07,0xFF,0xFF,0xFF,0xF0,0x00,0x1F,0xFF,0xFF,0xFF,0xE0,0x00,0xFF,0xFF,0xFF,
  0xFF,0xC0,0x07,0xFF,0xFF,0xFF,0xFF,0x00,0x1F,0xFF,0xFF,0xFF,0xFC,0x00,0x7F,0xFC,
  0xFE,0xFF,0xF8,0x03,0xFF,0xE3,0xF1,0xFF,0xE0,0x0F,0xFF,0x0F,0xC3,0xFF,0x80,0x3F,
  0xF8,0x3F,0x07,0xFF,0x00,0xFF,0xE0,0xFC,0x1F,0xFC,0x03,0xFF,0x03,0xF0,0x3F,0xF0,
  0x0F,0xFC,0x0F,0xC0,0xFF,0xC0,0x7F,0xF0,0x3F,0x03,0xFF,0x01,0xFF,0xC0,0xFC,0x0F,
  0xFC,0x07,0xFF,0x03,0xF0,0x00,0x00,0x0F,0xFE,0x0F,0xC0,0x00,0x00,0x3F,0xF8,0x3F,
  0x00,0x00,0x00,0xFF,0xF0,0xFC,0x00,0x00,0x03,0xFF,0xE3,0xF0,0x00,0x00,0x0F,0xFF,
  0xEF,0xC0,0x00,0x00,0x1F,0xFF,0xFF,0x00,0x00,0x00,0x7F,0xFF,0xFE,0x00,0x00,0x01,
  0xFF,0xFF,0xFE,0x00,0x00,0x03,0xFF,0xFF,0xFF,0x00,0x00,0x07,0xFF,0xFF,0xFF,0x00,
  0x00,0x0F,0xFF,0xFF,0xFF,0x00,0x00,0x3F,0xFF,0xFF,0xFE,0x00,0x00,0x3F,0xFF,0xFF,
  0xFC,0x00,0x00,0x7F,0xFF,0xFF,0xF8,0x00,0x00,0x7F,0xFF,0xFF,0xF0,0x00,0x00,0x7F,
  0xFF,0xFF,0xE0,0x00,0x00,0x7F,0xFF,0xFF,0x80,0x00,0x00,0x3F,0xFF,0xFF,0x00,0x00,
  0x00,0xFF,0xFF,0xFC,0x00,0x00,0x03,0xFF,0xFF,0xF8,0x00,0x00,0x0F,0xC7,0xFF,0xE0,
  0x00,0x00,0x3F,0x0F,0xFF,0x80,0x00,0x00,0xFC,0x1F,0xFE,0x00,0x00,0x03,0xF0,0x3F,
  0xF8,0x3F,0xF8,0x0F,0xC0,0x7F,0xF0,0xFF,0xE0,0x3F,0x01,0xFF,0xC1,0xFF,0x80,0xFC,
  0x07,0xFF,0x07,0xFE,0x03,0xF0,0x1F,0xFC,0x1F,0xF8,0x0F,0xC0,0x7F,0xE0,0x7F,0xF0,
  0x3F,0x01,0xFF,0x81,0xFF,0xC0,0xFC,0x0F,0xFE,0x07,0xFF,0x83,0xF0,0x3F,0xF8,0x1F,
  0xFE,0x0F,0xC1,0xFF,0xE0,0x3F,0xFC,0x3F,0x0F,0xFF,0x80,0xFF,0xFC,0xFC,0x7F,0xFC,
  0x01,0xFF,0xFF,0xFF,0xFF,0xF0,0x07,0xFF,0xFF,0xFF,0xFF,0x80,0x1F,0xFF,0xFF,0xFF,
  0xFE,0x00,0x3F,0xFF,0xFF,0xFF,0xF0,0x00,0x7F,0xFF,0xFF,0xFF,0xC0,0x01,0xFF,0xFF,
  0xFF,0xFE,0x00,0x03,0xFF,0xFF,0xFF,0xF0,0x00,0x07,0xFF,0xFF,0xFF,0x80,0x00,0x07,
  0xFF,0xFF,0xFC,0x00,0x00,0x0F,0xFF,0xFF,0xC0,0x00,0x00,0x0F,0xFF,0xFC,0x00,0x00,
  0x00,0x03,0xFF,0x80,0x00,0x00,0x00,0x03,0xF0,0x00,0x00,0x00,0x00,0x0F,0xC0,0x00,
  0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x03,0xF0,
  0x00,0x00,0x00,0x00,0x0F,0xC0,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x00,
  0xFC,0x00,0x00,0x00,0x00,0x03,0xF0,0x00,0x00,0x00,0x07,0xE0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x0F,0xFC,0x00,0x00,0x01,0xFC,0x00,0x00,0x0F,0xFF,0x80,0x00,0x01,0xFE,
  0x00,0x00,0x0F,0xFF,0xE0,0x00,0x00,0xFE,0x00,0x00,0x0F,0xFF,0xF0,0x00,0x00,0xFF,
  0x00,0x00,0x0F,0xFF,0xFC,0x00,0x00,0x7F,0x00,0x00,0x07,0xFF,0xFF,0x00,0x00,0x7F,
  0x80,0x00,0x07,0xFF,0xFF,0x80,0x00,0x3F,0x80,0x00,0x03,0xFE,0x7F,0xC0,0x00,0x3F,
  0xC0,0x00,0x03,0xFC,0x0F,0xF0,0x00,0x1F,0xC0,0x00,0x01,0xFE,0x07,0xF8,0x00,0x1F,
  0xE0,0x00,0x00,0xFE,0x01,0xFC,0x00,0x0F,0xE0,0x00,0x00,0x7F,0x00,0xFE,0x00,0x0F,
  0xF0,0x00,0x00,0x3F,0x80,0x7F,0x80,0x07,0xF0,0x00,0x00,0x3F,0xC0,0x3F,0xC0,0x07,
  0xF8,0x00,0x00,0x1F,0xE0,0x1F,0xE0,0x03,0xF8,0x00,0x00,0x0F,0xF0,0x07,0xF0,0x03,
  0xFC,0x00,0x00,0x07,0xF8,0x03,0xF8,0x01,0xFC,0x00,0x00,0x03,0xFC,0x01,0xFC,0x01,
  0xFE,0x00,0x00,0x01,0xFE,0x01,0xFE,0x00,0xFE,0x00,0x00,0x00,0x7F,0x00,0xFF,0x00,
  0xFF,0x00,0x00,0x00,0x3F,0x80,0x7F,0x80,0x7F,0x00,0x00,0x00,0x1F,0xC0,0x3F,0x80,
  0x7F,0x00,0x00,0x00,0x0F,0xF0,0x1F,0xC0,0x3F,0x80,0x00,0x00,0x07,0xF8,0x1F,0xE0,
  0x3F,0x80,0x00,0x00,0x01,0xFE,0x1F,0xF0,0x3F,0xC0,0x00,0x00,0x00,0xFF,0xFF,0xF0,
  0x1F,0xC0,0x00,0x00,0x00,0x7F,0xFF,0xF8,0x1F,0xE0,0x00,0x00,0x00,0x1F,0xFF,0xF8,
  0x0F,0xE0,0x00,0x00,0x00,0x07,0xFF,0xFC,0x0F,0xF0,0x00,0x00,0x00,0x03,0xFF,0xFC,
  0x07,0xF0,0x00,0x00,0x00,0x00,0xFF,0xFC,0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,0xFC,
  0x03,0xF8,0x00,0x00,0x00,0x00,0x07,0xF8,0x03,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,
  0x01,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xFE,0x00,0x7F,0x00,0x00,0x00,0x00,
  0x00,0xFE,0x00,0xFF,0xE0,0x00,0x00,0x00,0x00,0xFF,0x01,0xFF,0xF8,0x00,0x00,0x00,
  0x00,0x7F,0x01,0xFF,0xFE,0x00,0x00,0x00,0x00,0x7F,0x80,0xFF,0xFF,0x80,0x00,0x00,
  0x00,0x3F,0x80,0xFF,0xFF,0xC0,0x00,0x00,0x00,0x3F,0xC0,0xFF,0xFF,0xF0,0x00,0x00,
  0x00,0x1F,0xC0,0x7F,0xFF,0xF8,0x00,0x00,0x00,0x1F,0xE0,0x3F,0xC3,0xFE,0x00,0x00,
  0x00,0x0F,0xE0,0x3F,0xC0,0xFF,0x00,0x00,0x00,0x0F,0xF0,0x1F,0xE0,0x3F,0x80,0x00,
  0x00,0x07,0xF0,0x0F,0xE0,0x1F,0xC0,0x00,0x00,0x07,0xF8,0x07,0xF0,0x0F,0xF0,0x00,
  0x00,0x03,0xF8,0x03,0xF8,0x07,0xF8,0x00,0x00,0x03,0xF8,0x01,0xFC,0x01,0xFC,0x00,
  0x00,0x01,0xFC,0x01,0xFE,0x00,0xFE,0x00,0x00,0x01,0xFC,0x00,0xFF,0x00,0x7F,0x00,
  0x00,0x00,0xFE,0x00,0x7F,0x80,0x3F,0x80,0x00,0x00,0xFE,0x00,0x3F,0xC0,0x1F,0xC0,
  0x00,0x00,0xFF,0x00,0x0F,0xE0,0x1F,0xE0,0x00,0x00,0x7F,0x00,0x07,0xF0,0x0F,0xF0,
  0x00,0x00,0x7F,0x80,0x03,0xF8,0x07,0xF8,0x00,0x00,0x3F,0x80,0x01,0xFC,0x03,0xF8,
  0x00,0x00,0x3F,0xC0,0x00,0xFF,0x03,0xFC,0x00,0x00,0x1F,0xC0,0x00,0x7F,0x81,0xFE,
  0x00,0x00,0x1F,0xE0,0x00,0x1F,0xF3,0xFF,0x00,0x00,0x0F,0xE0,0x00,0x0F,0xFF,0xFF,
  0x00,0x00,0x0F,0xF0,0x00,0x07,0xFF,0xFF,0x80,0x00,0x07,0xF0,0x00,0x01,0xFF,0xFF,
  0x80,0x00,0x07,0xF8,0x00,0x00,0x7F,0xFF,0xC0,0x00,0x03,0xF8,0x00,0x00,0x1F,0xFF,
  0xC0,0x00,0x03,0xFC,0x00,0x00,0x07,0xFF,0xC0,0x00,0x01,0xFC,0x00,0x00,0x01,0xFF,
  0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,
  0x00,0x0F,0xFF,0x00,0x00,0x00,0x00,0x7F,0xFE,0x00,0x00,0x00,0x07,0xFF,0xFC,0x00,
  0x00,0x00,0x3F,0xFF,0xF8,0x00,0x00,0x00,0xFF,0xFF,0xF0,0x00,0x00,0x07,0xFF,0xFF,
  0xE0,0x00,0x00,0x3F,0xFF,0xFF,0x80,0x00,0x00,0xFF,0xFF,0xFF,0x00,0x00,0x03,0xFF,
  0xFF,0xFC,0x00,0x00,0x1F,0xFF,0xFF,0xF0,0x00,0x00,0x7F,0xF0,0xFF,0xC0,0x00,0x01,
  0xFF,0x01,0xFF,0x80,0x00,0x07,0xFC,0x03,0xFE,0x00,0x00,0x3F,0xE0,0x07,0xF8,0x00,
  0x00,0xFF,0x80,0x1F,0xE0,0x00,0x03,0xFE,0x00,0x7F,0x80,0x00,0x0F,0xF8,0x01,0xFE,
  0x00,0x00,0x3F,0xE0,0x07,0xF8,0x00,0x00,0x7F,0x80,0x3F,0xE0,0x00,0x01,0xFF,0x00,
  0xFF,0x00,0x00,0x07,0xFC,0x07,0xFC,0x00,0x00,0x1F,0xF0,0x3F,0xF0,0x00,0x00,0x7F,
  0xE1,0xFF,0x80,0x00,0x00,0xFF,0x8F,0xFE,0x00,0x00,0x03,0xFF,0x7F,0xF0,0x00,0x00,
  0x0F,0xFF,0xFF,0xC0,0x00,0x00,0x1F,0xFF,0xFE,0x00,0x00,0x00,0x7F,0xFF,0xF8,0x00,
  0x00,0x00,0xFF,0xFF,0xC0,0x00,0x00,0x03,0xFF,0xFE,0x00,0x00,0x00,0x07,0xFF,0xF0,
  0x00,0x00,0x00,0x1F,0xFF,0x80,0x00,0x00,0x00,0xFF,0xFC,0x00,0x00,0x00,0x07,0xFF,
  0xF0,0x00,0x00,0x00,0x3F,0xFF,0xC0,0x00,0x00,0x01,0xFF,0xFF,0x80,0x3F,0xC0,0x07,
  0xFF,0xFF,0x00,0xFF,0x00,0x3F,0xFF,0xFC,0x03,0xFC,0x01,0xFF,0xFF,0xF8,0x0F,0xF0,
  0x0F,0xFF,0xFF,0xE0,0x3F,0xC0,0x3F,0xFD,0xFF,0xC1,0xFF,0x01,0xFF,0xE3,0xFF,0x87,
  0xFC,0x07,0xFF,0x0F,0xFE,0x1F,0xF0,0x1F,0xF8,0x1F,0xFC,0x7F,0xC0,0xFF,0xE0,0x3F,
  0xFB,0xFE,0x03,0xFF,0x00,0xFF,0xFF,0xF8,0x0F,0xFC,0x01,0xFF,0xFF,0xE0,0x3F,0xF0,
  0x03,0xFF,0xFF,0x80,0xFF,0xC0,0x0F,0xFF,0xFE,0x03,0xFF,0x00,0x1F,0xFF,0xF0,0x0F,
  0xFC,0x00,0x3F,0xFF,0xC0,0x3F,0xF0,0x00,0xFF,0xFF,0x00,0xFF,0xE0,0x01,0xFF,0xF8,
  0x03,0xFF,0x80,0x03,0xFF,0xE0,0x07,0xFF,0x00,0x0F,0xFF,0x00,0x1F,0xFE,0x00,0x7F,
  0xFC,0x00,0x7F,0xFC,0x07,0xFF,0xF8,0x00,0xFF,0xFF,0xFF,0xFF,0xE0,0x03,0xFF,0xFF,
  0xFF,0xFF,0xC0,0x0F,0xFF,0xFF,0xFF,0xFF,0x80,0x1F,0xFF,0xFF,0xFF,0xFE,0x00,0x3F,
  0xFF,0xFF,0xFF,0xFC,0x00,0xFF,0xFF,0xFF,0xFF,0xF0,0x01,0xFF,0xFF,0xFC,0xFF,0xE0,
  0x01,0xFF,0xFF,0xE1,0xFF,0xC0,0x03,0xFF,0xFF,0x03,0xFF,0x00,0x03,0xFF,0xF0,0x0F,
  0xFE,0x00,0x01,0xFE,0x00,0x00,0x00,0x07,0xFC,0x00,0xFF,0x80,0x1F,0xF0,0x03,0xFE,
  0x00,0x7F,0xC0,0x0F,0xF8,0x01,0xFF,0x00,0x3F,0xE0,0x07,0xFC,0x00,0xFF,0x80,0x1F,
  0xF0,0x03,0xFE,0x00,0x7F,0xC0,0x0F,0xF8,0x01,0xFF,0x00,0x3F,0xE0,0x03,0xF8,0x00,
  0x7F,0x00,0x0F,0xE0,0x01,0xFC,0x00,0x3F,0x80,0x07,0xF0,0x00,0xFE,0x00,0x1F,0xC0,
  0x03,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,0x0F,0xFC,0x00,0x1F,0xF8,
  0x00,0x1F,0xF8,0x00,0x1F,0xF8,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,
  0x7F,0xF0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0xFF,0xE0,0x00,0xFF,0xE0,0x00,0xFF,
  0xC0,0x00,0xFF,0xC0,0x01,0xFF,0xC0,0x01,0xFF,0xC0,0x01,0xFF,0xC0,0x01,0xFF,0x80,
  0x01,0xFF,0x80,0x03,0xFF,0x80,0x03,0xFF,0x80,0x03,0xFF,0x00,0x03,0xFF,0x00,0x03,
  0xFF,0x00,0x03,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFE,
  0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,
  0x07,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,
  0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,
  0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,
  0x0F,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,
  0xFE,0x00,0x07,0xFE,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x03,0xFF,
  0x00,0x03,0xFF,0x00,0x03,0xFF,0x00,0x03,0xFF,0x80,0x03,0xFF,0x80,0x01,0xFF,0x80,
  0x01,0xFF,0x80,0x01,0xFF,0xC0,0x01,0xFF,0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x00,
  0xFF,0xE0,0x00,0xFF,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xF0,0x00,0x3F,
  0xF0,0x00,0x3F,0xF0,0x00,0x3F,0xF8,0x00,0x1F,0xF8,0x00,0x1F,0xF8,0x00,0x0F,0xFC,
  0x00,0x0F,0xFC,0x3F,0xE0,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,0x3F,0xF8,0x00,0x1F,
  0xF8,0x00,0x1F,0xF8,0x00,0x1F,0xFC,0x00,0x0F,0xFC,0x00,0x0F,0xFC,0x00,0x0F,0xFE,
  0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFE,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,
  0x03,0xFF,0x00,0x03,0xFF,0x00,0x03,0xFF,0x80,0x03,0xFF,0x80,0x01,0xFF,0x80,0x01,
  0xFF,0x80,0x01,0xFF,0x80,0x01,0xFF,0xC0,0x01,0xFF,0xC0,0x01,0xFF,0xC0,0x00,0xFF,
  0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xE0,0x00,0xFF,0xE0,
  0x00,0xFF,0xE0,0x00,0xFF,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,
  0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,
  0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,
  0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,
  0x7F,0xE0,0x00,0x7F,0xE0,0x00,0x7F,0xE0,0x00,0xFF,0xE0,0x00,0xFF,0xE0,0x00,0xFF,
  0xE0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x00,0xFF,0xC0,0x01,0xFF,0xC0,
  0x01,0xFF,0xC0,0x01,0xFF,0x80,0x01,0xFF,0x80,0x03,0xFF,0x80,0x03,0xFF,0x80,0x03,
  0xFF,0x80,0x03,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFF,0x00,0x07,0xFE,
  0x00,0x0F,0xFE,0x00,0x0F,0xFE,0x00,0x0F,0xFC,0x00,0x0F,0xFC,0x00,0x1F,0xFC,0x00,
  0x1F,0xF8,0x00,0x1F,0xF8,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,0x00,
  0x01,0xFC,0x00,0x00,0x00,0x0F,0xE0,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x01,0xF8,
  0x00,0x00,0x00,0x0F,0xC0,0x00,0x00,0x00,0x7E,0x00,0x00,0x02,0x03,0xE0,0x30,0x00,
  0x38,0x1F,0x03,0x80,0x01,0xE0,0xF8,0x3C,0x00,0x1F,0x87,0xC3,0xF0,0x00,0xFE,0x3E,
  0x3F,0x80,0x0F,0xF9,0xF3,0xFE,0x00,0x7F,0xFF,0xFF,0xF0,0x03,0xFF,0xFF,0xFF,0x80,
  0x0F,0xFF,0xFF,0xF8,0x00,0x1F,0xFF,0xFF,0x00,0x00,0x3F,0xFF,0xF0,0x00,0x00,0xFF,
  0xFE,0x00,0x00,0x01,0xFF,0xE0,0x00,0x00,0x0F,0xFF,0x00,0x00,0x01,0xFF,0xFC,0x00,
  0x00,0x1F,0xFF,0xF8,0x00,0x03,0xFF,0xFF,0xE0,0x00,0x7F,0xFF,0xFF,0xC0,0x07,0xFF,
  0xFF,0xFF,0x00,0x3F,0xFF,0xFF,0xF8,0x01,0xFF,0x3E,0x7F,0xC0,0x07,0xF1,0xF1,0xFC,
  0x00,0x3F,0x0F,0x87,0xE0,0x00,0xF0,0x7C,0x1E,0x00,0x07,0x03,0xE0,0x70,0x00,0x10,
  0x1F,0x00,0x80,0x00,0x00,0xFC,0x00,0x00,0x00,0x07,0xE0,0x00,0x00,0x00,0x3F,0x00,
  0x00,0x00,0x03,0xF8,0x00,0x00,0x00,0x1F,0xC0,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,
  0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,0xE0,
  0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,
  0xF8,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,
  0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,
  0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,
  0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x7F,0xFF,0xFF,
  0xFF,0xF8,0x01,0xFF,0xFF,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,0xFF,0xFF,0x80,0x1F,0xFF,
  0xFF,0xFF,0xFE,0x00,0x7F,0xFF,0xFF,0xFF,0xF8,0x01,0xFF,0xFF,0xFF,0xFF,0xE0,0x07,
  0xFF,0xFF,0xFF,0xFF,0x80,0x1F,0xFF,0xFF,0xFF,0xFE,0x00,0x7F,0xFF,0xFF,0xFF,0xF8,
  0x01,0xFF,0xFF,0xFF,0xFF,0xE0,0x03,0xFF,0xFF,0xFF,0xFF,0x80,0x00,0x00,0x7F,0x80,
  0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,
  0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,
  0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,
  0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,
  0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x07,0xF8,
  0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,
  0xF8,0x07,0xFC,0x03,0xFE,0x01,0xFF,0x00,0xFF,0x80,0x7F,0x80,0x7F,0xC0,0x3F,0xE0,
  0x1F,0xF0,0x0F,0xF8,0x07,0xF8,0x03,0xFC,0x01,0xFE,0x00,0xFF,0x00,0x7F,0x00,0x7F,
  0x80,0x3F,0xC0,0x1F,0xE0,0x0F,0xF0,0x07,0xF8,0x03,0xF8,0x01,0xFC,0x00,0xFE,0x00,
  0x7F,0x00,0x3F,0x00,0x00,0x0F,0xFF,0xFF,0xE0,0x1F,0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,
  0x80,0x7F,0xFF,0xFF,0x00,0xFF,0xFF,0xFE,0x01,0xFF,0xFF,0xFC,0x03,0xFF,0xFF,0xF8,
  0x07,0xFF,0xFF,0xF0,0x0F,0xFF,0xFF,0xE0,0x1F,0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,0x80,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x01,0xF0,0x01,0xFC,0x01,0xFF,0x00,0xFF,0x80,0xFF,0xC0,0x7F,0xF0,
  0x3F,0xF8,0x1F,0xFC,0x0F,0xFE,0x07,0xFF,0x03,0xFF,0x00,0xFF,0x80,0x7F,0x80,0x1F,
  0xC0,0x07,0x80,0x00,0x00,0x00,0x00,0x00,0x3F,0xE0,0x00,0x0F,0xF8,0x00,0x03,0xFE,
  0x00,0x00,0xFF,0x80,0x00,0x3F,0xC0,0x00,0x1F,0xF0,0x00,0x07,0xFC,0x00,0x01,0xFF,
  0x00,0x00,0x7F,0xC0,0x00,0x1F,0xE0,0x00,0x0F,0xF8,0x00,0x03,0xFE,0x00,0x00,0xFF,
  0x80,0x00,0x3F,0xE0,0x00,0x0F,0xF0,0x00,0x07,0xFC,0x00,0x01,0xFF,0x00,0x00,0x7F,
  0xC0,0x00,0x1F,0xF0,0x00,0x07,0xF8,0x00,0x03,0xFE,0x00,0x00,0xFF,0x80,0x00,0x3F,
  0xE0,0x00,0x0F,0xF8,0x00,0x03,0xFC,0x00,0x01,0xFF,0x00,0x00,0x7F,0xC0,0x00,0x1F,
  0xF0,0x00,0x07,0xFC,0x00,0x01,0xFE,0x00,0x00,0xFF,0x80,0x00,0x3F,0xE0,0x00,0x0F,
  0xF8,0x00,0x03,0xFC,0x00,0x01,0xFF,0x00,0x00,0x7F,0xC0,0x00,0x1F,0xF0,0x00,0x07,
  0xFC,0x00,0x01,0xFF,0x00,0x00,0xFF,0x80,0x00,0x3F,0xE0,0x00,0x0F,0xF8,0x00,0x03,
  0xFE,0x00,0x00,0xFF,0x00,0x00,0x7F,0xC0,0x00,0x1F,0xF0,0x00,0x07,0xFC,0x00,0x01,
  0xFF,0x00,0x00,0x7F,0x80,0x00,0x3F,0xE0,0x00,0x0F,0xF8,0x00,0x03,0xFE,0x00,0x00,
  0xFF,0x80,0x00,0x3F,0xC0,0x00,0x1F,0xF0,0x00,0x07,0xFC,0x00,0x01,0xFF,0x00,0x00,
  0x7F,0xC0,0x00,0x1F,0xE0,0x00,0x0F,0xF8,0x00,0x03,0xFE,0x00,0x00,0xFF,0x80,0x00,
  0x3F,0xE0,0x00,0x0F,0xF0,0x00,0x07,0xFC,0x00,0x01,0xFF,0x00,0x00,0x7F,0xC0,0x00,
  0x1F,0xF0,0x00,0x0F,0xF8,0x00,0x03,0xFE,0x00,0x00,0xFF,0x80,0x00,0x3F,0xE0,0x00,
  0x0F,0xF8,0x00,0x07,0xFC,0x00,0x01,0xFF,0x00,0x00,0x7F,0xC0,0x00,0x1F,0xF0,0x00,
  0x07,0xF8,0x00,0x03,0xFE,0x00,0x00,0xFF,0x80,0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,
  0x00,0x00,0x03,0xFF,0xF0,0x00,0x00,0x00,0x3F,0xFF,0xF0,0x00,0x00,0x01,0xFF,0xFF,
  0xE0,0x00,0x00,0x0F,0xFF,0xFF,0xC0,0x00,0x00,0x7F,0xFF,0xFF,0x80,0x00,0x03,0xFF,
  0xFF,0xFF,0x00,0x00,0x1F,0xFF,0xFF,0xFE,0x00,0x00,0xFF,0xFF,0xFF,0xFC,0x00,0x03,
  0xFF,0xFF,0xFF,0xF0,0x00,0x1F,0xFF,0xFF,0xFF,0xE0,0x00,0x7F,0xFE,0x1F,0xFF,0x80,
  0x03,0xFF,0xE0,0x1F,0xFF,0x00,0x0F,0xFF,0x00,0x3F,0xFC,0x00,0x7F,0xF8,0x00,0x7F,
  0xF8,0x01,0xFF,0xC0,0x00,0xFF,0xE0,0x07,0xFE,0x00,0x03,0xFF,0x80,0x3F,0xF8,0x00,
  0x07,0xFE,0x00,0xFF,0xE0,0x00,0x1F,0xFC,0x03,0xFF,0x00,0x00,0x3F,0xF0,0x0F,0xFC,
  0x00,0x00,0xFF,0xC0,0x7F,0xF0,0x00,0x03,0xFF,0x01,0xFF,0x80,0x00,0x0F,0xFE,0x07,
  0xFE,0x00,0x00,0x1F,0xF8,0x1F,0xF8,0x00,0x00,0x7F,0xE0,0x7F,0xE0,0x00,0x01,0xFF,
  0x81,0xFF,0x80,0x00,0x07,0xFE,0x0F,0xFE,0x00,0x00,0x1F,0xF8,0x3F,0xF0,0x00,0x00,
  0x7F,0xE0,0xFF,0xC0,0x00,0x00,0xFF,0x83,0xFF,0x00,0x00,0x03,0xFF,0x0F,0xFC,0x00,
  0x00,0x0F,0xFC,0x3F,0xF0,0x00,0x00,0x3F,0xF0,0xFF,0xC0,0x00,0x00,0xFF,0xC3,0xFF,
  0x00,0x00,0x03,0xFF,0x0F,0xFC,0x00,0x00,0x0F,0xFC,0x3F,0xF0,0x00,0x00,0x3F,0xF0,
  0xFF,0xC0,0x00,0x00,0xFF,0xC3,0xFF,0x00,0x00,0x03,0xFF,0x0F,0xFC,0x00,0x00,0x0F,
  0xF8,0x3F,0xF0,0x00,0x00,0x7F,0xE0,0xFF,0xE0,0x00,0x01,0xFF,0x81,0xFF,0x80,0x00,
  0x07,0xFE,0x07,0xFE,0x00,0x00,0x1F,0xF8,0x1F,0xF8,0x00,0x00,0x7F,0xE0,0x7F,0xE0,
  0x00,0x01,0xFF,0x81,0xFF,0x80,0x00,0x0F,0xFE,0x07,0xFF,0x00,0x00,0x3F,0xF0,0x0F,
  0xFC,0x00,0x00,0xFF,0xC0,0x3F,0xF0,0x00,0x03,0xFF,0x00,0xFF,0xE0,0x00,0x1F,0xFC,
  0x03,0xFF,0x80,0x00,0x7F,0xE0,0x07,0xFE,0x00,0x03,0xFF,0x80,0x1F,0xFC,0x00,0x0F,
  0xFE,0x00,0x7F,0xF8,0x00,0x7F,0xF8,0x00,0xFF,0xF0,0x03,0xFF,0xC0,0x03,0xFF,0xE0,
  0x1F,0xFF,0x00,0x07,0xFF,0xE1,0xFF,0xF8,0x00,0x1F,0xFF,0xFF,0xFF,0xE0,0x00,0x3F,
  0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFC,0x00,0x01,0xFF,0xFF,0xFF,0xE0,0x00,
  0x03,0xFF,0xFF,0xFF,0x00,0x00,0x07,0xFF,0xFF,0xF8,0x00,0x00,0x0F,0xFF,0xFF,0xC0,
  0x00,0x00,0x1F,0xFF,0xFE,0x00,0x00,0x00,0x3F,0xFF,0xF0,0x00,0x00,0x00,0x3F,0xFF,
  0x00,0x00,0x00,0x00,0x1F,0xE0,0x00,0x00,0x00,0x07,0xFE,0x00,0x01,0xFF,0xC0,0x00,
  0x7F,0xF8,0x00,0x1F,0xFF,0x00,0x0F,0xFF,0xE0,0x03,0xFF,0xFC,0x00,0xFF,0xFF,0x80,
  0x3F,0xFF,0xF0,0x0F,0xFF,0xFE,0x03,0xFF,0xFF,0xC0,0xFF,0xFF,0xF8,0x3F,0xFF,0xFF,
  0x07,0xFF,0xFF,0xE0,0xFF,0xEF,0xFC,0x1F,0xF9,0xFF,0x83,0xFE,0x3F,0xF0,0x7F,0x87,
  0xFE,0x0F,0xC0,0xFF,0xC1,0xF0,0x1F,0xF8,0x3C,0x03,0xFF,0x07,0x00,0x7F,0xE0,0xC0,
  0x0F,0xFC,0x00,0x01,0xFF,0x80,0x00,0x3F,0xF0,0x00,0x07,0xFE,0x00,0x00,0xFF,0xC0,
  0x00,0x1F,0xF8,0x00,0x03,0xFF,0x00,0x00,0x7F,0xE0,0x00,0x0F,0xFC,0x00,0x01,0xFF,
  0x80,0x00,0x3F,0xF0,0x00,0x07,0xFE,0x00,0x00,0xFF,0xC0,0x00,0x1F,0xF8,0x00,0x03,
  0xFF,0x00,0x00,0x7F,0xE0,0x00,0x0F,0xFC,0x00,0x01,0xFF,0x80,0x00,0x3F,0xF0,0x00,
  0x07,0xFE,0x00,0x00,0xFF,0xC0,0x00,0x1F,0xF8,0x00,0x03,0xFF,0x00,0x00,0x7F,0xE0,
  0x00,0x0F,0xFC,0x00,0x01,0xFF,0x80,0x00,0x3F,0xF0,0x00,0x07,0xFE,0x00,0x00,0xFF,
  0xC0,0x00,0x1F,0xF8,0x00,0x03,0xFF,0x00,0x00,0x7F,0xE0,0x00,0x0F,0xFC,0x00,0x01,
  0xFF,0x80,0x00,0x3F,0xF0,0x00,0x07,0xFE,0x00,0x00,0xFF,0xC0,0x00,0x1F,0xF8,0x00,
  0x03,0xFF,0x00,0x00,0x7F,0xE0,0x00,0x0F,0xFC,0x00,0x01,0xFF,0x80,0x00,0x3F,0xF0,
  0x00,0x07,0xFE,0x00,0x00,0xFF,0xC0,0x00,0x1F,0xF8,0x00,0x00,0x00,0x7F,0x80,0x00,
  0x00,0x00,0xFF,0xFC,0x00,0x00,0x00,0xFF,0xFF,0xC0,0x00,0x00,0x7F,0xFF,0xFC,0x00,
  0x00,0x3F,0xFF,0xFF,0x80,0x00,0x1F,0xFF,0xFF,0xF0,0x00,0x0F,0xFF,0xFF,0xFE,0x00,
  0x07,0xFF,0xFF,0xFF,0x80,0x03,0xFF,0xFF,0xFF,0xF0,0x00,0xFF,0xFF,0xFF,0xFE,0x00,
  0x7F,0xFF,0xFF,0xFF,0x80,0x1F,0xFF,0x83,0xFF,0xE0,0x07,0xFF,0x80,0x3F,0xFC,0x03,
  0xFF,0x80,0x07,0xFF,0x00,0xFF,0xE0,0x01,0xFF,0xC0,0x3F,0xF0,0x00,0x3F,0xF8,0x1F,
  0xFC,0x00,0x0F,0xFE,0x07,0xFE,0x00,0x01,0xFF,0x81,0xFF,0x80,0x00,0x7F,0xE0,0x7F,
  0xE0,0x00,0x1F,0xF8,0x1F,0xF0,0x00,0x07,0xFE,0x07,0xFC,0x00,0x01,0xFF,0x81,0xFF,
  0x00,0x00,0x7F,0xE0,0x7F,0xC0,0x00,0x1F,0xF8,0x00,0x00,0x00,0x07,0xFE,0x00,0x00,
  0x00,0x01,0xFF,0x80,0x00,0x00,0x00,0xFF,0xE0,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,
  0x00,0x0F,0xFC,0x00,0x00,0x00,0x07,0xFF,0x00,0x00,0x00,0x01,0xFF,0x80,0x00,0x00,
  0x00,0xFF,0xE0,0x00,0x00,0x00,0x7F,0xF8,0x00,0x00,0x00,0x3F,0xFC,0x00,0x00,0x00,
  0x0F,0xFE,0x00,0x00,0x00,0x07,0xFF,0x80,0x00,0x00,0x03,0xFF,0xC0,0x00,0x00,0x01,
  0xFF,0xE0,0x00,0x00,0x00,0xFF,0xF8,0x00,0x00,0x00,0x7F,0xFC,0x00,0x00,0x00,0x3F,
  0xFE,0x00,0x00,0x00,0x1F,0xFF,0x00,0x00,0x00,0x0F,0xFF,0xC0,0x00,0x00,0x03,0xFF,
  0xE0,0x00,0x00,0x01,0xFF,0xF0,0x00,0x00,0x00,0xFF,0xF8,0x00,0x00,0x00,0x7F,0xFC,
  0x00,0x00,0x00,0x3F,0xFE,0x00,0x00,0x00,0x1F,0xFF,0x00,0x00,0x00,0x0F,0xFF,0x80,
  0x00,0x00,0x07,0xFF,0xE0,0x00,0x00,0x03,0xFF,0xF0,0x00,0x00,0x00,0xFF,0xF8,0x00,
  0x00,0x00,0x7F,0xFC,0x00,0x00,0x00,0x3F,0xFE,0x00,0x00,0x00,0x1F,0xFF,0x00,0x00,
  0x00,0x0F,0xFF,0x80,0x00,0x00,0x07,0xFF,0xFF,0xFF,0xFF,0x81,0xFF,0xFF,0xFF,0xFF,
  0xE0,0x7F,0xFF,0xFF,0xFF,0xF8,0x1F,0xFF,0xFF,0xFF,0xFE,0x07,0xFF,0xFF,0xFF,0xFF,
  0x81,0xFF,0xFF,0xFF,0xFF,0xE0,0x7F,0xFF,0xFF,0xFF,0xF8,0x1F,0xFF,0xFF,0xFF,0xFE,
  0x07,0xFF,0xFF,0xFF,0xFF,0x81,0xFF,0xFF,0xFF,0xFF,0xE0,0x7F,0xFF,0xFF,0xFF,0xF8,
  0x00,0x00,0x7F,0xC0,0x00,0x00,0x00,0x7F,0xFF,0x00,0x00,0x00,0x1F,0xFF,0xF8,0x00,
  0x00,0x0F,0xFF,0xFF,0x80,0x00,0x03,0xFF,0xFF,0xF8,0x00,0x00,0xFF,0xFF,0xFF,0x80,
  0x00,0x3F,0xFF,0xFF,0xF8,0x00,0x0F,0xFF,0xFF,0xFF,0x80,0x01,0xFF,0xFF,0xFF,0xF8,
  0x00,0x7F,0xFF,0xFF,0xFF,0x00,0x0F,0xFF,0xFF,0xFF,0xF0,0x03,0xFF,0xE0,0x3F,0xFE,
  0x00,0x7F,0xF0,0x01,0xFF,0xC0,0x0F,0xFE,0x00,0x1F,0xFC,0x03,0xFF,0x80,0x03,0xFF,
  0x80,0x7F,0xE0,0x00,0x3F,0xF0,0x0F,0xFC,0x00,0x07,0xFE,0x01,0xFF,0x80,0x00,0xFF,
  0xC0,0x3F,0xF0,0x00,0x1F,0xF8,0x07,0xFC,0x00,0x01,0xFF,0x01,0xFF,0x80,0x00,0x3F,
  0xE0,0x00,0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x01,0xFF,0x80,0x00,0x00,0x00,0x3F,
  0xF0,0x00,0x00,0x00,0x07,0xFC,0x00,0x00,0x00,0x01,0xFF,0x80,0x00,0x00,0x00,0x7F,
  0xF0,0x00,0x00,0x00,0x1F,0xFC,0x00,0x00,0x00,0x0F,0xFF,0x00,0x00,0x01,0xFF,0xFF,
  0xE0,0x00,0x00,0x3F,0xFF,0xF8,0x00,0x00,0x07,0xFF,0xFC,0x00,0x00,0x00,0xFF,0xFF,
  0x00,0x00,0x00,0x1F,0xFF,0x80,0x00,0x00,0x03,0xFF,0xFE,0x00,0x00,0x00,0x7F,0xFF,
  0xF0,0x00,0x00,0x0F,0xFF,0xFF,0x00,0x00,0x01,0xFF,0xFF,0xF0,0x00,0x00,0x3F,0xFF,
  0xFF,0x00,0x00,0x00,0x07,0xFF,0xE0,0x00,0x00,0x00,0x3F,0xFE,0x00,0x00,0x00,0x01,
  0xFF,0xC0,0x00,0x00,0x00,0x1F,0xFC,0x00,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x00,
  0x3F,0xF0,0x00,0x00,0x00,0x07,0xFF,0x0F,0xFC,0x00,0x00,0xFF,0xE1,0xFF,0x80,0x00,
  0x0F,0xFC,0x1F,0xF8,0x00,0x01,0xFF,0x83,0xFF,0x00,0x00,0x7F,0xF0,0x7F,0xE0,0x00,
  0x0F,0xFE,0x0F,0xFC,0x00,0x01,0xFF,0xC1,0xFF,0xC0,0x00,0x3F,0xF8,0x3F,0xF8,0x00,
  0x0F,0xFE,0x03,0xFF,0x00,0x01,0xFF,0xC0,0x7F,0xF0,0x00,0x7F,0xF8,0x0F,0xFF,0x00,
  0x1F,0xFF,0x00,0xFF,0xF8,0x0F,0xFF,0xC0,0x1F,0xFF,0xFF,0xFF,0xF8,0x01,0xFF,0xFF,
  0xFF,0xFE,0x00,0x3F,0xFF,0xFF,0xFF,0xC0,0x03,0xFF,0xFF,0xFF,0xF0,0x00,0x3F,0xFF,
  0xFF,0xFC,0x00,0x03,0xFF,0xFF,0xFF,0x00,0x00,0x7F,0xFF,0xFF,0xC0,0x00,0x03,0xFF,
  0xFF,0xF0,0x00,0x00,0x3F,0xFF,0xF8,0x00,0x00,0x01,0xFF,0xFC,0x00,0x00,0x00,0x07,
  0xFC,0x00,0x00,0x00,0x00,0x00,0xFF,0xFC,0x00,0x00,0x00,0x03,0xFF,0xF0,0x00,0x00,
  0x00,0x1F,0xFF,0xC0,0x00,0x00,0x00,0x7F,0xFF,0x00,0x00,0x00,0x03,0xFF,0xFC,0x00,
  0x00,0x00,0x0F,0xFF,0xF0,0x00,0x00,0x00,0x7F,0xFF,0xC0,0x00,0x00,0x01,0xFF,0xFF,
  0x00,0x00,0x00,0x0F,0xFF,0xFC,0x00,0x00,0x00,0x7F,0xFF,0xF0,0x00,0x00,0x01,0xFF,
  0xFF,0xC0,0x00,0x00,0x0F,0xFF,0xFF,0x00,0x00,0x00,0x3F,0xFF,0xFC,0x00,0x00,0x01,
  0xFF,0xFF,0xF0,0x00,0x00,0x07,0xFE,0xFF,0xC0,0x00,0x00,0x3F,0xF3,0xFF,0x00,0x00,
  0x00,0xFF,0xCF,0xFC,0x00,0x00,0x07,0xFE,0x3F,0xF0,0x00,0x00,0x1F,0xF8,0xFF,0xC0,
  0x00,0x00,0xFF,0xC3,0xFF,0x00,0x00,0x03,0xFF,0x0F,0xFC,0x00,0x00,0x1F,0xF8,0x3F,
  0xF0,0x00,0x00,0xFF,0xE0,0xFF,0xC0,0x00,0x03,0xFF,0x03,0xFF,0x00,0x00,0x1F,0xFC,
  0x0F,0xFC,0x00,0x00,0x7F,0xE0,0x3F,0xF0,0x00,0x03,0xFF,0x80,0xFF,0xC0,0x00,0x0F,
  0xFC,0x03,0xFF,0x00,0x00,0x7F,0xE0,0x0F,0xFC,0x00,0x01,0xFF,0x80,0x3F,0xF0,0x00,
  0x0F,0xFC,0x00,0xFF,0xC0,0x00,0x3F,0xF0,0x03,0xFF,0x00,0x01,0xFF,0x80,0x0F,0xFC,
  0x00,0x07,0xFE,0x00,0x3F,0xF0,0x00,0x3F,0xF0,0x00,0xFF,0xC0,0x01,0xFF,0xC0,0x03,
  0xFF,0x00,0x07,0xFE,0x00,0x0F,0xFC,0x00,0x3F,0xF8,0x00,0x3F,0xF0,0x00,0xFF,0xC0,
  0x00,0xFF,0xC0,0x07,0xFF,0x00,0x03,0xFF,0x00,0x1F,0xF8,0x00,0x0F,0xFC,0x00,0xFF,
  0xC0,0x00,0x3F,0xF0,0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0xFF,0xFF,0xFF,0xFF,0xFC,
  0x3F,0xFF,0xFF,0xFF,0xFF,0xF0,0xFF,0xFF,0xFF,0xFF,0xFF,0xC3,0xFF,0xFF,0xFF,0xFF,
  0xFF,0x0F,0xFF,0xFF,0xFF,0xFF,0xFC,0x3F,0xFF,0xFF,0xFF,0xFF,0xF0,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xC3,0xFF,0xFF,0xFF,0xFF,0xFF,0x0F,0xFF,0xFF,0xFF,0xFF,0xFC,0x3F,0xFF,
  0xFF,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0x00,0xFF,0xC0,0x00,
  0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x00,0x3F,0xF0,
  0x00,0x00,0x00,0x00,0xFF,0xC0,0x00,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x00,0x0F,
  0xFC,0x00,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0x00,0xFF,0xC0,0x00,0x00,0x00,
  0x03,0xFF,0x00,0x00,0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,
  0x00,0x00,0xFF,0xC0,0x00,0x0F,0xFF,0xFF,0xFF,0xF8,0x03,0xFF,0xFF,0xFF,0xFE,0x00,
  0xFF,0xFF,0xFF,0xFF,0x80,0x3F,0xFF,0xFF,0xFF,0xE0,0x0F,0xFF,0xFF,0xFF,0xF8,0x03,
  0xFF,0xFF,0xFF,0xFE,0x00,0xFF,0xFF,0xFF,0xFF,0x80,0x3F,0xFF,0xFF,0xFF,0xE0,0x0F,
  0xFF,0xFF,0xFF,0xF8,0x03,0xFF,0xFF,0xFF,0xFE,0x00,0xFF,0xFF,0xFF,0xFF,0x80,0x3F,
  0xF0,0x00,0x00,0x00,0x0F,0xF8,0x00,0x00,0x00,0x03,0xFE,0x00,0x00,0x00,0x00,0xFF,
  0x80,0x00,0x00,0x00,0x3F,0xE0,0x00,0x00,0x00,0x0F,0xF8,0x00,0x00,0x00,0x07,0xFE,
  0x00,0x00,0x00,0x01,0xFF,0x80,0x00,0x00,0x00,0x7F,0xE0,0x00,0x00,0x00,0x1F,0xF8,
  0x00,0x00,0x00,0x07,0xFE,0x00,0x00,0x00,0x01,0xFF,0x83,0xFF,0x00,0x00,0x7F,0xE3,
  0xFF,0xF0,0x00,0x1F,0xF9,0xFF,0xFE,0x00,0x07,0xFE,0xFF,0xFF,0xE0,0x01,0xFF,0xFF,
  0xFF,0xF8,0x00,0x7F,0xFF,0xFF,0xFF,0x00,0x1F,0xFF,0xFF,0xFF,0xE0,0x07,0xFF,0xFF,
  0xFF,0xFC,0x01,0xFF,0xFF,0xFF,0xFF,0x00,0x7F,0xFF,0xFF,0xFF,0xE0,0x1F,0xFF,0xC0,
  0xFF,0xF8,0x07,0xFF,0xC0,0x1F,0xFF,0x01,0xFF,0xE0,0x01,0xFF,0xC0,0x7F,0xF0,0x00,
  0x7F,0xF8,0x1F,0xF8,0x00,0x0F,0xFE,0x07,0xFE,0x00,0x01,0xFF,0x81,0xFF,0x00,0x00,
  0x7F,0xE0,0x00,0x00,0x00,0x1F,0xFC,0x00,0x00,0x00,0x07,0xFF,0x00,0x00,0x00,0x00,
  0xFF,0xC0,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x03,
  0xFF,0x00,0x00,0x00,0x00,0xFF,0xC3,0xFF,0x00,0x00,0x3F,0xF0,0xFF,0xC0,0x00,0x0F,
  0xFC,0x3F,0xF0,0x00,0x03,0xFF,0x0F,0xFC,0x00,0x01,0xFF,0xC3,0xFF,0x80,0x00,0x7F,
  0xE0,0x7F,0xE0,0x00,0x1F,0xF8,0x1F,0xFC,0x00,0x0F,0xFE,0x07,0xFF,0x00,0x03,0xFF,
  0x81,0xFF,0xE0,0x01,0xFF,0xC0,0x3F,0xFC,0x00,0xFF,0xF0,0x0F,0xFF,0xC0,0xFF,0xF8,
  0x01,0xFF,0xFF,0xFF,0xFE,0x00,0x7F,0xFF,0xFF,0xFF,0x80,0x0F,0xFF,0xFF,0xFF,0xC0,
  0x03,0xFF,0xFF,0xFF,0xE0,0x00,0x7F,0xFF,0xFF,0xF8,0x00,0x0F,0xFF,0xFF,0xFC,0x00,
  0x01,0xFF,0xFF,0xFE,0x00,0x00,0x3F,0xFF,0xFE,0x00,0x00,0x03,0xFF,0xFF,0x00,0x00,
  0x00,0x3F,0xFF,0x00,0x00,0x00,0x01,0xFE,0x00,0x00,0x00,0x00,0x3F,0xC0,0x00,0x00,
  0x00,0x1F,0xFF,0x00,0x00,0x00,0x0F,0xFF,0xF8,0x00,0x00,0x07,0xFF,0xFF,0xC0,0x00,
  0x01,0xFF,0xFF,0xFC,0x00,0x00,0x7F,0xFF,0xFF,0xC0,0x00,0x0F,0xFF,0xFF,0xF8,0x00,
  0x03,0xFF,0xFF,0xFF,0x80,0x00,0xFF,0xFF,0xFF,0xF8,0x00,0x1F,0xFF,0xFF,0xFF,0x00,
  0x07,0xFF,0xFF,0xFF,0xF0,0x00,0xFF,0xF0,0x1F,0xFE,0x00,0x3F,0xF8,0x01,0xFF,0xE0,
  0x07,0xFE,0x00,0x1F,0xFC,0x01,0xFF,0x80,0x01,0xFF,0x80,0x3F,0xF0,0x00,0x3F,0xF8,
  0x07,0xFC,0x00,0x03,0xFF,0x01,0xFF,0x80,0x00,0x7F,0xE0,0x3F,0xF0,0x00,0x0F,0xFC,
  0x07,0xFC,0x00,0x00,0x00,0x00,0xFF,0x80,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,
  0x07,0xFC,0x00,0x00,0x00,0x00,0xFF,0x80,0x00,0x00,0x00,0x1F,0xF0,0x1F,0xF0,0x00,
  0x03,0xFE,0x0F,0xFF,0xC0,0x00,0x7F,0xC3,0xFF,0xFC,0x00,0x0F,0xF8,0xFF,0xFF,0xE0,
  0x03,0xFF,0x3F,0xFF,0xFE,0x00,0x7F,0xEF,0xFF,0xFF,0xC0,0x0F,0xFF,0xFF,0xFF,0xFC,
  0x01,0xFF,0xFF,0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,0xFF,0xF8,0x07,0xFF,0xFF,0xFF,0xFF,
  0x80,0xFF,0xFF,0xC1,0xFF,0xF0,0x1F,0xFF,0xE0,0x1F,0xFF,0x03,0xFF,0xF8,0x01,0xFF,
  0xE0,0x7F,0xFE,0x00,0x1F,0xFC,0x0F,0xFF,0x80,0x01,0xFF,0xC1,0xFF,0xF0,0x00,0x3F,
  0xF8,0x3F,0xFC,0x00,0x03,0xFF,0x07,0xFF,0x80,0x00,0x7F,0xE0,0xFF,0xF0,0x00,0x0F,
  0xFC,0x1F,0xFC,0x00,0x00,0xFF,0xC1,0xFF,0x80,0x00,0x1F,0xF8,0x3F,0xF0,0x00,0x03,
  0xFF,0x07,0xFE,0x00,0x00,0x7F,0xE0,0xFF,0xC0,0x00,0x0F,0xFC,0x1F,0xF8,0x00,0x01,
  0xFF,0x83,0xFF,0x80,0x00,0x7F,0xE0,0x3F,0xF0,0x00,0x0F,0xFC,0x07,0xFE,0x00,0x01,
  0xFF,0x80,0xFF,0xC0,0x00,0x7F,0xF0,0x1F,0xFC,0x00,0x0F,0xFE,0x01,0xFF,0xC0,0x03,
  0xFF,0x80,0x3F,0xF8,0x00,0xFF,0xF0,0x07,0xFF,0xC0,0x3F,0xFE,0x00,0x7F,0xFC,0x0F,
  0xFF,0x80,0x0F,0xFF,0xFF,0xFF,0xF0,0x00,0xFF,0xFF,0xFF,0xFE,0x00,0x1F,0xFF,0xFF,
  0xFF,0x80,0x01,0xFF,0xFF,0xFF,0xE0,0x00,0x1F,0xFF,0xFF,0xFC,0x00,0x01,0xFF,0xFF,
  0xFF,0x00,0x00,0x1F,0xFF,0xFF,0xC0,0x00,0x01,0xFF,0xFF,0xE0,0x00,0x00,0x1F,0xFF,
  0xF8,0x00,0x00,0x00,0xFF,0xFC,0x00,0x00,0x00,0x03,0xFC,0x00,0x00,0x3F,0xFF,0xFF,
  0xFF,0xF8,0x7F,0xFF,0xFF,0xFF,0xF0,0xFF,0xFF,0xFF,0xFF,0xE1,0xFF,0xFF,0xFF,0xFF,
  0xC3,0xFF,0xFF,0xFF,0xFF,0x87,0xFF,0xFF,0xFF,0xFF,0x0F,0xFF,0xFF,0xFF,0xFE,0x1F,
  0xFF,0xFF,0xFF,0xFC,0x3F,0xFF,0xFF,0xFF,0xF8,0x7F,0xFF,0xFF,0xFF,0xF0,0xFF,0xFF,
  0xFF,0xFF,0xE1,0x00,0x00,0x01,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x07,
  0xFE,0x00,0x00,0x00,0x1F,0xFC,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x7F,0xE0,
  0x00,0x00,0x01,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x0F,0xFE,0x00,0x00,
  0x00,0x1F,0xFC,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0xFF,0xE0,0x00,0x00,0x01,
  0xFF,0x80,0x00,0x00,0x07,0xFF,0x00,0x00,0x00,0x0F,0xFE,0x00,0x00,0x00,0x1F,0xF8,
  0x00,0x00,0x00,0x7F,0xF0,0x00,0x00,0x00,0xFF,0xE0,0x00,0x00,0x01,0xFF,0x80,0x00,
  0x00,0x07,0xFF,0x00,0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,
  0x7F,0xF0,0x00,0x00,0x00,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x07,0xFF,
  0x00,0x00,0x00,0x1F,0xFC,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x7F,0xE0,0x00,
  0x00,0x01,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x07,0xFE,0x00,0x00,0x00,
  0x1F,0xFC,0x00,0x00,0x00,0x3F,0xF0,0x00,0x00,0x00,0xFF,0xE0,0x00,0x00,0x01,0xFF,
  0xC0,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,0x0F,0xFE,0x00,0x00,0x00,0x1F,0xF8,0x00,
  0x00,0x00,0x7F,0xF0,0x00,0x00,0x00,0xFF,0xE0,0x00,0x00,0x01,0xFF,0x80,0x00,0x00,
  0x07,0xFF,0x00,0x00,0x00,0x0F,0xFE,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x7F,
  0xF0,0x00,0x00,0x00,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x07,0xFF,0x00,
  0x00,0x00,0x0F,0xFC,0x00,0x00,0x00,0x3F,0xF8,0x00,0x00,0x00,0x7F,0xF0,0x00,0x00,
  0x01,0xFF,0xC0,0x00,0x00,0x03,0xFF,0x80,0x00,0x00,0x07,0xFE,0x00,0x00,0x00,0x1F,
  0xFC,0x00,0x00,0x00,0x00,0x00,0x7F,0xC0,0x00,0x00,0x00,0x7F,0xFF,0x00,0x00,0x00,
  0x3F,0xFF,0xF8,0x00,0x00,0x0F,0xFF,0xFF,0x80,0x00,0x07,0xFF,0xFF,0xF8,0x00,0x01,
  0xFF,0xFF,0xFF,0x80,0x00,0x3F,0xFF,0xFF,0xF8,0x00,0x0F,0xFF,0xFF,0xFF,0x80,0x03,
  0xFF,0xFF,0xFF,0xF0,0x00,0x7F,0xFF,0xFF,0xFF,0x00,0x1F,0xFF,0xC7,0xFF,0xE0,0x03,
  0xFF,0xC0,0x3F,0xFE,0x00,0x7F,0xF0,0x03,0xFF,0xC0,0x1F,0xFC,0x00,0x3F,0xF8,0x03,
  0xFF,0x80,0x03,0xFF,0x00,0x7F,0xE0,0x00,0x7F,0xF0,0x0F,0xFC,0x00,0x0F,0xFE,0x01,
  0xFF,0x80,0x00,0xFF,0xC0,0x3F,0xF0,0x00,0x1F,0xF8,0x07,0xFE,0x00,0x03,0xFF,0x00,
  0xFF,0xC0,0x00,0x7F,0xE0,0x1F,0xF8,0x00,0x0F,0xFC,0x03,0xFF,0x00,0x03,0xFF,0x00,
  0x7F,0xE0,0x00,0x7F,0xE0,0x07,0xFE,0x00,0x0F,0xFC,0x00,0xFF,0xC0,0x03,0xFF,0x00,
  0x0F,0xFC,0x00,0xFF,0xE0,0x01,0xFF,0xC0,0x3F,0xF8,0x00,0x1F,0xFE,0x1F,0xFF,0x00,
  0x01,0xFF,0xFF,0xFF,0xC0,0x00,0x1F,0xFF,0xFF,0xF0,0x00,0x01,0xFF,0xFF,0xFC,0x00,
  0x00,0x0F,0xFF,0xFE,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x7F,0xFF,0xFC,0x00,
  0x00,0x3F,0xFF,0xFF,0xE0,0x00,0x0F,0xFF,0xFF,0xFE,0x00,0x03,0xFF,0xFF,0xFF,0xE0,
  0x00,0xFF,0xFF,0xFF,0xFE,0x00,0x3F,0xFF,0x07,0xFF,0xE0,0x07,0xFF,0x80,0x3F,0xFC,
  0x01,0xFF,0xC0,0x03,0xFF,0xC0,0x3F,0xF8,0x00,0x3F,0xF8,0x0F,0xFE,0x00,0x03,0xFF,
  0x01,0xFF,0x80,0x00,0x7F,0xF0,0x3F,0xF0,0x00,0x0F,0xFE,0x0F,0xFE,0x00,0x00,0xFF,
  0xC1,0xFF,0xC0,0x00,0x1F,0xF8,0x3F,0xF8,0x00,0x03,0xFF,0x07,0xFF,0x00,0x00,0x7F,
  0xE0,0xFF,0xE0,0x00,0x0F,0xFC,0x1F,0xFC,0x00,0x01,0xFF,0x83,0xFF,0x80,0x00,0x3F,
  0xF0,0x7F,0xF0,0x00,0x0F,0xFE,0x07,0xFF,0x00,0x01,0xFF,0xC0,0xFF,0xE0,0x00,0x7F,
  0xF8,0x1F,0xFE,0x00,0x1F,0xFE,0x03,0xFF,0xE0,0x07,0xFF,0xC0,0x3F,0xFF,0x83,0xFF,
  0xF8,0x07,0xFF,0xFF,0xFF,0xFE,0x00,0x7F,0xFF,0xFF,0xFF,0xC0,0x07,0xFF,0xFF,0xFF,
  0xF0,0x00,0xFF,0xFF,0xFF,0xFC,0x00,0x0F,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,
  0xC0,0x00,0x07,0xFF,0xFF,0xF0,0x00,0x00,0x7F,0xFF,0xF8,0x00,0x00,0x03,0xFF,0xFC,
  0x00,0x00,0x00,0x07,0xFC,0x00,0x00,0x00,0x00,0x7F,0x80,0x00,0x00,0x00,0x7F,0xFE,
  0x00,0x00,0x00,0x3F,0xFF,0xF0,0x00,0x00,0x1F,0xFF,0xFF,0x00,0x00,0x07,0xFF,0xFF,
  0xF0,0x00,0x01,0xFF,0xFF,0xFF,0x00,0x00,0x7F,0xFF,0xFF,0xF0,0x00,0x0F,0xFF,0xFF,
  0xFF,0x00,0x03,0xFF,0xFF,0xFF,0xE0,0x00,0xFF,0xFF,0xFF,0xFE,0x00,0x1F,0xFF,0xFF,
  0xFF,0xC0,0x07,0xFF,0xE0,0x7F,0xFC,0x00,0xFF,0xF0,0x07,0xFF,0x80,0x1F,0xFC,0x00,
  0x7F,0xF8,0x07,0xFF,0x00,0x07,0xFF,0x00,0xFF,0xE0,0x00,0x7F,0xE0,0x1F,0xF8,0x00,
  0x0F,0xFE,0x03,0xFF,0x00,0x00,0xFF,0xC0,0xFF,0xE0,0x00,0x1F,0xF8,0x1F,0xF8,0x00,
  0x03,0xFF,0x03,0xFF,0x00,0x00,0x7F,0xF0,0x7F,0xE0,0x00,0x07,0xFE,0x0F,0xFC,0x00,
  0x00,0xFF,0xC1,0xFF,0x80,0x00,0x1F,0xF8,0x3F,0xF0,0x00,0x07,0xFF,0x07,0xFE,0x00,
  0x00,0xFF,0xE0,0xFF,0xC0,0x00,0x1F,0xFC,0x1F,0xFC,0x00,0x03,0xFF,0x81,0xFF,0x80,
  0x00,0x7F,0xF8,0x3F,0xF8,0x00,0x1F,0xFF,0x07,0xFF,0x00,0x03,0xFF,0xE0,0xFF,0xF0,
  0x00,0xFF,0xFC,0x0F,0xFF,0x00,0x3F,0xFF,0x81,0xFF,0xF0,0x0F,0xFF,0xF0,0x1F,0xFF,
  0xCF,0xFF,0xFE,0x03,0xFF,0xFF,0xFF,0xFF,0xC0,0x3F,0xFF,0xFF,0xFF,0xF8,0x07,0xFF,
  0xFF,0xFF,0xFF,0x00,0x7F,0xFF,0xFF,0x7F,0xC0,0x07,0xFF,0xFF,0xCF,0xF8,0x00,0x7F,
  0xFF,0xF9,0xFF,0x00,0x07,0xFF,0xFE,0x3F,0xE0,0x00,0x7F,0xFF,0x87,0xFC,0x00,0x03,
  0xFF,0xC0,0xFF,0x80,0x00,0x1F,0xE0,0x1F,0xF0,0x00,0x00,0x00,0x07,0xFE,0x00,0x00,
  0x00,0x00,0xFF,0xC0,0x00,0x00,0x00,0x1F,0xF0,0x00,0x00,0x00,0x03,0xFE,0x00,0x00,
  0x00,0x00,0x7F,0xC0,0x7F,0xC0,0x00,0x1F,0xF8,0x0F,0xFC,0x00,0x03,0xFE,0x01,0xFF,
  0x80,0x00,0xFF,0xC0,0x3F,0xF0,0x00,0x1F,0xF8,0x07,0xFF,0x00,0x07,0xFF,0x00,0x7F,
  0xF0,0x01,0xFF,0xC0,0x0F,0xFF,0x00,0x7F,0xF8,0x01,0xFF,0xF0,0x1F,0xFE,0x00,0x1F,
  0xFF,0xFF,0xFF,0xC0,0x03,0xFF,0xFF,0xFF,0xF0,0x00,0x3F,0xFF,0xFF,0xFE,0x00,0x03,
  0xFF,0xFF,0xFF,0x80,0x00,0x7F,0xFF,0xFF,0xE0,0x00,0x07,0xFF,0xFF,0xF8,0x00,0x00,
  0x7F,0xFF,0xFE,0x00,0x00,0x07,0xFF,0xFF,0x80,0x00,0x00,0x3F,0xFF,0xE0,0x00,0x00,
  0x03,0xFF,0xF0,0x00,0x00,0x00,0x0F,0xF0,0x00,0x00,0x01,0x80,0x03,0xF0,0x03,0xFC,
  0x01,0xFF,0x01,0xFF,0x80,0xFF,0xE0,0x7F,0xF0,0x3F,0xF8,0x1F,0xFC,0x0F,0xFE,0x07,
  0xFF,0x03,0xFF,0x00,0xFF,0x80,0x7F,0x80,0x0F,0x80,0x03,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xE0,
  0x03,0xF8,0x03,0xFE,0x01,0xFF,0x01,0xFF,0x80,0xFF,0xE0,0x7F,0xF0,0x3F,0xF8,0x1F,
  0xFC,0x0F,0xFE,0x07,0xFE,0x01,0xFF,0x00,0xFF,0x00,0x3F,0x80,0x0F,0x00,
  0x00 };
const GFXglyph BigTimeGlyphs[] PROGMEM = {
  {      0,  15,   0,  15,    0,    0 },   // 0x20 ' '
  {      0,  17,  68,  17,    0,  -67 },   // 0x21 '!'
  {    145,  33,  67,  33,    0,  -67 },   // 0x22 '"'
  {    422,  45,  67,  44,   -1,  -67 },   // 0x23 '#'
  {    799,  46,  88,  46,    0,  -78 },   // 0x24 '$'
  {   1305,  65,  69,  65,    0,  -68 },   // 0x25 '%'
  {   1866,  46,  69,  46,    0,  -68 },   // 0x26 '&'
  {   2263,  19,  67,  19,    0,  -67 },   // 0x27 "'"
  {   2423,  24,  84,  23,    0,  -71 },   // 0x28 '('
  {   2675,  24,  84,  23,    0,  -71 },   // 0x29 ')'
  {   2927,  37,  67,  37,    0,  -67 },   // 0x2A '*'
  {   3237,  46,  49,  46,    0,  -49 },   // 0x2B '+'
  {   3519,  17,  25,  17,    0,  -11 },   // 0x2C ','
  {   3573,  31,  32,  31,    0,  -32 },   // 0x2D '-'
  {   3697,  17,  16,  17,    0,  -15 },   // 0x2E '.'
  {   3731,  26,  81,  25,    0,  -71 },   // 0x2F '/'
  {   3995,  46,  69,  46,    0,  -68 },   // 0x30 '0'
  {   4392,  27,  67,  46,    9,  -67 },   // 0x31 '1'
  {   4619,  42,  68,  46,    2,  -68 },   // 0x32 '2'
  {   4976,  43,  69,  46,    1,  -68 },   // 0x33 '3'
  {   5347,  46,  67,  46,    0,  -67 },   // 0x34 '4'
  {   5733,  42,  68,  46,    2,  -67 },   // 0x35 '5'
  {   6090,  43,  69,  46,    1,  -68 },   // 0x36 '6'
  {   6461,  39,  67,  46,    3,  -67 },   // 0x37 '7'
  {   6788,  43,  69,  46,    1,  -68 },   // 0x38 '8'
  {   7159,  43,  69,  46,    1,  -68 },   // 0x39 '9'
  {   7530,  17,  47,   9,   -4,  -46 },   // 0x3A ':'
};
const GFXfont BigTime PROGMEM = { (uint8_t *)BigTimeBitmaps, (GFXglyph *)BigTimeGlyphs, 0x20, 0x3A, 113 };
// Label27: generated from InterDisplay-SemiBold.otf at 27px by make_gfx_font.py
const uint8_t Label27Bitmaps[] PROGMEM = {
  0x79,0xE7,0x9E,0x38,0xE3,0x8E,0x38,0xE3,0x8E,0x38,0xE0,0x00,0x39,0xE7,0x8C,0x39,
  0xC3,0x9C,0x39,0xC3,0x9C,0x39,0xC3,0x9C,0x39,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x03,
  0x86,0x01,0x83,0x00,0xC1,0x80,0xE1,0xC1,0xFF,0xF9,0xFF,0xFC,0xFF,0xFE,0x0C,0x18,
  0x06,0x0C,0x07,0x0E,0x03,0x87,0x0F,0xFF,0xE7,0xFF,0xF3,0xFF,0xF8,0x30,0x60,0x38,
  0x70,0x1C,0x38,0x0E,0x1C,0x07,0x0E,0x00,0x00,0xC0,0x00,0x60,0x00,0x30,0x00,0xFE,
  0x00,0xFF,0xC0,0xFF,0xF0,0xF7,0x78,0x71,0x9C,0x78,0xCF,0x3C,0x60,0x0F,0x70,0x07,
  0xF8,0x01,0xFF,0x80,0x3F,0xF0,0x07,0xF8,0x01,0x9E,0x70,0xC7,0x3C,0x63,0x9E,0x33,
  0xC7,0xBB,0xE3,0xFF,0xE0,0xFF,0xE0,0x1F,0xC0,0x01,0x80,0x00,0xC0,0x00,0x60,0x00,
  0x0F,0x00,0x70,0x1F,0x80,0x60,0x39,0xC0,0xE0,0x30,0xC1,0xC0,0x30,0xC1,0x80,0x30,
  0xC3,0x80,0x30,0xC7,0x00,0x39,0xC6,0x00,0x1F,0x8E,0x00,0x0F,0x1C,0x00,0x00,0x18,
  0x00,0x00,0x38,0xF8,0x00,0x71,0xFC,0x00,0xE3,0x8C,0x00,0xE3,0x8E,0x01,0xC3,0x8E,
  0x03,0x83,0x8E,0x03,0x03,0x9C,0x07,0x01,0xFC,0x0E,0x00,0xF0,0x07,0xC0,0x03,0xF8,
  0x01,0xFF,0x00,0x71,0xC0,0x1C,0x38,0x07,0x1C,0x01,0xC7,0x00,0x7F,0x80,0x0F,0xC0,
  0x03,0xE0,0x01,0xF8,0x00,0xFF,0x1C,0x79,0xE7,0x1C,0x3D,0xC7,0x07,0xE1,0xC0,0xF8,
  0x78,0x3E,0x0F,0xFF,0x81,0xFF,0xF0,0x1F,0x1E,0x38,0x70,0xE1,0xC3,0x87,0x0E,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x07,0x83,0x83,0xC1,
  0xE0,0xE0,0x70,0x78,0x3C,0x1C,0x0E,0x07,0x03,0x81,0xC0,0xE0,0x70,0x38,0x1E,0x0F,
  0x03,0x81,0xC0,0xF0,0x38,0x1C,0x07,0x00,0x60,0x38,0x1E,0x07,0x03,0xC1,0xE0,0x70,
  0x38,0x1C,0x0F,0x07,0x83,0xC0,0xE0,0x70,0x78,0x3C,0x1E,0x0F,0x07,0x03,0x83,0xC1,
  0xC0,0xE0,0xF0,0x70,0x00,0x03,0x00,0x0C,0x03,0x32,0x0E,0xDC,0x3F,0xF0,0x3F,0x00,
  0xFC,0x0F,0xFC,0x3B,0x70,0x0C,0x00,0x30,0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,
  0x00,0x1C,0x01,0xFF,0xF0,0xFF,0xF8,0x7F,0xFC,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,
  0x38,0x00,0x1C,0x00,0x00,0x00,0x00,0xE7,0x9C,0x71,0xC6,0x18,0x3F,0xC3,0xFC,0x3F,
  0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x9E,0x30,0x01,0x80,
  0xE0,0x38,0x0E,0x07,0x01,0xC0,0x70,0x18,0x0E,0x03,0x80,0xE0,0x70,0x1C,0x07,0x01,
  0xC0,0xE0,0x38,0x0E,0x03,0x81,0xC0,0x70,0x1C,0x0E,0x03,0x80,0x03,0xE0,0x07,0xFC,
  0x07,0xFF,0x03,0xC7,0xC3,0xC0,0xE1,0xC0,0x79,0xE0,0x3C,0xE0,0x0E,0x70,0x07,0x38,
  0x03,0x9C,0x01,0xCE,0x00,0xE7,0x00,0x73,0xC0,0x78,0xE0,0x3C,0x78,0x1C,0x3E,0x1E,
  0x0F,0xFE,0x03,0xFE,0x00,0x7C,0x00,0x07,0x87,0xE3,0xF9,0xFE,0x77,0x91,0xE0,0x78,
  0x1E,0x07,0x81,0xE0,0x78,0x1E,0x07,0x81,0xE0,0x78,0x1E,0x07,0x81,0xE0,0x78,0x1E,
  0x07,0xE0,0x0F,0xF0,0x1F,0xF8,0x3C,0x3C,0x78,0x1C,0x78,0x1E,0x70,0x1E,0x00,0x1C,
  0x00,0x1C,0x00,0x3C,0x00,0x78,0x00,0xF0,0x01,0xE0,0x07,0xC0,0x0F,0x80,0x1F,0x00,
  0x3C,0x00,0x7F,0xFE,0x7F,0xFE,0x7F,0xFE,0x07,0xE0,0x07,0xFC,0x07,0xFF,0x07,0x83,
  0xC3,0x81,0xE1,0xC0,0x70,0x00,0x78,0x00,0x38,0x01,0xF8,0x01,0xF8,0x00,0xFF,0x00,
  0x07,0xC0,0x00,0xE3,0x80,0x71,0xE0,0x3C,0xF0,0x1C,0x3C,0x1E,0x0F,0xFE,0x03,0xFE,
  0x00,0xFC,0x00,0x00,0x7C,0x00,0x3E,0x00,0x3F,0x00,0x3F,0x80,0x1D,0xC0,0x1C,0xE0,
  0x1E,0x70,0x0E,0x38,0x0E,0x1C,0x0F,0x0E,0x0F,0x07,0x07,0x03,0x87,0x81,0xC3,0xFF,
  0xFD,0xFF,0xFE,0xFF,0xFF,0x00,0x1C,0x00,0x0E,0x00,0x07,0x00,0x03,0x80,0x3F,0xFC,
  0x3F,0xFC,0x3F,0xFC,0x38,0x00,0x78,0x00,0x78,0x00,0x79,0xC0,0x7F,0xF0,0x7F,0xF8,
  0x7C,0x3C,0x78,0x1E,0x00,0x1E,0x00,0x0E,0x00,0x0E,0x70,0x1E,0x78,0x1E,0x3C,0x3C,
  0x3F,0xF8,0x1F,0xF0,0x07,0xE0,0x03,0xE0,0x0F,0xF8,0x1F,0xFC,0x1C,0x1E,0x38,0x0E,
  0x38,0x00,0x78,0x00,0x71,0xE0,0x77,0xF8,0x7F,0xFC,0x7C,0x1E,0x78,0x1E,0x78,0x0E,
  0x78,0x0E,0x78,0x0E,0x38,0x1E,0x3C,0x3E,0x1F,0xFC,0x0F,0xF8,0x03,0xE0,0x7F,0xFC,
  0xFF,0xF9,0xFF,0xF0,0x01,0xE0,0x03,0x80,0x0F,0x00,0x1C,0x00,0x78,0x00,0xE0,0x03,
  0xC0,0x07,0x00,0x1E,0x00,0x38,0x00,0xF0,0x01,0xC0,0x07,0x80,0x0E,0x00,0x3C,0x00,
  0x70,0x01,0xE0,0x00,0x07,0xE0,0x0F,0xF8,0x1F,0xFC,0x3C,0x3C,0x38,0x1C,0x38,0x1E,
  0x38,0x1C,0x3C,0x3C,0x1F,0xF8,0x07,0xF0,0x1F,0xF8,0x3C,0x3C,0x78,0x1E,0x78,0x0E,
  0x70,0x0E,0x78,0x0E,0x7C,0x1E,0x3F,0xFC,0x1F,0xF8,0x07,0xE0,0x07,0xE0,0x1F,0xF0,
  0x3F,0xF8,0x3C,0x3C,0x78,0x1E,0x70,0x1E,0x70,0x0E,0x70,0x1E,0x78,0x1E,0x3C,0x3E,
  0x3F,0xFE,0x1F,0xEE,0x07,0xCE,0x00,0x0E,0x00,0x1E,0x78,0x1C,0x3C,0x3C,0x3F,0xF8,
  0x1F,0xF0,0x07,0xC0,0x31,0xE7,0x8E,0x00,0x00,0x00,0x00,0x03,0x9E,0x78,0xC0,0x00,
  0xE7,0x9E,0x38,0x00,0x00,0x00,0x00,0x0E,0x79,0xC7,0x1C,0x61,0x80,0x00,0x02,0x00,
  0x07,0x00,0x0F,0x80,0x1F,0xC0,0x7F,0x00,0xFE,0x00,0xFC,0x00,0x78,0x00,0x3F,0x00,
  0x07,0xE0,0x00,0xFE,0x00,0x1F,0xC0,0x03,0xE0,0x00,0x70,0x00,0x08,0x3F,0xFE,0x1F,
  0xFF,0x0F,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xF8,0x7F,0xFC,0x3F,0xFE,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x0F,0xC0,0x03,0xF8,0x00,
  0x7F,0x00,0x0F,0xE0,0x00,0xF8,0x00,0x3C,0x00,0x7E,0x00,0xFE,0x01,0xFC,0x03,0xF8,
  0x03,0xE0,0x01,0xC0,0x00,0x00,0x00,0x07,0xC0,0x3F,0xE0,0xFF,0xE3,0xC1,0xC7,0x83,
  0xCE,0x07,0x80,0x0F,0x00,0x1C,0x00,0xF8,0x03,0xE0,0x0F,0x00,0x1C,0x00,0x38,0x00,
  0x70,0x00,0x00,0x00,0x00,0x03,0x00,0x0F,0x00,0x1E,0x00,0x18,0x00,0x00,0x1F,0x00,
  0x00,0x1F,0xFC,0x00,0x0F,0xFF,0xE0,0x03,0xE0,0x3E,0x00,0xF0,0x01,0xE0,0x3C,0x00,
  0x1E,0x07,0x01,0x01,0xE1,0xC1,0xFF,0x1C,0x38,0x7F,0xE3,0x8E,0x1E,0x3C,0x39,0xC3,
  0x83,0x87,0x38,0xE0,0x70,0xE7,0x1C,0x0E,0x1C,0xE3,0x81,0xC3,0x9C,0x70,0x38,0x73,
  0x87,0x07,0x1C,0x70,0xF1,0xF3,0x87,0x0F,0xFF,0xF0,0xE0,0xFD,0xF8,0x0E,0x06,0x04,
  0x01,0xE0,0x00,0x00,0x1E,0x00,0x00,0x01,0xF0,0x18,0x00,0x1F,0xFF,0x80,0x00,0xFF,
  0xF0,0x00,0x03,0xE0,0x00,0x01,0xF0,0x00,0x3E,0x00,0x0F,0xC0,0x01,0xFC,0x00,0x3B,
  0x80,0x0F,0x70,0x01,0xCF,0x00,0x38,0xE0,0x0F,0x1E,0x01,0xC3,0xC0,0x78,0x38,0x0F,
  0x07,0x81,0xFF,0xF0,0x7F,0xFE,0x0F,0xFF,0xE1,0xC0,0x3C,0x78,0x03,0x8F,0x00,0x79,
  0xC0,0x0F,0x78,0x00,0xE0,0x3F,0xF0,0x1F,0xFE,0x0F,0xFF,0x87,0x03,0xC3,0x80,0xF1,
  0xC0,0x78,0xE0,0x3C,0x70,0x3C,0x3F,0xFC,0x1F,0xFC,0x0F,0xFF,0x87,0x03,0xE3,0x80,
  0xF1,0xC0,0x38,0xE0,0x1C,0x70,0x0E,0x38,0x0F,0x1F,0xFF,0x0F,0xFF,0x87,0xFF,0x00,
  0x01,0xF8,0x00,0x7F,0xE0,0x0F,0xFF,0x01,0xF0,0xF8,0x3E,0x03,0xC3,0xC0,0x1C,0x78,
  0x01,0xC7,0x80,0x00,0x78,0x00,0x07,0x80,0x00,0x78,0x00,0x07,0x80,0x00,0x78,0x00,
  0x07,0x80,0x1C,0x3C,0x01,0xC3,0xE0,0x3C,0x1F,0x0F,0x80,0xFF,0xF0,0x07,0xFE,0x00,
  0x1F,0x80,0x3F,0xF0,0x07,0xFF,0x80,0xFF,0xF8,0x1C,0x0F,0x83,0x80,0xF8,0x70,0x0F,
  0x0E,0x00,0xE1,0xC0,0x1E,0x38,0x03,0xC7,0x00,0x78,0xE0,0x0F,0x1C,0x01,0xE3,0x80,
  0x3C,0x70,0x07,0x0E,0x01,0xE1,0xC0,0x7C,0x38,0x1F,0x07,0xFF,0xC0,0xFF,0xF0,0x1F,
  0xF8,0x00,0x3F,0xFE,0x3F,0xFE,0x3F,0xFE,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,
  0x38,0x00,0x3F,0xFC,0x3F,0xFC,0x3F,0xFC,0x38,0x00,0x38,0x00,0x38,0x00,0x38,0x00,
  0x38,0x00,0x38,0x00,0x3F,0xFE,0x3F,0xFE,0x3F,0xFE,0x3F,0xFE,0x7F,0xFC,0xFF,0xF9,
  0xC0,0x03,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x7F,0xF8,0xFF,0xF1,0xFF,
  0xE3,0x80,0x07,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x00,
  0x01,0xF8,0x00,0x7F,0xE0,0x0F,0xFF,0x01,0xF0,0xF8,0x3E,0x03,0xC3,0xC0,0x1C,0x78,
  0x01,0xC7,0x80,0x00,0x78,0x00,0x07,0x01,0xFE,0x78,0x1F,0xE7,0x81,0xFE,0x78,0x01,
  0xE3,0x80,0x1E,0x3C,0x01,0xC3,0xE0,0x3C,0x1F,0x0F,0x80,0xFF,0xF0,0x07,0xFE,0x00,
  0x1F,0x80,0x38,0x03,0xC7,0x00,0x78,0xE0,0x0F,0x1C,0x01,0xE3,0x80,0x3C,0x70,0x07,
  0x8E,0x00,0xF1,0xC0,0x1E,0x3F,0xFF,0xC7,0xFF,0xF8,0xFF,0xFF,0x1C,0x01,0xE3,0x80,
  0x3C,0x70,0x07,0x8E,0x00,0xF1,0xC0,0x1E,0x38,0x03,0xC7,0x00,0x78,0xE0,0x0F,0x1C,
  0x01,0xE0,0x38,0x70,0xE1,0xC3,0x87,0x0E,0x1C,0x38,0x70,0xE1,0xC3,0x87,0x0E,0x1C,
  0x38,0x70,0xE1,0xC0,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,0x00,
  0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x8E,0x07,0x1C,0x0E,
  0x3C,0x3C,0x78,0x78,0x7F,0xE0,0x7F,0x80,0x7E,0x00,0x38,0x07,0x8E,0x03,0xC3,0x81,
  0xF0,0xE0,0xF8,0x38,0x3C,0x0E,0x1E,0x03,0x8F,0x00,0xE7,0x80,0x3B,0xC0,0x0F,0xF8,
  0x03,0xFE,0x00,0xFB,0xC0,0x3E,0x78,0x0F,0x1E,0x03,0x83,0xC0,0xE0,0x78,0x38,0x1E,
  0x0E,0x03,0xC3,0x80,0x78,0xE0,0x1E,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,
  0x07,0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xE0,0x01,0xC0,0x03,0x80,0x07,
  0x00,0x0E,0x00,0x1C,0x00,0x38,0x00,0x7F,0xF8,0xFF,0xF1,0xFF,0xE0,0x3E,0x00,0x7C,
  0x3E,0x00,0x7C,0x3F,0x00,0xFC,0x3F,0x00,0xFC,0x3F,0x00,0xFC,0x3F,0x81,0xFC,0x3B,
  0x81,0xDC,0x3B,0x83,0xDC,0x3B,0xC3,0x9C,0x39,0xC3,0x9C,0x39,0xC7,0x9C,0x39,0xE7,
  0x1C,0x38,0xE7,0x1C,0x38,0xEF,0x1C,0x38,0xFE,0x1C,0x38,0x7E,0x1C,0x38,0x7E,0x1C,
  0x38,0x3C,0x1C,0x38,0x3C,0x1C,0x38,0x3C,0x1C,0x3C,0x03,0xC7,0x80,0x78,0xF8,0x0F,
  0x1F,0x81,0xE3,0xF0,0x3C,0x7F,0x07,0x8E,0xE0,0xF1,0xDE,0x1E,0x39,0xE3,0xC7,0x1C,
  0x78,0xE3,0xCF,0x1C,0x39,0xE3,0x87,0xBC,0x70,0x7F,0x8E,0x07,0xF1,0xC0,0xFE,0x38,
  0x0F,0xC7,0x01,0xF8,0xE0,0x1F,0x1C,0x01,0xE0,0x01,0xF8,0x00,0x7F,0xE0,0x0F,0xFF,
  0x01,0xF0,0xF8,0x3E,0x03,0xC3,0xC0,0x3C,0x78,0x01,0xE7,0x80,0x1E,0x78,0x01,0xE7,
  0x80,0x0E,0x78,0x00,0xE7,0x80,0x0E,0x78,0x01,0xE7,0x80,0x1E,0x3C,0x03,0xC3,0xE0,
  0x3C,0x1F,0x0F,0x80,0xFF,0xF0,0x07,0xFE,0x00,0x1F,0x80,0x3F,0xF0,0x1F,0xFE,0x0F,
  0xFF,0x87,0x03,0xC3,0x80,0xF1,0xC0,0x38,0xE0,0x1C,0x70,0x0E,0x38,0x0F,0x1C,0x0F,
  0x0F,0xFF,0x87,0xFF,0x83,0xFF,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x1C,
  0x00,0x0E,0x00,0x07,0x00,0x00,0x01,0xF8,0x00,0x7F,0xE0,0x0F,0xFF,0x01,0xF0,0xF8,
  0x3E,0x03,0xC3,0xC0,0x3C,0x78,0x01,0xE7,0x80,0x1E,0x78,0x01,0xE7,0x80,0x0E,0x78,
  0x00,0xE7,0x80,0x0E,0x78,0x01,0xE7,0x87,0x1E,0x3C,0x39,0xC3,0xE3,0xFC,0x1F,0x1F,
  0x80,0xFF,0xF0,0x07,0xFF,0x00,0x1F,0xF8,0x00,0x03,0xC0,0x00,0x00,0x3F,0xF0,0x1F,
  0xFE,0x0F,0xFF,0x87,0x03,0xE3,0x80,0xF1,0xC0,0x38,0xE0,0x1C,0x70,0x0E,0x38,0x0F,
  0x1C,0x0F,0x8F,0xFF,0x87,0xFF,0x83,0xFF,0x81,0xC3,0xC0,0xE0,0xF0,0x70,0x38,0x38,
  0x1E,0x1C,0x07,0x0E,0x03,0xC7,0x00,0xE0,0x03,0xF0,0x07,0xFC,0x07,0xFF,0x07,0x83,
  0xC3,0x80,0xE3,0xC0,0x79,0xE0,0x00,0x78,0x00,0x3F,0xC0,0x0F,0xFC,0x01,0xFF,0x80,
  0x0F,0xC0,0x00,0xF3,0x80,0x39,0xE0,0x1C,0xF0,0x1E,0x3C,0x1F,0x1F,0xFF,0x03,0xFF,
  0x00,0xFE,0x00,0x7F,0xFF,0x3F,0xFF,0x9F,0xFF,0xC0,0x38,0x00,0x1C,0x00,0x0E,0x00,
  0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x1C,0x00,0x0E,
  0x00,0x07,0x00,0x03,0x80,0x01,0xC0,0x00,0xE0,0x00,0x70,0x00,0x38,0x00,0x38,0x03,
  0x87,0x00,0x70,0xE0,0x0E,0x1C,0x01,0xC3,0x80,0x38,0x70,0x07,0x0E,0x00,0xE1,0xC0,
  0x1C,0x38,0x03,0x87,0x00,0x70,0xE0,0x0E,0x1C,0x01,0xC3,0x80,0x38,0x70,0x07,0x0E,
  0x00,0xE1,0xE0,0x3C,0x1F,0x1F,0x03,0xFF,0xE0,0x1F,0xF0,0x01,0xFC,0x00,0xF0,0x01,
  0xCE,0x00,0x79,0xE0,0x0F,0x3C,0x01,0xC3,0x80,0x78,0x78,0x0E,0x0F,0x01,0xC0,0xE0,
  0x78,0x1E,0x0E,0x03,0xC1,0xC0,0x38,0x78,0x07,0x8E,0x00,0x71,0xC0,0x0E,0x78,0x01,
  0xEE,0x00,0x1D,0xC0,0x03,0xF8,0x00,0x7E,0x00,0x07,0xC0,0x00,0xF8,0x00,0xF0,0x1E,
  0x03,0xCE,0x03,0xC0,0x79,0xC0,0x7C,0x0F,0x3C,0x0F,0x81,0xC7,0x83,0xF0,0x38,0x70,
  0x7E,0x0F,0x0E,0x0E,0xE1,0xE1,0xE1,0xDC,0x38,0x3C,0x73,0x87,0x03,0x8E,0x71,0xE0,
  0x71,0xC7,0x3C,0x0F,0x38,0xE7,0x01,0xEE,0x1C,0xE0,0x1D,0xC3,0xBC,0x03,0xB8,0x3F,
  0x00,0x7F,0x07,0xE0,0x07,0xC0,0xFC,0x00,0xF8,0x1F,0x80,0x1F,0x01,0xE0,0x03,0xE0,
  0x3C,0x00,0x78,0x07,0x87,0x80,0xF0,0xF0,0x3C,0x0F,0x0F,0x00,0xF1,0xE0,0x1E,0x78,
  0x01,0xFE,0x00,0x1F,0xC0,0x03,0xF0,0x00,0x3C,0x00,0x07,0xC0,0x01,0xF8,0x00,0x7F,
  0x80,0x0F,0x78,0x03,0xCF,0x00,0xF0,0xF0,0x1E,0x0F,0x07,0x81,0xE1,0xE0,0x1E,0x38,
  0x01,0xE0,0xF0,0x03,0xCF,0x00,0xF0,0xF0,0x1E,0x1E,0x07,0x81,0xE1,0xE0,0x3C,0x3C,
  0x03,0xCF,0x00,0x39,0xE0,0x07,0xF8,0x00,0x7E,0x00,0x0F,0xC0,0x00,0xF0,0x00,0x1E,
  0x00,0x03,0xC0,0x00,0x78,0x00,0x0F,0x00,0x01,0xE0,0x00,0x3C,0x00,0x07,0x80,0x00,
  0xF0,0x00,0x7F,0xFF,0x3F,0xFF,0x9F,0xFF,0xC0,0x03,0xE0,0x01,0xE0,0x01,0xE0,0x01,
  0xE0,0x01,0xE0,0x00,0xF0,0x00,0xF0,0x00,0xF0,0x00,0xF0,0x00,0xF8,0x00,0x78,0x00,
  0x78,0x00,0x78,0x00,0x78,0x00,0x3F,0xFF,0x9F,0xFF,0xCF,0xFF,0xE0,0x3E,0x1F,0x8F,
  0xC7,0x03,0x81,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x81,0xC0,0xE0,0x70,0x38,
  0x1C,0x0E,0x07,0x03,0x81,0xC0,0xF8,0x7E,0x3F,0x00,0x60,0x38,0x07,0x01,0xC0,0x70,
  0x1C,0x03,0x80,0xE0,0x38,0x06,0x01,0xC0,0x70,0x1C,0x03,0x80,0xE0,0x38,0x0E,0x01,
  0xC0,0x70,0x1C,0x07,0x00,0xE0,0x38,0x0E,0x7E,0x3F,0x1F,0x83,0xC1,0xE0,0xF0,0x78,
  0x3C,0x1E,0x0F,0x07,0x83,0xC1,0xE0,0xF0,0x78,0x3C,0x1E,0x0F,0x07,0x83,0xC1,0xE0,
  0xF1,0xF8,0xFC,0x7E,0x00,0x0F,0x00,0xF0,0x1F,0x81,0xD8,0x39,0xC3,0x8C,0x30,0xE7,
  0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFC,0x0F,0x01,0xF8,0x3B,0xC7,0x0C,0x70,0xC7,0x0C,
  0x39,0xC3,0xF8,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x07,0xC0,0x3F,0xE0,0xFF,0xE1,0xC3,0xC3,0x03,0x80,0x0F,
  0x03,0xFE,0x1F,0xFC,0x78,0x38,0xE0,0x71,0xC1,0xE3,0xFF,0xC3,0xFB,0x83,0xE7,0x00,
  0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x79,0xE0,0x7F,0xF8,
  0x7F,0xFC,0x7C,0x3C,0x78,0x1E,0x78,0x1E,0x78,0x0E,0x78,0x0E,0x78,0x1E,0x78,0x1E,
  0x7C,0x3C,0x7F,0xFC,0x7F,0xF8,0x79,0xE0,0x07,0xC0,0x3F,0xE0,0xFF,0xE1,0xE1,0xC7,
  0x83,0xCE,0x00,0x1C,0x00,0x38,0x00,0x70,0x00,0xF0,0x78,0xF0,0xE1,0xFF,0xC1,0xFF,
  0x00,0xF8,0x00,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x07,
  0x9C,0x1F,0xDC,0x3F,0xFC,0x3C,0x3C,0x78,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,
  0x1C,0x78,0x1C,0x3C,0x3C,0x3F,0xFC,0x1F,0xDC,0x0F,0x9C,0x07,0xC0,0x3F,0xE0,0xFF,
  0xE1,0xC1,0xC7,0x01,0xCE,0x03,0x9F,0xFF,0x3F,0xFE,0x70,0x00,0xE0,0x00,0xE0,0xF1,
  0xFF,0xC1,0xFF,0x00,0xF8,0x00,0x07,0x8F,0xC7,0xE3,0x81,0xC0,0xE3,0xFF,0xFF,0xFF,
  0x8E,0x07,0x03,0x81,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x80,0x07,0x9C,0x1F,
  0xDC,0x3F,0xFC,0x3C,0x3C,0x78,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x78,
  0x1C,0x3C,0x3C,0x3F,0xFC,0x1F,0xDC,0x0F,0x9C,0x00,0x1C,0x38,0x1C,0x3C,0x3C,0x1F,
  0xF8,0x0F,0xF0,0x01,0x00,0x78,0x00,0xF0,0x01,0xE0,0x03,0xC0,0x07,0x80,0x0F,0x00,
  0x1E,0x78,0x3F,0xFC,0x7F,0xF8,0xF8,0x79,0xE0,0xF3,0xC0,0xE7,0x81,0xCF,0x03,0x9E,
  0x07,0x3C,0x0E,0x78,0x1C,0xF0,0x39,0xE0,0x73,0xC0,0xE0,0x31,0xE7,0x8C,0x00,0x07,
  0x9E,0x79,0xE7,0x9E,0x79,0xE7,0x9E,0x79,0xE7,0x9E,0x18,0x78,0xF0,0xC0,0x00,0x0F,
  0x1E,0x3C,0x78,0xF1,0xE3,0xC7,0x8F,0x1E,0x3C,0x78,0xF1,0xE3,0xC7,0x8F,0x7C,0xF9,
  0xC0,0x78,0x00,0xF0,0x01,0xE0,0x03,0xC0,0x07,0x80,0x0F,0x00,0x1E,0x0F,0x3C,0x3C,
  0x78,0xF0,0xF3,0xC1,0xEF,0x03,0xFC,0x07,0xF8,0x0F,0xF8,0x1F,0xF0,0x3C,0xF0,0x78,
  0xF0,0xF0,0xF1,0xE1,0xE3,0xC1,0xE0,0x79,0xE7,0x9E,0x79,0xE7,0x9E,0x79,0xE7,0x9E,
  0x79,0xE7,0x9E,0x79,0xE7,0x9E,0x79,0xE1,0xE0,0xF7,0xE7,0xF1,0xFF,0xFF,0xE3,0xE3,
  0xE1,0xE7,0x83,0xC3,0xCF,0x07,0x03,0x9E,0x0E,0x07,0x3C,0x1C,0x0E,0x78,0x38,0x1C,
  0xF0,0x70,0x39,0xE0,0xE0,0x73,0xC1,0xC0,0xE7,0x83,0x81,0xCF,0x07,0x03,0x80,0x79,
  0xE0,0xFF,0xF1,0xFF,0xE3,0xE1,0xE7,0x83,0xCF,0x03,0x9E,0x07,0x3C,0x0E,0x78,0x1C,
  0xF0,0x39,0xE0,0x73,0xC0,0xE7,0x81,0xCF,0x03,0x80,0x07,0xC0,0x3F,0xE0,0xFF,0xE1,
  0xE1,0xE7,0x81,0xCE,0x03,0x9C,0x07,0x38,0x0E,0x70,0x1C,0xF0,0x38,0xF0,0xF1,0xFF,
  0xC1,0xFF,0x00,0xF8,0x00,0x79,0xE0,0x7F,0xF8,0x7F,0xFC,0x7C,0x3C,0x78,0x1E,0x78,
  0x1E,0x78,0x0E,0x78,0x0E,0x78,0x1E,0x78,0x1E,0x7C,0x3C,0x7F,0xFC,0x7F,0xF8,0x79,
  0xE0,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x78,0x00,0x00,0x00,0x07,0x9C,0x1F,
  0xFC,0x3F,0xFC,0x3C,0x3C,0x78,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x70,0x1C,0x78,
  0x1C,0x3C,0x3C,0x3F,0xFC,0x1F,0xDC,0x0F,0x9C,0x00,0x1C,0x00,0x1C,0x00,0x1C,0x00,
  0x1C,0x00,0x1C,0x00,0x00,0x7B,0x9F,0xE7,0xF9,0xF0,0x78,0x1E,0x07,0x81,0xE0,0x78,
  0x1E,0x07,0x81,0xE0,0x78,0x1E,0x00,0x0F,0x80,0xFF,0x83,0xFF,0x1E,0x1C,0x78,0x31,
  0xF0,0x03,0xFC,0x07,0xFC,0x03,0xF0,0x01,0xE7,0x07,0x9E,0x1C,0x3F,0xF0,0x7F,0x80,
  0x00,0x00,0x1C,0x0E,0x07,0x03,0x8F,0xFF,0xFF,0xFE,0x38,0x1C,0x0E,0x07,0x03,0x81,
  0xC0,0xE0,0x70,0x3F,0x1F,0x87,0xC0,0x78,0x1C,0xF0,0x39,0xE0,0x73,0xC0,0xE7,0x81,
  0xCF,0x03,0x9E,0x07,0x3C,0x0E,0x78,0x1C,0x70,0x78,0xF1,0xF1,0xFF,0xE1,0xFD,0xC1,
  0xF3,0x80,0xF0,0x1C,0xE0,0x79,0xE0,0xE1,0xC1,0xC3,0x87,0x87,0x8E,0x07,0x1C,0x0E,
  0x78,0x1E,0xE0,0x1D,0xC0,0x3F,0x00,0x3E,0x00,0x7C,0x00,0xF0,0x00,0xF0,0x70,0x39,
  0xC1,0xE1,0xE7,0x07,0x87,0x1E,0x3E,0x1C,0x38,0xFC,0x70,0xE3,0x73,0xC3,0x9D,0xCE,
  0x07,0x73,0x38,0x1D,0xCE,0xE0,0x76,0x3F,0x01,0xF8,0xFC,0x03,0xE1,0xF0,0x0F,0x07,
  0x80,0x3C,0x1E,0x00,0x78,0x38,0xE1,0xE3,0xCF,0x07,0x38,0x0F,0xE0,0x3F,0x00,0x78,
  0x01,0xE0,0x0F,0xC0,0x7F,0x81,0xCF,0x0F,0x1C,0x78,0x79,0xC0,0xF0,0xF0,0x1C,0xE0,
  0x79,0xE0,0xE1,0xC1,0xC3,0x87,0x87,0x8E,0x07,0x1C,0x0E,0x78,0x1E,0xE0,0x1D,0xC0,
  0x3F,0x00,0x3E,0x00,0x7C,0x00,0xF0,0x01,0xE0,0x03,0x80,0x0F,0x00,0x7E,0x00,0xF8,
  0x01,0xC0,0x00,0x7F,0xF1,0xFF,0xC7,0xFF,0x00,0x3C,0x01,0xE0,0x0F,0x00,0x78,0x01,
  0xC0,0x0F,0x00,0x78,0x03,0xC0,0x1F,0xFC,0x7F,0xF1,0xFF,0xC0,0x00,0x40,0x78,0x1F,
  0x03,0xC0,0x70,0x1E,0x03,0xC0,0x78,0x0F,0x01,0xE0,0x38,0x1F,0x03,0xC0,0x7C,0x07,
  0x80,0x78,0x0F,0x01,0xE0,0x3C,0x07,0x80,0xF0,0x0E,0x01,0xF0,0x1E,0x01,0xC0,0x1C,
  0x0E,0x07,0x03,0x81,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x81,0xC0,0xE0,0x70,
  0x38,0x1C,0x0E,0x07,0x03,0x81,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x81,0xC0,
  0xE0,0x70,0x38,0x00,0x00,0x40,0x0F,0x01,0xF0,0x0F,0x01,0xE0,0x1C,0x03,0x80,0x70,
  0x0E,0x01,0xC0,0x3C,0x03,0xE0,0x3C,0x0F,0x83,0xC0,0x70,0x0E,0x01,0xC0,0x38,0x07,
  0x00,0xE0,0x3C,0x1F,0x03,0xE0,0x70,0x00,0x0E,0x00,0x0F,0xC3,0x8F,0xF3,0x87,0x3F,
  0xC3,0x8F,0xE0,0x01,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00 };
const GFXglyph Label27Glyphs[] PROGMEM = {
  {      0,   6,   0,   6,    0,    0 },   // 0x20 ' '
  {      0,   6,  20,   6,    0,  -20 },   // 0x21 '!'
  {     15,  12,  20,  12,    0,  -20 },   // 0x22 '"'
  {     45,  17,  20,  17,    0,  -20 },   // 0x23 '#'
  {     88,  17,  26,  17,    0,  -23 },   // 0x24 '$'
  {    144,  24,  20,  24,    0,  -20 },   // 0x25 '%'
  {    204,  18,  20,  17,    0,  -20 },   // 0x26 '&'
  {    249,   7,  20,   7,    0,  -20 },   // 0x27 "'"
  {    267,   9,  25,   9,    0,  -21 },   // 0x28 '('
  {    296,   9,  25,   9,    0,  -21 },   // 0x29 ')'
  {    325,  14,  20,  14,    0,  -20 },   // 0x2A '*'
  {    360,  17,  14,  17,    0,  -14 },   // 0x2B '+'
  {    390,   6,   8,   6,    0,   -4 },   // 0x2C ','
  {    396,  12,   9,  12,    0,   -9 },   // 0x2D '-'
  {    410,   6,   5,   6,    0,   -5 },   // 0x2E '.'
  {    414,  10,  24,   9,    0,  -21 },   // 0x2F '/'
  {    444,  17,  20,  17,    0,  -20 },   // 0x30 '0'
  {    487,  10,  20,  17,    3,  -20 },   // 0x31 '1'
  {    512,  16,  20,  17,    0,  -20 },   // 0x32 '2'
  {    552,  17,  20,  17,    0,  -20 },   // 0x33 '3'
  {    595,  17,  20,  17,    0,  -20 },   // 0x34 '4'
  {    638,  16,  20,  17,    0,  -20 },   // 0x35 '5'
  {    678,  16,  20,  17,    0,  -20 },   // 0x36 '6'
  {    718,  15,  20,  17,    1,  -20 },   // 0x37 '7'
  {    756,  16,  20,  17,    0,  -20 },   // 0x38 '8'
  {    796,  16,  20,  17,    0,  -20 },   // 0x39 '9'
  {    836,   6,  14,   6,    0,  -14 },   // 0x3A ':'
  {    847,   6,  18,   6,    0,  -14 },   // 0x3B ';'
  {    861,  17,  15,  17,    0,  -15 },   // 0x3C '<'
  {    893,  17,  12,  17,    0,  -12 },   // 0x3D '='
  {    919,  17,  15,  17,    0,  -15 },   // 0x3E '>'
  {    951,  15,  20,  15,    0,  -20 },   // 0x3F '?'
  {    989,  27,  26,  27,    0,  -20 },   // 0x40 '@'
  {   1077,  19,  20,  19,    0,  -20 },   // 0x41 'A'
  {   1125,  17,  20,  17,    0,  -20 },   // 0x42 'B'
  {   1168,  20,  20,  20,    0,  -20 },   // 0x43 'C'
  {   1218,  19,  20,  19,    0,  -20 },   // 0x44 'D'
  {   1266,  16,  20,  16,    0,  -20 },   // 0x45 'E'
  {   1306,  15,  20,  15,    0,  -20 },   // 0x46 'F'
  {   1344,  20,  20,  20,    0,  -20 },   // 0x47 'G'
  {   1394,  19,  20,  19,    0,  -20 },   // 0x48 'H'
  {   1442,   7,  20,   7,    0,  -20 },   // 0x49 'I'
  {   1460,  15,  20,  15,    0,  -20 },   // 0x4A 'J'
  {   1498,  18,  20,  18,    0,  -20 },   // 0x4B 'K'
  {   1543,  15,  20,  15,    0,  -20 },   // 0x4C 'L'
  {   1581,  24,  20,  24,    0,  -20 },   // 0x4D 'M'
  {   1641,  19,  20,  19,    0,  -20 },   // 0x4E 'N'
  {   1689,  20,  20,  20,    0,  -20 },   // 0x4F 'O'
  {   1739,  17,  20,  17,    0,  -20 },   // 0x50 'P'
  {   1782,  20,  22,  20,    0,  -20 },   // 0x51 'Q'
  {   1837,  17,  20,  17,    0,  -20 },   // 0x52 'R'
  {   1880,  17,  20,  17,    0,  -20 },   // 0x53 'S'
  {   1923,  17,  20,  17,    0,  -20 },   // 0x54 'T'
  {   1966,  19,  20,  19,    0,  -20 },   // 0x55 'U'
  {   2014,  19,  20,  19,    0,  -20 },   // 0x56 'V'
  {   2062,  27,  20,  26,    0,  -20 },   // 0x57 'W'
  {   2130,  19,  20,  18,    0,  -20 },   // 0x58 'X'
  {   2178,  19,  20,  18,    0,  -20 },   // 0x59 'Y'
  {   2226,  17,  20,  17,    0,  -20 },   // 0x5A 'Z'
  {   2269,   9,  25,   9,    0,  -21 },   // 0x5B '['
  {   2298,  10,  24,   9,    0,  -21 },   // 0x5C '\\\\'
  {   2328,   9,  25,   9,    0,  -21 },   // 0x5D ']'
  {   2357,  12,  19,  12,    0,  -19 },   // 0x5E '^'
  {   2386,  13,   3,  13,    0,    0 },   // 0x5F '_'
  {   2391,  12,  20,  12,    0,  -20 },   // 0x60 '`'
  {   2421,  15,  14,  15,    0,  -14 },   // 0x61 'a'
  {   2448,  16,  20,  16,    0,  -20 },   // 0x62 'b'
  {   2488,  15,  14,  15,    0,  -14 },   // 0x63 'c'
  {   2515,  16,  20,  16,    0,  -20 },   // 0x64 'd'
  {   2555,  15,  14,  15,    0,  -14 },   // 0x65 'e'
  {   2582,   9,  20,   9,    0,  -20 },   // 0x66 'f'
  {   2605,  16,  20,  16,    0,  -14 },   // 0x67 'g'
  {   2645,  15,  20,  15,    0,  -20 },   // 0x68 'h'
  {   2683,   6,  20,   6,    0,  -20 },   // 0x69 'i'
  {   2698,   7,  26,   6,   -1,  -20 },   // 0x6A 'j'
  {   2721,  15,  20,  14,    0,  -20 },   // 0x6B 'k'
  {   2759,   6,  20,   6,    0,  -20 },   // 0x6C 'l'
  {   2774,  23,  14,  23,    0,  -14 },   // 0x6D 'm'
  {   2815,  15,  14,  15,    0,  -14 },   // 0x6E 'n'
  {   2842,  15,  14,  15,    0,  -14 },   // 0x6F 'o'
  {   2869,  16,  20,  16,    0,  -14 },   // 0x70 'p'
  {   2909,  16,  20,  16,    0,  -14 },   // 0x71 'q'
  {   2949,  10,  14,  10,    0,  -14 },   // 0x72 'r'
  {   2967,  14,  15,  14,    0,  -14 },   // 0x73 's'
  {   2994,   9,  18,   9,    0,  -18 },   // 0x74 't'
  {   3015,  15,  14,  15,    0,  -14 },   // 0x75 'u'
  {   3042,  15,  14,  15,    0,  -14 },   // 0x76 'v'
  {   3069,  22,  14,  22,    0,  -14 },   // 0x77 'w'
  {   3108,  14,  14,  14,    0,  -14 },   // 0x78 'x'
  {   3133,  15,  20,  15,    0,  -14 },   // 0x79 'y'
  {   3171,  14,  14,  14,    0,  -14 },   // 0x7A 'z'
  {   3196,  11,  25,  11,    0,  -21 },   // 0x7B '{'
  {   3231,   9,  33,   9,    0,  -26 },   // 0x7C '|'
  {   3269,  11,  25,  11,    0,  -21 },   // 0x7D '}'
  {   3304,  17,  11,  17,    0,  -11 },   // 0x7E '~'
};
const GFXfont Label27 PROGMEM = { (uint8_t *)Label27Bitmaps, (GFXglyph *)Label27Glyphs, 0x20, 0x7E, 34 };
// Small13: generated from InterDisplay-SemiBold.otf at 13px by make_gfx_font.py
const uint8_t Small13Bitmaps[] PROGMEM = {
  0x49,0x24,0x90,0x48,0x59,0x65,0x80,0x00,0x00,0x00,0x00,0x00,0x32,0x32,0x36,0x7F,
  0x24,0x24,0xFE,0x6C,0x4C,0x4C,0x00,0x08,0x3C,0x7E,0x4A,0x68,0x78,0x1E,0x0B,0x4B,
  0x7E,0x3C,0x08,0x00,0x30,0x85,0x98,0x49,0x05,0xA0,0x32,0x00,0x40,0x0D,0xC0,0x92,
  0x11,0x63,0x1C,0x38,0x36,0x19,0x0D,0x83,0x83,0xC1,0x2D,0x8C,0x6E,0x1D,0x80,0x49,
  0x00,0x00,0x00,0x32,0x66,0x66,0x46,0x66,0x23,0xC6,0x66,0x22,0x22,0x66,0x44,0x10,
  0xA8,0xE3,0xC5,0x02,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x7E,0x18,0x18,0x00,0x09,
  0x60,0x01,0xE0,0x00,0x00,0x48,0x10,0x8C,0x62,0x11,0x88,0x42,0x30,0x00,0x3C,0x7E,
  0x62,0x43,0x43,0x43,0x43,0x62,0x76,0x3C,0x33,0xA4,0x21,0x08,0x42,0x10,0x80,0x3C,
  0x6E,0x46,0x06,0x06,0x0C,0x18,0x30,0x60,0x7E,0x3C,0x66,0x46,0x06,0x1C,0x0E,0x02,
  0x42,0x6E,0x3C,0x0E,0x0E,0x1E,0x36,0x26,0x66,0xC6,0xFF,0x06,0x06,0x7E,0x60,0x40,
  0x7C,0x6E,0x46,0x02,0x46,0x6E,0x3C,0x3C,0x7E,0x62,0x40,0x7C,0xE6,0x42,0x42,0x66,
  0x3C,0xFE,0x0C,0x30,0x60,0x83,0x04,0x18,0x20,0xC0,0x3C,0x6E,0x66,0x66,0x3C,0x6E,
  0x42,0x42,0x66,0x3C,0x3C,0x6E,0x46,0x42,0x66,0x7E,0x02,0x46,0x6C,0x38,0x48,0x00,
  0x90,0x48,0x00,0x96,0x00,0x02,0x0E,0x38,0x60,0x38,0x0E,0x02,0x7E,0x00,0x00,0x7E,
  0x00,0x00,0x00,0x60,0x78,0x0E,0x06,0x1E,0x70,0x40,0x38,0xD9,0x18,0x20,0xC3,0x04,
  0x00,0x10,0x20,0x00,0x00,0x7E,0x0C,0x18,0xC0,0x64,0xF9,0x24,0xC9,0x62,0x49,0x12,
  0x4D,0xF3,0x3B,0x08,0x00,0x79,0x00,0xF8,0x00,0x1C,0x0E,0x05,0x06,0xC3,0x61,0x11,
  0xFC,0xC6,0x41,0x60,0xC0,0x7C,0x66,0x43,0x46,0x7C,0x66,0x43,0x43,0x67,0x7E,0x1E,
  0x1F,0x98,0x68,0x04,0x02,0x01,0x00,0xC3,0x3F,0x0F,0x00,0x7C,0x37,0x10,0xC8,0x24,
  0x1A,0x0D,0x04,0x86,0x6E,0x3E,0x00,0x7E,0x60,0x40,0x40,0x7E,0x60,0x40,0x40,0x60,
  0x7E,0x7E,0x60,0x40,0x40,0x7E,0x60,0x40,0x40,0x40,0x40,0x1E,0x0F,0xC6,0x19,0x00,
  0x40,0x11,0xE4,0x19,0x86,0x3F,0x07,0x80,0x41,0x20,0x90,0x48,0x27,0xF3,0x19,0x04,
  0x82,0x41,0x20,0x80,0x49,0x24,0x92,0x48,0x04,0x08,0x10,0x20,0x40,0x91,0x62,0x7C,
  0x70,0x43,0x23,0x13,0x09,0x87,0x83,0xE1,0xB0,0x8C,0x43,0x21,0x80,0x40,0x81,0x02,
  0x04,0x08,0x10,0x20,0x60,0xFC,0x60,0xEE,0x1D,0xC7,0xB8,0xF5,0x96,0xB6,0xD2,0xDA,
  0x73,0x4E,0x68,0xCC,0x61,0xB0,0xDC,0x6F,0x35,0x9A,0x6D,0x1E,0x8F,0x43,0xA1,0xC0,
  0x1E,0x0F,0xC6,0x19,0x06,0x41,0x90,0x64,0x19,0x86,0x3F,0x07,0x80,0x7C,0x6E,0x43,
  0x43,0x46,0x7E,0x60,0x40,0x40,0x40,0x1E,0x0F,0xC6,0x19,0x06,0x41,0x90,0x64,0x19,
  0x9E,0x3F,0x07,0xC0,0x10,0x7C,0x33,0x10,0xC8,0x64,0x33,0xF1,0xB0,0x8C,0x46,0x21,
  0x80,0x3C,0x7E,0x42,0x60,0x78,0x1E,0x03,0x43,0x76,0x3C,0xFF,0x18,0x18,0x18,0x18,
  0x18,0x18,0x18,0x18,0x18,0x41,0x20,0x90,0x48,0x24,0x12,0x09,0x04,0xC6,0x7F,0x0F,
  0x00,0xC1,0xA0,0x98,0xCC,0x62,0x21,0xB0,0xD8,0x28,0x1C,0x0E,0x00,0xC6,0x1E,0x38,
  0x91,0xCC,0xCE,0x66,0xD3,0x36,0xD0,0xA7,0x87,0x1C,0x38,0xC1,0xC6,0x00,0x63,0x31,
  0x8D,0x83,0x81,0xC0,0xE0,0xF0,0x6C,0x63,0x61,0x80,0xC3,0x31,0x99,0x86,0xC1,0xC0,
  0xE0,0x20,0x10,0x08,0x04,0x00,0x7F,0x07,0x06,0x0C,0x18,0x18,0x30,0x60,0x60,0xFF,
  0x76,0x44,0x44,0x44,0x44,0x67,0xC6,0x10,0x86,0x10,0x84,0x30,0x84,0x00,0xE6,0x22,
  0x22,0x22,0x22,0x6E,0x00,0xC7,0x12,0x48,0x00,0x00,0x00,0x00,0xFC,0x00,0x31,0x64,
  0x96,0x30,0x00,0x00,0x00,0x00,0x38,0xD8,0x33,0xE4,0x49,0x9D,0x00,0x40,0x40,0x40,
  0x5C,0x66,0x66,0x42,0x66,0x66,0x5C,0x38,0xD9,0x16,0x04,0x4D,0x8E,0x00,0x06,0x06,
  0x06,0x3E,0x6E,0x46,0xC6,0x46,0x6E,0x3E,0x38,0xD9,0x1F,0xF4,0x0C,0x8E,0x00,0x33,
  0x19,0xE6,0x31,0x8C,0x63,0x00,0x3E,0x6E,0x46,0xC6,0x46,0x6E,0x3E,0x06,0x6E,0x3C,
  0x40,0x81,0x02,0xE7,0xC8,0xD1,0xA3,0x46,0x8C,0x48,0x24,0x92,0x48,0x22,0x02,0x22,
  0x22,0x22,0x26,0x60,0x40,0x40,0x40,0x4C,0x4C,0x78,0x70,0x78,0x4C,0x46,0x49,0x24,
  0x92,0x48,0x59,0xCD,0xD9,0x19,0xA2,0x34,0x46,0x88,0xD1,0x18,0x5C,0xF9,0x1A,0x34,
  0x68,0xD1,0x80,0x38,0xD9,0x1E,0x34,0x6D,0x8E,0x00,0x5C,0x66,0x66,0x42,0x66,0x66,
  0x5C,0x40,0x40,0x40,0x3E,0x6E,0x46,0xC6,0x46,0x6E,0x3E,0x06,0x06,0x00,0x73,0x90,
  0x84,0x21,0x00,0x78,0xD9,0x03,0xC0,0xC9,0x9E,0x00,0x63,0x3C,0xC6,0x31,0x8C,0x30,
  0x46,0x8D,0x1A,0x34,0x6D,0xCD,0x80,0xC6,0x89,0xB3,0x63,0x87,0x0E,0x00,0xCC,0xC9,
  0x99,0x3A,0x35,0x47,0xB8,0x67,0x0C,0xC0,0x44,0xD8,0xE1,0xC3,0x8D,0xB1,0x00,0xC6,
  0x89,0xB3,0x62,0x87,0x0E,0x08,0x30,0xC0,0x7C,0x18,0x61,0x82,0x0C,0x1F,0x00,0x19,
  0x8C,0x63,0x31,0x86,0x31,0x8C,0x30,0x06,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0xC3,
  0x08,0x43,0x0C,0xC4,0x21,0x19,0x80,0x00,0x32,0x7E,0x00,0x00,0x00,
  0x00 };
const GFXglyph Small13Glyphs[] PROGMEM = {
  {      0,   3,   0,   3,    0,    0 },   // 0x20 ' '
  {      0,   3,  10,   3,    0,  -10 },   // 0x21 '!'
  {      4,   6,  10,   6,    0,  -10 },   // 0x22 '"'
  {     12,   8,  10,   8,    0,  -10 },   // 0x23 '#'
  {     22,   8,  14,   8,    0,  -12 },   // 0x24 '$'
  {     36,  12,  10,  12,    0,  -10 },   // 0x25 '%'
  {     51,   9,  10,   8,    0,  -10 },   // 0x26 '&'
  {     63,   3,  10,   3,    0,  -10 },   // 0x27 "'"
  {     67,   4,  12,   4,    0,  -10 },   // 0x28 '('
  {     73,   4,  12,   4,    0,  -10 },   // 0x29 ')'
  {     79,   7,  10,   7,    0,  -10 },   // 0x2A '*'
  {     88,   8,   7,   8,    0,   -7 },   // 0x2B '+'
  {     95,   3,   4,   3,    0,   -2 },   // 0x2C ','
  {     97,   6,   5,   6,    0,   -5 },   // 0x2D '-'
  {    101,   3,   2,   3,    0,   -2 },   // 0x2E '.'
  {    102,   5,  12,   5,    0,  -10 },   // 0x2F '/'
  {    110,   8,  10,   8,    0,  -10 },   // 0x30 '0'
  {    120,   5,  10,   8,    1,  -10 },   // 0x31 '1'
  {    127,   8,  10,   8,    0,  -10 },   // 0x32 '2'
  {    137,   8,  10,   8,    0,  -10 },   // 0x33 '3'
  {    147,   8,  10,   8,    0,  -10 },   // 0x34 '4'
  {    157,   8,  10,   8,    0,  -10 },   // 0x35 '5'
  {    167,   8,  10,   8,    0,  -10 },   // 0x36 '6'
  {    177,   7,  10,   8,    0,  -10 },   // 0x37 '7'
  {    186,   8,  10,   8,    0,  -10 },   // 0x38 '8'
  {    196,   8,  10,   8,    0,  -10 },   // 0x39 '9'
  {    206,   3,   7,   3,    0,   -7 },   // 0x3A ':'
  {    209,   3,   8,   3,    0,   -6 },   // 0x3B ';'
  {    212,   8,   8,   8,    0,   -8 },   // 0x3C '<'
  {    220,   8,   6,   8,    0,   -6 },   // 0x3D '='
  {    226,   8,   8,   8,    0,   -8 },   // 0x3E '>'
  {    234,   7,  10,   7,    0,  -10 },   // 0x3F '?'
  {    243,  13,  13,  13,    0,  -10 },   // 0x40 '@'
  {    265,   9,  10,   9,    0,  -10 },   // 0x41 'A'
  {    277,   8,  10,   8,    0,  -10 },   // 0x42 'B'
  {    287,   9,  10,   9,    0,  -10 },   // 0x43 'C'
  {    299,   9,  10,   9,    0,  -10 },   // 0x44 'D'
  {    311,   8,  10,   8,    0,  -10 },   // 0x45 'E'
  {    321,   8,  10,   7,    0,  -10 },   // 0x46 'F'
  {    331,  10,  10,  10,    0,  -10 },   // 0x47 'G'
  {    344,   9,  10,   9,    0,  -10 },   // 0x48 'H'
  {    356,   3,  10,   3,    0,  -10 },   // 0x49 'I'
  {    360,   7,  10,   7,    0,  -10 },   // 0x4A 'J'
  {    369,   9,  10,   9,    0,  -10 },   // 0x4B 'K'
  {    381,   7,  10,   7,    0,  -10 },   // 0x4C 'L'
  {    390,  11,  10,  11,    0,  -10 },   // 0x4D 'M'
  {    404,   9,  10,   9,    0,  -10 },   // 0x4E 'N'
  {    416,  10,  10,  10,    0,  -10 },   // 0x4F 'O'
  {    429,   8,  10,   8,    0,  -10 },   // 0x50 'P'
  {    439,  10,  11,  10,    0,  -10 },   // 0x51 'Q'
  {    453,   9,  10,   8,    0,  -10 },   // 0x52 'R'
  {    465,   8,  10,   8,    0,  -10 },   // 0x53 'S'
  {    475,   8,  10,   8,    0,  -10 },   // 0x54 'T'
  {    485,   9,  10,   9,    0,  -10 },   // 0x55 'U'
  {    497,   9,  10,   9,    0,  -10 },   // 0x56 'V'
  {    509,  13,  10,  13,    0,  -10 },   // 0x57 'W'
  {    526,   9,  10,   9,    0,  -10 },   // 0x58 'X'
  {    538,   9,  10,   9,    0,  -10 },   // 0x59 'Y'
  {    550,   8,  10,   8,    0,  -10 },   // 0x5A 'Z'
  {    560,   4,  12,   4,    0,  -10 },   // 0x5B '['
  {    566,   5,  12,   5,    0,  -10 },   // 0x5C '\\\\'
  {    574,   4,  12,   4,    0,  -10 },   // 0x5D ']'
  {    580,   6,  10,   6,    0,  -10 },   // 0x5E '^'
  {    588,   7,   2,   6,    0,    0 },   // 0x5F '_'
  {    590,   6,  10,   6,    0,  -10 },   // 0x60 '`'
  {    598,   7,   7,   7,    0,   -7 },   // 0x61 'a'
  {    605,   8,  10,   8,    0,  -10 },   // 0x62 'b'
  {    615,   7,   7,   7,    0,   -7 },   // 0x63 'c'
  {    622,   8,  10,   8,    0,  -10 },   // 0x64 'd'
  {    632,   7,   7,   7,    0,   -7 },   // 0x65 'e'
  {    639,   5,  10,   5,    0,  -10 },   // 0x66 'f'
  {    646,   8,  10,   8,    0,   -7 },   // 0x67 'g'
  {    656,   7,  10,   7,    0,  -10 },   // 0x68 'h'
  {    665,   3,  10,   3,    0,  -10 },   // 0x69 'i'
  {    669,   4,  13,   3,   -1,  -10 },   // 0x6A 'j'
  {    676,   8,  10,   7,    0,  -10 },   // 0x6B 'k'
  {    686,   3,  10,   3,    0,  -10 },   // 0x6C 'l'
  {    690,  11,   7,  11,    0,   -7 },   // 0x6D 'm'
  {    700,   7,   7,   7,    0,   -7 },   // 0x6E 'n'
  {    707,   7,   7,   7,    0,   -7 },   // 0x6F 'o'
  {    714,   8,  10,   8,    0,   -7 },   // 0x70 'p'
  {    724,   8,  10,   8,    0,   -7 },   // 0x71 'q'
  {    734,   5,   7,   5,    0,   -7 },   // 0x72 'r'
  {    739,   7,   8,   7,    0,   -7 },   // 0x73 's'
  {    746,   5,   9,   5,    0,   -9 },   // 0x74 't'
  {    752,   7,   7,   7,    0,   -7 },   // 0x75 'u'
  {    759,   7,   7,   7,    0,   -7 },   // 0x76 'v'
  {    766,  11,   7,  10,    0,   -7 },   // 0x77 'w'
  {    776,   7,   7,   7,    0,   -7 },   // 0x78 'x'
  {    783,   7,  10,   7,    0,   -7 },   // 0x79 'y'
  {    792,   7,   7,   7,    0,   -7 },   // 0x7A 'z'
  {    799,   5,  12,   5,    0,  -10 },   // 0x7B '{'
  {    807,   4,  16,   4,    0,  -13 },   // 0x7C '|'
  {    815,   5,  12,   5,    0,  -10 },   // 0x7D '}'
  {    823,   8,   6,   8,    0,   -6 },   // 0x7E '~'
};
const GFXfont Small13 PROGMEM = { (uint8_t *)Small13Bitmaps, (GFXglyph *)Small13Glyphs, 0x20, 0x7E, 17 };


// =====================================================================
//  SETTINGS PAGE (HTML, CSS and script in one piece)
// =====================================================================
const char SETTINGS_PAGE[] PROGMEM = R"HTMLPAGE(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#21170f">
<title>Smart Clock</title>
<style>
/* Walnut-and-teak, like the clock itself. Cards on a warm ground, one teak accent. */
:root{
  --bg:#f2ebe2; --card:#fffaf4; --ink:#2a1d13; --mut:#7b6a5b; --line:#e6d9ca;
  --acc:#a8622e; --accInk:#fff8f0; --good:#3f7d4f; --warn:#b26a12; --sheet:#fffaf4;
  --shadow:0 1px 2px rgba(60,35,15,.08),0 6px 18px rgba(60,35,15,.06);
}
@media (prefers-color-scheme: dark){:root:not([data-theme=light]){
  --bg:#1a130e; --card:#261c15; --ink:#f3e7da; --mut:#b29e8b; --line:#3a2c22;
  --acc:#e0a46a; --accInk:#21170f; --good:#7cc08b; --warn:#e9a54a; --sheet:#2b2018;
  --shadow:0 1px 2px rgba(0,0,0,.3);color-scheme:dark}}
:root[data-theme=dark]{--bg:#1a130e;--card:#261c15;--ink:#f3e7da;--mut:#b29e8b;--line:#3a2c22;--acc:#e0a46a;--accInk:#21170f;--good:#7cc08b;--warn:#e9a54a;--sheet:#2b2018;--shadow:0 1px 2px rgba(0,0,0,.3);color-scheme:dark}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;background:var(--bg);color:var(--ink);font:17px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",sans-serif}
main{max-width:520px;margin:0 auto;padding:18px 16px calc(96px + env(safe-area-inset-bottom))}
header{padding:6px 2px 16px}
.hello{font-size:15px;color:var(--mut);margin:0}
.now{font-size:44px;font-weight:700;letter-spacing:-.5px;margin:2px 0 2px;font-variant-numeric:tabular-nums}
.facts{display:flex;flex-wrap:wrap;gap:6px 14px;color:var(--mut);font-size:15px}
.card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:16px;margin:0 0 14px;box-shadow:var(--shadow)}
.card h2{font-size:15px;font-weight:600;color:var(--mut);margin:0 0 12px;letter-spacing:.2px}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px}
.big{font-size:21px;font-weight:650}
.small{font-size:14px;color:var(--mut)}
.tab{display:none}.tab.on{display:block}
/* switch */
.sw{position:relative;width:58px;height:34px;flex:none}
.sw input{opacity:0;width:0;height:0}
.sw span{position:absolute;inset:0;border-radius:34px;background:var(--line);transition:.2s}
.sw span:before{content:"";position:absolute;width:28px;height:28px;left:3px;top:3px;border-radius:50%;background:#fff;box-shadow:0 1px 3px rgba(0,0,0,.3);transition:.2s}
.sw input:checked+span{background:var(--acc)}
.sw input:checked+span:before{transform:translateX(24px)}
.sw input:focus-visible+span{outline:3px solid var(--acc);outline-offset:2px}
/* style tiles */
.tiles{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.tile{border:2px solid transparent;background:var(--bg);border-radius:14px;padding:12px 6px 10px;text-align:center;font-size:13.5px;color:var(--ink);cursor:pointer;font:inherit;font-size:13.5px}
.tile.sel{border-color:var(--acc)}
.ring{width:54px;height:54px;border-radius:50%;margin:0 auto 8px;position:relative;
  -webkit-mask:radial-gradient(circle,transparent 18px,#000 19px);mask:radial-gradient(circle,transparent 18px,#000 19px)}
.r-glow{background:var(--c)}
.r-breathe{background:var(--c);animation:breathe 4s ease-in-out infinite}
.r-rainbow{background:conic-gradient(red,orange,yellow,lime,cyan,blue,magenta,red);animation:spin 6s linear infinite}
.r-markers{background:conic-gradient(from -8deg,var(--c) 0 16deg,color-mix(in srgb,var(--c) 15%,transparent) 16deg 112deg,#fff 112deg 124deg,color-mix(in srgb,var(--c) 15%,transparent) 124deg)}
.r-comet{background:conic-gradient(transparent 0 200deg,var(--c));animation:spin 3s linear infinite}
.r-ferris{background:conic-gradient(#ff3a2a 0 12deg,#3a1d0a 12deg 30deg,#ffbe14 30deg 42deg,#3a1d0a 42deg 60deg,#2878ff 60deg 72deg,#3a1d0a 72deg 90deg,#28dc5a 90deg 102deg,#3a1d0a 102deg 120deg,#ff46b4 120deg 132deg,#3a1d0a 132deg 150deg,#ff7814 150deg 162deg,#3a1d0a 162deg 180deg,#ff3a2a 180deg 192deg,#3a1d0a 192deg 210deg,#ffbe14 210deg 222deg,#3a1d0a 222deg 240deg,#2878ff 240deg 252deg,#3a1d0a 252deg 270deg,#28dc5a 270deg 282deg,#3a1d0a 282deg 300deg,#ff46b4 300deg 312deg,#3a1d0a 312deg 330deg,#ff7814 330deg 342deg,#3a1d0a 342deg);animation:spin 12s linear infinite}
.r-orbit{background:conic-gradient(color-mix(in srgb,var(--c) 10%,transparent) 0 250deg,var(--c) 340deg,#fff 352deg,color-mix(in srgb,var(--c) 10%,transparent) 356deg);animation:spin 60s steps(60) infinite}
.r-ocean{background:conic-gradient(#0a2f78,#13a0a0,#0a3c8c,#2ab4b4,#0a2f78,#1890a8,#0a2f78);animation:spin 9s linear infinite}
.r-aurora{background:conic-gradient(#20e070,#18b4a0,#7a4ad0,#28d080,#1aa8b0,#9a50d8,#20e070);animation:spin 14s linear infinite}
.r-candle{background:radial-gradient(circle at 30% 30%,#ffb040,transparent 60%),conic-gradient(#ff6a10,#ff9a2a,#e25008,#ff8a1c,#ff6a10);animation:twinkle .7s steps(2) infinite}
.r-weather{background:conic-gradient(#3c6eff,#78c8ff,#ffbe3c,#ff6e14,#ffbe3c,#78c8ff,#3c6eff)}
.r-twinkle{background:radial-gradient(circle at 20% 30%,#fff 0 3px,transparent 4px),radial-gradient(circle at 75% 20%,var(--c) 0 3px,transparent 4px),radial-gradient(circle at 80% 75%,#fff 0 3px,transparent 4px),radial-gradient(circle at 25% 80%,var(--c) 0 3px,transparent 4px),color-mix(in srgb,var(--c) 20%,transparent);animation:twinkle 1.6s steps(2) infinite}
@keyframes spin{to{transform:rotate(360deg)}}
@keyframes breathe{50%{opacity:.25}}
@keyframes twinkle{50%{filter:brightness(.5)}}
@media (prefers-reduced-motion:reduce){.ring{animation:none!important}}
/* colours */
.swatches{display:flex;flex-wrap:wrap;gap:12px}
.dot{width:44px;height:44px;border-radius:50%;border:3px solid var(--card);box-shadow:0 0 0 2px var(--line);cursor:pointer;padding:0;position:relative}
.dot.sel{box-shadow:0 0 0 3px var(--acc)}
.dot.custom{background:conic-gradient(red,yellow,lime,cyan,blue,magenta,red);overflow:hidden}
.dot.custom input{position:absolute;inset:0;opacity:0;width:100%;height:100%;cursor:pointer}
/* slider */
input[type=range]{width:100%;accent-color:var(--acc);height:34px}
.sun{display:flex;align-items:center;gap:10px}.sun svg{flex:none;color:var(--mut)}
select,input[type=text],input[type=number],input[type=time],input[type=date]{font:inherit;font-size:16px;padding:10px 12px;border-radius:12px;border:1px solid var(--line);background:var(--bg);color:var(--ink);max-width:100%}
.btn{font:inherit;font-size:16px;font-weight:600;padding:13px 16px;border-radius:14px;border:0;background:var(--acc);color:var(--accInk);cursor:pointer;width:100%}
.btn.ghost{background:transparent;color:var(--acc);border:1.5px solid var(--acc)}
.btns{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
button:focus-visible,select:focus-visible,input:focus-visible{outline:3px solid var(--acc);outline-offset:2px}
.seg{display:inline-flex;background:var(--bg);border-radius:12px;padding:3px;border:1px solid var(--line)}
.seg button{font:inherit;font-size:15px;border:0;background:transparent;color:var(--mut);padding:8px 16px;border-radius:9px;cursor:pointer}
.seg button.sel{background:var(--card);color:var(--ink);box-shadow:var(--shadow);font-weight:600}
/* gestures */
.glist{list-style:none;margin:0;padding:0}
.glist li{display:flex;align-items:center;gap:12px;padding:12px 0;border-top:1px solid var(--line);cursor:pointer}
.glist li:first-child{border-top:0}
.gico{width:40px;height:40px;border-radius:12px;background:var(--bg);display:grid;place-items:center;flex:none;color:var(--acc)}
.gname{flex:1;min-width:0}
.pill{font-size:14px;padding:6px 10px;border-radius:20px;background:var(--bg);color:var(--ink);white-space:nowrap;border:1px solid var(--line)}
.chev{color:var(--mut)}
.seen{display:flex;align-items:center;gap:10px;padding:10px 12px;border-radius:12px;background:var(--bg);font-size:15px;margin-top:4px}
.seen b{color:var(--acc)}
.flash{animation:flash .9s ease}
@keyframes flash{0%{background:var(--acc);color:var(--accInk)}}
/* status chip */
.chip{display:inline-flex;align-items:center;gap:6px;font-size:14px;font-weight:600;padding:5px 10px;border-radius:20px;background:color-mix(in srgb,var(--good) 15%,transparent);color:var(--good)}
.chip.warn{background:color-mix(in srgb,var(--warn) 15%,transparent);color:var(--warn)}
.chip:before{content:"";width:8px;height:8px;border-radius:50%;background:currentColor}
/* bottom tabs */
nav{position:fixed;left:0;right:0;bottom:0;background:var(--card);border-top:1px solid var(--line);padding:6px 8px calc(6px + env(safe-area-inset-bottom));display:flex;justify-content:center;z-index:5}
nav div{display:grid;grid-template-columns:repeat(4,1fr);width:100%;max-width:520px}
nav button{font:inherit;font-size:12px;border:0;background:none;color:var(--mut);display:flex;flex-direction:column;align-items:center;gap:3px;padding:6px 0;cursor:pointer}
nav button.on{color:var(--acc);font-weight:600}
fieldset{border:0;padding:0;margin:0;min-width:0}
fieldset[disabled] .card{opacity:.55}
fieldset[disabled] .card .small{opacity:1}
.lockcard .row{display:flex;align-items:center;justify-content:space-between;gap:12px}
.msgs{list-style:none;margin:10px 0 0;padding:0}
.msgs li{padding:10px 0;border-top:1px solid var(--line)}
.msgs li:first-child{border-top:0}
.msgs li.new>div:first-child{font-weight:600}
.msgs li.new>div:first-child::before{content:"";display:inline-block;width:8px;height:8px;border-radius:50%;background:var(--acc);margin-right:8px;vertical-align:middle}
.remrow{display:flex;justify-content:space-between;align-items:center;gap:10px}
.rembtns{display:flex;gap:6px;flex-shrink:0}
.btn.sm{width:auto;padding:6px 10px;font-size:14px}
.addrem{margin-top:12px}
.addrem summary{list-style:none;text-align:center;cursor:pointer}
.addrem summary::-webkit-details-marker{display:none}
.addrem label{display:block;font-size:14px;color:var(--mut);margin:12px 0 6px}
.addrem input[type=text],.addrem input[type=time],.addrem input[type=date],.addrem select{width:100%}
.addrem .two{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.days{display:flex;gap:6px;flex-wrap:wrap}
.day{display:flex!important;align-items:center;gap:4px;margin:0!important;padding:6px 8px;border:1px solid var(--line);border-radius:8px;color:var(--ink)!important}
/* bottom sheet */
.shade{position:fixed;inset:0;background:rgba(20,10,4,.45);display:none;z-index:9}
.shade.on{display:block}
.sheet{position:fixed;left:0;right:0;bottom:0;max-width:520px;margin:0 auto;background:var(--sheet);border-radius:22px 22px 0 0;padding:10px 18px calc(20px + env(safe-area-inset-bottom));transform:translateY(105%);visibility:hidden;transition:transform .25s ease,visibility 0s .25s;z-index:10;max-height:88vh;overflow:auto}
.sheet.on{transform:none;visibility:visible;transition:transform .25s ease}
.grab{width:40px;height:5px;border-radius:3px;background:var(--line);margin:4px auto 12px}
.sheet h3{margin:0 0 4px;font-size:20px}
.opts{list-style:none;margin:10px 0 0;padding:0}
.opts li{padding:14px 12px;border-radius:12px;cursor:pointer;display:flex;justify-content:space-between;align-items:center}
.opts li.sel{background:var(--bg);font-weight:600}
.opts li.sel:after{content:"✓";color:var(--acc);font-weight:700}
.steps{counter-reset:s;list-style:none;padding:0;margin:12px 0 0}
.steps li{counter-increment:s;padding:0 0 16px 40px;position:relative}
.steps li:before{content:counter(s);position:absolute;left:0;top:0;width:28px;height:28px;border-radius:50%;background:var(--acc);color:var(--accInk);display:grid;place-items:center;font-weight:700;font-size:15px}
.timepick{display:flex;align-items:center;gap:6px;font-size:22px;font-weight:600;margin:6px 0}
.timepick select{font-size:20px;padding:8px 6px}
.toast{position:fixed;left:50%;bottom:calc(84px + env(safe-area-inset-bottom));transform:translate(-50%,20px);background:var(--ink);color:var(--bg);padding:10px 16px;border-radius:24px;font-size:15px;opacity:0;transition:.25s;z-index:20;pointer-events:none;max-width:90vw;text-align:center}
.toast.on{opacity:1;transform:translate(-50%,0)}
.warnbox{font-size:14px;color:var(--mut);background:var(--bg);border-radius:12px;padding:10px 12px;margin-bottom:12px}
.adv label{display:block;font-size:14px;color:var(--mut);margin:12px 0 6px}
.adv .two{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.adv .two>*{min-width:0}
.adv select,.adv input{width:100%}
.chk{display:flex;align-items:center;gap:10px;margin:8px 0;font-size:15px}
.chk input{width:22px;height:22px;accent-color:var(--acc)}
.hide{display:none!important}
</style>
</head>
<body>
<main>
  <header>
    <p class="hello" id="hello">Hello</p>
    <div class="now" id="now">--:--</div>
    <div class="facts"><span id="f_room">Inside --</span><span id="f_out"></span></div>
  </header>

  <!-- ============ LIGHT ============ -->
  <section class="tab on" id="t_light">
    <div class="card">
      <div class="row">
        <div><div class="big">Ring light</div><div class="small" id="lightNote">Off</div></div>
        <label class="sw"><input type="checkbox" id="on" aria-label="Ring light"><span></span></label>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <div><div class="big" style="font-size:18px">Breathing guide</div><div class="small">The ring rises and falls slowly: breathe in, hold, breathe out. About a minute and a half. Wave to stop.</div></div>
        <button class="btn" style="width:auto" onclick="act('breathe',null,'Breathing guide starting')">Start</button>
      </div>
    </div>

    <div class="card">
      <h2>Style</h2>
      <div class="tiles" id="tiles"></div>
    </div>

    <div class="card">
      <h2>Colour</h2>
      <div class="swatches" id="swatches"></div>
    </div>

    <div class="card">
      <h2>Brightness</h2>
      <div class="sun">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="4"/></svg>
        <input type="range" id="bright" min="10" max="255" aria-label="Brightness">
        <svg width="26" height="26" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="4"/><path d="M12 1v3M12 20v3M1 12h3M20 12h3M4.2 4.2l2.1 2.1M17.7 17.7l2.1 2.1M4.2 19.8l2.1-2.1M17.7 6.3l2.1-2.1"/></svg>
      </div>
    </div>

    <div class="card">
      <div class="row"><div class="big" style="font-size:18px">Turns itself off after</div>
        <select id="onsec" aria-label="Turns itself off after">
          <option value="10">10 seconds</option><option value="30">30 seconds</option><option value="60">1 minute</option>
          <option value="300">5 minutes</option><option value="900">15 minutes</option><option value="3600">1 hour</option>
        </select></div>
    </div>

    <div class="card">
      <div class="row">
        <div><div class="big" style="font-size:18px">Hour chime</div><div class="small">A sparkle runs round the ring on the hour, while the light is on</div></div>
        <label class="sw"><input type="checkbox" id="chime" aria-label="Hour chime"><span></span></label>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <div><div class="big" style="font-size:18px">Quiet at night</div><div class="small">The light stays off while you sleep</div></div>
        <label class="sw"><input type="checkbox" id="night" aria-label="Quiet at night"><span></span></label>
      </div>
      <div class="row" id="nightTimes" style="margin-top:12px;justify-content:flex-start;gap:8px;flex-wrap:wrap">
        <span class="small">From</span><select id="nfrom" aria-label="Quiet from"></select>
        <span class="small">to</span><select id="nto" aria-label="Quiet until"></select>
      </div>
    </div>
  </section>

  <!-- ============ GESTURES ============ -->
  <section class="tab" id="t_gest">
    <div class="card">
      <div class="big">Wave to control</div>
      <div class="small" style="margin-top:4px" id="gIntro">Bring your hand toward the little window just below the clock hands, a few centimetres away, then take it away again.</div>
      <div class="small hide" id="gBlock" style="margin-top:8px;color:var(--warn)">A clock hand is covering the sensor window right now, so waves can't be seen. It will work again once the hand moves on.</div>
      <div class="small hide" id="gLearn" style="margin-top:8px;color:var(--warn)">Just started: the clock is learning where its hands pass the sensor. Gestures work again in about a minute.</div>
      <div class="seen" id="seenBox"><div><div class="small">Last thing the clock saw</div><b id="seen">Nothing yet</b></div></div>
    </div>

    <div class="card">
      <h2>What each gesture does <span style="font-weight:400">(tap to change)</span></h2>
      <ul class="glist" id="glist"></ul>
    </div>

    <div class="card">
      <div class="row">
        <div><div class="big" style="font-size:18px">Any gesture turns the light on</div><div class="small">When the light is off, the first gesture just switches it on</div></div>
        <label class="sw"><input type="checkbox" id="wake" aria-label="Any gesture turns the light on"><span></span></label>
      </div>
    </div>
  </section>

  <!-- ============ CLOCK ============ -->
  <section class="tab" id="t_clock">
    <div class="card" id="msgCard">
      <h2>Messages</h2>
      <div class="small" id="msgEmpty">No messages yet. When one arrives it shows on the clock's screen and the ring glows a soft heartbeat.</div>
      <ul class="msgs" id="msgList"></ul>
      <div class="btns hide" id="msgBtns">
        <button class="btn" onclick="act('msgheart',null,'Heart sent')">Send a heart back</button>
        <button class="btn ghost" onclick="act('msgseen',null,'Marked as seen')">Mark all as seen</button>
      </div>
      <label class="chk" style="margin-top:12px"><input type="checkbox" id="hbglow"> Heartbeat glow for new messages</label>
      <div class="small" style="margin-top:6px">On the clock: tap to send a heart back, tap twice to close the message.</div>
    </div>

    <div class="card">
      <h2>Reminders</h2>
      <div class="small" id="remEmpty">No reminders yet. When one is due, the screen shows it and the ring glows gold. Wave to finish it, or tap twice to snooze 10 minutes.</div>
      <ul class="msgs" id="remList"></ul>
      <details class="addrem" id="remAdd">
        <summary class="btn ghost">Add a reminder</summary>
        <label for="remLabel">What to remember</label>
        <input type="text" id="remLabel" maxlength="40" placeholder="Water the plants">
        <div class="two">
          <div><label for="remTime">Time</label><input type="time" id="remTime" value="08:00"></div>
          <div><label for="remRep">Repeat</label><select id="remRep"><option value="0">Once</option><option value="127">Every day</option><option value="62">Weekdays</option><option value="65">Weekends</option><option value="pick">Pick days</option></select></div>
        </div>
        <div id="remDateBox"><label for="remDate">Date</label><input type="date" id="remDate"></div>
        <div id="remDaysBox" class="hide"><label>Days</label><div class="days" id="remDays"></div></div>
        <button class="btn" style="margin-top:12px" onclick="addRem()">Save reminder</button>
      </details>
    </div>

    <div class="card">
      <h2>Clock hands</h2>
      <div class="row"><div class="big" id="handsShow">--:--:--</div><span class="chip" id="handsChip">On time</span></div>
      <div class="small" id="handsNote" style="margin-top:6px"></div>
      <button class="btn" style="margin-top:14px" onclick="openSet()">Hands showing the wrong time?</button>
      <div class="small" style="margin-top:14px">Second hand a tiny bit off?</div>
      <div class="btns">
        <button class="btn ghost" onclick="act('nudge',-16,'Hands moved 1 second forward')">It's 1 s behind</button>
        <button class="btn ghost" onclick="act('nudge',16,'Hands will wait 1 second')">It's 1 s ahead</button>
      </div>
    </div>

    <div class="card">
      <h2>Screen</h2>
      <div class="row"><span>Temperature</span>
        <div class="seg" id="units"><button data-v="F">°F</button><button data-v="C">°C</button></div></div>
      <div class="btns">
        <button class="btn ghost" onclick="act('weather',null,'Weather on the screen for 20 s')">Show weather</button>
      </div>
      <label class="chk" style="margin-top:12px"><input type="checkbox" id="rainwarn"> Rain warning: show on the screen when rain is likely in the next few hours</label>
      <div class="small" id="rainNow"></div>
    </div>

    <div class="card">
      <h2>Weather location</h2>
      <div class="row"><input type="text" id="zip" inputmode="numeric" maxlength="5" placeholder="Zip code" aria-label="Zip code" style="flex:1;min-width:0">
        <button class="btn" style="width:auto" onclick="setv('zip',document.getElementById('zip').value)">Save</button></div>
      <div class="small" id="place" style="margin-top:8px"></div>
    </div>
  </section>

  <!-- ============ MORE (setup) ============ -->
  <section class="tab adv" id="t_more">
    <div class="warnbox">These are setup settings. They were set when the clock was built, so you shouldn't need to change them.</div>
    <div class="card lockcard">
      <div class="row"><span id="lockText">🔒 Locked, so nothing changes by accident</span>
        <button class="btn" id="lockBtn" style="width:auto" onclick="toggleLock()">Unlock</button></div>
    </div>
    <fieldset id="advlock" disabled>

    <div class="card">
      <h2>Light ring setup</h2>
      <div class="two">
        <div><label for="nled">Number of LEDs</label><input type="number" id="nled" min="1" max="300"></div>
        <div><label for="maxma">Power limit</label><select id="maxma"><option value="250">Low</option><option value="350">Medium</option><option value="400">Normal</option><option value="500">Bright</option></select></div>
        <div><label for="lstart">First LED is at</label><select id="lstart"></select></div>
        <div><label for="ldir">Strip runs</label><select id="ldir"><option value="1">Clockwise</option><option value="0">Anticlockwise</option></select></div>
      </div>
      <button class="btn ghost" style="margin-top:12px" onclick="act('locate',null,'First LED is red, the next five green')">Light up the first LED</button>
      <label>Styles that "Next style" goes through</label>
      <div id="emask"></div>
    </div>

    <div class="card">
      <h2>Gesture sensor</h2>
      <div class="two">
        <div><label for="sens">Sensitivity</label><select id="sens"><option value="1">Low (hand very close)</option><option value="2">Medium</option><option value="3">High (from farther away)</option></select></div>
        <div id="srotBox"><label for="srot">Sensor turned</label><select id="srot"><option value="0">Not turned</option><option value="1">90°</option><option value="2">180°</option><option value="3">270°</option></select></div>
      </div>
      <label class="chk" id="smirBox"><input type="checkbox" id="smir"> Left and right are swapped</label>
      <div class="small">Sensor: <b id="sensor">-</b></div>
      <div class="small">Hand distance right now: <b id="prox">-</b> (with no hand: about <span id="proxBase">-</span>)</div>
      <div class="small">Room light: <b id="amb">-</b></div>
    </div>

    <div class="card">
      <h2>Hand motor</h2>
      <div class="two">
        <div><label for="fast">Catch-up speed</label><select id="fast"><option value="62">Off</option><option value="56">56 ms</option><option value="50">50 ms (tested)</option><option value="45">45 ms</option><option value="40">40 ms</option></select></div>
        <div><label for="pulse">Pulse (ms)</label><input type="number" id="pulse" min="3" max="30"></div>
      </div>
      <div class="btns"><button class="btn ghost" onclick="act('hold',null)">Pause / run hands</button><button class="btn ghost" onclick="act('speedtest',null,'Speed test running')">Speed test</button></div>
    </div>

    <div class="card">
      <h2>WiFi</h2>
      <div class="small" id="net"></div>
      <button class="btn ghost" style="margin-top:10px" onclick="act('qr',null,'QR code on the clock screen for 1 minute')">Show the settings QR code on the clock</button>
      <button class="btn ghost" style="margin-top:10px;color:var(--warn);border-color:var(--warn)" onclick="forget()">Forget WiFi and set up again</button>
    </div>

    <div class="card">
      <h2>Software and updates</h2>
      <div class="small">Software <b id="ver">-</b> · running for <span id="uptime">-</span></div>
      <div class="small" style="margin-top:4px">Updates: <b id="upd">-</b></div>
      <label class="chk" style="margin-top:10px"><input type="checkbox" id="autoupd"> Install new software automatically (at night, around 3 AM)</label>
      <div class="btns">
        <button class="btn ghost" onclick="act('update',null,'Checking for an update...')">Check for updates now</button>
        <button class="btn ghost" onclick="restartClock()">Restart the clock</button>
      </div>
      <label for="otaurl">Update address</label>
      <div class="row"><input type="text" id="otaurl" placeholder="https://yourname.github.io/smart-clock" style="flex:1;min-width:0" autocapitalize="off" autocorrect="off" spellcheck="false">
        <button class="btn" style="width:auto" onclick="setv('otaurl',$('otaurl').value)">Save</button></div>
    </div>

    <div class="card">
      <h2>Health reports (ntfy)</h2>
      <div class="small">The clock sends a quiet "I'm OK" every 6 hours, and a message after updates, restarts or problems, to the free ntfy app. Subscribe to this topic in the app:</div>
      <div style="margin:8px 0;font-weight:600;word-break:break-all" id="ntfyShow">-</div>
      <div class="small">To send a message to the clock's screen, publish to the same name with <b>-msg</b> on the end. To control it, send <b>restart</b>, <b>update</b> or <b>status</b> to the name with <b>-cmd</b> on the end.</div>
      <label class="chk" style="margin-top:10px"><input type="checkbox" id="health"> Send health reports</label>
      <div class="btns">
        <a class="btn ghost" id="ntfyLink" href="#" target="_blank" rel="noopener" style="display:flex;align-items:center;justify-content:center;text-decoration:none">Open in ntfy</a>
        <button class="btn ghost" onclick="act('testmsg',null,'Test message sent')">Send a test message</button>
      </div>
      <label for="ntfy">Topic name</label>
      <div class="row"><input type="text" id="ntfy" style="flex:1;min-width:0" autocapitalize="off" autocorrect="off" spellcheck="false">
        <button class="btn" style="width:auto" onclick="setv('ntfy',$('ntfy').value)">Save</button></div>
    </div>

    <div class="card">
      <h2>Screen direction</h2>
      <div class="small">Only needed if the little screen on the clock is sideways or upside down.</div>
      <button class="btn ghost" style="margin-top:10px" onclick="turnScreen()">Turn screen a quarter turn ↻</button>
    </div>
    </fieldset>
  </section>
</main>

<nav><div>
  <button class="on" data-t="t_light"><svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/></svg>Light</button>
  <button data-t="t_gest"><svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M8 13V5a1.5 1.5 0 0 1 3 0v6M11 11V4a1.5 1.5 0 0 1 3 0v7M14 11V6a1.5 1.5 0 0 1 3 0v8a6 6 0 0 1-6 6h-1a5 5 0 0 1-4-2l-3-4a1.6 1.6 0 0 1 2.4-2L8 15"/></svg>Gestures</button>
  <button data-t="t_clock"><svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/></svg>Clock</button>
  <button data-t="t_more"><svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="5" cy="12" r="1.5"/><circle cx="12" cy="12" r="1.5"/><circle cx="19" cy="12" r="1.5"/></svg>Setup</button>
</div></nav>

<!-- gesture action picker -->
<div class="shade" id="shade" onclick="closeSheets()"></div>
<div class="sheet" id="gsheet" role="dialog" aria-modal="true" aria-labelledby="gtitle">
  <div class="grab"></div><h3 id="gtitle">Swipe left</h3><div class="small">Choose what happens</div>
  <ul class="opts" id="gopts"></ul>
</div>
<!-- set the hands -->
<div class="sheet" id="hsheet" role="dialog" aria-modal="true" aria-labelledby="htitle">
  <div class="grab"></div><h3 id="htitle">Set the hands</h3>
  <ol class="steps">
    <li><b>Pause the hands</b><div class="small">So they stay still while you read them.</div>
      <button class="btn ghost" style="margin-top:8px" id="pauseBtn" onclick="pauseHands()">Pause hands</button></li>
    <li><b>What time do the hands show?</b><div class="small">Hour, minute, and where the red second hand points.</div>
      <div class="timepick"><select id="ph" aria-label="Hour"></select>:<select id="pm" aria-label="Minute"></select>:<select id="ps" aria-label="Second"></select></div></li>
    <li><b>Done</b><div class="small">The clock moves the hands to the right time. A few minutes off takes a few minutes to catch up.</div>
      <button class="btn" style="margin-top:8px" onclick="setHands()">Set and start</button></li>
  </ol>
</div>
<div class="toast" id="toast"></div>

<script>
const EFFECTS=[["Warm glow","r-glow"],["Breathing","r-breathe"],["Rainbow","r-rainbow"],["Hour markers","r-markers"],["Comet","r-comet"],["Twinkle","r-twinkle"],["Ferris wheel","r-ferris"],["Second-hand orbit","r-orbit"],["Ocean","r-ocean"],["Aurora","r-aurora"],["Candle","r-candle"],["Weather colours","r-weather"]];
const ACTIONS=["Nothing","Light on / off","Next style","Previous style","Brighter","Dimmer","Show weather","Show QR code","Breathing guide"];
const GESTURES=[["Swipe left","M19 12H5M11 6l-6 6 6 6"],["Swipe right","M5 12h14M13 6l6 6-6 6"],["Swipe up","M12 19V5M6 11l6-6 6 6"],["Swipe down","M12 5v14M6 13l6 6 6-6"],
 ["Tap","M12 4v10M8 10l4 4 4-4M6 20h12"],["Hold 2 seconds","M12 7v5l3 2M12 3a9 9 0 1 0 0.01 0"],["Hold 5 seconds","M4 4h6v6H4zM14 4h6v6h-6zM4 14h6v6H4zM14 14h3M20 14v3M14 20h6"],["Double tap","M8 4v8M5 9l3 3 3-3M16 4v8M13 9l3 3 3-3M5 18h14"]];
const COLOURS=[["Amber","#ff8c3c"],["Candle","#ffb060"],["Soft white","#ffe2c0"],["Rose","#ff5d73"],["Ocean","#3fa0ff"],["Forest","#3fbf6a"]];
let S={},L={},busy=0,gIndex=0;
const $=id=>document.getElementById(id);
function toast(t){if(!t)return;const e=$('toast');e.textContent=t;e.classList.add('on');clearTimeout(e._t);e._t=setTimeout(()=>e.classList.remove('on'),2200)}
function hour12(h){return (h%12||12)+(h<12?' AM':' PM')}
function setv(k,v,msg){S[k]=v;busy=Date.now();fetch('/set?k='+k+'&v='+encodeURIComponent(v)).then(r=>r.text()).then(t=>{toast(msg||t||'Saved');refresh()}).catch(()=>toast('Could not reach the clock'))}
// reminders
(function(){const n=['S','M','T','W','T','F','S'];$('remDays').innerHTML=n.map((d,i)=>`<label class="day"><input type="checkbox" data-b="${i}">${d}</label>`).join('');
 const t=new Date();t.setDate(t.getDate()+1);$('remDate').value=t.toISOString().slice(0,10);
 $('remRep').onchange=()=>{const v=$('remRep').value;$('remDateBox').classList.toggle('hide',v!=='0');$('remDaysBox').classList.toggle('hide',v!=='pick')};})();
function addRem(){let d=$('remRep').value;if(d==='pick'){d=0;document.querySelectorAll('#remDays input').forEach(c=>{if(c.checked)d|=1<<c.dataset.b});if(!d){toast('Pick at least one day');return}}
 const q='op=add&label='+encodeURIComponent($('remLabel').value)+'&time='+encodeURIComponent($('remTime').value)+'&days='+d+'&date='+encodeURIComponent($('remDate').value);
 fetch('/rem?'+q).then(r=>r.text()).then(t=>{toast(t);$('remLabel').value='';$('remAdd').open=false;setTimeout(refresh,300)}).catch(()=>toast('Could not reach the clock'))}
function remOp(op,i){if(op==='del'&&!confirm('Delete this reminder?'))return;fetch('/rem?op='+op+'&i='+i).then(r=>r.text()).then(t=>{toast(t);setTimeout(refresh,300)})}
function esc(t){return String(t).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function act(a,v,msg){fetch('/do?a='+a+(v!=null?'&v='+encodeURIComponent(v):'')).then(r=>r.text()).then(t=>{toast(msg||t);setTimeout(refresh,300)}).catch(()=>toast('Could not reach the clock'))}

// build static pieces
$('tiles').innerHTML=EFFECTS.map((e,i)=>`<button class="tile" data-i="${i}"><div class="ring ${e[1]}"></div>${e[0]}</button>`).join('');
$('tiles').onclick=e=>{const t=e.target.closest('.tile');if(t){setv('effect',+t.dataset.i,EFFECTS[t.dataset.i][0]);if(!S.on)setv('on',1,EFFECTS[t.dataset.i][0]);paint()}};
$('swatches').innerHTML=COLOURS.map(c=>`<button class="dot" style="background:${c[1]}" data-c="${c[1]}" aria-label="${c[0]}"></button>`).join('')+`<label class="dot custom" aria-label="Pick any colour"><input type="color" id="custom"></label>`;
$('swatches').onclick=e=>{const d=e.target.closest('.dot');if(d&&d.dataset.c){setv('color',d.dataset.c,'Colour changed');paint()}};
$('custom').onchange=e=>{setv('color',e.target.value,'Colour changed');paint()};
const ORDER=[4,7,5,6,0,1,2,3];
$('glist').innerHTML=ORDER.map(i=>[GESTURES[i],i]).map(([g,i])=>`<li data-i="${i}"><span class="gico"><svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="${g[1]}"/></svg></span><span class="gname">${g[0]}</span><span class="pill" id="gp${i}"></span><span class="chev">›</span></li>`).join('');
$('glist').onclick=e=>{const li=e.target.closest('li');if(li)openGesture(+li.dataset.i)};
for(let h=0;h<24;h++){$('nfrom').add(new Option(hour12(h),h));$('nto').add(new Option(hour12(h),h))}
for(let h=1;h<=12;h++)$('lstart').add(new Option(h+" o'clock",h%12));
for(let h=1;h<=12;h++)$('ph').add(new Option(h,h));
for(let m=0;m<60;m++){$('pm').add(new Option(String(m).padStart(2,'0'),m));$('ps').add(new Option(String(m).padStart(2,'0'),m))}
$('emask').innerHTML=EFFECTS.map((e,i)=>`<label class="chk"><input type="checkbox" data-b="${i}"> ${e[0]}</label>`).join('');
$('emask').onchange=()=>{let m=0;document.querySelectorAll('#emask input').forEach(c=>{if(c.checked)m|=1<<c.dataset.b});setv('em',m||1)};

// simple controls: change -> save
$('on').onchange=e=>setv('on',e.target.checked?1:0,e.target.checked?'Light on':'Light off');
let bt;$('bright').oninput=e=>{clearTimeout(bt);bt=setTimeout(()=>setv('bright',e.target.value,'Brightness set'),250)};
['onsec','nfrom','nto','nled','maxma','lstart','ldir','sens','srot','fast','pulse'].forEach(k=>$(k).onchange=e=>setv(k,e.target.value));
['night','wake','smir','chime','autoupd','health','hbglow','rainwarn'].forEach(k=>$(k).onchange=e=>{setv(k,e.target.checked?1:0);paint()});
$('units').onclick=e=>{const b=e.target.closest('button');if(b)setv('units',b.dataset.v)};

// tabs
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>{document.querySelectorAll('nav button').forEach(x=>x.classList.toggle('on',x===b));document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('on',t.id===b.dataset.t));scrollTo(0,0);try{localStorage.tab=b.dataset.t}catch(e){}});
try{const t=localStorage.tab;if(t)document.querySelector(`nav button[data-t=${t}]`).click()}catch(e){}

// setup lock: unlocks for 5 minutes, locks again when you leave the tab
let lockT=0;
function setLock(on){$('advlock').disabled=on;$('lockText').textContent=on?'🔒 Locked, so nothing changes by accident':'🔓 Unlocked for 5 minutes';$('lockBtn').textContent=on?'Unlock':'Lock';clearTimeout(lockT);if(!on)lockT=setTimeout(()=>setLock(true),300000)}
function toggleLock(){setLock(!$('advlock').disabled)}
document.querySelectorAll('nav button').forEach(b=>b.addEventListener('click',()=>{if(b.dataset.t!=='t_more')setLock(true)}));
function restartClock(){if(confirm('Restart the clock now?\n\nIt takes about 30 seconds. The hands stay in place and catch up afterwards.'))act('restart',null,'Restarting - back in about 30 seconds')}
function turnScreen(){if(confirm('Turn the clock screen a quarter turn?\n\nOnly do this if the screen on the clock is sideways or upside down. You can press it again to keep turning.'))act('rotate',null,'Screen turned')}
// sheets
function closeSheets(){document.querySelectorAll('.sheet,.shade').forEach(x=>x.classList.remove('on'))}
function openGesture(i){gIndex=i;$('gtitle').textContent=GESTURES[i][0];
  $('gopts').innerHTML=ACTIONS.map((a,j)=>`<li data-j="${j}" class="${S.g&&S.g[i]==j?'sel':''}">${a}</li>`).join('');
  $('shade').classList.add('on');$('gsheet').classList.add('on')}
$('gopts').onclick=e=>{const li=e.target.closest('li');if(!li)return;S.g[gIndex]=+li.dataset.j;setv('g'+gIndex,li.dataset.j,GESTURES[gIndex][0]+': '+ACTIONS[li.dataset.j]);paint();setTimeout(closeSheets,200)};
function openSet(){$('pauseBtn').textContent='Pause hands';$('shade').classList.add('on');$('hsheet').classList.add('on');
  const p=(L.handsShow||'12:00:00').split(':');$('ph').value=+p[0]||12;$('pm').value=+p[1]||0;$('ps').value=+p[2]||0}
function pauseHands(){fetch('/do?a=pause').then(()=>{$('pauseBtn').textContent='Paused ✓';toast('Hands paused')})}
function setHands(){const t=$('ph').value+':'+String($('pm').value).padStart(2,'0')+':'+String($('ps').value).padStart(2,'0');act('sethands',t,'Hands set to '+t+' and running');closeSheets()}
let armed=0;function forget(){if(!armed){armed=1;toast('Tap again to forget the WiFi');setTimeout(()=>armed=0,4000);return}act('forgetwifi',null,'Restarting into WiFi setup')}

// show state
function paint(){
  const c=S.color||'#ff8c3c';document.documentElement.style.setProperty('--c',c);
  $('on').checked=!!S.on;
  document.querySelectorAll('.tile').forEach(t=>t.classList.toggle('sel',+t.dataset.i===S.effect));
  let matched=false;document.querySelectorAll('.dot[data-c]').forEach(d=>{const m=d.dataset.c.toLowerCase()===c.toLowerCase();d.classList.toggle('sel',m);matched=matched||m});
  document.querySelector('.dot.custom').classList.toggle('sel',!matched);
  if(Date.now()-busy>1500){$('bright').value=S.bright;}
  ['onsec','nfrom','nto','nled','maxma','lstart','ldir','sens','srot','fast','pulse'].forEach(k=>{if(document.activeElement!==$(k)&&S[k]!=null)$(k).value=S[k]});
  ['night','wake','smir','chime','autoupd','health','hbglow','rainwarn'].forEach(k=>$(k).checked=!!S[k]);
  $('nightTimes').classList.toggle('hide',!S.night);
  if(S.g)S.g.forEach((a,i)=>$('gp'+i).textContent=ACTIONS[a]);
  document.querySelectorAll('#units button').forEach(b=>b.classList.toggle('sel',b.dataset.v===S.units));
  if(document.activeElement!==$('zip'))$('zip').value=S.zip||'';
  document.querySelectorAll('#emask input').forEach(c=>c.checked=!!(S.em&(1<<c.dataset.b)));
}
function live(){
  const h=new Date().getHours();$('hello').textContent=(h<12?'Good morning':h<18?'Good afternoon':'Good evening')+(L.date?' · '+L.date:'');
  $('now').textContent=L.time||'--:--';
  $('f_room').textContent=L.room?'Inside '+L.room:'';$('f_out').textContent=L.outside?'Outside '+L.outside:'';
  $('lightNote').textContent=L.ledsBlocked?('Waiting: '+L.ledsBlocked):(S.on?EFFECTS[S.effect][0]+' · turns off by itself':'Off');
  if(L.gesture){$('seen').textContent=L.gesture+' · '+(L.gestureAgo<60?L.gestureAgo+' s ago':Math.round(L.gestureAgo/60)+' min ago');
    if(L.gestureId!==live.last){if(live.last!==undefined){const b=$('seenBox');b.classList.remove('flash');void b.offsetWidth;b.classList.add('flash')}live.last=L.gestureId}}
  $('handsShow').textContent=L.handsShow||'--:--:--';
  const chip=$('handsChip');chip.textContent=L.handsChip||'';chip.classList.toggle('warn',L.handsState!=='ok');
  $('handsNote').textContent=L.handsNote||'';
  $('prox').textContent=L.prox;$('proxBase').textContent=L.proxBase;
  $('sensor').textContent=L.sensor==='9960'?'APDS-9960 (swipes, taps and holds)':L.sensor==='9930'?'APDS-9930 (taps and holds)':'not found';
  const sup=L.gestures||[];const swipes=sup.indexOf(0)>=0;
  document.querySelectorAll('#glist li').forEach(li=>li.classList.toggle('hide',sup.indexOf(+li.dataset.i)<0));
  $('srotBox').classList.toggle('hide',!swipes);$('smirBox').classList.toggle('hide',!swipes);
  $('gLearn').classList.toggle('hide',L.gready!==false);$('gBlock').classList.toggle('hide',!L.gblock);
  $('amb').textContent=L.amb!=null&&L.amb>=0?L.amb:'-';
  $('gIntro').textContent=swipes?'Swipe or tap over the little window just below the clock hands, a few centimetres away.':'Bring your hand toward the little window just below the clock hands, a few centimetres away, then take it away again. Tap, tap twice, or hold your hand there.';$('place').textContent=L.place||'';
  const ms=L.msgs||[];$('msgEmpty').classList.toggle('hide',ms.length>0);$('msgBtns').classList.toggle('hide',!ms.length);
  $('msgList').innerHTML=ms.map(m=>`<li class="${m.seen?'':'new'}"><div>${esc(m.t)}</div><div class="small">${m.ago}${m.seen?' · seen':' · new'}${m.heart?' · heart sent':''}</div></li>`).join('');
  const rs=S.rem||[];$('remEmpty').classList.toggle('hide',rs.length>0);
  $('remList').innerHTML=rs.map((r,i)=>`<li class="rem"><div class="remrow"><div><div>${esc(r.l)}</div><div class="small">${esc(r.when)}${r.on?'':' · off'}</div></div><div class="rembtns"><button class="btn ghost sm" onclick="remOp('toggle',${i})">${r.on?'Turn off':'Turn on'}</button><button class="btn ghost sm" onclick="remOp('del',${i})">Delete</button></div></div></li>`).join('');
  $('rainNow').textContent=L.rain||'';
  $('ver').textContent=L.ver||'-';$('uptime').textContent=L.uptime||'-';$('upd').textContent=L.upd||'-';
  if(S.ntfy){$('ntfyShow').textContent=S.ntfy;$('ntfyLink').href='https://ntfy.sh/'+S.ntfy;}
  ['otaurl','ntfy'].forEach(k=>{if(document.activeElement!==$(k)&&S[k]!=null)$(k).value=S[k]});
  $('net').textContent=(L.wifi?'Connected to '+L.wifi:'')+(L.ip?' · '+L.ip:'');
}
function refresh(){fetch('/state').then(r=>r.json()).then(j=>{if(Date.now()-busy>800)S=j.s;L=j.live;paint();live()}).catch(()=>{})}
refresh();setInterval(refresh,2500);
</script>
</body>
</html>
)HTMLPAGE";

#!/usr/bin/env bash
# Builds the newest sketch in drop/ and lays out the website in _site/.
# Used by .github/workflows/build.yml (you never need to run it yourself).
set -euo pipefail
BUILD="${BUILD_NUMBER:?}"                       # GitHub's run number -> software version 5.<BUILD>
BASE="${PAGES_URL:?}"; BASE="${BASE%/}"          # e.g. https://yourname.github.io/smart-clock
FQBN="${FQBN:-esp32:esp32:XIAO_ESP32C6}"
ACLI="${ACLI:-arduino-cli}"

# 1. the newest .ino in drop/ (by the time it was committed; works with any file name)
newest=""; newest_t=-1
for f in drop/*.ino; do
  [ -e "$f" ] || continue
  t=$(git log -1 --format=%ct -- "$f" 2>/dev/null || true); t=${t:-0}
  if [ "$t" -ge "$newest_t" ]; then newest="$f"; newest_t="$t"; fi
done
[ -n "$newest" ] || { echo "No .ino file in drop/"; exit 1; }
echo "Building $newest as version 5.$BUILD"

# 2. sketch folder: the .ino + support files + build_info.h (version and update address)
rm -rf work out _site
W=work/smart_clock; mkdir -p "$W" out _site/firmware _site/install
cp "$newest" "$W/smart_clock.ino"
cp support/partitions.csv support/clock_support.h "$W/"
cat > "$W/build_info.h" <<INFO
#pragma once
#define FW_BUILD $BUILD
#define OTA_BASE_DEFAULT "$BASE"
INFO

# 3. compile (1966080 bytes = one 1.9 MB program slot)
$ACLI compile -b "$FQBN" --build-property "upload.maximum_size=1966080" ${EXTRA_FLAGS:-} --output-dir out "$W"
APP=out/smart_clock.ino.bin
SIZE=$(stat -c%s "$APP")
[ "$SIZE" -le 1966080 ] || { echo "Too big for one program slot: $SIZE bytes"; exit 1; }
SHA=$(sha256sum "$APP" | cut -d' ' -f1)

# 4. what the clock downloads
cp "$APP" "_site/firmware/app-$BUILD.bin"
cat > _site/firmware/manifest.json <<JSON
{"build": $BUILD, "version": "5.$BUILD", "bin": "firmware/app-$BUILD.bin", "size": $SIZE, "sha256": "$SHA",
 "built": "$(date -u +%Y-%m-%dT%H:%MZ)", "source": "$(basename "$newest")"}
JSON

# 5. the "install with a USB cable" page (ESP Web Tools). The settings area of the chip
#    (WiFi, hand position, light settings) sits between these parts and is left alone.
BOOT_APP0="${BOOT_APP0:-$(find "${ARDUINO_DATA:-$HOME/.arduino15}" -path '*esp32*' -name boot_app0.bin 2>/dev/null | head -1)}"
[ -f "$BOOT_APP0" ] || { echo "boot_app0.bin not found"; exit 1; }
cp out/smart_clock.ino.bootloader.bin _site/install/bootloader.bin
cp out/smart_clock.ino.partitions.bin _site/install/partitions.bin
cp "$BOOT_APP0" _site/install/boot_app0.bin
cp "$APP" _site/install/app.bin
cat > _site/install/manifest.json <<JSON
{"name": "Smart Clock", "version": "5.$BUILD", "new_install_prompt_erase": true,
 "builds": [{"chipFamily": "ESP32-C6", "parts": [
   {"path": "bootloader.bin", "offset": 0},
   {"path": "partitions.bin", "offset": 32768},
   {"path": "boot_app0.bin", "offset": 57344},
   {"path": "app.bin", "offset": 65536}]}]}
JSON

# 6. the web page
cp -r site/. _site/
echo "Done: 5.$BUILD, $SIZE bytes, sha256 $SHA"

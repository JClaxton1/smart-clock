# Smart Clock software

This repository builds the clock's software and publishes it on GitHub Pages.
The clock checks the published page by itself and installs new versions at night.

## Day to day: updating the clock
1. Open the `drop` folder here → **Add file → Upload files** → drop in the new `.ino` → **Commit changes**.
2. Wait about 5 minutes (watch the **Actions** tab turn green).
3. Done. The clock installs it around 3 AM. To install now: Setup tab → **Check for updates now**,
   or send `update` to your ntfy command topic.

If a new version doesn't start properly, the clock goes back to the previous version by itself and sends you a message.

## What's here
| Folder | What it is |
|---|---|
| `drop/` | Put new `.ino` files here |
| `support/` | Files every build needs (memory layout, certificates, build script) |
| `site/` | The web page (USB install button and current version) |
| `.github/workflows/build.yml` | Builds and publishes when something is dropped in |
| `.github/workflows/health.yml` | Every 6 hours: warns your phone if the clock has gone quiet |

## One-time settings (already done if the page works)
- **Settings → Pages → Source: GitHub Actions**
- **Settings → Secrets and variables → Actions → New repository secret**: `NTFY_TOPIC` = the topic name from the clock's Setup tab

# FINAL — reorganized Cubli firmware + UI

This folder is the **canonical layout** for flashing and the wireless console.
The previous tree lives intact next door as `PreFINAL/` (including
`PreFINAL/WIFIMODE/dashboard` and all bring-up sketches). Nothing was deleted
from PreFINAL; content here was **copied** and renamed for Arduino folder=sketch
rules.

## PreFINAL vs FINAL

| | PreFINAL | FINAL |
|---|---|---|
| Role | Archive of the working Gam/FINAL tree (USBMODE, WIFIMODE, telemetry, …) | Clean map: `firmware/`, `UI/`, `builds/` |
| Dashboard UI | `WIFIMODE/dashboard` (Python FastAPI) — still the full operator stack | Not copied; use PreFINAL or keep running against the same UDP link |
| Console web | `WIFIMODE/console/` | `UI/web/` + embed tool `UI/tools/embed_web.py` |
| Teensy sketches | Scattered under `WIFIMODE/` | `firmware/teensy/<id>/` |
| XIAO host | `WIFIMODE/xiao_cubli_console/` | `firmware/xiao/console_host/` |

## Folder map

`
FINAL/
  README.md                 this file
  UI/                       operator web console (served by XIAO)
  builds/                   .hex/.bin outputs + manifest.json (gitignored binaries)
  firmware/
    teensy/                 balance + OTA base sketches
    xiao/                   console_host (HTTP/WS/UDP/OTA bridge)
`

## USB vs wireless flash

- **USB:** open any sketch under `firmware/teensy/<id>/` or `firmware/xiao/console_host/` in Arduino IDE and flash normally.
- **Wireless (OTA):** flash `ota_base` (and later OTA-capable derivatives) via the XIAO console once `console_host` is on the network. Only `ota_base` is marked `ota_capable: true` in `builds/manifest.json` today.
- **Build outputs:** intended path `builds/teensy/<id>/latest.hex` (see `UI/tools/build.py` stub).

## Quick start

1. Embed the web UI into the XIAO sketch: `cd UI/tools && python embed_web.py`
2. Flash XIAO: `firmware/xiao/console_host/console_host.ino` (set WiFi password).
3. Flash Teensy over USB: pick a target under `firmware/teensy/`.
4. For the legacy Python dashboard, use `../PreFINAL/WIFIMODE/dashboard`.
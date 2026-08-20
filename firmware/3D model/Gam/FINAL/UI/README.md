# UI — Cubli wireless console

Browser UI served **by the XIAO** (HTTP :80, WebSocket :81), plus tools to embed
the page into PROGMEM and (later) compile firmware into `../builds/`.

## Layout

| Path | Purpose |
|---|---|
| `web/index.html` | Single-page console (charts, commands, OTA client) |
| `tools/embed_web.py` | Gzip `web/index.html` → `../firmware/xiao/console_host/web_index.h` |
| `tools/build.py` | Stub documenting compile → `../builds/teensy/<id>/latest.hex` |
| `docs/` | Layout and flashing notes |

## PreFINAL vs this UI

- **FINAL/UI** is the lean console embedded on the XIAO.
- **PreFINAL/WIFIMODE/dashboard** remains the full Python FastAPI dashboard; it was not moved. Both can share the same UDP telemetry path.

## Regenerate `web_index.h`

`
cd UI/tools
python embed_web.py
python embed_web.py --check
`

## USB vs wireless

- Flash XIAO and non-OTA Teensy sketches over **USB**.
- **Wireless** Teensy updates go through the console OTA flow (requires an `ota_capable` Teensy image; see `../builds/manifest.json`).
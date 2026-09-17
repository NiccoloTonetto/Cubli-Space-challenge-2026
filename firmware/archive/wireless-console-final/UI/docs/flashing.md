# Flashing

## XIAO (console host)

1. `cd UI/tools && python embed_web.py`
2. Open `firmware/xiao/console_host/console_host.ino`
3. Set `WIFI_PASSWORD`, board **XIAO_ESP32C6**, flash over USB
4. Install WebSockets (Links2004) if needed

## Teensy (USB)

Open `firmware/teensy/<id>/<id>.ino` in Arduino IDE / teensy loader and flash over USB.
Targets are listed in `builds/manifest.json`.

## Teensy (wireless OTA)

1. Teensy must run an `ota_capable` image (currently `ota_base`)
2. Connect phone/laptop to the same WiFi as the XIAO
3. Open the console in a browser and use the OTA upload UI
4. Hex files are expected under `builds/teensy/<id>/latest.hex` once `tools/build.py` is wired up

## Legacy dashboard

Python dashboard and bring-up tools remain under `PreFINAL/WIFIMODE/` (not removed).
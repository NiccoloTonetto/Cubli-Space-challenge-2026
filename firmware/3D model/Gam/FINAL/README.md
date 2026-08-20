# Cubli FINAL — Wireless GUI guide (follow this)

This is the **step-by-step** guide to run the new Cubli console **over Wi‑Fi**.

You will:

1. Change a few lines in the XIAO code (Wi‑Fi name/password).
2. Flash the **XIAO** over **USB**.
3. Flash the **Teensy** over **USB** (first time).
4. Open the GUI in a phone/laptop browser on the same Wi‑Fi.
5. (Later) upload a Teensy **`.hex`** file from the GUI — **not** an `.exe`.

---

## Critical: there is no `firmware.exe`

| Wrong idea | Correct for Cubli |
|---|---|
| Windows `.exe` program | **Teensy firmware = `.hex` file** (Intel HEX) |
| “Generate exe and run it” | **Compile in Arduino IDE → get `.hex` → flash to the board** |
| ESP32 “exe” | XIAO firmware is flashed from the `.ino` sketch (Arduino produces `.bin` internally; you usually just press **Upload**) |

**How you get a Teensy `.hex` today**

1. Open a sketch in Arduino IDE (e.g. `ota_base.ino`).
2. Board = **Teensy 4.1**.
3. Menu: **Sketch → Export compiled Binary**.
4. Arduino writes a `.hex` **next to the `.ino`** (or shows the path in the console).
5. That `.hex` is what the wireless UI can upload later.

`python build.py` does **not** build `.hex` yet (stub only). Use Arduino for now.

---

## What to flash first (order matters)

```
① USB → XIAO     console_host.ino     (puts the website on Wi‑Fi)
② USB → Teensy   ota_base.ino         (safe first Teensy; enables wireless flash later)
③ Browser        http://172.20.10.14/  (GUI)
④ Optional       wireless UPLOAD/FLASH of a .hex from the GUI
```

Do **not** try wireless Teensy flash before steps ① and ②.

---

## Which code files are which

| Board | File you open in Arduino | Role |
|---|---|---|
| **XIAO ESP32-C6** | `firmware/xiao/console_host/console_host.ino` | Wi‑Fi + serves GUI + talks to Teensy + OTA bridge |
| **Teensy 4.1** (recommended first) | `firmware/teensy/ota_base/ota_base.ino` | Link + telemetry + wireless receive; **no balancing torque** |
| Teensy (real corner) | `firmware/teensy/corner_full_law/corner_full_law.ino` | Balance — flash over **USB** for now |
| Teensy (real edge) | `firmware/teensy/edge_balance/edge_balance.ino` | Balance — flash over **USB** for now |

**GUI page (not flashed alone):** `UI/web/index.html`  
It is packed into the XIAO sketch by `embed_web.py` → `web_index.h`.

**Old full dashboard (optional, laptop):**  
`../PreFINAL/WIFIMODE/dashboard/` — separate from this wireless GUI.

---

## Codes you must change (XIAO Wi‑Fi)

Open:

```
firmware/xiao/console_host/console_host.ino
```

Find this block (near the top, “USER CONFIG”):

```cpp
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "12345678";   // <<< SET THIS

IPAddress XIAO_IP     (172, 20, 10, 14);
IPAddress XIAO_GATEWAY(172, 20, 10,  1);
IPAddress XIAO_SUBNET (255, 255, 255, 240);
```

### What to edit

| Setting | Change? | Rule |
|---|---|---|
| `WIFI_SSID` | **Yes** if your hotspot name is not `cubli1` | Must match your phone hotspot exactly |
| `WIFI_PASSWORD` | **Yes** — set your real password | Must match the hotspot |
| `XIAO_IP` / gateway / subnet | **Only if** your hotspot is not the usual iPhone `/28` (`172.20.10.x`) | If unsure, keep defaults **and** use the same hotspot style as before |
| `TEENSY_LINK_BAUD` (`1000000`) | **Do not change** unless you also change the Teensy sketch | Mismatch = dead UART |

Save the file after editing.

---

## One-time Arduino setup

### For XIAO

1. Install **Arduino IDE**.
2. Add board support for **Seeed XIAO ESP32C6** (Espressif ESP32 boards as needed).
3. Install library: **Sketch → Include Library → Manage Libraries**  
   Search: **WebSockets** → author **Markus Sattler** (Links2004) → **Install**.

### For Teensy

1. Install **Teensyduino** / Teensy board support.
2. Board menu must show **Teensy 4.1**.

### For `ota_base` only (FlasherX)

Download: https://github.com/joepasquariello/FlasherX  

Copy **only these two files** into:

```
firmware/teensy/ota_base/
```

- `FlashTxx.c`
- `FlashTxx.h`

**Do not** copy `FXUtil.cpp` / `FXUtil.h`.

Your folder should look like:

```
ota_base/
  ota_base.ino
  FlashTxx.c
  FlashTxx.h
  README.md
```

---

## STEP-BY-STEP: run the GUI wirelessly

All PowerShell commands assume this repo root:

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026"
```

---

### STEP 0 — (Optional) preview GUI on PC without hardware

```powershell
cd "firmware\3D model\Gam\FINAL\UI\web"
python -m http.server 8765
```

Browser: `http://127.0.0.1:8765/`  
Stop with `Ctrl+C`.

This does **not** talk to the cube. Use it only to look at the page.

---

### STEP 1 — Pack the GUI into the XIAO firmware

Run this **before every XIAO upload** if you changed `UI/web/index.html`.  
Safe to run every time anyway.

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\3D model\Gam\FINAL\UI\tools"
python embed_web.py
python embed_web.py --check
```

**Good result:** prints byte sizes, then `ok: web_index.h is current`.

**What this does:** gzips `UI/web/index.html` into  
`firmware/xiao/console_host/web_index.h` so the XIAO can serve the page.

---

### STEP 2 — Flash the XIAO over USB (FIRST board)

1. Edit Wi‑Fi in `console_host.ino` (see **Codes you must change** above).
2. Arduino IDE → **File → Open** →  
   `...\FINAL\firmware\xiao\console_host\console_host.ino`
3. Tools → Board → **XIAO_ESP32C6** (or exact Seeed XIAO ESP32C6 name).
4. Tools → Port → the COM port of the XIAO.
5. Click **Upload**.
6. Wait until “Done uploading”.

**What you just flashed:** the wireless web server + bridge.

Leave the cube powered so the XIAO stays on (USB or cube power).

---

### STEP 3 — Flash the Teensy over USB (SECOND board)

**Recommended first firmware:** `ota_base` (safe, OTA-ready, no motors spinning for balance).

1. Confirm `FlashTxx.c` / `FlashTxx.h` are in `ota_base/` (see above).
2. Arduino IDE → Open  
   `...\FINAL\firmware\teensy\ota_base\ota_base.ino`
3. Tools → Board → **Teensy 4.1**.
4. Tools → Port → Teensy COM port.
5. Click **Upload**.

**Wiring (must already be correct):**

| Teensy | XIAO |
|---|---|
| TX1 pin 1 | D7 (RX) |
| RX1 pin 0 | D6 (TX) |
| GND | GND |

---

### STEP 4 — Connect Wi‑Fi and open the GUI

1. Turn on your phone hotspot (SSID/password = what you set in `console_host.ino`).
2. Connect the **laptop or phone** to that hotspot.
3. Power the cube (XIAO + Teensy on).
4. Open a browser:

```
http://172.20.10.14/
```

If you changed `XIAO_IP` in the sketch, use that IP instead.

---

### STEP 5 — Check the GUI is alive

Top bar:

| Pill | Success | Fail |
|---|---|---|
| **WS** | ● WS | ○ WS / WS DOWN → Wi‑Fi or XIAO flash problem |
| **UART** | ● UART … Hz | ○ UART → Teensy power/wiring/firmware |
| **DISARMED** | normal at start | |

You should see tilt / wheel charts update (with `ota_base`).

**E-STOP** (or spacebar) disarms.

---

### STEP 6 — (Later) wireless flash a Teensy `.hex` from the GUI

Only after steps 1–5 work, and Teensy is still on an OTA-capable image (`ota_base`).

#### 6a — Generate the `.hex` (Arduino)

1. Open the Teensy sketch you want (example: `ota_base` again, or a future OTA build).
2. Board = Teensy 4.1.
3. **Sketch → Export compiled Binary**.
4. Find the `.hex` file (same folder as the `.ino`, name like `ota_base.ino.HEX` or similar).

**That file is your firmware image — not an `.exe`.**

#### 6b — Upload from the browser

1. Cube **IDLE** + **DISARMED**.
2. In GUI → **Wireless flash** → Choose File → select the `.hex`.
3. **UPLOAD** → wait for progress.
4. **FLASH** → confirm → Teensy reboots.

**Not ready yet:** auto-building into `builds/teensy/.../latest.hex` via `build.py`, and wireless-updating `corner_full_law` / `edge_balance` while keeping OTA forever.

---

## If you want real balancing today

Wireless GUI can still show telemetry if the XIAO is `console_host`, but:

1. Flash Teensy over **USB** with  
   `corner_full_law.ino` or `edge_balance.ino`.
2. Open `http://172.20.10.14/`.
3. Mode buttons / OTA may not match those older sketches — use E-STOP carefully; prefer PreFINAL Python tools if you need the full control panel for those builds.

---

## Command cheat sheet (copy/paste)

```powershell
# Always start here
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026"

# Preview GUI on PC only
cd "firmware\3D model\Gam\FINAL\UI\web"
python -m http.server 8765
# → http://127.0.0.1:8765/

# Pack GUI into XIAO (before USB flash of XIAO)
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\3D model\Gam\FINAL\UI\tools"
python embed_web.py
python embed_web.py --check

# List planned build targets (does NOT compile yet)
python build.py --list

# Real wireless GUI after USB flashes:
# → http://172.20.10.14/
```

**Arduino (no PowerShell flash yet):**

| Action | Sketch |
|---|---|
| USB flash #1 | `FINAL\firmware\xiao\console_host\console_host.ino` |
| USB flash #2 | `FINAL\firmware\teensy\ota_base\ota_base.ino` |
| Make `.hex` for later OTA | Sketch → **Export compiled Binary** |

---

## What is NOT implemented yet

| Item | Meaning for you |
|---|---|
| `build.py` real compiler | You still use Arduino to make `.hex` |
| Firmware `.exe` | Does not exist; use `.hex` |
| GUI dropdown of `builds/` firmwares | Use the file picker |
| Wireless flash of corner/edge and remain OTA-capable | USB those sketches for now |
| `ota_base` actually balancing | Torque stubs; instrument + OTA only |
| Terminal USB flash (`flash_usb.py`) | Use Arduino **Upload** |
| Phone app | Browser page is enough (`http://172.20.10.14/`) |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Browser can’t open `172.20.10.14` | Same Wi‑Fi as XIAO? Password/SSID match sketch? XIAO uploaded? |
| ○ WS | Reconnect hotspot; reflash XIAO; check IP |
| ○ UART | Teensy powered? Wiring? Uploaded `ota_base`? |
| `ota_base` compile error about FlasherX | Add `FlashTxx.c` + `FlashTxx.h` |
| WebSockets compile error on XIAO | Install Links2004 **WebSockets** library |
| Charts empty | Teensy not running / wrong sketch |
| Looking for `.exe` | Export **compiled Binary** → use the `.hex` |

---

## Safe first session checklist

- [ ] Edited `WIFI_SSID` / `WIFI_PASSWORD` in `console_host.ino`
- [ ] Ran `python embed_web.py` in `UI\tools`
- [ ] USB flashed **XIAO** `console_host`
- [ ] Copied FlasherX into `ota_base/`
- [ ] USB flashed **Teensy** `ota_base`
- [ ] Phone hotspot on; device joined that Wi‑Fi
- [ ] Browser opened `http://172.20.10.14/`
- [ ] Saw ● WS and ● UART
- [ ] Only then try wireless `.hex` UPLOAD/FLASH

---

## Related files

| Path | Purpose |
|---|---|
| `UI/web/index.html` | GUI source |
| `UI/tools/embed_web.py` | Pack GUI → XIAO |
| `UI/tools/build.py` | Stub (no compile yet) |
| `builds/manifest.json` | Names of firmware targets |
| `firmware/teensy/ota_base/README.md` | OTA details |
| `../PreFINAL/WIFIMODE/dashboard/` | Old laptop GUI |

# CubliUI — everything needed to run the cube

One folder: the **Teensy** balance firmware, the **XIAO** Wi-Fi firmware, the
**laptop dashboard**, and every Python helper.

You should not need to look anywhere else in the repo to run the cube.

```
Browser (laptop)  ──►  Python app.py  ──UDP──►  XIAO  ──UART──►  Teensy
   :8000                (your laptop)          bridge
```

---

## 1. What changed

There used to be two UIs and two Teensy binaries. Now there is one of each.

- **MODE B (the wireless console + OTA flashing) is gone.** It was abandoned as
  unreliable. Its source is still in the archive under
  `firmware/3D model/Gam/FINAL/` if you ever need to look at it.
- **Edge and corner are one sketch.** `CubliBalance.ino` holds both control
  laws and switches between them at runtime, so changing what the cube balances
  on is a button, not a re-flash. This is what OTA existed to work around.

---

## 2. Folder map

```
firmware/CubliUI/
│
├── README.md                          ← you are here
│
├── teensy/
│   ├── CubliBalance/CubliBalance.ino  ← FUSION: both laws + m0/m1 ← use this
│   ├── CubliCorner/CubliCorner.ino    ← standalone corner (A/B reference)
│   ├── CubliEdge/CubliEdge.ino        ← standalone edge   (A/B reference)
│   └── cubli_gains.h                  ← simulation-generated constants
│
├── xiao/
│   └── xiao_teensy_bridge/            ← the ONLY XIAO program
│
├── dashboard/
│   ├── app.py                         ← run this
│   ├── schemas.py                     ← wire formats + command whitelist
│   ├── replay.py                      ← desk test with no hardware
│   └── static/                        ← the web UI (+ classic/ fallback)
│
├── tools/
│   ├── terminal_wifi.py               ← serial monitor over Wi-Fi
│   ├── link_check.py                  ← one-shot verdict on the link
│   ├── telemetry_python_wifi.py       ← live matplotlib plots — edge
│   ├── telemetry_python_wifi_corner.py← live matplotlib plots — corner
│   └── analysis/                      ← offline plotters and reports
│
├── docs/
│   ├── WIRING.md                      ← pins, baud, power
│   ├── COMMANDS.md                    ← every command letter
│   └── TELEMETRY_FORMATS.md           ← 10 / 26 column meanings
│
└── telemetry/                         ← recordings land here (created on first run)
    ├── plot/                          ← dashboard + live plotter CSV
    └── serial/                        ← terminal_wifi.py logs
```

---

## 3. Flash the two boards (over USB, Arduino IDE)

| Board | Sketch | Board setting |
|---|---|---|
| **XIAO ESP32-C6** | `xiao/xiao_teensy_bridge/xiao_teensy_bridge.ino` | XIAO_ESP32C6 |
| **Teensy 4.1** | `teensy/CubliBalance/CubliBalance.ino` | Teensy 4.1 |

### The three Teensy targets

`CubliBalance` is the one to run. The other two exist so you can A/B it against
the builds it was fused from — same XIAO firmware, same dashboard, no other
changes.

| Sketch | Balances on | Mode switch | Halt | Use it for |
|---|---|---|---|---|
| **`CubliBalance`** | edge **and** corner | ✓ `m0` / `m1` | ✓ `h1`/`h0` | **normal operation** |
| `CubliCorner` | corner only (3 wheels) | ✗ | ✓ `h1`/`h0` | proving the corner law is unchanged |
| `CubliEdge` | edge only (1 wheel) | ✗ | ✗ **none** | proving the edge law is unchanged |

The two standalone sketches are verbatim copies of the archive references, so
"does the fusion behave identically?" is answerable by flashing each in turn
and comparing. Their headers list exactly which dashboard buttons go quiet
against them (mode switch on both, **halt on `CubliEdge`**). That is expected,
not a fault — those commands only exist in the fused build.

Compile all three without uploading:

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\CubliUI"
foreach ($s in "CubliCorner","CubliEdge","CubliBalance") {
  arduino-cli compile -b teensy:avr:teensy41 "teensy/$s"; "$s -> $LASTEXITCODE"
}
```

Set your Wi-Fi in the XIAO sketch before uploading:

```cpp
const char* WIFI_SSID     = "cubli1";
const char* WIFI_PASSWORD = "your-password";
```

Once flashed you should not need to touch the XIAO again — mode, gain, trim and
arming are all runtime commands.

> ⚠ **Corner mode has no arm gate.** `a1` arms at any tilt. Place the cube near
> balance first and keep a hand on it. See `docs/COMMANDS.md`.

---

## 4. Run the dashboard

Once, to install:

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\CubliUI\dashboard"
pip install -r requirements.txt
```

Then, every session — **close anything else using UDP 4210 first** (the
plotters, `terminal_wifi.py`):

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\CubliUI\dashboard"
python app.py
```

Open **http://127.0.0.1:8000**. Your laptop must be on the same Wi-Fi as the
XIAO.

| Route | UI |
|---|---|
| `/` | Minimal console — arm/disarm, mode switch, status, charts, 3D view |
| `/classic` | The previous Tailwind dashboard, kept as a fallback |

Useful flags:

```powershell
python app.py --read-only        # monitor mode: arming and mode switch blocked
python app.py --port 8080        # different web port
```

### Confirm it works

| Check | Expected |
|---|---|
| Link pill | shows a rate, not `NO LINK` |
| State pill | `DISARMED · CORNER` (or `· EDGE`) |
| Charts | moving |
| E-STOP | always available |

If nothing arrives:

```powershell
cd "..\tools"
python link_check.py
```

---

## 5. Driving the cube

Full grammar in `docs/COMMANDS.md`. The short version:

| Button | Sends |
|---|---|
| EDGE / CORNER | `m0` / `m1` — **always disarms first** |
| ARM | `a1` (two-step confirm) |
| E-STOP / DISARM | `a0` — also **spacebar** |
| Re-resolve pivot | `c` (corner) / `e` (edge) |
| Halt / Resume | `h1` / `h0` |
| Gain slider | `g<0..1>` |
| Edge trim | `o<deg>` (edge mode only) |

A typical corner session:

```
place the cube on the corner  →  CORNER  →  Re-resolve  →  gain 0.5  →  ARM
```

---

## 6. Proving the fusion is faithful (A/B on the bench)

Cube **held by hand**, wheels clear, `g0.3`, a finger on the spacebar. Same
XIAO firmware and same dashboard throughout — only the Teensy sketch changes.

**Round 1 — `CubliCorner`.** Place on the corner → Re-resolve → note the
resolved corner name and the resting `|φ|` → ARM briefly → DISARM. Record those
two numbers.

**Round 2 — `CubliEdge`.** Place on the edge → Re-resolve → note the resolved
edge name and resting `|φ|` → ARM (must refuse outside the gate) → DISARM.
Confirm HALT does nothing — that is this build having no halt command.

**Round 3 — `CubliBalance`.** Now both, without re-flashing:

1. Boot shows `# state=DISARMED mode=CORNER armed=0 halted=0`.
2. Corner: Re-resolve → **same corner name and resting `|φ|` as round 1**.
   ARM → `# state=CORNER_BALANCE`. DISARM.
3. Press **EDGE**. Watch, in order: the cube disarms, `# state=DISARMED
   mode=EDGE`, the packet width drops 26 → 10, the dashboard rolls to a new
   CSV, and the two inactive wheels stop.
4. Edge: Re-resolve → **same edge name and resting `|φ|` as round 2**. ARM →
   must refuse outside the gate exactly as it did in round 2.
5. Press **CORNER** and confirm it goes back cleanly.
6. While ARMED, confirm the mode buttons are greyed out.

Matching pivot names and resting `|φ|` across rounds is the real check: it
means the estimator, mount DCM and candidate tables survived the fusion. If
round 3 disagrees with rounds 1–2, the fusion changed something it should not
have — the control paths were copied verbatim, so suspect the mode dispatch in
`loop()` or the renames in the sketch's SECTION 0b before suspecting the law.

---

## 7. Desk test with no hardware

```powershell
# terminal 1
cd "...\CubliUI\dashboard"
python app.py --target 127.0.0.1 --target-port 4211

# terminal 2
python replay.py
```

`--target-port 4211` is required on loopback, otherwise the server talks to its
own bound port and every button looks dead.

Expected quirk: the state pill stays DISARMED and ARM behaves as if far from
equilibrium, because the recorded run has `armed = 0` and starts well off
balance.

`replay.py` needs a recording in `telemetry/plot/`. There are none there yet —
either make one from a live session, copy a `telemetry_corner_*.csv` in from
`firmware/3D model/Gam/PreFINAL/telemetry/plot/`, or use
`python replay.py --schema edge`, which synthesizes a 10-field stream and needs
no file at all.

---

## 8. Other tools

Run these **instead of** `app.py` — they compete for the same UDP port.

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\CubliUI\tools"
```

| Command | Purpose |
|---|---|
| `python terminal_wifi.py` | Serial-monitor over Wi-Fi; type commands directly |
| `python telemetry_python_wifi_corner.py` | Live matplotlib plots — **corner** |
| `python telemetry_python_wifi.py` | Live matplotlib plots — **edge** |
| `python link_check.py` | One-shot verdict on the link |

Each live plotter only parses one mode's format, so switching the cube to the
other mode makes its window go quiet. That is the other mode talking, not a
fault.

### Offline analysis

```powershell
cd "...\tools\analysis"
python plot_session_csv.py          # no args + press d = newest recording
python fft_tilt_analysis.py <file.csv>
python standing_speed_report.py <file.csv>
```

---

## 9. Wi-Fi settings

Baked into the XIAO sketch:

| Setting | Value |
|---|---|
| SSID | `cubli1` |
| Password | set it yourself in the `.ino` |
| XIAO IP | `172.20.10.14` |
| Gateway | `172.20.10.1` |
| UART to Teensy | **1 000 000 baud** |

**Do not change the baud** unless you change `kLinkBaud` in the Teensy sketch
too. A mismatch is silent and looks exactly like a dead cable.

---

## 10. Known rough edges

| Thing | Status |
|---|---|
| Edge arm gate reads 9.58°, comment says 0.5° | Carried forward verbatim on purpose; suspected typo, fix it deliberately with a bench check |
| Corner mode has no arm gate at all | By design. Operator discipline, not a bug |
| `gEdgeAxis` has no command letter | Edge mode is Y-only in practice |
| Battery gauge | No wire format carries voltage; the UI shows link health instead |
| Velocity caps | Bring-up values in both modes, not the Stage 5 policy numbers |

---

## 11. Where the old files live

Nothing was deleted from the archives.

| Old location | Still there? |
|---|---|
| `firmware/3D model/Gam/PreFINAL/` | ✓ full archive — the edge reference build |
| `firmware/3D model/Gam/FINAL/` | ✓ full archive — the corner reference build, the old wireless console, your logs |

`CubliBalance.ino` is a **fusion** of two of those sketches, not a move.
`CubliCorner` / `CubliEdge` are verbatim copies of them. The originals are
untouched: retune there, then re-copy.

---

## 12. Read next

| Question | File |
|---|---|
| How do I wire it? | `docs/WIRING.md` |
| What does `a1` / `m0` / `g0.5` do? | `docs/COMMANDS.md` |
| What are the telemetry columns? | `docs/TELEMETRY_FORMATS.md` |
| How does the mode switch actually work? | `teensy/CubliBalance/CubliBalance.ino` header |

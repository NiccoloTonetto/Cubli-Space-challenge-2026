# FINAL — the four builds that balance the cube

Corner and edge balancing, each in a USB-tethered and a WiFi-untethered
variant. All four are **Stage 4 full law** (position + rate + momentum
management + friction feedforward + 0.5° arm gate), cube **held by hand**.

```
FINAL/
├── plot_session_csv.py          offline plotter — reads either format
├── telemetry/                   every recording, split by mode
│   ├── plot/                    PLOTMODE .csv — from the live plotters
│   └── serial/                  SERIALMONITORMODE .log — from terminal_wifi.py
├── USBMODE/
│   ├── CornerBalance/           3 wheels, USB only
│   └── EdgeBalance/             1 axis (X/Y/Z selectable), USB only
└── WIFIMODE/
    ├── CornerBalance_WiFi/      3 wheels, Serial1 → XIAO → UDP  (Stage 4)
    ├── EdgeBalance_WiFi/        1 axis, Serial1 → XIAO → UDP
    ├── corner-bringup/          WiFi Stages 1, 2, 3, 5 — terminal, no plot
    ├── xiao_teensy_bridge/      the ESP32C6 relay (shared by both)
    ├── terminal_wifi.py         Serial Monitor over UDP — every corner stage
    ├── link_check.py            staged link diagnostic — run this first
    ├── telemetry_python_wifi.py         live plotter — EDGE (10 col)
    └── telemetry_python_wifi_corner.py  live plotter — CORNER Stage 4 (21 col)
```

If the link is silent, run `WIFIMODE/link_check.py` before anything else —
it isolates which hop is broken.

## Which one do I flash?

| | USB | WiFi |
|---|---|---|
| **Corner** (3 wheels) | [`USBMODE/CornerBalance/`](USBMODE/CornerBalance/CornerBalance.ino) | [`WIFIMODE/CornerBalance_WiFi/`](WIFIMODE/CornerBalance_WiFi/CornerBalance_WiFi.ino) |
| **Edge** (1 axis) | [`USBMODE/EdgeBalance/`](USBMODE/EdgeBalance/EdgeBalance.ino) | [`WIFIMODE/EdgeBalance_WiFi/`](WIFIMODE/EdgeBalance_WiFi/EdgeBalance_WiFi.ino) |

Bench work with a cable attached → USB. Untethered running → WiFi. You can
also flash a WiFi build and send `t0` to move telemetry back onto USB live,
with no reflash.

## Validation status — read this before the first run

| Build | Status |
|---|---|
| `USBMODE/CornerBalance` | Hardware-validated. Byte-identical copy of the flown build. |
| `USBMODE/EdgeBalance` | Hardware-validated. Byte-identical copy of the flown build. |
| `WIFIMODE/EdgeBalance_WiFi` | **Never run on hardware.** Bring-up pending. |
| `WIFIMODE/CornerBalance_WiFi` | **Never run on hardware.** Compiles clean for Teensy 4.1 (FLASH 69 kB, RAM1 vars 21 kB). |

Neither WiFi build has flown. Both reuse a control path that has, but the
link layer around it is unexercised. Treat the first run of either as a
bring-up: hand-held, one hand on the cube, `a0` ready.

### Expected build warnings

`CornerBalance_WiFi` and `USBMODE/CornerBalance` both emit three
`-Wnarrowing` warnings on the `rho[3]` initialiser in `loop()`:

```
narrowing conversion of '... moteusX...values.velocity ...' from 'double' to 'float'
```

`Query::Result::velocity` is a `double` in the moteus library, so the
expression is a double and brace-initialising a `float[3]` from it warns.
**This is pre-existing and benign** — identical text appears in corner-bringup
Stages 1 through 5, including the flown Stage 4. Wheel speeds run ~0–200
rad/s against float's ~7 significant digits, so the lost precision is ~1e-5
rad/s, orders of magnitude below gyro noise.

The edge builds assign rather than brace-initialise
(`const float wheel_omega = ...`), which performs the same conversion
silently. That is the only reason they are quiet.

Do not silence these in one build alone — that would break the byte-identity
guarantee below. If you want them gone, `static_cast<float>` every corner
stage in one sweep.

## Provenance — and how to prove it

The USB builds are byte-identical copies of their bring-up sources; only the
filename changed. Verify at any time:

```powershell
git diff --no-index "USBMODE/CornerBalance/CornerBalance.ino" `
  "../corner-bringup/Stage4_FullLaw/Stage4_FullLaw.ino"

git diff --no-index "USBMODE/EdgeBalance/EdgeBalance.ino" `
  "../edge-bringup/Stage4_FullLaw/Stage4_FullLaw.ino"
```

Both should print nothing.

```
corner-bringup/Stage4_FullLaw/  ──copy──>  USBMODE/CornerBalance/
                                               │ + link layer
                                               ▼
                                           WIFIMODE/CornerBalance_WiFi/

edge-bringup/Stage4_FullLaw/    ──copy──>  USBMODE/EdgeBalance/
                                │
                                └─ edge-balance-final/Stage4_FullLaw_WiFi/
                                               │ ──copy──>
                                               ▼
                                           WIFIMODE/EdgeBalance_WiFi/
```

**Retune in `USBMODE/`, then re-copy the changed constants into the WiFi
build.** Do not let a pair drift — the whole value of this folder is that
the WiFi build's physics is provably the USB build's physics.

## The one grammar difference to keep straight

`h` means different things depending on which build is flashed:

| | Corner USB | Corner WiFi | Edge USB | Edge WiFi |
|---|---|---|---|---|
| `h` | **HALT** | keepalive (no-op) | — | keepalive (no-op) |
| `p` | — | **HALT** | — | — |

The WiFi PC scripts send a bare `h` every 100 ms as a link keepalive, so `h`
cannot also be halt on a WiFi build — it would halt the cube ten times a
second. Halt therefore moves to `p` there.

It is **not** bound to `x`: the XIAO bridge consumes `x` packets to switch
its own mode and never forwards them, so an `x`-bound halt would silently
fail over WiFi while still working over USB.

## Telemetry formats

| Build | Fields | Rate | Parser |
|---|---|---|---|
| Edge (both) | 10 | 500 Hz | `telemetry_python_wifi.py` |
| Corner (both) | 21 | **250 Hz** (WiFi) | `telemetry_python_wifi_corner.py` |

The corner WiFi build decimates telemetry to 250 Hz. **Its control loop still
runs at 500 Hz** — only the emission is throttled. 21 fields at 500 Hz is
~850 kbit/s against ~800 kbit/s usable on the 1 Mbaud link; overrunning it
would back up the Teensy TX buffer and stall the 2 ms control cycle.

The two formats are not interchangeable. Each script rejects the other's
lines and says so.

## After a run: `plot_session_csv.py`

Both live scripts save a timestamped CSV when you close the plot window, into
[`telemetry/plot/`](telemetry/README.md) — `telemetry_edge_<stamp>.csv` from
the edge plotter, `telemetry_corner_<stamp>.csv` from the corner one, so the
name says which build produced a run. `terminal_wifi.py`'s session logs go to
`telemetry/serial/`. See [`telemetry/README.md`](telemetry/README.md) for what
separates the two.

```bash
# no arguments — asks which file:
#   d  newest recording (the run you just did)
#   f  list both modes and pick by number, name, or timestamp
python plot_session_csv.py

# interactive — every trace as a checkbox, tick what you want
python plot_session_csv.py telemetry/plot/telemetry_corner_20260816_143022.csv

# the timestamp alone is enough to name a file
python plot_session_csv.py 143022 --cols tilt

# straight to a figure
python plot_session_csv.py run.csv --cols tilt
python plot_session_csv.py run.csv --cols "phi_x,phi_y,|phi|"
python plot_session_csv.py run.csv --cols wheels,torque --t 5:12
python plot_session_csv.py run.csv --cols tilt --save tilt.png

# what's in this file?
python plot_session_csv.py run.csv --list
```

Edge or corner format is auto-detected from the column count, with or
without a header row; tab-delimited `SERIALMONITORMODE` captures work too.
Traces are grouped onto subplots by unit, armed regions are shaded, and
`|phi|` is plotted against the 0.5° arm gate. Groups: `tilt`, `rates`,
`wheels`, `torque`, `all`.

## Corner bring-up over WiFi

Stages 1, 2, 3 and 5 of the corner ladder also have WiFi builds, under
[`WIFIMODE/corner-bringup/`](WIFIMODE/corner-bringup/README.md). Those are
read with [`WIFIMODE/terminal_wifi.py`](WIFIMODE/terminal_wifi.py) — a
Serial-Monitor clone, no plotting. Stage 4 is still
`WIFIMODE/CornerBalance_WiFi/` and can be read either way: the existing
plotter (`TELEMETRY_MODE PLOTMODE`, the default) or `terminal_wifi.py`
(`SERIALMONITORMODE`).

Halt on those bring-up stages stays `h1`/`h0` (USB grammar kept). Keepalive
moved to `k`. That is the opposite of `CornerBalance_WiFi`, where `h` is the
keepalive and halt is `p`. `terminal_wifi.py` sends `k`, which both grammars
accept.

## Next

`../edge-bringup/Stage5_Release/` is still USB-only. The corner Stage 5 WiFi
build is `WIFIMODE/corner-bringup/Stage5_Release_WiFi/`.

# WIFIMODE — untethered builds

Teensy → Serial1 @ 1 Mbaud → XIAO ESP32C6 → WiFi/UDP → laptop. The control
path is identical to the matching `../USBMODE/` build; only the transport and
a link watchdog are added.

> ### ⚠️ Neither build has run on hardware
>
> | | Status |
> |---|---|
> | `EdgeBalance_WiFi` | Never run on hardware. Bring-up pending. |
> | `CornerBalance_WiFi` | Never run on hardware. Compiles clean for Teensy 4.1, with three expected `-Wnarrowing` warnings on `rho[3]` that are pre-existing in every corner stage — see the top-level README. |
>
> Both reuse a flown control path, but the link layer around it is
> unexercised. First run of either is a bring-up: hand-held, one hand on the
> cube, `a0` ready.

## Contents

| | |
|---|---|
| [`CornerBalance_WiFi/`](CornerBalance_WiFi/CornerBalance_WiFi.ino) | 3 wheels, 21-field CSV @ 250 Hz |
| [`EdgeBalance_WiFi/`](EdgeBalance_WiFi/EdgeBalance_WiFi.ino) | 1 axis, 10-field CSV @ 500 Hz |
| [`xiao_teensy_bridge/`](xiao_teensy_bridge/xiao_teensy_bridge.ino) | ESP32C6 relay — copy of `firmware/XIAO/xiao_teensy_bridge/`, unchanged |
| [`telemetry_python_wifi.py`](telemetry_python_wifi.py) | live console — **EDGE only** (10 col) |
| [`telemetry_python_wifi_corner.py`](telemetry_python_wifi_corner.py) | live console — **CORNER only** (21 col) |
| [`link_check.py`](link_check.py) | one-command verdict on an already-flashed link |
| [`link-bringup/`](link-bringup/README.md) | **staged bring-up of the link itself** — six stages, each one hop longer, nothing that can move a wheel until stage 6 |

**One XIAO, one static IP → run one build at a time.** Both Teensy sketches
talk to the same bridge firmware on the same address.

## What the WiFi builds add

- **Second channel on Serial1** (1 Mbaud) to the XIAO, which relays lines
  to/from the laptop over UDP.
- **Link watchdog**: in WiFi mode, no line on Serial1 for 300 ms → auto-
  disarm, exactly what an `a0` does. USB mode never auto-disarms.
- **Non-blocking line reading** on both streams. The USB builds'
  `Serial.readStringUntil()` can block up to its timeout on a partial line,
  which is not acceptable inside a 2 ms control cycle.
- **Boot diagnostics mirrored to both streams**, so an untethered bring-up
  still shows the CAN check, IMU status, the "hold still" gyro-calibration
  prompt and the resolved candidate in the WiFi console.
- `TELEMETRY_MODE` defaults to `PLOTMODE` (the USB builds default to
  `SERIALMONITORMODE`).

Commands are read from **both** streams every loop iteration, so a mode
switch is never missed regardless of which channel is reachable.

## Commands

| Cmd | Corner WiFi | Edge WiFi |
|---|---|---|
| `a1` / `a0` | arm / disarm (0.5° gate) | arm / disarm (0.5° gate) |
| `g<0..1>` | gain scale | gain scale |
| `c` | re-resolve corner | — |
| `z1` / `z0` | tare / clear phi offset | — |
| `o<deg>` | — | `kPhiOffset`, per axis |
| `e` | — | re-resolve edge |
| `p1` / `p0` | **HALT** / resume | — |
| `t0` / `t1` | link mode USB / WiFi (default `t1`) | same |
| `h` or `k` | no-op keepalive | no-op keepalive |
| `x0` / `x1` | *XIAO's* mode — never reaches the Teensy | same |

### Why halt is `p`, and why it is not `x`

The PC scripts send a bare `h` every 100 ms to feed the 300 ms watchdog. So
`h` must be a no-op here — but the corner **USB** build uses `h1`/`h0` for
halt. Flashing that grammar onto a WiFi build would halt the cube ten times
a second.

`x` is not available either: [`xiao_teensy_bridge.ino`](xiao_teensy_bridge/xiao_teensy_bridge.ino)
consumes `x` packets to switch its own mode and **never relays them**. A halt
bound to `x` would silently do nothing over WiFi while still working over
USB — the worst possible failure mode for a safety command.

Hence `p`. Keep this straight when swapping between USB and WiFi builds.

## Telemetry rate — why corner is 250 Hz

A 21-field corner line is ~170 B typical / ~200 B worst case. At 500 Hz that
is ~850 kbit/s against ~800 kbit/s usable on a 1 Mbaud 8N1 link — over
budget. The TX buffer would back up and `Serial1` writes would start blocking
inside the 2 ms control cycle, which is precisely the failure the non-
blocking `LineReader` exists to prevent.

So `kTelemetryDecim = 2`: telemetry is emitted every 2nd cycle → 250 Hz,
~340 kbit/s, ~42% utilization with margin for `#` bursts and boot text.
**The control loop still runs at 500 Hz.** 250 Hz is still 12× the plotter's
20 fps redraw.

Do not raise it without also raising `kLinkBaud` in the sketch **and**
`TEENSY_LINK_BAUD` in the XIAO sketch. The edge build's 10 fields fit at
500 Hz and are not decimated.

## `#` lines

`telemetry_python_wifi_corner.py` prints `#`-prefixed lines to its terminal
as firmware console output instead of parsing them, so command echoes, the
`z1` tare readback and arm refusals stay visible in PLOTMODE without
polluting the CSV. The 10-column edge script has no such handling and
suppresses most echoes in PLOTMODE instead.

---

# Flashing — end to end

Two boards, two USB cables, two flashes. Do the XIAO first: once programmed
you never touch it again, and you can verify the WiFi half before the cube is
ever armed.

## ⚠️ UNPLUG THE XIAO'S USB FOR REAL TELEMETRY RUNS

Leaving the XIAO's USB cable attached with **no application reading the
port** collapses the bridge. Measured, not theorised:

```
loop() rate with cable attached, monitor closed:   0.49 iterations/s
```

`pollUdpIn()` and `pollSerial1Out()` each handle exactly **one item per
`loop()` iteration**, so the bridge then relays **0.5 lines/s** — against the
250/s the corner build emits and 500/s from edge. Telemetry appears dead for
reasons that look nothing like the actual cause.

Why: on this chip `if (Serial)` is true whenever the cable is plugged in,
even with nothing draining the buffer, and `printf()` then blocks ~2 s. The
1 s gate on `printLinkStatus()` has always expired by the time it returns, so
it re-blocks every iteration.

**Three ways to be safe, in order of preference:**

1. **Unplug the XIAO's USB and power it from the 5 V rail.** No cable, no CDC
   to stall on. This is the normal operating configuration anyway.
2. Keep a serial monitor **open and reading** — that drains the buffer.
3. Set `ENABLE_LINK_STATUS 0` in the sketch if the cable must stay attached
   unattended.

The sketch now also guards with `availableForWrite()`, so it skips the print
rather than stalling. Belt and braces — rule 1 is still the one to follow.

**Diagnose it with:** `link_check.py` stage 3 — if PONGs trickle back one
every ~2 s instead of immediately, the bridge is in this state.

## ⚠️ 0a. Power sequencing — read before plugging anything in

**Never have cube power and USB connected at the same time** (unless you have
cut the Teensy's VUSB↔VIN pad — see below).

Per [`docs/electronics/Electrical-Design-Guide.md`](../../../../../docs/electronics/Electrical-Design-Guide.md),
both boards are fed from the same LM2596 5 V rail:

```
LM2596 (22 V → 5 V) ──► 5 V RAIL ──┬──► Teensy VIN
                                    └──► ESP32-C6 5 V pin
```

Both `Teensy VIN` and the `XIAO 5V` pin are tied to their board's USB 5 V
internally. Plugging in USB while the rail is live therefore connects the
**laptop's 5 V directly to the LM2596's 5 V** — two stiff sources in
parallel, neither current-limited toward the other. Whichever sits higher
back-feeds the other. That can damage the laptop's USB port, the LM2596, or
the board.

**Correct order, every time:**

| | |
|---|---|
| **Power up** | cube power **OFF** → plug USB → flash → unplug USB → cube power **ON** |
| **Power down** | cube power **OFF** first → *then* USB, if you need it |
| **Wiring** | all power off before touching the D6/D7/GND jumpers. Hot-plugging signal wires back-feeds an unpowered board through its input clamp diodes. GND connects first and disconnects last. |

**If you need USB and cube power simultaneously** (e.g. running the USB
builds off the rail, or watching Teensy boot logs with the motors live), cut
the **VUSB↔VIN trace on the underside of the Teensy 4.1**. PJRC documents
this as the supported way to combine external power with USB. Note the
consequence: after cutting, the Teensy **will not run from USB alone** — it
needs the 5 V rail present to power up at all.

The WiFi builds exist precisely so you don't need both: cube power only, no
cable.

## 0b. Bring-up safety — logic first, motors second

If your e-stop breaks the **motor bus only** and leaves the logic rail live
(which is what the Electrical Design Guide calls for), use that: bring up the
WiFi link with the motor bus dead. `setup()` does not need the moteus to be
powered — CAN3 initialises against the local peripheral, not the bus — so the
Teensy boots, calibrates, resolves its candidate and streams telemetry with
the wheels completely unpowered.

Confirm the whole pipeline that way first. Only close the e-stop once
telemetry is flowing and `a0`/`a1` are acknowledged. Both WiFi builds have
never run on hardware; do not let the first test of the link and the first
test of the motors be the same test.

## 0. One-time toolchain setup

**Teensy 4.1** — Arduino IDE + Teensyduino. Libraries: `MoteusTeensy` (brings
in `ACAN_T4`) and `SparkFun_BMI270_Arduino_Library`.

**XIAO ESP32C6** —
1. *File → Preferences → Additional Board Manager URLs*, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Tools → Board → Boards Manager* → install **esp32** (Espressif).
3. *Tools → Board* → **XIAO_ESP32C6**.
4. *Tools → USB CDC On Boot* → **Enabled** (boot logs only — the operational
   bridge runs entirely over Serial1 + WiFi).

## 1. Wiring (check before powering anything)

```
Teensy Serial1 TX1 (pin 1)  ->  XIAO D7 (RX)
Teensy Serial1 RX1 (pin 0)  <-  XIAO D6 (TX)
GND                          --  GND        <- required, common ground
```

RX/TX crossed, and the common ground is not optional — without it you get
garbage or nothing at all.

## 2. Flash the XIAO

In [`xiao_teensy_bridge/xiao_teensy_bridge.ino`](xiao_teensy_bridge/xiao_teensy_bridge.ino) set:

- `WIFI_SSID` / `WIFI_PASSWORD`
- `XIAO_IP`, `XIAO_GATEWAY`, `XIAO_SUBNET` — a free static address on your
  actual subnet. Static so the PC script has one fixed address, no discovery
  handshake.

⚠️ The credentials currently committed point at a guest network (`ES-Guest`).
Guest/campus networks very often use **client isolation**, which blocks
laptop↔XIAO traffic entirely and breaks this bridge no matter how correct
everything else is. Use a home network or a phone hotspot.

Plug the XIAO in by its own USB-C, select its port, upload. Its Serial Monitor
at 115200 prints connect progress, IP, RSSI, then a `[status]` line each
second.

## 3. Flash the Teensy

Check near the top of whichever sketch you're using:

```cpp
#define TELEMETRY_MODE PLOTMODE     // for the python plotter
static const Axis kAxis = AXIS_X;   // EdgeBalance_WiFi only
```

Plug the Teensy in by **its** USB, select its port (Board: Teensy 4.1),
upload.

## 4. Bring-up sequence

1. Power the moteus / cube. Keep the cube **perfectly still** through boot —
   `setup()` spends ~2 s calibrating gyro bias and bakes in any motion.
2. Confirm the boot lines: `mount DCM check` with det ≈ 1 and all row norms
   ≈ 1, `BMI270 connected!`, and the resolved candidate. If it warns the top
   two candidates are close, re-seat and re-resolve (`c` / `e`) first.
3. On the laptop, set `XIAO_IP` in the matching script, then run it. The XIAO
   only learns the laptop's address once it has heard from it, so telemetry
   starts flowing after the first keepalive lands.
4. Correct the resting tilt — `z1` (corner) or `o<deg>` (edge). **Per axis**
   for edge; it does not carry over.
5. `a1` to arm. It only takes inside the 0.5° gate.
6. `a0` to disarm — **that is the safety net.** Keep a hand on the cube.

### How to tell an arm actually took

The authoritative signal is the **`armed` column** — both scripts print
`Teensy: ARMED` / `DISARMED` whenever it flips. That also catches the control
law self-disarming on a trip (overtilt / overspeed / NaN / link loss). On the
corner script the `# ARM REFUSED …` line is printed to the terminal too.

## If nothing shows up

**Bringing the link up for the first time?** Do not start here — start at
[`link-bringup/`](link-bringup/README.md). Six stages, each isolating one hop,
each with a pass/fail number: boards alive → the Serial1 wire → WiFi join →
UDP both directions → the full pipeline at 250 lines/s with a fake telemetry
source → the same on rail power → the real firmware. Nothing in it can move a
wheel. It also lays out which stages run on USB power and which on the rail,
so cube power and USB are never connected at the same time.

**Already flashed and debugging? Run [`link_check.py`](link_check.py) first.**
It walks the pipeline one hop at a time and names the broken one, instead of
leaving you to infer it from a silent plot window:

```bash
python link_check.py --list-ports          # find the XIAO's COM port
python link_check.py --serial COM10        # <-- the conclusive test
python link_check.py                       # network only, ambiguous
python link_check.py --teensy edge
python link_check.py --ip 192.168.1.42     # override without editing files
```

| Stage | Proves |
|---|---|
| 1. Local | which interface reaches the XIAO, and whether its static IP is even on your current subnet |
| 2. Reachable | ICMP + ARP — is the XIAO on the network at all (ARP is cached, so a hint only) |
| 3. UDP RTT | WIFI_TEST + PING/PONG. **Needs no Teensy** — isolates the WiFi half completely |
| 4. Teensy | bridge mode, then a re-resolve (`c`/`e`), waiting for the `#` answer — the full loop |
| 5. USB | `--serial` only: correlates a measured UDP burst against the XIAO's own `udp_in` counter |

### Always use `--serial` if you can

Without it, total silence is **ambiguous** — your packets may never reach the
XIAO, or its replies may never get back, and from the laptop those look
identical. With the XIAO's USB attached, stage 5 reads its once-a-second
`[status]` line while sending a measured burst, and watches the board's own
`udp_in`. That settles the direction, and it keeps working on a network that
blocks device-to-device traffic entirely — exactly when you most need an
answer.

Wiring for `--serial`: cube power **off** (or the XIAO's `5V` pin lifted off
the rail), XIAO USB-C to the laptop, GND still common. **Close the Arduino
Serial Monitor first** — it holds the port and this cannot share it.

Stage 3 tries **both** `x0` and `x1` and reports which one your board
honours, so it works against old and current bridge firmware alike. A board
that answers on `x1` is running the pre-fix sketch and should be reflashed
from this folder — see below.

It prints and classifies every packet it receives, so wrong-firmware
(10 vs 21 fields) and wrong-mode (tab-delimited = `SERIALMONITORMODE`) show
up as readable data rather than as silence. Close the telemetry script first
if you can; it falls back to an ephemeral port if 4210 is taken.

### ⚠️ Reflash the XIAO — `x0`/`x1` were inverted

`xiao_teensy_bridge.ino` had its mode condition reversed against its own
header and boot message: `x0` selected TEENSY_BRIDGE and `x1` selected
WIFI_TEST, the opposite of what every doc said. Because PING is only answered
in WIFI_TEST, the *documented* command made the round-trip test impossible.

Fixed in both this copy and `firmware/XIAO/`, which are byte-identical again.
**Reflash the XIAO to pick it up.** Until you do, `x0`/`x1` behave backwards
and `link_check.py` will tell you so.

Then work down the pipeline — the XIAO's once-a-second `[status]` line over
USB tells you which stage is broken:

| Symptom on `[status]` | Where the problem is |
|---|---|
| `serial1_lines=0` | Teensy side — wiring (RX/TX swapped, no common GND), the Teensy is in `t0`/USB mode (send `t1`), or it's halted (`p0` to resume). |
| `serial1_lines` climbing, `udp_out=0` | The XIAO has never heard from the laptop. Firewall, wrong `XIAO_IP` in the script, or client isolation. |
| both counts climbing, laptop still blank | PC side: wrong `LOCAL_PORT`, or firewall blocking inbound UDP. |
| `wifi=DOWN` | Credentials, or the network needs a captive-portal login. |

"Got a packet but it isn't N-field PLOTMODE CSV" means either the firmware is
still in `SERIALMONITORMODE`, or you're running the wrong script — 10 fields
is edge, 21 is corner. Each script says which.

## Fallback

If the link misbehaves and you just need to balance: flash the matching
`../USBMODE/` build and use the USB tooling. Same control law, no XIAO
needed. You can also leave the WiFi build flashed and send `t0` to move
telemetry back onto USB live, without a reflash.

## After a run

Both scripts save a timestamped CSV here when you close the plot window.
Plot it with [`../plot_session_csv.py`](../plot_session_csv.py) — see the
top-level README for usage.

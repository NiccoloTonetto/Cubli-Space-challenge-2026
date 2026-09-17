# ota_base — mode state machine + wireless flashing

Teensy 4.1 side of the [console](../../../UI/web/) / [console_host](../../xiao/console_host/). This is a **template**: the
link layer, estimator, pivot geometry, telemetry, state machine and the whole
OTA pipeline are real and work; the two control laws are stubs that command
zero torque.

## Setup

Copy from [FlasherX](https://github.com/joepasquariello/FlasherX) into **this
folder**, next to the `.ino`:

```
FlashTxx.c
FlashTxx.h
```

That's it — FlasherX is source you vendor into a sketch folder, not a library.
The sketch has an `__has_include` guard that fails the build with a pointer
back here if they're missing.

**Do not copy `FXUtil.cpp` / `FXUtil.h`.** Their `update_firmware()` is
interactive: it prompts `enter %d to flash or 0 to abort` on a `Stream` and
waits for a typed reply, which is unusable across a WiFi bridge. `FXUtil.h`
also doesn't export `parse_hex_line()` or `hex_info_t`, so there's nothing to
reuse even if you wanted to. This sketch drives the stable `FlashTxx`
primitives directly (`firmware_buffer_init` / `flash_write_block` /
`check_flash_id` / `flash_move`) and carries its own Intel HEX parser — which
is also where the per-record checksum validation lives.

## What works as-is

Flash it unmodified and you get a genuinely useful instrument:

- real attitude telemetry — the BMI270 path, mounting DCM and gam estimator are
  transplanted verbatim from `CornerBalance_WiFi.ino` SECTION 2b;
- real corner and edge pivot resolution and projection, so `|φ|` and `θ` are
  true numbers;
- real wheel speeds, bus voltage and controller temperature;
- IDLE / EDGE / CORNER switching with full interlocks;
- working E-STOP, arm gate, link watchdog;
- working over-the-air flashing.

It just never commands torque. `CONTROL_LAW_TRANSPLANTED` is `0`, and while it
is, `cornerControlStep()` and `edgeControlStep()` issue query-only CAN frames.

IDLE emits the 21-field corner row with zero torque and `armed = 0`, rather
than going silent — a silent link is indistinguishable from a dead one on
every display in this tree, and the tilt and wheel columns are still true.

## Transplanting a control law

| from | copy |
|---|---|
| `../CornerBalance_WiFi/CornerBalance_WiFi.ino` | `CornerCandidate` **with** its `float Kp[3][9]` and the full `kCorners[8]` table (line 382); all of SECTION 2e — `kMaxTilt`, `kMaxOmega`, `kTauMax`, `kTaperStart`, friction FF constants, `kAxisWheelSign`, `kTorqueFormat`, and `commandWheels()` (line 633) |
| `../EdgeBalance_WiFi/EdgeBalance_WiFi.ino` | `EdgeCandidate` **with** its `float K[3]` and `kCandidates[3][2]` (line 348); its SECTION 2e body, driving only the `kAxis` wheel |

This file keeps only `name` / `gB` / `placeOffsetDeg` from those tables. The
gains are part of the law and belong with the transplant — separating them is
what stops this file from looking like a controller it isn't.

Then set `CONTROL_LAW_TRANSPLANTED` to `1`. Both hooks carry an `#error` that
fires if you flip the flag with a hook still empty, so the two can't drift.

### Three things that will bite

1. **Shared torque state.** Both laws own `gLastTau` / `gLastTauCmd` and both
   were written assuming they're the only law in the binary. `setMode()` zeroes
   them, but a law that caches its own integrator state needs that cleared too.

2. **Silent trips.** The corner law's overtilt / overspeed / NaN trips clear
   `gArmed` themselves and never announce it. The telemetry `armed` **column**
   is the authority, not the `#` echoes — the assumption
   `dashboard/schemas.py:82` already documents. Keep it true.

3. **The arm gate.** This file states one value (`kArmGate`, 1.0°) and reports
   it in its refusal line, which is how the console learns the real threshold.
   Don't let a transplant reintroduce `EdgeBalance_WiFi.ino:484`'s
   `0.1672664619` rad — that's **9.58°**, it looks like a corrupted copy of
   `cubli_gains.h`'s `0.00872664619` (shared `72664619` tail), and its comment
   still claims 0.5°. As flashed, that build arms with the cube leaning nearly
   ten degrees.

## Command grammar

```
m<0/1/2>  mode: IDLE / EDGE / CORNER — always disarms + stops wheels + clears tare
a<0/1>    arm / disarm.  a1 refused in IDLE and above the 1.0° gate
g<0..1>   gain scale
c | e     re-resolve the corner / edge pivot
z<0/1>    tare / clear the corner φ offset
o<deg>    edge φ offset
p<0/1>    halt / resume
t<0/1>    link mode: USB / WiFi  (default WiFi)
h | k     no-op keepalive
u1        begin OTA  (IDLE + disarmed only)
```

`h` is a **keepalive**, not halt — same choice `CornerBalance_WiFi.ino` made,
and for the same reason: the link heartbeat is a bare letter sent ten times a
second, and binding halt to it would halt the cube continuously. Halt is `p`.

## Baud

`kLinkBaud = 1000000`, matching `TEENSY_LINK_BAUD` in the XIAO sketch. Not
115200: a 21-field corner line is ~170 B at 250 Hz ≈ 340 kbit/s, which doesn't
fit — `Serial1` writes would start blocking inside the 2 ms control cycle,
which is the exact failure the non-blocking `LineReader` exists to prevent.
See `CornerBalance_WiFi.ino:33` for the full budget. It also makes OTA ~8×
faster (~6 s for a 570 KB hex instead of ~50 s).

**A mismatch between the two sketches is silent and looks exactly like a dead
link.** Change both or neither.

## Status

Not compiled, not run on hardware. The corner and edge WiFi builds it borrows
from are themselves still marked unvalidated in their own headers. Treat the
first run as a bring-up: cube on the bench, one hand on it, E-STOP in reach,
USB attached so you can watch the Teensy's own console.

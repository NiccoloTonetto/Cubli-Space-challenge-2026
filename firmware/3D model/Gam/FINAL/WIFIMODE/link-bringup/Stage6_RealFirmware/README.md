# Stage 6 — The real firmware, motor bus still dead

**Power:** logic rail on, **motor bus dead** (e-stop open / XT30s unplugged).
**Can anything move?** Not while the motor bus is open. Keep it open for the
whole of this stage.
**New code:** none in this folder — you now flash the real build.

## Why the motor bus stays dead

`setup()` does not need the moteus to be powered: CAN3 initialises against the
Teensy's local peripheral, not the bus. So the Teensy boots, calibrates the
gyro, resolves its corner candidate and streams real 21-field telemetry with
the wheels completely unpowered.

**Neither WiFi build has ever run on hardware.** Do not let the first test of
the link and the first test of the motors be the same test.

## Run

1. Flash [`../../CornerBalance_WiFi/CornerBalance_WiFi.ino`](../../CornerBalance_WiFi/CornerBalance_WiFi.ino)
   (or `EdgeBalance_WiFi` — then everything below is 10 fields at 500 Hz
   instead of 21 at 250 Hz, and the script is `telemetry_python_wifi.py`).
   Check near the top: `#define TELEMETRY_MODE PLOTMODE`.
2. The XIAO keeps the bridge you flashed at Stage 4. Do not touch it.
3. Cube power off → unplug USB → cube power on. Hold the cube **perfectly
   still** through boot: `setup()` spends ~2 s calibrating gyro bias and bakes
   in any motion.
4. `python ../../telemetry_python_wifi_corner.py`

## Pass

**Boot diagnostics** — these arrive as `#` lines in the script's terminal:

- [ ] `mount DCM check`, det ≈ 1, all row norms ≈ 1
- [ ] `BMI270 connected!`
- [ ] a resolved corner candidate. If it warns the top two candidates are
      close, re-seat the cube and re-resolve (`c`) before going further.

**Telemetry** — in the plot window:

- [ ] ~250 lines/s, all four panels live
- [ ] `phi` responds correctly when you tilt the cube by hand
- [ ] `om` responds to rotation and returns to ~0 when still
- [ ] `rho` stays at 0 — nothing is driving the wheels

**Commands** — each must produce a visible answer:

- [ ] `c` re-resolves and echoes the candidate
- [ ] `z1` tares, and `|phi|` drops toward 0 (`z0` clears it)
- [ ] `g0.5` acknowledged
- [ ] `a1` **refuses** unless `|phi|` is inside the 0.5° gate — the refusal
      itself is a pass, it means the gate works
- [ ] `a0` acknowledged, `armed` column stays 0

### Test the watchdog on purpose

With the motor bus still dead: close the telemetry script (or unplug the
laptop's WiFi) and confirm the Teensy self-disarms. In WiFi mode, no line on
Serial1 for **300 ms** auto-disarms, exactly as an `a0` does. This is the
behaviour that protects you when the link drops mid-balance — verify it now,
while nothing can spin.

## If it fails

Everything up to Stage 5 passed, so the transport is a known-good constant.
The fault is in the firmware or the sensors, and the symptom tells you which:

| Symptom | Where to look |
|---|---|
| Telemetry, but no `#` boot lines | You connected after boot. Reset the Teensy with the script already running. |
| `#` lines, no CSV | `TELEMETRY_MODE` is `SERIALMONITORMODE` (tab-delimited), or the board is in `t0`. Send `t1`. |
| 10 fields, not 21 | Edge firmware flashed, corner script running. Match them. |
| `BMI270` not found | SPI wiring / CS on pin 10. Nothing to do with WiFi. |
| DCM det ≠ 1 | The mount matrix, not the link. |
| Rate far below 250/s | Re-run [Stage 4](../Stage4_BridgeThroughput/) with the faker. If the faker is clean and the real build is not, the control loop is overrunning its 2 ms budget — a firmware problem, and now you can prove it. |

## Then — arming

That is deliberately **not** in this folder. Go to
[`../../README.md`](../../README.md) § *Bring-up sequence*, from step 4:
correct the resting tilt, `a1` inside the gate, hand on the cube, `a0` ready.

Close the motor bus **only** once telemetry is flowing and `a0`/`a1` are
being acknowledged.

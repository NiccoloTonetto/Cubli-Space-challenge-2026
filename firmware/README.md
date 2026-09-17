# Firmware

Teensy 4.1 (control, 400 Hz) and XIAO ESP32-C6 (Wi-Fi telemetry bridge), talking
to three mjbots moteus-n1 drivers over CAN-FD and a Bosch BMI270 IMU.

**To actually run the cube, go straight to [`cubli-ui/README.md`](cubli-ui/README.md).**
That folder is self-contained: the flight firmware, the bridge, the dashboard and
every helper. Everything else here is how it was built.

## The bring-up ladders

Every control mode was reached the same way: five stages, each one flashed and
passed on hardware before the next exists. A stage that fails is a stage that
tells you *which* assumption was wrong, because only one thing changed.

```
Stage 1  open-loop torque     does the motor turn the way I think it does?
Stage 2  damping only         can I make it sluggish without making it unstable?
Stage 3  position + damping   does the tilt signal have the sign and scale I expect?
Stage 4  full law             the real controller, hand-held
Stage 5  release              let go
```

| Ladder | Stages | Target |
|---|---|---|
| [`panel-bringup/`](panel-bringup/) | `OpenLoopTorque` → `DampingOnly` → `PositionDamping` → `FullLaw` (+ `_WiFi`) → `Release` | 1-wheel 2D test panel |
| [`cube-bringup/`](cube-bringup/) | `Stage0_SingleMoteusQuery` → `Stage0b_ThreeMoteusLatency` → `Stage0c_IMUJitter` → `Stage0d_Simultaneous` → `Stage4_FullLaw` | 3-wheel platform validation — **do these before any control** |
| [`edge-bringup/`](edge-bringup/) | `WheelSignCheck` → `RateOnly` → `PositionDamping` → `FullLaw` → `Release` | single-wheel edge balance |
| [`corner-bringup/`](corner-bringup/) | `CornerIDAndPulse` → `RateOnly` → `PositionDamping` → Stage 4 family → Stage 5 family | 3-wheel corner balance |
| [`link-bringup/`](link-bringup/) | `BoardsAlive` → `UartLink` → `WiFiJoin` → `UdpEcho` → `BridgeThroughput` → `RailPower` → `RealFirmware` | the Wi-Fi telemetry link, brought up the same way |

The **Stage 0 series** in `cube-bringup/` is worth a look on its own: before any
controller ran on three wheels, it measured three-driver CAN round-trip latency,
IMU sample jitter, and whether three simultaneous torque commands actually land
together. Those are the numbers that set the 400 Hz loop rate and the 3.5 ms delay
budget the [discrete-loop study](../docs/simulation/reports/Discrete-Loop-Test-Block-B.md)
designed against.

### The Stage 4/5 variants in `corner-bringup/`

Corner balance needed more than one Stage 4, and each variant answers a specific
question rather than being a tuning fork:

| Variant | The question it answers |
|---|---|
| `Stage4_FullLaw` | does the per-corner LQR hold at all? |
| `Stage4_AutoTrim` | can the COM offset be learned online instead of measured? |
| `Stage4_FixedOffset` | once trim converges, can adaptation be retired for a constant? |
| `*_RateFilter` | does low-passing the gyro rate help the ~30 Hz structural mode? |
| `Stage4_FixedOffset_Kalman` | does a Kalman rate estimator beat the complementary filter? |
| `Stage5_*` | the same laws, released |

The answers — including the ones that came out **no** — are in
[`docs/testing/`](../docs/testing/). The rate filter in particular delivers about
6 dB, not the 20–30 dB an early firmware comment claimed.

## Other folders

| Folder | What it is |
|---|---|
| [`cubli-ui/`](cubli-ui/) | the consolidated final system — `CubliBalance.ino` fuses edge and corner into one runtime-switchable sketch, plus the Flask dashboard and analysis tools |
| [`imu-calibration/`](imu-calibration/) | BMI270 offset/scale calibration sketch |
| [`xiao/`](xiao/) | early XIAO Wi-Fi experiments and a laptop-side listener |
| [`archive/`](archive/) | superseded builds kept for provenance — the Wi-Fi-mode PreFINAL tree, the abandoned wireless console with OTA flashing, and the first dashboard |

`archive/` exists because several shipped constants cite the run and the build
they came from. It is history, not a menu — nothing there should be flashed.

## Before you touch the cube firmware

Read [`docs/Firmware-Lessons-2D-Panel-to-3D-Cube.md`](../docs/Firmware-Lessons-2D-Panel-to-3D-Cube.md).
It is the condensed list of bugs found on the panel, how they were fixed, and
which parts of the panel's approach do **not** carry over to three wheels and a
real attitude estimate. Two of them are structurally guaranteed to reappear
per-wheel on the cube.

Two standing hazards, both deliberate:

- **Corner mode has no arm gate.** `a1` arms at any tilt. Place the cube near
  balance and keep a hand on it.
- **`cubli_gains.h` is generated.** Regenerate it with `cubli_export_gains.m`
  ([`simulation/cube-3d/export/`](../simulation/cube-3d/export/)) rather than
  editing constants by hand — the stage sketches read their literals from it so
  there is exactly one source of truth.

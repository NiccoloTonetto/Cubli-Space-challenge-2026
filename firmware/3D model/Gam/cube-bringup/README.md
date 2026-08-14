# Cube Bring-Up — Phase 0, gam build

Staged Arduino IDE firmware bringing the 3D cube's CAN and IMU links up to
the point where `Gam/Skeleton_3Axis.ino` can be trusted to run against real
hardware. Not part of the panel's Stage 0-5 progression — those tests were
about the control loop on one axis; these are about whether the three-wheel
CAN bus and the IMU can sustain the loop rate the control law will need,
before any estimator or gain is involved at all.

Driver commissioning (encoder offset, phase order, `v_per_hz`) is assumed
already done via `moteus_tool`/`tview` — none of this re-does that. This is
the application layer: our CAN traffic pattern, our `SetPosition` `Format`
contract, and clean timing on this specific cube's wiring.

## Files, in run order

| File | Tests | Pass criterion |
|---|---|---|
| `Stage0_SingleMoteusQuery/` | One wheel, query/reply/stop/zero-torque | 60 s, zero errors — **run 3x, once per wheel ID**, before Stage0b |
| `Stage0b_ThreeMoteusLatency/` | All 3 wheels, one 400 Hz loop, CAN contention | p99.9 round-trip < 2.0 ms, zero dropped cycles, 10 min |
| `Stage0c_IMUJitter/` | BMI270 at 800 Hz ODR, polled at 400 Hz | zero stale (repeated) samples over 60 s |

CAN and IMU are tested separately here on purpose — combining them is what
`Gam/Skeleton_3Axis.ino` already does, and isolating which subsystem is
responsible for a timing problem is the whole point of a staged bring-up
(see [`Firmware Lessons — 2D Panel to 3D Cube.md`](../../../Firmware%20Lessons%20—%202D%20Panel%20to%203D%20Cube.md) §7).

## What's still open after these pass

- **`Stage0_SingleMoteusQuery`'s `kMoteusId`** must be changed and re-run
  for wheel IDs 2 and 3 — a clean link on one wheel says nothing about the
  other two, different CAN node and different physical wiring run.
- **`Stage0c`'s filter bandwidth (`bwp`)** is deliberately left at the
  library default. Picking the value that lands the cutoff near 100 Hz at
  800 Hz ODR, and reading off that setting's group delay, requires the
  BMI270 datasheet's filter response table — not something firmware
  behavior can determine on its own. TODO is in the file.
- **`kWheelSign[3]`** is not tested by any of these three files — that's
  Phase 1.3, isolated per-wheel pulse tests with the panel firmly held,
  same method as the panel's Stage 1. Do that before Phase 2.

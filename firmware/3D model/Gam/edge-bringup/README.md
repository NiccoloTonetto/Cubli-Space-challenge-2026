# Edge Bring-Up — axis-selectable (X / Y / Z)

Staged Arduino IDE firmware for bringing the cube up to closed-loop edge
balance, one wheel at a time, same staged shape as the panel's
`2D model/panel-bringup/Stage0-5` — see
[`Firmware Lessons — 2D Panel to 3D Cube.md`](../../../Firmware%20Lessons%20—%202D%20Panel%20to%203D%20Cube.md)
§7 for why that progression is worth repeating "nearly unchanged" rather
than jumping straight to a closed loop.

Started ahead of Phase 4 (corner) because edge only ever needs one moteus
powered — full corner balance needs all three wheels soldered to the
battery.

Every stage now carries an `enum Axis { AXIS_X, AXIS_Y, AXIS_Z }` and a
single `static const Axis kAxis = ...;` switch near the top (Objects and
Settings section). Change that one line, re-upload, and the whole file —
moteus CAN id, edge-candidate table, wheel sign, boot-time confirmation
print — follows automatically:

```cpp
enum Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static const Axis kAxis = AXIS_Y;   // <<< CHANGE THIS to test X or Z
```

Physically confirmed wheel/axis/CAN-id mapping (bench-verified, see
`Gam/Skeleton_3Axis.ino`'s mapping comment): **id 2 = X, id 3 = Y, id 1 = Z**.
`kAxisMoteusId[3] = { 2, 3, 1 }` encodes this once; nothing else in these
files should hardcode a moteus id.

## Files

| File | Motor can move? | What it adds |
|---|---|---|
| `Stage1_WheelSignCheck/` | Yes — fixed pulse only | First real torque command to the active wheel; determines that axis's `kWheelSign` and, as a side effect, which edge candidate (`[+1,+1]` vs `[-1,-1]`) is physically under the cube |
| `Stage2_RateOnly/` | Yes — closed loop | Rate feedback only (`K[1]`/`om_edge`), full safety scaffold introduced |
| `Stage3_PositionDamping/` | Yes — closed loop | Adds position feedback (`K[0]`/`phi_edge`), gain ramped 0.1→1.0 by hand |
| `Stage4_FullLaw/` | Yes — closed loop | Adds momentum management (`K[2]`), friction feedforward, the real arm gate (0.5°), live `kPhiOffset` (`o<deg>`), velocity cap removed for this stage only |
| `Stage5_Release/` | Yes — closed loop, unsupported | Real `DISARM`/`OMEGA_CAP` trip policy (velocity cap restored to 40 rad/s), latched trip-reason, the actual balancing attempt |

## What's genuinely new here, not copied from the panel

The panel's `atan2` estimator trick is 2D/45°-mount specific and does not
apply — the cube tips in a real 3D way even with only one wheel actuating.
Every stage carries the same reduced-attitude (`ghat`/`w_b`) estimator
`Gam/Skeleton_3Axis.ino` uses, projected onto the single active edge's
direction (`phi_edge`, `om_edge`), not a different filter per axis. Edge is
a 3-state *projection* of the same estimator the corner build uses, not a
separate one — swapping `kAxis` only changes which edge direction (`e`) and
`gB` pair that projection uses, not the estimator itself.

**One deliberate difference from `Gam/Skeleton_3Axis.ino` as it stands
today:** these files use the validated `kP=4`/`kI=0.5` cross-product
complementary filter from `Attitude representation for the firmware.md` /
`cubli_gains.h` directly. `Gam/Skeleton_3Axis.ino` still runs the LERP-blend
filter that was flagged as needing replacement before any real closed loop
(it's past the measured `kP` cliff, just never triggered there because
`commandWheels()` is a zero-torque stub). Edge balance *is* a real closed
loop, so it gets the correct filter now. `Gam/Skeleton_3Axis.ino` needs the
same swap separately — not done by these files.

## Which edge, per axis

`cubli_gains.h` gives each axis two edge candidates, same edge direction
`e` but different `gB` (which pair of transverse corners is up):

| Axis | `e` | Candidate A | Candidate B |
|---|---|---|---|
| X (id 2) | `[1,0,0]` | `X[+1,+1]` (+Y,+Z up, 0.758°) | `X[-1,-1]` (-Y,-Z up, 0.837°) |
| Y (id 3) | `[0,1,0]` | `Y[+1,+1]` (+X,+Z up, 0.006°) | `Y[-1,-1]` (-X,-Z up, 0.007°) |
| Z (id 1) | `[0,0,1]` | `Z[+1,+1]` (+X,+Y up, 0.764°) | `Z[-1,-1]` (-X,-Y up, 0.844°) |

Stored as `kCandidates[3][2]`, indexed `[kAxis][gEdgeIdx]`. Every stage
resolves the second index **by measurement, not memory** — at startup, it
compares the measured `ghat` against both of the active axis's candidate
`gB` vectors and picks whichever one it actually agrees with, printing the
answer and a warning if the two are too close to call. Re-run with the `e`
serial command any time.

## Hardware status

All three wheels (X, Y, Z) have been run through the staged bring-up and
balance successfully on their respective edges. `kAxisWheelSign[3]` is
`+1.0f` across the board and confirmed for all three via each axis's own
Stage 1 pulse test — no sign flip was needed on this build. Re-verify per
axis rather than assuming it still holds if the mount or wiring changes
(Firmware Lessons S4).

## What's still open

- **IMU calibration** is still the 2D panel's numbers, copied forward with
  the same TODO as `Gam/Skeleton_3Axis.ino`. Telemetry-only risk in Stage 1
  (no feedback loop yet); a real accuracy problem from Stage 2 onward.
- **`Gam/Skeleton_3Axis.ino`'s own estimator** still needs the LERP→kP/kI
  filter swap these files already carry — separate, not done by this folder.

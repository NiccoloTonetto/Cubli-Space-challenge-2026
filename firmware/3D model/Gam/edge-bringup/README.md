# Edge Bring-Up — Y edge, moteus id 3

Staged Arduino IDE firmware for bringing the cube up to closed-loop edge
balance, one wheel (Y, id 3), same staged shape as the panel's
`2D model/panel-bringup/Stage0-5` — see
[`Firmware Lessons — 2D Panel to 3D Cube.md`](../../../Firmware%20Lessons%20—%202D%20Panel%20to%203D%20Cube.md)
§7 for why that progression is worth repeating "nearly unchanged" rather
than jumping straight to a closed loop.

Started ahead of Phase 4 (corner) because only one moteus (id 3, physically
confirmed as the Y-axis wheel) has power right now — full corner balance
needs all three wheels soldered to the battery. Edge only ever needed one.

## Files

| File | Motor can move? | What it adds |
|---|---|---|
| `Stage1_YWheelSignCheck/` | Yes — fixed pulse only | First real torque command to this wheel; determines `kWheelSign` and, as a side effect, which Y-edge (`Y[+1,+1]` vs `Y[-1,-1]`) is physically under the cube |
| `Stage2_RateOnly/` | Yes — closed loop | Rate feedback only (`K[1]`/`om_edge`), full safety scaffold introduced |
| `Stage3_PositionDamping/` | Yes — closed loop | Adds position feedback (`K[0]`/`phi_edge`), gain ramped 0.1→1.0 by hand |
| `Stage4_FullLaw/` | Yes — closed loop | Adds momentum management (`K[2]`), friction feedforward, the real arm gate (0.5°) |
| `Stage5_Release/` | Yes — closed loop, unsupported | Real `DISARM`/`OMEGA_CAP` trip policy, latched trip-reason, the actual balancing attempt |

## What's genuinely new here, not copied from the panel

The panel's `atan2` estimator trick is 2D/45°-mount specific and does not
apply — the cube tips in a real 3D way even with only one wheel actuating.
Every stage carries the same reduced-attitude (`ghat`/`w_b`) estimator
`Gam/Skeleton_3Axis.ino` uses, projected onto the single edge direction
(`phi_edge`, `om_edge`), not a different filter. Edge is a 3-state
*projection* of the same estimator the corner build uses, not a separate one.

**One deliberate difference from `Gam/Skeleton_3Axis.ino` as it stands
today:** these files use the validated `kP=4`/`kI=0.5` cross-product
complementary filter from `Attitude representation for the firmware.md` /
`cubli_gains.h` directly. `Gam/Skeleton_3Axis.ino` still runs the LERP-blend
filter that was flagged as needing replacement before any real closed loop
(it's past the measured `kP` cliff, just never triggered there because
`commandWheels()` is a zero-torque stub). Edge balance *is* a real closed
loop, so it gets the correct filter now. `Gam/Skeleton_3Axis.ino` needs the
same swap separately — not done by these files.

## Which Y edge

`cubli_gains.h` has two Y-axis edges, same edge direction (`e=[0,1,0]`) but
different `gB` (which pair of transverse corners is up): `Y[+1,+1]`
(+X,+Z up, 0.006° placement offset) and `Y[-1,-1]` (-X,-Z up, 0.007°).
Every stage resolves this **by measurement, not memory** — at startup, it
compares the measured `ghat` against both candidate `gB` vectors and picks
whichever one it actually agrees with, printing the answer and a warning if
the two are too close to call. Re-run with the `e` serial command any time.

## What's still open

- **`kWheelSign`** is a placeholder (`+1.0f`) in every stage until Stage 1
  determines it — fix it in Stages 2-5 by hand once known, same as the
  panel required.
- **IMU calibration** is still the 2D panel's numbers, copied forward with
  the same TODO as `Gam/Skeleton_3Axis.ino`. Telemetry-only risk in Stage 1
  (no feedback loop yet); a real accuracy problem from Stage 2 onward.
- **`Gam/Skeleton_3Axis.ino`'s own estimator** still needs the LERP→kP/kI
  filter swap these files already carry — separate, not done by this folder.

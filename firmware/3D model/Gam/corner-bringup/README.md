# Corner Bring-Up — three wheels, one corner at a time

Staged Arduino IDE firmware for bringing the cube up to closed-loop corner
balance, all three wheels live at once, same staged shape as
[`edge-bringup`](../edge-bringup/README.md) and the panel's
`2D model/panel-bringup/Stage0-5` before it — see
[`Firmware Lessons — 2D Panel to 3D Cube.md`](../../../Firmware%20Lessons%20—%202D%20Panel%20to%203D%20Cube.md)
§7 for why that progression is worth repeating "nearly unchanged" rather
than jumping straight to a closed loop.

Started once `cube-bringup`'s Stage0/0b/0c all passed with all three
wheels powered simultaneously (bench power supply, not the flight battery
yet) — this is the corner-balance follow-on `edge-bringup`'s own NOTES
pointed at once the other two wheels were live.

## Why this is a bigger jump than edge-bringup

Edge balance is single-wheel: one active wheel, a 3-state law
(`phi_edge`, `om_edge`, `rho_axis`), independent scalar gains. Corner
balance is genuinely coupled: **all three wheels contribute to every
wheel's torque command.** Each wheel's row of `cubli_gains.h`'s per-corner
`Kp[3][9]` dots against the FULL 9-state vector
`[phi(3); om(3); rho(3)]` — there is no independent per-axis gain to
tune one wheel at a time the way edge-bringup could. What IS still
per-axis: each wheel's `kWheelSign` (a wiring-convention fact, carried
forward from edge-bringup, all three already confirmed `+1.0f` there) and
which of the 8 corners is currently down (resolved by measurement, same
discipline as edge-bringup's per-axis candidate resolution, just 8-way
instead of 2-way).

## The state-masking trick

Rather than five files each carrying a DIFFERENT slice of `Kp` (risking a
transcription mismatch between them), every stage in this folder carries
the exact SAME `kCorners[8]` table, copy-paste identical. What changes
per stage is which entries of the 9-wide state vector `x` are zeroed
before the `Kp . x` dot product:

| Stage | phi (cols 0-2) | om (cols 3-5) | rho (cols 6-8) |
|---|---|---|---|
| 2 (Rate Only) | zero | live | zero |
| 3 (Position Damping) | live | live | zero |
| 4 (Full Law) | live | live | live |
| 5 (Release) | live | live | live |

Stage 1 has no closed loop at all (open-loop pulse only). This is the same
idea as edge-bringup's `K[0]=K[2]=0` in its scalar law, generalized to a
coupled matrix by masking the STATE instead of the GAIN.

## Files

| File | Motors move? | What it adds |
|---|---|---|
| `Stage1_CornerIDAndPulse/` | Yes — fixed pulse, one wheel at a time | Corner identification (8-way, by measurement); per-wheel pulse test re-confirms `kWheelSign` under the new state framing and that all three wheels commanded together (even with two at zero torque) behaves cleanly |
| `Stage2_RateOnly/` | Yes — closed loop | om block only, full safety scaffold introduced (single `gArmed` for all three wheels) |
| `Stage3_PositionDamping/` | Yes — closed loop | Adds phi block, gain ramped 0.1→1.0 by hand |
| `Stage4_FullLaw/` | Yes — closed loop | Adds rho block (momentum management), friction feedforward per wheel, real arm gate (0.5° on `norm3(phi)`) |
| `Stage5_Release/` | Yes — closed loop, unsupported | Real `DISARM` (15° on `norm3(phi)`) / `OMEGA_CAP` (40 rad/s per wheel) trip policy, latched trip-reason (now also names WHICH wheel tripped an omega limit), the actual balancing attempt |

## `cubli_gains.h` and the attitude doc are now real files

Both [`../cubli_gains.h`](../cubli_gains.h) and
[`../Attitude representation for the firmware.md`](../Attitude%20representation%20for%20the%20firmware.md)
are checked into the repo as of this folder — previously they were only
referenced by name in edge-bringup's comments. `cubli_gains.h` is the
single source of truth every stage file's literal `kCorners`/`kCandidates`
tables are copied from; if the plant/LQR design ever changes, regenerate
that file and re-copy the relevant table into each `.ino`, the same
manual-sync workflow edge-bringup already established (Arduino sketches in
this environment can't reliably `#include` a header outside their own
sketch folder — see `Firmware Lessons` on the IDE's sketchbook-resolution
quirks — so literal duplication per file is the deliberate, tested choice,
not an oversight).

## Which corner

`cubli_gains.h` has all 8 corners (`[±1,±1,±1]`), each with its own `gB`,
full `Kp[3][9]`, and `theta_eq` (lean vs. the body diagonal, comparable to
edge-bringup's `place_offset`):

| corner | lean (deg) | corner | lean (deg) |
|---|---|---|---|
| `[+1,+1,+1]` | 0.714 (best) | `[+1,-1,+1]` | 2.609 |
| `[-1,-1,-1]` | 0.797 | `[-1,+1,-1]` | 2.773 |
| `[-1,+1,+1]` | 3.097 | `[+1,+1,-1]` | 3.095 |
| `[+1,-1,-1]` | 3.171 | `[-1,-1,+1]` | 3.170 |

Every stage resolves which one is down **by measurement, not memory** —
at startup (and on the `c` serial command any time), it compares the
measured `ghat` against all 8 candidate `gB` vectors and picks whichever
one it actually agrees with best, printing the answer, the runner-up, and
a warning if the two are too close to call.

## What's still open

- **Only ONE corner has hardware data at all, and even that is zero so
  far** — this folder has not yet been run on real hardware. Do not treat
  Stage 1 passing on one corner as validating any other; run the full
  Stage 1→5 sequence per corner, same discipline edge-bringup established
  per-axis (Firmware Lessons S4).
- **No live phi-offset correction** (unlike edge-bringup's `kPhiOffset`) —
  that was added there only after a specific measured resting-point shift
  on the Y edge (missing battery cable/DC-DC). Add the 3-vector equivalent
  here only if the same problem actually shows up on a corner, not
  preemptively.
- **Stage 4's velocity cap is NOT removed by default**, unlike
  edge-bringup's Stage 4 — corner commands three wheels at once, so that
  call is left to be made explicitly, per corner, if the taper is
  observed choking spin-up torque (see Stage 4's header for the precedent
  and reasoning).
- **Friction feedforward (`kTauCw`/`kBw`) is one placeholder pair shared
  across all three wheels** — plausible for three copies of the same
  motor/mount, but unverified; a real per-wheel spin-down test would
  confirm or correct it.
- **IMU calibration** is still the 2D panel's numbers, copied forward with
  the same TODO as `Gam/Skeleton_3Axis.ino` and `edge-bringup`.
- **`Gam/Skeleton_3Axis.ino`'s own estimator** still needs the LERP→kP/kI
  filter swap this folder (and edge-bringup) already carries — separate,
  not done by these files.
- **Still on the bench power supply, not the flight battery** — expect
  `Sg`/`lambda` (and therefore how well the current `Kp` performs) to be
  somewhat off from the final mass distribution; re-derive gains once the
  real battery/DC-DC are mounted, same measure→derive→re-tune workflow as
  everywhere else in this project.

---
tags:
  - space-challenge
  - sofia
  - cubli
  - firmware
  - teensy
  - moteus
  - lessons-learned
---

# Firmware Lessons — 2D Panel to 3D Cube

Everything in [`2D model/panel-bringup/`](2D%20model/panel-bringup/) works end to end on hardware now — the
2D planar panel balances. This note exists so the 3D cube doesn't re-pay the
cost of finding the same bugs twice. Two classes of content: things that
generalize directly to every future wheel/axis (fix once, remember forever),
and things that don't generalize at all and need deliberate rethinking before
the cube starts (flagged explicitly in §7).

Companion to [`2D model/panel-bringup/Bring-Up Stages — Implementation Notes.md`](2D%20model/panel-bringup/Bring-Up%20Stages%20—%20Implementation%20Notes.md),
which has the full blow-by-blow. This is the condensed, forward-looking version.

---

## 1. Toolchain and libraries

- **Arduino IDE**, not PlatformIO — each stage is a self-contained sketch
  (a folder containing one `.ino` of the same name).
- **`mjbots/moteus-arduino`**, included as `#include <MoteusTeensy.h>` — the
  Teensy CAN-FD backend of the official moteus Arduino library, built on
  `ACAN_T4` for the Teensy 4.1's onboard CAN3.
- **`SparkFun_BMI270_Arduino_Library`** for the IMU, over SPI (not I2C —
  faster, and frees up the I2C bus).
- CAN-FD bus: 1 Mbps arbitration (`ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1)`),
  Teensy pins 30/31 (CAN3 — the only CAN-FD-capable bus on the Teensy 4.1).

## 2. The moteus command gotcha that cost the most time

`moteus1.SetPosition(cmd)` — called with **no second argument** — does not
send the whole `Command` struct. It uses a default `Format` that only
transmits `position` and `velocity`. Every other field (`feedforward_torque`,
`kp_scale`, `kd_scale`, `maximum_torque`, `watchdog_timeout`,
`ignore_position_bounds`) defaults to `Resolution::kIgnore` and is silently
dropped **before the CAN frame is built** — set in the local C++ struct,
never transmitted, no error, no warning.

Practical effect: a "torque mode" command built the textbook way (`kp_scale
= kd_scale = 0`, `feedforward_torque = tau`) actually becomes a plain
**velocity-mode hold at 0**, because the moteus never hears about the scale
overrides and falls back to its own onboard `kp_scale=1`/`kd_scale=1`. This
is exactly why "velocity mode works, torque mode does nothing" was our
first real symptom — velocity mode was, unknowingly, the *only* thing ever
actually being sent.

**Fix — build once, reuse every cycle, pass explicitly:**

```cpp
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque     = Moteus::kFloat;
  f.kp_scale                = Moteus::kFloat;
  f.kd_scale                 = Moteus::kFloat;
  f.maximum_torque           = Moteus::kFloat;
  f.watchdog_timeout         = Moteus::kFloat;
  f.ignore_position_bounds   = Moteus::kFloat;
  return f;
}();
...
moteus1.SetPosition(cmd, &kTorqueFormat);   // the &kTorqueFormat is not optional
```

**This generalizes exactly as-is to every wheel on the cube** — same
`Format`, same fields, one per moteus instance. Easy to forget per-instance
if the cube code isn't structured to share it.

## 3. `ignore_position_bounds` — needed for any continuously-spinning wheel

moteus's defaults assume a bounded joint. `servopos.position_min`/`_max`
get checked on **every transition into Position mode**, and a reaction
wheel — which has no natural travel limit and is expected to accumulate many
rotations — will eventually violate them, faulting with code 39 ("outside
limit"). The servo then sits in Fault mode indefinitely: **it keeps
replying to CAN queries normally** (position/velocity telemetry looks
fine), it just silently stops applying any commanded torque. This looks
exactly like "everything reports healthy but nothing happens" — the same
signature the `Format` bug produces, for a completely different reason.
Set `cmd.ignore_position_bounds = 1.0f` (with the matching `Format`
resolution) on every wheel, always.

**Diagnostic worth keeping around:** read and print `Query::Result::mode`
and `::fault` (registers `0x000`/`0x00f`) in telemetry. We didn't have this
at first and it cost real debugging time — mode `1` = FAULT, and the fault
code tells you exactly why (`39` outside limit, `36` motor never
calibrated via `moteus_tool`, `34`/`40` over/under voltage). Cast both to
`(int)` before printing — `fault` is `int8_t`, which `Serial.print()` can
silently bind to its `print(char)` overload instead of the numeric one.

## 4. The wheel-mounting sign convention — will need re-doing per wheel

The LQR model's convention (`Θ̄θ̈ = Sg·sinθ − u`) requires a positive
commanded wheel torque to produce a *restoring* reaction on the body. There
is no way to know in advance whether a given physical motor/wheel mounting
satisfies that convention or its exact opposite — it depends on which way
the motor happens to be wired and mounted, and calibration doesn't fix it
(mjbots' `moteus_tool --calibrate` only ties phase order to encoder
direction *internally consistently*, it says nothing about which way that
is relative to your airframe).

We found ours was backwards — confirmed empirically, not assumed — via a
**deliberately isolated open-loop test**: hold the panel firmly, fire a
single small fixed torque pulse with no feedback loop involved, and
physically watch which way the panel pushes. Fixed with **one constant**,
applied at the hardware boundary, gains left completely untouched:

```cpp
static const float kWheelSign = -1.0f;
...
cmd.feedforward_torque = kWheelSign * tau;
...
const float wheel_omega = kWheelSign * v.velocity * 2.0f * (float)PI;
```

Both sides (outgoing torque *and* incoming velocity) need the same
constant wherever the code has a taper or a momentum-management term that
depends on `tau` and `wheel_omega` staying mutually consistent — flipping
only one side fixes the panel-reaction direction while breaking that
internal consistency.

**Does not generalize automatically to the cube.** Three (or more) wheels
means three independent physical mountings, and there is no reason to
assume they share a sign — verify each wheel's `kWheelSign` independently,
with the same isolated single-pulse test, before ever combining them in a
closed loop. Assuming symmetry here is exactly the kind of shortcut that
turns into an unexplainable multi-axis runaway that's much harder to
isolate than a single-axis one.

**One firmware rule that made isolating both of the above tractable, worth
keeping as a hard rule going forward:** never flip a sign to compensate for
unexpected behavior without first proving *which* layer is wrong via an
isolated, feedback-free test. `K1`/`K2`/`K3` (or their cube-scale
equivalents) should never be hand-flipped — every sign in the gains traces
back to a `Q`/`R`/plant choice; the mounting/estimator convention is a
separate, physical fact that gets corrected in exactly one place.

## 5. Estimator (Stage 1 complementary filter)

```cpp
const float theta_acc = atan2f(imuData.ax - imuData.ay,
                               imuData.ax + imuData.ay) - kThetaOffset;
```

Works specifically because the IMU sits at 45° to the panel's rotation
axis: `ax - ay ∝ sin(theta)`, `ax + ay ∝ cos(theta)`, the ratio cancels `g`
so calibration scale doesn't matter, and `atan2` stays singularity-free
over the full ±180°. **This exact trick is 2D/45°-mount specific** — it
does not extend to a 3-axis attitude estimate. See §7.

- `kThetaOffset`: mount angle is never exactly the design value; this gets
  measured on the bench (balance by hand at true vertical, read the raw
  angle, that's the offset) and is live-settable over serial rather than
  baked in, specifically so recalibration doesn't cost a reflash.
- `kTau = 1.0` s (not the more intuitive-looking 0.5): the filter's
  error has a U-shaped optimum in `tau` — too short and lever-arm/
  accelerometer noise leaks through, too long and residual gyro bias
  dominates (`error ≈ bias × tau`). Our measured gyro bias was small
  (0.0005–0.0023 rad/s), which pushes the optimum up, not down. Re-measure
  this for the cube's actual IMU/gyro rather than assuming 1.0 s carries
  over — it's a function of the specific sensor's bias, not the mechanism.
- Accelerometer gate at `|a|` within ±15% of 1g before trusting the
  accel-derived angle — rejects the systematic error where wheel reaction
  torque injects tangential acceleration that isn't gravity. Keep this
  loose, not tight: the contamination is roughly perpendicular to gravity
  and barely moves `|a|`, so a tight gate catches little while risking
  rejecting good samples.
- Gyro bias subtraction, applied per-axis before anything else touches the
  raw reading. Small (sub-0.01 rad/s) but real, and free to correct once
  measured.

## 6. Real actuator limits vs. modeled ones

- Real static friction/stiction on this rig needed **≥0.05 N·m** before
  producing any visible wheel motion — confirmed empirically, not
  predicted. `0.01 N·m` (a seemingly-reasonable "deliberately small, safe"
  first test pulse) sat below that threshold and looked exactly like a
  dead command path even with everything else working correctly. Size the
  cube's first open-loop test pulses with real margin above whatever
  stiction turns out to be for its actuators, and don't conclude "broken"
  from a null result at a torque that was never going to move anything.
- **Sanity-check gain magnitude against actuator capability before ever
  arming.** `K_t` and `servo.max_current_A` give a real, computable ceiling
  on deliverable torque (`τ_max ≈ K_t × I_max`). A gain vector where a
  fraction of a degree of tilt alone would already demand more torque than
  that ceiling means the "controller" is actually running open-loop
  bang-bang against noise, not proportional LQR — regardless of how
  correctly the rest of the pipeline is wired. Worth computing this ratio
  explicitly for each cube wheel's actual gains before first power-on, not
  after something looks wrong.
- Reaction-wheel torque and panel push are coupled by Newton's third law
  *the instant current flows*, independent of whether the rotor actually
  overcomes friction and turns. A correction that produces no visible
  wheel motion but *is* physically pushing against your hand is working
  correctly, not silently failing — don't use "does the wheel spin" as the
  only signal when isolating gain terms by hand.

## 7. Staged bring-up — the methodology, not just the specific stages

The concrete stages (`2D model/panel-bringup/Stage0`…`Stage5`) are 2D-specific, but
the *shape* of the progression is exactly what caught both bugs in §2 and
§4 cleanly instead of as an unexplainable tangle, and is worth repeating
for the cube nearly unchanged:

1. **Estimator-only, motor structurally incapable of moving** (not just
   commanded zero — no code path that *can* send nonzero torque). Prove
   the sensor signs and filter before any actuation risk exists.
2. **A single, small, fixed open-loop pulse, held by hand, no feedback
   loop.** The only stage that can isolate the mounting/reaction sign
   convention (§4) without a closed loop's dynamics confounding the
   result.
3. **One gain term at a time**, closed loop, still hand-held. A sign error
   here fights your hand, not gravity — cheap to catch, cheap to recover
   from.
4. **Combined terms, gain ramped from low to full authority manually**
   between runs — a residual sign or scale error shows up as "this feels
   backwards at low gain," not a runaway at full gain.
5. **Full authority, unsupported**, only after all of the above passed.

Safety scaffold that made this survivable and worth carrying forward
unchanged in spirit:
- **Latching trips, never auto-clearing** — tilt limit, wheel-speed limit,
  NaN/non-finite guard. Re-arming is always a deliberate action.
- **NaN guard strictly before any clamp** — a clamp alone lets NaN through
  silently (every comparison against NaN is false).
- **Taper that fades spin-up torque only, never braking** — approaching a
  wheel-speed cap should never remove your only way to *stop*.
- **A halt command independent of arm/disarm** — closing a serial monitor
  does not stop a running microcontroller; something explicit has to.

For the cube specifically: don't jump from "one wheel/axis isolated and
working" straight to "three wheels closed loop simultaneously." Isolate
each wheel's sign convention and each axis's estimator independently
first, exactly as this progression did for one.

## 8. LQR retuning workflow (unchanged in principle for 3D)

Two different activities, easy to conflate:

- **Re-deriving `K` from a better plant** (do this first, always): measure
  the real `Theta`/inertia from a free-swing period test on the *built*
  hardware (`Θ̂ = Sg·T²/4π²` for the 2D case), update the plant model,
  re-run `lqr(A, B, Q, R)`. Not tuning — just a more accurate derivation.
- **Adjusting behavior via the Bryson weights** (`theta_max`, `rate_max`,
  `omega_des`, `rho_lqr`), never by hand-editing `K` values directly.
  `rho_lqr` is the one master speed-vs-effort knob; the sweep in
  `cubli_lqr_design.m` CHECK 9 already does this systematically rather
  than by guessing one value at a time.

For the cube: the plant matrices themselves change completely (3D rigid-body
dynamics, not the 1D `Θ̄θ̈ = Sg sinθ − u` reduction) — see
[`../docs/dynamics/3D-Cubli-Lagrangian-Derivation.md`](../docs/dynamics/3D-Cubli-Lagrangian-Derivation.md).
The *workflow* (measure → derive → simulate → validate on hardware →
re-measure) carries over; none of the actual numbers do.

## 9. Verify published numbers against the code that produced them

The closed-loop pole figures quoted in two different docs for this project
(`"about -15, -9, -6"` and, separately, `"-9.72 ± 0.76j, -5.05"`) were both
wrong — neither matched `eig(A - B*K)` run against the actual deployed `K`
and the measured-mode plant, which gives **≈ -12.5, -9.7, -6.3 rad/s, all
real**. `K` itself was correct; only the documented poles were stale/mis-
transcribed. Small thing, but a reminder: derived numbers drift from the
code that produces them, especially across multiple doc revisions. Worth
independently re-deriving anything load-bearing (pole locations, margins,
predicted envelopes) from the actual current code before trusting a number
quoted in prose, rather than propagating it forward another time.

## 10. What genuinely does NOT carry over to 3D — explicit list

- **The 45°-atan2 estimator trick (§5)** is mechanically specific to a
  single-axis panel with the IMU at 45° to the pivot. The cube needs a
  real attitude estimator — [`Panel Controller Workflow.md`](../docs/simulation/Panel-Controller-Workflow.md)
  already documents the intended upgrade path (complementary filter →
  scalar Kalman filter → MEKF, MEKF specifically deferred *until* the cube
  stage). See [`../docs/dynamics/Quaternions-Complete-Guide.md`](../docs/dynamics/Quaternions-Complete-Guide.md)
  for the representation.
- **One wheel, one `kWheelSign`, one set of trip thresholds** → three (or
  more) independent wheels, independent CAN IDs, independent sign
  verification per wheel (§4), and a 3D-appropriate trip condition (an
  attitude-error norm or per-axis limits — not yet decided, worth deciding
  deliberately rather than defaulting to "just check each Euler angle
  separately," which reintroduces gimbal-lock-shaped problems the
  quaternion representation exists to avoid).
- **The gains themselves** — `K1`/`K2`/`K3` for the panel are meaningless
  for the cube; full re-derivation per §8, from 3D dynamics per §7 of the
  Lagrangian derivation doc.
- **`kMaxTilt` as a single scalar** — was tied directly to the 2D Build
  Guide's recovery-envelope prediction; the cube's equivalent envelope
  will come from the 3D simulation work, not from picking a number that
  feels similar.

## Related

- [`2D model/panel-bringup/README.md`](2D%20model/panel-bringup/README.md) — the 2D firmware itself
- [`2D model/panel-bringup/Bring-Up Stages — Implementation Notes.md`](2D%20model/panel-bringup/Bring-Up%20Stages%20—%20Implementation%20Notes.md) — full per-stage code walkthrough
- [`2D model/panel-bringup/Arduino Bring-Up Plan — Sections 2b and 2d.md`](2D%20model/panel-bringup/Arduino%20Bring-Up%20Plan%20—%20Sections%202b%20and%202d.md) — the original procedure
- [`../docs/simulation/Panel-Controller-Workflow.md`](../docs/simulation/Panel-Controller-Workflow.md) — plant/LQR/retuning pipeline, MEKF upgrade path
- [`../docs/dynamics/1D-Jig-to-3D-Cube-Strategy.md`](../docs/dynamics/1D-Jig-to-3D-Cube-Strategy.md) — the staged 2D→3D plan this note feeds into
- [`../docs/dynamics/3D-Cubli-Lagrangian-Derivation.md`](../docs/dynamics/3D-Cubli-Lagrangian-Derivation.md) — the 3D dynamics the cube's `A`/`B` will come from

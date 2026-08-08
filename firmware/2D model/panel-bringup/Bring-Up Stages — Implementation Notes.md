---
tags:
  - space-challenge
  - sofia
  - cubli
  - firmware
  - teensy
  - arduino
  - panel
  - controls
  - estimator
  - bring-up
---

# Bring-Up Stages — Implementation Notes

What each of the six sketch files actually does, and how, at the code level.
Companion to [[Arduino Bring-Up Plan — Sections 2b and 2d]] (the *why* and
the procedure) and [[Simscape Panel Model — Build Guide]] (the plant and the
gains). This note documents the *implementation* — read it when you need to
know exactly what a line of firmware does, not why the stage exists.

---

## 1 — File map

| File | Type | Motor can move? | Introduces |
|---|---|---|---|
| `test_control.txt` | Stage 0 | **No** — hard-wired off | loop rate, ODR, offset tuning, preview-only control law |
| `Stage1_OpenLoopTorque/Stage1_OpenLoopTorque.ino` | Stage 1 | Yes — fixed pulse only | torque-mode command structure |
| `Stage2_DampingOnly/Stage2_DampingOnly.ino` | Stage 2 | Yes — closed loop | full safety scaffold: latches, taper, arm/disarm |
| `Stage3_PositionDamping/Stage3_PositionDamping.ino` | Stage 3 | Yes — closed loop | k1, ramped gain scale |
| `Stage4_FullLaw/Stage4_FullLaw.ino` | Stage 4 | Yes — closed loop | k3, standing-speed low-pass |
| `Stage5_Release/Stage5_Release.ino` | Stage 5 | Yes — closed loop, unsupported | latched trip-reason readout |

Stages 1–5 each live in a folder named after the `.ino` file — that's an
Arduino IDE requirement (a sketch must be a folder containing one `.ino` of
the same name), not a style choice. `test_control.txt` stays a loose file at
the top level, as it was before this rewrite.

Every file is **self-contained**: includes, IMU calibration, the
complementary filter, and the moteus/CAN setup are duplicated in each one
rather than shared through a library. That's deliberate for a bring-up
sequence you flash one stage at a time on a bench — there's no `#ifdef
STAGE` to get wrong, no shared header to desync from the file that's
actually running. The cost is that a few constants (IMU calibration,
`kThetaOffset`, `kTau`) have to be hand-copied forward between files; see
§7.

---

## 2 — Infrastructure shared by every file

### 2.1 Loop rate

```cpp
const uint32_t imuClockFrequency = 4000000;   // was 100000
const uint32_t kPeriodMs         = 2;          // was 20 (500 Hz vs 50 Hz)
```

Raised once, before Stage 0, per the bring-up plan's Order of work step 1.
At 100 kHz SPI a 12-byte IMU burst read cost ~1 ms — 20% of even a 5 ms
budget. At 4 MHz it costs ~24 µs. 500 Hz sits inside the 400–1000 Hz target
band: comfortably above the 281 Hz anti-aliasing floor (Nyquist on the
141 Hz wheel fundamental at `omega_max`) and far above the 35 Hz raw
stability floor. The fastest closed-loop pole for the deployed gains is
≈12.5 rad/s (≈2 Hz — see §6), so 500 Hz is roughly 250× oversampled
relative to control dynamics; anti-aliasing, not control bandwidth, is what
the rate has to satisfy.

### 2.2 BMI270 output data rate

```cpp
imu.setAccelODR(BMI2_ACC_ODR_400HZ);
imu.setGyroODR(BMI2_GYR_ODR_400HZ);
```

Added in `setup()`, after `beginSPI()` succeeds, as a non-fatal warning (not
a retry loop — losing 400 Hz ODR degrades the estimator, it doesn't make the
board unsafe, so it doesn't block boot the way a missing IMU does). Verified
against the actual SparkFun_BMI270_Arduino_Library source (`setAccelODR`/
`setGyroODR` methods, `BMI2_ACC_ODR_400HZ`/`BMI2_GYR_ODR_400HZ` constants in
`bmi2_defs.h`) rather than assumed — reading faster than the sensor's ODR
just re-reads the same sample and the gyro looks stepped, so this has to
actually take effect, not just compile.

### 2.3 IMU calibration (identical, verbatim, in every file)

```cpp
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f }; // rad/s
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f }; // m/s^2
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };
```

From `data/imu_calibration/imu-cal-20260805T125013.log`. Unchanged from the
original `test_control.txt` — these are measured hardware constants, not
something the bring-up sequence touches.

### 2.4 Complementary filter

Unchanged algorithm (gyro-integrate, blend toward the accelerometer angle
when `|a|` is within ±15% of 1g), one change:

```cpp
static const float kTau = 1.00f;   // was 0.50f
```

Per the Block C tau sweep (1.39° / 1.01° / 0.50° / **0.29°** / 0.70° error at
tau = 0.05 / 0.10 / 0.30 / **1.00** / 3.00 s) and the measured residual gyro
bias (0.0005–0.0023 rad/s — small, which pushes the U-shaped optimum up).
Shallow optimum; re-sweep on the bench once the estimator is trusted, not
before.

### 2.5 Mount offset

```cpp
static float kThetaOffset = 0.0f;   // rad — NOT const, tuned live
...
const float theta_acc = atan2f(imuData.ax - imuData.ay,
                               imuData.ax + imuData.ay) - kThetaOffset;
```

This subtraction did not exist in the original file at all. `atan2f(ax-ay,
ax+ay)` assumes the IMU sits at *exactly* 45°; any mounting error becomes a
constant tilt offset, which — once the full law closes in Stage 4 — forces
a standing wheel speed of roughly `K1·e / K3` (≈11 rad/s per degree of
mounting error, ~27% of the 40 rad/s cap). `kThetaOffset` is deliberately
**not** `const`: it's set live over serial (§2.7) so measuring it in Stage 0
doesn't cost a recompile-and-reflash cycle.

### 2.6 Torque-mode actuation (from Stage 1 onward)

```cpp
// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field defaults to Resolution::kIgnore and is
// silently dropped before it reaches the CAN bus unless a Format
// explicitly turns each one on.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque = Moteus::kFloat;
  f.kp_scale            = Moteus::kFloat;
  f.kd_scale             = Moteus::kFloat;
  f.maximum_torque       = Moteus::kFloat;
  f.watchdog_timeout     = Moteus::kFloat;
  return f;
}();

...

Moteus::PositionMode::Command cmd;
cmd.position           = NaN;
cmd.velocity           = 0.0f;
cmd.kp_scale           = 0.0f;
cmd.kd_scale           = 0.0f;
cmd.feedforward_torque = tau_cmd;
cmd.maximum_torque     = kTauMax;
cmd.watchdog_timeout   = 0.10f;
moteus1.SetPosition(cmd, &kTorqueFormat);
```

This is Bring-Up Plan §0 Option A: position mode with both inner gains
zeroed, the command carried entirely by `feedforward_torque`, units N·m.
`cmd.maximum_torque` and `cmd.watchdog_timeout` are **not** in the plan
document — they're moteus command-struct fields, added because torque mode
has no local safety net the way position/velocity modes do: a stale CAN
link would otherwise hold whatever torque was last sent. Belt-and-suspenders
on top of the firmware clamp and latch, not a replacement for either.

> [!danger] Bug found during hardware bring-up, fixed after the fact in all
> five files — record it here so it isn't reintroduced.
> The first version of this section set `feedforward_torque`, `kp_scale`,
> `kd_scale`, `maximum_torque`, and `watchdog_timeout` on `cmd` and called
> `moteus1.SetPosition(cmd)` with **no second argument**. That compiles
> fine and looks correct, but `SetPosition()`'s default `Format` (confirmed
> against `mjbots/moteus-arduino`'s `moteus_protocol.h`) only transmits
> `position` and `velocity` — every other field defaults to
> `Resolution::kIgnore` and is dropped before the CAN frame is built. In
> practice this meant every "torque mode" command in Stages 1–5 was
> actually just `position=NaN, velocity=0.0` with the moteus's own onboard
> `kp_scale=1`/`kd_scale=1` — a plain velocity-mode hold at zero, not
> torque mode at all. Symptom on the bench: velocity-mode test commands
> spun the wheel fine; every "torque" command, including Stage 1's fixed
> 0.01 N·m pulse, produced nothing. The `kTorqueFormat` object above is the
> fix — it has to be constructed and passed as `SetPosition(cmd,
> &kTorqueFormat)` in every file that sends torque; `cmd` alone is not
> sufficient no matter what's set on it.
>
> **Resolution, in full, since the first fix alone didn't finish the job:**
> After the `Format` fix, the wheel *still* didn't spin at 0.01 N·m.
> Added `moteus_mode`/`moteus_fault` telemetry (`Query::Result::mode`,
> `::fault` — register 0x000/0x00f) to check for a genuine servo fault;
> both read clean (`mode=10` Position, `fault=0`) throughout, which ruled
> that out as *this* symptom's cause. Added `moteus_torque`/`moteus_qcurrent`
> telemetry next (the servo's own measured output, not the command echoed
> back) plus a live-adjustable pulse size (`t<Nm>`) to test without
> reflashing. At `t0.05` the wheel spun. **Conclusion: 0.01 N·m was simply
> below real wheel/bearing stiction on this rig — the command path was
> already working correctly after the `Format` fix; the remaining "no
> spin" was never a software bug.** Stage 1's default pulse is now 0.05
> N·m, empirically confirmed rather than assumed.
>
> `cmd.ignore_position_bounds = 1.0f` (register 0x02d) was added to all
> five files during this investigation as the leading hypothesis for the
> *second* round of "still doesn't spin" — moteus defaults assume a
> bounded joint and can raise fault 39 ("outside limit") entering Position
> mode with the current position outside `servopos.position_min`/`_max`,
> which a continuously-spinning reaction wheel will eventually hit and a
> bounded joint won't. **This was never actually confirmed as a cause here**
> (`fault` stayed `0` the entire time) — but it's kept in all five files
> anyway because it's the textbook-correct setting for this kind of
> actuator regardless, and it's far more likely to matter later in Stages
> 4–5, where the wheel accumulates many more rotations during actual
> balancing than it ever did in Stage 1's single one-second pulse.
>
> Takeaway for reading any of this later: a silently-wrong default
> (fields dropped, not rejected; a command that "works" by entering the
> right mode but just not moving) doesn't throw a compiler error or even
> a runtime one. The `moteus_mode`/`fault`/`torque`/`qcurrent` columns
> exist in Stage 1 specifically because guessing past that point wasn't
> converging — they're not in Stages 2–5 (yet) because the fix here
> wasn't actually about those stages' logic.

Stage 0 never uses this struct — see §3.

### 2.7 Serial command protocol (identical across Stages 0/2–5; Stage 1 differs)

One line, first character is the command, rest of the line (if any) is a
float argument:

| Command | Stages | Effect |
|---|---|---|
| `o<deg>` | 0, 1, 2, 3, 4, 5 | Set `kThetaOffset` (degrees → radians internally) |
| `p` | 1 only | Fire one 0.01 N·m / 1000 ms open-loop pulse |
| `a<0/1>` | 2, 3, 4, 5 | Arm (`1`) or disarm (`0`) — non-zero float arms |
| `g<0..1>` | 2, 3, 4, 5 | Set `gGainScale`, clamped to `[0, 1]` |

Parsing (`Section 2e` in every file) is the same seven lines everywhere:
`Serial.readStringUntil('\n')`, trim, dispatch on the first character. No
framing, no checksums — this runs over a tethered USB-serial link on a
bench, not a noisy channel.

---

## 3 — Stage 0: Observe mode (`test_control.txt`)

**What changed from the original file:** loop rate/ODR (§2.1–2.2), `kTau`
(§2.4), `kThetaOffset` added (§2.5), telemetry gained a `theta_acc_deg`
column and a `tau_preview_Nm` column, and Section 2d was rewritten from "do
nothing" to "compute the real law, print it, never act on it."

**The actuation guarantee.** The motor call in `loop()` is:

```cpp
moteus1.SetStop();
```

every cycle, unconditionally — not a `Command` struct with `tau=0`, not a
flag-gated branch. There is no variable in this file whose value determines
whether torque gets sent; the only call that touches the motor is a hard
stop. This is why torque mode itself isn't introduced until Stage 1: Stage
0's job is estimator-only confidence, and the safest way to guarantee "the
motor cannot move" is to not give it a torque-capable code path at all.

**The preview.** `previewControlTorque()` computes the full three-term LQR
law from live `theta`, `theta_dot`, and `wheel_omega` (read from the
`SetStop()` reply) and returns it — printed as `tau_preview_Nm`, sent
nowhere. This exists so a wildly implausible number (outside ±0.4 N·m, wrong
sign on a slow hand-tilt) is visible before the wheel is ever allowed to
spin, at zero actuation risk.

**Telemetry:** `t_ms  theta_deg  theta_dot_dps  theta_acc_deg  tau_preview_Nm
wheel_pos  wheel_vel`. `theta_acc_deg` (raw, offset-corrected accelerometer
angle, *before* the complementary filter) sits next to `theta_deg` (filtered)
specifically for the "tilt fast, watch them diverge" checklist item — the
lever-arm rejection is otherwise invisible without external logging.

---

## 4 — Stage 1: Open-loop torque

**State machine**, not a continuous command — a pulse only fires when
explicitly requested:

```cpp
static bool     gPulseActive  = false;
static uint32_t gPulseStartMs = 0;
```

`p` over serial sets `gPulseActive = true` and latches `gPulseStartMs`.
Inside `commandWheel()`, each cycle:

```cpp
float tau = 0.0f;
if (gPulseActive) {
  if (millis() - gPulseStartMs < kPulseDurationMs) {
    tau = kTauPulse;                 // 0.01 N*m
  } else {
    gPulseActive = false;            // single-shot: auto-clears at 1000 ms
  }
}
```

`gTauPulse` (originally a `const kTauPulse`, made live-adjustable via
`t<Nm>` during the investigation below) started at `0.01f` — deliberately
small, 8% of `tau_cont` (0.12 N·m), so a wrong-sign reaction under sign
check 4 would be a light nudge to counter by hand, not a problem. Default
is now `0.05f`; see §7 below for why.

This is the first file to build the torque-mode `Command` struct (§2.6);
everything else in Section 2d is the NaN guard and hard clamp applied even
to this fixed, un-computed constant — establishing the pattern here rather
than only where it's load-bearing.

**What this stage cannot tell you automatically:** sign check 4 ("does the
panel push toward negative theta?") is read off the telemetry `theta_deg`
column, or physically felt, by a human watching/holding the panel — there
is no code that asserts a verdict on it, because that would require
assuming the answer.

### 4.1 Sign check 4 — confirmed failed, and the fix

On this hardware, it failed: a positive pulse pushed the panel toward
**increasing** `|theta|` (toward the mechanical limit), not back toward
vertical.

The fix is a single constant, deliberately **not** a change to `K1`/`K2`/
`K3` or the `tau = -(...)` structure — the bring-up plan's own rule
("never flip the minus in the control law to compensate") extends to any
other sign in the chain too. What actually needs correcting is a physical
fact about how the wheel is mounted relative to the panel's `theta`
convention, so it gets corrected exactly once, at the hardware boundary:

```cpp
static const float kWheelSign = -1.0f;
...
cmd.feedforward_torque = kWheelSign * tau;
```

In Stage 1 specifically, that's the *only* place it needs to apply —
there's no closed loop here, so `wheel_omega` never feeds back into
anything. Stages 2–5 apply `kWheelSign` on **both** sides — the outgoing
torque and the incoming `wheel_omega` read from `v.velocity` — because
their taper logic (`spinning_up = (tau>=0)==(wheel_omega>=0)`) and Stage
4/5's `k3` momentum term both depend on `tau` and `wheel_omega` staying
mutually self-consistent, not just on the panel-reaction direction being
right. Flipping only one side there would fix the panel direction while
breaking that internal consistency.

One consequence worth being explicit about: Stage 2's *original* sign
test (torque tracked `theta_dot` correctly, reported early in this
project) was run **before** the `Format` fix in §2.6 existed — meaning no
real torque was ever reaching the wheel at that time. That result was
checking arithmetic, not physical behavior, and doesn't count as
validation of anything. Stages 2–5 all need retesting from scratch with
both fixes in place before trusting them again.

---

## 5 — Stage 2: Damping only

This is where the full safety scaffold from Bring-Up Plan §3 first appears,
matching the plan's own Order of work (latches + taper introduced right
before Stage 2, not before Stage 0/1 — those stages can't run away by
construction, so the scaffold wasn't needed yet).

**Control law:**

```cpp
float tau = -(kK2 * theta_dot) * gGainScale;   // k1 = k3 = 0, hardcoded out
```

`kK1` and `kK3` are declared (for reference/consistency with later files)
but never appear in the expression — not zeroed via a runtime flag, removed
from the formula entirely, so there's no `k1_active` boolean whose state you
could get wrong.

**Taper** (fades spin-up torque only, as `|wheel_omega|` crosses from
`kTaperStart` = 36 rad/s toward `kMaxOmega` = 40 rad/s; braking is never
faded):

```cpp
const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
if (spinning_up) {
  float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
  s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
  tau *= s;
}
```

**NaN guard, then clamp** (order matters — a clamp alone lets NaN through,
since every comparison against NaN is false):

```cpp
if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }
tau = tau >  kTauMax ?  kTauMax : tau;
tau = tau < -kTauMax ? -kTauMax : tau;
```

**Latching trips** — plain assignment, no auto-clear anywhere except the
serial handler's `a1`:

```cpp
if (fabsf(theta) > kMaxTilt)        { gArmed = false; }
if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
if (!isfinite(theta) || !isfinite(theta_dot)) { gArmed = false; }
```

**The arm gate** — the *only* place `gArmed` affects what leaves the board:

```cpp
const float tau_cmd = gArmed ? tau : 0.0f;
```

`tau` (what the law wants) and `tau_cmd` (what was actually sent) are both
printed, so a disarmed run still shows you what *would* have happened.
Boots disarmed (`gArmed = false` at declaration) — arming is always a
deliberate `a1`.

---

## 6 — Stage 3: Position + damping, ramped

Identical structure to Stage 2. Two changes:

```cpp
float tau = -(kK1 * theta + kK2 * theta_dot) * gGainScale;   // k3 still zero
static float gGainScale = 0.1f;   // was 1.0f — starts LOW, ramped by hand
```

`gGainScale` boots at 0.1 rather than 1.0 specifically because this is the
first stage where a sign error on `theta` *can* reinforce into a runaway —
starting at low authority and stepping `g0.3` → `g0.6` → `g1.0` between
runs catches that at low stiffness before it's tested at full stiffness.
Nothing else differs from Stage 2's implementation — same taper, same NaN
guard, same latches, same arm gate.

**Note on the gains themselves:** `K = [-1.0998, -0.1232, -0.001732]` is
unchanged from the original file — verified independently by reconstructing
`A`, `B` from `cubli_panel_params('measured')` and re-running the exact
Gate 7 LQR call. The closed-loop poles printed in both existing docs
(`"about -15, -9, -6"` and, separately, `"-9.72 ± 0.76j, -5.05"`) were both
wrong; the actual poles for this `K` are ≈ **-12.5, -9.7, -6.3 rad/s, all
real** — a documentation correction only, `K` itself needed no change.

---

## 7 — Stage 4: Full law

```cpp
float tau = -(kK1 * theta + kK2 * theta_dot + kK3 * wheel_omega) * gGainScale;
```

All three terms active; `gGainScale` boots at 1.0 (already validated by the
Stage 3 ramp — no reason to re-ramp a law you've already proven at full
authority one term at a time).

**Standing wheel speed monitor**, added at the bottom of `commandWheel()`:

```cpp
static const float kLpTau = 5.0f;
const float dtNom = kPeriodMs * 1e-3f;
const float alphaLp = dtNom / (kLpTau + dtNom);
gWheelOmegaLp += alphaLp * (wheel_omega - gWheelOmegaLp);
```

A first-order low-pass on `wheel_omega` with a 5 s time constant, printed as
its own telemetry column (`wheel_omega_lp`) rather than reading the raw
`wheel_vel` column — the raw value moves every cycle with the correction
in progress and is useless for judging whether the wheel is *settling*
toward zero versus just correcting a transient. Uses the nominal period
rather than measured `dt` — a 5 s filter doesn't need millisecond accuracy.

Firmware does **not** attempt the 180° flip test itself (rotate the panel
physically, compare the sign of `wheel_omega_lp` across two orientations to
separate a COM offset from a gyro bias) — that's a procedure documented as
a comment, not something code can do to itself.

---

## 8 — Stage 5: Release

Control law, taper, clamp, latches: byte-for-byte the same as Stage 4.
Nothing about the *law* is being tested here — only whether the closed loop
survives running with nobody's hand on the panel. Two additions, both about
diagnosability once physical contact is gone:

**Trip reason, latched and printed once at the moment it fires:**

```cpp
enum TripReason { TRIP_NONE = 0, TRIP_TILT = 1, TRIP_OMEGA = 2, TRIP_NAN = 3 };
static int gTripReason = TRIP_NONE;
...
if (gArmed && fabsf(theta) > kMaxTilt) {
  gArmed = false; gTripReason = TRIP_TILT;
  Serial.println("# TRIP: tilt limit");
}
```

Each trip condition is now guarded by `gArmed &&` so it only fires — and
only prints — on the actual transition from armed to disarmed, not on every
cycle a trip condition happens to still be true. `gTripReason` is cleared
back to `TRIP_NONE` only by the serial handler's `a1` (deliberate re-arm),
matching the "never auto-unlatch" rule from Stage 2 onward. In Stage 4 the
same three checks exist but don't bother recording *which* one fired,
because a hand is still on the panel to have felt it happen; Stage 5 is
exactly the stage where that's no longer true, so the extra bookkeeping is
worth it there specifically.

**Telemetry** gains `trip_reason` (int, 0–3) in place of the plain `armed`
boolean context Stage 4 has — `armed` is still printed too, so you can see
both "is it armed right now" and "what disarmed it last."

---

## 9 — What's identical across every stage, verbatim

To make future edits easy to scope: these blocks are copy-identical (aside
from `kThetaOffset` starting at whatever value you've hardcoded forward)
across every file that has them —

- `struct IMUData` and `readIMU()`
- `calculateState()` (Stage 0 differs only by an added `theta_acc_out`
  parameter for its extra telemetry column)
- The CAN/moteus object setup (`canSettings`, `canBus`, `moteus1`)
- The IMU bring-up block in `setup()` (`pinMode` → `SPI.begin()` →
  `beginSPI()` retry loop → ODR calls)
- The rollover-safe scheduler check at the top of `loop()`
- `kTorqueFormat` (Stages 1–5 only — Stage 0 never builds a `Command` at
  all) and the `SetPosition(cmd, &kTorqueFormat)` call pattern
- `kWheelSign = -1.0f` (Stages 1–5 only, §4.1) — same value everywhere by
  construction (it's a fact about one physical wheel, not a per-stage
  tuning knob), applied to the outgoing torque in all five, and
  additionally to the incoming `wheel_omega` in Stages 2–5

If a bug turns up in any of these, it's in all six files and needs fixing
in all six — there is no shared header to patch once. If `kWheelSign`
were ever found to be wrong for a *different* reason later (e.g. after a
hardware rebuild), the value only needs to change in one place per file,
but every file needs the same value, checked against Stage 1 sign check 4
again.

---

## 10 — Known gaps (carried forward from file to file, not automated)

- **`kThetaOffset`**: each file's initial value is `0.0f` with a `// TODO`
  comment. You measure it once in Stage 0 and have to hand-copy the number
  into Stages 1–5 before flashing each one. It's live-settable over serial
  in every stage as a partial mitigation, but the *initial* value at boot
  is only as good as what you typed into the source.
- **`kK1`/`kK2`/`kK3`**: currently the Gate 7 values from the measured
  plant. Panel Controller Workflow's step 4 retune (free-swing period
  measured on *this* built panel) will change these — when it does, all
  five of Stages 2–5 need the same three constants updated, not just one
  file.
- **No shared library**: intentional per §1, but means any change to the
  estimator, calibration constants, or moteus setup has to be applied by
  hand to up to six files. Worth converting the common blocks into a
  `.h` once the sequence is no longer actively changing week to week.
- **Not compiled against your installed library versions.** The BMI270 ODR
  calls, the moteus `maximum_torque`/`watchdog_timeout` fields, and the
  `Format`/`Resolution` fix in §2.6 were all checked against upstream
  library source (`mjbots/moteus-arduino`, `SparkFun_BMI270_Arduino_Library`)
  rather than your local toolchain — run an Arduino IDE verify on each
  stage before it goes anywhere near the bench. The `Format` bug in
  particular shows why: it compiled cleanly and looked right for every
  stage until it was tested on real hardware, because a wrong-but-silent
  default (fields dropped, not rejected) doesn't throw a compiler error or
  even a runtime one.

---

## Related

- [[Arduino Bring-Up Plan — Sections 2b and 2d]] — the procedure and checklists these files implement
- [[Simscape Panel Model — Build Guide]] — plant, gains, LQR derivation
- [[Bench Test Checklist & Failure Diagnosis]] — symptom-to-cause tables
- [[Firmware — core Walkthrough]] — sign-convention reference (§4.6)

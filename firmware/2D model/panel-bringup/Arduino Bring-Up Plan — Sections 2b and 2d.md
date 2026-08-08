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

# Arduino Bring-Up Plan — Sections 2b and 2d

Platform moved to **Arduino IDE**; CAN and IMU communication now working.
This note covers what to change in `test_control.txt` sections **2b (state
estimation)** and **2d (control)**, and the staged plan for closing the loop
without booting the controller straight up.

Plant reference: [[Simscape Panel Model — Build Guide]]
Procedures: [[Bench Test Checklist & Failure Diagnosis]]

---

## 0 — BLOCKING ISSUE: velocity mode vs torque mode

**Settle this before touching either section. It changes what the gains mean.**

`commandWheel()` currently sends `cmd.velocity` with `position = NaN`, i.e. it
commands **wheel speed**.

Every gain we derived produces **torque**:

```
K = [-1.0998, -0.1232, -0.001732]
units: N*m/rad, N*m/(rad/s), N*m/(rad/s)
```

Dropping these into `omega_cmd` gives a number in the wrong dimension. It will
look almost plausible while being wrong by roughly `1/(I_w*dt)`. The entire
validated model assumes `hdot = u`, i.e. commanded torque.

### Option A — switch to torque mode (RECOMMENDED)

```cpp
Moteus::PositionMode::Command cmd;
cmd.position           = NaN;
cmd.velocity           = 0.0f;
cmd.kp_scale           = 0.0f;    // disable the position loop
cmd.kd_scale           = 0.0f;    // disable the velocity loop
cmd.feedforward_torque = tau;     // N*m -- this is the whole command
moteus1.SetPosition(cmd);
```

This is what moteus calls torque mode: position mode with both inner gains
scaled to zero and the command carried entirely by feedforward torque. Not
obvious from the docs.

### Option B — velocity bridge (fallback)

If the velocity path is what is proven and you do not want to disturb it,
convert torque to a velocity command each cycle:

```cpp
omega_cmd = wheel_omega + (tau / kIw) * dt;   // kIw = 1.858e-4
```

That is the speed the wheel *should* have after `dt` if torque `tau` were
applied. No integrator, so no windup, and it self-corrects against the measured
speed every cycle. Valid because the moteus inner velocity loop runs at
15-30 kHz FOC, far faster than the outer loop.

**Take A. B is a fallback if torque mode misbehaves.**

---

## 1 — Section 2b changes

### 1.1 Mount offset — MISSING, and it matters

`atan2f(ax - ay, ax + ay)` assumes the IMU sits at **exactly** 45 deg. It will
not.

Any mounting error becomes a constant tilt offset, and a constant offset forces
a **standing wheel speed** of

$$\dot\phi_{ss} = -\frac{K_1 e}{K_3}$$

| Mount error `e` | Standing wheel speed | % of the 40 rad/s cap |
|---|---|---|
| 0.1 deg | 1.1 rad/s | 2.8 % |
| 0.5 deg | 5.4 rad/s | 13.5 % |
| **1.0 deg** | **10.8 rad/s** | **27 %** |

That momentum is burned before any disturbance arrives, and it comes straight
off the recovery envelope.

```cpp
static float kThetaOffset = 0.0f;   // rad, MEASURED at the balance point
const float theta_acc = atan2f(imuData.ax - imuData.ay,
                               imuData.ax + imuData.ay) - kThetaOffset;
```

**How to measure:** balance the panel by hand at true vertical (eyeball against
a plumb line, or rest it on a shim of known geometry), read `theta`, and that
value is the offset. Do it in Stage 0.

> This is the firmware analogue of `p.phi_mount = 44.990 deg` in the model — the
> same physical fact, that the sensor's zero is not the balance point.

### 1.2 `kTau = 0.50` is probably short

The complementary filter time constant has a **U-shaped** optimum: below it the
lever-arm error and accelerometer noise leak through; above it the residual gyro
bias term `b*tau` dominates.

From [[IMU Lever Arm & Estimator — Block C]]:

| tau [s] | Expected estimate error |
|---|---|
| 0.05 | 1.39 deg |
| 0.10 | 1.01 deg |
| 0.30 | 0.50 deg |
| **1.00** | **0.29 deg** |
| 3.00 | 0.70 deg |

The calibration log shows residual gyro bias around **0.0005-0.0023 rad/s**,
which is small and pushes the optimum **up**. Try tau = 1.0 s.

Shallow optimum — anything in 0.5-1.5 s works. Sweep it on the bench once the
estimator is trusted, not before.

### 1.3 Loop rate — the real problem

`kPeriodMs = 20` is **50 Hz**. That is **1.4x the 35 Hz stability floor** from
[[Discrete Loop Test — Block B]] — essentially no margin, and that floor assumed
**zero jitter**. It is also far below the **281 Hz** anti-aliasing requirement
for the 141 Hz wheel fundamental.

Three things gate raising it, **in this order**:

| # | Change | From | To | Why |
|---|---|---|---|---|
| 1 | `imuClockFrequency` | 100 kHz | **4-8 MHz** | at 100 kHz a 12-byte read costs ~1 ms |
| 2 | BMI270 ODR | 100 Hz | **400 Hz** | above ODR you re-read the same sample; the gyro looks stepped |
| 3 | `kPeriodMs` | 20 | **2 or 3** | 500 or 333 Hz |

> [!warning] Do this BEFORE closing the loop, not after
> Debugging a marginal-rate controller is much harder than raising the rate
> first. A 50 Hz loop will ring visibly on every recovery and you will not know
> whether it is the rate, the gains, or the estimator.

Target **400-1000 Hz**: above the anti-aliasing floor, 10-28x clear of the
stability floor, inside the CAN budget, matched to a realistic IMU ODR.

### 1.4 Keep as-is

- **The atan2 difference/sum trick** is correct and elegant — the ratio cancels
  `g`, so calibration scale and units do not matter, and it is singularity-free
  over the full circle.
- **The accel gate** at +/-15 % is a sensible width. Keep it loose: lever-arm
  contamination is *perpendicular* to gravity and barely moves the magnitude, so
  a tight gate catches little and risks breaking the error cancellation that
  makes the filter work in the first place.
- **Seeding from the accelerometer** on the first sample — correct, and it is
  what stops the filter walking to truth over several tau while the controller
  acts on a lie.
- **dt from an internal timer with a guard** — correct.

---

## 2 — Section 2d: staged bring-up

### 2.0 Globals that gate everything

```cpp
static bool  gArmed     = false;   // false => torque forced to zero
static float gGainScale = 0.0f;    // 0 .. 1

static const float kMaxTilt    = 0.14f;    // rad, 8 deg -- trip
static const float kMaxOmega   = 40.0f;    // rad/s     -- trip
static const float kTauMax     = 0.12f;    // N*m       -- saturation
static const float kTaperStart = 36.0f;    // rad/s, 90 % of cap
static const float kIw         = 1.858e-4f;
```

Plus a **serial command** to set `gGainScale` and toggle `gArmed` at runtime.
Recompiling between gain steps will waste an hour over the session.

### Stage 0 — Observe mode (~30 min)

`gArmed = false`. Estimator running, **torque forced to zero at the output** —
but still compute and print the torque that *would* have been commanded. That is
what makes the stage useful.

- [ ] `theta` against a protractor at **+/-10 deg and +/-30 deg**
- [ ] Sign: tilt right -> `theta` positive **and** `theta_dot` positive
- [ ] Record `kThetaOffset` at the balance point
- [ ] **Tilt fast** and watch `theta_acc` vs `theta` side by side. The raw angle
      should overshoot wildly; the filtered one should stay clean. That is the
      lever-arm rejection working — seeing it once is worth more than reading
      about it
- [ ] Hold still 60 s: `theta` must not drift

If `theta` is inverted -> swap the two `atan2f` arguments.
If `theta_dot` alone is inverted -> negate `imuData.wz`.

### Stage 1 — Open-loop torque (~15 min)

Panel **held firmly by hand**. Command a fixed 0.01 N*m for one second.

- [ ] Wheel spins -> torque mode works
- [ ] `wheel_omega` positive -> **sign check 3**
- [ ] **Does the panel push toward negative theta?** -> **sign check 4**

> [!danger] Sign check 4 is the one that kills hardware
> If a positive torque pushes the panel the *wrong* way, the correct negative
> sign in the control law will actively drive the fall. Fix the estimator or the
> encoder convention — **never** the minus in the control law. This is also the
> only check that cannot be done without spinning the motor.

### Stage 2 — Damping only (~20 min)

`k1 = 0`, `k2 = -0.1232`, `k3 = 0`, `gGainScale = 1.0`. Still held by hand.

The panel should feel **viscous** — resisting rotation in either direction, with
no tendency to hold a position. If it fights you or feels like it is helping the
fall, the `theta_dot` sign is wrong.

Isolates one term with no possibility of runaway. Cheapest confidence available.

### Stage 3 — Position + damping, ramped

`k1 = -1.0998`, `k2 = -0.1232`, `k3 = 0`. Still held by hand.
`gGainScale`: **0.1 -> 0.3 -> 0.6 -> 1.0**, one step per run.

- At 0.1: a gentle push back toward vertical
- At 1.0: distinctly stiff

The wheel **will** spin up steadily without `k3`. That is expected, and it is
exactly why you do not stop here.

### Stage 4 — Full law

Add `k3 = -0.001732`. The wheel should now **unwind** after each correction
instead of accumulating.

Watch the **standing wheel speed** (low-pass `wheel_omega`, tau ~ 5 s):
near zero is healthy; a few rad/s means gyro bias or a COM offset.

> **The 180 deg flip test** separates them in fifteen seconds: rotate the panel
> 180 deg about the pivot axis and re-run. A COM offset **reverses sign**; a
> gyro bias **does not**.

### Stage 5 — Release

Rails in place, e-stop in hand, `gGainScale = 1.0`.
Expect recovery from **7-9 deg** — below the 11-12 deg ideal figure, because
quantisation, noise, delay and friction all consume margin.

---

## 3 — Safety additions to 2d

**Missing entirely right now. Stage 5 must not happen without these.**

```cpp
// Latch on trip. Do NOT auto-unlatch.
if (fabsf(theta) > kMaxTilt)        { gArmed = false; }
if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
if (!isfinite(theta) || !isfinite(theta_dot) || !isfinite(tau)) {
  gArmed = false;
}
```

**Latching matters.** A rig that re-arms itself trips repeatedly while you try to
work out what happened. Re-arming is a deliberate human act.

### Wheel taper — before saturation

```cpp
const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
if (spinning_up) {
  float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
  s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
  tau *= s;
}
tau = tau >  kTauMax ?  kTauMax : tau;
tau = tau < -kTauMax ? -kTauMax : tau;
```

Two requirements:

- **Only fade spin-up torque.** Braking is always allowed — it is the recovery
  action the taper exists to preserve.
- **Fade, never switch.** A hard cutoff chatters; on hardware that becomes
  audible buzzing and current ripple that reads as a failing motor.

Why it matters numerically: at the recovery envelope edge the wheel reaches
**39.5 of 40 rad/s**. Without a taper, any recovery near the limit trips instead
of recovering, and the usable envelope shrinks below the simulated 11-12 deg.

### NaN guard before the clamp

Every comparison against NaN is false, so a clamp passes it straight through. A
NaN reaching `feedforward_torque` is not a no-op — it is an invalid command.

---

## 4 — Gains reference

From [[Simscape Panel Model — Build Guide]], LQR on the measured plant:

```matlab
theta_max = 0.20;  rate_max = 2.0;
omega_des = 0.5*p.omega_cap;   % 20 rad/s
rho_lqr   = 12;
Q = diag([1/theta_max^2, 1/rate_max^2, 1/omega_des^2]);
R = rho_lqr / p.tau_cont^2;
K_lqr = lqr(p.A, p.B, Q, R);
```

```
K = [-1.0998   -0.1232   -0.001732]
closed-loop poles: -9.72 +/- 0.76j, -5.05
```

Control law:
`tau = -(K1*theta + K2*theta_dot + K3*wheel_omega) * gGainScale`

| Constant | Value |
|---|---|
| I_w | 1.858e-4 kg m^2 |
| Theta | 3.399e-3 kg m^2 |
| lambda | 9.037 rad/s (tau = 111 ms) |
| tau_cont | 0.12 N m |
| omega_cap | 40 rad/s |

---

## 5 — Order of work

1. **Raise the loop rate** (SPI clock -> ODR -> period). Before anything else.
2. **Switch to torque mode.**
3. **Stage 0** — observe mode, protractor, record `kThetaOffset`, signs 1-2.
4. **Stage 1** — open-loop torque, signs 3-4.
5. Add safety latches and the taper.
6. **Stages 2 -> 5**, one per run, writing down what changed each time.

> Change one thing at a time. Two changes between runs and a surprising result
> tells you nothing.

---

## Related

- [[Simscape Panel Model — Build Guide]] — plant, gains, LQR derivation
- [[Bench Test Checklist & Failure Diagnosis]] — full symptom-to-cause tables
- [[IMU Lever Arm & Estimator — Block C]] — tau sweep, bias, the flip test
- [[Discrete Loop Test — Block B]] — loop rate and the 35 Hz floor
- [[Saturation Envelope Test — Block A]] — the taper and the cap
- [[Firmware — core Walkthrough]] — the C++ reference implementation

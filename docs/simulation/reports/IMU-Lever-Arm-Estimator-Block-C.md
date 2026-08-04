---
tags:
  - space-challenge
  - sofia
  - cubli
  - simulation
  - panel
  - controls
  - estimator
  - imu
  - firmware
  - validation
---

# IMU Lever Arm & Estimator — Block C

Third nonlinear test. Answers: **can we actually measure the tilt we are
controlling?**

**Result: yes, with a complementary filter — and the constraint turns out to be
gyro bias, not IMU mounting position.**

Script: `cubli_panel_imu.m`
Depends on: `cubli_panel_params.m` (measured mode) and the LQR cell of
`cubli_panel_simscape_gates.m`.

---

## 1 — The physical problem

An accelerometer does **not** measure gravity. It measures **specific force** —
the difference between the acceleration of the case and free fall. Sitting on a
table, the table pushes up at 1 g, so it reads 1 g upward and we interpret that
as "down is that way". The instant the case accelerates for any *other* reason,
that reading is contaminated.

Our case is bolted to a panel rotating about a pivot. Put the IMU a distance
$d$ from the pivot and it experiences two extra accelerations:

$$\underbrace{\ddot\theta\,d}_{\text{tangential}} \qquad\qquad \underbrace{\dot\theta^2 d}_{\text{centripetal}}$$

so the specific force in body axes is

$$f_x = \ddot\theta\,d + g\sin\theta, \qquad f_y = g\cos\theta - \dot\theta^2 d$$

and a naive tilt readout is $\hat\theta = \operatorname{atan2}(f_x, f_y)$.

### Why this is worse than it sounds

The $\ddot\theta d$ term is **largest exactly when the estimate matters most.**
When the panel is upright and still, $\ddot\theta = 0$ and the accelerometer is
perfect. When the controller is fighting hardest — mid-recovery — $\ddot\theta$
peaks and the accelerometer lies worst.

From the Block A envelope edge, $\ddot\theta$ reaches **−45.6 rad/s²**:

| IMU distance from pivot | Apparent tilt error |
|---|---|
| 10 mm | 2.6° |
| 20 mm | 5.2° |
| 50 mm | 12.9° |
| 106 mm (panel centre) | **25.9°** |

At the panel centre, during a 10° recovery, the accelerometer reports **26° of
tilt in the wrong direction** — an error larger than the disturbance being
corrected. Feed that to the controller and it pushes the wrong way, hard.

The centripetal term is secondary: at $\dot\theta = 3$ rad/s it is under 10 % of
$g$ even at the panel centre.

---

## 2 — The three configurations tested

| Mode | Tilt estimate | Rate estimate |
|---|---|---|
| **truth** | perfect $\theta$ | perfect $\dot\theta$ |
| **naive** | $\operatorname{atan2}(f_x,f_y)$ | gyro |
| **compfilter** | see below | gyro |

The complementary filter, one line:

```matlab
alpha  = tau/(tau + Ts);
th_hat = alpha*(th_hat + gyro*Ts) + (1-alpha)*th_acc;
```

**Read it as two sensors covering two frequency bands.** The gyro is integrated
forward — accurate over short intervals, but drifts because any bias
accumulates. The accelerometer has no drift — it always knows where down is on
average — but is corrupted by $\ddot\theta d$ at high frequency. So: trust the
gyro fast, trust the accelerometer slow. $\tau$ is the crossover.

The lever-arm error is a *transient* lasting ~0.2 s. A filter with $\tau = 1$ s
rejects it almost entirely.

### Simulation note

This block is **pure MATLAB**, not Simscape — the same 3-state nonlinear plant
integrated with RK4. Gate 3 already proved the two agree to 1.7e-07, so for
parameter sweeps the MATLAB version is equivalent and runs in seconds rather
than minutes. Saturation and the wheel cap are carried over from Block A.

Sensor placeholders used (**to be replaced with BMI270 measurements**):
accelerometer 0.02 m/s² rms, gyro noise 0.002 rad/s rms, gyro bias 0.005 rad/s.

---

## 3 — Results

### 3.1 Estimator comparison, d = 50 mm, τ = 1 s, θ₀ = 5°

| Mode | θ_end | Max estimate error | Recovered |
|---|---|---|---|
| perfect state | 0.00° | 0.00° | yes |
| **naive accel** | **88.83°** | **32.44°** | **NO** |
| complementary filter | 0.01° | 0.29° | yes |

The naive case fails outright — the panel ends up on its side. This is the
counterfactual worth keeping: it shows the filter is doing real work.

### 3.2 Recovery envelope with the estimator in the loop

| Configuration | Edge |
|---|---|
| Perfect state | 11.59° |
| Complementary filter | **11.53°** |

**A 0.5 % penalty.** The envelope from Blocks A and B survives.

### 3.3 IMU mounting position sweep — the surprise

| d [mm] | 0 | 10 | 20 | 30 | 50 | 75 | 106 |
|---|---|---|---|---|---|---|---|
| Max est. error | 0.29° | 0.29° | 0.29° | 0.29° | 0.29° | 0.29° | 0.37° |

**Mounting position makes essentially no difference.**

### 3.4 Filter time constant sweep, d = 50 mm

| τ [s] | 0.05 | 0.10 | 0.30 | **1.00** | 3.00 | 10.00 |
|---|---|---|---|---|---|---|
| Max est. error | 1.39° | 1.01° | 0.50° | **0.29°** | 0.70° | 1.13° |

U-shaped, with a shallow optimum around 0.5–1 s.

---

## 4 — Discussion

### 4.1 Why the lever arm stopped mattering

This is the key insight of the block. The error floor of 0.29° is **not** the
lever arm. It is:

$$\text{gyro bias} \times \tau = 0.005 \times 1.0 = 0.005\ \text{rad} = 0.286°$$

which matches the observed 0.29° to two decimals, at every mounting distance.

The filter has already pushed the lever-arm contribution *below the bias floor*.
So the constraint has moved from **mechanical** to **sensor calibration**.

> [!important] Practical consequence
> **Mount the BMI270 wherever is convenient.** Cable routing, board space,
> keeping it off the motor for thermal reasons — all of these now outrank
> proximity to the pivot. What you cannot do is ignore gyro bias.

This also explains the U-shape in §3.4: below τ ≈ 0.3 s the lever arm and
accelerometer noise leak through; above τ ≈ 1 s the bias term $b\tau$ takes
over. The optimum is where the two cross.

### 4.2 What gyro bias actually costs

A constant tilt-estimate error $e$ means the controller drives the *estimate* to
zero, so the panel physically balances $e$ off true vertical. Solving the
steady state ($u = 0$, $\theta = 0$):

$$\dot\phi_{ss} = -\frac{K_1 e}{K_3}$$

| Gyro bias | τ = 0.3 s | τ = 1.0 s |
|---|---|---|
| 0.005 rad/s (0.29°/s) — assumed | 0.95 rad/s (2 %) | 3.17 rad/s (8 %) |
| 0.017 rad/s (1°/s) — **uncalibrated BMI270** | 3.24 rad/s (8 %) | **10.79 rad/s (27 %)** |
| 0.035 rad/s (2°/s) | 6.67 rad/s (17 %) | **22.22 rad/s (56 %)** |

The 0.005 rad/s placeholder is **optimistic**. An uncalibrated consumer MEMS
gyro typically shows 1–2°/s of zero-rate offset. At 1°/s with τ = 1 s you burn
**27 % of the momentum budget standing perfectly still**, before any disturbance
arrives — and that comes straight off the 11.5° envelope.

> [!note] The wheel settles, it does not ramp
> Note the wheel reaches a *steady* speed rather than accelerating to
> saturation. That is the momentum penalty $Q_{33}$ doing its job. With $Q_{33}$
> too small there would be no equilibrium and you would get the classic
> "balances for eight seconds, then falls over" failure.
> See [[Disturbance Estimation & Attitude Representation]].

### 4.3 Do we need an online bias estimator? — No, for the panel

| | Bias | Tilt error (τ=1 s) | Standing φ̇ | % of cap |
|---|---|---|---|---|
| Uncalibrated | 1°/s | 0.97° | 10.8 rad/s | 27 % |
| **Startup calibration** | ~0.05°/s | 0.05° | 0.55 rad/s | **1.4 %** |
| Online estimation | ~0.01°/s | 0.01° | 0.11 rad/s | 0.3 % |

**Startup calibration buys a 20× improvement. Online estimation buys another 5×
on top of something already negligible.** Bad return for the effort at this
stage.

**Startup calibration:** hold the panel still for 2–3 s at boot, average the
gyro, subtract from every subsequent reading. Five lines.

**Where it stops being enough:** thermal drift. The BMI270 sits near a motor
dissipating several watts with no airflow, and MEMS gyro bias moves with
temperature — plausibly 0.1–0.3°/s over a ten-minute session. That puts you back
at 2–6 rad/s standing wheel speed. Acceptable for a demo, a problem for extended
running.

**So: panel → startup calibration. Cube → online estimation**, which is already
the plan (3 gyro-bias states in the 6-state MEKF). A 2-state panel filter
(tilt + bias) would be a cheap way to prototype that architecture on the easier
problem, but it is not needed to pass.

### 4.4 A free diagnostic

A non-zero standing wheel speed has two possible causes: **gyro bias** or a
**COM offset**. They are separable in fifteen seconds:

> Rotate the panel 180° about the pivot axis and re-run.
> A COM offset **reverses sign**. A gyro bias **does not**.

That tells you whether to recalibrate or to shim. It is the diagnostic use of
the disturbance estimate from the sizing notes, in its simplest possible form,
and it needs no estimator at all.

---

## 5 — What Andrea needs from this

1. **Mount the IMU wherever is convenient.** Position is no longer a constraint.
   Prioritise thermal isolation from the motor and clean cable routing.
2. **Startup gyro calibration is mandatory**, not optional. Without it, 27 % of
   the momentum budget is gone before the panel is even disturbed.
3. **Complementary filter, τ ≈ 0.5–1 s.** Shallow optimum, so the exact value is
   not critical. One line of code.
4. **Log the standing wheel speed.** It is a direct health indicator: near zero
   is good, a few rad/s means bias or COM offset, and the 180° flip test
   distinguishes them.
5. **Measure the real BMI270 noise and bias.** Log stationary for a few minutes.
   Those three numbers replace the placeholders and make this whole block
   quantitative rather than indicative.
6. **Consider thermal drift** if we ever run for more than a few minutes.

---

## 6 — Caveats

- **Sensor figures are placeholders.** Accelerometer noise, gyro noise and bias
  are all guesses. §4.2 shows the result is *sensitive* to bias, so this is the
  first thing to replace.
- **Bias modelled as constant.** No random walk, no thermal drift.
- **5 s runs.** The bias-driven wheel speed settles well inside that, but a 20 s
  run should confirm nothing drifts on a longer timescale.
- **Single axis, scalar filter.** Corner balancing needs the reduced-attitude
  MEKF; nothing here transfers directly except the physical insight about
  specific force.
- **No IMU misalignment.** A mounting angle error is a constant tilt offset and
  behaves exactly like gyro bias in §4.2 — worth measuring once the panel is
  assembled.

---

## 7 — Open items

- [ ] BMI270 stationary log → real noise density and bias
- [ ] 20 s run to confirm long-term settling
- [ ] Finer τ sweep between 0.3 and 1.5 s with the measured noise figures
- [ ] Thermal drift test: log bias while the motor runs
- [ ] IMU mounting misalignment, once the panel is assembled

---

## Related

- [[Saturation Envelope Test — Block A]] — actuator authority
- [[Discrete Loop Test — Block B]] — loop rate
- [[Firmware Handoff — Panel Stage]] — startup calibration goes here
- [[Disturbance Estimation & Attitude Representation]] — why $Q_h$ matters, and
  the estimator architecture for the cube
- [[Simscape Panel Model — Build Guide]] — §6.5

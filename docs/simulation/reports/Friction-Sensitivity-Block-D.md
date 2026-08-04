---
tags:
  - space-challenge
  - sofia
  - cubli
  - simulation
  - panel
  - controls
  - friction
  - firmware
  - validation
---

# Friction Sensitivity — Block D

Fourth nonlinear test. Unlike A–C this one runs **without any measurement**, so
the question is inverted:

> Not *"what does our friction do"* but **"how much friction can the loop
> tolerate before it matters"** — producing thresholds to measure against.

Script: `cubli_panel_friction.m`
Depends on: `cubli_panel_params.m` (measured mode) and the LQR cell of
`cubli_panel_simscape_gates.m`.

---

## 1 — Three mechanisms, three different effects

They are not interchangeable. Two hurt, one helps.

### 1.1 Pivot Coulomb — τ_cp — external, on the panel

Creates a **stick deadband**. Below a threshold tilt the friction torque exceeds
the gravity torque, so the panel simply does not move — and a controller that
sees no motion gets no information.

$$\theta_{dead} = \arcsin\!\left(\frac{\tau_{cp}}{Sg}\right)$$

| τ_cp | % of τ_cont | Deadband |
|---|---|---|
| 0.5 mN·m | 0.4 % | 0.11° |
| 1 mN·m | 0.8 % | 0.22° |
| 2 mN·m | 1.7 % | 0.44° |
| 5 mN·m | 4.2 % | 1.09° |
| 10 mN·m | 8.3 % | 2.18° |

Symptom: **stick-slip limit cycling.** The panel drifts to the deadband edge,
sticks, the controller winds up, it breaks free, overshoots, sticks again.

Our pivot is a steel pin in a printed bracket, so this is not negligible.

### 1.2 Wheel Coulomb — τ_cw — internal, wheel-to-panel

Bearing drag **plus cogging torque**. Worst at zero crossings: the wheel does
not respond to small commands, so fine control near equilibrium degrades. This
is normally the one that bites.

A 12-pole-pair outrunner like the MN4006 can easily show **5–15 mN·m of
cogging**, i.e. **4–12 % of τ_cont**. Cogging appears at 24 cycles per
revolution (2 × pole pairs).

### 1.3 Wheel viscous — b_w — internal, speed-proportional

**Beneficial.** A passive momentum sink that unwinds the wheel with time
constant $I_w/b_w$, costing a little authority in exchange for drift immunity.

| b_w [N·m·s/rad] | Bleed time constant | Drag at ω_cap |
|---|---|---|
| 1e-6 | 186 s | 0.04 mN·m |
| 1e-5 | 18.6 s | 0.40 mN·m |
| 5e-5 | 3.7 s | 2.00 mN·m |
| 2e-4 | 0.9 s | 8.00 mN·m |

Relevant to the Block C finding: gyro bias drives a **standing wheel speed**.
Viscous drag bleeds that passively, for free, with no estimator.

---

## 2 — Equations

Pivot friction is **external** (acts on the panel against ground). Wheel
friction is **internal** (equal and opposite between wheel and panel), so it
enters exactly where the motor torque does:

$$\ddot\theta = \frac{Sg\sin\theta + \tau_p - u - \tau_w}{\bar\Theta}, \qquad
\ddot\phi = \frac{u + \tau_w}{I_w} - \ddot\theta$$

with

$$\tau_p = -\tau_{cp}\tanh(\dot\theta/\varepsilon), \qquad
\tau_w = -\left(b_w\dot\phi + \tau_{cw}\tanh(\dot\phi/\varepsilon)\right)$$

> [!note] Why `tanh` and not `sign`
> A true `sign()` is discontinuous at zero velocity and makes the solver hunt —
> the same failure as the hard wheel-speed cap in [[Saturation Envelope Test — Block A]].
> `tanh(v/ε)` with ε = 0.05 rad/s is a smooth approximation. It slightly
> understates stiction very near zero, so the results are marginally
> **optimistic** on the Coulomb terms.

Pure MATLAB with RK4, same as Block C. Runs are **20 s**, not 5 — stick-slip
limit cycles are slow and would hide inside a short run.

---

## 3 — What the script does

```
runfric()          one run at given (tau_cp, tau_cw, b_w)
   |
   +-- 1. pivot Coulomb sweep     0 .. 20 mNm
   +-- 2. wheel Coulomb sweep     0 .. 40 mNm
   +-- 3. wheel viscous sweep     0 .. 1e-3
   +-- 4. envelope vs friction    4 named cases, bisected
   +-- 5. limit-cycle time plots
```

Metrics returned per run, all computed over the **last half** of the trajectory
so the initial recovery does not contaminate them:

| Field | Meaning |
|---|---|
| `th_ss` | mean steady tilt — non-zero means a deadband offset |
| `phid_ss` | mean steady wheel speed — the momentum cost of friction |
| `lc_amp` | half peak-to-peak tilt — **limit-cycle amplitude** |
| `ok` | survived 20 s with \|θ\| < 2° |

`lc_amp` is the important one. A run can score `ok = true` while hunting
±1° forever, which is a fail in practice even though it never falls over.

---

## 4 — Results

*(fill from the run — table shape below)*

### 4.1 Pivot Coulomb

| τ_cp [mN·m] | % τ_cont | Deadband | θ_ss | φ̇_ss | ok |
|---|---|---|---|---|---|
| 0 | 0 | 0.00° | | | |
| 0.5 | 0.4 | 0.11° | | | |
| 1 | 0.8 | 0.22° | | | |
| 2 | 1.7 | 0.44° | | | |
| 5 | 4.2 | 1.09° | | | |
| 10 | 8.3 | 2.18° | | | |
| 20 | 16.7 | 4.39° | | | |

### 4.2 Wheel Coulomb

| τ_cw [mN·m] | % τ_cont | θ_ss | φ̇_ss | LC amplitude | ok |
|---|---|---|---|---|---|
| 0 … 40 | | | | | |

### 4.3 Wheel viscous

| b_w | Bleed τ | Drag at cap | θ_ss | ok |
|---|---|---|---|---|
| 0 … 1e-3 | | | | |

### 4.4 Recoverable tilt

| Case | τ_cp | τ_cw | b_w | Edge |
|---|---|---|---|---|
| frictionless | 0 | 0 | 0 | (11.5° from Blocks A–C) |
| light | 1 mN·m | 2 mN·m | 1e-5 | |
| moderate | 3 mN·m | 8 mN·m | 5e-5 | |
| heavy | 8 mN·m | 20 mN·m | 2e-4 | |

---

## 5 — Expected behaviour (to check against)

Written before the run, so the results either confirm or surprise:

- **Pivot Coulomb tolerable to ~5 mN·m.** Beyond that the 1°+ deadband should
  produce visible stick-slip.
- **Wheel Coulomb tolerable to ~10 mN·m** before a limit cycle appears. This is
  uncomfortably close to plausible cogging (5–15 mN·m), so it may well be the
  binding mechanism.
- **Envelope drops ~1° in the "moderate" case**, from 11.5° to ~10.5°.
- **Viscous friction should show no downside** across the whole sweep, and
  should reduce `phid_ss`.

If wheel Coulomb turns out above ~20 mN·m, expect the panel to hunt around
vertical rather than settle — balanced but visibly nervous, which reads badly
in a demo even though it technically passes.

---

## 6 — How to measure each, cheaply

All three are moteus-native. Together they take under an hour and turn this
sensitivity study into a prediction.

### 6.1 Pivot Coulomb — static breakaway

Clamp the wheel. Tilt the panel **slowly** by hand and find the angle at which
it just begins to move on its own. Then

$$\tau_{cp} = Sg\sin\theta_{break} = 0.2623\sin\theta_{break}$$

A protractor and two minutes. Repeat both directions — asymmetry indicates a
bent pin or a misaligned bracket.

### 6.2 Wheel Coulomb **and** viscous — spin-down

Command a wheel speed, cut torque, log ω(t) to a stop.

- **Coulomb** gives a **linear** decay (constant deceleration)
- **Viscous** gives an **exponential** decay

Fit both at once:

$$\dot\omega = -\frac{b_w\omega + \tau_{cw}}{I_w}$$

One test, both constants. Do it from a high speed so the exponential region is
well resolved before the linear tail takes over.

### 6.3 Cogging — slow rotation

Position-control the wheel through one revolution at low speed, log current.
Ripple at **24 cycles/rev** (2 × 12 pole pairs) is cogging; amplitude × Kt
(0.02513 N·m/A) gives the torque.

Note the moteus calibration already characterises some of this — worth checking
whether its stored table can be read out rather than re-measuring.

---

## 7 — Design implications

**If pivot friction is high**, it is a *mechanical* fix, not a control one. A
better bearing surface at the pivot, or a proper ball bearing instead of a pin
in printed plastic. No amount of gain tuning removes a deadband — the
information simply is not there.

**If wheel Coulomb is high**, options in order of cost:
1. Accept the limit cycle if amplitude is small — it is cosmetic, not a failure
2. Add a small dither, or a friction feedforward term of magnitude τ_cw
3. Confirm the moteus cogging compensation is enabled

**Viscous friction should be welcomed, not minimised.** It is the only passive
mechanism that unwinds the wheel. Worth remembering when someone proposes
better bearings "to reduce losses" — losses here are 0.4 mN·m against a 120
mN·m budget, and they buy real drift immunity.

---

## 8 — Caveats

- **`tanh` regularisation understates stiction near zero.** Results are
  optimistic on both Coulomb terms.
- **No Stribeck effect.** Real friction has a velocity-dependent dip between
  static and kinetic — worse stick-slip than modelled.
- **Static ≠ kinetic.** Modelled as one coefficient; breakaway is typically
  20–50 % higher than sliding.
- **Cogging modelled as constant Coulomb**, not as ripple at 24 cycles/rev.
  Adequate for a threshold study, wrong for predicting the ripple spectrum.
- **Perfect state knowledge.** Not combined with the Block C estimator; the two
  effects likely interact, since a deadband offset looks like a tilt bias.
- **Friction is temperature and wear dependent.** Whatever is measured on day
  one will drift.

---

## 9 — Open items

- [ ] Run the script, fill §4
- [ ] Breakaway test → τ_cp
- [ ] Spin-down test → τ_cw and b_w
- [ ] Re-run with measured values, compare to the tolerance thresholds
- [ ] Combine with the Block C estimator — deadband plus gyro bias may compound
- [ ] Check whether the moteus cogging table can be read out

---

## Related

- [[Saturation Envelope Test — Block A]] — actuator authority, and the
  `tanh` regularisation precedent
- [[Discrete Loop Test — Block B]] — loop rate
- [[IMU Lever Arm & Estimator — Block C]] — gyro bias and the standing wheel
  speed that viscous friction helps bleed
- [[Firmware Handoff — Panel Stage]] — the measurement procedures in §6 belong
  in Andrea's test plan
- [[Simscape Panel Model — Build Guide]] — §6.6

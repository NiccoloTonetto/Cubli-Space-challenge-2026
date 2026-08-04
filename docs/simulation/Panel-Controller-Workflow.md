---
tags: [space-challenge, sofia, cubli, controls, simscape, workflow, panel]
---

# Panel Controller Workflow — plant, LQR, Simscape, retuning

The design pipeline for the Stage 1 planar panel controller: from equations to deployable gains, with three retunes as the plant model gets more accurate.

Companions: [[3D Cubli Lagrangian Derivation]], [[Reaction Wheel Sizing Guide]], [[Simulation Strategies]] (pre-Sofia sprint history), [[Week 1 Plan]].

---

## The core principle

The controller design (LQR) is a **one-line call**: `K = lqr(A, B, Q, R)`. It never changes. What changes over the project is the **plant model** that produces $A$ and $B$.

Every deployable set of gains comes from re-running the *same* call with better plant numbers.

So the pipeline has one axis of progress — the plant — and the controller rides along, re-tuning each time.

---

## The three sources of plant parameters

The 1D panel dynamics depend on exactly two scalars: $S = m\ell$ (first moment about the pivot) and $\Theta$ (second moment). Nothing else about the geometry matters.

| Retune | $S$, $\Theta$ from | Purpose |
|---|---|---|
| **1. Estimate** | uniform-plate formulas from the derivation | initial gains; prove closed loop works in sim |
| **2. CAD** | mass-property export from SolidWorks → Simscape XML import | refined gains matching the designed panel |
| **3. Measured** | free-swing period on the built panel (test M1) | deployment gains flashed to the Teensy |

Each retune re-runs `lqr(A,B,Q,R)`. Same code, updated numbers.

---

## The correct step order

Two things that seem intuitive but are backwards, worth stating explicitly:

**Design LQR before Simscape, not after.** LQR is a linear-algebra call that needs $A$, $B$ — which come *from* the plant. You can't design a controller against nothing. Get the derivation, produce $A, B$, run `lqr()` — that gives you gains to *test in* Simscape.

**Simscape does not derive $S$ and $\Theta$.** CAD does. Simscape *uses* them. The retuning source at step 2 is the CAD export, not the simulator.

---

## The pipeline, step by step

### Step 1 — Plant matrices from the derivation
From [[3D Cubli Lagrangian Derivation]] §8 (1D specialisation), the panel dynamics are

$$\bar\Theta\,\ddot\theta = S g \sin\theta - u, \qquad \dot h = u$$

Linearise about $\theta = 0$, state $x = [\theta,\ \dot\theta,\ h]$:

$$A = \begin{bmatrix}0&1&0\\ Sg/\bar\Theta&0&0\\ 0&0&0\end{bmatrix},\quad B = \begin{bmatrix}0\\ -1/\bar\Theta\\ 1\end{bmatrix}$$

In MATLAB: compute $A, B$ from estimated parameters; verify `rank(ctrb(A,B)) == 3`; run `K = lqr(A, B, Q, R)`; check closed-loop poles are stable.

**Ten lines of code, no Simulink needed. This is the design pipeline.**

### Step 2 — Simscape as the nonlinear test bench
Build the panel in Simscape Multibody: ground → revolute joint (pivot) → panel body → revolute joint (motor shaft) → wheel body. Torque input on the wheel joint, angle/rate outputs from both joints.

Wrap the LQR gains from Step 1 around it. Add:
- **Complementary filter** as the estimator (not MEKF — scalar attitude, one axis, keep it simple)
- **Saturation block** at 0.4 N·m on the torque command
- **Discrete rate transition** at 1 kHz on the controller
- **Sensor noise** matching BMI270 specs on the angle output
- **Encoder quantisation** on the wheel angle

Now you have the *nonlinear* plant with realistic actuator and sensor effects, running the discrete-time controller. Verify it still stabilises. If it doesn't, either the gains are too aggressive (raise $R$), or saturation is hit (reduce $Q$ on $\theta$), or the estimator has too much lag (retune the filter).

**What Simscape gives you that MATLAB doesn't:** large-angle effects, saturation behaviour, quantisation, sample-and-hold, sensor noise interaction with the estimator — none of which appear in the linear analysis, all of which appear on hardware.

### Step 3 — CAD delivers real mass properties
When SolidWorks exports the panel geometry, import into Simscape Multibody. The revolute joint frame is set on the pivot edge; Simscape computes $\Theta$ and $S$ from the imported geometry automatically.

Re-linearise (`linmod` or Model Linearizer around $\theta = 0$) to get updated $A, B$. Re-run `lqr(A,B,Q,R)`. New gains for the *designed* panel.

Re-run the Step 2 nonlinear tests with these gains. Iterate on $Q, R$ if needed.

### Step 4 — Measured $\Theta$ from the built panel
This is Stage 1 test M1 ([[Week 1 Plan]]): disable the motor, displace the panel a few degrees, time 20 free-swing oscillations. Then

$$\hat\Theta = \frac{SgT^2}{4\pi^2}$$

Replace CAD's $\Theta$ with $\hat\Theta$ in the Simscape model, re-run `lqr()` one final time. **These are the gains flashed to the Teensy.**

### Step 5 — Hardware validation (M6)
Deploy those gains. Compare the closed-loop response on hardware to the final Simscape prediction. Matching response = model → controller → hardware pipeline validated, scale to 3D. Non-matching (but stable) = the model is missing something specific — friction, compliance, unmodelled cross-coupling — and it's diagnostic gold for the cube stage.

---

## Why MEKF is NOT part of Stage 1

The panel is one axis. Scalar attitude is enough. Use a **complementary filter**: gyro integrated for high-rate estimation, accelerometer's gravity direction for low-frequency correction. Two-line implementation.

Upgrade path if it isn't good enough:
1. Complementary filter → **scalar Kalman filter** (adds gyro bias estimation)
2. Scalar Kalman → **MEKF** (only when moving to 3D on the cube)

Starting with MEKF on the panel is over-engineering: it delays getting closed loop working, and its benefits — proper attitude representation for 3D, quaternion double-cover handling — are irrelevant in one dimension.

**Principle: start with the simplest thing that could work.** Getting plant + LQR + a working filter running end to end teaches you more than a clean MEKF built into a system that doesn't yet balance.

---

## What Simscape validates that MATLAB can't

Five specific things:

1. **Sim-designed gains work on the nonlinear plant** — including large-angle effects the linearisation misses.
2. **Realistic actuator limits** — 0.4 N·m saturation. Does the controller command more than that? When?
3. **Sensor noise and quantisation** — the estimator running on realistic signals, not perfect ones.
4. **Sample-and-hold + discretisation** — the actual discrete-time system at 1 kHz.
5. **Panel-geometry iterations** — change CAD, re-import, re-run. Compare designs without printing anything.

---

## What Simscape does NOT reproduce

Two things that only come from hardware:

- **Real IMU noise/vibration spectrum** on the specific built panel with its specific wheel imbalance
- **Real brake efficiency** of the servo (the reason the panel exists in the first place)

So Simscape closes ~90 % of the design risk, not 100 %. Final tuning still happens on hardware.

---

## The controller "does not change" — a caveat

The *architecture* doesn't change: estimator → LQR → torque command → CAN. The *gains* change three times (steps 1, 3, 4). Each retune is one `lqr()` call with updated $A, B$.

The gain magnitudes typically shift 20–40 % between estimate and measurement — mostly because the uniform-plate approximation is imperfect and print density is variable. This is expected, and it's exactly why the retune loop exists rather than a single design pass.

---

## Summary

- **One controller design pipeline** used three times: `K = lqr(A, B, Q, R)`.
- **Three sources of plant parameters** of increasing accuracy: estimate → CAD → measured.
- **Simscape is the nonlinear test bench**, not a plant-parameter source.
- **Order:** derive → LQR → Simscape closed-loop test → CAD-update → re-tune → measure → re-tune → deploy → hardware validate.
- **Complementary filter for Stage 1**, MEKF only on the cube.

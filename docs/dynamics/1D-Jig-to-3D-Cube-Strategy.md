---
tags: [space-challenge, sofia, cubli, prototyping, controls, validation, strategy]
---

# The 1D Jig → 3D Cube Strategy — what each stage validates and what transfers

The single most important strategic decision in the build: **we do not build the 3D cube first.** We build a 1-dimensional jig, prove the entire control loop on it, then replicate that proven loop into 3D. This note explains exactly *why*, exactly *what the jig validates*, and exactly *what transfers to the cube and how*.

The core claim: **these are not three separate prototypes. They are one system revealed in increasing dimension.** Each stage is the next stage minus a dimension, and nearly everything — theory, code, estimator, tuning method — carries forward untouched.

Companion notes: [[Sizing Memo]], [[Structural Constraints]], [[3D Cubli Lagrangian Derivation]] (§8 is the mathematical basis for why 1D→3D works), [[Revised Schedule - Provided Hardware]].

---

## 1. The three stages and the ONE risk each retires

A prototype earns its place only if it lets us **fail cheaply now instead of expensively later.** By that test, exactly three stages, no more:

| Stage | Physical form | The one question it answers |
|---|---|---|
| **Simscape model** | virtual | Is the control *theory* correct at all? |
| **1D jig** | 1 motor, 1 wheel, balances on an **edge** | Does the whole control *loop* work on *real hardware*? |
| **3D cube** | 3 motors, 3 wheels, balances on a **corner** | Do the genuinely-3D problems (coupling, allocation, frames) work? |

The logic: the Simscape model kills all *theory* risk with zero hardware. The jig kills all *component and single-loop* risk with one motor. By the time we build the cube, the only risks left are the ones that are *inherently* three-dimensional — everything else is already dead.

---

## 2. Why the 1D jig is the highest-leverage object in the whole project

This is the stage people skip to "save time," and it is the single most expensive mistake available to us. Here is the exact reasoning.

### 2.1 What the jig physically is

An arm that pivots on a single horizontal axis (an **edge**, not a corner), with **one** motor driving **one** reaction wheel whose spin axis is parallel to the pivot. It balances like a broomstick on a hand, but in one plane only. That's it. A printed arm, one motor, one wheel, one encoder, one IMU, the real driver, the real compute.

### 2.2 The physics is IDENTICAL to the cube, minus two axes

This is the crucial fact and the mathematical justification is already in our derivation. From [[3D Cubli Lagrangian Derivation]] §8: **the 1D edge-balancing equation is the full 3D equation with the cross-product (gyroscopic) terms set to zero.**

$$\underbrace{(\hat\Theta_e - I_w)\,\ddot\theta = mg\ell\sin\theta - u}_{\text{1D jig}} \qquad\Longleftarrow\qquad \underbrace{\bar\Theta\,\dot{\boldsymbol\omega} = -\boldsymbol\omega\times(\bar\Theta\boldsymbol\omega + A\boldsymbol h) + m\,\boldsymbol c\times\boldsymbol g_B - A\boldsymbol u}_{\text{3D cube}}$$

The 1D case is not an *analogy* for the cube. It is *literally the same system of equations*, evaluated on a single axis where the $\boldsymbol\omega\times(\cdot)$ coupling vanishes. Same instability ($mg\ell$ gravity term), same actuation ($u$), same wheel-momentum bookkeeping ($\dot h = u$). A controller that balances the jig is solving the identical mathematical problem the cube solves, in one axis.

### 2.3 What the jig VALIDATES — the exhaustive list

Everything below is **1-dimensional physics**, so it can be tested and fixed with a single motor, but every item is a risk that would otherwise surface — coupled with two other axes — in the 3D cube during week 3, when there is no schedule slack:

**Hardware/component risks:**
1. **Torque-mode driver actually works.** Our entire model assumes the driver takes a *torque* command ($\dot h = u$). If it's secretly speed-mode or the FOC tuning is off, *nothing* works. The jig proves this with one motor. (This is the open EnduroSat question — the jig is where it gets answered definitively.)
2. **Encoder mount survives at speed.** An outrunner encoder that slips or loses counts at 8000 rpm is a mechanical problem you want to find on a jig, not inside an assembled cube.
3. **Real control loop rate and jitter.** We computed a ~200 Hz minimum; the jig *measures* what the hardware actually delivers, under real sensor-read and motor-write load.
4. **Sensor noise on the real IMU.** The estimator was designed against modeled noise; the jig exposes it to the actual gyro/accel noise, vibration, and bias.

**Control/estimation risks:**
5. **The estimator works on real signals.** Complementary filter or Kalman — does it reconstruct the tilt angle from real, noisy, vibrating sensor data? 1D is the honest first test.
6. **LQR gains from the model transfer to hardware.** We design gains in sim; the jig is the first proof that a model-designed controller stabilizes *real* hardware. If it doesn't, the model is wrong and we fix it here, cheaply.
7. **Parameter identification method works.** Free-swing period → inertia, spin-down → friction (see [[Deriving Dynamics from CAD]] §4). The jig is where we practice and validate the ID procedure that we'll later apply to the cube.

**The one number jump-up depends on:**
8. **Brake efficiency $\eta_{brake}$.** Our jump-up feasibility (margins ×1.3+) assumes $\eta_{brake} \approx 0.65$ — an *unmeasured estimate*. The jig is where we spin a wheel up, brake it, and *measure* how much momentum actually transfers. Until this number is measured, jump-up is a hypothesis. The jig turns it into a fact.

### 2.4 Why skipping it is the expensive path

Skip the jig, build the full cube, and when it won't balance the cause could be **any** of: bad gains, wrong encoder counts, slow loop, estimator noise, a coordinate-frame sign error, driver not in torque mode, mechanical wheel imbalance, cross-axis coupling — **across three coupled axes simultaneously.** Isolating one failure among eight candidates times three axes is a multi-day debugging nightmare, and it lands in week 3.

The jig collapses that search space. It isolates risks 1–8 in one plane with one of everything, where the physics is trivial to reason about. **The jig does not add a build — it removes days of week-3 debugging.** That is why it is faster, not slower, under time pressure.

---

## 3. What EXACTLY transfers from jig to cube, and how

The scalability question decided whether this strategy is efficient. Answer: nearly everything transfers, because the jig and cube share structure by design. Item by item:

### 3.1 The equations of motion — transfer: STRUCTURAL, exact
The jig's equation *is* the cube's equation restricted to one axis (§2.2). Going 1D→3D you don't rewrite the dynamics — you **re-enable the terms you had zeroed**: the gyroscopic coupling $\boldsymbol\omega\times(\cdot)$ and the two extra axes. The gravity term, the actuation term, the wheel dynamics are unchanged in form. In the code, the 1D plant model is the 3D plant model with `A = a_single_axis` instead of `A = eye(3)`.

### 3.2 The control law — transfer: IDENTICAL structure, re-solved
LQR is LQR. On the jig you solve `K = lqr(A,B,Q,R)` for a 3-state model; on the cube you solve the *same call* for a 9-state model. The **design method is byte-for-byte identical** — same cost-function philosophy (penalize tilt, rate, and wheel momentum), same Riccati solve, same tuning intuition for Q and R. What changes is only the matrix dimensions. Every hour spent understanding *why* a gain choice behaves a certain way on the jig is directly reusable on the cube.

### 3.3 The firmware — transfer: ~80% reused verbatim
This is the big one. The firmware architecture is **identical**:
```
read sensors → estimate state → compute u = -K*x → command torque → repeat
```
- **Sensor drivers** (IMU read, encoder read): identical code, just instantiated 3× on the cube.
- **Estimator**: same filter, extended from scalar angle to attitude — same structure, more states.
- **Control loop timing, torque command path, safety cutoffs**: identical.
- **What's genuinely new on the cube**: reading 3 encoders instead of 1, the 3-axis attitude estimator (quaternion instead of scalar angle), and the control *allocation* (mapping 3 desired torques to 3 wheels — trivial for orthogonal wheels, $A = I$). That's it. The scaffolding — the ~80% that is tedious to get right and easy to get subtly wrong — is written and debugged *once*, on the jig.

### 3.4 The estimator — transfer: same structure, +2 axes
Jig: reconstruct scalar tilt $\theta$ from gyro + accelerometer. Cube: reconstruct attitude (quaternion / $\boldsymbol g_B$) from gyro + accelerometer. **Same sensor-fusion principle** (integrate gyro, correct drift with the gravity vector from the accelerometer), same tuning approach, extended from 1 angle to a reduced-attitude vector. The 1D version is the honest debugging ground for the fusion logic before 3D bookkeeping is added.

### 3.5 The parameter-ID procedure — transfer: identical method
Free-swing period → inertia; spin-down → friction; spin-up → wheel inertia and torque-mode verification (see [[Deriving Dynamics from CAD]] §4). **The exact same measurements**, done on the jig first (fast, one axis) then on the assembled cube (three axes). You learn the procedure where it's cheap.

### 3.6 The Simscape model — transfer: 1D model → 3D model, same pattern
The virtual side scales the same way: `cubli_1d.slx` (one body, one wheel, revolute pivot) → `cubli_3d.slx` (one body, three wheels, spherical pivot). Same parameter file (`cubli_params.m`), same LQR-from-linearization workflow, same validation checklist. The 1D Simscape model validates against the jig; the 3D model validates against the cube. Structure identical, dimension increased.

### 3.7 Summary table — the transfer map

| Element | Jig → Cube | What's actually new |
|---|---|---|
| Equations of motion | structural, exact | re-enable gyroscopic + 2 axes |
| LQR control law | identical method | 3→9 states, same solve |
| Firmware | ~80% verbatim | 3× sensors, attitude estimator, allocation |
| Estimator | same fusion principle | scalar angle → reduced attitude |
| Parameter ID | identical procedure | done 3× instead of 1× |
| Simscape model | same workflow | 1 wheel → 3 wheels, revolute → spherical |

**Nothing is thrown away.** The jig is not a throwaway prototype — it is the cube's control system, born and debugged in one dimension.

---

## 4. What does NOT transfer (the genuinely-3D problems)

Being honest about the boundary. These are the risks the jig *cannot* retire, and therefore the *only* things that should still be unknown when we start the cube:

1. **Gyroscopic cross-coupling.** The $\boldsymbol\omega\times(\cdot)$ terms that are zero in 1D. At low balancing speeds these are small, but they're real and only appear in 3D.
2. **Control allocation across 3 wheels.** Trivial for orthogonal wheels (identity map), but it's new code.
3. **Full attitude estimation.** Quaternion/reduced-attitude fusion vs. scalar angle — more bookkeeping, the double-cover and yaw-unobservability subtleties from [[Quaternions Complete Guide]].
4. **Corner pivot vs. edge pivot.** Geometry, the hardened tip, 3-axis symmetry of the mechanical build.

That's the whole list. Because the jig retired everything else, the cube's bring-up is *only* these four — a tractable problem, not an eight-headed one.

---

## 5. The bottom line for scheduling

- The jig is a **parallel week-1 workstream** — with ~15 people it costs nothing on the critical path and buys down the most risk of any single activity.
- It is the first place the **open driver-mode question** and the **unmeasured brake efficiency** get definitively answered — both of which gate later decisions.
- If (and only if) the team were too small to parallelize it, the jig would be the first cut — but almost everything else should be cut before it, because it is the highest-leverage risk-reducer in the plan.

**One system, three dimensions of it. Build the 1D truth first; the 3D cube is that truth replicated three times.**

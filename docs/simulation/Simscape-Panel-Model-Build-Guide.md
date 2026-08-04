---
tags:
  - space-challenge
  - sofia
  - cubli
  - simscape
  - simulation
  - panel
  - controls
  - validation
  - cad
---

# Simscape Panel Model — Build Guide

Exact build procedure for the **Stage 1 planar panel** multibody model.
Companion to [[Panel Controller Workflow]] (which says *why*) and
[[Simulation Strategies]]. This note says *how*, block by block.

**Prerequisites**
- MATLAB + Simulink + Simscape + Simscape Multibody
- Simulink Control Design (needed for `linearize` in Gate 3)
- `cubli_panel_params.m` in `~/Documents/MATLAB/Space Challenge 2026/`
- The two STEP files, decompressed from `.stpZ` (they are gzip):
  `gunzip -c Static_Assembly_CORRECTED_AXES_A.stpZ > static.step`

**Estimated build time:** ~1 hour to a model that passes all six gates.

---

## 0 — The one decision that saves the most time

**Do NOT use `smimport`.**

Three reasons:
1. The `MN4006` arrives as **90 separate solids**. `smimport` generates a
   block for each. The model becomes unreadable and slow to compile.
2. `smimport` assigns **one density per imported body**. Both STEP files mix
   steel, plastic and motor. Any single density is wrong for most of the mass.
3. CATIA did not export materials at all, so there is nothing to inherit.

**Instead:** hand-build a **two-body** model. Use `File Solid` blocks with
**Inertia → Custom**, fed from `cubli_panel_params`. The STEP geometry then
drives *only* the rendering; the dynamics come from numbers you control and
can sweep. This is what makes the three retunes in [[Panel Controller Workflow]]
a one-line change.

> If the mesh looks 20 mm out of place in Mechanics Explorer, that is
> **cosmetic**. It cannot corrupt the plant. Do not spend time on it until
> the gates pass.

---

## 1 — Plant numbers

From the CATIA geometry, densities applied by part name
(steel 7850, motor 6793 kg/m³, plastic = `rho_plastic` argument).

| Quantity | Symbol | ρ = 1300 | ρ = 845 |
|---|---|---|---|
| Panel body mass (frame + motor) | `m_panel` | 0.3258 kg | 0.2532 kg |
| Wheel body mass | `m_wheel` | 0.1294 kg | 0.0940 kg |
| Total | | 0.4552 kg | 0.3472 kg |
| COM distance from pivot | $\ell$ | 100.41 mm | 100.92 mm |
| First moment | $S = m\ell$ | 0.04570 kg·m | 0.03504 kg·m |
| Second moment about pivot | $\Theta$ | 6.341e-3 kg·m² | 4.728e-3 kg·m² |
| Wheel inertia | $I_w$ | 3.581e-4 kg·m² | 2.789e-4 kg·m² |
| $\bar\Theta = \Theta - I_w$ | | 5.983e-3 | 4.449e-3 |
| $Sg$ | | 0.4484 N·m | 0.3437 N·m |
| Unstable pole | $\lambda$ | **8.657 rad/s** | **8.790 rad/s** |
| Free-swing period (hanging, wheel locked) | $T$ | **0.747 s** | **0.737 s** |

> [!important] $\lambda$ moves 2.9 % across a 2.2× density range
> $S$ and $\Theta$ both scale with mass, so the ratio that sets the time
> constant is nearly invariant — the same "mass cancels" result from the
> sizing notes. **Do not block the build on the density measurement.**
> Build at ρ = 845, pass the gates, then re-run with the measured value.

**Actuation ratios** (these are *not* invariant — see §6):

| ρ | $\tau_{cont}/Sg$ | $\theta_{static}$ cont | $\theta_{static}$ peak | $X = h_{max}^2/(2\Theta S g)$ |
|---|---|---|---|---|
| 1300 | 0.268 | 15.5° | 63.1° | 17.6 |
| 845 | 0.349 | 20.4° | **unbounded** | 18.7 |
| 700 | 0.387 | 22.7° | **unbounded** | 19.2 |

---

## 2 — Coordinate and sign conventions

Fix these **before placing a single block**. Every sign error downstream
traces back to skipping this section.

### World frame
- **+X** right, **+Y** up, **+Z** out of the panel plane, toward the viewer.
- Gravity: `Mechanism Configuration → Uniform Gravity = [0 -9.81 0]`.
- The panel plane is the **world XY plane**. Both rotation axes are **+Z**.

This choice is deliberate: Simscape's `Revolute Joint` always rotates about
the **Z axis of its base frame** (the `Rz` primitive). Putting the panel
normal on Z means **no rotation is needed on any Rigid Transform** — pure
translations only. Fewer places to get a sign wrong.

### Pivot location
The panel pivots about a **corner** of the 150 mm square, axis normal to the
panel plane. In the static STEP file the square spans 0…150 mm in x and y,
so the pivot corner is the **file origin (0, 0)** and the wheel axis is at
**(75, 75) mm**.

> [!warning] Verify this on the hardware
> If the pivot is actually at an **edge midpoint**, then $\ell$, $\Theta$ and
> the mount angle all change and this whole note needs rederiving.
> Check the bracket position on the built panel before trusting the numbers.

### Mount angle
At $\theta = 0$ the total COM must be **directly above the pivot**. The COM
sits at (70.98, 71.02) mm from the pivot corner — i.e. along the diagonal at
45.014°. So the panel geometry must be rotated by

$$\phi_{mount} = 90° - \operatorname{atan2}(c_y, c_x) = 44.986°$$

about Z to bring the diagonal vertical. This is **almost exactly 45°** and
almost density-independent (44.9863° at ρ=1300, 44.9876° at ρ=845), but
compute it rather than hard-coding it. Add to `cubli_panel_params.m`:

```matlab
p.phi_mount = pi/2 - atan2(p.com_total(2), p.com_total(1));   % rad
```

### State and torque signs
- $\theta$ = pivot joint position, **zero = upright**, positive = the
  direction of positive rotation about +Z.
- $\phi$ = wheel joint position **relative to the panel**.
- $u$ = torque commanded at the **wheel** joint.

$$\bar\Theta\,\ddot\theta = Sg\sin\theta - u, \qquad \dot h = u$$

> [!tip] Simscape gives you the reaction torque for free
> Actuating the wheel joint applies $+u$ to the follower (wheel) and $-u$ to
> the base (panel) automatically. **Do not add a reaction torque anywhere.**
> If you find yourself typing a minus sign to make the panel respond to the
> wheel, something upstream is wrong — go back and fix it, don't compensate.

---

## 3 — Block-by-block build

### 3.1 Skeleton

```
Solver Configuration ──┐
World Frame ───────────┤
  │                    │  (both connect to the same physical network)
  ├─ Rigid Transform  T_MOUNT      rotate +phi_mount about Z
  │    └─ Revolute Joint  PIVOT    q = theta
  │         └─ File Solid  PANEL_BODY
  │              └─ Rigid Transform  T_WHEEL   translate [0.075 0.075 z_w]
  │                   └─ Revolute Joint  WHEEL   q = phi, torque actuated
  │                        └─ File Solid  WHEEL_BODY
Mechanism Configuration (unconnected, sets gravity)
```

Two rigid bodies. Two joints. That is the entire mechanical model.

### 3.2 Block parameters — exact entries

**Solver Configuration** (Simscape → Utilities)
- Leave defaults for now. Local solver settings go in §7.

**Mechanism Configuration** (Simscape → Multibody → Utilities)
- Uniform Gravity: `Constant`
- Gravity: `[0 -9.81 0]`
- Linearization Delta: leave default

**World Frame** (Simscape → Multibody → Frames and Transforms)
- No parameters.

**Rigid Transform `T_MOUNT`**
- Rotation → Method: `Standard Axis`
- Axis: `+Z`, Angle: `p.phi_mount` , units `rad`
- Translation → Method: `None`

**Revolute Joint `PIVOT`** (Simscape → Multibody → Joints)
- **State Targets**
  - Position → Specify: ✅, Value `0`, Priority `High`
  - Velocity → Specify: ✅, Value `0`, Priority `High`
- **Internal Mechanics**
  - Equilibrium Position `0`, Spring Stiffness `0`
  - Damping Coefficient `0` ← **must be zero for Gates 3–5**
- **Limits** — leave off for now (added in §6)
- **Actuation**
  - Torque: `None`
  - Motion: `Automatically Computed`
- **Sensing**
  - Position ✅, Velocity ✅

**File Solid `PANEL_BODY`** (Simscape → Multibody → Body Elements)
- **Geometry**
  - File Type: `STEP`
  - File Name: path to the decompressed static STEP
  - Units: `mm`
- **Inertia**
  - Type: **`Custom`** ← the critical setting
  - Mass: `p.m_panel`
  - Center of Mass: `[p.com_panel(1) p.com_panel(2) 0]`
  - Moments of Inertia: `[p.Izz_panel_com/2 p.Izz_panel_com/2 p.Izz_panel_com]`
    - *(planar model — only Izz enters the dynamics; the in-plane terms are
      placeholders and never affect the result. Set them properly only if you
      later reuse this body in a 3D model.)*
  - Products of Inertia: `[0 0 0]`
- **Graphic** → Visual Properties → simple, colour to taste
- **Frames** → add a frame `W` at the wheel axis if you prefer that to a
  separate Rigid Transform (either works; the separate block is easier to read)

> The STEP file's own origin is the pivot corner, so the graphic lands
> correctly with no offset. The z extent (0…55 mm) will render the panel
> in front of the XY plane — harmless.

**Rigid Transform `T_WHEEL`**
- Rotation → Method: `None`
- Translation → Method: `Cartesian`
- Offset: `[0.075 0.075 p.z_wheel]` , units `m`
  - `p.z_wheel` is cosmetic. Pick whatever makes the render look right;
    it has **zero** effect on planar dynamics.

**Revolute Joint `WHEEL`**
- **State Targets**
  - Position → Specify: ✅, Value `0`, Priority `Low`
  - Velocity → Specify: ✅, Value `0`, Priority `High`
- **Internal Mechanics**
  - Damping Coefficient `0` ← zero for the gates, identified later (§6)
- **Actuation**
  - Torque: **`Provided by Input`**
  - Motion: `Automatically Computed`
- **Sensing**
  - Position ✅, Velocity ✅

**File Solid `WHEEL_BODY`**
- **Geometry**: the moving-assembly STEP, units `mm`
- **Inertia**
  - Type: `Custom`
  - Mass: `p.m_wheel`
  - Center of Mass: `[0 0 0]` (the wheel is balanced on its axis by design —
    the CAD confirms COM within 0.01 mm of the axis)
  - Moments of Inertia: `[p.Iw/2 p.Iw/2 p.Iw]`
  - Products of Inertia: `[0 0 0]`

### 3.3 Signal interfacing

- Torque input: **Simulink-PS Converter** into the `WHEEL` joint `t` port.
  Input Signal Unit: `N*m`. Filtering: `Filtering and derivatives` →
  **do not** leave this on `Provide input derivatives` unless you actually
  supply them; a first-order filter here will quietly add lag that shows up
  as a mysterious phase loss in Gate 3.
- All sensing ports: **PS-Simulink Converter**, units `rad` and `rad/s`.
- Bus the three states into one signal in the order **`[theta; theta_dot; phi_dot]`**
  to match the ETH convention and `cubli_lqr_design.m`.

### 3.4 Model callbacks

`Model Properties → Callbacks → InitFcn`:

```matlab
p = cubli_panel_params(845);     % measured rho goes here later
```

Everything in the block dialogs references `p.*`. One number changes,
the whole model follows.

---

## 4 — Validation gates

Same discipline as the eleven gates in `cubli_lqr_design.m`. **Nothing from
§6 goes in until all six pass.** Keep them as a script,
`cubli_panel_simscape_gates.m`, so they re-run after every change.

### Gate 1 — Hang test
Gravity on, zero torque, `WHEEL` joint temporarily **locked**
(set Actuation → Motion to `Provided by Input` with a constant 0).
Release from near-hanging.

**Pass:** the panel settles with the total COM directly *below* the pivot,
i.e. $\theta \to 180°$.
**Catches:** wrong gravity direction, wrong mount angle, COM entered in the
wrong frame. Takes 30 seconds and catches the most embarrassing errors.

### Gate 2 — Free-swing period
Small oscillation about hanging, wheel locked, zero damping.

$$T = 2\pi\sqrt{\Theta/(Sg)} = 0.747\ \text{s}\ (\rho{=}1300),\quad 0.737\ \text{s}\ (\rho{=}845)$$

**Pass:** simulated period within 0.5 % of analytic.
**Why this gate matters most:** it is the **same measurement you make on the
built panel** — a stopwatch and a phone camera. That makes it a three-way
check: analytic ↔ Simscape ↔ hardware. It is also how you obtain measured
$\Theta$ for the deployment gains in Step 4 of [[Panel Controller Workflow]].

### Gate 3 — Linearisation gate ★
The definitive one. Upright with zero torque **is** an exact equilibrium
(unstable, but exact), so **no trimming is needed** — linearise at `t = 0`
with the state targets already at zero.

```matlab
mdl = 'cubli_panel';
io(1) = linio([mdl '/TorqueIn'],  1, 'openinput');
io(2) = linio([mdl '/StateOut'],  1, 'openoutput');
sys   = linearize(mdl, 0, io);
[Aq, Bq] = ssdata(sys);

fprintf('A err: %.3e\n', norm(Aq - p.A)/norm(p.A));
fprintf('B err: %.3e\n', norm(Bq - p.B)/norm(p.B));
assert(norm(Aq - p.A)/norm(p.A) < 1e-6, 'A mismatch')
assert(norm(Bq - p.B)/norm(p.B) < 1e-6, 'B mismatch')
```

**Pass:** relative error < 1e-6.
If it passes, the multibody model and the LQR design script are **provably
the same plant**, and every gain from `lqr()` is valid in the sim.

**Reading the failure pattern:**

| Symptom | Cause |
|---|---|
| `A(2,1)` off by a scale factor | $S$ or $\bar\Theta$ wrong → check `m_panel`, `com_panel` |
| `B(3)` off | $I_w$ wrong → check `Custom` inertia actually took, not `Calculate from Geometry` |
| `B(2)` sign flipped | reaction torque convention → you added a manual reaction somewhere |
| `A(3,1)` ≠ `-A(2,1)` | the two bodies are not sharing one pivot correctly |
| Everything ~0 | linearisation I/O points not on the right lines |

### Gate 4 — Pole match
```matlab
eig(Aq)   % expect  +8.657, -8.657, 0   (rho = 1300)
```
Mirrors Gate 1 of `cubli_lqr_design.m`, now on the multibody model.

### Gate 5 — Energy conservation
Damping zero, torque zero, release from $\theta_0 = 20°$.
Total mechanical energy constant to solver tolerance (< 0.1 % drift over 10 s
on `ode23t` with `RelTol = 1e-8`).
**Catches:** inertia errors that Gates 3–4 can miss if two compensating
mistakes were made.

### Gate 6 — Momentum coupling
Gravity **off**, pivot free, torque impulse on the wheel.

**Pass:** angular momentum about the pivot stays zero —
$$\bar\Theta\,\Delta\dot\theta = -I_w\,\Delta\dot\phi$$

Isolates the coupling term, which is precisely what control authority
depends on. A model can pass Gates 1–5 and still have the coupling scaled
wrong if the wheel body is attached to the wrong frame.

---

## 5 — Gate checklist

- [x] Gate 1 — hang test settles COM below pivot
- [x] Gate 2 — period within 0.5 % of $2\pi\sqrt{\Theta/Sg}$
- [x] Gate 3 — `norm(Aq-p.A)/norm(p.A) < 1e-6` and same for B
- [x] Gate 4 — poles $\{+\lambda, -\lambda, 0\}$
- [x] Gate 5 — energy drift < 0.1 % over 10 s
- [x] Gate 6 — $\bar\Theta\Delta\dot\theta + I_w\Delta\dot\phi = 0$

---

## 6 — The nonlinear layer

This is what Simscape is **for** — see "What Simscape validates that MATLAB
can't" in [[Panel Controller Workflow]]. Add in this order. After each
addition, **disable it and re-run Gate 3** to confirm you haven't changed
the underlying plant.

### 6.1 Torque saturation
Saturation block on the torque command: `[-p.tau_cont, +p.tau_cont]`,
with a separate peak branch at `p.tau_peak` if you model a duty limit.
The panel is genuinely torque-limited (15.5° static at ρ=1300), so this
bites and is worth modelling honestly.

### 6.2 Wheel speed cap — **mandatory, not optional**
As built, $X = 17.6$: the panel could recover from beyond 180° on momentum
alone, and balancing at 10° uses **2.9 % of $h_{max}$**. It is impossible to
provoke momentum saturation. Worse, below ρ ≈ 900 the *peak* torque exceeds
$mg\ell$ and the static torque bound disappears too — the panel becomes
over-actuated in **both** channels, exactly the failure mode flagged in the
sizing notes.

So cap both, in **sim and firmware, from the same struct**:

| Cap | Value | Resulting bound |
|---|---|---|
| `p.omega_cap` | 40 rad/s | $X_{cap} \approx 0.038$ → ~15.9° |
| torque limit | 0.10–0.12 N·m (moteus current limit) | ~20° static |

$X_{cap}$ moves only 0.036 → 0.039 across the whole density range, so this
cap is safe to fix now and never revisit.

> Without these caps the saturation gate is testing an envelope the hardware
> will never see, and the classic "balances 8 seconds then falls" failure
> (see [[Disturbance Estimation & Attitude Representation]]) cannot be
> reproduced in simulation before you meet it on the bench.

### 6.3 Encoder quantisation
Quantizer blocks on both joint positions, LSB `p.enc_lsb = 2*pi/2^16`
(MA600, 65536 cpr). Then **differentiate the quantised signal** to get rate —
do not sense velocity directly from the joint. The derivative noise this
produces is real and is what forces the filter design.

### 6.4 Sample-and-hold and computational delay
Zero-Order Hold at `p.f_outer`, plus **one sample of delay**.
Do not skip the delay. It is the difference between margins that survive
deployment and margins that don't, and it is invisible in the continuous
design. See [[Simulation Strategies]] on loop-rate selection — remember the
binding constraint is anti-aliasing the ~141 Hz wheel fundamental, not
control bandwidth.

### 6.5 IMU model
Mount a **Transform Sensor** at the *actual* IMU body-frame position on the
panel — **not** at the pivot. Output **specific force**, not tilt angle.

This is the single most valuable thing the panel sim does: a lever arm $r$
from the pivot injects an $\dot\omega \times r$ error that is worst exactly
during recovery, when you most need the estimate. Record the IMU's
body-frame position from CAD, feed the complementary filter the raw
accelerometer and gyro signals, and let the filter deal with it.

Add: accelerometer noise, gyro noise, gyro bias random walk.

### 6.6 Friction
- `PIVOT`: viscous + Coulomb
- `WHEEL`: viscous + cogging torque ripple (12 pole pairs → 24 cycles/rev)

Leave the coefficients as parameters. Identify them from a **spin-down test**
on the real wheel and a **decay-envelope fit** on the free-swing test from
Gate 2 — both are measurements you're doing anyway.

### 6.7 Hard stops
`PIVOT` joint → **Limits** → lower and upper bound matching the **grey rails
on the base plate**. Those rails define the safe test envelope. Having them
in the model means aggressive tuning runs terminate meaningfully instead of
producing nonsense trajectories.

---

## 7 — Solver and performance

| Setting | Development | Controller-in-the-loop |
|---|---|---|
| Solver | `ode23t`, variable step | `ode4`, fixed step `1e-4` |
| RelTol | `1e-8` (needed for Gate 5) | n/a |
| Simscape local solver | off | consider `Backward Euler`, `1e-4` |

- Fixed-step `ode4` at 1e-4 is the honest test: it tells you whether the
  **discrete** implementation works, not just the continuous design.
- For batch runs (ρ sweeps, ±30 % robustness, gain sweeps) disable
  visualisation — typically a 5–10× speedup:
  ```matlab
  set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');
  ```
- Lower the `File Solid` graphic tessellation. The motor's 90 solids will
  make Mechanics Explorer crawl and contribute **nothing**, since inertia
  is `Custom`.
- Use `parsim` for the robustness sweep once the gates pass.

---

## 8 — Retune hooks

The three retunes from [[Panel Controller Workflow]] are all one line here:

| Retune | Action |
|---|---|
| 1. Estimate | `p = cubli_panel_params(845);` |
| 2. CAD / measured density | `p = cubli_panel_params(rho_measured);` |
| 3. Measured $\Theta$ | override `p.Theta` from the Gate 2 period on hardware, recompute `p.A`, `p.B` |

After each: re-run Gate 3, then `lqr()`. The gains will barely move —
$\lambda$ shifts under 3 % across the plausible density range — but the
**actuation ratios** in §6.2 will, so re-check the caps.

---

## 9 — Open items

- [ ] **Confirm the pivot is at a corner, not an edge midpoint.** Everything
      in §2 assumes the corner. Measure the bracket on the built panel.
- [ ] **Measure $\rho_{eff}$.** 15 % infill is *not* a 0.15 density factor —
      perimeters and top/bottom layers dominate on a ribbed structure, so
      expect 40–60 % of nominal. Fastest routes: read the slicer's estimated
      filament mass per part, or weigh the assembled panel and invert:
      $$\rho_{eff} = (M_{measured} - 0.14656)/2.3740\times10^{-4}$$
      (fixed steel + motor mass 146.56 g; total plastic volume 237.40 cm³)
- [ ] **Motor rotor/stator split.** The `Moving Assembly` wraps the whole
      MN4006 with no split, and this model lumps all 90 solids into the panel
      body. The bell and magnets really do spin — worth about **+2–3 % on
      $I_w$**. Ask Andrea for the bell solid names if we want it exact; it
      sits below the plastic-density uncertainty either way.
- [ ] **IMU body-frame position** — needed from CAD for §6.5.
- [ ] Identify friction coefficients from spin-down and swing-decay tests.

---

## Related

- [[Panel Controller Workflow]] — why this model exists, and the retune order
- [[Simulation Strategies]] — what Simscape does and does not reproduce
- [[Disturbance Estimation & Attitude Representation]] — why $Q_h$ is 1–10
  and why a constant disturbance can't be cancelled
- [[References]]

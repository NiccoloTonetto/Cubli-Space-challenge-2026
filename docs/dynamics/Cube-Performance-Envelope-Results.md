---
tags:
  - space-challenge
  - sofia
  - cubli
  - cube
  - simulation
  - results
---

# Cube Performance Envelope — Simulation Results

**Scope note, read first:** this document compiles the **corner-balance**
simulation study (MATLAB design plant + Simscape multibody validation +
nonlinear sensor/actuator sim, run before any corner hardware bring-up).
**No equivalent simulation study exists for edge balance** — edge-bringup
went directly from the reduced-attitude estimator design to staged
hardware bring-up (`edge-bringup/Stage1-5`) without a prior MATLAB/Simscape
envelope study. If the results section needs an edge-balance performance
figure, the source is the **hardware telemetry** from `edge-bringup`
and `FINAL/EdgeBalance*` runs, not a simulation — flag this distinction
in the write-up rather than presenting edge alongside corner as if both
were simulated the same way.

Recovered from the original project design conversation (two source
documents, "Cube envelope - methodology and results.md" and "Final
configuration and per-corner results.md" — not currently committed as
files in this repo; the MATLAB scripts they reference, listed at the end,
aren't in `matlab/3Dmodel/` either, only their outputs are reproduced
here). Presented in two parts: the methodology + full sweep results, then
the corrected final configuration that supersedes some of the
intermediate numbers below (noted inline where they differ).

---

## Part 1 — Methodology

### Why three models, not one

| model | purpose | cost |
|---|---|---|
| `cubli_cube_params` + `cubli_corner_plant` | 9-state linear design plant, one per corner | instant |
| `simscape3d.slx` | independent multibody truth, imported STEP geometry | ~20 s per run |
| `cubli_nlsim` | full nonlinear plant + sensors + actuator limits | ~50 ms per run |

The Simscape model is the reference but far too slow for the thousands of
runs a bisection sweep needs. The linear model can't see saturation,
gyroscopic coupling, or sensor geometry at all. `cubli_nlsim` is the
working tool — its only job is to be fast and to agree with Simscape.

**Nonlinear plant state**: `x = [gam(3); om(3); rho(3)]` — `gam` is the
gravity direction in BODY coordinates (what an ideal accelerometer
reports), `om` is body rate about the contact point, `rho` is wheel rate
relative to the body (what the encoder reads).

```
L      = Theta*om + Is*rho
Tb*omd = m*g*(r_c x gam) - u_eff - om x L - tau_pivot
Is*rhod= u_eff - Is*omd
gamd   = -om x gam
```

`Theta` is body-constant (wheels axisymmetric about their spin axes).
Linearizing `m*g*(r_c x gam)` about `gam = gB` with `r_c = -ell*gB`
recovers `Sg*P*phi` exactly — the consistency check between this model and
the design plant. Using `gam` rather than Euler angles is deliberate: no
parameterization, no singularity, and it's the same quantity the firmware
actually has.

### Validation chain

1. `cubli_corner_plant` reproduces `cubli_cube_params` for the primary
   corner to all printed digits (`ell`, `Sg`, `lambda`).
2. Simscape Gate 3 at the corner: `A err 1.740e-07`, `B err 2.128e-16`.
3. `cubli_nlsim` vs Simscape, 2° recovery, ideal sensors:

   | t [s] | nonlinear tilt | Simscape tilt |
   |---|---|---|
   | 0.0 | 1.6254 | 1.6254 |
   | 0.1 | 1.0766 | 1.0854 |
   | 0.3 | 0.1135 | 0.1108 |
   | 0.6 | 0.3369 | 0.3389 |
   | 1.0 | 0.1224 | 0.1285 |

   Agreement better than 1% through the transient; late-time divergence
   is Simscape solver tolerance, not a modelling difference.
4. Every gain set reports `Ms`. The reduced-order LQR return-difference
   identity gives `Ms = 1.000000` exactly — any deviation is an
   implementation bug, asserted rather than inspected.

### The metric: worst-case recovery angle

**Definition**: the largest initial tilt, released from rest with wheels
stationary, from which the cube returns to upright — minimized over tilt
azimuth in the plane perpendicular to `gB`.

- 4 or 8 azimuths, 12-step bisection. Best/worst azimuths differ by ~6%,
  so a single-axis number overstates capability.
- Success = final tilt < 1° over the last second AND peak tilt < 60°.
- **Caveat**: a 2.5s scoring window scores a 6.7s divergence as a
  success. Any run that might be marginally unstable needs >15s or a
  direct eigenvalue check (this is how the antipodal corner initially
  looked fine — see Part 1, Multi-corner section).
- "Released from rest" is deliberately harsh — a cube nudged while
  already balancing has wheel momentum available and does better.

### What's modelled, and how

| effect | model | parameter status |
|---|---|---|
| wheel bearing friction | `tau_cw*tanh(rho/eps) + b_w*rho`, reaction on the body | PLACEHOLDER 8 mN·m |
| friction feedforward | same expression added to `u` | — |
| pivot friction | `tau_cp*tanh(om/0.01)` opposing body rate | assumed 5 mN·m |
| torque saturation | per-axis clip at `tau_max` | `tau_cont` ESTIMATE |
| momentum cap | zero torque that would push `|rho|` past `omega_cap` | firmware policy |
| motor envelope | `tau <= Kt*(V_bus - Kt*|rho|)/R_phase` | `R_phase` ESTIMATE 0.10 Ω |
| loop delay | integer-sample buffer on the applied torque | 3.5 ms budget |
| gyro | white noise + constant bias | BMI270 datasheet order |
| accelerometer | specific force at `r_imu`, then normalized, then noise | see below |
| estimator | raw / reduced-attitude complementary filter with bias states | — |

**The accelerometer model is the part that matters.** An accelerometer at
position `r` from the contact corner does not measure gravity — it
measures specific force:

```
f = omd x r + om x (om x r) - g*gam
```

The tangential term is proportional to `omd`, which is proportional to
the control action — it's **in band and positively correlated with the
loop**. No low-pass filter removes it. This turned out to be the single
largest sensing effect in the whole budget (§9 below).

### Deliberate limitations

- Rigid bodies only — frame flexibility is invisible to this method.
- Point contact — no corner radius, no rolling, no slip check.
- Wheel unbalance computed analytically, not simulated (a rigid model
  can't show the resonance that's the actual risk).
- `R_phase = 0.10 Ω` is an estimate; only binds above ~1.0 N·m.
- No motor cogging, no current-loop dynamics (moteus FOC is ~kHz, far
  above the control loop).
- Single-corner geometry per run — corner-to-corner transitions not
  simulated.

---

## Part 2 — Results

### 1. Recovery envelope, shipping-adjacent configuration

`tau_max = 0.12 N·m`, `omega_cap = 40 rad/s`, ideal sensors.

**Worst case 3.34°.** Torque saturates 5% of the time, the momentum cap
11% — both bind at once, which is why relaxing either alone does little.

**Gain tuning is exhausted**: 48-point Bryson grid, `qa` ∈ {0.05, 0.1,
0.2, 0.4}, `qw` ∈ {10, 20, 40, 80}, `rt` ∈ {0.06, 0.12, 0.24}, `qr = 2.0`.
`Ms = 1.000000` on all 48.

| set | worst recovery | slowest pole |
|---|---|---|
| baseline 0.20 / 2.0 / 20 / 0.12 | 3.35° | -3.53 |
| best 0.20 / 2.0 / 10 / 0.24 | 3.42° | -7.56 |

+2% over an 8x span in every weight. Only `qw` moves anything, by
penalizing wheel rate harder (i.e. respecting the momentum cap). **The
envelope is set by hardware, not by the controller.**

**Actuator design chart** — worst-case recovery [deg], gains fixed at
0.20 / 2.0 / 10 / 0.24:

| tau_max \ omega_cap | 40 | 60 | 80 | 120 | 200 |
|---|---|---|---|---|---|
| **0.10** | 3.09 | 3.53 | 3.45 | 3.45 | 3.45 |
| **0.15** | 3.80 | 4.61 | 5.12 | 5.16 | 5.16 |
| **0.20** | 4.25 | 5.38 | 6.15 | 7.04 | 6.86 |
| **0.30** | 4.79 | 6.36 | 7.53 | 9.21 | 10.83 |
| **0.40** | 5.03 | 6.92 | 8.41 | 10.76 | 13.39 |

- Below ~0.15 N·m the cube is torque-bound and the wheel cap is
  irrelevant.
- Neither limit is worth relaxing alone; together they multiply.
- Non-monotonic cells (0.20/200 worse than 0.20/120): with fixed gains a
  larger cap lets the wheel run away and it can't unwind. **Raising
  `omega_cap` requires re-running the gain grid, not editing a
  constant.**

### 2. Full-capability test — cap removed, torque swept to the motor limit

`omega_cap = inf`, motor voltage envelope on (22.2V, R=0.10Ω), full
nonlinearities (friction + feedforward, 5ms delay, fused-grade sensor
noise), gains retuned for the uncapped case.

Retuning matters more without the cap: at `tau_max = 0.40`, sweeping
`qa × qr × qw` (48 sets), the wheel-rate weight wants `qw = 80` (not 10):
recovery 12.96° → 14.59°, **+12.6%**. Above `qw = 200` it gets worse
again; `qa`/`qr` remain almost irrelevant.

| tau [N·m] | I [A] | recovery | wheel peak [rad/s] | f [Hz] | h [N·m·s] | ecc for 10% unbalance | fastener load [N] |
|---|---|---|---|---|---|---|---|
| 0.12 | 4.8 | 4.17° | 116 | 18.4 | 0.056 | 0.025 mm | 3 |
| 0.20 | 8.0 | 7.13° | 218 | 34.7 | 0.106 | 0.012 mm | 9 |
| 0.30 | 11.9 | 10.85° | 321 | 51.1 | 0.157 | 0.008 mm | 20 |
| 0.40 | 15.9 | 14.58° | 415 | 66.1 | 0.202 | 0.006 mm | 33 |
| 0.60 | 23.9 | 22.02° | 588 | 93.6 | 0.286 | 0.005 mm | 67 |
| 0.80 | 31.8 | 29.40° | 714 | 113.6 | 0.348 | 0.004 mm | 99 |
| 1.00 | 39.8 | 36.27° | 766 | 122.0 | 0.373 | 0.005 mm | 114 |
| 1.20 | 47.8 | 42.29° | 786 | 125.1 | 0.383 | 0.005 mm | 120 |

Recovery is close to linear in torque up to ~0.8 N·m, then rolls off as
the back-EMF envelope pins the wheel near 780 rad/s against the 883 rad/s
no-load speed. Beyond ~1.0 N·m you're buying current, not capability.

**Three things that make the upper half of this table fictional:**

1. **Unbalance.** Forced torque scales as `omega^2`; the `ecc` column is
   the residual eccentricity needed to keep it under 10% of the torque
   budget. Above 0.3 N·m that's **4–8 microns** on a printed wheel
   holding 32 loose fasteners — not achievable without dynamic
   balancing.
2. **Frequency sweep.** The wheel passes 18→125 Hz on the way up. Every
   structural mode of a printed PETG-CF frame lives in that band. The
   rigid model can't see this and it's the most likely real failure
   mode.
3. **Gyroscopic coupling the design plant doesn't know about.** `Is*rho`
   reaches 0.38 N·m·s; `om × L` at a modest 1.5 rad/s body rate:

   | rho [rad/s] | h [N·m·s] | coupling [N·m] |
   |---|---|---|
   | 77 | 0.037 | 0.056 |
   | 272 | 0.132 | 0.199 |
   | 558 | 0.272 | 0.408 |
   | 846 | 0.412 | 0.618 |

   The LQR is linearized at `rho = 0`, where this term vanishes. At
   550+ rad/s the coupling exceeds the entire actuator budget. The
   nonlinear sims still recover, but on margin the design plant doesn't
   account for — uncharacterized.

**Honest read**: the motors can support ~30° of recovery on paper. The
defensible operating point is `tau ~ 0.3 N·m` (12A) with `omega_cap ~
300 rad/s`, giving ~10°, and even that requires balanced wheels and a
structural tap test first. The 40 rad/s policy costs about 20% of what's
safely available; going to full capability is not a firmware constant
change.

### 3. Multi-corner balance (intermediate sweep — see Part 2 §5 for final numbers)

| corner | ell mm | Sg | lambda | theta_eq | rec, own gains | primary gains | max Re |
|---|---|---|---|---|---|---|---|
| (-1,-1,-1) | 122.84 | 1.8875 | 8.2572 | 0.797 | 3.88 | false pass | +0.149 |
| (-1,-1,+1) | 128.54 | 1.9750 | 8.1212 | 3.170 | 3.67 | FALLS | +7.908 |
| (-1,+1,-1) | 126.08 | 1.9373 | 8.1591 | 2.773 | 3.74 | FALLS | +7.938 |
| (-1,+1,+1) | 131.64 | 2.0226 | 8.0480 | 3.097 | 3.61 | FALLS | +7.796 |
| (+1,-1,-1) | 128.56 | 1.9753 | 8.1180 | 3.171 | 3.66 | FALLS | +7.909 |
| (+1,-1,+1) | 134.01 | 2.0591 | 7.9804 | 2.609 | 3.43 | FALLS | +7.735 |
| (+1,+1,-1) | 131.66 | 2.0229 | 8.0501 | 3.095 | 3.51 | FALLS | +7.795 |
| (+1,+1,+1) | 136.99 | 2.1048 | 7.9341 | 0.714 | 3.42 | 3.42 | 0.000 |

All eight balance individually, 3.42–3.88°. The primary corner
(longest lever arm) is the worst of the eight in this pass — **superseded
by the corrected per-corner numbers in Part 2 §5**, where retuning moved
the worst corner.

**One gain set does not work.** Six corners diverge at essentially the
open-loop rate (+7.9): the controller isn't slow, it's pushing the wrong
way. `gB` rotates 67–107° between corners and both `P` and `Theta` follow
it.

**The antipodal trap.** `P = I - gB·gB'` is invariant under `gB → -gB`,
so the primary gains almost work at the antipodal corner: `max Re =
+0.1486`, a 6.7s divergence that a short simulation scores as a recovery
— it's a slow fall, not a real one. This is the direct justification for
the ">15s or eigenvalue check" rule in the metric definition above.

Implementation: 8 gain matrices = **216 floats** (this is exactly
`cubli_gains.h`'s `CORNER[8]` table already in this repo). Corner
identification from the accelerometer has **67.2° minimum separation**
between corner `gB` vectors (`ID_SEP_MIN` in `cubli_gains.h`), so 33° of
tilt error still identifies correctly — no IMU reconfiguration needed.

`theta_eq` is **0.71–3.17°**, not the 8.3° in older notes (that figure
was the 180mm build and is stale) — locomotion is much less compromised
by COM offset than previously recorded.

### 4. Nonlinearity budget

From 1° tilt, 8s, steady state over the last 2s, ideal sensor placement.

| effect | recovery | tilt rms | wheel drift | tau rms |
|---|---|---|---|---|
| ideal | 3.42 | 0.000 | 0.0 | 0.0000 |
| wheel Coulomb 8 mN·m, no FF | 3.29 | 0.001 | 1.8 rad/s | 0.0080 |
| same, WITH feedforward | 3.29 | 0.000 | 0.0 | 0.0000 |
| pivot friction 5 mN·m | 3.36 | 0.002 | 0.0 | 0.0001 |
| accel noise 3.2 mrad | 3.42 | 0.040 | 0.7 | 0.0308 |
| gyro noise 2.4 mrad/s | 3.42 | 0.004 | 0.1 | 0.0030 |
| delay 2 samples (5 ms) | 3.33 | — | — | — |
| delay 3 samples (7.5 ms) | 3.29 | — | — | — |
| **all, FF on, 5 ms delay** | **3.16** | 0.038 | 0.5 | 0.0308 |

Total realistic degradation **-7.6%**.

**Feedforward is confirmed mandatory and priced**: without it the wheel
drifts to 1.8 rad/s and the loop permanently burns **6.7% of the
continuous torque** cancelling drag. With it, both go to exactly zero.

Parameter robustness (nonlinear, gains fixed): `Theta` ×0.70 to ×1.30
gives 3.41→3.00°, never destabilizes. The `rho_scale` assumption doesn't
affect the envelope — the plumb-line and swing tests are worth doing but
not blocking.

### 5. Sensing and estimation

**The accelerometer lever arm is the dominant sensing effect.**
Apparent-tilt error during a successful 2° recovery, no noise:

| IMU distance from contact corner | peak error | rms |
|---|---|---|
| 34 mm | 0.439° | 0.052 |
| 68 mm | 0.879° | 0.109 |
| 137 mm | 1.758° | 0.247 |
| 206 mm | 2.638° | 0.451 |

At 137mm the apparent-tilt error peaks at **88% of the signal**, in phase
with the control action.

**Consequences, measured** (IMU at 137mm, kP=5):

| estimator | recovery | note |
|---|---|---|
| raw accelerometer | **0.00°** | falls every time, at any lever arm |
| Mahony CF + bias, no compensation | **2.75°** | **use this** |
| Mahony compensate with known torque only | 1.86° | partial compensation is WORSE |
| Mahony compensate with full model prediction | **0.00°** | self-referential, loop gain 6.0 |
| Mahony ORACLE, true `omd` | 2.85–3.12° | not implementable; the bound |
| ideal sensor, no lever arm | 3.42° | — |

> **Correction on record**: an earlier pass reported the full-model-prediction
> compensation at 2.51°. That came from code that fed the filter the TRUE
> `omd`/`om` from the integrator — an oracle. Re-implemented with signals
> the firmware can actually have, it diverges at every kP.

Feeding the raw accelerometer to the gain doesn't work at all. The
complementary filter rejects the lever arm almost entirely (2.85° at 0mm
→ 2.75° at 137mm) **but there's a cliff between 137mm and 206mm**.

**Design rule: keep the IMU inside ~150mm of every corner you intend to
balance on.** For multi-corner that means near the geometric centre
(130mm from all eight) — mounting near the primary corner buys accuracy
there but puts the antipode at 260mm, past the cliff. (This repo's actual
build mounts the IMU at the geometric centre — see §Final configuration
below.)

**Filter gain sweep** (recovery °, full nonlinearities, IMU at 137mm):

| kP \ kI | 0.1 | 0.5 | 2.0 | est err ss | tau rms |
|---|---|---|---|---|---|
| 1 | 2.22 | 2.28 | 2.41 | 0.196° | 0.0079 |
| 2 | 2.49 | 2.51 | 2.58 | 0.038° | 0.0085 |
| **5** | **2.75** | **2.75** | **2.75** | 0.052° | 0.0100 |
| 10 | 0.00 | 0.00 | 0.00 | 25.8° | 0.0756 |
| 20+ | 0.00 | 0.00 | 0.00 | 36–43° | 0.10 |

`kP = 5` is a clear optimum with a hard edge above it: at `kP >= 10`
enough lever-arm error reaches the estimate to close a positive feedback
loop. `kI` barely matters; below `kP = 2` the loss is filter lag. (**Note:
superseded by §Final configuration below, where the measured cliff moved
to between 6 and 7, and the shipped value is `kP=4`, not 5** — this
sweep predates that correction.)

**Where the remaining 20% is**: CF at `kP=5` gives 2.75° against 3.42°
for a perfect sensor. Running the CF with zero noise/bias still gives
2.76°, so the gap is *not* noise and *not* lever arm (already rejected)
— it's **phase lag**: the CF is model-free and can only integrate the
gyro.

**Why the lever arm is not algebraically compensated** (§9.1 of the
source): reconstructing gravity as `g*gacc + omd×r + om×(om×r)` fails
because `omd` isn't measured, and predicting it from the model needs the
gravity torque, which needs the attitude estimate — self-referential,
with gain `Sg·‖Tb⁻¹‖·|r_imu|/g = 6.0`. Unconditionally divergent.
Compensating with only the known commanded torque (dropping the gravity
term) is stable but *worse* than no compensation (1.86° vs 2.75°) — the
uncompensated dominant term was proportional to control torque and acted
against tilt; removing it leaves only the gravity-proportional term,
positive feedback at the same gain of 6. (This explanation is a
hypothesis from the source note, not independently confirmed there.)

**Recommended estimator path, in order** (from the source; step 1 is
what's actually implemented in this repo's firmware today):

1. **Reduced-attitude complementary filter with gyro-bias states** — `e =
   ghat × g_acc`, integrate `bhat += kI*e*dt` (sign verified: the
   opposite sign is stable-looking but drives the estimate to 5x the
   true bias). ~30 flops, no quaternion, no matrix, singularity-free.
   **This is what's shipped.**
2. **Steady-state linear Kalman filter (LQG)** — not yet implemented.
   State `[phi(3); om(3); b_gyro(3)]`, driven by the known commanded
   torque through `B`. Reconstructs the lever-arm term analytically
   instead of rejecting it, with less lag than pure gyro integration.
   Estimated ~0.67° of the remaining gap lives here.
3. **Disturbance-torque state augmentation** — not yet implemented. Gives
   integral action for free (three extra KF states), removing the
   permanent tilt offset any constant unmodelled torque (COM estimate
   error, cable pull, residual friction) currently costs.
4. **Quaternion MEKF — deliberately not yet.** Only justified for large
   attitude excursions or full 3-DoF need: stage-3 jump-up, and
   corner-to-corner transitions where `gB` moves 67–107°. Not for corner
   balance itself (tilt stays under 15°), where the linear KF is
   provably adequate. **Trap, measured not theoretical**: with one
   vector measurement, yaw is unobservable and so is the gyro-bias
   component along gravity — true bias projected onto `gB` was
   0.149°/s, the estimator correctly recovered exactly 0.000 (nothing
   observes it). A naive MEKF lets that direction's covariance grow
   unboundedly; must be constrained, or avoided by staying in reduced-
   attitude form. A magnetometer would make yaw observable but yaw is
   also uncontrollable (`rank(ctrb) = 8 of 9`) — buys nothing for
   balance.

---

## Part 2 §5 — Final configuration and per-corner results (corrected, authoritative)

**All numbers below are the SHIPPING configuration**, not the
ideal-sensor one: `tau_max = 0.12 N·m`, `omega_cap = 40 rad/s`, bearing
friction + feedforward, 5ms loop delay, BMI270-grade noise (accel 3.2
mrad, gyro 2.4 mrad/s), gyro bias `[0.3, -0.2, 0.15] deg/s`, **IMU at the
geometric centre (129.9mm from every corner)**, reduced-attitude
complementary filter with bias states.

### Per-corner results, own gains (supersedes Part 2 §3's table)

| corner | ell mm | Sg | lambda | theta_eq | recovery | wheel peak [rad/s] |
|---|---|---|---|---|---|---|
| (-1,-1,-1) | 122.84 | 1.8875 | 8.2572 | 0.797 | **3.14°** | 41.7 |
| (-1,+1,-1) | 126.08 | 1.9373 | 8.1591 | 2.773 | 3.03 | 41.7 |
| (-1,-1,+1) | 128.54 | 1.9750 | 8.1212 | 3.170 | 2.93 | 41.3 |
| (+1,-1,-1) | 128.56 | 1.9753 | 8.1180 | 3.171 | 2.92 | 41.7 |
| (-1,+1,+1) | 131.64 | 2.0226 | 8.0480 | 3.097 | 2.89 | 41.3 |
| (+1,+1,-1) | 131.66 | 2.0229 | 8.0501 | 3.095 | 2.83 | 41.7 |
| (+1,-1,+1) | 134.01 | 2.0591 | 7.9804 | 2.609 | 2.77 | 41.7 |
| (+1,+1,+1) | 136.99 | 2.1048 | 7.9341 | 0.714 | **2.76°** | 41.7 |

**All eight balance.** Spread 14%, recovery tracks `1/Sg` exactly — the
longest lever arm is the hardest corner. **The primary `(+1,+1,+1)` is
the WORST of the eight** in this corrected pass (note: this reverses
which corner is "worst" relative to the earlier intermediate sweep in
Part 2 §3) — if it works there, it works everywhere.

Wheel peak is **41.7 rad/s against the 40 rad/s cap** at every corner —
small overshoot because the cap zeroes torque rather than braking.
**Every corner is momentum-bound, not torque-bound.**

Each corner needs its own gains: with the primary corner's gains applied
everywhere, six corners diverge at the open-loop rate (max Re +7.7 to
+7.9) and the antipodal corner sits at +0.149 — a 6.7s fall that short
runs score as a pass (same trap as Part 2 §3).

### Final LQR gains

```
qa = 0.20 rad     qr = 2.0 rad/s     qw = 10 rad/s     rt = 0.24 N*m
Q  = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)])
R  = (12/rt^2)*eye(3)
```

Chosen on **robustness**, not peak — the grid optimum is `qw=5` at 2.91°,
but it fails on plant error:

| qw | Θ ×0.8 | ×0.9 | ×1.0 | ×1.1 | ×1.2 | verdict |
|---|---|---|---|---|---|---|
| 5 | 3.11 | 3.05 | 2.99 | **0.00** | **0.00** | fails at +10% on Θ |
| 6 | 3.07 | 3.00 | 2.94 | 2.57 | **0.00** | fails at +20% |
| 8 | 2.98 | 2.92 | 2.86 | 2.80 | **0.00** | fails at +20% |
| **10** | 2.91 | 2.85 | **2.79** | 2.73 | 2.68 | **survives the whole band** |
| 12 | 2.84 | 2.78 | 2.72 | 2.67 | 2.62 | survives, 3% worse |

`qw = 10` costs 4% against the fragile optimum and is the only set that
holds across ±20% on Θ — Θ is still an *assumed* split (`rho_scale`), so
this margin is not optional. Below `qw = 5` there's a cliff: `qw=4 →
2.06°`, `qw=3 → 1.58°`, `qw=2 → 0.00°`, as `max|Kp|` climbs 13→29 and the
controller starts fighting the estimator.

### Final filter gains — correction on record

```
kP = 4      kI = 0.5      reduced-attitude complementary filter with bias
```

An earlier note said the `kP` cliff was at 10 and to set 5. **The cliff
is actually between 6 and 7**, measured at `qw=10` with the IMU at the
geometric centre:

| kP | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|
| recovery | 2.67 | 2.74 | 2.79 | 2.82 | **0.00** | **0.00** |

`kP=5` is only 1.4x below a hard cliff. **`kP=4` costs 2% and gives
1.75x margin** — take the margin for hardware bring-up, since the plant
on the bench is not the plant in the model. Do not tune `kP` upward until
it looks crisp. **This is exactly the `kP_FILT = 4.0` / `kI_FILT = 0.5`
already shipped in this repo's firmware** (`corner-bringup`'s
`attitudeUpdate()`, `cubli_gains.h`'s header comment) — the "HARD CLIFF
between 6 and 7 — do not raise" comment there is this result.

### Expected result

**2.76° worst case, 3.14° best, on the shipping configuration.** Roughly
hand-placement accuracy. Every corner is momentum-bound, so the single
highest-value change remains **raising `omega_cap`** — after balancing
the wheels, and re-running the gain grid, in that order (same sequencing
already captured in the Fine-Tuning Test Plan's Test 6).

---

## Part 3 — Recommended actions, in order (from the source, as originally sequenced)

1. **Spin-down and breakaway tests** → real `tau_cw`, `b_w`, cogging.
   Feedforward is not optional and is worth 6.7% of the torque budget.
2. **Complementary filter with bias states, `kP=5`** (later corrected to
   `kP=4`, see above), before the gain block. Raw accelerometer doesn't
   work at any realistic IMU position.
3. **Confirm IMU placement** within ~150mm of every intended balance
   corner — geometric centre satisfies all eight at 130mm. *(Done —
   shipped configuration.)*
4. **Thermal test for the real `tau_cont`.** Single biggest lever on the
   envelope: 0.12→0.20 at `omega_cap=120` takes 3.3°→7.0°.
5. **Structural tap test.** Strike the frame, log the gyro at maximum
   rate, FFT. A first mode under ~50Hz is inside the wheel speed range.
   *(This is the same tap-test recommended earlier in this project's own
   conversation for the frame-bending-under-load finding — same
   diagnostic, independently arrived at twice.)*
6. **Balance the wheels, THEN raise `omega_cap`, THEN re-run the gain
   grid** — in that order; the grid result changes with the cap. *(This
   is Fine-Tuning Test Plan Tests 4→5→6, in the same order.)*
7. **Per-corner gain tables** (216 floats) plus accelerometer corner ID.
   *(Done — `cubli_gains.h`'s `CORNER[8]`.)*
8. **Multi-corner tests must run >15s** or check eigenvalues directly —
   the antipodal-trap false-pass risk (Part 2 §3).
9. **Then, and only then**, the steady-state KF with disturbance-torque
   states (estimator roadmap items 2–3 above).

### Source files referenced (not present in this repo — outputs only)

| file | role |
|---|---|
| `cubli_cube_params.m` | mass properties, geometry, eight corner equilibria |
| `cubli_corner_plant.m` | per-corner Theta, Tb, gB, P, A, B, lambda |
| `cubli_gains.m` | reduced-order LQR + yaw projection, returns Ms |
| `cubli_nlsim.m` | nonlinear plant, sensors, actuator limits, estimator |
| `cubli_maxrec.m` | bisection recovery sweep over tilt azimuth |
| `cubli_corner_run.m` | Simscape corner balance, Gate 3 + closed loop |

---

## Hardware-stage update — corner-to-housing strut (2026-08-19)

**Different source from Part 1–3 above** — not part of the original
recovered pre-hardware design study. This is a re-derivation triggered by
a real hardware change made during corner bring-up (after
`hw-run-analysis.md`'s 373.5s run): a new rigid strut connecting corner
`[-1,-1,-1]` to the electronics housing, added for structural rigidity,
plus the resulting mass increase. Only this one corner has been
re-derived so far — see "What's not re-derived yet" below.

### Why the strut barely hurts — it's geometric, not incidental

The pole runs 4.49° off the balancing diagonal — essentially straight
down the load path (a compression member, not a bending one). That
placement does two things that partly cancel: it adds mass, **and** it
pulls the COM toward the balancing corner (shortening `ell`, which by
itself would *help* Sg). A 4.2% mass increase costing only 2.7% on Sg is
the result — bolted anywhere else on the cube, the same added mass would
have cost 4–5%.

| Corner `[-1,-1,-1]` | Before | After | Change |
|---|---|---|---|
| `m_total` | 1.5668 kg | 1.6330 kg | +66.2 g (+4.2%) |
| `ell` | 122.84 mm | 121.08 mm | −1.76 mm (−1.4%) |
| `Sg` | 1.8875 N·m | 1.9390 N·m | +2.7% |
| `λ` | 8.2572 s⁻¹ | 8.3688 s⁻¹ | +1.4% |
| `θ_eq` | 0.797° | 0.894° | +0.097° |

### New gains, corner `[-1,-1,-1]`

```
Kp =
  -7.0685   3.4801   3.4675  -0.8446   0.4235   0.4244  -0.000998   0.006001   0.005931
   3.5743  -6.9184   3.5758   0.4173  -0.8435   0.4177   0.004252  -0.002662   0.004252
   3.4703   3.4844  -7.0670   0.4242   0.4237  -0.8452   0.005879   0.005948  -0.001049
```

Robustness diagnostics on this gain set: `Ms = 0.999999`, slowest mode
`−7.93 s⁻¹` (126.1 ms), `|K1|` spread 1.022, discrete `max|z| = 0.9813`,
robust ±30% worst-case `−1.947`. Still momentum-limited: bound 4.81°
against a torque bound of 5.02°.

Worst-case recovery on this corner: **2.89°, down from 3.14°.**

### Multi-corner picture — the finding that matters

The recovery-envelope drop above is the smaller story. The COM shift
moved `θ_eq` at all eight corners (the strut's mass and position affect
the whole cube, not just its own corner), and the change was uneven:

| corner | θ_eq | recovery | margin |
|---|---|---|---|
| `(-1,-1,-1)` | 0.894° | 2.89° | **+2.00°** |
| `(-1,+1,-1)` | 3.521° | 2.83° | −0.69° |
| `(-1,-1,+1)` | 3.935° | 2.75° | −1.19° |
| `(+1,-1,-1)` | 3.940° | 2.73° | −1.21° |
| `(+1,+1,-1)` | 3.811° | 2.48° | −1.33° |
| `(+1,-1,+1)` | 3.274° | 2.41° | −0.86° |
| `(-1,+1,+1)` | 3.820° | 2.40° | −1.42° |
| `(+1,+1,+1)` | 0.780° | 2.52° | **+1.74°** |

Before the strut, `θ_eq` ran 2.6–3.2° against a 2.8–3.0° recovery
envelope — marginal but positive everywhere. The perpendicular COM
offset moved 1.708 → 1.890 mm, and that pushed six of the eight corners'
equilibrium tilt **past their own recovery envelope** (`θ_eq` now
3.3–3.9° against 2.4–2.8° recovery — negative margin). Only the two ends
of the primary body diagonal, `[-1,-1,-1]` and `[+1,+1,+1]`, are
unaffected and still balance comfortably.

**Corner balancing — the current pass criterion — is unaffected**: the
cube only needs to balance on whichever single corner it's resting on,
and that's `[-1,-1,-1]` for this rig. **Multi-corner locomotion is a
different claim**, and this table says it's currently off the table on
six of eight corners unless the pole is rebalanced or a counterweight
goes on the opposite side. Worth deciding before it becomes a mission-
capability assumption baked into later planning.

### What's not re-derived yet

Only corner `[-1,-1,-1]`'s `Kp`/`ell`/`Sg`/`λ`/`θ_eq` have been
re-derived and (partially — `Kp`/`ell`/`θ_eq` only, not `gB`) carried
into firmware (`corner-bringup/Stage4_AutoTrim*`, `Stage5_AutoTrim*` —
see the `TODO: MIXED-GENERATION PLANT` comments above each file's
`kCorners` table). `gB` (the 3-vector corner-resolution direction),
plus the full `gB`/`Kp`/`ell` for the other seven corners, still need
deriving before this table's margin figures can be treated as more than
a planning input — the recovery/margin numbers above come from the
source analysis, not a re-run against firmware-exact gains for corners
2–8.

---

## Edge balance — no simulation study found

Searched this repo (`matlab/`, `docs/simulation/`) and the full project
design conversation history for an edge-balance equivalent of the above —
none exists. Edge balance in this project went straight from the
reduced-attitude estimator design to staged hardware bring-up
(`edge-bringup/Stage1-5`), validated directly on hardware rather than
through a prior MATLAB/Simscape envelope study.

If the results section needs an edge-balance performance number, the
honest source is the **hardware bring-up telemetry** (`edge-bringup`'s
own hardware-confirmed results — `kWheelSign` confirmed on all three
axes, place-offsets from `cubli_gains.h`'s `EDGE[12]` table — 0.006° to
3.648° depending on which of the 12 edges) rather than a simulated
recovery-angle figure. Worth stating that distinction explicitly in the
write-up rather than presenting an edge number alongside the corner
numbers above as if both came from the same kind of study.

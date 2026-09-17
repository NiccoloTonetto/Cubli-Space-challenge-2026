# Cubli Cube - Performance Envelope: Methodology and Results

Supersedes *Cube performance envelope and limits*. Everything below is
reproducible from the files listed at the end.

---

# Part I - Methodology

## 1. Why three models, not one

| model | purpose | cost |
|---|---|---|
| `cubli_cube_params` + `cubli_corner_plant` | 9-state linear design plant, one per corner | instant |
| `simscape3d.slx` | independent multibody truth, imported STEP geometry | ~20 s per run |
| `cubli_nlsim` | full nonlinear plant + sensors + actuator limits | ~50 ms per run |

The Simscape model is the reference but is far too slow for the thousands of
runs a bisection sweep needs. The linear model cannot see saturation,
gyroscopic coupling or sensor geometry at all. `cubli_nlsim` is the working
tool; its ONLY job is to be fast and to agree with Simscape.

### Nonlinear plant

State `x = [gam(3); om(3); rho(3)]` where `gam` is the gravity direction
expressed in BODY coordinates - i.e. exactly what an ideal accelerometer
reports - `om` the body rate about the contact point, `rho` the wheel rates
RELATIVE to the body (what the encoder reads).

```
L      = Theta*om + Is*rho
Tb*omd = m*g*(r_c x gam) - u_eff - om x L - tau_pivot
Is*rhod= u_eff - Is*omd
gamd   = -om x gam
```

`Theta` is body-constant because the wheels are axisymmetric about their spin
axes. Linearising `m*g*(r_c x gam)` about `gam = gB` with `r_c = -ell*gB`
recovers `Sg*P*phi` exactly, which is the consistency check between this model
and the design plant.

Using `gam` rather than Euler angles is deliberate: no parameterisation, no
singularity, and it is the same quantity the firmware will actually have. The
controller input is the reduced attitude `phi = -gB x gam`, which satisfies
`P*phi_true = phi` identically.

### Validation chain

1. `cubli_corner_plant` reproduces `cubli_cube_params` for the primary corner
   to all printed digits (ell, Sg, lambda).
2. Simscape Gate 3 at the corner: `A err 1.740e-07`, `B err 2.128e-16`.
3. `cubli_nlsim` vs Simscape, 2 deg recovery, ideal sensors:

| t [s] | nonlinear tilt | Simscape tilt |
|---|---|---|
| 0.0 | 1.6254 | 1.6254 |
| 0.1 | 1.0766 | 1.0854 |
| 0.3 | 0.1135 | 0.1108 |
| 0.6 | 0.3369 | 0.3389 |
| 1.0 | 0.1224 | 0.1285 |

Agreement better than 1 % through the transient. Late-time divergence is
Simscape solver tolerance, not a modelling difference.

4. Every gain set reports `Ms`. The reduced-order LQR return-difference
   identity gives `Ms = 1.000000` exactly; any deviation is an implementation
   bug, so it is asserted rather than inspected.

## 2. The metric: worst-case recovery angle

**Definition.** The largest initial tilt, released from rest with the wheels
stationary, from which the cube returns to upright, MINIMISED over the tilt
azimuth in the plane perpendicular to `gB`.

- 4 or 8 azimuths, 12-step bisection. Best and worst azimuths differ by ~6 %,
  so quoting a single-axis number overstates capability.
- Success = final tilt < 1 deg over the last second AND peak tilt < 60 deg.
- **Caveat that bit us:** a 2.5 s window scores a 6.7 s divergence as a
  success. Any run that might be marginally unstable needs >15 s or a direct
  eigenvalue check. This is how the antipodal corner initially looked fine.

Released from rest is a deliberately harsh definition. A cube nudged while
already balancing has wheel momentum available and does better.

## 3. What is modelled, and how

| effect | model | parameter status |
|---|---|---|
| wheel bearing friction | `tau_cw*tanh(rho/eps) + b_w*rho`, reaction on the body | PLACEHOLDER 8 mNm |
| friction feedforward | same expression added to `u` | - |
| pivot friction | `tau_cp*tanh(om/0.01)` opposing body rate | assumed 5 mNm |
| torque saturation | per-axis clip at `tau_max` | tau_cont ESTIMATE |
| momentum cap | zero torque that would push `abs(rho)` past `omega_cap` | firmware policy |
| motor envelope | `tau <= Kt*(V_bus - Kt*abs(rho))/R_phase` | R_phase ESTIMATE 0.10 ohm |
| loop delay | integer-sample buffer on the applied torque | 3.5 ms budget |
| gyro | white noise + constant bias | BMI270 datasheet order |
| accelerometer | **specific force at r_imu**, then normalised, then noise | see below |
| estimator | raw / reduced-attitude complementary filter with bias states | - |

### The accelerometer model is the part that matters

An accelerometer at position `r` from the contact corner does not measure
gravity. It measures specific force:

```
f = omd x r + om x (om x r) - g*gam
```

The tangential term is proportional to `omd`, which is proportional to the
control action. It is therefore IN BAND and POSITIVELY CORRELATED with the
loop. No low-pass filter removes it. Most Cubli write-ups skip this; it turns
out to be the single largest sensing effect in the whole budget.

## 4. Deliberate limitations

- Rigid bodies only. Frame flexibility is invisible to this method.
- Point contact. No corner radius, no rolling, no slip check.
- Wheel unbalance is computed analytically, not simulated - a rigid model
  cannot show the resonance that is the actual risk.
- `R_phase = 0.10 ohm` is an estimate. It only binds above ~1.0 N m.
- No motor cogging, no current-loop dynamics (moteus FOC is ~kHz, far above).
- Single-corner geometry per run; corner-to-corner transitions not simulated.

---

# Part II - Results

## 5. Recovery envelope, current configuration

`tau_max = 0.12 N m`, `omega_cap = 40 rad/s`, ideal sensors.

**Worst case 3.34 deg.** Torque saturates 5 % of the time, the momentum cap
11 %. Both bind at once, which is why relaxing either alone does little.

### Gain tuning is exhausted

48-point Bryson grid, `qa` in {0.05 0.1 0.2 0.4}, `qw` in {10 20 40 80},
`rt` in {0.06 0.12 0.24}, `qr = 2.0`. `Ms = 1.000000` on all 48.

| set | worst recovery | slowest pole |
|---|---|---|
| baseline 0.20 / 2.0 / 20 / 0.12 | 3.35 deg | -3.53 |
| best 0.20 / 2.0 / 10 / 0.24 | 3.42 deg | -7.56 |

**+2 % over an 8x span in every weight.** Only `qw` moves anything, and it
helps by penalising wheel rate harder - i.e. by respecting the momentum cap.
The envelope is set by hardware, not by the controller.

### Actuator design chart

Worst-case recovery [deg], gains fixed at 0.20 / 2.0 / 10 / 0.24:

| tau_max \\ omega_cap | 40 | 60 | 80 | 120 | 200 |
|---|---|---|---|---|---|
| **0.10** | 3.09 | 3.53 | 3.45 | 3.45 | 3.45 |
| **0.15** | 3.80 | 4.61 | 5.12 | 5.16 | 5.16 |
| **0.20** | 4.25 | 5.38 | 6.15 | 7.04 | 6.86 |
| **0.30** | 4.79 | 6.36 | 7.53 | 9.21 | 10.83 |
| **0.40** | 5.03 | 6.92 | 8.41 | 10.76 | 13.39 |

- Below ~0.15 N m the cube is torque-bound and the wheel cap is irrelevant.
- Neither limit is worth relaxing alone; together they multiply.
- Non-monotonic cells (0.20/200 worse than 0.20/120). With fixed gains a
  larger cap lets the wheel run away and it cannot unwind. **Raising
  `omega_cap` requires re-running the gain grid, not editing a constant.**

## 6. Full-capability test - cap removed, torque swept to the motor limit

`omega_cap = inf`, motor voltage envelope on (22.2 V, R = 0.10 ohm), full
nonlinearities (friction + feedforward, 5 ms delay, fused-grade sensor noise),
gains retuned for the uncapped case.

### Retuning matters more without the cap

At `tau_max = 0.40`, sweeping `qa` x `qr` x `qw` (48 sets): the wheel-rate
weight wants to be **`qw = 80`**, not 10. Recovery 12.96 -> 14.59 deg,
**+12.6 %**. Above `qw = 200` it gets worse again. `qa` and `qr` remain
almost irrelevant.

### The envelope

| tau [N m] | I [A] | recovery | wheel peak [rad/s] | f [Hz] | h [N m s] | ecc for 10 % unbalance | fastener load [N] |
|---|---|---|---|---|---|---|---|
| 0.12 | 4.8 | 4.17 deg | 116 | 18.4 | 0.056 | 0.025 mm | 3 |
| 0.20 | 8.0 | 7.13 deg | 218 | 34.7 | 0.106 | 0.012 mm | 9 |
| 0.30 | 11.9 | 10.85 deg | 321 | 51.1 | 0.157 | 0.008 mm | 20 |
| 0.40 | 15.9 | 14.58 deg | 415 | 66.1 | 0.202 | 0.006 mm | 33 |
| 0.60 | 23.9 | 22.02 deg | 588 | 93.6 | 0.286 | 0.005 mm | 67 |
| 0.80 | 31.8 | 29.40 deg | 714 | 113.6 | 0.348 | 0.004 mm | 99 |
| 1.00 | 39.8 | 36.27 deg | 766 | 122.0 | 0.373 | 0.005 mm | 114 |
| 1.20 | 47.8 | 42.29 deg | 786 | 125.1 | 0.383 | 0.005 mm | 120 |

Recovery is close to linear in torque up to ~0.8 N m, then rolls off as the
back-EMF envelope pins the wheel near 780 rad/s against the 883 rad/s no-load
speed. Beyond ~1.0 N m you are buying current, not capability.

### Three things that make the upper half of this table fictional

**1. Unbalance.** Forced torque is `m_wheel * ecc * omega^2 * 167 mm` and
scales as `omega^2`. The `ecc` column is the residual eccentricity needed to
keep it under 10 % of the torque budget. Above 0.3 N m that is **4 to 8
microns** on a printed wheel holding 32 loose fasteners. Not achievable
without dynamic balancing, and arguably not achievable at all.

**2. Frequency sweep.** The wheel passes 18 -> 125 Hz on the way up. Every
structural mode of a printed PETG-CF frame lives in that band. The rigid model
cannot see this and it is the most likely real failure mode.

**3. Gyroscopic coupling the design plant does not know about.** `Is*rho`
reaches 0.38 N m s, and `om x L` at a modest 1.5 rad/s body rate is:

| rho [rad/s] | h [N m s] | coupling [N m] |
|---|---|---|
| 77 | 0.037 | 0.056 |
| 272 | 0.132 | 0.199 |
| 558 | 0.272 | 0.408 |
| 846 | 0.412 | 0.618 |

The LQR is linearised at `rho = 0`, where this term vanishes. At 550+ rad/s
the coupling exceeds the entire actuator budget. The nonlinear sims still
recover, but on margin the design plant does not account for - and that margin
has not been characterised.

**Honest read:** the motors can support ~30 deg of recovery on paper. The
defensible operating point is `tau ~ 0.3 N m` (12 A) with `omega_cap ~ 300
rad/s`, giving ~10 deg, and even that requires balanced wheels and a structural
tap test first. The 40 rad/s policy is costing about 20 % of what is safely
available; going to full capability is not a firmware constant change.

## 7. Multi-corner balance

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

All eight balance individually, 3.42-3.88 deg. **The primary corner is the
worst of the eight** - longest lever arm.

**One gain set does not work.** Six corners diverge at essentially the
open-loop rate (+7.9): the controller is not slow, it is pushing the wrong way.
`gB` rotates 67-107 deg between corners and both `P` and `Theta` follow it.

**The antipodal trap.** `P = I - gB gB'` is invariant under `gB -> -gB`, so
the primary gains almost work at (-1,-1,-1): `max Re = +0.1486`, a 6.7 s
divergence that a short simulation scores as a recovery. It is a slow fall.

Implementation: 8 gain matrices = **216 floats**. Corner identification from
the accelerometer has **67.2 deg minimum separation** between corner `gB`
vectors, so 33 deg of tilt error still IDs correctly. No IMU reconfiguration.

`theta_eq` is **0.71 to 3.17 deg**, not the 8.3 deg in the older notes - that
figure was the 180 mm build and is stale. Locomotion is much less compromised
by the COM offset than previously recorded.

## 8. Nonlinearity budget

From 1 deg tilt, 8 s, steady state over the last 2 s, ideal sensor placement.

| effect | recovery | tilt rms | wheel drift | tau rms |
|---|---|---|---|---|
| ideal | 3.42 | 0.000 | 0.0 | 0.0000 |
| wheel Coulomb 8 mNm, no FF | 3.29 | 0.001 | 1.8 rad/s | 0.0080 |
| the same, WITH feedforward | 3.29 | 0.000 | 0.0 | 0.0000 |
| pivot friction 5 mNm | 3.36 | 0.002 | 0.0 | 0.0001 |
| accel noise 3.2 mrad | 3.42 | 0.040 | 0.7 | 0.0308 |
| gyro noise 2.4 mrad/s | 3.42 | 0.004 | 0.1 | 0.0030 |
| delay 2 samples (5 ms) | 3.33 | - | - | - |
| delay 3 samples (7.5 ms) | 3.29 | - | - | - |
| **all, FF on, 5 ms delay** | **3.16** | 0.038 | 0.5 | 0.0308 |

Total realistic degradation **-7.6 %**.

Feedforward is confirmed mandatory and now priced: without it the wheel drifts
to 1.8 rad/s and the loop permanently burns **6.7 % of the continuous torque**
cancelling drag. With it both go to exactly zero.

Parameter robustness, nonlinear, gains fixed: `Theta` x0.70 to x1.30 gives
3.41 -> 3.00 deg and never destabilises. **The `rho_scale` assumption does not
affect the envelope.** The plumb-line and swing tests are worth doing but are
not blocking.

## 9. Sensing and estimation

### The accelerometer lever arm is the dominant sensing effect

Apparent-tilt error during a SUCCESSFUL 2 deg recovery, no noise:

| IMU distance from contact corner | peak error | rms |
|---|---|---|
| 34 mm | 0.439 deg | 0.052 |
| 68 mm | 0.879 deg | 0.109 |
| 137 mm | 1.758 deg | 0.247 |
| 206 mm | 2.638 deg | 0.451 |

At 137 mm the apparent-tilt error peaks at **88 % of the signal**, and it is
in phase with the control action.

### Consequences, measured

| estimator (IMU at 137 mm, kP = 5) | recovery | note |
|---|---|---|
| `raw` accelerometer | **0.00 deg** | falls every time, at any lever arm |
| `mahony` CF + bias, no compensation | **2.75 deg** | **use this** |
| `mahony_u` compensate with known torque only | 1.86 deg | partial compensation is WORSE |
| `mahony+` compensate with full model prediction | **0.00 deg** | self-referential, loop gain 6.0 |
| `mahony_or` ORACLE, true omd | 2.85-3.12 deg | not implementable; the bound |
| ideal sensor, no lever arm at all | 3.42 deg | |

CORRECTION: an earlier version of this note reported `mahony+` at 2.51 deg.
That came from code that fed the filter the TRUE `omd` and `om` from the
integrator - an oracle. Re-implemented with signals the firmware can actually
have, it diverges at every kP. See 9.1.

`mahony` holds 2.85 deg at 0 mm and 2.75 deg at 137 mm, so the CF rejects the
lever arm almost entirely - but there is a cliff between 137 and 206 mm.

Feeding the raw accelerometer to the gain does not work at all. The
complementary filter rejects the lever arm almost completely (2.85 -> 2.75 deg
across 0 to 137 mm) **but there is a cliff between 137 and 206 mm.**

**Design rule: keep the IMU inside ~150 mm of every corner you intend to
balance on.** For multi-corner that means near the geometric centre, which is
130 mm from all eight. Mounting it near the primary corner buys accuracy there
and puts the antipode at 260 mm, past the cliff.

### Filter gain sweep (recovery deg, full nonlinearities, IMU at 137 mm)

| kP \\ kI | 0.1 | 0.5 | 2.0 | est err ss | tau rms |
|---|---|---|---|---|---|
| 1 | 2.22 | 2.28 | 2.41 | 0.196 deg | 0.0079 |
| 2 | 2.49 | 2.51 | 2.58 | 0.038 deg | 0.0085 |
| **5** | **2.75** | **2.75** | **2.75** | 0.052 deg | 0.0100 |
| 10 | 0.00 | 0.00 | 0.00 | 25.8 deg | 0.0756 |
| 20+ | 0.00 | 0.00 | 0.00 | 36-43 deg | 0.10 |

`kP = 5` is a clear optimum with a hard edge above it: at `kP >= 10` enough
lever-arm error reaches the estimate to close a positive feedback loop. `kI`
barely matters. Below `kP = 2` the loss is filter lag.

### Where the remaining 20 % is

With the CF at `kP = 5`: **2.75 deg** against **3.42 deg** with a perfect
sensor. Running the CF with zero noise and zero bias still gives 2.76 deg, so
the gap is **not** noise, and it is **not** lever arm (rejected). It is
**phase lag** - the CF is model-free and can only integrate the gyro.

### Next step: what to build, in order

### 9.1 Do NOT algebraically compensate the lever arm

Reconstructing gravity as `g*gacc + omd x r + om x (om x r)` fails, and the
failure is instructive.

`omd` is not measured. Predicting it from the model needs the gravity torque,
which needs the attitude estimate, which is what the compensation is supposed
to produce. The self-referential gain is

```
Sg * ||Tb^-1|| * |r_imu| / g  =  2.1048 * 204.36 * 0.01397  =  6.0
```

Six. Unconditionally divergent; no gain choice rescues it.

Dropping the gravity term and compensating with only the KNOWN commanded
torque and the MEASURED rate (`mahony_u`) is stable but WORSE than no
compensation: 1.86 vs 2.75 deg. Measured peaks over a 2 deg recovery are
`|Tb^-1*tau_g| = 2.28`, `|Tb^-1*u| = 7.95`, `|omd| = 7.33 rad/s^2`. Removing
the control-torque term leaves the gravity term, which is proportional to tilt
and so acts as positive feedback with that same gain of 6, whereas the
uncompensated dominant term was proportional to the control torque and acted
against the tilt. The RESULT is measured; this EXPLANATION is a hypothesis and
has not been independently confirmed.

The oracle reaches 3.12 deg at kP = 20 and keeps improving, so about 0.4 deg
is genuinely available from knowing `omd`. It is reachable only where `omd` is
part of a jointly consistent, covariance-weighted estimate - the Kalman filter
below - not an open-loop algebraic substitution.

**1. Reduced-attitude complementary filter with gyro-bias states. Do this now.**
Propagate `ghat` with the gyro, correct with `e = ghat x g_acc`, integrate
`bhat`. Roughly 30 flops, no quaternion, no matrix. Singularity-free because
it acts on the 2-DoF gravity direction, which is exactly what the controller
consumes. **Watch the sign** - the bias update is `bhat += kI*e*dt`; the
opposite sign is stable-looking but drives the estimate to 5x the true bias.

**2. Steady-state linear Kalman filter (LQG). This is where the 0.67 deg is.**
State `[phi(3); om(3); b_gyro(3)]`, driven by the KNOWN commanded torque
through `B`. Because the filter has the model it predicts `omd`, so it both
(a) reconstructs the lever-arm term analytically instead of rejecting it, and
(b) has far less lag than pure gyro integration. Compute the gain offline with
`kalman` and ship a constant matrix - runtime cost is the same as the CF.
Separation applies, so it composes with the existing `Kp` unchanged.

**3. Augment with a disturbance-torque state. High value, low cost.**
The LQR has no integral action, so any constant unmodelled torque - COM
estimate error, cable pull, residual friction - shows up as a permanent tilt
offset that eats recovery margin directly. Three extra states in the same KF
gives integral action for free. Design it on the same 8-dimensional
controllable subspace: the conserved yaw momentum does not disappear because
you added an observer.

**4. MEKF - not yet, and know the trap.**
A quaternion MEKF is the right structure only when attitude excursions are
large or full 3-DoF attitude is needed: **stage 3 jump-up, and corner-to-corner
transitions where the reference `gB` moves 67-107 deg**. For corner balance the
controller uses reduced attitude, tilt stays under 15 deg, and the linear KF is
provably adequate.

The trap, and it is measured, not theoretical: with a single vector
measurement **yaw is unobservable, and so is the gyro-bias component along
gravity.** In the CF run the true bias projected onto `gB` was 0.149 deg/s and
the estimator recovered exactly 0.000 - correctly, because nothing observes
it. The perpendicular components converged to the true values. A naive MEKF
will let that direction's covariance grow without bound. Either constrain it,
or use the reduced-attitude form which structurally cannot have the problem.

Adding a magnetometer would make yaw observable, but yaw is also
**uncontrollable** (`rank(ctrb) = 8 of 9`), so it would buy nothing for balance.

---

# Part III - Recommended actions, in order

1. **Spin-down and breakaway tests** -> real `tau_cw`, `b_w`, cogging.
   Feedforward is not optional and is worth 6.7 % of the torque budget.
2. **Complementary filter with bias states, kP = 5**, before the gain block.
   Raw accelerometer does not work at all at any realistic IMU position.
3. **Confirm IMU placement** is within ~150 mm of every intended balance
   corner. Geometric centre satisfies all eight at 130 mm.
4. **Thermal test for the real `tau_cont`.** Single biggest lever on the
   envelope: 0.12 -> 0.20 at `omega_cap` 120 takes 3.3 -> 7.0 deg.
5. **Structural tap test.** Strike the frame, log the gyro at maximum rate,
   FFT. A first mode under ~50 Hz is inside the wheel speed range.
6. **Balance the wheels, THEN raise `omega_cap`, THEN re-run the gain grid.**
   In that order. The grid result changes with the cap.
7. **Per-corner gain tables** (216 floats) plus accelerometer corner ID.
8. **Multi-corner tests must run >15 s** or check eigenvalues directly.
9. Then, and only then, the steady-state KF with disturbance-torque states.

# Files

| file | role |
|---|---|
| `cubli_cube_params.m` | mass properties, geometry, eight corner equilibria |
| `cubli_corner_plant.m` | per-corner Theta, Tb, gB, P, A, B, lambda |
| `cubli_gains.m` | reduced-order LQR + yaw projection, returns Ms |
| `cubli_nlsim.m` | nonlinear plant, sensors, actuator limits, estimator |
| `cubli_maxrec.m` | bisection recovery sweep over tilt azimuth |
| `cubli_corner_run.m` | Simscape corner balance, Gate 3 + closed loop |

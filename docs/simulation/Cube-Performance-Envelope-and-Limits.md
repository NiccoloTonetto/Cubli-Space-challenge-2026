# Cubli Cube - Performance Envelope and Limits

Generated from `cubli_nlsim` (full nonlinear, 400 Hz ZOH controller), validated
against the Simscape model to <1 % on the 2 deg transient. Primary corner
(+1,+1,+1) unless stated. `tau_cont = 0.12 N m`, `omega_cap = 40 rad/s`.

Recovery angle = largest initial tilt FROM REST that returns to upright,
minimised over the tilt azimuth in the plane perpendicular to gB. The azimuth
sweep matters: best and worst directions differ by ~6 %.

## 1. Maximum recovery angle

**Worst case at the current settings: 3.34 deg.** Both actuator limits bind
at once - torque saturates 5 % of the time, the momentum cap 11 %.

### Gain retuning buys almost nothing

48-point Bryson grid, qa in {0.05 0.1 0.2 0.4}, qw in {10 20 40 80},
rt in {0.06 0.12 0.24}, qr = 2.0:

| set | worst rec | slowest pole |
|---|---|---|
| baseline 0.20 / 2.0 / 20 / 0.12 | 3.35 deg | -3.53 |
| best     0.20 / 2.0 / 10 / 0.24 | 3.42 deg | -7.56 |

**+2 % across an 8x span in the weights.** Only `qw` moves the needle, and it
moves it by penalising wheel rate harder - i.e. by respecting the momentum cap.
The recovery angle is a HARDWARE limit, not a tuning limit. Stop tuning.

Ms = 1.000000 on every set, so none of this is an implementation artefact.

### What does move it: the actuator budget

Worst-case recovery [deg], gains fixed at 0.20 / 2.0 / 10 / 0.24:

| tau_max \\ omega_cap | 40 | 60 | 80 | 120 | 200 |
|---|---|---|---|---|---|
| **0.10** | 3.09 | 3.53 | 3.45 | 3.45 | 3.45 |
| **0.15** | 3.80 | 4.61 | 5.12 | 5.16 | 5.16 |
| **0.20** | 4.25 | 5.38 | 6.15 | 7.04 | 6.86 |
| **0.30** | 4.79 | 6.36 | 7.53 | 9.21 | 10.83 |
| **0.40** | 5.03 | 6.92 | 8.41 | 10.76 | 13.39 |

Reading the table:

- Below tau ~0.15 the cube is torque-bound and the wheel cap is irrelevant.
- At tau = 0.12 the ceiling is ~3.5 deg no matter how fast the wheels may spin.
- Neither limit alone is worth relaxing. Together they multiply:
  0.12/40 -> 0.30/120 is 3.3 -> 9.2 deg.
- **Non-monotonic** at tau 0.20 (7.04 at wc=120, 6.86 at wc=200) and tau 0.10.
  With fixed gains a higher cap lets the wheel run away and it cannot unwind.
  Raising omega_cap requires re-running the gain grid, not just the constant.
- omega_cap = 40 is firmware policy only; the motor free-runs at 883 rad/s.

### Practical read

3.4 deg is roughly hand-placement accuracy. It works, but there is no margin
for a table bump, a draught, or a sloppy release. Getting tau_cont measured and
raised is the single highest-value action on the whole project.

## 2. Multi-corner balance

All eight corners are individually balanceable. The primary corner is the
WORST of the eight, because it has the longest lever arm.

| corner | ell mm | Sg | lambda | theta_eq | rec, own gains | rec, primary gains | max Re |
|---|---|---|---|---|---|---|---|
| (-1,-1,-1) | 122.84 | 1.8875 | 8.2572 | 0.797 | 3.88 | (false pass) | +0.149 |
| (-1,-1,+1) | 128.54 | 1.9750 | 8.1212 | 3.170 | 3.67 | FALLS | +7.908 |
| (-1,+1,-1) | 126.08 | 1.9373 | 8.1591 | 2.773 | 3.74 | FALLS | +7.938 |
| (-1,+1,+1) | 131.64 | 2.0226 | 8.0480 | 3.097 | 3.61 | FALLS | +7.796 |
| (+1,-1,-1) | 128.56 | 1.9753 | 8.1180 | 3.171 | 3.66 | FALLS | +7.909 |
| (+1,-1,+1) | 134.01 | 2.0591 | 7.9804 | 2.609 | 3.43 | FALLS | +7.735 |
| (+1,+1,-1) | 131.66 | 2.0229 | 8.0501 | 3.095 | 3.51 | FALLS | +7.795 |
| (+1,+1,+1) | 136.99 | 2.1048 | 7.9341 | 0.714 | 3.42 | 3.42 | 0.000 |

### One gain set does NOT work

Six corners diverge at essentially the OPEN-LOOP rate (+7.9): the controller is
not slow, it is pushing the wrong way. gB rotates 67-107 deg between corners and
both `P = I - gB gB'` and `Theta` change with it.

The antipodal corner is the trap. `P` is invariant under gB -> -gB, so the
primary gains ALMOST work there: max Re = +0.1486, a 6.7 s divergence. A 2.5 s
simulation calls that a successful recovery. **It is a slow fall.** Any
multi-corner test must run long enough to distinguish 6.7 s from stable, or
check the eigenvalues directly.

### Implementation

- Store 8 gain matrices, 3x9 each = **216 floats**. Nothing.
- Corner identification from the accelerometer: pick the corner whose gB is
  closest to the measured gravity vector. Minimum separation between any two
  corner gB vectors is **67.2 deg**, so up to 33 deg of tilt error still IDs
  correctly. No IMU reconfiguration, no ambiguity.
- theta_eq is **0.71 to 3.17 deg**, not the 8.3 deg carried in the old notes.
  That figure was the 180 mm configuration and is stale. Locomotion is much
  less compromised by the COM offset than previously recorded.

## 3. Nonlinearities

From 1 deg initial tilt, 8 s, steady state measured over the last 2 s.

| effect | rec [deg] | tilt rms | wheel drift | tau rms |
|---|---|---|---|---|
| ideal | 3.42 | 0.000 | 0.0 | 0.0000 |
| wheel Coulomb 8 mNm, no FF | 3.29 | 0.001 | 1.8 rad/s | 0.0080 |
| the same, WITH feedforward | 3.29 | 0.000 | 0.0 | 0.0000 |
| pivot friction 5 mNm | 3.36 | 0.002 | 0.0 | 0.0001 |
| accel noise 3.2 mrad | 3.42 | 0.040 | 0.7 | 0.0308 |
| gyro noise 2.4 mrad/s | 3.42 | 0.004 | 0.1 | 0.0030 |
| delay 1 sample (2.5 ms) | 3.38 | - | - | - |
| delay 2 samples (5 ms) | 3.33 | - | - | - |
| delay 3 samples (7.5 ms) | 3.29 | - | - | - |
| **all of it, FF on, 5 ms delay** | **3.16** | 0.038 | 0.5 | 0.0308 |

Total realistic degradation **3.42 -> 3.16 deg, -7.6 %**. Nothing here is a
showstopper, but the ranking is not what you would guess.

### Friction feedforward - confirmed mandatory, now quantified

Without it the wheel drifts to 1.8 rad/s and the loop burns **6.7 % of the
continuous torque budget** permanently just cancelling bearing drag. With it,
both go to exactly zero. tau_cw = 8 mNm is still a placeholder - the spin-down
test sets this number and it directly buys back torque headroom.

### Accelerometer noise is the expensive one

| tilt-estimate noise | tau rms | % of budget | tilt rms |
|---|---|---|---|
| 3.2 mrad (raw BMI270 @ 400 Hz) | 0.0313 | **26.1 %** | 0.041 deg |
| 1.0 mrad | 0.0116 | 9.7 % | 0.014 deg |
| 0.32 mrad | 0.0067 | 5.6 % | 0.005 deg |
| 0.10 mrad | 0.0042 | 3.5 % | 0.002 deg |

Note the recovery angle does NOT change (3.18 deg throughout). Noise does not
cost stability - it costs **thermal headroom**, and thermal headroom is exactly
what sets tau_cont. Feeding the raw accelerometer to the controller throws away
a quarter of the actuator before the cube has done anything. Fuse it with the
gyro (complementary filter is enough) before it reaches the gain.

### Wheel unbalance - the sleeper risk

Not in the rigid model. Forced torque = m_wheel * ecc * omega^2 * 167.1 mm,
as a percentage of tau_cont = 0.12 N m:

| ecc | 40 rad/s (6.4 Hz) | 120 rad/s (19.1 Hz) | 200 rad/s (31.8 Hz) |
|---|---|---|---|
| 0.05 mm | 2.4 % | 21.7 % | 60.1 % |
| 0.10 mm | 4.8 % | 43.3 % | 120.3 % |
| 0.20 mm | 9.6 % | 86.6 % | 240.6 % |
| 0.50 mm | 24.1 % | 216.5 % | 601.5 % |

It scales as omega^2. At the present 40 rad/s cap a 0.1 mm eccentricity is a
5 % nuisance. At the 120 rad/s the recovery chart says you want, the same wheel
is a 43 % disturbance at 19 Hz. **Balance the wheels before raising the cap**,
and note that 0.1 mm on a printed part with 16 nuts and bolts pressed into it
is optimistic, not conservative.

### Parameter robustness (nonlinear, gains fixed)

| Theta scaling | recovery | max Re |
|---|---|---|
| x0.70 | 3.41 | 0.000 |
| x0.85 | 3.29 | 0.000 |
| x1.00 | 3.18 | 0.000 |
| x1.15 | 3.09 | 0.000 |
| x1.30 | 3.00 | 0.000 |

Stable across the whole +/-30 % band, and the envelope moves only +/-0.2 deg.
**The rho_scale assumption in `cubli_cube_params` does not matter for the
recovery envelope.** The plumb-line and swing tests are still worth doing, but
they are not blocking.

## Not modelled - go measure these

- **Frame flexibility.** A rigid-body simulation cannot see it. Tap test: strike
  the frame, log the gyro at maximum rate, FFT. If the first mode is under
  ~50 Hz it will be inside the wheel speed range and will couple.
- **Motor cogging.** MN4006 is an outrunner; cogging is likely the same order as
  the bearing friction. Characterise it in the same spin-down run and fold it
  into the same feedforward.
- **Corner geometry.** The model assumes a perfect point contact. A rounded
  corner rolls rather than pivots, which lowers lambda slightly, and the
  required friction coefficient to avoid slip is unchecked.
- **Thermal.** tau_cont = 0.12 N m is still an estimate, and 26 % of it is
  currently being spent on sensor noise. The usable figure today is ~0.089.

## Recommended order of work

1. Spin-down and breakaway tests -> real tau_cw, b_w, cogging. Feedforward is
   not optional and is worth 6.7 % of the torque budget.
2. Gyro/accel fusion before the gain block. Worth another ~20 % of the budget.
3. Thermal test for the real tau_cont. Biggest single lever on the envelope.
4. Balance the wheels, THEN raise omega_cap, THEN re-run the gain grid.
5. Per-corner gain tables (216 floats) plus accelerometer corner ID.
6. Multi-corner tests must run >15 s or check eigenvalues - the antipodal
   corner fails slowly enough to look like a pass.

## Files

- `cubli_corner_plant.m` - per-corner Theta, Tb, gB, P, A, B, lambda
- `cubli_gains.m` - reduced-order LQR + yaw projection, returns Ms
- `cubli_nlsim.m` - full nonlinear sim, friction / noise / delay / caps
- `cubli_maxrec.m` - bisection recovery-angle sweep over tilt azimuth
- `cubli_corner_run.m` - Simscape corner balance, Gate 3 + closed loop

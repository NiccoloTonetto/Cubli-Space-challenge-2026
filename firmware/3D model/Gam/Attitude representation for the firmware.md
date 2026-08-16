# Attitude representation for the Teensy firmware

**Use `gam`, the gravity direction in body coordinates. Not Euler angles, not
a quaternion.** This note is the reasoning, because the choice is not a style
preference - it decides whether the loop works.

## 1. What the controller actually needs

The design plant is  `Tb*omdot = Sg*P*phi - u`,  `P = I - gB*gB'`.

Only the component of attitude error PERPENDICULAR to `gB` appears. The
parallel component - yaw about the vertical - is:

- **unobservable.** One vector measurement fixes 2 DoF of attitude, not 3. An
  accelerometer tells you which way is down, never which way you are facing.
- **uncontrollable.** `rank(ctrb) = 8 of 9`, because `gB'*(Theta*om + Is*rho)`
  is conserved: the wheels are internal and gravity has no moment about the
  vertical.

Yaw is not merely unnecessary. Any representation carrying it is carrying a
state that cannot be measured and cannot be driven, and that state will find a
way to contaminate the two that matter.

## 2. What `gam` is

A unit 3-vector: the direction of gravity in the CUBE BODY frame. `gam == gB`
is balanced. Three numbers, one norm constraint, **2 DoF - exactly the
observable and controllable subspace, nothing more.**

It is also what the sensor gives: `gam = normalize(-a_cal)`.

Kinematics is one line, `gamdot = -om x gam`, and the control input is one
cross product, `phi = -gB x gam`.

`phi` is perpendicular to `gB` **by construction** - a cross product with `gB`
always is. Yaw therefore cannot leak into the tilt channel. Not approximately,
not to first order: exactly, at any attitude. Verified: `P*phi - phi = 6.0e-18`
and a 30 deg pure-yaw rotation moves `gam` by `1.6e-16`. The nonlinear gravity
torque `m*g*(r_c x gam)` matches `Sg*phi` to `9.9e-17 N m`.

## 3. Why Euler angles fail - three MEASURED failures

**Gimbal lock.** The first Simscape model produced `q_y` spikes to +/-20000
deg. A rigid body cannot do that; the parameterisation went singular. On
hardware that is a divide-by-near-zero mid-recovery.

**Euler angles are not the reduced attitude.** Even far from lock the triple
is not a rotation vector, and `P` removes yaw from it only to FIRST ORDER. Yaw
here is not small: `gB'*Theta*gB = 0.00538 kg m^2` makes the cube **4.5x
lighter in yaw than in tilt**, so wheel momentum along `gB` spins the body at
0.090 rad/s per rad/s of `rho.gB`. Released from 3.6 deg, yaw reaches **23.6
deg within 0.3 s** - momentum-conserving, entirely physical. Feeding the Euler
triple cost **19.5 % of the recovery angle** and the loop diverged.

**Euler rates are not body rates.** `[qd1 qd2 qd3]` equals `omega` only at
`q = 0`; over the trajectory it is **up to 73 % wrong**. Correcting both took
the Simscape model from 3.33 to 4.14 deg, matching the nonlinear simulator to
**0.35 %**.

All three are the same mistake: a signal correct at the equilibrium treated as
correct everywhere.

## 4. Why not a quaternion / MEKF

A quaternion is 4 numbers with a norm constraint and carries yaw EXPLICITLY.
With one vector measurement that direction is unobservable, so a naive MEKF's
covariance grows without bound there unless constrained by hand.

Measured, not theoretical: in the complementary filter the gyro-bias component
along gravity is likewise unobservable. True value 0.149 deg/s, estimator
recovered **exactly 0.000**, while both perpendicular components converged to
truth. Correct behaviour - nothing observes it - and the reduced-attitude form
cannot even represent the problem.

A quaternion MEKF is the right structure when you need large-attitude tracking
against a full reference: **stage 3 jump-up, and corner-to-corner transitions
where `gB` moves 67-107 deg.** Not for corner balance, where tilt stays under
15 deg. A magnetometer would make yaw observable and buy nothing, because yaw
is still uncontrollable.

## 5. The C++ implementation

```cpp
// per corner, from cubli_gains: 3 + 27 floats; 8 corners = 240 total
const float gB[3];              // gravity direction at balance, BODY axes
const float Kp[3][9];           // rows = wheel X,Y,Z; cols = [phi om rho]

// estimator state, persistent
float ghat[3] = { gB[0], gB[1], gB[2] };
float bhat[3] = { 0, 0, 0 };

// ---- once per 400 Hz cycle ------------------------------------------
// 1. sensors, calibrated and already rotated into BODY axes by R_IB
float a[3], w[3], rho[3];       // m/s^2, rad/s, rad/s

// 2. accelerometer -> gravity direction
float ga[3] = normalize(neg(a));

// 3. reduced-attitude complementary filter,  kP = 4, kI = 0.5
float e[3]  = cross(ghat, ga);
float wc[3] = sub(w, bhat);
ghat = add(ghat, scale(add(neg(cross(wc, ghat)), scale(cross(e, ghat), kP)), dt));
normalize(ghat);
bhat = add(bhat, scale(e, kI*dt));       // PLUS. minus converges to 5x the bias
float om[3] = sub(w, bhat);

// 4. the entire attitude computation: one cross product
float phi[3] = neg(cross(gB, ghat));

// 5. LQR, feedforward, limits
float x[9] = { phi[0],phi[1],phi[2], om[0],om[1],om[2], rho[0],rho[1],rho[2] };
float u[3];
for (int i=0;i<3;i++) {
    u[i] = 0.0f;
    for (int j=0;j<9;j++) u[i] -= Kp[i][j]*x[j];
    u[i] += tau_cw*tanhf(rho[i]/eps_ff) + b_w*rho[i];    // friction FF
    u[i]  = clampf(u[i], -tau_max, tau_max);
    if ((rho[i] >=  omega_cap && u[i] > 0.0f) ||
        (rho[i] <= -omega_cap && u[i] < 0.0f)) u[i] = 0.0f;
}
```

### Cost

Three cross products, one 3x9 matvec, one reciprocal square root, three
`tanhf`. About 60 multiply-adds. **No `atan2`, no `sin`/`cos`, no quaternion
product, no matrix exponential, no sequence convention to get wrong.** `float`
is sufficient - everything is a unit vector or an O(100) rate. On a 600 MHz
Teensy 4.1 the 2.5 ms budget goes to CAN, not to this.

### The contract

- State order `[phi(3); om(3); rho(3)]`, body axes X, Y, Z, in THAT order.
- **Do not port the Simscape Mux permutation `[1 4 2 5 3 6 7 8 9]`.** It is an
  artefact of how the model happens to be wired. Firmware builds the vector
  directly in design order.
- `phi` and `om` are in the CUBE BODY frame. Rotate the IMU output by `R_IB`
  first, and verify `R_IB` with the six-face test - never from the datasheet
  drawing.
- `rho[k] > 0` means rotation about `+body axis k`. Verify per wheel on the
  bench.

### One numerical caution

`e = cross(ghat, ga)` has magnitude `sin(error)`: well behaved under 90 deg,
but zero again at 180 deg. If the cube is picked up and inverted, `ghat` can
lock onto the antipode. Add a reset - if `dot(ghat, ga) < 0` for N consecutive
cycles, set `ghat = ga` and zero `bhat`. Disarm above ~15 deg of tilt: past
recovery range the torque is only noise and heat.
"},"
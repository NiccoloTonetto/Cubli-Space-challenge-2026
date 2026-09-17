# Final configuration and per-corner results

All numbers below are the SHIPPING configuration, not the ideal-sensor one:
tau_max 0.12 N m, omega_cap 40 rad/s, bearing friction + feedforward, 5 ms
loop delay, BMI270-grade noise (accel 3.2 mrad, gyro 2.4 mrad/s), gyro bias
[0.3 -0.2 0.15] deg/s, IMU at the GEOMETRIC CENTRE (129.9 mm from every
corner), reduced-attitude complementary filter with bias states.

## 1. Per-corner results, own gains

| corner | ell mm | Sg | lambda | theta_eq | recovery | wheel peak |
|---|---|---|---|---|---|---|
| (-1,-1,-1) | 122.84 | 1.8875 | 8.2572 | 0.797 | **3.14 deg** | 41.7 |
| (-1,+1,-1) | 126.08 | 1.9373 | 8.1591 | 2.773 | 3.03 | 41.7 |
| (-1,-1,+1) | 128.54 | 1.9750 | 8.1212 | 3.170 | 2.93 | 41.3 |
| (+1,-1,-1) | 128.56 | 1.9753 | 8.1180 | 3.171 | 2.92 | 41.7 |
| (-1,+1,+1) | 131.64 | 2.0226 | 8.0480 | 3.097 | 2.89 | 41.3 |
| (+1,+1,-1) | 131.66 | 2.0229 | 8.0501 | 3.095 | 2.83 | 41.7 |
| (+1,-1,+1) | 134.01 | 2.0591 | 7.9804 | 2.609 | 2.77 | 41.7 |
| (+1,+1,+1) | 136.99 | 2.1048 | 7.9341 | 0.714 | **2.76 deg** | 41.7 |

**All eight balance.** Spread 14 %, and recovery tracks 1/Sg exactly: the
longest lever arm is the hardest corner. The primary (+1,+1,+1) is the WORST
of the eight - if it works there it works everywhere.

Wheel peak is 41.7 rad/s against a 40 rad/s cap at every corner (small
overshoot because the cap zeroes torque rather than braking). **Every corner
is momentum-bound**, not torque-bound.

Each corner needs its OWN gains. With the primary corner's gains: six corners
diverge at the open-loop rate (max Re +7.7 to +7.9) and the antipodal corner
sits at +0.149, a 6.7 s fall that short runs score as a pass.

## 2. Final LQR gains

```
qa = 0.20 rad     qr = 2.0 rad/s     qw = 10 rad/s     rt = 0.24 N m
Q  = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)])
R  = (12/rt^2)*eye(3)
```

Chosen on ROBUSTNESS, not on peak. The grid optimum is qw = 5 at 2.91 deg,
but it dies on plant error:

| qw | Theta x0.8 | x0.9 | x1.0 | x1.1 | x1.2 | verdict |
|---|---|---|---|---|---|---|
| 5 | 3.11 | 3.05 | 2.99 | **0.00** | **0.00** | fails at +10 % on Theta |
| 6 | 3.07 | 3.00 | 2.94 | 2.57 | **0.00** | fails at +20 % |
| 8 | 2.98 | 2.92 | 2.86 | 2.80 | **0.00** | fails at +20 % |
| **10** | 2.91 | 2.85 | **2.79** | 2.73 | 2.68 | **survives the whole band** |
| 12 | 2.84 | 2.78 | 2.72 | 2.67 | 2.62 | survives, 3 % worse |

qw = 10 costs 4 % against the fragile optimum and is the only set that holds
across +/-20 % on Theta. Theta is still an ASSUMED split (rho_scale), so this
margin is not optional. Below qw = 5 there is a cliff: qw 4 -> 2.06,
qw 3 -> 1.58, qw 2 -> 0.00, as max|Kp| climbs 13 -> 29 and the controller
starts fighting the estimator.

## 3. Final filter gains - CORRECTION

```
kP = 4      kI = 0.5      reduced-attitude complementary filter with bias
```

An earlier note said the kP cliff was at 10 and to set 5. **The cliff is
between 6 and 7**, measured at qw = 10 with the IMU at the geometric centre:

| kP | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|
| recovery | 2.67 | 2.74 | 2.79 | 2.82 | **0.00** | **0.00** |

kP = 5 is only 1.4x below a hard cliff. kP = 4 costs 2 % and gives 1.75x.
Take the margin for hardware bring-up; the plant on the bench is not the
plant in the model. Do NOT tune kP upward until it looks crisp.

## 4. Expected result

**2.76 deg worst case, 3.14 deg best, on the shipping configuration.**
Roughly hand-placement accuracy. Every corner is momentum-bound, so the
single highest-value change remains raising omega_cap - after balancing the
wheels, and re-running the gain grid, in that order.

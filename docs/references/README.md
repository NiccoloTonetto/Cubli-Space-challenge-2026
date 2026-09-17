# References

The literature this project builds on. The PDFs themselves are not redistributed
here — three of the four are IEEE copyright — so each entry below gives the DOI
or arXiv link to fetch it from the publisher.

## Primary — the ETH Zurich Cubli

**[1] The Cubli: A Cube that can Jump Up and Balance**
M. Gajamohan, M. Merz, I. Thommen, R. D'Andrea.
*IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2012*, pp. 3722–3727.
DOI: [10.1109/IROS.2012.6385902](https://doi.org/10.1109/IROS.2012.6385902)

> The 1D prototype. Establishes the reaction-wheel-braking jump-up and the
> single-axis balancing plant this project's [2D panel model](../simulation/Simulation-Strategies.md)
> reproduces.

**[2] The Cubli: A Reaction Wheel Based 3D Inverted Pendulum**
M. Gajamohan, M. Muehlebach, T. Widmer, R. D'Andrea.
*European Control Conference (ECC), 2013*, pp. 268–274.
DOI: [10.23919/ECC.2013.6669562](https://doi.org/10.23919/ECC.2013.6669562)

> The full 3D cube: mechatronic design, multi-body dynamics, frequency-domain
> parameter identification on the edge, and corner balancing by linear feedback.
> This is the closest reference to what is built here, and the source of the
> corner-balance formulation used in [`3D-Cubli-Lagrangian-Derivation.md`](../dynamics/3D-Cubli-Lagrangian-Derivation.md).

**[3] Nonlinear Analysis and Control of a Reaction Wheel-based 3D Inverted Pendulum**
M. Muehlebach, R. D'Andrea.
*IEEE Transactions on Control Systems Technology*, 2017, vol. 25, no. 1, pp. 235–246.
DOI: [10.1109/TCST.2016.2549266](https://doi.org/10.1109/TCST.2016.2549266)

> Backstepping controller with global stability, plus a gradient-based learning
> algorithm for jump-up. The benchmark this project's LQR is deliberately
> *simpler* than — see the estimator roadmap in
> [`Cube-Performance-Envelope-Results.md`](../dynamics/Cube-Performance-Envelope-Results.md)
> for why a quaternion MEKF and a nonlinear law were scoped out for corner
> balance specifically.

## Attitude representation

**[4] The Cubli: Modeling and Nonlinear Control Utilizing Unit Complex Numbers**
F. Bobrow, B. A. Angelico, P. S. P. da Silva. 2020.
arXiv: [2009.14625 [eess.SY]](https://arxiv.org/abs/2009.14625)

> Unit complex numbers for the 1D case, quaternions for the 3D case. Read
> alongside [`Quaternions-Complete-Guide.md`](../dynamics/Quaternions-Complete-Guide.md)
> and [`Attitude-Representation-for-Firmware.md`](../dynamics/Attitude-Representation-for-Firmware.md);
> this project ends up using a *reduced-attitude* formulation (gravity direction
> in body frame) rather than a full quaternion, for the reasons set out there.

## How these are used

The derivations in [`docs/dynamics/`](../dynamics/) are worked from scratch rather
than copied, and the notation is stated locally in each document. Where a result
matches one of the papers above it is cited inline at the point of use; where this
project departs from them — the reduced-attitude estimator, the per-corner gain
tables, the momentum-bound rather than torque-bound operating point — the
departure is argued rather than assumed.

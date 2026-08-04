---
tags: [space-challenge, sofia, cubli, dynamics, derivation]
---

# Cubli 3D — Full Lagrangian Derivation of the Equations of Motion

Target system: the **most advanced Cubli realistically buildable in Sofia** — the full ETH-style 3D Cubli. A rigid cube pivoting on one **corner**, with **three reaction wheels** mounted on three mutually orthogonal faces, each driven by a BLDC motor. Capable of corner balancing and (stretch) jump-up. This is the configuration of Gajamohan et al. (ECC 2013) and Muehlebach & D'Andrea (TCST 2016); the derivation below follows their structure, with the quaternion/vector formalism of Ribas Martins & da Silva (arXiv:2009.14625) where it's cleaner.

Everything is derived from scratch so it can be checked line by line, then specialized to the 1D edge case as a sanity check at the end.

---

## 1. System definition and assumptions

**Bodies.** Four rigid bodies:
- the **structure** $\mathcal{S}$: cube housing, motors' stators, electronics, batteries — everything that does *not* spin with the wheels;
- three **reaction wheels** $\mathcal{W}_1, \mathcal{W}_2, \mathcal{W}_3$, each spinning about a body-fixed axis.

**Assumptions.**
1. The pivot corner $O$ is a fixed, frictionless point contact — the cube rotates about $O$ but does not translate or slip. (Holonomic constraint: 3 translational DOF removed.)
2. Each wheel is axisymmetric about its spin axis, so its inertia tensor is invariant under its own spin. This is *the* key simplification: the total system inertia about $O$ is constant in the body frame.
3. Wheel spin axes $\{\boldsymbol a_1, \boldsymbol a_2, \boldsymbol a_3\}$ are mutually orthogonal unit vectors, fixed in the body frame, aligned with the cube's face normals.
4. Motors apply torques $u_i$ about $\boldsymbol a_i$ between structure and wheel $i$ (action–reaction pair).
5. Aerodynamic drag neglected; wheel bearing friction optionally modeled later as viscous ($-b_w\dot\varphi_i$).

**Degrees of freedom.** After the pivot constraint: 3 (attitude of the cube) + 3 (wheel spin angles) = **6 DOF**. Configuration space $SO(3) \times \mathbb{T}^3$ — this non-Euclidean configuration space is why plain Lagrange equations in coordinates are awkward and we will use the Euler–Poincaré form for the rotational part (§7).

---

## 2. Frames and notation

- $\{I\}$: inertial frame, origin at the pivot $O$, with $\boldsymbol e_z^I$ pointing **up** (against gravity).
- $\{B\}$: body frame, origin at $O$, fixed to the structure.
- $R \in SO(3)$: rotation matrix from body to inertial, so a vector $\boldsymbol v$ expressed in $\{B\}$ appears in $\{I\}$ as $R\,{}^B\boldsymbol v$.
- $\boldsymbol\omega \in \mathbb{R}^3$: angular velocity of the structure, **expressed in $\{B\}$**.
- $\varphi_i$, $\dot\varphi_i$: spin angle and rate of wheel $i$ **relative to the structure**.
- $m_s$, $m_w$: mass of structure, mass of each wheel; $m = m_s + 3m_w$ total.
- $\boldsymbol r_i$: position of wheel $i$'s center of mass in $\{B\}$ (constant).
- $\boldsymbol c$: position of the **total system COM** in $\{B\}$ (constant, by assumption 2).
- $I_w$: wheel moment of inertia about its spin axis; $J_w$: about any transverse axis through its COM.
- $\hat{(\cdot)}$: skew map, $\hat{\boldsymbol a}\boldsymbol b = \boldsymbol a \times \boldsymbol b$.

**Gravity in the body frame.** Define
$$\boldsymbol g_B \;\equiv\; R^\top(-g\,\boldsymbol e_z^I) \in \mathbb{R}^3,$$
the gravity acceleration vector seen from the body frame. It has constant norm $g$ and evolves purely kinematically (§3). Using $\boldsymbol g_B$ as a state (instead of Euler angles) is the trick that keeps the whole derivation singularity-free.

---

## 3. Kinematics

**Attitude.** With unit quaternion $q = (q_0, \boldsymbol q_v)$ representing $R$:
$$\dot q = \tfrac12\, q \otimes \begin{pmatrix} 0 \\ \boldsymbol\omega \end{pmatrix},$$
which is the standard quaternion kinematic equation with body-frame angular velocity.

**Gravity vector propagation.** Since $\boldsymbol e_z^I$ is constant in $\{I\}$, differentiating $\boldsymbol g_B = -gR^\top \boldsymbol e_z^I$ and using $\dot R = R\hat{\boldsymbol\omega}$:
$$\boxed{\;\dot{\boldsymbol g}_B = \boldsymbol g_B \times \boldsymbol\omega\;}$$
Two remarks: (i) $\|\boldsymbol g_B\| = g$ is an invariant — a good numerical health check in simulation; (ii) $\boldsymbol g_B$ captures only the *reduced attitude* (2 DOF): rotation about the vertical is invisible to it. That symmetry will reappear in the equilibrium analysis (§9).

**Wheel angular velocities.** Wheel $i$ is rigidly attached to the structure except for its spin, so its total angular velocity in $\{B\}$ is
$$\boldsymbol\omega_{w,i} = \boldsymbol\omega + \dot\varphi_i\,\boldsymbol a_i.$$

---

## 4. Kinetic energy

We build $T = T_{\mathcal S} + \sum_i T_{\mathcal W_i}$ term by term. All about the fixed point $O$, all in $\{B\}$.

### 4.1 Structure

The structure rotates rigidly about $O$ with $\boldsymbol\omega$:
$$T_{\mathcal S} = \tfrac12\,\boldsymbol\omega^\top \Theta_{\mathcal S}\,\boldsymbol\omega,$$
where $\Theta_{\mathcal S}$ is the structure's inertia tensor **about $O$** (COM inertia + parallel-axis / Steiner term), constant in $\{B\}$.

### 4.2 One wheel — the careful part

Wheel $i$'s kinetic energy splits (König's theorem) into translation of its COM plus rotation about its COM:
$$T_{\mathcal W_i} = \underbrace{\tfrac12 m_w \|\boldsymbol\omega \times \boldsymbol r_i\|^2}_{\text{COM translation}} \;+\; \underbrace{\tfrac12\,\boldsymbol\omega_{w,i}^\top I_{w,i}^{\mathrm{COM}}\,\boldsymbol\omega_{w,i}}_{\text{rotation about COM}}.$$

By axisymmetry (assumption 2), the wheel's COM inertia tensor in $\{B\}$ is constant:
$$I_{w,i}^{\mathrm{COM}} = I_w\,\boldsymbol a_i\boldsymbol a_i^\top + J_w\,(\mathbb 1 - \boldsymbol a_i\boldsymbol a_i^\top).$$

Substitute $\boldsymbol\omega_{w,i} = \boldsymbol\omega + \dot\varphi_i \boldsymbol a_i$ and expand the rotational term. Using $\boldsymbol a_i^\top\boldsymbol a_i = 1$ and writing $\omega_{a_i} \equiv \boldsymbol a_i^\top \boldsymbol\omega$ (the component of body rate along the spin axis):

$$\tfrac12(\boldsymbol\omega + \dot\varphi_i\boldsymbol a_i)^\top I_{w,i}^{\mathrm{COM}} (\boldsymbol\omega + \dot\varphi_i\boldsymbol a_i)
= \tfrac12\,\boldsymbol\omega^\top I_{w,i}^{\mathrm{COM}}\,\boldsymbol\omega + I_w\,\dot\varphi_i\,\omega_{a_i} + \tfrac12 I_w\,\dot\varphi_i^2.$$

(The cross terms with $J_w$ vanish because $(\mathbb 1 - \boldsymbol a_i\boldsymbol a_i^\top)\boldsymbol a_i = 0$.)

The COM-translation term expands as $\tfrac12 m_w\,\boldsymbol\omega^\top(\|\boldsymbol r_i\|^2\mathbb 1 - \boldsymbol r_i\boldsymbol r_i^\top)\boldsymbol\omega$ — a Steiner contribution.

### 4.3 Total kinetic energy

Collect all $\boldsymbol\omega$-quadratic terms into one constant tensor. Define the **locked inertia** (total inertia of the assembly about $O$ with all wheels frozen):
$$\hat\Theta \;\equiv\; \Theta_{\mathcal S} + \sum_{i=1}^{3}\Big[ m_w(\|\boldsymbol r_i\|^2\mathbb 1 - \boldsymbol r_i\boldsymbol r_i^\top) + I_w\,\boldsymbol a_i\boldsymbol a_i^\top + J_w(\mathbb 1 - \boldsymbol a_i\boldsymbol a_i^\top)\Big].$$

Then
$$\boxed{\;T = \tfrac12\,\boldsymbol\omega^\top\hat\Theta\,\boldsymbol\omega \;+\; \sum_{i=1}^{3}\Big( I_w\,\dot\varphi_i\,\boldsymbol a_i^\top\boldsymbol\omega + \tfrac12 I_w\,\dot\varphi_i^2 \Big)\;}$$

The middle term is the rotor–body coupling — the entire mechanism by which the wheels control the cube lives in that term.

---

## 5. Potential energy

Only gravity. With the total COM at $\boldsymbol c$ (in $\{B\}$) and using $\boldsymbol g_B$:
$$V = -m\,\boldsymbol c^\top \boldsymbol g_B.$$
Check: cube upright ⇒ $\boldsymbol g_B$ anti-parallel to $\boldsymbol c$ ⇒ $V = +mg\|\boldsymbol c\|$, the maximum — correct for an inverted pendulum.

---

## 6. Lagrangian and generalized forces

$$L = T - V = \tfrac12\boldsymbol\omega^\top\hat\Theta\boldsymbol\omega + \sum_i\Big(I_w\dot\varphi_i\,\boldsymbol a_i^\top\boldsymbol\omega + \tfrac12 I_w\dot\varphi_i^2\Big) + m\,\boldsymbol c^\top\boldsymbol g_B.$$

**Generalized forces.** Motor $i$ exerts $+u_i$ on wheel $i$ and $-u_i$ on the structure about $\boldsymbol a_i$ (Newton's third law). Optional viscous friction $-b_w \dot\varphi_i$ on each wheel, and (if desired) pivot friction $-\boldsymbol\tau_f(\boldsymbol\omega)$ on the body.

---

## 7. Equations of motion

### 7.1 Wheel equations (Lagrange, cyclic-style coordinates)

$\varphi_i$ are honest generalized coordinates, so standard Lagrange applies:
$$\frac{d}{dt}\frac{\partial L}{\partial \dot\varphi_i} - \frac{\partial L}{\partial \varphi_i} = u_i .$$
Since $\varphi_i$ does not appear in $L$ (cyclic!):
$$\frac{d}{dt}\Big[ I_w(\dot\varphi_i + \boldsymbol a_i^\top\boldsymbol\omega) \Big] = u_i \;(-\,b_w\dot\varphi_i).$$

Define the **total axial angular momentum of wheel $i$**:
$$h_i \equiv I_w\big(\dot\varphi_i + \boldsymbol a_i^\top\boldsymbol\omega\big) \quad\Rightarrow\quad \boxed{\;\dot h_i = u_i\;}\;(\text{minus friction if modeled}).$$
This is a beautiful structural fact: the motor torque directly commands the rate of the wheel's absolute axial momentum, independent of everything else.

### 7.2 Body equations (Euler–Poincaré on SO(3))

The attitude is not a vector of coordinates, so instead of coordinate-Lagrange we use the Euler–Poincaré equation (equivalently: the rate of total angular momentum about the fixed point $O$, expressed in the rotating frame, equals the external torque):
$$\frac{d}{dt}\Big(\frac{\partial L}{\partial\boldsymbol\omega}\Big) + \boldsymbol\omega\times\frac{\partial L}{\partial\boldsymbol\omega} = \boldsymbol\tau_{\mathrm{ext}} + \frac{\partial L}{\partial \text{(attitude)}}\text{-terms},$$
which for our $L$ (gravity entering through $\boldsymbol g_B$, whose variation contributes the gravity torque) becomes the momentum balance
$$\dot{\boldsymbol p}\big|_B + \boldsymbol\omega\times\boldsymbol p = \boldsymbol\tau_g, \qquad \boldsymbol p \equiv \frac{\partial T}{\partial \boldsymbol\omega}.$$

**Total angular momentum about $O$, in $\{B\}$:**
$$\boldsymbol p = \hat\Theta\,\boldsymbol\omega + \sum_i I_w\dot\varphi_i\,\boldsymbol a_i.$$
Rewrite using $h_i$: since $I_w \dot\varphi_i = h_i - I_w\boldsymbol a_i^\top\boldsymbol\omega$,
$$\boldsymbol p = \underbrace{\Big(\hat\Theta - I_w\textstyle\sum_i \boldsymbol a_i\boldsymbol a_i^\top\Big)}_{\displaystyle \bar\Theta}\,\boldsymbol\omega + A\boldsymbol h, \qquad A \equiv [\boldsymbol a_1\;\boldsymbol a_2\;\boldsymbol a_3],\; \boldsymbol h = (h_1,h_2,h_3)^\top.$$
With orthonormal axes, $\sum_i \boldsymbol a_i \boldsymbol a_i^\top = \mathbb 1$, so $\bar\Theta = \hat\Theta - I_w\mathbb 1$: the "wheels-decoupled" inertia.

**Gravity torque about $O$:** $\boldsymbol\tau_g = \boldsymbol c \times m\boldsymbol g_B = m\,\boldsymbol c\times\boldsymbol g_B$.

**Momentum balance:** $\dot{\boldsymbol p}\big|_B = \bar\Theta\dot{\boldsymbol\omega} + A\dot{\boldsymbol h} = \bar\Theta\dot{\boldsymbol\omega} + A\boldsymbol u$. Substituting:

$$\boxed{\;\bar\Theta\,\dot{\boldsymbol\omega} = -\,\boldsymbol\omega\times\big(\bar\Theta\boldsymbol\omega + A\boldsymbol h\big) + m\,\boldsymbol c\times\boldsymbol g_B - A\boldsymbol u\;}$$

Interpretation of each term:
- $-\boldsymbol\omega\times\bar\Theta\boldsymbol\omega$: standard Euler gyroscopic term of the locked body;
- $-\boldsymbol\omega\times A\boldsymbol h$: **gyroscopic coupling from spinning wheels** — at high wheel speeds this dominates and is exactly what makes high-momentum operation tricky;
- $m\,\boldsymbol c\times\boldsymbol g_B$: gravity torque, the destabilizing term;
- $-A\boldsymbol u$: reaction torque of the motors on the body — the control input, entering with a minus sign (spin a wheel one way, the cube reacts the other way).

### 7.3 Complete state-space model

State $x = (q, \boldsymbol\omega, \boldsymbol h) \in S^3\times\mathbb R^3\times\mathbb R^3$ (or replace $q$ with $\boldsymbol g_B$ for the reduced model):

$$
\begin{aligned}
\dot q &= \tfrac12\, q\otimes(0,\boldsymbol\omega) &&\text{(attitude kinematics)}\\
\dot{\boldsymbol g}_B &= \boldsymbol g_B\times\boldsymbol\omega &&\text{(reduced alternative)}\\
\dot{\boldsymbol\omega} &= \bar\Theta^{-1}\Big[-\boldsymbol\omega\times(\bar\Theta\boldsymbol\omega + A\boldsymbol h) + m\,\boldsymbol c\times\boldsymbol g_B - A\boldsymbol u\Big] &&\text{(body dynamics)}\\
\dot{\boldsymbol h} &= \boldsymbol u &&\text{(wheel dynamics)}
\end{aligned}
$$

Recover wheel speeds when needed: $\dot\varphi_i = h_i/I_w - \boldsymbol a_i^\top\boldsymbol\omega$.

This is exactly the structure of ECC 2013's model. **9 states** (or 8 independent with quaternion constraint; 8 with $\boldsymbol g_B$ formulation counting its norm invariant), **3 inputs**.

---

## 8. Sanity check — specialization to the 1D edge case

Restrict rotation to a single axis $\boldsymbol e$ (the balancing edge), with one active wheel aligned to it: $\boldsymbol\omega = \dot\theta\,\boldsymbol e$, one wheel, $\boldsymbol a = \boldsymbol e$. Cross products along the same axis vanish, so the gyroscopic terms die and:

- Body: $\bar\Theta_e\ddot\theta = m g \ell\sin\theta - u$, with $\bar\Theta_e = \hat\Theta_e - I_w$ the (scalar) locked edge inertia minus wheel axial inertia, $\ell = \|\boldsymbol c\|$ the COM distance from the edge.
- Wheel: $\dot h = u$, i.e. $I_w(\ddot\theta + \ddot\varphi) = u$.

This matches the classic 1D reaction-wheel pendulum, and matches what the Lagrangian gives if you redo the 1D case directly with $L = \tfrac12\hat\Theta_e\dot\theta^2 + I_w\dot\theta\dot\varphi + \tfrac12 I_w\dot\varphi^2 - mg\ell\cos\theta$:
$$\frac{d}{dt}(\hat\Theta_e\dot\theta + I_w\dot\varphi) = mg\ell\sin\theta,\qquad \frac{d}{dt}\big(I_w(\dot\theta+\dot\varphi)\big) = u,$$
subtracting: $(\hat\Theta_e - I_w)\ddot\theta = mg\ell\sin\theta - u$. ✓ Consistent — use this as the first cross-check between analytical and Simscape models.

---

## 9. Equilibria — corner balancing

Set $\dot{\boldsymbol\omega} = 0$, $\boldsymbol\omega = 0$, $\dot{\boldsymbol g}_B = 0$, $\boldsymbol u = 0$:
$$m\,\boldsymbol c\times\boldsymbol g_B = 0 \;\Longleftrightarrow\; \boldsymbol g_B \parallel \boldsymbol c.$$
Two families:
- $\boldsymbol g_B = -g\,\boldsymbol c/\|\boldsymbol c\|$: **COM above pivot — the corner-balancing equilibrium** (unstable, the one we stabilize);
- $\boldsymbol g_B = +g\,\boldsymbol c/\|\boldsymbol c\|$: hanging equilibrium (stable, irrelevant here).

Two structural features worth knowing before controller design:
1. **Wheel momenta are free at equilibrium**: any constant $\boldsymbol h$ with $\boldsymbol\omega = 0$ satisfies the equations (as long as $\boldsymbol\omega \times A\boldsymbol h = 0$ trivially). Physically the cube can balance with wheels spinning at constant speed — but this eats actuator headroom, so the controller must regulate $\boldsymbol h\to 0$. This is why $\boldsymbol h$ (or wheel speeds) must be in the LQR state penalty.
2. **Rotation about the vertical is a symmetry** of the reduced ($\boldsymbol g_B$) model: yaw about gravity doesn't change $\boldsymbol g_B$. In the full model this shows up as a neutrally stable direction; ECC 2013 handles it by controlling full attitude, the reduced model simply doesn't see it.

---

## 10. Linearization about the balancing equilibrium

Let $\boldsymbol n \equiv \boldsymbol c/\|\boldsymbol c\|$, equilibrium $\boldsymbol g_B^\star = -g\boldsymbol n$, $\boldsymbol\omega^\star = 0$, $\boldsymbol h^\star = 0$. Perturbations: $\boldsymbol g_B = \boldsymbol g_B^\star + \delta\boldsymbol g$, etc. Keep first-order terms:

$$
\begin{aligned}
\delta\dot{\boldsymbol g} &= \boldsymbol g_B^\star \times \delta\boldsymbol\omega = -g\,\hat{\boldsymbol n}\,\delta\boldsymbol\omega\\
\bar\Theta\,\delta\dot{\boldsymbol\omega} &= m\,\hat{\boldsymbol c}\,\delta\boldsymbol g - A\,\boldsymbol u\\
\delta\dot{\boldsymbol h} &= \boldsymbol u
\end{aligned}
$$

(All gyroscopic terms are second order since $\boldsymbol\omega^\star = 0$, $\boldsymbol h^\star = 0$.) Stack $x = (\delta\boldsymbol g, \delta\boldsymbol\omega, \delta\boldsymbol h) \in \mathbb R^9$:

$$
\dot x = 
\underbrace{\begin{bmatrix}
0 & -g\hat{\boldsymbol n} & 0\\
m\bar\Theta^{-1}\hat{\boldsymbol c} & 0 & 0\\
0 & 0 & 0
\end{bmatrix}}_{A_{\mathrm{lin}}}
x +
\underbrace{\begin{bmatrix}
0\\ -\bar\Theta^{-1}A\\ \mathbb 1
\end{bmatrix}}_{B_{\mathrm{lin}}}
\boldsymbol u .
$$

Notes for implementation:
- $\delta\boldsymbol g$ has only 2 independent components (norm constraint) — either project onto the tangent plane of the sphere at $\boldsymbol g_B^\star$ (rigorous, 8 states) or keep 9 states and accept one uncontrollable direction along $\boldsymbol n$ plus the neutral yaw mode. `ctrb` rank will reveal exactly these.
- The unstable modes come from the coupling block $m\bar\Theta^{-1}\hat{\boldsymbol c}\cdot(-g\hat{\boldsymbol n})$ — eigenvalues $\pm\sqrt{\lambda}$ pairs, the 3D analogue of the 1D pendulum's $\pm\sqrt{mg\ell/\bar\Theta}$.
- LQR: penalize $\delta\boldsymbol g$ (attitude), $\delta\boldsymbol\omega$, **and $\delta\boldsymbol h$** (see §9 point 1). A sensible starting point: Bryson's rule with max tilt ~5°, max wheel momentum from motor datasheet.

**1D linearized check:** $(\bar\Theta_e)\ddot{\delta\theta} = mg\ell\,\delta\theta - u$, $\dot h = u$ — poles at $\pm\sqrt{mg\ell/\bar\Theta_e}$ and $0$; controllable with one input. Matches §8. ✓

---

## 11. What to add on top for a *high-fidelity* Sofia build model

The EOM above are the exact rigid-body core. Fidelity gaps to close in Simscape, in priority order:

1. **Motor dynamics:** replace $\dot h_i = u_i$ with a first-order torque lag $\tau_m\dot u_i^{\mathrm{actual}} = u_i^{\mathrm{cmd}} - u_i^{\mathrm{actual}}$, plus torque saturation and a torque–speed line (BLDC back-EMF limit). ETH used Maxon EC-45-flat 50 W.
2. **Wheel friction:** viscous $-b_w\dot\varphi_i$ (+ optional Coulomb). Enters the wheel equation and, by reaction, the body equation.
3. **Sensor models:** 6 IMUs (ETH used MPU-6050 on each face) → gyro noise + bias random walk, accelerometer noise; encoder quantization on $\dot\varphi_i$. Feeds the Kalman filter design.
4. **Discrete-time control loop:** ETH ran at a fixed rate over I2C; sample-and-hold + computation delay change margins noticeably at these bandwidths.
5. **Pivot imperfection:** real corner sits in a small cup/socket — compliance and slight rolling can be approximated with a stiff bushing joint if we get ambitious.
6. **Jump-up (hybrid extension):** impulsive wheel braking = instantaneous transfer of $h_i$ into body momentum through the momentum balance; landing = rigid impact. Separate note when we get there.

---

## 12. Validation checklist (analytical ↔ Simscape)

- [ ] Free swing (u = 0) from small angle: period matches $2\pi\sqrt{\bar\Theta_e/(mg\ell)}$ linearized prediction (hanging equilibrium).
- [ ] $\|\boldsymbol g_B\| = g$ invariant preserved by the integrator (drift < 0.1% over 60 s).
- [ ] Energy conservation with $u=0$, no friction (drift < 0.1% over 60 s, tighten solver tolerances if violated).
- [ ] Linearization: Simscape Model Linearizer $A, B$ vs. analytical $A_{\mathrm{lin}}, B_{\mathrm{lin}}$ — entrywise agreement to ~1%.
- [ ] `ctrb` rank: expect rank deficiency exactly matching the norm-constraint direction + yaw symmetry, nothing else.
- [ ] 1D LQR gains ported to the 3D model restricted to one axis reproduce 1D closed-loop response.

---

## References for this derivation
- Gajamohan, Muehlebach, Widmer, D'Andrea — *The Cubli: A Reaction Wheel-based 3D Inverted Pendulum*, ECC 2013. (Model structure, hardware parameters.)
- Muehlebach, D'Andrea — *Nonlinear Analysis and Control of a Reaction-Wheel-Based 3-D Inverted Pendulum*, IEEE TCST 2016. (Nonlinear control on this model.)
- Ribas Martins, da Silva — arXiv:2009.14625. (Quaternion/vector Lagrangian formalism.)
- See [[References]] for links and [[State of the Art Controllers]] for what to build on top of these equations.


---

# Validity audit — re-checked 2026-07-22

Re-derived every step after the hardware situation firmed up (EnduroSat-provided motors/drivers, Pennings reference build, team scope of balance + jump + locomotion).

## Verdict: the results are correct. Validity is conditional on three assumptions — one of which is not yet confirmed.

## What was re-verified and holds

- **Kinetic energy expansion (§4.2).** The cross-term collapses to $I_w\dot\varphi_i\,\boldsymbol a_i^\top\boldsymbol\omega$ because $\boldsymbol a_i^\top I^{\mathrm{COM}}_{w,i} = I_w\boldsymbol a_i^\top$ (the $J_w$ part is annihilated by $(\mathbb 1 - \boldsymbol a_i\boldsymbol a_i^\top)\boldsymbol a_i = 0$). Confirmed.
- **Wheel equation (§7.1).** $\partial L/\partial\dot\varphi_i = I_w(\dot\varphi_i + \boldsymbol a_i^\top\boldsymbol\omega) = h_i$, and $\varphi_i$ is cyclic, so $\dot h_i = u_i$. Confirmed.
- **Momentum rewrite (§7.2).** $\boldsymbol p = \hat\Theta\boldsymbol\omega + \sum_i I_w\dot\varphi_i\boldsymbol a_i = \bar\Theta\boldsymbol\omega + A\boldsymbol h$ with $\bar\Theta = \hat\Theta - I_w\sum_i\boldsymbol a_i\boldsymbol a_i^\top$. Confirmed.
- **Body equation, 1D specialisation, equilibria, linearised $A_{\mathrm{lin}}/B_{\mathrm{lin}}$.** All confirmed. The 1D cross-check in §8 closes both ways.

## New identity worth adding — free sanity check on $\bar\Theta$

Substituting $\hat\Theta$ into $\bar\Theta = \hat\Theta - I_w\sum_i\boldsymbol a_i\boldsymbol a_i^\top$, the $I_w\boldsymbol a_i\boldsymbol a_i^\top$ terms **cancel exactly**:

$$\bar\Theta = \Theta_{\mathcal S} + \sum_{i=1}^{3}\Big[m_w\big(\|\boldsymbol r_i\|^2\mathbb 1 - \boldsymbol r_i\boldsymbol r_i^\top\big) + J_w\big(\mathbb 1 - \boldsymbol a_i\boldsymbol a_i^\top\big)\Big]$$

So **$\bar\Theta$ contains no $I_w$ at all** — it is the structure plus the wheels treated as non-spinning bodies carrying only their *transverse* inertia. Physically sensible (the axial inertia is exactly the part that decouples into the wheel states), and a free numerical check: compute $\bar\Theta$ both ways in `cubli_params.m` and they must agree to machine precision.

## Condition 1 — ⚠️ torque-mode drivers (NOT YET CONFIRMED)

$\dot h_i = u_i$ assumes $u_i$ is a **commanded torque**. This is the open question from [[Revised Schedule - Provided Hardware]] §3 and [[Pennings Reference & Component Checklist]].

**If the drivers are speed-mode**, the model genuinely changes. The wheel speed becomes a directly-commanded quantity, the effective input is $v_i \equiv \ddot\varphi_i$, and the body equation becomes

$$\hat\Theta\,\dot{\boldsymbol\omega} = -\,\boldsymbol\omega\times\boldsymbol p + m\,\boldsymbol c\times\boldsymbol g_B - A\boldsymbol v, \qquad \boldsymbol p = \hat\Theta\boldsymbol\omega + \sum_i I_w\dot\varphi_i\boldsymbol a_i$$

Note it is now $\hat\Theta$, **not** $\bar\Theta$ — the inertia matrix in the state equation is different, so gains computed from the torque-mode model would be wrong. Everything else (kinematics, gravity torque, equilibria, structure of the linearisation) carries over unchanged.

**So: confirm the driver mode tomorrow before tuning anything.** If it's speed-mode, this is a half-day re-derivation, not a rewrite — but it must happen in week 1, not week 3.

## Condition 2 — wheel geometry

The simplification $\bar\Theta = \hat\Theta - I_w\mathbb 1$ requires the three spin axes to be **orthonormal** *and* the three wheels **identical**. If the team's CAD departs from the ETH/Pennings layout — e.g. three wheels arranged symmetrically about the balancing diagonal for isotropic authority, a legitimate alternative — keep the general form:

$$\bar\Theta = \hat\Theta - \sum_i I_{w,i}\,\boldsymbol a_i\boldsymbol a_i^\top$$

The derivation was written with general $A$ throughout, so nothing breaks; only the shortcut does. `cubli_params.m` already uses the general form.

## Condition 3 — fixed frictionless point pivot

Valid for **balancing** (the 75% pass criterion) and for the continuous phases of jump-up. **Not valid across contact transitions**, which matters for both improvements:

- **Jump-up:** the derivation covers the flight/rotation phases; the impulsive brake and the landing impact are separate hybrid events (§11 item 6, still unwritten).
- **Room locomotion:** tumbling face-over-face means the pivot *changes* — a sequence of fixed-pivot problems joined by impacts. The derivation is the per-phase model, not the whole maneuver.

Also: real corner contact has friction and slight compliance. Slip is a failure mode the model doesn't predict, which is why the high-friction pad in the component checklist is a real requirement rather than a nicety.

## One thing I would tighten in the text

§7.2 states the Euler–Poincaré equation loosely, with a hand-waved "$\partial L/\partial(\text{attitude})$-terms". The clean form for a Lagrangian with an advected parameter ($\boldsymbol g_B$) is

$$\dot{\boldsymbol p} = \boldsymbol p\times\boldsymbol\omega + \frac{\partial \ell}{\partial \boldsymbol g_B}\times\boldsymbol g_B, \qquad \boldsymbol p = \frac{\partial\ell}{\partial\boldsymbol\omega}$$

With $\partial\ell/\partial\boldsymbol g_B = m\boldsymbol c$ this gives $\dot{\boldsymbol p} + \boldsymbol\omega\times\boldsymbol p = m\,\boldsymbol c\times\boldsymbol g_B$ directly — the same result, properly justified. The displayed equation in §7.2 should be replaced with this; the boxed final result is unaffected.

## What does *not* affect validity

- **Bobrow single-IMU approach** — changes the measurement model and observer, not the dynamics.
- **Pennings' build** — same topology (three wheels, corner pivot); his cube is an instance of this model, not a different system.
- **Uniform-cube inertia formulas** ($\tfrac{11}{12}ma^2$ etc.) — those live in [[Sizing Memo]], are explicitly first-pass approximations, and are replaced by CAD then measured values. They were never part of this derivation.

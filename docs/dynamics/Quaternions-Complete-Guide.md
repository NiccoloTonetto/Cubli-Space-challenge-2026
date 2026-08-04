---
tags: [quaternions, attitude, math, robotics, cubli]
---

# Quaternions — Complete Guide for Attitude Representation

Written for someone who knows rotation matrices and Euler angles from a robotics course but never worked with quaternions properly. Goal: everything needed to (a) follow the [[3D Cubli Lagrangian Derivation]], (b) implement attitude kinematics in MATLAB/Simulink correctly, (c) not fall into the classic traps. Ordered from algebra → geometry → kinematics → practice.

---

## 1. What a quaternion is

A quaternion is a 4-component object
$$q = q_0 + q_1 i + q_2 j + q_3 k \;\equiv\; (q_0, \boldsymbol q_v), \qquad q_0 \in \mathbb R \text{ (scalar part)},\; \boldsymbol q_v = (q_1, q_2, q_3)^\top \in \mathbb R^3 \text{ (vector part)},$$
where the imaginary units satisfy Hamilton's relations:
$$i^2 = j^2 = k^2 = ijk = -1,$$
which imply $ij = k$, $jk = i$, $ki = j$, and **anti-commutativity**: $ji = -k$, etc.

Think of quaternions as the 4D generalization of complex numbers. And that analogy is not decorative — it's the whole point:

> **Unit complex numbers $e^{i\theta}$ represent rotations in 2D. Unit quaternions represent rotations in 3D.**

This is exactly why Ribas Martins & da Silva derive the 1D Cubli with unit complex numbers first, then the 3D Cubli with quaternions: same machinery, one dimension up.

---

## 2. Algebra you actually need

### 2.1 Quaternion product

For $q = (q_0, \boldsymbol q_v)$ and $p = (p_0, \boldsymbol p_v)$, expanding Hamilton's relations gives the compact vector form worth memorizing:
$$\boxed{\;q \otimes p = \big(q_0 p_0 - \boldsymbol q_v^\top \boldsymbol p_v,\;\; q_0\boldsymbol p_v + p_0 \boldsymbol q_v + \boldsymbol q_v \times \boldsymbol p_v\big)\;}$$

Key properties:
- **Not commutative**: $q\otimes p \neq p\otimes q$ in general (the $\times$ term flips sign). This is a feature, not a bug — 3D rotations don't commute either, and the quaternion product inherits exactly that non-commutativity.
- Associative: $(q\otimes p)\otimes r = q\otimes(p\otimes r)$. ✓
- Bilinear in both arguments.

Matrix form (useful for Jacobians and Simulink implementation): $q\otimes p = [q]_L\, p = [p]_R\, q$ where
$$[q]_L = \begin{bmatrix} q_0 & -\boldsymbol q_v^\top \\ \boldsymbol q_v & q_0\mathbb 1 + \hat{\boldsymbol q}_v \end{bmatrix}, \qquad
[p]_R = \begin{bmatrix} p_0 & -\boldsymbol p_v^\top \\ \boldsymbol p_v & p_0\mathbb 1 - \hat{\boldsymbol p}_v \end{bmatrix},$$
with $\hat{(\cdot)}$ the skew map. Note the sign flip on the skew term between left and right matrices.

### 2.2 Conjugate, norm, inverse

$$q^* = (q_0, -\boldsymbol q_v), \qquad \|q\|^2 = q\otimes q^* = q_0^2 + \|\boldsymbol q_v\|^2, \qquad q^{-1} = \frac{q^*}{\|q\|^2}.$$

For a **unit quaternion** ($\|q\| = 1$): $q^{-1} = q^*$ — inversion is free. This is one reason quaternions beat rotation matrices computationally (matrix inverse = transpose is also free, but quaternion storage is 4 numbers vs 9, and renormalization is cheaper than re-orthogonalization).

### 2.3 Pure quaternions

A quaternion with zero scalar part, $v = (0, \boldsymbol v)$, is called **pure** and is how we embed 3D vectors into quaternion algebra. Products of pure quaternions encode both dot and cross product at once:
$$(0,\boldsymbol a)\otimes(0,\boldsymbol b) = (-\boldsymbol a^\top\boldsymbol b,\; \boldsymbol a\times\boldsymbol b).$$

---

## 3. Unit quaternions as rotations

### 3.1 The axis–angle connection

Any 3D rotation is a rotation by angle $\theta$ about a unit axis $\boldsymbol n$ (Euler's theorem). The corresponding unit quaternion is
$$\boxed{\;q = \Big(\cos\tfrac\theta2,\; \boldsymbol n\sin\tfrac\theta2\Big)\;}$$

**The half angle is not a typo.** It's the single most important structural fact about quaternions, and the reason for both their power and their one quirk (double cover, §3.4). Compare with 2D: $e^{i\theta}$ rotates by $\theta$; the quaternion "lives on" $\theta/2$ because a vector gets sandwiched between $q$ and $q^*$, picking up the rotation twice.

Sanity checks:
- $\theta = 0$: $q = (1, \boldsymbol 0)$ — the identity quaternion. ✓
- $\theta = 180°$ about $z$: $q = (0, 0, 0, 1)$ — a pure quaternion. ✓

### 3.2 Rotating a vector

To rotate vector $\boldsymbol v$ by the rotation encoded in unit quaternion $q$:
$$\boxed{\;(0, \boldsymbol v') = q \otimes (0, \boldsymbol v) \otimes q^*\;}$$
The result is automatically pure (the scalar part cancels — worth verifying once by hand). Expanding gives the rotation-matrix equivalent:
$$\boldsymbol v' = R(q)\,\boldsymbol v, \qquad R(q) = (q_0^2 - \boldsymbol q_v^\top\boldsymbol q_v)\,\mathbb 1 + 2\,\boldsymbol q_v\boldsymbol q_v^\top + 2 q_0\,\hat{\boldsymbol q}_v.$$

Written out (the form you'll type into MATLAB exactly once and then wrap in a function):
$$R(q) = \begin{bmatrix}
1-2(q_2^2+q_3^2) & 2(q_1q_2 - q_0q_3) & 2(q_1q_3 + q_0q_2)\\
2(q_1q_2 + q_0q_3) & 1-2(q_1^2+q_3^2) & 2(q_2q_3 - q_0q_1)\\
2(q_1q_3 - q_0q_2) & 2(q_2q_3 + q_0q_1) & 1-2(q_1^2+q_2^2)
\end{bmatrix}$$

### 3.3 Composition

Rotating first by $q_1$ then by $q_2$ composes as
$$q_{\mathrm{total}} = q_2 \otimes q_1$$
(same order convention as rotation matrices, $R_2 R_1$). Verify: $q_2\otimes(q_1\otimes v\otimes q_1^*)\otimes q_2^* = (q_2\otimes q_1)\otimes v\otimes(q_2\otimes q_1)^*$, using $(q_2\otimes q_1)^* = q_1^*\otimes q_2^*$.

### 3.4 The double cover — $q$ and $-q$ are the same rotation

From the half-angle formula: replacing $\theta \to \theta + 2\pi$ gives $q \to -q$ but obviously the same physical rotation. So:
$$R(q) = R(-q).$$
The unit quaternions form the 3-sphere $S^3$, which wraps around the rotation group $SO(3)$ **twice** ("double cover").

Practical consequences:
1. **Attitude checks:** "is the attitude close to target?" must test $|q_0^{\mathrm{err}}|\approx 1$, not $q^{\mathrm{err}} \approx (1,\boldsymbol 0)$ — the error could legitimately be near $(-1, \boldsymbol 0)$.
2. **The unwinding problem in control:** naive quaternion feedback can drive the attitude the *long way around* (359° instead of −1°) when $q_0^{\mathrm{err}} < 0$. Standard fix: multiply the feedback by $\mathrm{sign}(q_0^{\mathrm{err}})$. For Cubli corner balancing you stay near one equilibrium so it rarely bites, but the check costs one line — include it.
3. **Interpolation/filtering/averaging:** before combining two quaternions, flip the sign of one if their dot product is negative.

### 3.5 Why not Euler angles?

You know this from the robotics course but it's worth making crisp, because it's *the* justification for the quaternion overhead:

| | Euler angles | Rotation matrix | Quaternion |
|---|---|---|---|
| Parameters | 3 | 9 | 4 |
| Constraints | 0 | 6 (orthonormality) | 1 (unit norm) |
| Singularities | **Yes — gimbal lock** | No | **No** |
| Composition cost | trig-heavy | 27 mult | 16 mult |
| Renormalization | — | Gram–Schmidt (awkward) | divide by norm (trivial) |
| Interpolation | poor | poor | SLERP (clean) |

Gimbal lock is not a numerical nuisance, it's a topological fact: no 3-parameter chart covers $SO(3)$ without singularities. For the Cubli, the corner-balancing equilibrium can be *placed* away from any given Euler singularity, so Euler angles would *work* — but the 3D derivation becomes a trigonometric swamp, and any large-angle maneuver (jump-up!) risks passing near the singularity. Quaternions cost one extra state and one norm constraint, and buy global, singularity-free, trig-free dynamics. That trade is why ECC 2013, TCST 2016, Ribas Martins, and every modern satellite ADCS (including what you'd fly on ASTERIA) use quaternions.

---

## 4. Kinematics — the equation that matters for the Cubli

### 4.1 Derivation of $\dot q$

Let $q(t)$ be the attitude (body→inertial) and $\boldsymbol\omega$ the angular velocity **in the body frame**. Over a small time $dt$ the body rotates by axis $\boldsymbol\omega/\|\boldsymbol\omega\|$, angle $\|\boldsymbol\omega\|dt$, so with the half-angle formula the increment quaternion is, to first order,
$$dq = \Big(1, \tfrac12\boldsymbol\omega\,dt\Big).$$
Since the increment is expressed in the **body** frame, it composes on the **right**:
$$q(t+dt) = q(t)\otimes dq \;\Rightarrow\; \boxed{\;\dot q = \tfrac12\, q\otimes(0,\boldsymbol\omega)\;}$$

This is the equation in §3 of the [[3D Cubli Lagrangian Derivation]]. If $\boldsymbol\omega$ were expressed in the inertial frame, it would compose on the left: $\dot q = \tfrac12 (0,\boldsymbol\omega_I)\otimes q$. **Getting body-vs-inertial and left-vs-right consistent is the #1 source of sign bugs in attitude code.** Pick one convention (body-frame $\boldsymbol\omega$, right multiplication — matches our Cubli note and most robotics literature) and never mix.

In matrix form for implementation:
$$\dot q = \tfrac12\,\Omega(\boldsymbol\omega)\,q, \qquad \Omega(\boldsymbol\omega) = \begin{bmatrix} 0 & -\boldsymbol\omega^\top \\ \boldsymbol\omega & -\hat{\boldsymbol\omega} \end{bmatrix}.$$

### 4.2 Norm preservation and numerical integration

Analytically, $\frac{d}{dt}\|q\|^2 = 2q^\top\dot q = q^\top\Omega q = 0$ ($\Omega$ is skew-symmetric) — the kinematics preserve the unit norm exactly. Numerical integrators do **not**: `ode45` will let $\|q\|$ drift.

Handling, from crude to correct:
1. **Brute-force renormalization** $q \leftarrow q/\|q\|$ every step (or at each filter update). Fine for simulation; what almost everyone does.
2. **Exact discrete propagation** for constant $\boldsymbol\omega$ over a step $\Delta t$:
$$q_{k+1} = q_k \otimes \Big(\cos\tfrac{\|\boldsymbol\omega\|\Delta t}{2},\; \tfrac{\boldsymbol\omega}{\|\boldsymbol\omega\|}\sin\tfrac{\|\boldsymbol\omega\|\Delta t}{2}\Big).$$
Norm-preserving by construction; the right choice inside a fixed-step Simulink attitude propagator or an onboard filter.
3. In Simscape Multibody you don't integrate $q$ yourself — the engine handles it — but the analytical model you validate against does, so use 1 or 2 there.

**Simulation health check** (already in the Cubli validation checklist): $|\,\|q\|-1\,| < 10^{-6}$ throughout; tighten `RelTol/AbsTol` if violated.

### 4.3 Attitude error quaternion

For control you need "how far is current attitude $q$ from desired $q_d$":
$$q_{\mathrm{err}} = q_d^{*}\otimes q = (q_{e0}, \boldsymbol q_{ev}).$$
For small errors, $q_{e0}\approx 1$ and $\boldsymbol q_{ev} \approx \tfrac12\,\boldsymbol\delta$ where $\boldsymbol\delta$ is the small rotation vector — so **$2\boldsymbol q_{ev}$ is the attitude error "vector"** you feed to a linear controller. (Factor of 2 again from the half angle. Forgetting it just rescales your gains, but be consistent.)

This is why the linearized Cubli model can use a 3-component attitude perturbation: near equilibrium, the quaternion vector part *is* (half) the small rotation vector. The reduced $\boldsymbol g_B$ formulation in the Cubli note sidesteps quaternions for the same small-signal purpose — the two views are consistent.

### 4.4 Relationship to the $\boldsymbol g_B$ trick

In the Cubli derivation we propagate $\dot{\boldsymbol g}_B = \boldsymbol g_B\times\boldsymbol\omega$ instead of (or alongside) $\dot q$. Connection: $\boldsymbol g_B = -g\,R(q)^\top \boldsymbol e_z$ — $\boldsymbol g_B$ is the part of the attitude that gravity can "see." It loses yaw (rotation about vertical) but needs no normalization beyond $\|\boldsymbol g_B\| = g$ and gives the smallest possible state for balancing control. Full quaternion: keep when you care about full attitude (jump-up, yaw regulation, Simscape ground truth). $\boldsymbol g_B$: enough for pure corner balancing.

---

## 5. Conversions (implement once, unit-test, trust forever)

### Quaternion → rotation matrix
Given in §3.2. MATLAB: `quat2rotm` — but **check its convention** (MATLAB uses $q = [w\,x\,y\,z]$ scalar-first, Hamilton convention, consistent with ours).

### Rotation matrix → quaternion (Shepperd's method)
Naive extraction $q_0 = \tfrac12\sqrt{1+\mathrm{tr}R}$ is numerically bad when $\mathrm{tr}R \to -1$ (rotations near 180°). Robust approach: compute all four candidate pivots $\big(1{+}\mathrm{tr}R,\; 1{+}2R_{11}{-}\mathrm{tr}R,\; \ldots\big)$, take the largest, recover the rest from off-diagonal sums/differences. `rotm2quat` does this internally — use it, but know why.

### Axis–angle ↔ quaternion
§3.1 forward; inverse: $\theta = 2\,\mathrm{atan2}(\|\boldsymbol q_v\|, q_0)$, $\boldsymbol n = \boldsymbol q_v/\|\boldsymbol q_v\|$ (undefined at $\theta = 0$; return any axis). Use `atan2`, never `acos` — better conditioning near 0 and π.

### Euler angles ↔ quaternion
Compose the three elemental quaternions in the chosen sequence, e.g. ZYX: $q = q_z(\psi)\otimes q_y(\theta)\otimes q_x(\phi)$ with $q_z(\psi) = (\cos\tfrac\psi2, 0,0,\sin\tfrac\psi2)$ etc. Only needed at interfaces (human-readable I/O, comparing against Euler-based results) — never inside the dynamics.

---

## 6. SLERP — interpolation (for trajectory generation)

To interpolate smoothly from $q_1$ to $q_2$ with parameter $s\in[0,1]$:
$$\mathrm{slerp}(q_1, q_2, s) = q_1\otimes\big(q_1^*\otimes q_2\big)^s, \qquad p^s \equiv \Big(\cos\tfrac{s\theta}2,\;\boldsymbol n\sin\tfrac{s\theta}2\Big)\text{ for } p = \big(\cos\tfrac\theta2, \boldsymbol n\sin\tfrac\theta2\big),$$
i.e. constant-angular-velocity motion along the geodesic. Remember the double-cover rule: if $q_1^\top q_2 < 0$, flip $q_2 \to -q_2$ first, or you'll interpolate the long way around. Relevant if we ever generate reference attitude trajectories (e.g. edge→corner transition profiles); not needed for pure stabilization.

---

## 7. Conventions — read before touching anyone else's code

Attitude code from different sources silently disagrees on four independent choices. Any mismatch = sign bugs that "almost work":

1. **Scalar first $(w,x,y,z)$ vs scalar last $(x,y,z,w)$.** MATLAB, Eigen: scalar first. ROS, SciPy, many game engines: scalar last.
2. **Hamilton ($ij = k$) vs JPL ($ij = -k$) convention.** Almost everything modern is Hamilton; older aerospace/JPL filter literature (including some Kalman filter papers you might read for IMU fusion!) is JPL. A JPL-convention formula pasted into Hamilton code flips the rotation direction.
3. **Active vs passive** (rotate the vector vs rotate the frame): $R$ vs $R^\top$.
4. **Body→inertial vs inertial→body** as "the" attitude quaternion.

**Our convention stack (matches [[3D Cubli Lagrangian Derivation]] and MATLAB):** Hamilton, scalar-first, $q$ maps body→inertial, $\boldsymbol\omega$ in body frame, kinematics $\dot q = \tfrac12 q\otimes(0,\boldsymbol\omega)$ (right multiplication). Write this at the top of every script.

---

## 8. Worked micro-examples (do these by hand once)

1. **90° about $z$ applied to $\boldsymbol e_x$:** $q = (\tfrac{\sqrt2}2, 0, 0, \tfrac{\sqrt2}2)$; compute $q\otimes(0,\boldsymbol e_x)\otimes q^*$ and verify you get $(0, \boldsymbol e_y)$.
2. **Non-commutativity:** 90° about $x$ then 90° about $z$, vs the reverse order. Compute both $q_z\otimes q_x$ and $q_x\otimes q_z$, convert to axis–angle, confirm they differ — and match what you get physically rotating a book.
3. **Double cover:** verify $R(q) = R(-q)$ by plugging $-q$ into the §3.2 matrix (every entry is quadratic in $q$ — that's the proof).
4. **Kinematics sanity:** constant $\boldsymbol\omega = (0,0,\omega_z)$, initial $q=(1,\boldsymbol 0)$: solve $\dot q = \tfrac12 q\otimes(0,\boldsymbol\omega)$ analytically → $q(t) = (\cos\tfrac{\omega_z t}2, 0,0,\sin\tfrac{\omega_z t}2)$. The half angle appearing in the solution = everything consistent.

---

## 9. Cheat sheet

$$
\begin{aligned}
&q\otimes p = (q_0p_0 - \boldsymbol q_v^\top\boldsymbol p_v,\; q_0\boldsymbol p_v + p_0\boldsymbol q_v + \boldsymbol q_v\times\boldsymbol p_v) && \text{product}\\
&q = (\cos\tfrac\theta2, \boldsymbol n\sin\tfrac\theta2) && \text{axis–angle}\\
&(0,\boldsymbol v') = q\otimes(0,\boldsymbol v)\otimes q^* && \text{rotate vector}\\
&q_{\mathrm{tot}} = q_2\otimes q_1 && \text{compose (1 then 2)}\\
&\dot q = \tfrac12\,q\otimes(0,\boldsymbol\omega_B) && \text{kinematics, body }\boldsymbol\omega\\
&q_{\mathrm{err}} = q_d^*\otimes q,\quad \text{error vec} \approx 2\boldsymbol q_{ev} && \text{control error}\\
&q \text{ and } -q: \text{same rotation} && \text{double cover}\\
&q^{-1} = q^* \text{ iff } \|q\|=1 && \text{cheap inverse}
\end{aligned}
$$

## Where this plugs into the project
- [[3D Cubli Lagrangian Derivation]] §3 uses $\dot q = \tfrac12 q\otimes(0,\boldsymbol\omega)$ and the $\boldsymbol g_B$ reduction (§4.4 here).
- The Kalman filter for IMU fusion will use the error-quaternion small-angle state (§4.3) — that's the "multiplicative EKF" structure, worth a dedicated note when we get there.
- ASTERIA ADCS uses the identical formalism — this note doubles as GNC reference material.

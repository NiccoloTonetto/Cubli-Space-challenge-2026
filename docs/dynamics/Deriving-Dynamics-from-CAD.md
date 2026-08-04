---
tags: [space-challenge, sofia, cubli, cad, dynamics, controls, interface]
---

# Deriving the Dynamics from CAD — the CAD↔Controls interface

How the structural team's CAD becomes the numbers in the equations of motion. This is an **interface document**: it defines exactly what Structure delivers, in what frame, with what conventions, and how Controls validates it against reality.

Companion file: `cubli_params.m` — the single source of truth, with the contract encoded and sanity checks that fire on convention errors.
Model reference: [[3D Cubli Lagrangian Derivation]] §2, §4.3, §7.

---

## 0. What the model actually needs

From the derivation, the EOM need exactly these quantities — nothing more:

| Symbol | Meaning | Who provides |
|---|---|---|
| $m$ | total mass | CAD → measured |
| $\boldsymbol c$ | total COM position in body frame | CAD → measured |
| $\Theta_{\mathcal S}$ | **structure** inertia tensor about pivot $O$ | CAD |
| $m_w$, $\boldsymbol r_i$ | wheel mass, wheel COM positions | CAD → measured |
| $I_w$, $J_w$ | wheel inertia about spin axis / transverse | CAD → measured |
| $\boldsymbol a_i$ | wheel spin axes | design (frame definition) |

Then $\hat\Theta$ (locked inertia) and $\bar\Theta$ are **computed**, not exported — `cubli_params.m` does that assembly. Structure should never hand over "the assembly inertia" as one number, because…

**⚠️ The single most important point on this page:** the wheels must be **excluded** from the structure's tensor and delivered separately. Our model relies on the wheels being axisymmetric and treated as separate spinning bodies (§1 assumption 2 of the derivation). If CAD exports one lumped inertia for the whole assembly with wheels frozen in place, that's $\hat\Theta$ — usable for the locked case only, and **it silently destroys the rotor-coupling term that is the entire control mechanism.** Ask for two exports, always: structure-with-wheels-suppressed, and one wheel alone.

Note on the split: motor **stators** go with the structure; motor **rotors** go with the wheels (fold rotor inertia into $I_w$).

---

## 1. Frame definition — agree on this before anyone models anything

Non-negotiable, identical in CAD, Simscape, MATLAB and firmware:

- **Origin:** the pivot corner of the cube.
- **Axes:** $x,y,z$ along the three cube edges emanating from that corner. Right-handed.
- **Upright equilibrium:** body diagonal $(1,1,1)/\sqrt3$ points up.
- **Units:** metres, kilograms, kg·m². (CAD defaults to mm and grams — a factor of $10^{-9}$ in the tensor. This *will* happen at least once.)

In Fusion/SolidWorks this means creating an explicit coordinate system at the pivot corner and reporting mass properties **relative to that**, not relative to the default origin or the COM. Both tools will happily report about three different points; the export must say which.

---

## 2. Getting the numbers out of CAD

### 2.1 SolidWorks
`Evaluate → Mass Properties`. Set "Output coordinate system" to the pivot-corner CSYS. It reports mass, COM, and two tensors: *at the centre of mass* and *at the output coordinate system*. Take the latter (or take the COM one and apply §2.4 yourself as a cross-check — doing both once is a good validation).

**Sign convention trap.** SolidWorks reports "Moments of inertia taken at the output coordinate system" as $L_{xx}, L_{xy}, \ldots$ arranged as a matrix — that is already the **tensor**, so off-diagonals are $-\int xy\,dm$. It *also* reports "Products of inertia" elsewhere, which are $+\int xy\,dm$. Pasting the wrong one flips off-diagonal signs, which produces a model that looks plausible, passes casual inspection, and gives subtly wrong coupling. The assertion in `cubli_params.m` catches gross cases (non-positive-definite), but not all sign flips — so state explicitly in the export which one was copied.

### 2.2 Fusion 360
`Properties` panel → Physical. Gives mass, COM, and moments/products of inertia. Fusion reports about the COM and about the origin; again, be explicit. Fusion's products of inertia follow the $+\int xy\,dm$ convention, so **negate them** when building the tensor. Set the document units to metric before reading.

### 2.3 Material assignment — where most of the error lives
- **Assign real materials to everything.** Default "Steel" on a printed PLA bracket is a 6× density error.
- **3D printed parts are not solid.** At 20% infill the effective density is roughly $0.2\rho_{\text{solid}} + \text{shell}$. Do not compute this — **print a test coupon of known volume, weigh it, back out effective density, and put that number into CAD as a custom material.** This is a 20-minute job that removes the largest single source of CAD error.
- **Override masses for bought parts.** Motors, battery, PCBs, connectors are modelled as simplified blocks. Weigh each on a scale and use the mass-override feature so CAD uses the true mass with the block's geometry. A battery modelled as a uniform box at the right mass is fine; at the wrong mass it poisons both $m$ and $\boldsymbol c$.
- **Don't forget:** cables, fasteners, adhesive, zip ties. Typically 3–8% of total mass and often asymmetric — which matters for $\boldsymbol c$ far more than for $\Theta$ (see the momentum-saturation note in [[Sizing Memo]] §5).

### 2.4 Transformations you may need to apply

Moving a tensor from COM to the pivot (parallel axis / Steiner), with $\boldsymbol d$ the vector from the new point to the COM:
$$\Theta_O = \Theta_{\mathrm{COM}} + m\big(\|\boldsymbol d\|^2\mathbb 1 - \boldsymbol d\boldsymbol d^\top\big)$$

Rotating a tensor into the body frame with rotation $R$ (CAD frame → body frame):
$$\Theta_B = R\,\Theta_{\mathrm{CAD}}\,R^\top$$

Both are implemented in `cubli_params.m` for the wheels; do the structure one in CAD if possible (fewer hand-transcription errors).

---

## 3. Validating the CAD numbers before you trust them

Cheap checks, in order. Run all of them — each catches a different class of error.

1. **Symmetry:** $\Theta = \Theta^\top$. Trivially true if exported correctly; fails if entries were transcribed into the wrong slots.
2. **Positive definiteness:** all eigenvalues $> 0$. Catches most sign errors.
3. **Triangle inequality on principal moments:** $I_1 + I_2 \ge I_3$ for any ordering. This is a *necessary physical condition* for a real rigid body — no mass distribution can violate it. Catches unit errors and bad products of inertia that survive check 2. (Both 2 and 3 are asserted in `cubli_params.m`.)
4. **Order-of-magnitude:** for a roughly-uniform cube, $\Theta_{\text{corner,perp}} \approx \tfrac{11}{12}ma^2$ and $\Theta_{\text{edge}} \approx \tfrac23 ma^2$. If CAD differs by more than ~30% from these, either the mass is very unevenly distributed (plausible for a real Cubli — mass sits near the faces, so expect CAD *lower*) or something is wrong. A factor of 2+ means investigate.
5. **COM plausibility:** $\boldsymbol c$ should be near the geometric centre, i.e. near $(a/2, a/2, a/2)$. A large offset means an asymmetry that will eat wheel momentum — flag it to Structure *early*, when it's still cheap to move the battery.

---

## 4. Then measure it on the real hardware

**CAD is an estimate. Measured is truth.** Expect 5–15% discrepancy even with careful modelling; more if printed parts are involved. Final gains get tuned on measured values. Four measurements, in increasing order of effort:

### 4.1 Mass — scale. Do it for every subassembly, not just the total.

### 4.2 COM — two-orientation balance
Balance the cube on a knife edge (or a thin rod) in two different orientations; the COM lies on the vertical plane through the edge each time; intersect. Alternatively, three-point scale method: support on three load cells / three scales at known positions, COM is the weighted centroid of the reactions. 10 minutes, and it directly checks the number that drives momentum saturation.

### 4.3 Inertia about the pivot — free-swing period ★ the best one
Hang the assembled cube from the actual pivot corner (or rest it on the pivot with the COM below), displace slightly, and time small oscillations. For small angles:
$$T = 2\pi\sqrt{\frac{\hat\Theta}{m\,g\,\ell}} \quad\Longrightarrow\quad \hat\Theta = \frac{m g \ell\,T^2}{4\pi^2}$$

Why this is the best measurement available: it measures **exactly the quantity the model needs, about exactly the right point, in the fully assembled configuration** — cables, battery, fasteners and all. No decomposition, no transformation, no trust in material densities. Time 20 periods and divide, to beat down timing error.

Do this about at least two different tilt axes to populate more than one direction of the tensor. The decay envelope additionally gives you pivot friction, which the model currently ignores.

### 4.4 Wheel inertia — spin-down / spin-up test
- **Spin-up:** command a known constant torque $\tau$, log $\dot\varphi(t)$, fit the slope: $I_w = \tau/\alpha$. Needs a trustworthy torque command (see the driver-mode question in [[Revised Schedule - Provided Hardware]] §3) — which makes this a *dual-purpose* test: it also verifies the driver really is in torque mode.
- **Spin-down:** cut power, log the decay. Fits viscous + Coulomb friction: $I_w\dot\omega = -b\omega - \tau_c\,\mathrm{sign}(\omega)$. Those two numbers go straight into `cubli_params.m`.

Do these on the 1D jig in week 1 — they're the fastest path to a parameter set you can actually trust.

### 4.5 Optional: bifilar pendulum for individual parts
For a single part suspended on two parallel wires of length $L$, separation $2d$:
$$I = \frac{m g d^2 T^2}{4\pi^2 L}$$
Useful for characterising a wheel on its own before assembly. Nice-to-have, not on the critical path.

---

## 5. Reconciliation procedure

When measured and CAD disagree (they will):

1. Check mass first — if total mass is off, everything else is too. Usually an unmodelled part or wrong printed-part density.
2. Check COM second — usually cables or battery placement.
3. Only then look at the tensor. If mass and COM match but $\hat\Theta$ is off by >10%, the mass *distribution* is wrong: a dense part is modelled in the wrong place, or infill density is off.
4. **Update CAD to match reality**, don't just override the params file — Structure needs the corrected model for v2 design decisions.
5. Bump `p.rev` and set `p.source = 'meas'`. Re-run the sizing script; re-derive gains.

---

## 6. Getting it into Simscape

Two routes. **I recommend the second.**

**(a) Full CAD import.** Simscape Multibody Link plugin (SolidWorks/Inventor/Creo — *not* Fusion 360) exports an XML + STEP set, imported with `smimport`. Preserves the whole assembly tree automatically.
*Downside:* hundreds of parts, slow simulation, joint frames often misaligned on import, and the model is painful to parameterise or sweep. Debugging an import artifact in week 3 is exactly the kind of time sink that costs the pass criterion.

**(b) Hand-built parameterised Simscape model, fed by `cubli_params.m`.** Four `Solid` blocks (structure + 3 wheels) with **explicit inertia entered from the CAD numbers**, a 6-DOF or spherical joint at the pivot, three revolute joints for the wheels. Everything driven from the params struct.
*Advantages:* fast, sweepable, every parameter visible and versioned, and it matches the analytical model term-for-term — which is what makes the §12 validation checklist in the derivation meaningful. Also works regardless of which CAD tool Structure uses (important if it's Fusion, which has no Link plugin).

Use the STEP file only for **visualisation** (attach as a graphical shape to the Solid blocks) so the animation looks like the real cube while the physics comes from clean, explicit parameters.

---

## 7. The deliverable — what Structure hands Controls

Give them this table to fill. One row per revision.

| Field | Symbol | Units | Frame/point |
|---|---|---|---|
| Structure mass (wheels excluded) | $m_{\mathcal S}$ | kg | — |
| Structure COM | — | m | body frame, origin at pivot |
| Structure inertia tensor | $\Theta_{\mathcal S}$ | kg·m² | **about pivot corner**, body frame |
| Wheel mass (one) | $m_w$ | kg | — |
| Wheel inertia, spin axis | $I_w$ | kg·m² | about wheel COM |
| Wheel inertia, transverse | $J_w$ | kg·m² | about wheel COM |
| Wheel COM positions (×3) | $\boldsymbol r_i$ | m | body frame |
| Cube edge length | $a$ | m | — |
| Convention statement | — | — | "off-diagonals are $-\int xy\,dm$" ✔ |

Plus: which CAD tool, which material densities used, which parts had mass overrides, and whether printed-part density was measured or assumed.

**Cadence:** v0 estimate day 2 (so Controls is unblocked immediately — placeholder numbers are fine, the architecture doesn't care), v1 after the first print, `meas` after assembly. Never let Controls wait on CAD; let it re-tune when better numbers land.

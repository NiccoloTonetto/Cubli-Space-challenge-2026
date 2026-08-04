---
tags: [space-challenge, sofia, cubli, cad, handoff, bom]
---

# CAD Handoff — fill in when BOM arrives

Purpose: the moment the BOM lands, this note turns into CAD's starting brief in minutes. Numbers are **blank slots** to fill after the sizing script runs on real inputs and the team picks the envelope. Everything that *doesn't* depend on a number is already here and final.

Sizing tool: `cubli_sizing.m` in `Documents/MATLAB/Space Challenge 2026` (run it with real inputs → read the sweep → the team chooses).
Companion: [[Deriving Dynamics from CAD]] (export format), [[Structural Constraints]], [[Week 1 Plan]].

---

## STEP 1 — three questions that gate scope (answer first, from BOM/mentor)

CAD is technically unblocked before these, but the *jump-up scope* and *component list* depend on them:

- [ ] **Driver control mode** — torque/current (FOC) or throttle ESC? → decides if the control architecture stands or needs re-derivation.
- [ ] **Encoders present in BOM?** → if not, flag immediately (sensorless can't balance at zero speed).
- [ ] **Battery 4S or 6S?** → sets wheel momentum (±50%), which sets jump-up margin.

---

## STEP 2 — lock the envelope (fill after team decides)

Run `cubli_sizing.m` with real BOM numbers, read the sweep, let the team choose. Then fill:

- **Cube side length:** ________ mm   *(recommendation going in: ~120 mm — smaller improves hold/catch/jump together; floor ~110 mm set by wheel radius room)*
- **Max total mass:** ________ kg   *(target ~0.9 kg)*
- **Recovery angle:** ________ deg   *(going in: 10°)*
- **Max (mass × COM) budget:** ________ kg·m   *(from sizing script §2)*
- **Jump-up in scope?** yes / no   *(depends on jump margin ≥ ×1.3 in the sweep)*

---

## STEP 3 — wheel targets (fill after envelope)

- **Wheel outer radius:** ________ mm   *(as large as the face allows — bigger = mass-efficient momentum)*
- **Wheel mass each:** ________ g   *(≈55–80 g; ring/rim-weighted)*
- **Hub requirement:** must allow **adding rim mass later** (start light for the pass, bolt on rings if jump-up margin needs it).

---

## STEP 4 — CAD rules (final, number-independent — apply regardless)

These don't wait on the BOM. Non-negotiable:

1. **Sensors in CAD from v0.** Every rigidly-attached mass — IMU, encoder + magnet/PCB, driver board, compute, wiring — goes in as a mass-and-volume placeholder from the *first* pass, not just the structural shell. Leaving them out = wrong COM and wrong inertia tensor = gains tuned against a model that doesn't match the built object.

2. **Jig first, cube second.** Design the 1D jig (one arm, one motor, one wheel, edge pivot) *before* the cube. It's printable in a day and unblocks the whole week-1 validation.

3. **Jig simplification:** keep **compute and power off-board** (bench supply + tethered Teensy) so the jig stays light. But **IMU and encoder positions must be deliberate and in-model** — their placement affects the physics being validated.

4. **Frame convention** (identical in CAD, Simscape, firmware): origin at the **pivot corner**, axes along the three cube edges from that corner, right-handed, **SI units** (metres/kg — CAD defaults to mm/g, a 10⁻⁹ tensor error waiting to happen).

5. **Print-density calibration** early: print a coupon of known volume, weigh it, back out effective density, enter as custom material. Removes the biggest single CAD mass error. Weigh every bought part (motor, battery, boards) and use mass-override.

6. **Wheels excluded from the structure tensor.** Export **two** mass-property sets: structure-with-wheels-suppressed, and one wheel alone. Motor stators → structure; rotors → wheel. (Why: the model treats wheels as separate spinning bodies; a single lumped inertia destroys the rotor-coupling term.)

7. **Fabrication must-haves in the design:** hardened pivot tip (steel insert, not printed plastic) + friction pad; heat-set inserts for fasteners; wheel guards; designed-in COM-trim shim provision; battery mounted **dead-centre on the body diagonal** (dominates COM).

---

## STEP 5 — the deliverable CAD hands back to controls

Fill this table per revision and hand to controls (feeds `cubli_params.m`). See [[Deriving Dynamics from CAD]] §7 for full detail.

| Field | Symbol | Units | Frame/point |
|---|---|---|---|
| Structure mass (wheels excluded) | m_S | kg | — |
| Structure COM | — | m | body frame, origin at pivot |
| Structure inertia tensor | Θ_S | kg·m² | **about pivot corner**, body frame |
| Wheel mass (one) | m_w | kg | — |
| Wheel inertia, spin axis | I_w | kg·m² | about wheel COM |
| Wheel inertia, transverse | J_w | kg·m² | about wheel COM |
| Wheel COM positions (×3) | r_i | m | body frame |
| Cube edge length | a | m | — |
| Convention statement | — | — | "off-diagonals are −∫xy dm" ✔ |

**Cadence:** v0 estimate ASAP (placeholder masses fine — controls just needs *a* number to start the model), real measured masses after first print/assembly.

---

## Reference: the trade-off sweep (verified MN4006, 6S, wheel r58/60g)

Why the envelope is what it is — CAD should see the reasoning, not just the number:

```
 side  mass   | hold@10 | catch@10 | jump f->e | tc   | loop
 (mm)  (g)    | cont    | peak     | margin    | ms   | Hz
 --------------------------------------------------------------
 110   700    |  15.0   |  59.9    | x2.84 OK  | 109  | 229
 110   900    |  11.6   |  42.3    | x2.22 OK  | 109  | 229
 120   700    |  13.8   |  52.4    | x2.50 OK  | 114  | 220
 120   900    |  10.7   |  38.1    | x1.95 OK  | 114  | 220
 130   900    |   9.8   |  34.7    | x1.73 OK  | 118  | 211
 150   900    |   8.5   |  29.6    | x1.40 OK  | 127  | 197
 150   1100   |   7.0   |  23.8    | x1.15 WEAK| 127  | 197
```

Pattern: **smaller improves hold, catch AND jump at once** (shorter lever arm). Only cost is a marginally faster loop the Teensy ignores. Counter-pressure: smaller face → less room for large wheels. Sweet spot ~110–120 mm.
```
```

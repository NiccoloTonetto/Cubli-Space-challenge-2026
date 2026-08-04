---
tags: [space-challenge, sofia, cubli, motor, hardware, structure, cad]
---

# Motor: T-Motor Antigravity MN4006 380KV

Reference sheet for the team. **Part 1** = what the motor is. **Part 2** = what that forces on the structure. Everything in Part 2 is a CAD input.

Driver/encoder pairing is confirmed from the BOM: **moteus-n1 + MA600 per motor** — torque-mode FOC with absolute rotor feedback, which is what the control architecture assumes.

---

## Part 1 — Specifications

### Verified (datasheet or exact derivation)

| Parameter | Value | Note |
|---|---|---|
| KV | 380 rpm/V | datasheet |
| **Torque constant $K_t$** | **0.02513 N·m/A** | $= 9.5493 / \mathrm{KV}$, exact |
| Configuration | 18N24P → **12 pole pairs** | matters for FOC commutation |
| Stator | 40 mm × 6 mm | mounting footprint |
| Shaft | 4 mm | wheel hub bore |
| Mass | **66 g** each | ×3 = 198 g |
| Battery | 4–6S LiPo | BOM supplies 6S |
| No-load speed | ~5 600 rpm (4S) → **~8 400 rpm (6S)** | $= \mathrm{KV} \times V$ |

### Estimated — must be measured or confirmed

| Parameter | Estimate | How to resolve |
|---|---|---|
| Continuous torque $\tau_{\max}$ | ~0.12 N·m | thermally limited; **measure motor temperature under load** |
| Peak torque $\tau_{\text{peak}}$ | ~0.40 N·m | ~16 A brief |
| Rotor inertia $J_{\text{rotor}}$ | ~1×10⁻⁵ kg·m² | not published; small vs. wheel, low priority |
| Phase resistance $R$ | ~0.15 Ω | **multimeter, 10 seconds** |
| Phase inductance $L$ | unknown | LCR meter; sets electrical time constant |

⚠️ **Thermal note:** these are multirotor motors, rated assuming propeller airflow. A reaction wheel provides **none**. Real continuous torque is therefore lower than any propeller-based rating suggests. Treat 0.12 N·m as provisional and watch temperature during testing.

Also: available torque falls at high speed (back-EMF), so the constant-$\tau_{\max}$ assumption is optimistic near 8400 rpm. Fine for balancing (which lives near zero wheel speed), relevant for jump-up spin-up.

---

## Part 2 — What this means for the structure

### 2.1 Mass budget

**198 g of motors** (3 × 66 g), located on three faces. This is a fixed, non-negotiable ~20% of a 1 kg cube, positioned away from the centre — so it drives both total mass **and** the inertia tensor.

### 2.2 Mounting — these are outrunners

The **base is stationary and bolts to the structure**; the **bell rotates** and carries the wheel. Consequences:

- Three motor mounts, on **three mutually orthogonal faces** meeting at the pivot corner. Orthogonality matters — misalignment appears as cross-axis coupling in the controller.
- Each mount carries the **full reaction torque** into the frame. **Printed brackets flex, and that compliance is an unmodelled resonance that caps control bandwidth.** Print for prototyping; plan for aluminium if the loop turns out twitchy.
- Verify the **bolt pattern** (standard multirotor base, likely M3) before designing brackets.
- **Leave airflow around the motors** — no tight enclosure. See the thermal note above.

### 2.3 Wheel interface

- Wheel mounts to the **bell**, at the prop interface.
- **4 mm shaft** — hub bore, if the design uses the shaft rather than the bell face.
- Wheel design targets for a ~120 mm cube: **outer radius ~55–58 mm, ~60 g each, rim-weighted (mass concentrated at the rim, not a solid disc).**
- **Design the hub so rim mass can be added later.** Start light for the pass criterion; bolt on additional rings only if measured jump-up margin requires it. This also covers the material decision — printed rings first, steel rims swapped in without touching the hub.

### 2.4 Encoder mounting — the fiddly geometric constraint

The MA600 sensor must sit **stationary**, facing a **diametric magnet on the rotating part**, with a small and *repeatable* air gap (~0.5 mm — confirm against the MA600 datasheet).

On an outrunner this is genuinely awkward and needs solving early, because it dictates how the motor attaches:

- Magnet carrier must be **non-ferrous** and **concentric** — eccentricity becomes angle error, which becomes bad commutation and torque ripple.
- Keep the sensor as far as geometry allows from the **rotor's own magnets** (the bell is full of them, millimetres away, and the sensor reads a weak field direction).
- Keep the **steel wheel rim** geometrically distant from the sensor too.
- **Slotted mounting holes** for gap adjustment — this will not be right first try.
- Verify a clean, monotonic reading over a full revolution **with the wheel installed**, not just a bare motor.

### 2.5 Cube envelope implied by these motors

The motors are **not** the binding constraint on cube size — torque headroom is generous. The binding constraint is **jump-up momentum**, which degrades with both size and mass:

| Cube | Jump margin (flat→edge) |
|---|---|
| 110 mm, 900 g | ×2.22 |
| 120 mm, 900 g | ×1.95 |
| 120 mm, ~1.3 kg | ×1.34 |
| 150 mm, ~1.3 kg | **×0.96 — fails** |

Need ×1.3+ to schedule jump-up as a deliverable.

**Design target: ≤120 mm side, mass as low as achievable.** Smaller improves hold angle, catch angle *and* jump margin simultaneously — the only cost is a marginally faster control loop, which the Teensy absorbs easily. Practical floor ~110 mm, set by the room needed for large-radius wheels.

---

## Open items

- [ ] Measure phase resistance (multimeter)
- [ ] Confirm bolt pattern for bracket design
- [ ] Confirm MA600 datasheet air gap tolerance
- [ ] Measure motor temperature under sustained load — validates the 0.12 N·m assumption
- [ ] Confirm whether the BOM supplies 4 motors (3 + spare) or exactly 3

Sizing tool: `cubli_sizing.m` in `Documents/MATLAB/Space Challenge 2026`. Companion notes: [[Structural Constraints]], [[System Integration - Full Cube]], [[CAD Handoff - fill on BOM]].

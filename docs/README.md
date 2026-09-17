# Documentation

Theory, design studies and test reports. Roughly in the order the project went
through them.

## `dynamics/` — the plant

| Document | What it covers |
|---|---|
| [`3D-Cubli-Lagrangian-Derivation.md`](dynamics/3D-Cubli-Lagrangian-Derivation.md) | Full 3D corner-balancing equations of motion, derived from scratch. §8 specialises to the 1D/2D edge case |
| [`Deriving-Dynamics-from-CAD.md`](dynamics/Deriving-Dynamics-from-CAD.md) | Turning CAD mass properties into model parameters — including what to do when print density is unknown |
| [`Quaternions-Complete-Guide.md`](dynamics/Quaternions-Complete-Guide.md) | Attitude representation from first principles |
| [`Attitude-Representation-for-Firmware.md`](dynamics/Attitude-Representation-for-Firmware.md) | Why this project uses a **reduced attitude** rather than a quaternion, and what that costs |
| [`1D-Jig-to-3D-Cube-Strategy.md`](dynamics/1D-Jig-to-3D-Cube-Strategy.md) | Exactly what transfers from the 2D panel to the 3D cube, and what does not |
| [`Cube-Performance-Envelope-Results.md`](dynamics/Cube-Performance-Envelope-Results.md) | **The main simulation result.** Methodology, recovery envelopes, per-corner results, the nonlinearity budget, the estimator study, and the strut re-derivation |

## `simulation/` — the models

| Document | What it covers |
|---|---|
| [`Simulation-Strategies.md`](simulation/Simulation-Strategies.md) | Why three models rather than one, and what each is for |
| [`Simscape-Panel-Model-Build-Guide.md`](simulation/Simscape-Panel-Model-Build-Guide.md) | Building the 2D panel model block by block |
| [`Panel-Controller-Workflow.md`](simulation/Panel-Controller-Workflow.md) | Design workflow from plant to gains |
| [`Cube-Envelope-Methodology-and-Results.md`](simulation/Cube-Envelope-Methodology-and-Results.md) | Cube envelope study, source write-up |
| [`Cube-Performance-Envelope-and-Limits.md`](simulation/Cube-Performance-Envelope-and-Limits.md) | Where the limits come from |
| [`Cube-Final-Configuration-Per-Corner-Results.md`](simulation/Cube-Final-Configuration-Per-Corner-Results.md) | Final configuration, corner by corner |

### `simulation/reports/` — the four nonlinearity studies

Each takes one effect the linear design cannot see, and prices it.

| Report | Question |
|---|---|
| [Block A — Saturation Envelope](simulation/reports/Saturation-Envelope-Test-Block-A.md) | How far can it tip and still come back, with a real actuator? |
| [Block B — Discrete Loop](simulation/reports/Discrete-Loop-Test-Block-B.md) | How slowly can the loop run before delay outruns the unstable pole? |
| [Block C — IMU Lever Arm](simulation/reports/IMU-Lever-Arm-Estimator-Block-C.md) | The accelerometer measures specific force, not gravity. How much does that cost? |
| [Block D — Friction Sensitivity](simulation/reports/Friction-Sensitivity-Block-D.md) | With no friction measurements, how much can the loop tolerate? |

## `testing/` — hardware

| Document | What it covers |
|---|---|
| [`Corner-RateFilter-and-Edge-Hardware-Tests-2026-08-19.md`](testing/Corner-RateFilter-and-Edge-Hardware-Tests-2026-08-19.md) | Six captures analysed: corner quiet-hold A/B, pushes to failure, edge quiet hold, edge recovery. Includes what the data **cannot** answer and why |
| [`Kalman-Filter-Rate-Estimator-Evaluation-2026-08-20.md`](testing/Kalman-Filter-Rate-Estimator-Evaluation-2026-08-20.md) | Evaluating a Kalman rate estimator against the shipped complementary filter |
| [`Corner-Fine-Tuning-Test-Plan.md`](testing/Corner-Fine-Tuning-Test-Plan.md) | The ordered test plan for fine-tuning corner balance |

## `electronics/` and `bom/`

[`Electrical-Design-Guide.md`](electronics/Electrical-Design-Guide.md) — power
tree, grounding, CAN topology and the reasoning behind the schematic, with
rendered [schematic PDFs](electronics/) from the KiCad project in
[`hardware/`](../hardware/).

[`bom/`](bom/) — [full component reference](bom/Component-Reference.md) with
verified-vs-estimate flags per parameter, [motor notes](bom/Motor-TMotor-Antigravity-4006.md),
the [CAD↔BOM handoff](bom/CAD-Handoff-BOM.md) and a
[reference-build checklist](bom/Pennings-Reference-Component-Checklist.md).

## `references/`

[Literature with DOIs](references/README.md) — the ETH Zurich Cubli papers and
the unit-complex-number formulation.

## Also here

[`Firmware-Lessons-2D-Panel-to-3D-Cube.md`](Firmware-Lessons-2D-Panel-to-3D-Cube.md)
— bugs found on the panel, how they were fixed, and which ones are structurally
guaranteed to reappear on the cube.

---

**A note on how these are written.** Where a result was later found to be wrong,
the correction is made *in place and on the record* rather than silently — see
the "Correction on record" blocks in the envelope study, and §1 of the
2026-08-19 test report, which states plainly that its own headline comparison is
confounded and should not be read as a verdict.

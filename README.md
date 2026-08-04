# Cubli — Space Challenge 2026

Virtual simulation and design documentation for a reaction-wheel self-balancing cube (Cubli), built for the Space Challenge Sofia programme.

## Structure

- **`docs/dynamics/`** — equations of motion. [`3D-Cubli-Lagrangian-Derivation.md`](docs/dynamics/3D-Cubli-Lagrangian-Derivation.md) derives the full 3D corner-balancing EOM from scratch (§8 specializes it to the 1D/2D edge-balancing case). [`1D-Jig-to-3D-Cube-Strategy.md`](docs/dynamics/1D-Jig-to-3D-Cube-Strategy.md) explains exactly what transfers from the 2D test model to the 3D cube. [`Deriving-Dynamics-from-CAD.md`](docs/dynamics/Deriving-Dynamics-from-CAD.md) covers turning CAD numbers into model parameters, and [`Quaternions-Complete-Guide.md`](docs/dynamics/Quaternions-Complete-Guide.md) covers the attitude representation used in the 3D model.
- **`docs/simulation/`** — the 2D nonlinear panel model. [`Simulation-Strategies.md`](docs/simulation/Simulation-Strategies.md) and [`Panel-Controller-Workflow.md`](docs/simulation/Panel-Controller-Workflow.md) lay out the approach; [`Simscape-Panel-Model-Build-Guide.md`](docs/simulation/Simscape-Panel-Model-Build-Guide.md) documents the Simulink/Simscape build. `reports/` holds the four nonlinearity test blocks (saturation, discrete-time loop, IMU lever arm, friction), each corresponding to a script in `matlab/`.
- **`docs/electronics/`** — [`Electrical-Design-Guide.md`](docs/electronics/Electrical-Design-Guide.md) plus rendered schematic PDFs from the KiCad project.
- **`docs/bom/`** — component reference, CAD↔BOM handoff notes, the Pennings reference-build checklist, motor datasheet notes, and the filled BOM PDF.
- **`docs/references/`** — the core Cubli research papers this work builds on (ETH ECC2013, IROS2012, TCST2016 nonlinear control, unit-complex-number formalism).
- **`hardware/kicad/`** — the KiCad project (`project/`), project-specific component libraries (`libraries/`), and the XIAO ESP32-C6 sub-board design (`xiao-esp32-c6-board/`). Bulk vendor symbol/footprint libraries (SparkFun, Seeed) are not included — re-fetch them from their upstream repos if opening the project fresh.
- **`matlab/`** — the 2D panel simulation code (`cubli_panel_*.m`, `cubli_lqr_design.m`) and the Simulink model (`Cubli_sim.slx`).
- **`media/`** — supporting photos, starting with a photo of the weighed/measured component parameters.

## Status

Simulation-only phase, preparing to contribute to the full team build in Sofia from day 1. No physical build photos or CAD renders yet — worth adding a hero render of the assembly (from the `.step` files) and jig/build photos once available.

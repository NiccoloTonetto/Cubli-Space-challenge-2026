# Figures

Every plot, CAD capture and video. Simulation figures are committed as **both**
PDF (vector, for reports) and PNG (raster, so they render inline on GitHub).
Regenerate the whole simulation set with
[`cubli_figures.m`](../simulation/cube-3d/export/cubli_figures.m).

## `build/` — the physical cube

| File | What it shows |
|---|---|
| [`cubli-corner-balance.jpg`](build/cubli-corner-balance.jpg) | The cube balancing unsupported on a single corner. Printed frame, three ballasted reaction wheels, moteus drivers and the Teensy stack visible inside, battery top-centre. This is the README hero. |

## `simulation/` — the design study

| Figure | What it shows |
|---|---|
| [`hw_run_overview`](simulation/png/hw_run_overview.png) | **The headline.** 373 s of continuous corner balance — tilt, wheel rates, torque-clamp duty and trim convergence. Generated from real telemetry, not simulation. |
| [`cube_recovery`](simulation/png/cube_recovery.png) | Corner `(-1,-1,-1)` released from 2.8°, shipping configuration: true tilt against estimate error |
| [`estimator_comparison`](simulation/png/estimator_comparison.png) | Ideal sensor vs complementary filter vs raw accelerometer. The raw trace runs to 200° — it falls every time |
| [`multi_corner`](simulation/png/multi_corner.png) | Per-corner recovery envelope against equilibrium tilt, all eight corners, with the usable margin |
| [`actuator_chart`](simulation/png/actuator_chart.png) | Worst-case recovery across the τ_max × ω_cap design space. The envelope is set here, not by the controller |
| [`qw_robustness`](simulation/png/qw_robustness.png) | Wheel-rate weight vs robustness to inertia error. Gaps are failures — why `qw = 10` ships instead of the grid optimum |
| [`kp_cliff`](simulation/png/kp_cliff.png) | Complementary-filter gain: a hard cliff between 6 and 7, and why the shipped value is 4 |
| [`loop_rate`](simulation/png/loop_rate.png) | Discrete closed-loop damping vs loop rate with one-cycle delay — the stability floor at 38.6 Hz and the 400 Hz design point |
| [`pole_map`](simulation/png/pole_map.png) | Open- and closed-loop eigenvalues |
| [`ctrb_spectrum`](simulation/png/ctrb_spectrum.png) | Controllability matrix spectrum: rank 8 of 9. Yaw is uncontrollable — which is why the gains are yaw-projected |
| [`swing_amplitude`](simulation/png/swing_amplitude.png) | Amplitude error in swing-test inertia measurement, exact vs small-angle |

## `hardware/` — test campaign plots

[`2026-08-19-corner-edge/`](hardware/2026-08-19-corner-edge/) — nine plots
generated from flight telemetry for the
[corner rate-filter and edge hardware tests](../docs/testing/Corner-RateFilter-and-Edge-Hardware-Tests-2026-08-19.md),
which embeds and discusses each one: quiet-hold tilt, wheel effort, PSD and
filter effect, the push-to-failure timeline and a time-aligned zoom, edge quiet
hold, edge recovery from 6°, and a Kalman-vs-complementary PSD comparison.

## `cad/` — mass properties and measurement

| File | What it shows |
|---|---|
| [`params-weighed.jpg`](cad/params-weighed.jpg) | The physical weighing session behind the measured mass properties |
| [`all_cube_data.jpg`](cad/all_cube_data.jpg) | CATIA mass-property readout, full assembly |
| [`wheel_data.jpg`](cad/wheel_data.jpg) | CATIA mass-property readout, reaction wheel |
| [`non_rotational_data.jpg`](cad/non_rotational_data.jpg) | CATIA mass-property readout, non-rotating structure |

These are the bridge between CAD and the model — the method is written up in
[`Deriving-Dynamics-from-CAD.md`](../docs/dynamics/Deriving-Dynamics-from-CAD.md).

## `media/` — video

| File | What it is |
|---|---|
| [`corner-balance-demo.mp4`](media/corner-balance-demo.mp4) | The cube balancing on its corner |
| [`panel-recovery-sim.mp4`](media/panel-recovery-sim.mp4) | Simscape animation of the 2D panel recovering |

## Conventions

- Simulation figures are generated, never hand-edited. If one is wrong, fix the
  script and regenerate.
- PNG renders are produced from the PDFs at 150 dpi:
  `pdftoppm -png -r 150 -singlefile in.pdf out`
- Hardware plots are produced by the analysis tools in
  [`firmware/cubli-ui/tools/analysis/`](../firmware/cubli-ui/tools/analysis/)
  from the captures in [`data/`](../data/README.md).

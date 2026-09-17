# Simulation

MATLAB, Simulink and Simscape Multibody models for the 2D panel and the 3D cube.

## Requirements

MATLAB R2023b or newer with **Simulink**, **Simscape**, **Simscape Multibody**
and the **Control System Toolbox**. The `cubli_demo` entry point below needs only
base MATLAB and the Control System Toolbox — no Simulink — and is the fastest way
to see the plant and controller work.

Everything assumes the whole tree is on the path:

```matlab
addpath(genpath('simulation'))
```

## Start here

```matlab
cubli_demo          % 3D cube: plant summary, LQR design, 2° recovery
cubli_demo(3.5)     % ...from 3.5° instead
cubli_demo_anim     % animated version
```

Self-contained, no Simulink, a few seconds. If you only run one thing, run this.

---

## `cube-3d/` — the 3D cube

Two physical builds are modelled. **NEMESIS** (156 mm, 1.615 kg, measured
2026-08-21) is the final one and the one that flew; the earlier cube is kept
because the shipped [`cubli_gains.h`](../firmware/cubli-ui/teensy/cubli_gains.h)
and the [performance-envelope study](../docs/dynamics/Cube-Performance-Envelope-Results.md)
were both generated from it.

| Folder | What's in it |
|---|---|
| `params/` | mass properties, geometry, the eight corner equilibria |
| `plant/` | per-corner and per-edge linear plant; reduced-order LQR with yaw projection |
| `analysis/` | nonlinear plant with sensors and actuator limits; recovery-envelope bisection; attitude helpers |
| `simscape/` | `simscape3d.slx` and the scripts that drive it |
| `studies/` | the sweeps and case studies that produced the published numbers |
| `export/` | generates `cubli_gains.h` for firmware, and the figures |
| `cad/` | the STEP geometry the Simscape model imports |

### Run order — NEMESIS (the flown cube)

```matlab
p = cubli_nemesis_params;              % 1. plant from measured masses + STEP geometry
S = cubli_nemesis_sweep(p, 'grid');    % 2. choose Bryson weights (peak torque is the metric)
[Kp,K,info] = cubli_nemesis_gains(p);  % 3. reduced-order LQR, yaw-projected -> Kp is what firmware uses
C = cubli_nemesis_allcorners(p);       % 4. plant + gains for all eight corners
cubli_nemesis_run                      % 5. closed-loop Simscape run  (slow, ~20 s)
```

Step 5 needs steps 1–3 in the workspace and will assert if they are missing.

### Run order — the earlier cube (reproduces the published envelope)

`corner_case_study.m` is a single control panel for the whole study: edit its
CONFIG block, set the RUN flags, press play. It drives the entire toolchain —
`cubli_cube_params` → `cubli_corner_plant` → `cubli_gains` → `cubli_nlsim` →
`cubli_maxrec` → `cubli_corner_run`. Use it rather than calling the pieces by
hand.

```matlab
edit corner_case_study      % set CONFIG + RUN flags, then run
cubli_figures               % regenerate every figure in figures/simulation/
cubli_export_gains          % regenerate firmware/cubli-ui/teensy/cubli_gains.h
```

> `cubli_gains.h` is a **generated file**. Regenerate it with `cubli_export_gains`
> rather than editing the constants by hand — several firmware stages read their
> literals from it precisely so there is one source of truth.

### A note on the Simscape model

`simscape3d.slx` has three wiring quirks that the run scripts handle and that
will bite anyone driving it directly, all documented in the header of
[`cubli_nemesis_run.m`](cube-3d/simscape/cubli_nemesis_run.m):

1. Mux channels come out **interleaved**, not grouped — the permutation is
   detected at runtime, never assumed.
2. Euler angles are not the reduced attitude; `cubli_ss_patch_attitude` inserts
   the `gB` cross-product block. Without it, yaw runs away.
3. Validation ("Gate 3") must run on the **raw** wiring and the closed loop on the
   **patched** wiring. Checking stability in the wrong configuration is misleading.

---

## `panel-2d/` — the 1D/2D panel

The single-wheel test panel that came first. Its job was to make every modelling
mistake cheap, on a rig that cannot hurt anyone, before the cube existed.

| Script | Purpose |
|---|---|
| `cubli_panel_params.m` | plant parameters — measured masses by default |
| `cubli_panel_simscape_gates.m` | six validation gates against `Cubli_sim.slx` |
| `cubli_panel_saturation.m` | Block A — recoverable-tilt envelope with a real actuator |
| `cubli_panel_discrete.m` | Block B — sample-and-hold plus computational delay |
| `cubli_panel_imu.m` | Block C — accelerometer lever arm and complementary filter |
| `cubli_panel_friction.m` | Block D — friction sensitivity and tolerance thresholds |
| `cubli_lqr_design.m` | standalone LQR design and deployability gates |
| `cubli_panel_demo.m` | quick self-contained demo |

**Run the parameter and LQR sections of `cubli_panel_simscape_gates.m` first.**
Blocks A–D all assert on `p` and `K_lqr` being in the workspace and will tell you
if they are missing. Blocks A and B also need manual edits to the Simulink model
first — each script's header states exactly which blocks to insert and how to wire
them.

Written up in [`docs/simulation/`](../docs/simulation/), with one report per block
under [`reports/`](../docs/simulation/reports/).

---

## `validation/` — model vs hardware

Scripts that plot recorded flight telemetry against the model's prediction.
The matching captures are in [`../data/`](../data/README.md); the 2D panel
release tests at 7°, 8° and 12° are in `data/panel/`.

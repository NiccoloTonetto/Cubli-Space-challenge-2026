# Flight telemetry

Curated recordings from the cube and the 2D panel. These are the runs the
[test reports](../docs/testing/) analyse and the runs that firmware constants were
measured from — not a full session archive. Roughly 150 MB of redundant repeats,
zero-length captures and superseded sessions were removed during cleanup.

Every number quoted below was recomputed from the file in this folder, over the
single longest contiguous `armed == 1` window.

## `corner/` — 3-wheel corner balance

| File | Armed | Tilt RMS | Tilt max | What it is |
|---|---|---|---|---|
| [`corner-balance-373s-2026-08-20.log`](corner/corner-balance-373s-2026-08-20.log) | **373.5 s** | **0.40°** | 1.25° | The longest continuous balance. Auto-trim converged; wheels peak at 26 rad/s against a 40 rad/s cap. Plotted in [`hw_run_overview`](../figures/simulation/png/hw_run_overview.png). |
| [`FINAL_CALIBRATION.log`](corner/FINAL_CALIBRATION.log) | 278.1 s | **0.25°** | 0.92° | Quietest run recorded. Its last ~55 s is the source of the hardcoded `kPhiOffset` in `CubliCorner`/`CubliBalance`. |
| [`perfect_equilibrium_2.log`](corner/perfect_equilibrium_2.log) | 101.8 s | 0.33° | 0.99° | The converged-trim run (per-axis std < 0.0012°) that justified retiring trim adaptation for a fixed offset. |
| [`1minCORNER_autotrim.log`](corner/1minCORNER_autotrim.log) | 46.7 s | 0.39° | 0.93° | Quiet hold, **no** rate filter. A-side of the rate-filter comparison. |
| [`1minCORNER_lpf.log`](corner/1minCORNER_lpf.log) | 50.4 s | 0.46° | 1.22° | Quiet hold, 20 Hz rate filter. B-side. 28 columns — carries the filtered rates too. |
| [`PUSHES_CORNER_autotrim.log`](corner/PUSHES_CORNER_autotrim.log) | — | — | 24.0° | Repeated manual pushes to failure, no rate filter. 5 recovered pushes, then the 25° trip. |
| [`PUSHES_CORNER_lpf.log`](corner/PUSHES_CORNER_lpf.log) | — | — | 23.8° | Same, with the rate filter. 4 recovered pushes. |
| [`corner-trim-convergence-2026-08-21.csv`](corner/corner-trim-convergence-2026-08-21.csv) | — | — | — | Short dashboard capture; a small, readable example of the plot-mode CSV. |

> **The two `1minCORNER_*` runs are not a clean A/B.** They are separate bench
> sessions, and the filtered build also shipped a faster trim-adaptation gain.
> [The report says so and explains why](../docs/testing/Corner-RateFilter-and-Edge-Hardware-Tests-2026-08-19.md#1-corner-quiet-hold--autotrim-vs-rate-filter) —
> don't read the delta as the filter's effect.

## `edge/` — single-wheel edge balance

| File | Armed | Tilt RMS | Tilt max | What it is |
|---|---|---|---|---|
| [`1minEDGE.log`](edge/1minEDGE.log) | **197.0 s** | 0.58° | 3.58° | Longest edge hold. Intended as undisturbed, but contains an unexplained ~3.5° excursion between t≈100–135 s — see report §4. |
| [`EDGE6degrees.log`](edge/EDGE6degrees.log) | 10.7 s | 1.30° | 6.18° | Release from −6.18°. A lightly damped ring-down, not a clean recovery: overshoots to +5.05°, settles under 0.5° only at 9.6 s. |
| [`EDGE5degrees.log`](edge/EDGE5degrees.log) | 8.0 s | 0.85° | 5.26° | Release from ~5°, companion to the above. |
| [`EDGE_BALANCE.csv`](edge/EDGE_BALANCE.csv) | — | — | — | Dashboard plot-mode capture of an edge session. |

## `panel/` — 2D panel release tests

Hardware validation of the 1D/2D panel model: release from rest at 7°, 8° and 12°.
Same 10-column schema as the edge format, no header row. Plot them against the
model with [`simulation/validation/panel/plot_validation_csv.py`](../simulation/validation/panel/plot_validation_csv.py).

## Formats

Two wire formats. The analysis tools detect which by counting columns.

**Corner — 26 columns** (28 on rate-filter builds), tab-delimited, header row present:

```
t_ms  phi_x_deg phi_y_deg phi_z_deg   om_x_dps om_y_dps om_z_dps
      rho_x rho_y rho_z                rho_x_lp rho_y_lp rho_z_lp
      tau_x tau_y tau_z                tau_cmd_x tau_cmd_y tau_cmd_z
      armed  gain_scale
      trim_x_deg trim_y_deg trim_z_deg trim_com_mm trim_enabled
      [om_x_filt_dps om_y_filt_dps]    <- 28-column builds only
```

`phi` is the reduced-attitude tilt vector `-cross(gB, ghat)` in body axes, **not**
Euler angles. Tilt magnitude is `norm(phi(1:3))`. `rho` is wheel rate relative to
the body — what the encoder reads — and `_lp` is its low-passed version.

**Edge — 10 columns**, comma- or tab-delimited, header row sometimes absent:

```
t_ms, theta_deg, theta_dot_dps, tau_Nm, tau_cmd_Nm,
      armed, gain_scale, wheel_omega_lp, wheel_pos, wheel_vel
```

`.log` files also carry firmware console output interleaved with the data —
command echoes, arm refusals, trip reasons — on lines beginning with `#`. That is
deliberate: the file is what the serial monitor would have shown. Every parser
here skips `#` lines.

Angles are degrees, rates deg/s, torques N·m, wheel rates rad/s, time
milliseconds since boot.

## Reading them

```bash
python firmware/cubli-ui/tools/analysis/plot_session_csv.py      # newest recording
python firmware/cubli-ui/tools/analysis/fft_tilt_analysis.py       data/corner/1minCORNER_lpf.log
python firmware/cubli-ui/tools/analysis/standing_speed_report.py   data/corner/corner-balance-373s-2026-08-20.log
```

New captures written during a live session land under `firmware/**/telemetry/`,
which is gitignored. Promote a run into `data/` deliberately, and add a row above
saying what it shows.

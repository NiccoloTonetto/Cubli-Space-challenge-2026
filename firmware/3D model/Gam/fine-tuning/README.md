# Fine-Tuning — the 7 tests, what to actually run

Index for **Cube Fine-Tuning — Test Plan.md**'s 7 tests. That doc has the
reasoning, the numbers and the pass/fail tables; this page has the exact
command or file for each one, since nothing here is exploratory and every
test should take you straight to a concrete action, not a re-read of the
theory first.

No new firmware lives in this folder — it points at
[`../corner-bringup/`](../corner-bringup/README.md) and
[`../FINAL/`](../FINAL/README.md), same as everywhere else in this project:
one implementation, referenced from wherever it's relevant, not copied.

**Rule from the Test Plan, repeated because it's the one that gets skipped
under time pressure: one change at a time, written down. Tests 1-3 are
free and take fifteen minutes — do not jump to Test 7.**

## Session 1 — Diagnose (30 min, free)

| # | Test | Run this | Pass / what you're looking for |
|---|---|---|---|
| 1 | FFT the tilt | Balance quietly, log 60 s, then `python ../FINAL/fft_tilt_analysis.py run.csv` (or `--channel wheels` for the rates too — Test 1 wants both) | No sharp peak — the design's poles are all near-critically-damped. If there is one, the printed hint names which test to run next (2, 3, or "check the loop period") |
| 2 | Estimator lag audit | Code read, not a bench test — see [Findings so far](#findings-so-far) below, already done once against `Stage4_AutoTrim.ino`/`Stage5_AutoTrim.ino` | `rate_cutoff_hz`/tau/gate items don't map onto this codebase's cross-product filter (architecture difference, see below) — but BMI270 ODR vs loop rate was a real, fixed finding |
| 3 | Cable strain relief | Physical — zip-tie the bundle to the frame, kill free length | **Already done** per hardware update. Re-run Test 1 after any further cable/harness changes |

## Session 2 — Commission (1.5 h)

| # | Test | Run this | Pass / what you're looking for |
|---|---|---|---|
| 4 | Standing wheel speeds | Balance quietly 2 min, then `python ../FINAL/standing_speed_report.py run.csv --window 30` | Verdict per axis (healthy / worth attention / trim it / will not survive) straight from the Test Plan's table. `--k1`/`--k3`/`--ell` override the corner `[-1,-1,-1]` defaults once another corner's own values are known |
| 5 | Automatic trim | [`../corner-bringup/Stage5_AutoTrim/`](../corner-bringup/Stage5_AutoTrim/Stage5_AutoTrim.ino) (or `Stage4_AutoTrim/` for hand-held bring-up first) — arm, watch `trim_x/y/z_deg` converge, or `z1` to fast-start. If a hardware run shows torque saturating on a coherent high-frequency mode (see hw-run-analysis.md finding below), use the `_RateFilter` variant instead, same procedure | ~90% of the error cancelled in 4 time constants — `tau_a`=60s (default `k_a`=2.922e-6) in the plain AutoTrim files, ~10s (default `k_a`=1e-4) in the `_RateFilter` files. Converged value is a live COM-error readout (`trim_com_mm`) |
| 6 | Wheel-speed cap | `Stage5_AutoTrim.ino`'s `o<rad/s>`/`p<rad/s>` (live, no reflash) — set `omega_cap` to ~3x the Test 4 post-trim standing speed, `taper_start` to 90% of that, verify, step toward `o40 p36` | Fade, not a switch (already true — taper only touches spin-up, never braking, verified in `commandWheels()`). No audible buzzing/chatter as it tightens |
| 7 | (start here, finish in Session 3) | Same file, same `o`/`p` from Test 6 | — |

## Session 3 — Validate (45 min)

| # | Test | Run this | Pass / what you're looking for |
|---|---|---|---|
| 7 | Endurance run | `Stage5_AutoTrim.ino`, arm, leave it 30 min. `python ../FINAL/plot_session_csv.py run.csv --cols wheels,trim,endurance` | `trim` flat (not drifting), standing speed flat (not climbing), `temp_x/y/z` plateauing (not climbing continuously — if it never does, `TAU_MAX`=0.12 N·m may be too high, an estimate per the Test Plan), `loop_overrun_count` barely moving, `sat_duty_x/y/z` low outside disturbances |

## Findings so far (Test 2, done once)

Code-read against `Stage4_AutoTrim.ino`/`Stage5_AutoTrim.ino`'s
`attitudeUpdate()` (the same estimator in every corner-bringup file since
Stage 1):

| Test 2 checklist item | Verdict |
|---|---|
| BMI270 ODR >= loop rate | **Was failing, now fixed** — every balance-stage file configured 400 Hz ODR against a 500 Hz loop (loop outruns the sensor, ~1 in 5 reads stale). `cube-bringup/Stage0c_IMUJitter.ino` already validated 800 Hz as the fix back in Phase 0.4 and it was never carried forward — now is, in `Stage4_AutoTrim.ino`/`Stage5_AutoTrim.ino` only (the already-hardware-validated Stage1-3/Stage5_Release/edge-bringup/FINAL files are untouched; same fix is one line if you want it there too) |
| alpha recomputed from measured `dt` each cycle, never hardcoded | **Pass** — `dt` comes from `micros()` timing every loop iteration, used directly in the continuous-time filter update |
| `rate_cutoff_hz` >= 30 Hz | **Doesn't apply as written** — this codebase's filter is the cross-product complementary form (`kP=4`, `kI=0.5` on `ghat`/`gam` directly), not a classical gyro-rate low-pass with a configurable cutoff. There's no `rate_cutoff_hz` parameter to check |
| complementary filter `tau` in 0.5-1.5 s | **Doesn't map cleanly** — same architecture difference. `kP=4` gives a rough correction bandwidth well outside that range in the classical-filter sense, but comparing the two numbers directly isn't apples-to-apples given the different filter structure |
| accelerometer gate 10-20%, deliberately loose | **Gap, not fixed** — there is no accelerometer-magnitude gate in the current filter at all; a sudden linear acceleration (a bump, a catch) corrupts `ga` with no protection this cycle. Worth adding if Test 1 ever shows a peak tied to handling transients specifically, not fixed preemptively here since it's a real architecture change, not a one-line constant |

## Findings so far (hw-run-analysis.md, 373.5 s hardware run, auto-trim on)

A real continuous corner-balance run found a problem the sim envelope
didn't predict: torque saturated 71.8% of the time, driven by a ~35 Hz
structural mode riding on the gyro signal, in phase across `phi`, `om`
AND `u` (a resonant shape white noise cannot produce) — not because the
cube was fighting a real disturbance (saturated vs unsaturated mean tilt
were statistically identical). The 1.3 Hz-bandwidth control loop cannot
damp a 35 Hz mode; it can only feed it energy through `K2` and the torque
clamp, in a closed loop that re-excites the mode every cycle.

| Fix | Where |
|---|---|
| 4.1 — first-order low-pass (15-25 Hz, 20 Hz default) on the rate signal feeding the control law's `om` block only (not `w_b` globally — trip checks and estimator stay on raw, unfiltered rate) | [`../corner-bringup/Stage4_AutoTrim_RateFilter/`](../corner-bringup/Stage4_AutoTrim_RateFilter/Stage4_AutoTrim_RateFilter.ino), [`../corner-bringup/Stage5_AutoTrim_RateFilter/`](../corner-bringup/Stage5_AutoTrim_RateFilter/Stage5_AutoTrim_RateFilter.ino) — live-settable with `f<Hz>` |
| 4.5 — adaptation gain default raised from `tau_a`=60s (`k_a`=2.922e-6) to `k_a`=1e-4 (~21x below the re-derived stability limit on the 15-state augmented model, converges in ~10s) | same two files, `k<value>` unchanged as the live override |
| 4.2 — reduce `qr` (needs the actual re-derived `Kp` for the new plant, not implementable from the analysis alone) | **not done** — see "What's still open" below |

**Update 2026-08-19: corner `[-1,-1,-1]`'s `Kp`/`ell`/`theta_eq` are now
re-derived and applied** in all four AutoTrim-family files (mass 1.633 kg
+ the new strut). The other seven corners are still the old 1.5668 kg /
no-strut values — every `kCorners` table is a MIXED-GENERATION table, see
the TODO comment above each. Full numbers and the multi-corner finding
that came with them (six of eight corners now have `theta_eq` beyond
their own recovery envelope — see below) are in
[`../../docs/dynamics/Cube-Performance-Envelope-Results.md`](../../../../docs/dynamics/Cube-Performance-Envelope-Results.md)'s
"Hardware-stage update" section.

## Files this points at

| Test(s) | File |
|---|---|
| 1 | [`../FINAL/fft_tilt_analysis.py`](../FINAL/fft_tilt_analysis.py) |
| 4 | [`../FINAL/standing_speed_report.py`](../FINAL/standing_speed_report.py) |
| 5, 6, 7 | [`../corner-bringup/Stage4_AutoTrim/`](../corner-bringup/Stage4_AutoTrim/Stage4_AutoTrim.ino) (hand-held), [`../corner-bringup/Stage5_AutoTrim/`](../corner-bringup/Stage5_AutoTrim/Stage5_AutoTrim.ino) (unsupported, real policy) |
| 5, 6, 7 + hw-run-analysis.md's 35 Hz fix | [`../corner-bringup/Stage4_AutoTrim_RateFilter/`](../corner-bringup/Stage4_AutoTrim_RateFilter/Stage4_AutoTrim_RateFilter.ino), [`../corner-bringup/Stage5_AutoTrim_RateFilter/`](../corner-bringup/Stage5_AutoTrim_RateFilter/Stage5_AutoTrim_RateFilter.ino) — same procedures as the plain AutoTrim files, plus `f<Hz>` for the rate-filter corner |
| 7 (plots) | [`../FINAL/plot_session_csv.py`](../FINAL/plot_session_csv.py) — recognizes all five corner telemetry widths now: 21 (`CornerBalance`/`CornerBalance_WiFi`), 26 (`Stage4_AutoTrim`), 29 (`Stage4_AutoTrim_RateFilter`), 33 (`Stage5_AutoTrim`, adds `trim`/`endurance` trace groups), 36 (`Stage5_AutoTrim_RateFilter`, adds filtered-rate traces to the `rates` group) |

## What's still open

- **Multi-corner locomotion is currently off the table on six of eight
  corners** — the strut update above shifted `theta_eq` unevenly across
  the cube; only `[-1,-1,-1]` and `[+1,+1,+1]` (the primary body-diagonal
  ends) still have positive recovery margin. Single-corner balancing
  (this project's current test scope) is unaffected. Full table in
  [`../../docs/dynamics/Cube-Performance-Envelope-Results.md`](../../../../docs/dynamics/Cube-Performance-Envelope-Results.md).
  Needs a decision (rebalance the pole, or a counterweight) before this
  becomes a mission-capability assumption elsewhere.
- **The `Kp[3][9]` gain matrix is stale for SEVEN of eight corners, in
  every corner-bringup file** (corner `[-1,-1,-1]` is done, see above).
  `gB` (the corner-resolution direction) is stale for `[-1,-1,-1]` too —
  not provided alongside the new `Kp`, though low-risk given `theta_eq`
  only moved +0.097°. Fix 4.2 ("reduce `qr`") specifically needs a
  re-derived matrix, not something derivable from the rate-filter fix
  alone — moot for `[-1,-1,-1]` now, still open for the other seven.
- **Tests 5-7 are Stage 4/5 AutoTrim only, on ONE corner.** Per the Test
  Plan and every other file in this project: a result on one corner says
  nothing about another. Repeat for the other seven once this one is
  validated.
- **Test 2's accelerometer gate gap** (see table above) — architecture
  change, not done here.
- **EEPROM/LittleFS trim persistence** — noted in both AutoTrim files'
  own headers/NOTES, not implemented. Trim starts at zero (or wherever
  `z1` seeds it) every boot until this is added.
- **K1/K3 are only known for corner `[-1,-1,-1]`** (the Test Plan's own
  measured values) — `standing_speed_report.py`'s defaults are that
  corner's. Other corners' K1/K3 aren't in `cubli_gains.h` (which carries
  the full `Kp[3][9]` matrix, not the reduced SISO K1/K3 pair) — pass
  `--k1`/`--k3`/`--ell` once another corner's own values are derived, or
  the default still gives a directionally-useful (if not exact) number
  since all 8 corners' `ell` only vary 122-137 mm.

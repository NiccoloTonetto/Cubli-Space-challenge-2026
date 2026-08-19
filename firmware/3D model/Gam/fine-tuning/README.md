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
| 5 | Automatic trim | [`../corner-bringup/Stage5_AutoTrim/`](../corner-bringup/Stage5_AutoTrim/Stage5_AutoTrim.ino) (or `Stage4_AutoTrim/` for hand-held bring-up first) — arm, watch `trim_x/y/z_deg` converge, or `z1` to fast-start | ~90% of the error cancelled in 4 time constants (~4 min at the default `tau_a`=60s). Converged value is a live COM-error readout (`trim_com_mm`) |
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

## Files this points at

| Test(s) | File |
|---|---|
| 1 | [`../FINAL/fft_tilt_analysis.py`](../FINAL/fft_tilt_analysis.py) |
| 4 | [`../FINAL/standing_speed_report.py`](../FINAL/standing_speed_report.py) |
| 5, 6, 7 | [`../corner-bringup/Stage4_AutoTrim/`](../corner-bringup/Stage4_AutoTrim/Stage4_AutoTrim.ino) (hand-held), [`../corner-bringup/Stage5_AutoTrim/`](../corner-bringup/Stage5_AutoTrim/Stage5_AutoTrim.ino) (unsupported, real policy) |
| 7 (plots) | [`../FINAL/plot_session_csv.py`](../FINAL/plot_session_csv.py) — recognizes all three corner telemetry widths now: 21 (`CornerBalance`/`CornerBalance_WiFi`), 26 (`Stage4_AutoTrim`), 33 (`Stage5_AutoTrim`, adds `trim`/`endurance` trace groups) |

## What's still open

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

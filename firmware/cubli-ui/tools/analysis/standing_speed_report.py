"""standing_speed_report.py  --  Test 4 (standing wheel speed) verdict

The most informative number the machine produces. A constant reference
error `delta` forces the momentum-management term (K3) to trade wheel
speed against tilt instead of letting torque ramp the wheel to
saturation, so the wheel settles at a fixed STANDING SPEED proportional
to the error:

    phidot_ss = -(K1 * delta) / K3        =>       delta = -phidot_ss * K3/K1

No new sensor needed -- this script just reads the standing speed this
codebase already computes and telemeters (rho_x_lp/rho_y_lp/rho_z_lp, a
5 s low-pass of wheel speed, "gRhoLp" in the firmware, present in every
corner-bringup Stage2+ file and both CORNER telemetry formats), converts
it through K1/K3/ell into a tilt error and an equivalent COM offset in
mm, and prints the same verdict table Test 4 uses to decide what to do
next.

----------------------------------------------------------------------
USAGE

    python standing_speed_report.py run.csv
    python standing_speed_report.py run.csv --t 30:90
    python standing_speed_report.py run.csv --k1 3.6084 --k3 0.000633 --ell 122.84

Defaults are corner [-1,-1,-1]'s measured values (|K1|=3.6084,
|K3|=0.000633, ell=122.84mm) from the Test Plan -- override with --k1/
--k3/--ell for a different corner once its own values are known. The
verdict table's mm column only means what it says for the SAME corner
those K1/K3/ell came from; using the default on another corner still
gives a directionally-useful number (all 8 corners' ell only vary
122-137mm, K1/K3 by less still) but isn't exact.

----------------------------------------------------------------------
FLAGS

  --t LO:HI     Crop to this window (seconds from recording start) before
                averaging. Use this to select a quiet, already-armed,
                already-settled stretch -- same reasoning as
                fft_tilt_analysis.py's --armed-crop, but you pick the
                window by hand here since "settled" (trim converged, or
                no trim and just sitting quietly) isn't a single
                detectable event the way "longest armed run" is.
  --window SEC  How many seconds at the END of the (possibly --t
                cropped) window to average over for the reported
                standing speed. Default 10 -- long enough to ride out
                the 5 s low-pass's own settling, short enough to still
                be "now" rather than "the whole session".
  --k1 / --k3 / --ell   Override the conversion constants (see above).
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_session_csv import (  # noqa: E402
    load_rows, CORNER_COLS, CORNER_TRIM_COLS, CORNER_TRIM_FILT_COLS,
    CORNER_TRIM_KALMAN_COLS, CORNER_ENDURANCE_COLS, CORNER_ENDURANCE_FILT_COLS,
)

CORNER_WIDTHS = (len(CORNER_COLS), len(CORNER_TRIM_COLS), len(CORNER_TRIM_FILT_COLS),
                 len(CORNER_TRIM_KALMAN_COLS),
                 len(CORNER_ENDURANCE_COLS), len(CORNER_ENDURANCE_FILT_COLS))

# Corner [-1,-1,-1]'s measured values (Test Plan S4 / "Automatic Trim" S3).
DEFAULT_K1  = 3.6084
DEFAULT_K3  = 0.000633
DEFAULT_ELL = 122.84   # mm

# Test 4's verdict table, standing speed in rad/s -> (label, action). The
# table gives four ANCHOR points (<2, 10, 20, >40), not four clean bin
# edges -- 10 and 20 are single example values inside the "worth
# attention"/"trim it" range, not its upper bound. The only two boundaries
# the table actually states are <2 (healthy) and >40 (will not survive);
# everything from 2 up to 40 is "worth attention" shading into "trim it"
# with no stated crossover, so 10 (the midpoint-ish anchor already in the
# table) is used as that crossover here -- explicit rather than silently
# picking 20 and making the 20-40 range read as worse than the table says.
VERDICT_TABLE = [
    (2.0,  "healthy",             "no action"),
    (10.0, "worth attention",     "keep an eye on it, not urgent"),
    (40.0, "trim it",             "run Test 5 (automatic trim) on this corner"),
    (float("inf"), "will not survive a disturbance",
     "trim before doing anything else on this corner"),
]


def verdict_for(speed_abs):
    for threshold, label, action in VERDICT_TABLE:
        if speed_abs < threshold or threshold == float("inf"):
            return label, action
    return VERDICT_TABLE[-1][1], VERDICT_TABLE[-1][2]   # unreachable, safety net


def parse_t_range(s):
    if s is None:
        return None
    lo_s, _, hi_s = s.partition(":")
    lo = float(lo_s) if lo_s else None
    hi = float(hi_s) if hi_s else None
    return (lo, hi)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("file", type=Path)
    ap.add_argument("--t", dest="t_range", default=None)
    ap.add_argument("--window", type=float, default=10.0)
    ap.add_argument("--k1", type=float, default=DEFAULT_K1)
    ap.add_argument("--k3", type=float, default=DEFAULT_K3)
    ap.add_argument("--ell", type=float, default=DEFAULT_ELL)
    args = ap.parse_args()

    if not args.file.is_file():
        sys.exit(f"No such file: {args.file}")

    rows, spec, groups, derived = load_rows(args.file)
    if len(spec) not in CORNER_WIDTHS:
        sys.exit(f"{args.file.name}: this is an EDGE-format file (no per-wheel "
                 f"rho) -- Test 4/5 (standing speed, trim) are CORNER-only.")

    names = [n for n, _ in spec]
    t0 = rows[0][0]
    t = np.array([(r[0] - t0) / 1000.0 for r in rows])

    lo, hi = parse_t_range(args.t_range) if args.t_range else (None, None)
    keep = np.ones_like(t, dtype=bool)
    if lo is not None:
        keep &= t >= lo
    if hi is not None:
        keep &= t <= hi
    if not keep.any():
        sys.exit(f"No samples inside --t {args.t_range} "
                 f"(recording spans 0 to {t[-1]:.2f} s).")
    t = t[keep]
    rows = [r for r, k in zip(rows, keep) if k]

    window_start = t[-1] - args.window
    win_idx = [i for i, ti in enumerate(t) if ti >= window_start]
    if len(win_idx) < 3:
        sys.exit(f"Only {len(win_idx)} samples in the last {args.window} s "
                 f"of the selected range -- widen --t or shrink --window.")

    col = {n: i for i, n in enumerate(names)}
    lp_cols = ["rho_x_lp", "rho_y_lp", "rho_z_lp"]
    if not all(c in col for c in lp_cols):
        sys.exit(f"{args.file.name} has no rho_x/y/z_lp columns -- unexpected "
                 f"for a CORNER-format file, check the source.")

    standing = np.array([
        [rows[i][col[c]] for c in lp_cols] for i in win_idx
    ])
    mean_speed = standing.mean(axis=0)     # signed, rad/s, per axis
    std_speed  = standing.std(axis=0)      # spread within the window

    axis_names = ["X", "Y", "Z"]
    k3_over_k1 = args.k3 / args.k1

    print(f"{args.file.name}: averaged over the last {args.window:.1f} s "
          f"of the selected range (t={t[win_idx[0]]:.2f}..{t[win_idx[-1]]:.2f} s, "
          f"{len(win_idx)} samples)")
    print(f"Conversion constants: K1={args.k1}  K3={args.k3}  ell={args.ell} mm "
          f"(K3/K1={k3_over_k1:.4e})")
    print()
    print(f"{'axis':>4}  {'speed (rad/s)':>14}  {'+/- (rad/s)':>12}  "
          f"{'tilt err (deg)':>15}  {'COM (mm)':>9}  verdict")

    worst_label_rank = -1
    worst_axes = []
    labels_in_order = [v[1] for v in VERDICT_TABLE]

    for i, name in enumerate(axis_names):
        speed = mean_speed[i]
        tilt_err_rad = -speed * k3_over_k1   # sign per phidot_ss = -(K1*delta)/K3
        tilt_err_deg = np.degrees(tilt_err_rad)
        com_mm = args.ell * np.radians(abs(tilt_err_deg))   # small-angle
        label, action = verdict_for(abs(speed))
        print(f"{name:>4}  {speed:14.3f}  {std_speed[i]:12.3f}  "
              f"{tilt_err_deg:15.4f}  {com_mm:9.3f}  {label}")
        rank = labels_in_order.index(label)
        if rank > worst_label_rank:
            worst_label_rank = rank
            worst_axes = [name]
        elif rank == worst_label_rank:
            worst_axes.append(name)

    print()
    _, worst_label, worst_action = VERDICT_TABLE[worst_label_rank]
    axes_str = "/".join(worst_axes) + (" (tied)" if len(worst_axes) > 1 else "")
    print(f"Worst axis: {axes_str} ({worst_label}) -- {worst_action}")
    if std_speed.max() > 0.3 * max(abs(mean_speed).max(), 1.0):
        print("NOTE: standing speed is still moving within this window (std is "
              ">30% of the mean on at least one axis) -- this may not be a "
              "settled reading yet. Widen --window, or check --t actually "
              "starts after trim/transients have died down.")


if __name__ == "__main__":
    main()

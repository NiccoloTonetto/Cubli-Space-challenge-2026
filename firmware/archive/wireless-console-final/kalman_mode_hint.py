"""kalman_mode_hint.py  --  suggest n<Hz>/z<value> for Stage4_FixedOffset_Kalman.ino

Stage4_FixedOffset_Kalman.ino's mode-augmented Kalman filter (docs/testing/
Kalman-Filter-Rate-Estimator-Evaluation-2026-08-20.md, Option 2) needs a
starting guess for the structural mode's natural frequency (gModeHz,
live-settable "n<Hz>") and damping ratio (gModeZeta, "z<value>"). The
observed mode moved 29.7-41.0 Hz across three different sessions, so the
compiled-in defaults are explicitly not something to trust blind -- this
script measures THIS capture's own peak instead of guessing, the same
Welch-PSD approach used to validate the filter design in the first place.

Works on ANY corner-format capture with raw om_x/y/z columns -- you do not
need to already be running the Kalman file to use this. The natural
workflow is: capture a short quiet hold on whichever corner file you have
(plain AutoTrim, rate-filter, doesn't matter), point this script at it,
then type the two suggested commands into Stage4_FixedOffset_Kalman.ino's
serial terminal.

----------------------------------------------------------------------
USAGE

    python kalman_mode_hint.py run.csv
    python kalman_mode_hint.py run.csv --band 20:50
    python kalman_mode_hint.py run.csv --no-armed-crop
    python kalman_mode_hint.py run.csv --save mode_psd.png

----------------------------------------------------------------------
FLAGS

  --t LO:HI        Crop to this time window (seconds from recording
                    start) before anything else. Same syntax as
                    plot_session_csv.py's --t.
  --no-armed-crop  Defaults to auto-cropping to the LONGEST continuous
                    armed=1 stretch, same reasoning as
                    fft_tilt_analysis.py -- an arm/disarm edge smears
                    energy across the whole spectrum. Pass this to
                    analyze the raw file instead.
  --band LO:HI     Frequency range to search for the mode, Hz. Default
                    15:55 -- comfortably brackets the 29.7-41.0 Hz
                    observed so far with margin either side. Widen this
                    if the reported peak sits right at an edge.
  --save FILE       Also write a 3-panel PSD plot (one per axis, peak and
                    half-power points marked) to FILE. Without this flag
                    the script only prints the report -- no plotting
                    library is imported unless you ask for a plot.

----------------------------------------------------------------------
HOW THE ZETA ESTIMATE WORKS, AND WHY IT'S SHAKIER THAN THE FREQUENCY ONE

Frequency is just "where is the PSD's peak" -- robust, and a Welch PSD's
peak location is usually a good, low-variance estimate even from one
capture.

Zeta comes from the peak's -3dB (half-power) bandwidth: Q = f_peak /
(f_hi - f_lo), zeta = 1/(2Q). This is the standard method, but a
bandwidth measured off ONE noisy PSD realization is a much shakier
number than a peak location -- treat it as a rough starting point, more
so than the frequency suggestion. If the half-power crossing runs off
the edge of --band without a value under half the peak, that axis's
zeta is reported as unavailable rather than guessed.

----------------------------------------------------------------------
WHAT THIS DOES NOT DO

This is not the tap test both docs still call for. It measures whatever
mode was loudest in THIS capture's om_x/y/z content within --band, which
could be a genuine structural mode, wheel noise, cable resonance, or
something else -- it does not know the difference. Cross-check against
fft_tilt_analysis.py --channel wheels on the same file if the peak looks
surprising, and see this same peak persisting across multiple sessions
(not just one) before trusting the frequency more than the "spread
29.7-41.0 Hz across three sessions" warning already in the firmware.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.signal import welch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_session_csv import (  # noqa: E402
    load_rows, CORNER_COLS, CORNER_TRIM_COLS, CORNER_TRIM_FILT_COLS,
    CORNER_TRIM_KALMAN_COLS, CORNER_ENDURANCE_COLS, CORNER_ENDURANCE_FILT_COLS,
)

CORNER_WIDTHS = (len(CORNER_COLS), len(CORNER_TRIM_COLS), len(CORNER_TRIM_FILT_COLS),
                 len(CORNER_TRIM_KALMAN_COLS),
                 len(CORNER_ENDURANCE_COLS), len(CORNER_ENDURANCE_FILT_COLS))

AXES = ["om_x", "om_y", "om_z"]


def armed_crop(t, armed):
    """(start, end) indices of the longest contiguous armed==1 run -- same
    logic as fft_tilt_analysis.py's own armed_crop, kept local here rather
    than imported so this script stays matplotlib-free unless --save is
    used (fft_tilt_analysis.py imports matplotlib at module level)."""
    best = (0, 0)
    run_start = None
    for i, a in enumerate(armed):
        if a >= 0.5 and run_start is None:
            run_start = i
        elif a < 0.5 and run_start is not None:
            if i - run_start > best[1] - best[0]:
                best = (run_start, i)
            run_start = None
    if run_start is not None and len(armed) - run_start > best[1] - best[0]:
        best = (run_start, len(armed))
    return best


def parse_range(s, kind):
    lo_s, _, hi_s = s.partition(":")
    try:
        lo = float(lo_s) if lo_s else None
        hi = float(hi_s) if hi_s else None
    except ValueError:
        sys.exit(f"--{kind} needs LO:HI (e.g. 20:50), got {s!r}")
    return lo, hi


def half_power_zeta(freqs, psd, peak_i):
    """Q = f_peak/bandwidth, zeta = 1/(2Q) via the -3dB (half-power)
    crossings either side of the peak. Returns (f_lo, f_hi, zeta,
    low_res) -- low_res True means the crossing landed within ~3 bins of
    the peak, i.e. the bandwidth (and so zeta) is closer to this
    capture's frequency resolution than a real measurement; still
    returned, but flag it as rougher than the others. (None, None, None,
    None) if a crossing runs off the search band entirely."""
    peak_val = psd[peak_i]
    half = peak_val / 2.0
    df = freqs[1] - freqs[0]   # Welch output is evenly spaced

    i = peak_i
    while i > 0 and psd[i] > half:
        i -= 1
    if psd[i] > half:   # ran off the low edge without crossing
        return None, None, None, None
    f_lo = freqs[i]

    j = peak_i
    while j < len(psd) - 1 and psd[j] > half:
        j += 1
    if psd[j] > half:   # ran off the high edge without crossing
        return None, None, None, None
    f_hi = freqs[j]

    bw = f_hi - f_lo
    if bw <= 0:
        return None, None, None, None
    q = freqs[peak_i] / bw
    low_res = bw < 3 * df
    return f_lo, f_hi, 1.0 / (2.0 * q), low_res


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("file", type=Path)
    ap.add_argument("--t", dest="t_range", default=None)
    ap.add_argument("--no-armed-crop", action="store_true")
    ap.add_argument("--band", default="15:55")
    ap.add_argument("--save", type=Path, default=None)
    args = ap.parse_args()

    if not args.file.is_file():
        sys.exit(f"No such file: {args.file}")

    band_lo, band_hi = parse_range(args.band, "band")
    if band_lo is None or band_hi is None or band_lo >= band_hi:
        sys.exit(f"--band needs both LO and HI, LO < HI (got {args.band!r}).")

    rows, spec, groups, derived = load_rows(args.file)
    if len(spec) not in CORNER_WIDTHS:
        sys.exit(f"{args.file.name}: this is an EDGE-format file (no om_x/y/z) "
                 f"-- this script is CORNER-only.")
    names = [n for n, _ in spec]
    if not all(a in names for a in AXES):
        sys.exit(f"{args.file.name}: missing om_x/y/z columns -- unexpected "
                 f"for a CORNER-format file, check the source.")

    arr = np.array(rows)
    t = (arr[:, 0] - arr[0, 0]) / 1000.0

    if args.t_range:
        lo, hi = parse_range(args.t_range, "t")
        keep = np.ones_like(t, dtype=bool)
        if lo is not None:
            keep &= t >= lo
        if hi is not None:
            keep &= t <= hi
        if not keep.any():
            sys.exit(f"No samples inside --t {args.t_range} "
                     f"(recording spans 0 to {t[-1]:.2f} s).")
        t, arr = t[keep], arr[keep]

    if not args.no_armed_crop and "armed" in names:
        armed = arr[:, names.index("armed")]
        a0, a1 = armed_crop(t, armed)
        if a1 - a0 < 10:
            print("NOTE: no armed>=10-sample stretch found -- analyzing the "
                  "full (possibly unarmed) window instead. Pass --t to crop "
                  "by hand, or check the file actually armed.", file=sys.stderr)
        else:
            t, arr = t[a0:a1], arr[a0:a1]

    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    dt_jitter = np.std(np.diff(t)) / dt if dt > 0 else float("nan")
    print(f"{args.file.name}: {len(t)} samples, {t[-1]-t[0]:.1f}s, "
          f"fs~{fs:.1f}Hz (median dt={dt*1000:.2f}ms, jitter={dt_jitter*100:.0f}% of dt)")
    if fs < 200:
        print(f"NOTE: fs~{fs:.0f}Hz is well under the firmware's nominal 500Hz "
              f"loop rate -- same sampling-rate discrepancy flagged in "
              f"docs/testing/Kalman-Filter-Rate-Estimator-Evaluation-2026-08-20"
              f".md Section 4. Nyquist here is ~{fs/2:.0f}Hz; keep --band "
              f"comfortably under that.", file=sys.stderr)
    if band_hi > fs / 2 * 0.9:
        print(f"WARNING: --band upper edge ({band_hi:.0f}Hz) is close to or "
              f"above this file's Nyquist (~{fs/2:.0f}Hz) -- narrow --band or "
              f"get a higher-rate capture.", file=sys.stderr)

    tu = np.arange(t[0], t[-1], dt)
    nperseg = min(2048, max(256, len(tu) // 4))

    results = {}
    print()
    print(f"{'axis':>5}  {'peak (Hz)':>10}  {'zeta est.':>10}  {'prominence':>10}")
    for name in AXES:
        col = names.index(name)
        y = np.interp(tu, t, arr[:, col])
        y = np.deg2rad(y) - np.deg2rad(y).mean()   # deg/s -> rad/s, remove DC
        freqs, psd = welch(y, fs=fs, nperseg=nperseg)

        band_mask = (freqs >= band_lo) & (freqs <= band_hi)
        if not band_mask.any():
            sys.exit(f"--band {args.band} has no frequency bins at fs={fs:.1f}Hz "
                     f"-- check --band is inside 0..{fs/2:.0f}Hz.")
        band_idx = np.where(band_mask)[0]
        peak_local = np.argmax(psd[band_idx])
        peak_i = band_idx[peak_local]
        peak_f = freqs[peak_i]
        median_in_band = np.median(psd[band_idx])
        prominence = psd[peak_i] / median_in_band if median_in_band > 0 else float("inf")

        f_lo, f_hi, zeta, low_res = half_power_zeta(freqs, psd, peak_i)
        if zeta is None:
            zeta_str = "n/a"
        else:
            zeta_str = f"{zeta:.4f}" + ("*" if low_res else "")
        edge_flag = " (at band edge!)" if peak_f in (freqs[band_idx[0]], freqs[band_idx[-1]]) else ""
        print(f"{name:>5}  {peak_f:10.2f}  {zeta_str:>10}  {prominence:10.1f}{edge_flag}")

        results[name] = dict(freqs=freqs, psd=psd, peak_i=peak_i, peak_f=peak_f,
                              f_lo=f_lo, f_hi=f_hi, zeta=zeta, low_res=low_res,
                              prominence=prominence)

    if any(r["low_res"] for r in results.values() if r["zeta"] is not None):
        print("  (* bandwidth is under 3 frequency bins wide -- resolution-")
        print("     limited, treat as a rough upper bound, not a measurement)")

    freqs_found = [r["peak_f"] for r in results.values()]
    spread = max(freqs_found) - min(freqs_found)
    rec_hz = float(np.median(freqs_found))

    # Report what the data actually says, not an artificially inflated
    # floor -- only clamp against the firmware's own hard limits (0 <
    # zeta < 1) and cap the top end since >0.5 stops being a resonance
    # this filter's shape is built for.
    zetas_found = [r["zeta"] for r in results.values() if r["zeta"] is not None]
    if zetas_found:
        rec_zeta = float(np.median(zetas_found))
        rec_zeta = min(max(rec_zeta, 0.005), 0.5)
    else:
        rec_zeta = None

    print()
    if spread > 5.0:
        print(f"NOTE: per-axis peaks span {spread:.1f}Hz ({min(freqs_found):.1f}-"
              f"{max(freqs_found):.1f}Hz) -- either a real per-axis difference "
              f"or a sign this capture's peak isn't a clean single mode. Median "
              f"used for the recommendation below, but low confidence.")

    low_prom = [n for n, r in results.items() if r["prominence"] < 3.0]
    if low_prom:
        print(f"NOTE: {', '.join(low_prom)} peak is not very prominent (<3x the "
              f"band median) -- may just be broadband noise, not a real mode. "
              f"Weight the recommendation accordingly.")

    print()
    print(f"Suggested starting point (type into Stage4_FixedOffset_Kalman.ino's "
          f"serial terminal):")
    print(f"    n{rec_hz:.1f}")
    if rec_zeta is not None:
        print(f"    z{rec_zeta:.3f}   (zeta estimate is rough -- see the "
              f"docstring's HOW THE ZETA ESTIMATE WORKS note)")
    else:
        print(f"    (no usable zeta estimate from this capture -- leave "
              f"gModeZeta at its 0.05 default)")
    print()
    print(f"Then watch mode_x/y/z_dps in telemetry for a plausible ~{rec_hz:.0f}Hz "
          f"oscillation before trusting anything downstream of it.")

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.rcParams.update({"figure.facecolor": "white", "axes.facecolor": "white",
                              "axes.grid": True, "grid.color": "#dedcd3",
                              "grid.linewidth": 0.7, "font.size": 10.5,
                              "axes.titlesize": 11, "legend.frameon": False,
                              "savefig.dpi": 150, "figure.dpi": 110})
        BLUE, ORANGE = "#2a78d6", "#eb6834"

        fig, axs = plt.subplots(3, 1, figsize=(8.5, 8.5), sharex=True)
        for ax, name in zip(axs, AXES):
            r = results[name]
            ax.semilogy(r["freqs"], r["psd"], color=BLUE, lw=1.0)
            ax.axvline(r["peak_f"], color=ORANGE, lw=1.2, ls="--",
                       label=f"peak {r['peak_f']:.1f}Hz")
            if r["f_lo"] is not None:
                ax.axvspan(r["f_lo"], r["f_hi"], color=ORANGE, alpha=0.15,
                           label=f"-3dB width (zeta~{r['zeta']:.3f})")
            ax.set_xlim(0, band_hi * 1.3)
            ax.set_ylabel("PSD ((rad/s)^2/Hz)")
            ax.set_title(f"{name}", loc="left", fontsize=10)
            ax.legend(loc="upper right", fontsize=8.5)
        axs[-1].set_xlabel("frequency (Hz)")
        fig.suptitle(f"{args.file.name} -- mode search in {args.band} Hz", fontsize=12)
        fig.tight_layout()
        fig.savefig(args.save)
        print(f"\nSaved plot to {args.save}")


if __name__ == "__main__":
    main()

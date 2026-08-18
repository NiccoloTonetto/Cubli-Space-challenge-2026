"""fft_tilt_analysis.py  --  FFT the tilt from a balance-test recording

Test 1 of Cube Fine-Tuning -- Test Plan.md (refines "Automatic Trim --
Replacing the Hardcoded IMU Offset" S8/S9): log the standing wheel speeds,
FFT the tilt, identify the mode, BEFORE touching gains or implementing the
trim. This script is that FFT step.

Takes either a telemetry CSV (from CornerBalance/CornerBalance_WiFi or
EdgeBalance/EdgeBalance_WiFi -- same loader plot_session_csv.py uses, so
format detection, delimiter sniffing and header handling all behave
identically to that tool) or a .mat file, extracts the tilt time series,
and reports/plots its frequency spectrum.

----------------------------------------------------------------------
WHY THIS SHAPE

The design's closed-loop poles are all near-critically-damped (see the
Test Plan's Test 1 table) -- if a real recording shows a sharp spectral
peak, that mode is UNMODELLED, and where the peak sits tells you what to
go look at (also BANDS below, which is what the code actually uses):

    ~0.56 Hz        wheel-unwinding mode -- Test Plan S8, lower omega_des
    1.1 - 1.6 Hz    tilt mode OR cable resonance -- freehanging cable at
                    100-200 mm reproduces exactly this band (100mm-
                    >1.58Hz, 150mm->1.29Hz, 200mm->1.11Hz). Test 3
                    (strain-relieve, re-run) first; if cables are already
                    relieved, a peak that persists here is more likely a
                    real tilt mode than the cable.
    2 - 3 Hz        estimator phase lag -- Test 2 (rate_cutoff_hz,
                    complementary filter tau, accelerometer gate, BMI270
                    ODR vs loop rate -- all four are a code read, not a
                    bench test, and free)
    > 5 Hz          loop rate or aliasing, not the plant -- check the
                    actual achieved loop period, not the nominal one
    ~141 Hz (or a   wheel vibration through the accelerometer -- only
     harmonic)      resolvable if the recording's sample rate clears
                    ~2x that (Nyquist), which the ~250-400 Hz control
                    loop telemetry rate does not by much margin; a raw
                    higher-rate IMU capture would be needed to see it
                    cleanly, this script will say so if it's out of range.

None of that is hardcoded as a verdict -- it's printed as a hint next to
each detected peak, because the actual diagnosis still needs the rest of
Section 8's measurement plan (swing test, drop test, spin-down) to confirm.

----------------------------------------------------------------------
USAGE

    python fft_tilt_analysis.py run.csv
    python fft_tilt_analysis.py run.csv --channel phi_x
    python fft_tilt_analysis.py run.csv --channel wheels
    python fft_tilt_analysis.py run.csv --t 5:65
    python fft_tilt_analysis.py run.csv --no-armed-crop
    python fft_tilt_analysis.py run.csv --save fft_tilt.png
    python fft_tilt_analysis.py run.mat --time-var t --theta-var theta

List what a CSV contains without analyzing it:

    python fft_tilt_analysis.py run.csv --list

----------------------------------------------------------------------
FLAGS

  --channel NAME   What to FFT. Default "tilt" (|phi| for a CORNER file,
                    phi_edge_deg for an EDGE file). Also accepts phi_x,
                    phi_y, phi_z (CORNER only, per-axis -- a mode may
                    dominate one axis and wash out in the magnitude), or
                    "wheels" (|rho|, CORNER only -- cross-check whether a
                    peak is a wheel-momentum mode rather than a tilt mode,
                    same distinction the 0.56 Hz band above is about).
  --t LO:HI        Crop to this time window (seconds from recording
                    start) before anything else. Same syntax as
                    plot_session_csv.py's --t.
  --no-armed-crop  CSV inputs default to auto-cropping to the LONGEST
                    continuous armed=1 stretch before analysis -- an
                    arm/disarm transition is a step discontinuity that
                    smears energy across the whole spectrum, and "the
                    oscillation during a 60 s balance test" means the
                    balance test, not the seconds of hand-holding before
                    it armed. Pass this to analyze the raw file instead
                    (or use --t to pick the window by hand).
  --window NAME    hann (default), hamming, blackman, or rect (no window
                    -- only use rect if you already trimmed to an exact
                    integer number of cycles by hand, otherwise it leaks).
  --peaks N        How many spectral peaks to report, most prominent
                    first. Default 8.
  --fmin / --fmax  Restrict peak search to this frequency range, Hz.
                    Default 0.05 to Nyquist (excludes DC).
  --save FILE      Write the figure to FILE instead of opening a window.
  --list           Print what channels/format the file has, then exit.

  --time-var NAME   .mat only: variable holding the time vector. Auto-
                    detected (t, time, t_ms, t_s) if omitted.
  --theta-var NAME  .mat only: variable holding the tilt. Auto-detected
                    (theta, theta_deg, phi, tilt) if omitted. If the
                    matched variable is Nx3, its row-wise norm is used.

----------------------------------------------------------------------
A NOTE ON SAMPLE-RATE JITTER

Real telemetry (Serial or WiFi/UDP) does not arrive on a perfectly even
clock. An FFT assumes it does. This script resamples onto a uniform grid
at the median sample rate via linear interpolation before transforming,
and prints the observed jitter -- if it's large (WiFi drops, USB stalls),
that's worth knowing about independent of the tilt result.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_session_csv import (   # noqa: E402  (path must be set up first)
    load_rows, EDGE_COLS, CORNER_COLS, CORNER_TRIM_COLS, CORNER_ENDURANCE_COLS,
    CORNER_DERIVED,
)

# CORNER and CORNER_TRIM (Stage4_AutoTrim.ino's format, 21 CORNER fields +
# 5 trim fields) share the same first 21 columns, so every derived formula
# below (which only indexes 1-9) and every --channel option works
# unmodified on either -- only the width used to tell them apart from EDGE
# differs.
CORNER_WIDTHS = (len(CORNER_COLS), len(CORNER_TRIM_COLS), len(CORNER_ENDURANCE_COLS))

# ---------------------------------------------------------------------------
# Diagnostic frequency bands -- Cube Fine-Tuning Test Plan.md, Test 1's
# table (refines "Automatic Trim" S6: splits its single ">3 Hz" band into
# "estimator phase lag" vs "loop rate/aliasing", which matter differently
# enough to send you to different tests). "Go to" column folded into each
# hint so a peak points straight at the next test, not just a label.
# ---------------------------------------------------------------------------

BANDS = [
    # (lo_hz, hi_hz, label)
    (0.45, 0.70, "wheel-unwinding mode -- lower omega_des (S8), not a fault"),
    (1.10, 1.60, "tilt mode OR cable resonance (100-200mm free length "
                 "spans 1.11-1.58 Hz) -- Test 3 (strain-relieve, re-run) "
                 "first; if cables are already relieved, lean toward a "
                 "real tilt mode"),
    (2.00, 3.00, "estimator phase lag -- Test 2 (rate_cutoff_hz, complementary "
                 "filter tau, accel gate, BMI270 ODR vs loop rate)"),
    (5.00, 120.0, "loop rate or aliasing -- check the actual loop period, "
                  "not the plant"),
    (120.0, 165.0, "wheel vibration through the accelerometer (needs a "
                   "high-rate capture to resolve cleanly)"),
]


def band_hint(freq_hz):
    for lo, hi, label in BANDS:
        if lo <= freq_hz <= hi:
            return label
    return None


# ---------------------------------------------------------------------------
# Loading -- CSV (via plot_session_csv's loader, same format detection)
# ---------------------------------------------------------------------------

CHANNEL_UNITS = {
    "tilt": "deg", "phi_x": "deg", "phi_y": "deg", "phi_z": "deg",
    "wheels": "rad/s",
}


def armed_crop(t, armed):
    """Returns (start, end) indices of the LONGEST contiguous armed==1 run.

    An arm/disarm edge is a step in commanded torque -- left in, it puts a
    broadband transient into every FFT bin, which is exactly what "FFT the
    60 s balance test" is not asking for."""
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


def load_csv_channel(path, channel, t_range, do_armed_crop):
    rows, spec, groups, derived = load_rows(path)
    is_corner = len(spec) in CORNER_WIDTHS

    if channel == "tilt":
        if is_corner:
            unit, fn = CORNER_DERIVED["|phi|"]
            name, want = "|phi|", fn
        else:
            name, unit, want = "phi_edge_deg", "deg", None
    elif channel == "wheels":
        if not is_corner:
            sys.exit("--channel wheels needs a CORNER-format file "
                      "(EDGE has no rho_x/y/z).")
        unit, fn = CORNER_DERIVED["|rho|"]
        name, want = "|rho|", fn
    elif channel in ("phi_x", "phi_y", "phi_z"):
        if not is_corner:
            sys.exit(f"--channel {channel} needs a CORNER-format file.")
        name, unit, want = channel, "deg", None
    else:
        sys.exit(f"Unknown --channel {channel!r}. Use tilt, wheels, "
                  f"phi_x, phi_y, or phi_z.")

    t0 = rows[0][0]
    t_all = np.array([(r[0] - t0) / 1000.0 for r in rows])
    if want is not None:
        y_all = np.array([want(r) for r in rows])
    else:
        col = next(i for i, (n, _) in enumerate(spec) if n == name)
        y_all = np.array([r[col] for r in rows])

    armed_col = next((i for i, (n, _) in enumerate(spec) if n == "armed"), None)
    armed_all = (np.array([r[armed_col] for r in rows])
                 if armed_col is not None else None)

    lo, hi = t_range if t_range else (None, None)
    keep = np.ones_like(t_all, dtype=bool)
    if lo is not None:
        keep &= t_all >= lo
    if hi is not None:
        keep &= t_all <= hi
    t_all, y_all = t_all[keep], y_all[keep]
    if armed_all is not None:
        armed_all = armed_all[keep]

    if do_armed_crop and armed_all is not None:
        s, e = armed_crop(t_all, armed_all)
        if e - s < 10:
            print("  note: no armed>=10-sample stretch found -- analyzing "
                  "the full (possibly unarmed) window instead. Pass --t to "
                  "crop by hand, or check the file actually armed.")
        else:
            dropped_lo, dropped_hi = s, len(t_all) - e
            t_all, y_all = t_all[s:e], y_all[s:e]
            print(f"  armed-cropped to t={t_all[0]:.2f}..{t_all[-1]:.2f} s "
                  f"({len(t_all)} samples; dropped {dropped_lo} before, "
                  f"{dropped_hi} after)")

    return t_all, y_all, name, unit


def list_csv(path):
    rows, spec, groups, derived = load_rows(path)
    is_corner = len(spec) in CORNER_WIDTHS
    print(f"format: {'CORNER' if is_corner else 'EDGE'}")
    print("channels for --channel:")
    print("  tilt      -- |phi| (corner) or phi_edge_deg (edge)")
    if is_corner:
        print("  phi_x, phi_y, phi_z -- per-axis tilt, deg")
        print("  wheels    -- |rho|, rad/s")


# ---------------------------------------------------------------------------
# Loading -- .mat
# ---------------------------------------------------------------------------

TIME_VAR_CANDIDATES = ["t", "time", "t_ms", "t_s", "T"]
THETA_VAR_CANDIDATES = ["theta", "theta_deg", "tilt", "phi", "Theta"]


def load_mat_channel(path, time_var, theta_var):
    try:
        from scipy.io import loadmat
    except ImportError:
        sys.exit(".mat support needs scipy (pip install scipy). "
                 "CSV inputs don't need it.")

    data = loadmat(path, squeeze_me=True, struct_as_record=False)
    keys = [k for k in data if not k.startswith("__")]

    def pick(explicit, candidates, what):
        if explicit:
            if explicit not in data:
                sys.exit(f"{explicit!r} not in {path.name}. "
                         f"Variables present: {', '.join(keys)}")
            return explicit
        for c in candidates:
            if c in data:
                return c
        sys.exit(f"Couldn't auto-detect the {what} variable in {path.name}. "
                 f"Variables present: {', '.join(keys)}\n"
                 f"Pass --{'time' if what == 'time' else 'theta'}-var "
                 f"NAME explicitly.")

    tname = pick(time_var, TIME_VAR_CANDIDATES, "time")
    thname = pick(theta_var, THETA_VAR_CANDIDATES, "theta")
    print(f"  .mat: using {tname!r} for time, {thname!r} for theta "
          f"(out of: {', '.join(keys)})")

    t_all = np.asarray(data[tname], dtype=float).ravel()
    theta_raw = np.asarray(data[thname], dtype=float)
    if theta_raw.ndim == 2 and theta_raw.shape[1] == 3:
        y_all = np.linalg.norm(theta_raw, axis=1)
        unit_note = " (norm of an Nx3 array -- assumed [x,y,z], deg)"
    elif theta_raw.ndim == 2 and theta_raw.shape[0] == 3:
        y_all = np.linalg.norm(theta_raw, axis=0)
        unit_note = " (norm of a 3xN array -- assumed [x,y,z], deg)"
    else:
        y_all = theta_raw.ravel()
        unit_note = ""
    print(f"  theta variable shape {theta_raw.shape}{unit_note}")

    # t_ms-style variables are large integers; treat >1e4 typical magnitude
    # as milliseconds and convert, same convention as the CSV loader.
    if np.median(np.abs(t_all)) > 1.0e4:
        t_all = t_all / 1000.0
    t_all = t_all - t_all[0]

    return t_all, y_all, thname, "deg"


# ---------------------------------------------------------------------------
# Resampling, windowing, FFT
# ---------------------------------------------------------------------------

WINDOWS = {
    "hann": np.hanning, "hamming": np.hamming,
    "blackman": np.blackman, "rect": lambda n: np.ones(n),
}


def resample_uniform(t, y):
    dt_all = np.diff(t)
    dt_med = float(np.median(dt_all))
    jitter = float(np.std(dt_all) / dt_med) if dt_med > 0 else float("nan")
    fs = 1.0 / dt_med
    print(f"  {len(t)} samples, {t[-1] - t[0]:.2f} s, "
          f"median dt={dt_med * 1000:.2f} ms (fs~={fs:.1f} Hz), "
          f"jitter(std/median)={jitter:.1%}")
    if jitter > 0.20:
        print("  WARNING: >20% timing jitter -- resampling will smear some "
              "energy across nearby bins. Treat close-together peaks with "
              "extra suspicion.")

    n = int(np.floor((t[-1] - t[0]) / dt_med)) + 1
    t_uniform = t[0] + np.arange(n) * dt_med
    y_uniform = np.interp(t_uniform, t, y)
    return t_uniform, y_uniform, fs


def compute_fft(y, fs, window_name):
    n = len(y)
    y_detrend = y - np.mean(y)
    win = WINDOWS[window_name](n)
    # Coherent gain correction so peak amplitude reads in real units
    # (deg, rad/s) rather than "whatever the window scaled it to".
    coherent_gain = np.mean(win)
    y_windowed = y_detrend * win

    spectrum = np.fft.rfft(y_windowed)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    amplitude = np.abs(spectrum) / (n * coherent_gain)
    amplitude[1:] *= 2.0   # single-sided, DC bin excepted
    return freqs, amplitude


def find_peaks(freqs, amplitude, fmin, fmax, n_peaks):
    """Local-maxima peak pick, no scipy dependency -- this only needs to be
    good enough to point you at the right bin, not publication-grade."""
    mask = (freqs >= fmin) & (freqs <= fmax)
    idx = np.where(mask)[0]
    if len(idx) < 3:
        return []
    candidates = []
    for i in idx[1:-1]:
        if amplitude[i] > amplitude[i - 1] and amplitude[i] > amplitude[i + 1]:
            candidates.append(i)
    candidates.sort(key=lambda i: amplitude[i], reverse=True)

    # Suppress near-duplicate peaks (one broad hump's shoulders showing up as
    # several "peaks") without merging genuinely distinct nearby modes -- the
    # 0.56 Hz and 1.3-1.6 Hz diagnostic bands this script exists to tell apart
    # are under 1 Hz apart, so spacing must scale with the SEARCHED range
    # (fmax-fmin), not the full-spectrum Nyquist (a >100 Hz recording would
    # otherwise demand >2 Hz between any two reported peaks and silently
    # merge exactly the two bands that matter most).
    freq_res = freqs[1] - freqs[0] if len(freqs) > 1 else 0.0
    min_spacing = max(5 * freq_res, 0.01 * (fmax - fmin))
    picked = []
    for i in candidates:
        if all(abs(freqs[i] - freqs[j]) > min_spacing for j in picked):
            picked.append(i)
        if len(picked) >= n_peaks:
            break
    return picked


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def parse_t_range(s):
    if s is None:
        return None
    lo_s, _, hi_s = s.partition(":")
    lo = float(lo_s) if lo_s else None
    hi = float(hi_s) if hi_s else None
    return (lo, hi)


def main():
    ap = argparse.ArgumentParser(
        description="FFT the tilt from a balance-test recording (CSV or .mat).")
    ap.add_argument("file", type=Path)
    ap.add_argument("--channel", default="tilt")
    ap.add_argument("--t", dest="t_range", default=None,
                    help="LO:HI seconds, either side optional")
    ap.add_argument("--no-armed-crop", action="store_true")
    ap.add_argument("--window", choices=list(WINDOWS), default="hann")
    ap.add_argument("--peaks", type=int, default=8)
    ap.add_argument("--fmin", type=float, default=0.05)
    ap.add_argument("--fmax", type=float, default=None)
    ap.add_argument("--save", type=Path, default=None)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--time-var", default=None)
    ap.add_argument("--theta-var", default=None)
    args = ap.parse_args()

    if not args.file.is_file():
        sys.exit(f"No such file: {args.file}")

    is_mat = args.file.suffix.lower() == ".mat"

    if args.list:
        if is_mat:
            sys.exit("--list is CSV-only; for .mat, inspect with "
                     "scipy.io.loadmat directly, this script auto-detects.")
        list_csv(args.file)
        return

    t_range = parse_t_range(args.t_range)

    if is_mat:
        t, y, chan_name, unit = load_mat_channel(
            args.file, args.time_var, args.theta_var)
        if t_range:
            lo, hi = t_range
            keep = np.ones_like(t, dtype=bool)
            if lo is not None:
                keep &= t >= lo
            if hi is not None:
                keep &= t <= hi
            t, y = t[keep], y[keep]
    else:
        t, y, chan_name, unit = load_csv_channel(
            args.file, args.channel, t_range, not args.no_armed_crop)

    if len(t) < 16:
        sys.exit(f"Only {len(t)} samples after cropping -- too short for a "
                 f"meaningful FFT. Widen --t or drop --no-armed-crop.")

    t_uniform, y_uniform, fs = resample_uniform(t, y)
    nyquist = fs / 2.0
    fmax = args.fmax if args.fmax is not None else nyquist
    if fmax > nyquist:
        print(f"  note: --fmax {fmax} exceeds Nyquist {nyquist:.1f} Hz for "
              f"this recording's sample rate -- clamping.")
        fmax = nyquist

    freqs, amplitude = compute_fft(y_uniform, fs, args.window)
    freq_res = fs / len(y_uniform)
    print(f"  frequency resolution: {freq_res:.4f} Hz "
          f"(={1.0/freq_res:.1f} s window)")

    peaks = find_peaks(freqs, amplitude, args.fmin, fmax, args.peaks)

    print(f"\nTop {len(peaks)} peak(s) in {chan_name} "
          f"({args.fmin:.2f}-{fmax:.1f} Hz):")
    print(f"  {'freq (Hz)':>10}  {'period (s)':>10}  {'amp (' + unit + ')':>14}  hint")
    for i in peaks:
        f, a = freqs[i], amplitude[i]
        hint = band_hint(f) or "outside the S6 diagnostic bands"
        print(f"  {f:10.3f}  {1.0/f if f > 0 else float('inf'):10.3f}  "
              f"{a:14.5f}  {hint}")
    if not peaks:
        print("  (none found -- signal may be too flat/quiet, or "
              "--fmin/--fmax excludes everything present)")

    # ---- plot ----
    fig, (ax_t, ax_f) = plt.subplots(2, 1, figsize=(10, 7))

    ax_t.plot(t_uniform, y_uniform, lw=0.8)
    ax_t.set_xlabel("t (s)")
    ax_t.set_ylabel(f"{chan_name} ({unit})")
    ax_t.set_title(f"{args.file.name} -- {chan_name}, "
                   f"{t_uniform[-1] - t_uniform[0]:.1f} s @ ~{fs:.0f} Hz")
    ax_t.grid(alpha=0.3)

    ax_f.plot(freqs, amplitude, lw=0.9, color="tab:blue")
    ax_f.set_xlim(args.fmin, fmax)
    ax_f.set_yscale("log")
    ax_f.set_xlabel("frequency (Hz)")
    ax_f.set_ylabel(f"amplitude ({unit})")
    ax_f.grid(alpha=0.3, which="both")

    band_colors = ["tab:orange", "tab:green", "tab:red", "tab:purple"]
    for (lo, hi, label), color in zip(BANDS, band_colors):
        if hi < args.fmin or lo > fmax:
            continue
        ax_f.axvspan(max(lo, args.fmin), min(hi, fmax), color=color, alpha=0.08)

    for i in peaks:
        f = freqs[i]
        ax_f.axvline(f, color="k", lw=0.6, ls="--", alpha=0.6)
        ax_f.annotate(f"{f:.2f} Hz", (f, amplitude[i]),
                     textcoords="offset points", xytext=(3, 4), fontsize=8)

    ax_f.set_title("Spectrum (shaded bands = Automatic Trim S6 diagnostic zones)")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"\nSaved plot to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

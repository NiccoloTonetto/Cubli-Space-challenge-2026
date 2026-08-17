"""plot_session_csv.py  --  offline plotter for a saved session CSV

Takes a telemetry CSV recorded by either WiFi build's live script (or by
the USB-side telemetry_python.py) and plots whichever variables you want.
Handles BOTH wire formats, auto-detected from the column count:

    10 columns -> EDGE   (EdgeBalance / EdgeBalance_WiFi)
    21 columns -> CORNER (CornerBalance / CornerBalance_WiFi)

A header row is optional -- it is detected and skipped if present, and
the column names come from the detected format either way, so a headerless
raw capture plots identically to a script-saved file.

TAB-DELIMITED captures (a SERIALMONITORMODE log pasted out of a Serial
Monitor) are also accepted: the delimiter is sniffed per file.

----------------------------------------------------------------------
USAGE

  Interactive -- no idea yet what you want to see:

      python plot_session_csv.py run.csv

  Opens the plot with every trace available as a checkbox down the left
  side. Tick and untick to build the view you want; the axes rescale as
  you go. Nothing to re-run.

  Direct -- you know what you want:

      python plot_session_csv.py run.csv --cols phi_x,phi_y,|phi|
      python plot_session_csv.py run.csv --cols tilt
      python plot_session_csv.py run.csv --cols wheels,torque --t 5:12
      python plot_session_csv.py run.csv --cols tilt --save tilt.png

  List what's available in a given file:

      python plot_session_csv.py run.csv --list

----------------------------------------------------------------------
FLAGS

  --cols A,B,C   Columns and/or group names to plot. Groups expand to
                 their members. Omit for the interactive checkbox picker.
  --t LO:HI      Crop to a time range in seconds from the start of the
                 recording. Either side may be omitted: "5:", ":12".
  --save FILE    Write the figure to FILE instead of opening a window.
  --overlay      Force every trace onto one axis. By default traces are
                 grouped by UNIT onto stacked subplots sharing the time
                 axis -- deg, deg/s, rad/s and N*m have wildly different
                 scales and crushing them together hides everything but
                 the largest.
  --no-armed     Don't shade the armed regions.
  --list         Print available columns and groups, then exit.

----------------------------------------------------------------------
DERIVED COLUMNS

Computed on load, not present in the CSV:

  |phi|   norm of the phi vector, deg  -- CORNER only. This is the
          quantity the firmware's arm gate tests, so it is the one to
          look at when asking "why did a1 get refused?". Plotted with
          the 0.5 deg gate drawn as a dashed reference line.
  |om|    norm of the body rate vector, deg/s   -- CORNER only
  |rho|   norm of the wheel speed vector, rad/s -- CORNER only

----------------------------------------------------------------------
ARMED SHADING

Wherever the armed column is 1, the background is shaded. That makes the
part of the trace under active control immediately distinguishable from
coasting, and puts an obvious edge at the moment a trip disarmed the
controller. Disable with --no-armed.

Requires: pip install matplotlib
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons

ARM_GATE_DEG = 0.5   # matches kArmGate in both firmware builds

# ---------------------------------------------------------------------------
# Formats. Each column carries its unit, which is what drives subplot grouping.
# ---------------------------------------------------------------------------

EDGE_COLS = [
    ("t_ms", None),
    ("phi_edge_deg", "deg"),
    ("om_edge_dps", "deg/s"),
    ("tau_Nm", "N*m"),
    ("tau_cmd_Nm", "N*m"),
    ("armed", "flag"),
    ("gain_scale", "flag"),
    ("wheel_omega_lp", "rad/s"),
    ("wheel_pos", "rev"),
    ("wheel_vel", "rev/s"),
]

CORNER_COLS = [
    ("t_ms", None),
    ("phi_x", "deg"), ("phi_y", "deg"), ("phi_z", "deg"),
    ("om_x", "deg/s"), ("om_y", "deg/s"), ("om_z", "deg/s"),
    ("rho_x", "rad/s"), ("rho_y", "rad/s"), ("rho_z", "rad/s"),
    ("rho_x_lp", "rad/s"), ("rho_y_lp", "rad/s"), ("rho_z_lp", "rad/s"),
    ("tau_x", "N*m"), ("tau_y", "N*m"), ("tau_z", "N*m"),
    ("tau_cmd_x", "N*m"), ("tau_cmd_y", "N*m"), ("tau_cmd_z", "N*m"),
    ("armed", "flag"),
    ("gain_scale", "flag"),
]

EDGE_GROUPS = {
    "tilt":   ["phi_edge_deg"],
    "rates":  ["om_edge_dps"],
    "wheels": ["wheel_omega_lp", "wheel_vel"],
    "torque": ["tau_Nm", "tau_cmd_Nm"],
    "all":    ["phi_edge_deg", "om_edge_dps", "tau_Nm", "tau_cmd_Nm",
               "wheel_omega_lp"],
}

CORNER_GROUPS = {
    "tilt":   ["phi_x", "phi_y", "phi_z", "|phi|"],
    "rates":  ["om_x", "om_y", "om_z"],
    "wheels": ["rho_x", "rho_y", "rho_z",
               "rho_x_lp", "rho_y_lp", "rho_z_lp"],
    "torque": ["tau_x", "tau_y", "tau_z",
               "tau_cmd_x", "tau_cmd_y", "tau_cmd_z"],
    "all":    ["phi_x", "phi_y", "phi_z", "|phi|",
               "om_x", "om_y", "om_z",
               "rho_x_lp", "rho_y_lp", "rho_z_lp",
               "tau_x", "tau_y", "tau_z"],
}

# Derived series: name -> (unit, function of the raw row list)
CORNER_DERIVED = {
    "|phi|": ("deg", lambda r: (r[1] ** 2 + r[2] ** 2 + r[3] ** 2) ** 0.5),
    "|om|":  ("deg/s", lambda r: (r[4] ** 2 + r[5] ** 2 + r[6] ** 2) ** 0.5),
    "|rho|": ("rad/s", lambda r: (r[7] ** 2 + r[8] ** 2 + r[9] ** 2) ** 0.5),
}

# Stacking order for the by-unit subplot layout, most-watched first.
UNIT_ORDER = ["deg", "deg/s", "rad/s", "N*m", "rev", "rev/s", "flag"]
UNIT_TITLES = {
    "deg": "Tilt", "deg/s": "Body rate", "rad/s": "Wheel speed",
    "N*m": "Torque", "rev": "Wheel position", "rev/s": "Wheel velocity",
    "flag": "Armed / gain",
}


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def sniff_delimiter(path, sample_lines=50):
    """Comma for PLOTMODE, tab for a pasted SERIALMONITORMODE capture.

    Decided by majority vote over a sample of data lines rather than by the
    first line containing either character: '#' banners and free-text firmware
    messages routinely contain commas, so a single early comma must not be
    able to mis-detect a tab-delimited capture as CSV."""
    commas = tabs = seen = 0
    with path.open(newline="") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            commas += line.count(",")
            tabs += line.count("\t")
            seen += 1
            if seen >= sample_lines:
                break
    return "\t" if tabs > commas else ","


def load_rows(path):
    """Returns (rows, spec, groups, derived) or exits with a diagnostic.

    Format is decided by the modal field count across the file rather than by
    the first line alone -- a partial first packet from a UDP capture would
    otherwise pick the wrong format for the whole session."""
    delim = sniff_delimiter(path)
    raw_lines = []
    with path.open(newline="") as f:
        for parts in csv.reader(f, delimiter=delim):
            if parts and not parts[0].lstrip().startswith("#"):
                raw_lines.append(parts)

    if not raw_lines:
        sys.exit(f"No data rows found in {path}.")

    counts = {}
    for parts in raw_lines:
        counts[len(parts)] = counts.get(len(parts), 0) + 1
    width = max(counts, key=counts.get)

    if width == len(EDGE_COLS):
        spec, groups, derived, kind = EDGE_COLS, EDGE_GROUPS, {}, "EDGE"
    elif width == len(CORNER_COLS):
        spec, groups, derived, kind = (CORNER_COLS, CORNER_GROUPS,
                                       CORNER_DERIVED, "CORNER")
    else:
        sys.exit(f"{path.name}: rows are {width} fields wide; expected "
                 f"{len(EDGE_COLS)} (edge) or {len(CORNER_COLS)} (corner).")

    rows, skipped = [], 0
    for parts in raw_lines:
        if len(parts) != width:
            skipped += 1
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:
            skipped += 1     # header row, or a corrupted line

    if not rows:
        sys.exit(f"No numeric {width}-field rows found in {path}.")

    print(f"{path.name}: {kind} format, {len(rows)} samples, "
          f"{(rows[-1][0] - rows[0][0]) / 1000.0:.2f} s of data.")
    if skipped:
        print(f"  (skipped {skipped} header/malformed row(s))")

    return rows, spec, groups, derived


def build_series(rows, spec, derived, t_range):
    """Flattens rows into {name: (unit, values)} plus a shared time axis in
    seconds, cropped to t_range if given."""
    t0 = rows[0][0]
    t_all = [(r[0] - t0) / 1000.0 for r in rows]

    lo, hi = t_range if t_range else (None, None)
    keep = [i for i, t in enumerate(t_all)
            if (lo is None or t >= lo) and (hi is None or t <= hi)]
    if not keep:
        sys.exit(f"No samples inside --t {lo}:{hi} "
                 f"(recording spans 0 to {t_all[-1]:.2f} s).")

    t = [t_all[i] for i in keep]
    series = {}
    for col, (name, unit) in enumerate(spec):
        if unit is None:
            continue
        series[name] = (unit, [rows[i][col] for i in keep])
    for name, (unit, fn) in derived.items():
        series[name] = (unit, [fn(rows[i]) for i in keep])
    return t, series


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def base_signal(name):
    """The underlying signal a trace belongs to, so a measurement and its
    derived companion share a colour: rho_x_lp -> rho_x, tau_cmd_x -> tau_x,
    tau_cmd_Nm -> tau_Nm, wheel_omega_lp -> wheel_omega."""
    if name.endswith("_lp"):
        name = name[:-3]
    return name.replace("tau_cmd_", "tau_")


def resolve_cols(requested, series, groups):
    """Expands group names, validates the rest, preserves the given order and
    drops duplicates."""
    out = []
    for token in requested:
        token = token.strip()
        if not token:
            continue
        names = groups.get(token, [token])
        for name in names:
            if name not in series:
                sys.exit(f"Unknown column or group: {token!r}. "
                         f"Run with --list to see what this file has.")
            if name not in out:
                out.append(name)
    return out


def armed_spans(t, series):
    """Contiguous [start, end] time ranges where armed == 1."""
    if "armed" not in series:
        return []
    armed = series["armed"][1]
    spans, start = [], None
    for i, a in enumerate(armed):
        if a >= 0.5 and start is None:
            start = t[i]
        elif a < 0.5 and start is not None:
            spans.append((start, t[i]))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def make_figure(t, series, cols, overlay, shade, title, all_units):
    """Builds the axes and returns (fig, {name: Line2D}, axes).

    Every series in `all_units`' scope gets a Line2D so the interactive picker
    can toggle any of them without rebuilding the figure; the ones not in
    `cols` simply start hidden. `all_units=True` (interactive mode) gives every
    unit present in the file its own subplot, so nothing is unreachable from
    the checkbox panel; `--cols` mode only builds the subplots it needs."""
    plotted = cols if cols else []
    scope = set(series) if all_units else set(plotted)
    units = ([None] if overlay
             else sorted({series[n][0] for n in scope} or {"deg"},
                         key=lambda u: UNIT_ORDER.index(u)
                         if u in UNIT_ORDER else 99))

    fig, axes = plt.subplots(len(units), 1, sharex=True,
                             figsize=(11, 2.6 * len(units) + 1.6),
                             squeeze=False)
    axes = [a[0] for a in axes]
    fig.canvas.manager.set_window_title(title)

    ax_for_unit = {u: axes[i] for i, u in enumerate(units)}

    spans = armed_spans(t, series) if shade else []
    for ax in axes:
        for lo, hi in spans:
            ax.axvspan(lo, hi, color="0.85", zorder=0)
        ax.grid(alpha=0.25)
    axes[-1].set_xlabel("t (s)")

    for i, u in enumerate(units):
        axes[i].set_ylabel("mixed units" if u is None else u)
        axes[i].set_title("All traces" if u is None else UNIT_TITLES.get(u, u),
                          fontsize=10)

    # Colour by BASE signal, not by name, so rho_x/rho_x_lp and
    # tau_x/tau_cmd_x come out the same colour with the derived one dashed.
    # A commanded trace peeling away from its actual is the thing you are
    # looking for in these plots; different colours hide it.
    palette = {b: f"C{i % 10}"
               for i, b in enumerate(sorted({base_signal(n) for n in scope}))}

    lines = {}
    for name in sorted(scope):
        unit, values = series[name]
        ax = ax_for_unit.get(None if overlay else unit)
        if ax is None:
            continue
        derived_trace = name.endswith("_lp") or "cmd" in name
        (line,) = ax.plot(t, values, label=name,
                          color=palette[base_signal(name)],
                          linestyle="--" if derived_trace else "-",
                          alpha=0.75 if derived_trace else 1.0)
        line.set_visible(name in plotted)
        lines[name] = line

    # The arm gate is only meaningful next to |phi|, and only on the deg axis.
    if "|phi|" in lines and not overlay and "deg" in ax_for_unit:
        ax_for_unit["deg"].axhline(ARM_GATE_DEG, color="r", linestyle=":",
                                   linewidth=1.2,
                                   label=f"arm gate ({ARM_GATE_DEG} deg)")

    # A zero-width span purely as a legend proxy for the real shading, which
    # is a background Polygon and so never picked up by the line-based legend
    # rebuild in refresh() below.
    armed_proxy = (axes[0].axvspan(t[0], t[0], color="0.85", label="armed")
                   if spans else None)

    return fig, lines, axes, armed_proxy


def refresh(axes, armed_proxy=None):
    """Rescale y and rebuild legends from the VISIBLE traces only.

    Both halves matter after a checkbox toggle: an axis left stretched to fit
    a hidden trace flattens everything you can still see, and a legend still
    listing hidden traces is actively misleading about what is on screen."""
    for ax in axes:
        vis = [ln for ln in ax.get_lines()
               if ln.get_visible() and not ln.get_label().startswith("_")]
        ys = [y for ln in vis for y in ln.get_ydata()]
        if ys:
            lo, hi = min(ys), max(ys)
            pad = (hi - lo) * 0.08 or (abs(hi) * 0.1 or 1.0)
            ax.set_ylim(lo - pad, hi + pad)

        handles = list(vis)
        if armed_proxy is not None and ax is axes[0]:
            handles.append(armed_proxy)
        if handles:
            ax.legend(handles=handles, loc="upper left", fontsize=7, ncol=3)
        elif ax.get_legend():
            ax.get_legend().remove()


def main():
    p = argparse.ArgumentParser(
        description="Plot a saved Cubli telemetry CSV (edge or corner).")
    p.add_argument("csv", help="path to the CSV file")
    p.add_argument("--cols", default=None,
                   help="comma-separated columns and/or group names; "
                        "omit for the interactive checkbox picker")
    p.add_argument("--t", default=None, metavar="LO:HI",
                   help="crop to a time range in seconds, e.g. 5:12, 5:, :12")
    p.add_argument("--save", default=None, metavar="FILE",
                   help="write the figure to FILE instead of opening a window")
    p.add_argument("--overlay", action="store_true",
                   help="put every trace on one axis instead of grouping by unit")
    p.add_argument("--no-armed", action="store_true",
                   help="don't shade the armed regions")
    p.add_argument("--list", action="store_true",
                   help="print available columns and groups, then exit")
    args = p.parse_args()

    path = Path(args.csv)
    if not path.is_file():
        sys.exit(f"File not found: {path}")

    rows, spec, groups, derived = load_rows(path)

    t_range = None
    if args.t:
        lo_s, _, hi_s = args.t.partition(":")
        t_range = (float(lo_s) if lo_s else None, float(hi_s) if hi_s else None)

    t, series = build_series(rows, spec, derived, t_range)

    if args.list:
        print("\nColumns:")
        for name in sorted(series):
            unit, values = series[name]
            print(f"  {name:<16} {unit:<6} "
                  f"min {min(values):>10.4f}  max {max(values):>10.4f}  "
                  f"final {values[-1]:>10.4f}")
        print("\nGroups:")
        for g, members in groups.items():
            print(f"  {g:<10} {', '.join(members)}")
        return

    interactive = args.cols is None
    cols = (resolve_cols(args.cols.split(","), series, groups) if args.cols
            else resolve_cols(groups["all"], series, groups))

    fig, lines, axes, armed_proxy = make_figure(
        t, series, cols, args.overlay, not args.no_armed, path.name,
        all_units=interactive)
    refresh(axes, armed_proxy)

    if args.save:
        fig.tight_layout()
        fig.savefig(args.save, dpi=150)
        print(f"Wrote {args.save}")
        return

    if interactive:
        # Checkbox panel down the left. Reserve space for it rather than
        # overlaying, so it never sits on top of the traces.
        fig.subplots_adjust(left=0.20, right=0.98, top=0.95, bottom=0.07)
        names = sorted(lines)
        ax_check = fig.add_axes([0.005, 0.07, 0.155, 0.88])
        ax_check.set_title("traces", fontsize=8)
        check = CheckButtons(ax_check, names,
                             [lines[n].get_visible() for n in names])
        for lbl in check.labels:
            lbl.set_fontsize(7)

        def toggle(name):
            lines[name].set_visible(not lines[name].get_visible())
            refresh(axes, armed_proxy)
            fig.canvas.draw_idle()

        check.on_clicked(toggle)
        # Keep a reference on the figure: CheckButtons stops responding if it
        # is garbage-collected when main() returns into plt.show().
        fig._trace_picker = check
        print("Tick/untick traces in the panel on the left.")
    else:
        fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()

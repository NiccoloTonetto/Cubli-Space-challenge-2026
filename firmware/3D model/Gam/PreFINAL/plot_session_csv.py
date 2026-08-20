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

  No arguments at all -- the usual case after a recording session:

      python plot_session_csv.py

  Asks which file to open:

      d  default -- the newest recording found (that is, the one the session
                    you just finished wrote)
      f  file    -- list them and type a filename, a path, or a list number
      q  quit

  Recordings live in FINAL/telemetry/, split by the mode that wrote them:

      telemetry/plot/     telemetry_edge_*.csv    -- PLOTMODE, comma-delimited,
                          telemetry_corner_*.csv     from telemetry_python_wifi.py
                                                     and _corner.py respectively
      telemetry/serial/   session_*.log    -- SERIALMONITORMODE, tab-delimited,
                          from terminal_wifi.py

  The picker lists the two apart and labels which is which, since only a
  Stage 5 serial log is wide enough to plot (21 columns); Stage 1's 13 and
  Stages 2/3's 18 are outside the auto-detection below. Every line also
  carries EDGE or CORNER -- read from the file's first data row, not from
  its name, so a renamed or pre-_edge_ capture still says what it holds --
  and a '-' where the width is one this script cannot plot. Anything left
  elsewhere under FINAL/ is still found and still openable by name.

  Interactive -- no idea yet what you want to see:

      python plot_session_csv.py run.csv

  Opens the plot with a menu column down the left and a time slider along
  the bottom. Nothing to re-run: build the view by hand, then save it.

      traces   every series in the file, as checkboxes
      view     armed shading / grid / legend on and off
      t (s)    drag either end to crop the time window. The y axes rescale
               to whatever is inside the window, so narrowing to a quiet
               2 s stretch actually zooms in on it instead of leaving the
               axis stretched to a spike that is now off-screen
      presets  type a name, press save or load

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
  --t LO:HI      Time range in seconds from the start of the recording.
                 Either side may be omitted: "5:", ":12". Crops the data
                 in --cols/--save/--list mode; in interactive mode it sets
                 the slider's starting window, and the full recording stays
                 loaded so you can widen it again.
  --save FILE    Write the figure to FILE instead of opening a window.
  --overlay      Force every trace onto one axis. By default traces are
                 grouped by UNIT onto stacked subplots sharing the time
                 axis -- deg, deg/s, rad/s and N*m have wildly different
                 scales and crushing them together hides everything but
                 the largest.
  --no-armed     Don't shade the armed regions.
  --list         Print available columns and groups, then exit.

  --preset NAME       Load a saved view before opening.
  --save-preset NAME  Save the resolved view, then carry on.
  --list-presets      Print saved presets, then exit (no CSV needed).

----------------------------------------------------------------------
PRESETS

A preset is one JSON file in ./plot_settings/, holding the trace
selection, time window, shading, grid and legend. Readable, diffable,
copyable between machines, and committable next to the code it goes with.

    python plot_session_csv.py run.csv                    # build a view
                                                          # then press save
    python plot_session_csv.py other.csv --preset armgate # reuse it
    python plot_session_csv.py run.csv --cols tilt --save-preset tilt
    python plot_session_csv.py --list-presets

Presets are deliberately format-agnostic: one saved on a 21-column corner
run can be opened on a 10-column edge run. Traces the file does not have
are skipped and named, rather than refusing to load or -- worse -- quietly
showing you a different view than the one you asked for.

Typed flags beat the preset, the preset beats the built-in default, so
`--preset armgate --t 2:8` is a preset with a different window.

`overlay` is stored, but changing it needs a re-run: the subplot layout is
fixed when the figure is built. Loading a preset that disagrees says so.

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
import json
import sys
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, TextBox

try:
    from matplotlib.widgets import RangeSlider
except ImportError:                       # matplotlib < 3.4
    RangeSlider = None

ARM_GATE_DEG = 0.5   # matches kArmGate in both firmware builds

# Saved view preferences live next to this script, one JSON per preset, so a
# preset is a file you can read, diff, copy to another machine and commit.
SETTINGS_DIR = Path(__file__).resolve().parent / "plot_settings"

PRESET_KEYS = ("traces", "t_range", "overlay", "shade", "grid", "legend")

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

# Stage4_AutoTrim.ino's telemetry -- the same 21 CORNER fields, in the same
# order (every existing CORNER_DERIVED lambda above still works unmodified,
# since they only index into columns 1-9), PLUS 5 automatic-trim columns
# appended at the end. A separate format, not a superset match on width,
# because load_rows() picks by EXACT field count -- see its docstring for
# why a partial/wrong match is worse than a clear "unrecognized width" exit.
CORNER_TRIM_COLS = CORNER_COLS + [
    ("trim_x_deg", "deg"), ("trim_y_deg", "deg"), ("trim_z_deg", "deg"),
    ("trim_com_mm", "mm"), ("trim_enabled", "flag"),
]

CORNER_TRIM_GROUPS = dict(CORNER_GROUPS)
CORNER_TRIM_GROUPS["trim"] = ["trim_x_deg", "trim_y_deg", "trim_z_deg", "trim_com_mm"]
CORNER_TRIM_GROUPS["all"] = CORNER_GROUPS["all"] + ["trim_x_deg", "trim_y_deg",
                                                     "trim_z_deg", "trim_com_mm"]

# Stage4_AutoTrim_RateFilter.ino's telemetry -- CORNER_TRIM_COLS's 26 fields,
# same order, PLUS the 3 filtered-rate columns hw-run-analysis.md's fix 4.1
# added (om_x/y/z_filt_dps in the wire header). Named "om_x_filt" (dropping
# the wire's "_dps" suffix) to match how om_x/phi_x already drop theirs here,
# and placed in the "rates" group next to om_x/y/z on purpose -- the whole
# point of plotting them together is seeing the raw-vs-filtered gap that
# is this fix's own evidence of the 35 Hz mode it targets.
CORNER_TRIM_FILT_COLS = CORNER_TRIM_COLS + [
    ("om_x_filt", "deg/s"), ("om_y_filt", "deg/s"), ("om_z_filt", "deg/s"),
]

CORNER_TRIM_FILT_GROUPS = dict(CORNER_TRIM_GROUPS)
CORNER_TRIM_FILT_GROUPS["rates"] = CORNER_TRIM_GROUPS["rates"] + [
    "om_x_filt", "om_y_filt", "om_z_filt"]
CORNER_TRIM_FILT_GROUPS["all"] = CORNER_TRIM_GROUPS["all"] + [
    "om_x_filt", "om_y_filt", "om_z_filt"]

# corner-bringup/Stage5_AutoTrim.ino's telemetry. NOT built on CORNER_COLS
# despite sharing its first 19 fields -- Stage5's 20th/21st columns are
# "armed, trip_reason" where CORNER_COLS (matched to CornerBalance/
# CornerBalance_WiFi, which are Stage4-shaped) has "armed, gain_scale".
# Same width (21) as CORNER_COLS for that prefix, DIFFERENT meaning for
# the last field -- exactly the ambiguity load_rows()'s width-only
# detection cannot see, which is why this is registered as its own
# distinct total width (33) rather than reusing CORNER_COLS's 21-column
# prefix. Do not feed a Stage5-shaped 21-column-truncated capture through
# the CORNER format by hand; its "gain_scale" column would actually be
# trip_reason.
CORNER_ENDURANCE_COLS = [
    ("t_ms", None),
    ("phi_x", "deg"), ("phi_y", "deg"), ("phi_z", "deg"),
    ("om_x", "deg/s"), ("om_y", "deg/s"), ("om_z", "deg/s"),
    ("rho_x", "rad/s"), ("rho_y", "rad/s"), ("rho_z", "rad/s"),
    ("rho_x_lp", "rad/s"), ("rho_y_lp", "rad/s"), ("rho_z_lp", "rad/s"),
    ("tau_x", "N*m"), ("tau_y", "N*m"), ("tau_z", "N*m"),
    ("tau_cmd_x", "N*m"), ("tau_cmd_y", "N*m"), ("tau_cmd_z", "N*m"),
    ("armed", "flag"),
    ("trip_reason", "flag"),
    ("trim_x_deg", "deg"), ("trim_y_deg", "deg"), ("trim_z_deg", "deg"),
    ("trim_com_mm", "mm"), ("trim_enabled", "flag"),
    ("temp_x", "degC"), ("temp_y", "degC"), ("temp_z", "degC"),
    ("sat_duty_x", "frac"), ("sat_duty_y", "frac"), ("sat_duty_z", "frac"),
    ("loop_overrun_count", "count"),
]

CORNER_ENDURANCE_GROUPS = {
    "tilt":   ["phi_x", "phi_y", "phi_z", "|phi|"],
    "rates":  ["om_x", "om_y", "om_z"],
    "wheels": ["rho_x", "rho_y", "rho_z", "rho_x_lp", "rho_y_lp", "rho_z_lp"],
    "torque": ["tau_x", "tau_y", "tau_z", "tau_cmd_x", "tau_cmd_y", "tau_cmd_z"],
    "trim":   ["trim_x_deg", "trim_y_deg", "trim_z_deg", "trim_com_mm"],
    "endurance": ["temp_x", "temp_y", "temp_z",
                  "sat_duty_x", "sat_duty_y", "sat_duty_z",
                  "loop_overrun_count"],
    "all":    ["phi_x", "phi_y", "phi_z", "|phi|", "om_x", "om_y", "om_z",
               "rho_x_lp", "rho_y_lp", "rho_z_lp", "tau_x", "tau_y", "tau_z",
               "trim_x_deg", "trim_y_deg", "trim_z_deg", "trim_com_mm",
               "temp_x", "temp_y", "temp_z"],
}

# Stage5_AutoTrim_RateFilter.ino's telemetry -- CORNER_ENDURANCE_COLS's 33
# fields, same order, PLUS the same 3 filtered-rate columns CORNER_TRIM_
# FILT_COLS adds (see that entry's comment for the naming rationale).
CORNER_ENDURANCE_FILT_COLS = CORNER_ENDURANCE_COLS + [
    ("om_x_filt", "deg/s"), ("om_y_filt", "deg/s"), ("om_z_filt", "deg/s"),
]

CORNER_ENDURANCE_FILT_GROUPS = dict(CORNER_ENDURANCE_GROUPS)
CORNER_ENDURANCE_FILT_GROUPS["rates"] = CORNER_ENDURANCE_GROUPS["rates"] + [
    "om_x_filt", "om_y_filt", "om_z_filt"]
CORNER_ENDURANCE_FILT_GROUPS["all"] = CORNER_ENDURANCE_GROUPS["all"] + [
    "om_x_filt", "om_y_filt", "om_z_filt"]

# Stacking order for the by-unit subplot layout, most-watched first.
UNIT_ORDER = ["deg", "deg/s", "rad/s", "N*m", "rev", "rev/s", "mm",
              "degC", "frac", "count", "flag"]
UNIT_TITLES = {
    "deg": "Tilt", "deg/s": "Body rate", "rad/s": "Wheel speed",
    "N*m": "Torque", "rev": "Wheel position", "rev/s": "Wheel velocity",
    "mm": "Trim (COM-equivalent)", "degC": "Controller temperature",
    "frac": "Saturation duty", "count": "Loop overrun count",
    "flag": "Armed / gain",
}


# ---------------------------------------------------------------------------
# Presets -- saved view preferences
# ---------------------------------------------------------------------------

def preset_path(name):
    return SETTINGS_DIR / f"{name}.json"


def list_presets():
    if not SETTINGS_DIR.is_dir():
        return []
    return sorted(p.stem for p in SETTINGS_DIR.glob("*.json"))


def read_preset(name):
    """Returns the preset dict, or exits with the list of what does exist."""
    path = preset_path(name)
    if not path.is_file():
        have = ", ".join(list_presets()) or "(none saved yet)"
        sys.exit(f"No preset named {name!r} in {SETTINGS_DIR}.\nAvailable: {have}")
    try:
        # utf-8-sig, not utf-8: presets are meant to be hand-edited, and both
        # Notepad and PowerShell's Out-File write a UTF-8 BOM that plain
        # json.load() rejects with an opaque "Expecting value: line 1 column 1".
        with path.open(encoding="utf-8-sig") as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError) as exc:
        sys.exit(f"Could not read preset {path}: {exc}")
    return {k: data[k] for k in PRESET_KEYS if k in data}


def write_preset(name, settings):
    SETTINGS_DIR.mkdir(parents=True, exist_ok=True)
    path = preset_path(name)
    with path.open("w", encoding="utf-8") as f:
        json.dump(settings, f, indent=2, sort_keys=True)
        f.write("\n")
    return path


def filter_traces(wanted, series):
    """Keep only the traces this file actually has.

    Presets are deliberately format-agnostic: one saved from a 21-column corner
    run is a perfectly reasonable thing to open on a 10-column edge run, and it
    should apply the half that fits rather than refuse or crash. Returns
    (kept, missing) so the caller can say what it dropped instead of silently
    showing a different view than the one you asked for."""
    kept    = [n for n in wanted if n in series]
    missing = [n for n in wanted if n not in series]
    return kept, missing


# ---------------------------------------------------------------------------
# Choosing a file -- what runs when the script is started with no arguments
# ---------------------------------------------------------------------------

# The recording scripts all write into FINAL/telemetry/, split by the mode that
# produced the file: telemetry/plot/ for the live plotters' PLOTMODE csv,
# telemetry/serial/ for terminal_wifi.py's SERIALMONITORMODE logs. The search
# still starts one level up at FINAL/ rather than at telemetry/, so a capture
# saved somewhere else -- an older run, a file dropped in by hand, a USB-side
# recording -- is still reachable instead of invisible.
CAPTURE_ROOT = Path(__file__).resolve().parent
TELEMETRY_DIR = CAPTURE_ROOT / "telemetry"

CAPTURE_SUFFIXES = (".csv", ".log")


def capture_mode(path):
    """PLOT or SERIAL -- which recorder wrote this file.

    The folder decides, so a file keeps its mode even if it was renamed; the
    suffix is the fallback for anything sitting outside telemetry/. It is worth
    showing in the picker because the two are different wire formats: PLOT is
    comma-delimited and always 10 or 21 columns, SERIAL is tab-delimited and
    only plottable at all when the stage that wrote it emitted 21."""
    parents = {p.name.lower() for p in path.parents}
    if "serial" in parents:
        return "SERIAL"
    if "plot" in parents:
        return "PLOT"
    return "SERIAL" if path.suffix.lower() == ".log" else "PLOT"


_FORMAT_CACHE = {}


def capture_format(path):
    """EDGE, CORNER, or '' -- which wire format this capture holds.

    Read from the file's own first data row rather than from its name: both
    live plotters write into telemetry/plot/, the two formats are not
    interchangeable, and a capture recorded before the _edge_/_corner_ naming
    -- or renamed by hand, or copied in from another machine -- still has to
    answer correctly. '' means a width this script cannot plot at all, which
    is the Stage 1 (13) and Stage 2/3 (18) serial logs.

    Only the first data line is read, and the answer is cached, so listing a
    folder stays cheap next to the 24 MB a long corner run produces."""
    key = str(path)
    if key not in _FORMAT_CACHE:
        width = 0
        try:
            with path.open(newline="") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    # Both delimiters tried rather than sniffed: one line is
                    # not enough of a sample to vote on, and the wrong guess
                    # gives a width of 1, which matches no format anyway.
                    width = max(len(line.split(",")), len(line.split("\t")))
                    break
        except OSError:
            pass
        _FORMAT_CACHE[key] = {len(EDGE_COLS): "EDGE",
                              len(CORNER_COLS): "CORNER"}.get(width, "")
    return _FORMAT_CACHE[key]


def find_captures(root=CAPTURE_ROOT):
    """Every capture at or below `root`, newest first.

    Modification time, not the name's timestamp: a file copied in from another
    machine or renamed by hand should still sort where it belongs, and a capture
    that never got a timestamped name should still be reachable."""
    files = [p for p in root.rglob("*")
             if p.suffix.lower() in CAPTURE_SUFFIXES and p.is_file()]
    return sorted(files, key=lambda p: p.stat().st_mtime, reverse=True)


def describe(path, with_mode=True):
    """One line per file for the picker: which mode, which format, where, when,
    how big.

    `with_mode=False` under the grouped listing, whose section headings already
    say it -- but on its own, as the default line or an ambiguity list, the mode
    is the first thing worth knowing about a file. The FORMAT is shown either
    way: the section heading cannot give it, since telemetry/plot/ holds edge
    and corner recordings side by side, and it is what decides whether the file
    opens at all."""
    try:
        rel = path.relative_to(CAPTURE_ROOT)
    except ValueError:
        rel = path
    stat = path.stat()
    when = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M")
    mode = f"{capture_mode(path):<6} " if with_mode else ""
    fmt = f"{capture_format(path) or '-':<6} "
    return f"{mode}{fmt}{str(rel):<50} {when}  {stat.st_size / 1e6:6.2f} MB"


def resolve_typed_name(text, candidates, numbered=None):
    """A typed path, a bare filename, a unique fragment of one, or a list number.

    Typing the whole path after a recording session is tedious and easy to get
    wrong, and the interesting part of these names is the timestamp -- so
    "014914" is enough as long as it picks out exactly one file.

    `numbered` is the shortened list the picker actually printed, which is what
    a typed number has to index into; name and fragment matching still runs
    against the full `candidates`, so an older capture that did not make the
    printed list is reachable by name."""
    text = text.strip().strip('"').strip("'")
    if not text:
        return None

    numbered = candidates if numbered is None else numbered
    if text.isdigit() and 1 <= int(text) <= len(numbered):
        return numbered[int(text) - 1]

    path = Path(text)
    if path.is_file():
        return path
    if not path.is_absolute() and (CAPTURE_ROOT / path).is_file():
        return CAPTURE_ROOT / path

    exact = [c for c in candidates if c.name.lower() == text.lower()]
    if exact:
        return exact[0]

    partial = [c for c in candidates if text.lower() in c.name.lower()]
    if len(partial) == 1:
        return partial[0]
    if len(partial) > 1:
        print(f"  {text!r} matches {len(partial)} files -- be more specific:")
        for c in partial[:10]:
            print(f"    {describe(c)}")
    else:
        print(f"  no file matching {text!r}.")
    return None


def listing_order(captures, per_mode=8):
    """Display order for the picker: the PLOT recordings, then the SERIAL logs,
    each newest first, trimmed to the most recent `per_mode` of either.

    Grouped rather than one flat newest-first list because the two are answers
    to different questions -- "how did the run go" vs "what did the firmware
    say" -- and a session mixes them, so a flat list interleaves the two and
    makes you read the extension of every line to find the one you want.

    Returned as one list because the numbers the picker prints are indices into
    it: what you read is what you type."""
    plot   = [c for c in captures if capture_mode(c) == "PLOT"]
    serial = [c for c in captures if capture_mode(c) == "SERIAL"]
    return plot[:per_mode] + serial[:per_mode], len(plot), len(serial)


def pick_csv():
    """Menu shown when no capture was given on the command line."""
    found = find_captures()
    newest = found[0] if found else None
    captures, n_plot, n_serial = listing_order(found)

    # Piped or redirected: input() would raise EOFError on the first prompt, so
    # fail the way argparse would have.
    if not sys.stdin.isatty():
        sys.exit("No CSV given, and stdin is not a terminal to ask on. "
                 "Pass the file as an argument.")

    print("\nWhich file?")
    if newest:
        print(f"  d  default -- newest capture: {describe(newest)}")
    else:
        print(f"  d  default -- (nothing found under {TELEMETRY_DIR})")
    print("  f  file    -- type a name, path, or list number")
    print("  q  quit")

    while True:
        try:
            raw = input("choice [d]: ").strip()
        except (EOFError, KeyboardInterrupt):
            sys.exit("\nCancelled.")
        choice = raw.lower() or "d"

        if choice in ("q", "quit", "exit"):
            sys.exit("Cancelled.")

        if choice in ("d", "default"):
            if newest:
                return newest
            print(f"  nothing to default to -- no capture under {CAPTURE_ROOT}."
                  f" Use f and give a path.")
            continue

        if choice in ("f", "file"):
            if captures:
                print(f"\n  Recordings under {CAPTURE_ROOT}:")
                print("      (format column: EDGE = 10 col, CORNER = 21 col, "
                      "- = not plottable)")
                shown = 0
                for label, total in (("PLOT   -- live plotters, comma csv", n_plot),
                                     ("SERIAL -- terminal_wifi.py, tab log", n_serial)):
                    listed = [c for c in captures
                              if capture_mode(c) == label.split()[0]]
                    if not listed:
                        continue
                    print(f"\n  {label}")
                    for c in listed:
                        shown += 1
                        print(f"   {shown:>2}. {describe(c, with_mode=False)}")
                    if total > len(listed):
                        print(f"       ... and {total - len(listed)} older "
                              f"(reachable by name)")
            try:
                text = input("file: ")
            except (EOFError, KeyboardInterrupt):
                sys.exit("\nCancelled.")
            hit = resolve_typed_name(text, found, numbered=captures)
            if hit:
                return hit
            continue

        # A path typed straight at the menu, rather than pressing f first. Only
        # worth guessing at if it looks like a filename -- a mistyped menu key
        # should get the menu's own error, not a file-search one.
        if (any(ch in raw for ch in "./\\")
                or raw.lower().endswith(CAPTURE_SUFFIXES)):
            hit = resolve_typed_name(raw, found, numbered=captures)
            if hit:
                return hit
            continue
        print("  answer d, f or q.")


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
    elif width == len(CORNER_TRIM_COLS):
        spec, groups, derived, kind = (CORNER_TRIM_COLS, CORNER_TRIM_GROUPS,
                                       CORNER_DERIVED, "CORNER_TRIM")
    elif width == len(CORNER_TRIM_FILT_COLS):
        spec, groups, derived, kind = (CORNER_TRIM_FILT_COLS, CORNER_TRIM_FILT_GROUPS,
                                       CORNER_DERIVED, "CORNER_TRIM_FILT")
    elif width == len(CORNER_ENDURANCE_COLS):
        spec, groups, derived, kind = (CORNER_ENDURANCE_COLS, CORNER_ENDURANCE_GROUPS,
                                       CORNER_DERIVED, "CORNER_ENDURANCE")
    elif width == len(CORNER_ENDURANCE_FILT_COLS):
        spec, groups, derived, kind = (CORNER_ENDURANCE_FILT_COLS, CORNER_ENDURANCE_FILT_GROUPS,
                                       CORNER_DERIVED, "CORNER_ENDURANCE_FILT")
    else:
        sys.exit(f"{path.name}: rows are {width} fields wide; expected "
                 f"{len(EDGE_COLS)} (edge), {len(CORNER_COLS)} (corner), "
                 f"{len(CORNER_TRIM_COLS)} (corner + auto-trim), "
                 f"{len(CORNER_TRIM_FILT_COLS)} (corner + auto-trim + rate filter), "
                 f"{len(CORNER_ENDURANCE_COLS)} (corner + auto-trim + endurance), or "
                 f"{len(CORNER_ENDURANCE_FILT_COLS)} (corner + auto-trim + endurance + rate filter).")

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
    """Builds the axes and returns (fig, {name: Line2D}, axes, proxy, patches).

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

    # Build the shading patches even when `shade` is off, and just hide them.
    # The view menu can then turn shading back on without rebuilding the
    # figure -- axvspan needs the spans, and recomputing them on every toggle
    # would mean re-deriving armed_spans() against data that never changes.
    spans = armed_spans(t, series)
    shade_patches = []
    for ax in axes:
        for lo, hi in spans:
            patch = ax.axvspan(lo, hi, color="0.85", zorder=0)
            patch.set_visible(shade)
            shade_patches.append(patch)
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
    armed_proxy = None
    if spans:
        armed_proxy = axes[0].axvspan(t[0], t[0], color="0.85", label="armed")
        armed_proxy.set_visible(shade)

    return fig, lines, axes, armed_proxy, shade_patches


def set_grid(axes, on):
    """matplotlib treats grid(False, alpha=...) as 'enable, with these line
    properties' and warns -- so the off case must pass no styling at all."""
    for ax in axes:
        ax.grid(True, alpha=0.25) if on else ax.grid(False)


def refresh(axes, armed_proxy=None, xlim=None, show_legend=True):
    """Rescale y and rebuild legends from the VISIBLE traces only.

    Both halves matter after a checkbox toggle: an axis left stretched to fit
    a hidden trace flattens everything you can still see, and a legend still
    listing hidden traces is actively misleading about what is on screen.

    `xlim` narrows the rescale to the samples inside the current time window,
    which is what makes the time slider useful rather than cosmetic: without
    it, cropping to a quiet 2 s stretch still leaves the axis scaled to a spike
    somewhere off-screen, and you have zoomed in on a flat line."""
    for ax in axes:
        vis = [ln for ln in ax.get_lines()
               if ln.get_visible() and not ln.get_label().startswith("_")]
        ys = []
        for ln in vis:
            if xlim is None:
                ys.extend(ln.get_ydata())
            else:
                lo_x, hi_x = xlim
                ys.extend(y for x, y in zip(ln.get_xdata(), ln.get_ydata())
                          if lo_x <= x <= hi_x)
        if ys:
            lo, hi = min(ys), max(ys)
            pad = (hi - lo) * 0.08 or (abs(hi) * 0.1 or 1.0)
            ax.set_ylim(lo - pad, hi + pad)

        handles = list(vis)
        if (armed_proxy is not None and armed_proxy.get_visible()
                and ax is axes[0]):
            handles.append(armed_proxy)
        if show_legend and handles:
            ax.legend(handles=handles, loc="upper left", fontsize=7, ncol=3)
        elif ax.get_legend():
            ax.get_legend().remove()


def main():
    p = argparse.ArgumentParser(
        description="Plot a saved Cubli telemetry CSV (edge or corner).")
    p.add_argument("csv", nargs="?",
                   help="path to the capture, or a filename/timestamp fragment "
                        "of one under telemetry/. Omit it to be asked "
                        "(d = newest recording, f = pick from a list)")
    p.add_argument("--cols", default=None,
                   help="comma-separated columns and/or group names; "
                        "omit for the interactive checkbox picker")
    p.add_argument("--t", default=None, metavar="LO:HI",
                   help="time range in seconds, e.g. 5:12, 5:, :12. Crops in "
                        "--cols/--save/--list mode; sets the initial slider "
                        "window in interactive mode")
    p.add_argument("--save", default=None, metavar="FILE",
                   help="write the figure to FILE instead of opening a window")
    # default=None, not False: it lets a preset supply the value when the flag
    # was not typed, while a typed flag still wins.
    p.add_argument("--overlay", action="store_true", default=None,
                   help="put every trace on one axis instead of grouping by unit")
    p.add_argument("--no-armed", action="store_true", default=None,
                   help="don't shade the armed regions")
    p.add_argument("--list", action="store_true",
                   help="print available columns and groups, then exit")
    p.add_argument("--preset", default=None, metavar="NAME",
                   help="load a saved view preset before opening")
    p.add_argument("--save-preset", default=None, metavar="NAME",
                   help="save the resolved view as a preset, then carry on")
    p.add_argument("--list-presets", action="store_true",
                   help="print saved presets, then exit")
    args = p.parse_args()

    if args.list_presets:
        names = list_presets()
        print(f"Presets in {SETTINGS_DIR}:")
        for n in names:
            print(f"  {n}")
        if not names:
            print("  (none saved yet -- tick the view you want, type a name "
                  "in the box and press 'save')")
        return

    if args.csv:
        path = Path(args.csv)
        if not path.is_file():
            # A bare timestamp or filename typed after the flags should work the
            # same way it does in the picker, instead of "File not found".
            hit = resolve_typed_name(args.csv, find_captures())
            if hit is None:
                sys.exit(1)      # resolve_typed_name already said what it tried
            path = hit
    else:
        path = pick_csv()

    print(f"Opening {path}")

    rows, spec, groups, derived = load_rows(path)

    preset = read_preset(args.preset) if args.preset else {}

    # Typed flag > preset > built-in default, for each preference.
    overlay   = args.overlay if args.overlay is not None else preset.get("overlay", False)
    shade     = (not args.no_armed) if args.no_armed is not None else preset.get("shade", True)
    grid_on   = preset.get("grid", True)
    legend_on = preset.get("legend", True)

    t_range = None
    if args.t:
        lo_s, _, hi_s = args.t.partition(":")
        t_range = (float(lo_s) if lo_s else None, float(hi_s) if hi_s else None)
    elif preset.get("t_range"):
        t_range = tuple(preset["t_range"])

    interactive = args.cols is None

    # Interactive keeps the WHOLE recording loaded and uses the slider as a
    # view window, so a range can be widened again after narrowing it. The
    # non-interactive paths crop, which is what --t has always meant for them
    # and what --save needs in order to write the figure you asked for.
    crop = None if (interactive and not args.list) else t_range
    t, series = build_series(rows, spec, derived, crop)

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

    if args.cols:
        cols = resolve_cols(args.cols.split(","), series, groups)
    elif preset.get("traces"):
        cols, missing = filter_traces(preset["traces"], series)
        if missing:
            print(f"  preset {args.preset!r}: {len(missing)} trace(s) not in "
                  f"this file, skipped: {', '.join(missing)}")
        if not cols:
            print(f"  preset {args.preset!r} selected nothing this file has -- "
                  f"falling back to the 'all' group.")
            cols = resolve_cols(groups["all"], series, groups)
    else:
        cols = resolve_cols(groups["all"], series, groups)

    fig, lines, axes, armed_proxy, shade_patches = make_figure(
        t, series, cols, overlay, shade, path.name, all_units=interactive)
    set_grid(axes, grid_on)

    view = [t[0], t[-1]]
    if interactive and t_range:
        lo, hi = t_range
        view = [t[0] if lo is None else max(lo, t[0]),
                t[-1] if hi is None else min(hi, t[-1])]
        if view[1] <= view[0]:
            view = [t[0], t[-1]]
        axes[0].set_xlim(*view)          # x is shared, so this sets them all

    refresh(axes, armed_proxy, xlim=view if interactive else None,
            show_legend=legend_on)

    if args.save_preset:
        saved = write_preset(args.save_preset, {
            "traces": cols, "t_range": [round(view[0], 3), round(view[1], 3)],
            "overlay": overlay, "shade": shade,
            "grid": grid_on, "legend": legend_on})
        print(f"Saved preset {args.save_preset!r} -> {saved}")

    if args.save:
        fig.tight_layout()
        fig.savefig(args.save, dpi=150)
        print(f"Wrote {args.save}")
        return

    if interactive:
        # Menu column down the left, time slider along the bottom. Reserve the
        # space rather than overlaying, so no control ever sits on the traces.
        fig.subplots_adjust(left=0.21, right=0.98, top=0.95, bottom=0.14)
        names = sorted(lines)

        # Mutable so the widget callbacks below can write to it; plain locals
        # would need `nonlocal` in five closures.
        state = {"shade": shade, "grid": grid_on, "legend": legend_on,
                 "view": list(view)}

        def redraw():
            refresh(axes, armed_proxy, xlim=state["view"],
                    show_legend=state["legend"])
            fig.canvas.draw_idle()

        # --- traces -------------------------------------------------------
        ax_check = fig.add_axes([0.005, 0.33, 0.17, 0.61])
        ax_check.set_title("traces", fontsize=8)
        check = CheckButtons(ax_check, names,
                             [lines[n].get_visible() for n in names])
        for lbl in check.labels:
            lbl.set_fontsize(7)

        def toggle(name):
            lines[name].set_visible(not lines[name].get_visible())
            redraw()

        check.on_clicked(toggle)

        # --- view options -------------------------------------------------
        opt_keys = ("shade", "grid", "legend")
        opt_labels = ["armed shading", "grid", "legend"]
        ax_opts = fig.add_axes([0.005, 0.215, 0.17, 0.10])
        ax_opts.set_title("view", fontsize=8)
        opts = CheckButtons(ax_opts, opt_labels,
                            [state[k] for k in opt_keys])
        for lbl in opts.labels:
            lbl.set_fontsize(7)

        def toggle_opt(label):
            key = opt_keys[opt_labels.index(label)]
            state[key] = not state[key]
            if key == "shade":
                for patch in shade_patches:
                    patch.set_visible(state["shade"])
                if armed_proxy is not None:
                    armed_proxy.set_visible(state["shade"])
            elif key == "grid":
                set_grid(axes, state["grid"])
            redraw()

        opts.on_clicked(toggle_opt)

        # --- time window --------------------------------------------------
        slider = None
        if RangeSlider is not None and t[-1] > t[0]:
            ax_time = fig.add_axes([0.32, 0.045, 0.56, 0.022])
            slider = RangeSlider(ax_time, "t (s) ", t[0], t[-1],
                                 valinit=tuple(state["view"]))
            slider.label.set_fontsize(8)

            def on_time(val):
                state["view"] = [float(val[0]), float(val[1])]
                axes[0].set_xlim(*state["view"])
                redraw()

            slider.on_changed(on_time)
        elif RangeSlider is None:
            print("  (matplotlib < 3.4: no time slider -- use --t LO:HI)")

        # --- presets ------------------------------------------------------
        ax_name = fig.add_axes([0.005, 0.152, 0.17, 0.038])
        name_box = TextBox(ax_name, "", initial=(args.preset or "default"))
        name_box.text_disp.set_fontsize(8)
        ax_bsave = fig.add_axes([0.005, 0.100, 0.082, 0.040])
        ax_bload = fig.add_axes([0.093, 0.100, 0.082, 0.040])
        b_save, b_load = Button(ax_bsave, "save"), Button(ax_bload, "load")
        b_save.label.set_fontsize(8)
        b_load.label.set_fontsize(8)

        def current_settings():
            return {
                "traces": [n for n in names if lines[n].get_visible()],
                "t_range": [round(state["view"][0], 3),
                            round(state["view"][1], 3)],
                "overlay": overlay,
                "shade": state["shade"],
                "grid": state["grid"],
                "legend": state["legend"],
            }

        def do_save(_event):
            name = (name_box.text or "").strip()
            if not name:
                print("  type a preset name in the box first")
                return
            print(f"  saved preset {name!r} -> {write_preset(name, current_settings())}")

        def do_load(_event):
            name = (name_box.text or "").strip()
            if not preset_path(name).is_file():
                have = ", ".join(list_presets()) or "(none saved yet)"
                print(f"  no preset {name!r} in {SETTINGS_DIR}. have: {have}")
                return
            data = read_preset(name)

            kept, missing = filter_traces(data.get("traces", []), series)
            if missing:
                print(f"  {len(missing)} trace(s) not in this file, skipped: "
                      f"{', '.join(missing)}")
            want = set(kept)
            # set_active() fires toggle(), which keeps the widget's own state
            # and the Line2D in step -- there is no supported way to set a
            # CheckButtons value silently across matplotlib versions.
            for i, n in enumerate(names):
                if lines[n].get_visible() != (n in want):
                    check.set_active(i)
            for i, key in enumerate(opt_keys):
                if key in data and bool(data[key]) != state[key]:
                    opts.set_active(i)
            if data.get("t_range") and slider is not None:
                lo_v = max(float(data["t_range"][0]), t[0])
                hi_v = min(float(data["t_range"][1]), t[-1])
                if hi_v > lo_v:
                    slider.set_val((lo_v, hi_v))
            print(f"  loaded preset {name!r}")
            # Axis layout is fixed when the figure is built, so this one
            # preference cannot be applied live -- say so rather than load a
            # preset that silently comes out looking different.
            if bool(data.get("overlay", False)) != bool(overlay):
                flag = "--overlay" if data.get("overlay") else "without --overlay"
                print(f"  note: preset wants overlay="
                      f"{bool(data.get('overlay'))}; re-run {flag} for that.")

        b_save.on_clicked(do_save)
        b_load.on_clicked(do_load)

        # Keep references on the figure: these widgets stop responding if they
        # are garbage-collected when main() returns into plt.show().
        fig._widgets = (check, opts, slider, name_box, b_save, b_load)

        print("Tick traces on the left; drag the time slider along the bottom.")
        print(f"Presets: type a name, then save/load. Stored in {SETTINGS_DIR}")
    else:
        fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()

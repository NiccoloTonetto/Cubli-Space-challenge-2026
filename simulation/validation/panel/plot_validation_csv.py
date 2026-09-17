"""plot_validation_csv.py

Offline plotter for a single previously-recorded telemetry CSV in this
folder (e.g. 12degrees.csv, or one of the timestamped telemetry_*.csv
files saved by telemetry_python.py / telemetry_python_wifi.py). Draws the
full recorded trace across the same panel layout as those live scripts --
theta alone with fixed -30..30 deg limits, theta dot alone, and torque +
wheel together -- plus a per-panel text readout of each line's min / max /
final value, since with a static plot you can just read the numbers
instead of eyeballing the y-axis.

Same 10-column wire format as the live scripts, with or without a header
row (auto-detected):

    t_ms,theta_deg,theta_dot_dps,tau_Nm,tau_cmd_Nm,armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel

Requires: pip install matplotlib

Usage:
    python plot_validation_csv.py <path/to/file.csv>
"""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

NUM_COLS = 10
COL_NAMES = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm",
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"]


def load_rows(path):
    rows = []
    skipped = 0
    with path.open(newline="") as f:
        reader = csv.reader(f)
        for i, parts in enumerate(reader):
            if not parts:
                continue
            if i == 0:
                try:
                    float(parts[0])
                except ValueError:
                    continue    # header row -- skip it
            if len(parts) != NUM_COLS:
                skipped += 1
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                skipped += 1
    if skipped:
        print(f"Skipped {skipped} malformed row(s) in {path.name}.")
    return rows


def stats_text(label, values, unit, fmt="{:.3f}"):
    final = fmt.format(values[-1])
    lo = fmt.format(min(values))
    hi = fmt.format(max(values))
    prefix = f"{label}: " if label else ""
    return f"{prefix}final {final} | min {lo} | max {hi} {unit}"


def main():
    if len(sys.argv) != 2:
        print("Usage: python plot_validation_csv.py <path/to/file.csv>")
        sys.exit(1)

    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"File not found: {path}")
        sys.exit(1)

    rows = load_rows(path)
    if not rows:
        print(f"No valid {NUM_COLS}-field rows found in {path}.")
        sys.exit(1)

    t0 = rows[0][0]
    t = [(r[0] - t0) / 1000.0 for r in rows]
    theta      = [r[1] for r in rows]
    theta_dot  = [r[2] for r in rows]
    tau        = [r[3] for r in rows]
    tau_cmd    = [r[4] for r in rows]
    wheel_lp   = [r[7] for r in rows]

    print(f"Loaded {len(rows)} samples from {path.name} "
          f"({t[-1]:.2f}s of data).")

    fig_theta, ax_theta = plt.subplots(figsize=(9, 4))
    fig_theta.canvas.manager.set_window_title(f"{path.name} - Theta")
    ax_theta.set_title(f"Tilt (theta) -- {path.name}")
    ax_theta.set_ylabel("deg"); ax_theta.set_xlabel("t (s)")
    ax_theta.set_ylim(-30, 30)
    ax_theta.plot(t, theta, color="C0")
    ax_theta.text(0.98, 0.95, stats_text("", theta, "deg", "{:.2f}"),
                  transform=ax_theta.transAxes, ha="right", va="top",
                  fontsize=10, color="C0", fontweight="bold")

    fig_thetadot, ax_thetadot = plt.subplots(figsize=(9, 4))
    fig_thetadot.canvas.manager.set_window_title(f"{path.name} - Theta dot")
    ax_thetadot.set_title(f"Tilt rate (theta dot) -- {path.name}")
    ax_thetadot.set_ylabel("deg/s"); ax_thetadot.set_xlabel("t (s)")
    ax_thetadot.plot(t, theta_dot, color="C1")
    ax_thetadot.text(0.98, 0.95, stats_text("", theta_dot, "deg/s", "{:.2f}"),
                      transform=ax_thetadot.transAxes, ha="right", va="top",
                      fontsize=10, color="C1", fontweight="bold")

    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.canvas.manager.set_window_title(f"{path.name} - Torque & Wheel")
    ax2.set_title("Torque"); ax2.set_ylabel("N*m")
    ax3.set_title("Wheel");  ax3.set_ylabel("rad/s"); ax3.set_xlabel("t (s)")

    ax2.plot(t, tau, color="C0", label="tau")
    ax2.plot(t, tau_cmd, color="C1", linestyle="--", label="tau cmd")
    ax2.legend(loc="upper left")
    ax2.text(0.98, 0.95, stats_text("tau", tau, "N*m"),
             transform=ax2.transAxes, ha="right", va="top",
             fontsize=9, color="C0", fontweight="bold")
    ax2.text(0.98, 0.82, stats_text("tau cmd", tau_cmd, "N*m"),
             transform=ax2.transAxes, ha="right", va="top",
             fontsize=9, color="C1", fontweight="bold")

    ax3.plot(t, wheel_lp, color="C0", label="wheel omega lp")
    ax3.legend(loc="upper left")
    ax3.text(0.98, 0.95, stats_text("", wheel_lp, "rad/s"),
             transform=ax3.transAxes, ha="right", va="top",
             fontsize=9, color="C0", fontweight="bold")

    fig_theta.tight_layout()
    fig_thetadot.tight_layout()
    fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()

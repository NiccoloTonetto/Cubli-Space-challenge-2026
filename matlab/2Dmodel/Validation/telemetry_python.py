"""telemetry_python.py

Live telemetry viewer + logger for Stage4_FullLaw.ino running with
TELEMETRY_MODE set to TELEMETRY_MATLAB (see the selector near the top of
Stage4_FullLaw.ino). In that mode the Teensy sends one plain CSV line per
control cycle, no header/comment lines:

    t_ms,theta_deg,theta_dot_dps,tau_Nm,tau_cmd_Nm,armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel

Python equivalent of telemetry_matlab.m in this same folder -- same
settings block, same three-panel live scrolling plot (tilt / torque /
wheel), same behavior of saving the full session to a timestamped .csv
next to this script when the window is closed.

Requires: pip install pyserial matplotlib

Usage:
    1. In Stage4_FullLaw.ino, set TELEMETRY_MODE to TELEMETRY_MATLAB and
       re-upload.
    2. Set PORT below to the Teensy's serial port (Windows: check Device
       Manager -> Ports, e.g. "COM5").
    3. Run this script. A window opens with a live scrolling plot.
    4. Close the window when you're done -- the full session is saved to
       a timestamped .csv file next to this script.
"""

import csv
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial

# ---- settings ----
PORT = "COM5"       # <-- set to your Teensy's port
BAUD = 115200
NUM_COLS = 10
WINDOW_S = 10        # seconds visible in the scrolling plot
REDRAW_S = 0.05      # plot redraw throttle (mirrors MATLAB's "drawnow limitrate")
OUT_DIR = Path(__file__).resolve().parent

COL_NAMES = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm",
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"]


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    fig.canvas.manager.set_window_title("Stage4 Telemetry (Python mode)")

    ax1.set_title("Tilt");   ax1.set_ylabel("deg, deg/s")
    ax2.set_title("Torque"); ax2.set_ylabel("N*m")
    ax3.set_title("Wheel");  ax3.set_ylabel("rad/s"); ax3.set_xlabel("t (s)")

    (lTheta,)    = ax1.plot([], [], color="C0", label="theta (deg)")
    (lThetaDot,) = ax1.plot([], [], color="C1", label="theta dot (deg/s)")
    ax1.legend(loc="upper left")

    (lTau,)    = ax2.plot([], [], color="C0", label="tau")
    (lTauCmd,) = ax2.plot([], [], color="C1", linestyle="--", label="tau cmd")
    ax2.legend(loc="upper left")

    (lWheelLp,) = ax3.plot([], [], color="C0", label="wheel omega lp")
    ax3.legend(loc="upper left")

    fig.tight_layout()

    stop = {"flag": False}
    fig.canvas.mpl_connect("close_event", lambda evt: stop.__setitem__("flag", True))

    t_list, theta_list, theta_dot_list = [], [], []
    tau_list, tau_cmd_list, wheel_lp_list = [], [], []
    rows = []
    t0 = None
    last_draw = 0.0

    try:
        while plt.fignum_exists(fig.number) and not stop["flag"]:
            raw = ser.readline()
            if not raw:
                plt.pause(0.001)
                continue

            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != NUM_COLS:
                continue    # malformed/partial line -- skip it
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue

            rows.append(vals)
            if t0 is None:
                t0 = vals[0]
            t_s = (vals[0] - t0) / 1000.0

            t_list.append(t_s)
            theta_list.append(vals[1])
            theta_dot_list.append(vals[2])
            tau_list.append(vals[3])
            tau_cmd_list.append(vals[4])
            wheel_lp_list.append(vals[7])

            now = time.monotonic()
            if now - last_draw >= REDRAW_S:
                last_draw = now

                lTheta.set_data(t_list, theta_list)
                lThetaDot.set_data(t_list, theta_dot_list)
                lTau.set_data(t_list, tau_list)
                lTauCmd.set_data(t_list, tau_cmd_list)
                lWheelLp.set_data(t_list, wheel_lp_list)

                lo, hi = max(0.0, t_s - WINDOW_S), max(WINDOW_S, t_s)
                for ax in (ax1, ax2, ax3):
                    ax.set_xlim(lo, hi)
                    ax.relim()
                    ax.autoscale_view(scalex=False)

                fig.canvas.draw_idle()
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if plt.fignum_exists(fig.number):
            plt.close(fig)

    if rows:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_file = OUT_DIR / f"telemetry_{stamp}.csv"
        with out_file.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COL_NAMES)
            writer.writerows(rows)
        print(f"Saved {len(rows)} samples to {out_file}")
    else:
        print("No data captured -- nothing saved.")


if __name__ == "__main__":
    main()

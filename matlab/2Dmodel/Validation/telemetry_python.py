"""telemetry_python.py

Live telemetry viewer + logger for Stage4_FullLaw.ino running with
TELEMETRY_MODE set to PLOTMODE (see the selector near the top of
Stage4_FullLaw.ino). In that mode the Teensy sends one plain CSV line per
control cycle, no header/comment lines:

    t_ms,theta_deg,theta_dot_dps,tau_Nm,tau_cmd_Nm,armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel

Python equivalent of telemetry_matlab.m in this same folder -- same
settings block, same three-panel live scrolling plot (tilt / torque /
wheel), same behavior of saving the full session to a timestamped .csv
next to this script when the window is closed.

Serial is read on a background thread into a queue; the main thread only
runs matplotlib's own FuncAnimation + plt.show() loop. Doing the blocking
serial I/O and the GUI redraw in the same loop (as an earlier version of
this script did) starves the window's message pump under Windows and it
gets flagged "(Not Responding)" -- this split keeps the GUI thread free to
service its event loop continuously.

A serial port can only be opened by one process at a time, so the Arduino
Serial Monitor can't also connect while this script has the port open.
Since this script already owns the connection, it doubles as the command
console too: type the same commands the firmware's handleSerialCommands()
understands (a1, a0, g<0..1>, o<deg>) into the terminal you launched this
script from and press Enter -- they're forwarded straight to the Teensy.

Requires: pip install pyserial matplotlib

Usage:
    1. In Stage4_FullLaw.ino, set TELEMETRY_MODE to PLOTMODE and
       re-upload.
    2. Set PORT below to the Teensy's serial port (Windows: check Device
       Manager -> Ports, e.g. "COM9").
    3. Run this script. A window opens with a live scrolling plot.
    4. In the terminal (not the plot window), type commands like a1, g0.5,
       o-2.3 and press Enter to send them to the Teensy.
    5. Close the window when you're done -- the full session is saved to
       a timestamped .csv file next to this script.
"""

import csv
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

# ---- settings ----
PORT = "COM9"       # <-- set to your Teensy's port
BAUD = 115200
NUM_COLS = 10
WINDOW_S = 10        # seconds visible in the scrolling plot
REDRAW_MS = 50       # plot redraw period (~20 fps)
OUT_DIR = Path(__file__).resolve().parent

COL_NAMES = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm",
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"]


def reader_thread(ser, q, stop_event):
    """Blocking serial reads live entirely on this thread -- the GUI thread
    never waits on I/O, so it can keep servicing its event loop."""
    last_valid = time.monotonic()
    last_warn = 0.0
    warned_bad_shape = False
    last_armed = None   # PLOTMODE suppresses the "# gArmed = TRUE" echo, so
                         # this is the only confirmation a1/a0 landed -- and
                         # it also catches the control law self-disarming on
                         # a safety trip (overtilt/overspeed/NaN).

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            now = time.monotonic()
            if now - last_valid > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("... no valid PLOTMODE lines in the last 2s -- check "
                      "that Stage4_FullLaw.ino's TELEMETRY_MODE is set to "
                      "PLOTMODE (re-upload after changing it), and that "
                      "PORT/BAUD above match the Teensy.")
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        parts = line.split(",")
        if len(parts) != NUM_COLS:
            if not warned_bad_shape:
                warned_bad_shape = True
                print(f"Got a line but it isn't {NUM_COLS}-field PLOTMODE "
                      f"CSV ({len(parts)} fields): {line!r}")
                print("If that looks tab-delimited or has a header/'#' "
                      "text, the firmware is still in SERIALMONITORMODE.")
            continue    # malformed/partial line -- skip it
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        last_valid = time.monotonic()

        armed = int(vals[5])
        if last_armed is not None and armed != last_armed:
            print(f"Teensy: {'ARMED' if armed else 'DISARMED'}")
        last_armed = armed

        q.put(vals)


def command_thread(ser, stop_event):
    """Forwards lines typed in this terminal straight to the Teensy -- the
    same a<0/1> / g<0..1> / o<deg> commands handleSerialCommands() already
    understands. Runs on its own thread since input() blocks."""
    while not stop_event.is_set():
        try:
            line = input()
        except EOFError:
            break
        line = line.strip()
        if not line:
            continue
        ser.write((line + "\n").encode("ascii"))


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    print(f"Opened {PORT} @ {BAUD} baud. Waiting for PLOTMODE data "
          f"({NUM_COLS} comma-separated fields per line)...")

    q = queue.Queue()
    stop_event = threading.Event()
    thread = threading.Thread(target=reader_thread, args=(ser, q, stop_event), daemon=True)
    thread.start()

    cmd_thread = threading.Thread(target=command_thread, args=(ser, stop_event), daemon=True)
    cmd_thread.start()
    print("Type a1 / a0 / g<0..1> / o<deg> and press Enter to send to the "
          "Teensy (same commands as the Serial Monitor).")

    fig_theta, ax_theta = plt.subplots(figsize=(7, 4))
    fig_theta.canvas.manager.set_window_title("Stage4 Telemetry (PLOTMODE) - Theta")
    ax_theta.set_title("Tilt (theta)"); ax_theta.set_ylabel("deg"); ax_theta.set_xlabel("t (s)")
    ax_theta.set_ylim(-30, 30)
    (lTheta,) = ax_theta.plot([], [], color="C0", label="theta (deg)")
    txtTheta = ax_theta.text(0.98, 0.95, "", transform=ax_theta.transAxes,
                              ha="right", va="top", fontsize=12, color="C0", fontweight="bold")

    fig_thetadot, ax_thetadot = plt.subplots(figsize=(7, 4))
    fig_thetadot.canvas.manager.set_window_title("Stage4 Telemetry (PLOTMODE) - Theta dot")
    ax_thetadot.set_title("Tilt rate (theta dot)"); ax_thetadot.set_ylabel("deg/s"); ax_thetadot.set_xlabel("t (s)")
    (lThetaDot,) = ax_thetadot.plot([], [], color="C1", label="theta dot (deg/s)")
    txtThetaDot = ax_thetadot.text(0.98, 0.95, "", transform=ax_thetadot.transAxes,
                                    ha="right", va="top", fontsize=12, color="C1", fontweight="bold")

    fig, (ax2, ax3) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.canvas.manager.set_window_title("Stage4 Telemetry (PLOTMODE) - Torque & Wheel")
    ax2.set_title("Torque"); ax2.set_ylabel("N*m")
    ax3.set_title("Wheel");  ax3.set_ylabel("rad/s"); ax3.set_xlabel("t (s)")

    (lTau,)    = ax2.plot([], [], color="C0", label="tau")
    (lTauCmd,) = ax2.plot([], [], color="C1", linestyle="--", label="tau cmd")
    ax2.legend(loc="upper left")
    txtTau = ax2.text(0.98, 0.95, "", transform=ax2.transAxes,
                       ha="right", va="top", fontsize=10, color="C0", fontweight="bold")
    txtTauCmd = ax2.text(0.98, 0.82, "", transform=ax2.transAxes,
                          ha="right", va="top", fontsize=10, color="C1", fontweight="bold")

    (lWheelLp,) = ax3.plot([], [], color="C0", label="wheel omega lp")
    ax3.legend(loc="upper left")
    txtWheelLp = ax3.text(0.98, 0.95, "", transform=ax3.transAxes,
                           ha="right", va="top", fontsize=10, color="C0", fontweight="bold")

    fig_theta.tight_layout()
    fig_thetadot.tight_layout()
    fig.tight_layout()

    log = {"t": [], "theta": [], "theta_dot": [], "tau": [], "tau_cmd": [],
           "wheel_lp": [], "rows": [], "t0": None}

    def update(_frame):
        got_any = False
        while True:
            try:
                vals = q.get_nowait()
            except queue.Empty:
                break
            got_any = True

            log["rows"].append(vals)
            if log["t0"] is None:
                log["t0"] = vals[0]
            t_s = (vals[0] - log["t0"]) / 1000.0

            log["t"].append(t_s)
            log["theta"].append(vals[1])
            log["theta_dot"].append(vals[2])
            log["tau"].append(vals[3])
            log["tau_cmd"].append(vals[4])
            log["wheel_lp"].append(vals[7])

        artists = (lTheta, lThetaDot, lTau, lTauCmd, lWheelLp,
                   txtTheta, txtThetaDot, txtTau, txtTauCmd, txtWheelLp)
        if not got_any:
o            return artists

        lTheta.set_data(log["t"], log["theta"])
        lThetaDot.set_data(log["t"], log["theta_dot"])
        lTau.set_data(log["t"], log["tau"])
        lTauCmd.set_data(log["t"], log["tau_cmd"])
        lWheelLp.set_data(log["t"], log["wheel_lp"])

        txtTheta.set_text(f"{log['theta'][-1]:.2f} deg")
        txtThetaDot.set_text(f"{log['theta_dot'][-1]:.2f} deg/s")
        txtTau.set_text(f"tau: {log['tau'][-1]:.3f} N*m")
        txtTauCmd.set_text(f"tau cmd: {log['tau_cmd'][-1]:.3f} N*m")
        txtWheelLp.set_text(f"{log['wheel_lp'][-1]:.3f} rad/s")

        t_s = log["t"][-1]
        lo, hi = max(0.0, t_s - WINDOW_S), max(WINDOW_S, t_s)

        ax_theta.set_xlim(lo, hi)   # ylim stays fixed at (-30, 30), no autoscale

        ax_thetadot.set_xlim(lo, hi)
        ax_thetadot.relim()
        ax_thetadot.autoscale_view(scalex=False)

        for ax in (ax2, ax3):
            ax.set_xlim(lo, hi)
            ax.relim()
            ax.autoscale_view(scalex=False)

        # FuncAnimation only auto-redraws the figure it's attached to (fig);
        # the theta/theta_dot figures share the same GUI event loop but need
        # an explicit nudge to pick up their new data each tick.
        fig_theta.canvas.draw_idle()
        fig_thetadot.canvas.draw_idle()

        return artists

    # FuncAnimation drives redraws on a timer owned by the GUI's own event
    # loop; plt.show() below is the ONLY blocking call on this thread, so
    # the window's message pump is serviced continuously. plt.show() blocks
    # until ALL open figures (theta, theta_dot, torque/wheel) are closed.
    ani = FuncAnimation(fig, update, interval=REDRAW_MS, cache_frame_data=False)
    plt.show()

    stop_event.set()
    thread.join(timeout=1.0)
    ser.close()

    rows = log["rows"]
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

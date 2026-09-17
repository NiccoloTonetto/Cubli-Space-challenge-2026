"""imu_orientation_plotter_quaternion.py

Live 3D orientation viewer for the Skeleton_3Axis.ino / Skeleton_3Axis_WiFi.ino
firmware running with TELEMETRY_MODE set to PLOTMODE, QUATERNION variant. In
that mode the Teensy sends one plain CSV line per control cycle, no
header/comment lines:

    t_ms,q0,q1,q2,q3,wx_dps,wy_dps,wz_dps,tau_x,tau_y,tau_z,armed,gain_scale,
    wheelX_pos,wheelX_vel,wheelY_pos,wheelY_vel,wheelZ_pos,wheelZ_vel

q0..q3 is the attitude quaternion, Hamilton convention, scalar-first
(w,x,y,z), mapping body -> inertial (see
docs/dynamics/Quaternions-Complete-Guide.md). wx/wy/wz is body angular rate
in deg/s -- shown here purely as a live readout so you can cross-check it
against the rendered rotation (e.g. rotate the IMU about its body X axis
and confirm both wx's sign and the cube's on-screen rotation match what
you expect for that axis).

This is the quaternion-input counterpart to ../Euler/imu_orientation_plotter_euler.py,
which expects roll_deg,pitch_deg,yaw_deg fields instead of q0..q3 -- pick
whichever one matches what the firmware is actually printing, since the two
wire formats aren't interchangeable.

Python equivalent in spirit to matlab/2Dmodel/Validation/telemetry_python.py:
same background-thread-reads-serial-into-a-queue architecture, same
FuncAnimation-drives-the-only-blocking-call-on-the-main-thread structure,
same session-CSV-on-close behavior. See that file for the full rationale
on why serial I/O and the GUI loop are split across threads.

NOTE: as of this writing, Skeleton_3Axis.ino's calculateState() is a stub
that always reports the identity quaternion {1,0,0,0} -- wx/wy/wz already
reflect real gyro motion, but q0..q3 will stay frozen until the attitude
estimator (Madgwick/Mahony/EKF) is actually implemented there. This script
reads q0..q3 directly regardless, so it starts rendering real orientation
the moment that estimator lands, with no changes needed here.

Requires: pip install pyserial matplotlib numpy

Usage:
    1. In Skeleton_3Axis.ino, set TELEMETRY_MODE to PLOTMODE and re-upload.
    2. Set PORT below to the Teensy's serial port (Windows: check Device
       Manager -> Ports, e.g. "COM9").
    3. Run this script. A window opens with a live-rotating cube.
       Or run with --demo to preview the renderer with synthetic rotation
       and no hardware attached at all:
       python imu_orientation_plotter_quaternion.py --demo
    4. In the terminal (not the plot window), type commands like a1, g0.5,
       h1 and press Enter to send them to the Teensy (same grammar as
       telemetry_python.py; add t0/t1/k if using the _WiFi build's link-mode
       commands -- note the _WiFi build uses 'k' as its heartbeat, not 'h',
       since 'h' means HALT there).
    5. Close the window when you're done -- the full session is saved to
       a timestamped .csv file next to this script.
"""

import argparse
import csv
import math
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---- settings ----
PORT = "COM9"        # <-- set to your Teensy's port
BAUD = 115200
NUM_COLS = 19
REDRAW_MS = 33        # plot redraw period (~30 fps)
OUT_DIR = Path(__file__).resolve().parent

COL_NAMES = ["t_ms", "q0", "q1", "q2", "q3", "wx_dps", "wy_dps", "wz_dps",
             "tau_x", "tau_y", "tau_z", "armed", "gain_scale",
             "wheelX_pos", "wheelX_vel", "wheelY_pos", "wheelY_vel",
             "wheelZ_pos", "wheelZ_vel"]

# Unit cube (edge length 1, centered at origin) with a distinct color per
# face, so a 90-degree-off rotation is obvious to spot on screen -- an
# all-white cube looks the same rotated wrong.
_CUBE_VERTS = np.array([
    [-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5],
])
_CUBE_FACES = [
    ([0, 1, 2, 3], "#b71c1c"),  # -Z, dark red
    ([4, 5, 6, 7], "#e53935"),  # +Z, red
    ([0, 1, 5, 4], "#1b5e20"),  # -Y, dark green
    ([2, 3, 7, 6], "#43a047"),  # +Y, green
    ([0, 3, 7, 4], "#0d47a1"),  # -X, dark blue
    ([1, 2, 6, 5], "#1e88e5"),  # +X, blue
]

AXIS_LEN = 1.0


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def quat_to_rotmat(q):
    """Hamilton, scalar-first (w,x,y,z) -> 3x3 rotation matrix R such that
    v_inertial = R @ v_body, matching the firmware's stated convention."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler_deg(q):
    """Roll (X), pitch (Y), yaw (Z) in degrees, intrinsic Z-Y-X -- for the
    on-screen readout only. The render itself uses the rotation matrix
    directly, so it has no gimbal-lock ambiguity even if this does."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def reader_thread(ser, q, stop_event):
    """Blocking serial reads live entirely on this thread -- the GUI thread
    never waits on I/O, so it can keep servicing its event loop."""
    last_valid = time.monotonic()
    last_warn = 0.0
    warned_bad_shape = False
    last_armed = None

    while not stop_event.is_set():
        raw = ser.readline()
        if not raw:
            now = time.monotonic()
            if now - last_valid > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("... no valid PLOTMODE lines in the last 2s -- check "
                      "that Skeleton_3Axis.ino's TELEMETRY_MODE is set to "
                      "PLOTMODE (re-upload after changing it), that it's "
                      "printing quaternion fields (q0..q3), and that "
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
                      "text, the firmware is still in SERIALMONITORMODE. If "
                      "the field count is close but off by one, the "
                      "firmware may be printing Euler angles instead -- use "
                      "../Euler/imu_orientation_plotter_euler.py for that.")
            continue
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            continue

        last_valid = time.monotonic()

        armed = int(vals[11])
        if last_armed is not None and armed != last_armed:
            print(f"Teensy: {'ARMED' if armed else 'DISARMED'}")
        last_armed = armed

        q.put(vals)


def demo_thread(q, stop_event):
    """Synthetic rotating quaternion, no hardware needed -- lets you check
    the renderer/axes/colors are right before trusting it against real IMU
    data. Rotates steadily about a fixed, tilted axis."""
    axis = np.array([0.4, 0.8, 0.4])
    axis /= np.linalg.norm(axis)
    rate_dps = 45.0
    t0 = time.monotonic()
    angle = 0.0
    last = t0
    while not stop_event.is_set():
        now = time.monotonic()
        dt = now - last
        last = now
        angle += math.radians(rate_dps) * dt
        half = angle / 2.0
        w = math.cos(half)
        x, y, z = axis * math.sin(half)
        wx, wy, wz = axis * rate_dps  # deg/s, constant here by construction
        t_ms = (now - t0) * 1000.0
        vals = [t_ms, w, x, y, z, wx, wy, wz,
                0.0, 0.0, 0.0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q.put(vals)
        time.sleep(0.01)


def command_thread(ser, stop_event):
    """Forwards lines typed in this terminal straight to the Teensy -- same
    a<0/1> / g<0..1> / h<0/1> (/ t<0/1> / k for the _WiFi build) commands
    handleSerialCommands() already understands. Runs on its own thread
    since input() blocks."""
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true",
                         help="synthetic rotating cube, no serial hardware needed")
    args = parser.parse_args()

    q = queue.Queue()
    stop_event = threading.Event()
    ser = None

    if args.demo:
        print("Demo mode: synthetic rotation, no serial connection opened.")
        thread = threading.Thread(target=demo_thread, args=(q, stop_event), daemon=True)
        thread.start()
    else:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        print(f"Opened {PORT} @ {BAUD} baud. Waiting for PLOTMODE data "
              f"({NUM_COLS} comma-separated fields per line)...")
        thread = threading.Thread(target=reader_thread, args=(ser, q, stop_event), daemon=True)
        thread.start()
        cmd_thread = threading.Thread(target=command_thread, args=(ser, stop_event), daemon=True)
        cmd_thread.start()
        print("Type a1 / a0 / g<0..1> / h<0/1> and press Enter to send to "
              "the Teensy (same commands as the Serial Monitor).")

    fig = plt.figure(figsize=(8, 8))
    fig.canvas.manager.set_window_title(
        "IMU Orientation - Quaternion" + (" (DEMO)" if args.demo else " (PLOTMODE)"))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect([1, 1, 1])
    lim = 1.3
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")

    # Fixed world/inertial axes, dashed and gray, drawn once -- compare
    # these against the solid colored body axes below to validate the
    # IMU's reference frame.
    for vec, label in [((AXIS_LEN, 0, 0), "X"), ((0, AXIS_LEN, 0), "Y"), ((0, 0, AXIS_LEN), "Z")]:
        ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], linestyle="--", color="gray", linewidth=1)
        ax.text(vec[0] * 1.1, vec[1] * 1.1, vec[2] * 1.1, f"{label}_world", color="gray", fontsize=8)

    cube = Poly3DCollection([], edgecolor="k", linewidths=0.5)
    ax.add_collection3d(cube)

    # Solid body axes: red=X, green=Y, blue=Z, rotating with the cube.
    body_axis_lines = [ax.plot([], [], [], linewidth=3, color=c)[0]
                        for c in ("red", "green", "blue")]

    txt = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top",
                     fontsize=9, family="monospace")
    ax.text2D(0.02, 0.02,
              "body axes: X=red Y=green Z=blue (solid)\nworld axes: dashed gray",
              transform=ax.transAxes, va="bottom", fontsize=8, color="dimgray")

    log = {"rows": []}

    def update(_frame):
        latest = None
        while True:
            try:
                latest = q.get_nowait()
                log["rows"].append(latest)
            except queue.Empty:
                break
        if latest is None:
            return (cube, txt, *body_axis_lines)

        quat = quat_normalize(latest[1:5])
        w_dps = latest[5:8]
        R = quat_to_rotmat(quat)

        verts_rot = _CUBE_VERTS @ R.T
        faces = [[verts_rot[i] for i in idx] for idx, _ in _CUBE_FACES]
        cube.set_verts(faces)
        cube.set_facecolor([c for _, c in _CUBE_FACES])

        for line, axis_vec in zip(body_axis_lines, np.eye(3) * AXIS_LEN):
            end = R @ axis_vec
            line.set_data([0, end[0]], [0, end[1]])
            line.set_3d_properties([0, end[2]])

        roll, pitch, yaw = quat_to_euler_deg(quat)
        txt.set_text(
            f"q  = [{quat[0]:+.3f} {quat[1]:+.3f} {quat[2]:+.3f} {quat[3]:+.3f}]\n"
            f"rpy= [{roll:+7.2f} {pitch:+7.2f} {yaw:+7.2f}] deg\n"
            f"w  = [{w_dps[0]:+7.2f} {w_dps[1]:+7.2f} {w_dps[2]:+7.2f}] deg/s"
        )

        return (cube, txt, *body_axis_lines)

    ani = FuncAnimation(fig, update, interval=REDRAW_MS, cache_frame_data=False)
    plt.show()

    stop_event.set()
    thread.join(timeout=1.0)
    if ser is not None:
        ser.close()

    rows = log["rows"]
    if rows:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_file = OUT_DIR / f"imu_orientation_quat_{stamp}.csv"
        with out_file.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(COL_NAMES)
            writer.writerows(rows)
        print(f"Saved {len(rows)} samples to {out_file}")
    else:
        print("No data captured -- nothing saved.")


if __name__ == "__main__":
    main()

"""app.py  --  local web dashboard for the WiFi balance builds.

Serves http://127.0.0.1:8000 with live charts, a 3D cube showing real
orientation and the resolved pivot, and buttons for the actual firmware
command grammar. Same UDP transport, same heartbeat discipline and same CSV
format as telemetry_python_wifi_corner.py / telemetry_python_wifi.py, which
are left untouched -- this is a parallel front end, not a replacement for the
files themselves.

    pip install -r requirements.txt
    python app.py                       # talks to the XIAO at 172.20.10.14
    python app.py --target 127.0.0.1    # desk test, paired with replay.py

PORT CONFLICT: this binds UDP :4210, so it cannot run at the same time as the
matplotlib scripts or terminal_wifi.py. Close those first.

WHY THREADS AND NOT ASYNCIO FOR THE I/O
---------------------------------------
CornerBalance_WiFi.ino:170 sets kLinkTimeoutMs = 300, and an armed cube
auto-disarms if the link goes quiet for that long. The 100 ms 'h' heartbeat is
therefore the single most safety-critical thing this process does, and it must
not queue behind JSON serialization, a slow browser socket or a GC pause. So
every piece of I/O stays on a plain thread -- the pattern the existing scripts
already prove -- and asyncio is used only to fan telemetry out to browsers,
where being late costs nothing but a stuttery chart.

The thread -> asyncio handoff is a bounded deque. append() and popleft() are
atomic under the GIL, so it needs no lock, and the broadcaster drains it on
its own 20 Hz timer instead of the reader waking the event loop 250-500 times
a second.
"""

import argparse
import asyncio
import collections
import contextlib
import csv
import json
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

import schemas

# ---- settings ----
XIAO_IP = "172.20.10.14"      # must match XIAO_IP in xiao_teensy_bridge.ino
XIAO_PORT = 4210              # must match UDP_PORT in xiao_teensy_bridge.ino
LOCAL_PORT = 4210
HEARTBEAT_INTERVAL_S = 0.1    # well under the firmware's 300 ms kLinkTimeoutMs
BROADCAST_HZ = 20             # browser update rate; display only
MAX_ROWS_PER_BATCH = 50       # beyond this we stride-sample, CSV still gets all
STALE_AFTER_S = 1.0           # no packets for this long -> link shown as stale
CSV_FLUSH_S = 0.5

HERE = Path(__file__).resolve().parent
STATIC_DIR = HERE / "static"
# HERE = FINAL/WIFIMODE/dashboard, so parents[1] == FINAL/. Recordings land
# beside the ones the matplotlib scripts write, and plot_session_csv.py finds
# them with no changes.
OUT_DIR = HERE.parents[1] / "telemetry" / "plot"


class Hub:
    """Everything shared between the I/O threads and the event loop.

    One object rather than module globals so replay/test code can build an
    isolated instance, and so the locking story is stated in one place:
    the deques are lock-free (GIL-atomic append/popleft), `state` is guarded
    by `lock` because it is read and written as a group.
    """

    def __init__(self, target_ip, target_port, read_only=False):
        self.target = (target_ip, target_port)
        # Monitor mode: refuse a1 outright. That single command is exactly
        # sufficient, because gArmed gates the control output -- disarmed, the
        # law commands no torque no matter what else is sent. Everything else
        # is either observational (c/e re-resolve, which only reads the IMU and
        # prints) or safety-increasing (a0, p1), so blocking more would just
        # take away the controls you want during a watch-only run.
        self.read_only = read_only
        self.sock = None
        self.stop = threading.Event()
        self.lock = threading.Lock()

        # thread -> asyncio handoff
        self.display = collections.deque(maxlen=4000)
        self.console = collections.deque(maxlen=500)
        self.csv_q = queue.Queue()

        self.clients = set()
        self.loop = None

        # recording
        self.rec_path = None
        self.rec_rows = 0

        self.state = {
            "schema": None,          # 'corner' | 'edge' | None until first packet
            "schema_locked": False,  # True once the operator overrides auto-detect
            "armed": False,
            "halted": False,
            "gain": 1.0,
            "pivot": None,           # dict from schemas.pivot_info(), or None
            "phi_norm_deg": None,
            "arm_gate_deg": schemas.ARM_GATE_ASSUMED_DEG,
            "arm_gate_source": "assumed",
            "last_rx": 0.0,
            "rate_hz": 0.0,
            "packets": 0,
            "bad_shape": 0,
        }

    # -- socket ------------------------------------------------------------

    def open_socket(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", LOCAL_PORT))
        s.settimeout(1.0)

        # Windows only: without this, sending to a port nobody is listening on
        # makes the NEXT recvfrom() raise ConnectionResetError from the ICMP
        # Port Unreachable. That happens constantly during a desk test (the
        # heartbeat goes to a XIAO that isn't there) and would kill the reader
        # thread on the first tick.
        #
        # CPython's socket.ioctl() whitelists its control codes and raises
        # ValueError on this one, so the call is best-effort -- the reader's
        # own ConnectionResetError handler is what actually keeps the stream
        # alive, and this only spares it the work when the platform allows.
        if sys.platform == "win32":
            SIO_UDP_CONNRESET = 0x9800000C
            with contextlib.suppress(OSError, ValueError, AttributeError):
                s.ioctl(SIO_UDP_CONNRESET, False)

        self.sock = s
        return s

    def send(self, line):
        """Put one command line on the wire. Never raises into the caller --
        a transient network error must not take down a WebSocket handler."""
        try:
            self.sock.sendto((line + "\n").encode("ascii"), self.target)
            return True
        except OSError as exc:
            self.push_console(f"# [dashboard] send failed: {exc}", "warn")
            return False

    def push_console(self, line, level="info"):
        self.console.append({"line": line, "level": level,
                             "t": time.time()})


HUB = None


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------

def reader_thread(hub):
    """Blocking recvfrom lives here and nowhere else.

    Classifies each packet exactly as the existing scripts do: '#' lines are
    firmware console output and are surfaced, not parsed; everything else must
    be a full-width CSV row or it is dropped.
    """
    rate_window = collections.deque(maxlen=200)

    while not hub.stop.is_set():
        try:
            raw, _addr = hub.sock.recvfrom(2048)
        except socket.timeout:
            continue
        except ConnectionResetError:
            # See the SIO_UDP_CONNRESET note above -- an ICMP bounce from a
            # heartbeat to an absent XIAO. Not a stream error.
            continue
        except OSError:
            if hub.stop.is_set():
                break
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        if line.startswith("#"):
            updates, level = schemas.classify_console(line)
            hub.push_console(line, level)
            if updates:
                apply_console_updates(hub, updates)
            continue

        # Tab OR comma: SERIALMONITORMODE is tab-delimited with a header row,
        # PLOTMODE is comma with none. Both are in active use.
        parts = schemas.split_fields(line)
        schema = schemas.BY_FIELD_COUNT.get(len(parts))
        if schema is None:
            with hub.lock:
                hub.state["bad_shape"] += 1
                n = hub.state["bad_shape"]
            if n == 1:
                hub.push_console(
                    f"# [dashboard] unrecognised telemetry width: {len(parts)} "
                    f"fields. Known widths are "
                    f"{sorted(schemas.BY_FIELD_COUNT)}. If the firmware gained "
                    f"columns, add the schema to dashboard/schemas.py (and see "
                    f"plot_session_csv.py, which tracks the same table).",
                    "warn")
            continue

        try:
            vals = [float(p) for p in parts]
        except ValueError:
            # SERIALMONITORMODE reprints its header row on boot; it has the
            # right width and non-numeric cells, so it lands here. Not an error.
            continue

        now = time.monotonic()
        rate_window.append(now)

        spec = schemas.SCHEMAS[schema]
        armed = bool(int(vals[spec["armed_idx"]]))
        # The Stage5 endurance builds put trip_reason where gain_scale sits on
        # every other corner build, so they report no gain rather than a trip
        # code mislabelled as one.
        gi = spec["gain_idx"]
        gain = vals[gi] if gi is not None else None

        if spec["family"] == "corner":
            phi_norm = (vals[1] ** 2 + vals[2] ** 2 + vals[3] ** 2) ** 0.5
        else:
            phi_norm = abs(vals[1])

        rate = 0.0
        if len(rate_window) > 1:
            span = rate_window[-1] - rate_window[0]
            if span > 0:
                rate = (len(rate_window) - 1) / span

        with hub.lock:
            prev_schema = hub.state["schema"]
            prev_armed = hub.state["armed"]
            if not hub.state["schema_locked"]:
                hub.state["schema"] = schema
            hub.state.update({
                "armed": armed, "gain": gain, "phi_norm_deg": phi_norm,
                "last_rx": now, "rate_hz": rate,
                "packets": hub.state["packets"] + 1,
            })

        # A schema change mid-session means the Teensy was reflashed. Say so,
        # and ask the new build which pivot it resolved -- the ID never appears
        # in the data stream, only in a '#' line, and only on request.
        if schema != prev_schema:
            hub.push_console(
                f"# [dashboard] {schema.upper()} build detected "
                f"({spec['n']} fields, ~{spec['rate_hz']:.0f} Hz)", "info")
            hub.send(spec["resolve_cmd"])
            rate_window.clear()

        if prev_armed != armed:
            hub.push_console(
                f"# [dashboard] Teensy: {'ARMED' if armed else 'DISARMED'}",
                "warn" if not armed else "info")

        hub.csv_q.put((schema, vals))
        hub.display.append((schema, vals))


def apply_console_updates(hub, updates):
    """Merge parsed '#'-line facts into hub.state."""
    with hub.lock:
        if "pivot_name" in updates:
            info = schemas.pivot_info(updates.get("pivot_schema"),
                                      updates["pivot_name"])
            hub.state["pivot"] = info or {"name": updates["pivot_name"],
                                          "kind": updates.get("pivot_schema"),
                                          "gB": None}
        if "halted" in updates:
            hub.state["halted"] = updates["halted"]
        if "gain" in updates:
            hub.state["gain"] = updates["gain"]
        if "arm_gate_deg" in updates:
            hub.state["arm_gate_deg"] = updates["arm_gate_deg"]
            hub.state["arm_gate_source"] = "learned"
        # armed_echo is deliberately ignored: the telemetry column is
        # authoritative and also catches a self-disarm on a trip.


def heartbeat_thread(hub):
    """The safety-critical loop. Nothing else runs on this thread.

    Sends 'k', NOT 'h'. 'k' is a no-op keepalive on every build in the tree;
    'h' is only a keepalive on the legacy CornerBalance_WiFi/EdgeBalance_WiFi
    pair, and on the AutoTrim builds it is HALT -- a bare 'h' there parses as
    h0 and resumes from halt ten times a second. See schemas.KEEPALIVE.

    It also keeps the XIAO's learned laptop address fresh -- xiao_teensy_bridge
    .ino only knows where to send telemetry once it has heard from us.
    """
    while not hub.stop.is_set():
        try:
            hub.sock.sendto(schemas.KEEPALIVE, hub.target)
        except OSError:
            pass    # a dropped heartbeat is recoverable; a dead thread is not
        hub.stop.wait(HEARTBEAT_INTERVAL_S)


def csv_thread(hub):
    """Drains the full-rate queue to disk.

    File I/O is here rather than on the reader so a slow flush can never stall
    recvfrom, and rather than on the event loop so it can never delay a
    broadcast. Writing is incremental (the matplotlib scripts buffer the whole
    session in RAM and flush on window close) because the UI has a Stop button
    and a run that ends in a crash should still leave a usable file.
    """
    writer = None
    handle = None
    active_schema = None
    last_flush = time.monotonic()

    def close():
        nonlocal writer, handle, active_schema
        if handle:
            handle.flush()
            handle.close()
        writer = handle = active_schema = None

    while True:
        try:
            item = hub.csv_q.get(timeout=0.25)
        except queue.Empty:
            item = None

        if item is None:
            if hub.stop.is_set():
                break
            if handle and time.monotonic() - last_flush > CSV_FLUSH_S:
                handle.flush()
                last_flush = time.monotonic()
            continue

        if item is _STOP_RECORDING:
            close()
            continue

        schema, vals = item
        if hub.rec_path is None:
            continue    # not recording; drop on the floor

        # Roll to a new file if the build changed under us -- the two formats
        # have different widths and must never share a file.
        if active_schema is not None and schema != active_schema:
            close()

        if handle is None:
            spec = schemas.SCHEMAS[schema]
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            path = OUT_DIR / f"{spec['prefix']}_{stamp}.csv"
            handle = path.open("w", newline="")
            writer = csv.writer(handle)
            writer.writerow(spec["cols"])
            active_schema = schema
            hub.rec_path = path
            hub.rec_rows = 0
            hub.push_console(f"# [dashboard] recording -> {path.name}", "info")

        writer.writerow(vals)
        hub.rec_rows += 1

        if time.monotonic() - last_flush > CSV_FLUSH_S:
            handle.flush()
            last_flush = time.monotonic()

    close()


_STOP_RECORDING = object()


# ---------------------------------------------------------------------------
# asyncio side
# ---------------------------------------------------------------------------

def stride_sample(rows, limit):
    """Thin `rows` to at most `limit`, keeping the newest.

    Display-only: the CSV thread has already seen every one of these.
    """
    if len(rows) <= limit:
        return rows
    step = len(rows) / limit
    return [rows[min(int(i * step), len(rows) - 1)] for i in range(limit)]


def build_status(hub):
    with hub.lock:
        st = dict(hub.state)
    age = (time.monotonic() - st["last_rx"]) if st["last_rx"] else None
    return {
        "type": "status",
        "schema": st["schema"],
        # The five corner widths share one chart layout and one 3D scene, so
        # the frontend keys off the family and only shows the exact schema.
        "family": (schemas.SCHEMAS[st["schema"]]["family"]
                   if st["schema"] else None),
        "halt_cmd": (schemas.SCHEMAS[st["schema"]]["halt_cmd"]
                     if st["schema"] else None),
        "schema_locked": st["schema_locked"],
        "armed": st["armed"],
        "halted": st["halted"],
        "gain": st["gain"],
        "pivot": st["pivot"],
        "phi_norm_deg": st["phi_norm_deg"],
        "arm_gate_deg": st["arm_gate_deg"],
        "arm_gate_source": st["arm_gate_source"],
        "arm_gate_suspect": st["arm_gate_deg"] > schemas.ARM_GATE_SANE_MAX_DEG,
        "rate_hz": round(st["rate_hz"], 1),
        "packets": st["packets"],
        "link_age_s": round(age, 3) if age is not None else None,
        "stale": age is None or age > STALE_AFTER_S,
        "recording": hub.rec_path.name if hub.rec_path else None,
        "recorded_rows": hub.rec_rows,
        "read_only": hub.read_only,
    }


async def broadcast(hub, payload):
    """Fan out one message. A client that fails is dropped, never awaited on."""
    if not hub.clients:
        return
    text = json.dumps(payload)
    dead = []
    results = await asyncio.gather(
        *(ws.send_text(text) for ws in list(hub.clients)),
        return_exceptions=True)
    for ws, res in zip(list(hub.clients), results):
        if isinstance(res, Exception):
            dead.append(ws)
    for ws in dead:
        hub.clients.discard(ws)


async def broadcaster(hub):
    """The only asyncio task that touches the telemetry path."""
    period = 1.0 / BROADCAST_HZ
    while not hub.stop.is_set():
        await asyncio.sleep(period)
        if not hub.clients:
            hub.display.clear()      # nobody watching; don't let it grow stale
            hub.console.clear()
            continue

        rows = []
        schema = None
        while hub.display:
            schema, vals = hub.display.popleft()
            rows.append(vals)
        if rows:
            await broadcast(hub, {"type": "telemetry", "schema": schema,
                                  "rows": stride_sample(rows, MAX_ROWS_PER_BATCH)})

        while hub.console:
            entry = hub.console.popleft()
            await broadcast(hub, {"type": "console", **entry})

        await broadcast(hub, build_status(hub))


@contextlib.asynccontextmanager
async def lifespan(app):
    hub = HUB
    hub.loop = asyncio.get_running_loop()
    hub.open_socket()

    threads = [threading.Thread(target=fn, args=(hub,), daemon=True, name=name)
               for fn, name in ((reader_thread, "udp-reader"),
                                (heartbeat_thread, "heartbeat"),
                                (csv_thread, "csv-writer"))]
    for t in threads:
        t.start()

    task = asyncio.create_task(broadcaster(hub))
    print(f"  UDP    : listening :{LOCAL_PORT}, sending to "
          f"{hub.target[0]}:{hub.target[1]}")
    print(f"  CSV    : {OUT_DIR}")
    print(f"  UI     : http://127.0.0.1:8000")
    try:
        yield
    finally:
        # Ctrl-C with a recording open must still leave a valid CSV.
        hub.stop.set()
        hub.csv_q.put(_STOP_RECORDING)
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        for t in threads:
            t.join(timeout=2.0)
        if hub.sock:
            hub.sock.close()
        if hub.rec_path:
            print(f"Saved {hub.rec_rows} samples to {hub.rec_path}")


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    hub = HUB
    await ws.accept()
    hub.clients.add(ws)
    await ws.send_text(json.dumps(build_status(hub)))
    try:
        while True:
            msg = json.loads(await ws.receive_text())
            await handle_client_message(hub, msg)
    except (WebSocketDisconnect, json.JSONDecodeError):
        pass
    finally:
        hub.clients.discard(ws)


async def handle_client_message(hub, msg):
    kind = msg.get("type")

    if kind == "cmd":
        cmd = str(msg.get("cmd", "")).strip()
        with hub.lock:
            schema = hub.state["schema"]
            gate = hub.state["arm_gate_deg"]
            phi = hub.state["phi_norm_deg"]

        if schema is None:
            hub.push_console("# [dashboard] rejected: no stream yet, so the "
                             "build is unknown. Wait for telemetry.", "warn")
            return

        if hub.read_only and cmd == "a1":
            hub.push_console("# [dashboard] BLOCKED a1: started with "
                             "--read-only (monitor mode).", "warn")
            return

        if not schemas.command_allowed(schema, cmd):
            # Not shell-escaping paranoia -- this is the only thing standing
            # between a browser tab and three flywheels.
            hub.push_console(
                f"# [dashboard] REJECTED {cmd!r}: not in the {schema} command "
                f"grammar.", "warn")
            return

        # Server-side arm gate. The firmware enforces its own, but a UI that
        # can send a1 at 30 deg of lean and let the cube refuse is a UI that
        # trains you to ignore refusals.
        if cmd == "a1" and phi is not None and phi >= gate:
            hub.push_console(
                f"# [dashboard] REFUSED a1: |phi|={phi:.3f} deg exceeds the "
                f"{gate:.2f} deg arm gate.", "warn")
            return

        # Echo what we put on the wire. Without this a button press is
        # invisible until the firmware answers -- and some commands never
        # answer (the edge build gates most echoes behind SERIALMONITORMODE),
        # so a working press and a dropped UDP packet look identical.
        if hub.send(cmd):
            hub.push_console(f"> {cmd}", "tx")

    elif kind == "record":
        action = msg.get("action")
        if action == "start" and hub.rec_path is None:
            hub.rec_path = _PENDING     # csv_thread opens the real file
            hub.rec_rows = 0
        elif action == "stop" and hub.rec_path is not None:
            path, rows = hub.rec_path, hub.rec_rows
            hub.csv_q.put(_STOP_RECORDING)
            hub.rec_path = None
            name = getattr(path, "name", "(no data)")
            hub.push_console(
                f"# [dashboard] recording stopped: {name} ({rows} rows)", "info")

    elif kind == "mode":
        schema = msg.get("schema")
        with hub.lock:
            if schema in schemas.SCHEMAS:
                hub.state["schema"] = schema
                hub.state["schema_locked"] = True
            else:
                hub.state["schema_locked"] = False
        if schema in schemas.SCHEMAS:
            hub.send(schemas.SCHEMAS[schema]["resolve_cmd"])


class _Pending:
    """Placeholder while the CSV thread has not opened a file yet -- lets the
    UI show 'recording' the instant it is pressed, before the first row."""
    name = "(starting...)"


_PENDING = _Pending()


def main():
    global HUB
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--target", default=XIAO_IP,
                    help="XIAO IP to send commands/heartbeat to "
                         f"(default {XIAO_IP}; use 127.0.0.1 with replay.py)")
    ap.add_argument("--target-port", type=int, default=XIAO_PORT)
    ap.add_argument("--host", default="127.0.0.1",
                    help="web bind address -- keep it loopback unless you "
                         "really want the hotspot to reach the arm button")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--read-only", action="store_true",
                    help="monitor mode: refuse a1 (arm). Telemetry, charts, "
                         "recording, re-resolve, disarm and halt all still work")
    args = ap.parse_args()

    HUB = Hub(args.target, args.target_port, read_only=args.read_only)
    print("Cubli dashboard" + ("  [READ-ONLY: arming blocked]"
                               if args.read_only else ""))
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()

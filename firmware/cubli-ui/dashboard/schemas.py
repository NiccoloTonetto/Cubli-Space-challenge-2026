"""schemas.py  --  wire formats, pivot geometry and console-line parsing.

Everything here is transcribed from the firmware it has to agree with, so
each table cites the file and line it came from. If a sketch changes, this
is the file that has to follow it.

Two wire formats share one UDP stream, told apart purely by field count --
the same test the existing telemetry_python_wifi*.py scripts already make:

    10 fields -> EDGE   (comma, ~500 Hz)
    26 fields -> CORNER (tab,   ~250 Hz)

BOTH NOW COME FROM ONE SKETCH. ../teensy/CubliBalance/CubliBalance.ino holds
both control laws and switches between them at runtime on m0/m1, so the field
count flips mid-session when the operator changes mode. That is not an error
and app.py already handles it: it logs the change, re-sends the resolve
command and rolls the CSV rather than mixing two widths in one file.

The older per-build widths below (21, 29, 33, 36) are kept so recordings made
before the fusion still replay.

The column names are copied verbatim from the telemetry scripts so that a
recording made by this dashboard is byte-compatible with one made by them, and
plot_session_csv.py keeps working on both without modification.
"""

import re

# ---------------------------------------------------------------------------
# Column schemas -- verbatim from telemetry_python_wifi_corner.py:112 and
# telemetry_python_wifi.py:95. Do not reorder: plot_session_csv.py's
# CORNER_COLS/EDGE_COLS match these by position.
# ---------------------------------------------------------------------------

CORNER_COLS = [
    "t_ms",
    "phi_x_deg", "phi_y_deg", "phi_z_deg",
    "om_x_dps", "om_y_dps", "om_z_dps",
    "rho_x", "rho_y", "rho_z",
    "rho_x_lp", "rho_y_lp", "rho_z_lp",
    "tau_x", "tau_y", "tau_z",
    "tau_cmd_x", "tau_cmd_y", "tau_cmd_z",
    "armed", "gain_scale",
]

EDGE_COLS = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm",
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"]

# ---------------------------------------------------------------------------
# The corner family has grown. Widths, and the sketch that emits each, mirror
# ../../plot_session_csv.py -- THAT FILE IS THE SOURCE OF TRUTH for column
# names and it already carries all of these; keep this table in step with it.
# (It is not imported because it pulls in matplotlib at module scope, which the
# server has no business loading.)
#
# Every corner variant keeps the first 19 columns identical, so phi/om/rho/tau
# live at fixed indices no matter which build is flashed -- that is what lets
# the dashboard treat them as one family.
#
# The Stage5 "endurance" variants are the exception worth knowing: their
# column 20 is trip_reason, NOT gain_scale. Reading it as a gain would show a
# trip code in the gain tile, so those schemas declare gain_idx = None.
# ---------------------------------------------------------------------------

_TRIM = ["trim_x_deg", "trim_y_deg", "trim_z_deg", "trim_com_mm", "trim_enabled"]
_FILT = ["om_x_filt_dps", "om_y_filt_dps", "om_z_filt_dps"]

CORNER_TRIM_COLS = CORNER_COLS + _TRIM                      # Stage4_AutoTrim, 26
CORNER_TRIM_FILT_COLS = CORNER_TRIM_COLS + _FILT            # + RateFilter,    29

CORNER_ENDURANCE_COLS = CORNER_COLS[:20] + ["trip_reason"] + _TRIM + [
    "temp_x", "temp_y", "temp_z",
    "sat_duty_x", "sat_duty_y", "sat_duty_z",
    "loop_overrun_count",
]                                                            # Stage5_AutoTrim, 33
CORNER_ENDURANCE_FILT_COLS = CORNER_ENDURANCE_COLS + _FILT   # + RateFilter,   36


def _corner(cols, prefix, gain_idx=20, grammar="autotrim"):
    # halt letter differs by grammar -- see KEEPALIVE / HALT below.
    #
    # arm_policy: what the DASHBOARD does about a1, which is not the same
    # question as what the firmware does.
    #   "gate" -> refuse a1 above arm_gate_deg, as the firmware would
    #   "warn" -> the firmware has NO arm gate; warn loudly, still send
    # The fused CubliBalance sketch and the Stage4/5 corner builds it came
    # from deliberately removed the gate (the control law is only validated
    # within ~3 deg, and enforcing that is operator discipline there). A UI
    # that blocks what the firmware allows trains you to ignore refusals, so
    # corner warns instead. The older legacy 21-column build did have a gate.
    return {"cols": cols, "n": len(cols), "armed_idx": 19, "gain_idx": gain_idx,
            "rate_hz": 250.0,          # gDecim[CORNER] = 2 over a 500 Hz loop
            "prefix": prefix, "resolve_cmd": "c",
            "halt_cmd": "p" if grammar == "legacy" else "h",
            "mode_cmd": "m1",
            "arm_policy": "gate" if grammar == "legacy" else "warn",
            "grammar": grammar, "family": "corner"}


# Per-schema constants. `armed_idx` is the authoritative armed flag -- the
# column, not the '#' echo, because the control law also clears it by itself
# on an overtilt / overspeed / NaN trip and never announces that.
SCHEMAS = {
    "corner": _corner(CORNER_COLS, "telemetry_corner", grammar="legacy"),
    "corner_trim": _corner(CORNER_TRIM_COLS, "telemetry_corner_trim"),
    "corner_trim_filt": _corner(CORNER_TRIM_FILT_COLS, "telemetry_corner_trimfilt"),
    "corner_endurance": _corner(CORNER_ENDURANCE_COLS,
                                "telemetry_corner_endurance", gain_idx=None),
    "corner_endurance_filt": _corner(CORNER_ENDURANCE_FILT_COLS,
                                     "telemetry_corner_endurancefilt", gain_idx=None),
    "edge": {
        "cols": EDGE_COLS,
        "n": len(EDGE_COLS),
        "armed_idx": 5,
        "gain_idx": 6,
        "rate_hz": 500.0,      # kPeriodMs=2, gDecim[EDGE] = 1
        "prefix": "telemetry_edge",
        "resolve_cmd": "e",
        # Was None: the standalone EdgeBalance_WiFi build had no halt command
        # at all, and bound 'h' to the keepalive instead. CubliBalance.ino has
        # one halt for both modes, on 'h'.
        "halt_cmd": "h",
        "mode_cmd": "m0",
        "arm_policy": "gate",  # edge firmware does gate a1 -- see ARM_GATE below
        "family": "edge",
    },
}

BY_FIELD_COUNT = {SCHEMAS[k]["n"]: k for k in SCHEMAS}
assert len(BY_FIELD_COUNT) == len(SCHEMAS), "two schemas share a field count"


# ---------------------------------------------------------------------------
# Edge phi offset -- mirrors CubliBalance.ino's kEdgePhiOffsetDefaultDeg
#
# There is no protocol message that carries this constant from firmware to
# dashboard -- it is just a number both sides happen to agree on. If the
# firmware's hardcoded default ever changes, THIS must be edited to match, or
# the UI's "reset to default" button will send the wrong value.
# ---------------------------------------------------------------------------

EDGE_PHI_OFFSET_DEFAULT_DEG = -0.7


def split_fields(line):
    """Split a telemetry line, tab or comma.

    PLOTMODE emits comma-separated rows with no header. SERIALMONITORMODE emits
    TAB-separated rows WITH a header and interleaved '#' lines. Both builds ship
    with either mode selectable, so the dashboard sniffs per line rather than
    assuming -- assuming PLOTMODE is exactly the bug that made a live cube look
    like it was sending nothing.
    """
    return line.split("\t") if "\t" in line else line.split(",")


# ---------------------------------------------------------------------------
# Arm gate
#
# Three different values are live in the tree and they disagree:
#
#   cubli_gains.h:40            ARM_GATE  0.00872664619 rad = 0.50 deg
#   CornerBalance_WiFi.ino:584  kArmGate  0.01745329252 rad = 1.00 deg
#   CubliBalance.ino            kEdgeArmGate 0.1672664619 rad = 9.58 deg  <-- below
#
# The edge number looks like a corrupted copy of the header's (shared
# '72664619' tail) and the comment beside it still claims 0.5 deg. As flashed,
# edge mode will accept an arm command with the cube leaning ~9.6 deg.
#
# CubliBalance.ino carries that value forward DELIBERATELY -- it is a verbatim
# port of EdgeBalance_WiFi.ino:484, and correcting a control constant while
# also restructuring two sketches into one would make a bench regression
# impossible to attribute. The fix is a separate, deliberate change.
#
# Which means the dashboard is knowingly the TIGHTER of the two: nothing here
# hardcodes the firmware's gate. We assume the conservative 0.5 until the
# firmware states its own value in an ARM REFUSED line, then show what it
# actually said and flag it as implausible -- which, for edge mode, it is.
# ---------------------------------------------------------------------------

ARM_GATE_ASSUMED_DEG = 0.5    # cubli_gains.h:40, the generated truth
ARM_GATE_SANE_MAX_DEG = 1.5   # above this, flag it in the UI rather than trust it

# Corner mode has no arm gate at all, so there is no threshold to refuse at --
# but there IS a number worth saying out loud. The simulation study's recovery
# envelope tops out around 2.7-3.1 deg worst case across corners (see
# Cube-Performance-Envelope-Results.md); the Kp law was linearised around
# equilibrium and is not validated beyond it. Arming past this is allowed and
# warned about, not blocked -- see app.py's arm_policy handling.
CORNER_ENVELOPE_DEG = 3.0


# ---------------------------------------------------------------------------
# Pivot geometry
#
# gB is documented in cubli_gains.h:49 as "body-frame gravity direction at
# balance" -- it points DOWN in body frame, so it points at the feature
# resting on the table. That is the pivot, and it is what the 3D view
# highlights.
#
# Corner names agree with their gB octant ([-1,-1,-1] carries a (-,-,-) gB).
# Edge names do NOT: 'Y[+1,+1] (+X,+Z up)' has gB=(0.707,0,0.707), i.e. +X/+Z
# is down, not up. The parenthetical is mislabelled. So the frontend is driven
# from gB and never from the name text.
# ---------------------------------------------------------------------------

# name -> (gB, place_offset_deg). CornerBalance_WiFi.ino:382, kCorners[8].
CORNERS = {
    "[-1,-1,-1]": ((-0.571549475, -0.588645577, -0.571688354), 0.797),
    "[-1,-1,+1]": ((-0.546220303, -0.562558711,  0.620621562), 3.170),
    "[-1,+1,-1]": ((-0.556852818,  0.616181135, -0.556988120), 2.773),
    "[-1,+1,+1]": ((-0.533349514,  0.590173781,  0.605997622), 3.097),
    "[+1,-1,-1]": (( 0.620658159, -0.562471628, -0.546268404), 3.171),
    "[+1,-1,+1]": (( 0.595400333, -0.539581716,  0.595273018), 2.609),
    "[+1,+1,-1]": (( 0.606037736,  0.590086639, -0.533400357), 3.095),
    "[+1,+1,+1]": (( 0.582457066,  0.567126632,  0.582332492), 0.714),
}

# name -> (edge direction e, gB, place_offset_deg).
# EdgeBalance_WiFi.ino:348, kCandidates[3][2]. Only 6 of the cube's 12 edges
# appear: one wheel axis has to lie along the contact line for the single-axis
# law to work, so only the axis-aligned pairs are candidates.
EDGES = {
    "X[+1,+1] (+Y,+Z up)": ((1, 0, 0), (-0.0,  0.697691619,  0.716398239), 0.758),
    "X[-1,-1] (-Y,-Z up)": ((1, 0, 0), (-0.0, -0.717363894, -0.696698666), 0.837),
    "Y[+1,+1] (+X,+Z up)": ((0, 1, 0), ( 0.707182407, -0.0,  0.707031190), 0.006),
    "Y[-1,-1] (-X,-Z up)": ((0, 1, 0), (-0.707020879, -0.0, -0.707192659), 0.007),
    "Z[+1,+1] (+X,+Y up)": ((0, 0, 1), ( 0.716472805,  0.697615027, -0.0), 0.764),
    "Z[-1,-1] (-X,-Y up)": ((0, 0, 1), (-0.696611583, -0.717448473, -0.0), 0.844),
}


def pivot_info(schema, name):
    """Geometry for the resolved pivot, or None if the name is unknown.

    Returned in the shape the frontend wants: gB always present, `e` only for
    edges. Unknown names are not an error -- the firmware can print a name we
    have not transcribed, and the UI should degrade to "no highlight" rather
    than break the stream.
    """
    if schema == "corner" and name in CORNERS:
        gb, offset = CORNERS[name]
        return {"name": name, "kind": "corner", "gB": list(gb),
                "place_offset_deg": offset}
    if schema == "edge" and name in EDGES:
        e, gb, offset = EDGES[name]
        return {"name": name, "kind": "edge", "e": list(e), "gB": list(gb),
                "place_offset_deg": offset}
    return None


# ---------------------------------------------------------------------------
# Console lines
#
# Anything starting with '#' is firmware console output, not data. The corner
# build prints these unconditionally; the edge build gates most of them behind
# SERIALMONITORMODE but still answers 'e' in PLOTMODE, so the resolve line
# arrives on both. Everything below is matched against the exact print
# sequences in the sketches.
# ---------------------------------------------------------------------------

RE_CORNER_RESOLVED = re.compile(r"^#\s*corner resolved:\s*(\[[-+\d,]+\])")
RE_EDGE_RESOLVED = re.compile(
    r"^#\s*axis=(\w+)\s+edge candidate resolved:\s*(\S+\s*\([^)]*\))")
RE_ARMED = re.compile(r"^#\s*gArmed\s*=\s*(TRUE|FALSE)")
RE_HALTED = re.compile(r"^#\s*gHalted\s*=\s*(TRUE|FALSE)")
RE_GAIN = re.compile(r"^#\s*gGainScale\s*=\s*([\d.]+)")
RE_ARM_REFUSED = re.compile(
    r"^#\s*ARM REFUSED:\s*\|phi(?:_edge)?\|=([\d.]+)\s*deg exceeds ARM_GATE=([\d.]+)\s*deg")

# The state machine's own report. CubliBalance.ino prints this on every
# transition, at boot, and on demand ('s'), so the UI's idea of what the cube
# is doing comes from the cube rather than from what we last sent it:
#
#     # state=CORNER_BALANCE mode=CORNER armed=1 halted=0
#
# This is how a mode switch is announced. It could not be a telemetry column:
# every consumer in the tree identifies the build by field count, so widening
# either row would break plot_session_csv.py and every recording on disk.
RE_STATE = re.compile(
    r"^#\s*state=(DISARMED|EDGE_BALANCE|CORNER_BALANCE)\s+mode=(EDGE|CORNER)"
    r"\s+armed=([01])\s+halted=([01])")

# Lines that should stand out in the console pane rather than scroll past.
RE_WARNING = re.compile(
    r"WARNING|REFUSED|DISARMED:|force-disarmed|disarmed by halt", re.IGNORECASE)


def classify_console(line):
    """Parse one '#' line into (state_updates, level).

    state_updates is a dict of fields to merge into the server's view of the
    cube. Empty for lines we only want to display. level is used by the UI to
    colour the console pane.
    """
    updates = {}

    m = RE_CORNER_RESOLVED.match(line)
    if m:
        updates["pivot_name"] = m.group(1)
        updates["pivot_schema"] = "corner"

    m = RE_EDGE_RESOLVED.match(line)
    if m:
        updates["pivot_name"] = m.group(2).strip()
        updates["pivot_schema"] = "edge"

    m = RE_ARMED.match(line)
    if m:
        updates["armed_echo"] = (m.group(1) == "TRUE")

    m = RE_HALTED.match(line)
    if m:
        updates["halted"] = (m.group(1) == "TRUE")

    # The state line is authoritative for mode, and for halted. It is NOT used
    # for armed: the telemetry column is, because the control law also clears
    # gArmed by itself on an overtilt / overspeed / NaN trip and the state line
    # only reprints on a transition it noticed.
    m = RE_STATE.match(line)
    if m:
        updates["cube_state"] = m.group(1)
        updates["mode"] = m.group(2).lower()
        updates["halted"] = (m.group(4) == "1")

    m = RE_GAIN.match(line)
    if m:
        updates["gain"] = float(m.group(1))

    # The one place the firmware states its own arm gate. Worth more than the
    # refusal itself: it is how the UI learns the real threshold.
    m = RE_ARM_REFUSED.match(line)
    if m:
        updates["arm_gate_deg"] = float(m.group(2))
        updates["arm_gate_source"] = "learned"

    level = "warn" if RE_WARNING.search(line) else "info"
    return updates, level


# ---------------------------------------------------------------------------
# KEEPALIVE / HALT -- two grammars live in this tree and they conflict
#
#   CornerBalance_WiFi / EdgeBalance_WiFi   h OR k = keepalive,  p<0/1> = halt
#   Stage4/5_AutoTrim*, and CubliBalance    k      = keepalive,  h<0/1> = halt
#
# Stage4_AutoTrim_RateFilter_WiFi.ino:81 says it outright: "'h' is still halt
# (unlike ../CornerBalance_WiFi/, which predates this folder and binds
# h = keepalive, p = halt)".
#
# So a bare 'h' heartbeat is NOT safe: on those builds it parses as
# atof("") = 0 -> h0 -> RESUME, ten times a second, which quietly undoes the
# HALT button. 'k' is the only letter that is a no-op keepalive on BOTH
# grammars, so that is what the heartbeat sends. terminal_wifi.py:41 reached
# the same conclusion.
#
# CubliBalance.ino inherited the corner grammar wholesale, so 'h' is HALT in
# BOTH of its modes -- including edge, where the standalone build used to
# accept 'h' as a keepalive precisely so the old plotters would work
# unmodified. Those plotters were changed to send 'k'; if you resurrect an
# older copy of either, check its heartbeat before arming.
#
# HALT_CMD per schema follows the same split, keyed off field count, which is
# an exact proxy: 21 columns is the legacy CornerBalance_WiFi build, 26+ is
# AutoTrim, Stage5 or CubliBalance.
# ---------------------------------------------------------------------------

KEEPALIVE = b"k\n"


# ---------------------------------------------------------------------------
# Command whitelist
#
# The browser is an untrusted input to a machine that spins three flywheels.
# Only these exact forms reach the socket; anything else is rejected and
# logged. The keepalive is generated by the heartbeat thread, never a client.
# ---------------------------------------------------------------------------

_GAIN = r"g(?:0(?:\.\d+)?|1(?:\.0+)?)"          # g0 .. g1, clamped firmware-side anyway
_LINK = r"[tx][01]"                              # Teensy link mode / XIAO mode

# m0/m1 is the runtime mode switch and 's' asks for the state line. Both are
# CubliBalance.ino additions and both are safe on any build that predates them:
# an unknown letter is answered with a usage line, never acted on.
_MODE = r"m[01]|s"

# p/r/w<0..3>: CubliBalance.ino's live LQR tilt/rate/wheel gain-term scales
# (see its SECTION 5 note). Both modes accept all three letters -- there is no
# "wrong mode" to refuse, each mode keeps its own copy of the three scales, so
# setting one while the other mode is staged just stores it for later. Range
# isn't enforced here (the firmware clamps to kGainScaleTermMax); the
# whitelist only needs to accept the shape. Signed (like the archive board's
# original r<val> pattern below) even though p/r/w gain scales are clamped
# non-negative firmware-side -- the clamp is the real boundary, matching how
# gGainScale's own clamp works; the whitelist stays permissive rather than
# guessing which firmware is listening.
#
# NOTE: 'r' already existed below, inside "autotrim", as the archive AutoTrim
# board's log-marker letter (unrelated meaning, r<val> = a marker value to
# stamp into that board's log). CubliBalance.ino's corner mode does not
# implement a log marker; on THIS firmware 'r' is the rate-term gain scale.
# The whitelist doesn't need to disambiguate -- the shape is identical either
# way, and it's whichever firmware is actually running that decides what the
# letter does.
_GAINTERM = r"[prw]-?\d+(?:\.\d+)?"

CMD_OK = {
    # legacy 21-column corner: halt is p, z tares
    "legacy": re.compile(r"^(?:a[01]|" + _GAIN + r"|c|z[01]|p[01]|" + _LINK + r")$"),
    # AutoTrim / Stage5 / CubliBalance corner mode: halt is h, trim replaces
    # the manual z tare, plus the WiFi-only controls on letters no corner stage
    # uses (l link, d decim) and the trim knobs (y freeze, n adapt gain,
    # f filter Hz, r log marker on the archive board / rate-scale on CubliBalance).
    "autotrim": re.compile(
        r"^(?:a[01]|" + _GAIN + r"|c|h[01]|y[01]|n[\d.eE+-]+|f\d+(?:\.\d+)?"
        r"|d\d{1,3}|l[01]|" + _MODE + r"|" + _LINK + r"|" + _GAINTERM + r")$"),
    # CubliBalance edge mode. Two changes vs the standalone EdgeBalance_WiFi
    # grammar: link mode moved from 't' to the corner build's 'l' (the fusion
    # kept one letter, not both), and halt/decim exist here because the fused
    # sketch has one halt and one decimation control shared by both modes.
    # 't' stays only inside _LINK for the XIAO's own passthrough.
    "edge": re.compile(
        r"^(?:a[01]|" + _GAIN + r"|o-?\d+(?:\.\d+)?|e|h[01]|d\d{1,3}|l[01]"
        r"|" + _MODE + r"|" + _LINK + r"|" + _GAINTERM + r")$"),
}


def command_allowed(schema, cmd):
    """True if `cmd` is a command this schema's firmware actually understands."""
    spec = SCHEMAS.get(schema)
    if not spec:
        return False
    key = "edge" if spec["family"] == "edge" else spec["grammar"]
    pattern = CMD_OK.get(key)
    return bool(pattern and pattern.match(cmd))

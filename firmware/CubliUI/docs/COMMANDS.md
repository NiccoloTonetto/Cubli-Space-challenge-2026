# Command reference

Commands are single lines of text sent to the Teensy over the UART link. The
dashboard sends them, and you can type the same letters into
`tools/terminal_wifi.py` or an Arduino Serial Monitor.

**There is one Teensy program now.** `teensy/CubliBalance/CubliBalance.ino`
holds both control laws and switches between them at runtime, so there is one
grammar and no per-sketch column to check.

---

## Command table

| Command | Mode | Meaning |
|---|:--:|---|
| `a1` | both | **ARM** — start balancing in whichever mode is staged |
| `a0` | both | **DISARM / E-STOP** |
| `m0` | both | **Switch to EDGE mode** — force-disarms first |
| `m1` | both | **Switch to CORNER mode** — force-disarms first |
| `g<0..1>` | both | Gain scale, e.g. `g0.5` = half authority |
| `c` | corner | Re-resolve which **corner** is down |
| `e` | edge | Re-resolve which **edge** is down |
| `o<deg>` | edge | Edge tilt trim, **on top of** the hardcoded `-0.7°` default, e.g. `o0.25` |
| `p<0..3>` | both | LQR **tilt**-term gain scale — this mode's own copy, default `1.0` |
| `r<0..3>` | both | LQR **rate**-term gain scale — this mode's own copy, default `1.0` |
| `w<0..3>` | both | LQR **wheel**-term gain scale — this mode's own copy, default `1.0` |
| `h1` / `h0` | both | **HALT / resume** — halt also disarms |
| `d<N>` | both | Telemetry decimation for the **current** mode (`d2` = 250 Hz) |
| `l1` / `l0` | both | Choose output stream (WiFi / USB) |
| `s` | both | Print the state line |
| `k` | both | No-op heartbeat |

### `p` / `r` / `w` — live LQR gain scaling

Three multiplicative scale factors, grouped by physical term rather than by
raw matrix entry (corner's gain is a full 3×9 matrix per corner — 216 numbers
across all eight corners, so per-element control was never practical):

- **Tilt** (`p`) scales the phi/position-error columns
- **Rate** (`r`) scales the body-rate/angular-velocity columns
- **Wheel** (`w`) scales the wheel-speed columns

Applied at the point of use in `commandWheelsCorner()` / `commandWheelEdge()`
— the underlying hardcoded gain tables (`kCorners[...].Kp`,
`kCandidates[...].K`) are **never modified**. Corner and edge each keep their
own three scales at all times: unlike `c`/`e`/`o`, there is no wrong mode to
refuse — setting `p2.0` while corner is staged just waits there for when you
switch back to it, the same way `o<deg>` already survives a mode switch.

**Every scale resets to `1.0` (the unscaled hardcoded law) on every reflash.**
Values are clamped to `0.0`–`3.0` firmware-side, so a typo can't silently
multiply torque output by an order of magnitude — but the range itself is a
starting sanity bound, not a validated tuning envelope.

### The edge offset is now hardcoded, like corner's

Edge boots with `gEdgePhiOffsetRad` set from `kEdgePhiOffsetDefaultDeg =
-0.7°` — the number that used to get typed by hand with `o-0.7` every
session. `o<deg>` still works exactly as before, live, on top of that
baseline; it is **not** reset by a mode switch, only by a reflash (back to
`-0.7°`) or another explicit `o<deg>`. The dashboard's edge panel has a
**"Reset to default"** button that just sends `o-0.70` — the same command,
not a special one.

Unlike corner's offset, this number was **not** derived from a converged
adaptive-trim hold — there is no live-adapting mechanism for edge yet
(deliberately not built). Re-measure and update the constant by hand if the
mount changes.

A mode-specific letter sent in the wrong mode answers with a one-line refusal
naming the mode it belongs to. It is never silently swallowed — a command that
appears to work and does nothing is worse than a refusal.

`x0` / `x1` switch the **XIAO's** own mode and never reach the Teensy at all;
the bridge consumes them. Nothing in the Teensy grammar uses `x`, deliberately.

---

## The state machine

Three explicit states. `gMode` (EDGE or CORNER) is the *staged* law and is
always defined, including while disarmed; `gState` is what is actually running.

| From | Event | To |
|---|---|---|
| DISARMED | `a1`, corner staged | CORNER_BALANCE |
| DISARMED | `a1`, edge staged, inside the gate | EDGE_BALANCE |
| DISARMED | `a1`, edge staged, outside the gate | DISARMED (refused, says so) |
| any | `a0` | DISARMED |
| any | `m0` / `m1` | **DISARMED** |
| any | `h1` | DISARMED |
| EDGE / CORNER_BALANCE | tilt, overspeed or NaN trip | DISARMED |
| EDGE / CORNER_BALANCE | link quiet for 3 s | DISARMED |

The firmware announces every transition, and answers `s` with the same line:

```
# state=CORNER_BALANCE mode=CORNER armed=1 halted=0
```

That is how the UI knows the mode. It could not be a telemetry column: every
consumer in the tree identifies the format by field count, so widening either
row would break `plot_session_csv.py` and every recording already on disk.

### Mode switching always disarms first

`m0` / `m1` **never** switch while armed. The sequence is always:

```
DISARM  →  SWITCH_MODE  →  ARM
```

even when you only pressed one button. `m<n>` disarms, `SetStop()`s all three
wheels, re-initialises the new law's filters, re-resolves its candidate, and
lands in `DISARMED`. Arming again is a separate, deliberate `a1`. If it had to
force the disarm it says so:

```
# MODE SWITCH: force-disarmed first
```

The dashboard sends `a0` ahead of `m<n>` as well. The two travel as separate
UDP datagrams over a lossy hop, and the disarm is the one that must not be the
one that goes missing.

---

## Watch out: `h` is HALT, not a keepalive

This is the one grammar change that can bite an old script.

| Build | `h` means |
|---|---|
| **CubliBalance** (current) | **HALT** |
| `EdgeBalance_WiFi` (archive) | keepalive no-op |
| `CornerBalance_WiFi` (archive) | keepalive no-op, halt was on `p` |

A bare `h` on the current firmware parses as `atof("") = 0.0` → `h0` → RESUME.
Anything sending `h` ten times a second as a heartbeat therefore holds the HALT
command down in the released position.

**`k` is the only keepalive** and is a no-op on every build in the tree. The
dashboard, `terminal_wifi.py` and both live plotters all send `k`. If you
resurrect an older copy of any of them from the archive, check its heartbeat
before arming.

Likewise `t` is gone: the edge build used `t<0/1>` for link mode, the corner
build used `l<0/1>`, and the fusion kept `l`.

---

## Arm gates differ by mode

| Mode | Gate |
|---|---|
| **CORNER** | ⚠ **NONE** — `a1` arms at any tilt |
| **EDGE** | 9.58° — ⚠ suspected typo, see below |

**Corner will arm with the cube badly tilted.** The control law is linearised
around equilibrium and only validated to roughly ±3° (the simulation study's
recovery envelope tops out at 2.7–3.1° worst case). Place the cube close to
balance before arming — nothing in the firmware will do it for you. The only
automatic backstops are `kMaxTilt` (25°) and the link watchdog, and both stop a
runaway *after* it has started.

The dashboard does not block corner arming either — blocking what the firmware
allows would train you to ignore refusals. It warns past 3° and asks for a
second click.

**The edge gate is 0.1672664619 rad = 9.58°**, while the comment beside it says
0.5° and `teensy/cubli_gains.h` says `0.00872664619` — note the shared
`72664619` tail. It looks exactly like a decimal-point typo. It is carried
forward verbatim from `EdgeBalance_WiFi.ino` **on purpose**, so the fusion stays
a faithful port and the fix can be a separate change with its own bench check.
Until then the dashboard assumes the conservative 0.5° and is the tighter of
the two.

### What "IDLE" means

There is no IDLE mode — **disarmed is idle**. The loop keeps running and
streaming telemetry, but no torque is commanded. `a0` is how you get there.
`h1` goes further: it stops the loop entirely (no IMU reads, no CAN traffic)
and disarms on the way.

---

## Safety chain

```
dashboard sends 'k' every 100 ms
   └─► XIAO relays it to the Teensy
          └─► Teensy disarms 3 s after 'k' stops
```

Close the dashboard, kill the server, lose Wi-Fi, or walk out of range → the
cube disarms. E-STOP is the fast path, not the only one. **Spacebar** does the
same thing from anywhere on the page.

The watchdog is armed whenever the Teensy is in WiFi link mode, even if the
laptop has never spoken — otherwise a USB `a1` with no laptop attached would
leave a law running unattended.

---

## Typical session

Corner:

```
1.  m1             stage corner mode        ← cube already placed on the corner
2.  c              re-resolve the pivot
3.  g0.5           start at half gain
4.  a1             ARM
    ...            observe
5.  a0             DISARM when done
```

Edge:

```
1.  m0             stage edge mode          ← disarms if it was armed
2.  e              re-resolve the edge
3.  o0.2           trim if it drifts one way
4.  g0.5
5.  a1  ...  a0
```

---

## If a command seems ignored

| Cause | Check |
|---|---|
| Wrong mode | `c` is corner-only, `e`/`o` are edge-only — the firmware says so |
| Sketch is halted | Send `h0` |
| Link down | Link pill in the dashboard, or `python tools/link_check.py` |
| Sent `x` something | `x` never reaches the Teensy; the XIAO eats it |
| Sent `t` or `p` | Gone. Use `l` for link mode, `h` for halt |
| Rejected by the dashboard | It validates against the grammar first and logs the refusal in the console pane |

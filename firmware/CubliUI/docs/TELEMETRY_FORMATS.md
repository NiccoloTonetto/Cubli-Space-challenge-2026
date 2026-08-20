# Telemetry formats

The Teensy prints one line per sample. Every tool decides what the data means
by **counting the columns**.

| Columns | Separator | Emitted by | Rate |
|---|---|---|---|
| **10** | comma | `CubliBalance` in **EDGE** mode | ~500 Hz |
| **26** | tab | `CubliBalance` in **CORNER** mode | ~250 Hz |
| 21 | comma | legacy `CornerBalance_WiFi` (archive) | 250 Hz |
| 29 / 33 / 36 | tab | archive AutoTrim / Stage5 variants | 250 Hz |

Both live formats now come from **one sketch**, so **the column count changes
mid-session when you switch mode** (`m0` / `m1`). That is expected, not a
fault. The dashboard logs the change, re-sends the resolve command, and rolls
the CSV to a new file — the two widths never share one.

Rates are adjustable per mode with `d<N>`; the defaults above are each mode's
own (`gDecim = {1, 2}` for `{EDGE, CORNER}` over a 500 Hz control loop).

Lines beginning with `#` are **console text**, not data. Every tool ignores
them for plotting.

---

## 26 columns — CORNER mode

Tab-separated.

| # | Name | Unit |
|---|---|---|
| 0 | `t_ms` | ms since boot |
| 1–3 | `phi_x/y/z` | deg — tilt error |
| 4–6 | `om_x/y/z` | deg/s — body rate |
| 7–9 | `rho_x/y/z` | rad/s — wheel speed |
| 10–12 | `rho_*_lp` | rad/s — 5 s average ("standing speed") |
| 13–15 | `tau_x/y/z` | N·m — commanded torque |
| 16–18 | `tau_cmd_*` | N·m — after limits |
| **19** | `armed` | 0 / 1 |
| 20 | `gain_scale` | 0…1 |
| 21–23 | `trim_*_deg` | deg — **constant** in this build |
| 24 | `trim_com_mm` | mm — derived |
| 25 | `trim_enabled` | always `0` |

Columns 21–25 are fixed values (`kCornerPhiOffset`), kept only so the line
matches the shape older analysis scripts expect. `trim_enabled = 0` signals
"fixed offset, not adapting" to anyone comparing logs across builds.

A bare tab-separated **header row** is printed at boot, on `d<N>`, and when the
laptop first appears. It has the right width and non-numeric cells, so parsers
skip it.

---

## 10 columns — EDGE mode

Comma-separated, **no header row** — the plotters parse every non-`#` line as
data. The column list is emitted as a `# columns: …` comment instead, which
reads fine in a terminal and is invisible to a parser.

| # | Name | Unit |
|---|---|---|
| 0 | `t_ms` | ms |
| 1 | `phi_edge` | deg — tilt error about the edge |
| 2 | `om_edge` | deg/s |
| 3 | `tau` | N·m |
| 4 | `tau_cmd` | N·m |
| **5** | `armed` | 0 / 1 |
| 6 | `gain_scale` | 0…1 |
| 7 | `wheel_omega_lp` | rad/s |
| 8 | `wheel_pos` | rev |
| 9 | `wheel_vel` | rev/s |

---

## `armed` is the truth; `# state=` is the mode

**The `armed` column (19 corner / 5 edge) is authoritative** for arm state. The
control law disarms itself on an overtilt, overspeed or NaN trip, and does not
always get to announce it.

Mode is *not* a column. It arrives on the console line the firmware prints at
boot, on every state transition, and on demand (`s`):

```
# state=CORNER_BALANCE mode=CORNER armed=1 halted=0
```

Adding a `mode` column would have been the obvious alternative and would have
broken every consumer in the tree, all of which key off field count.

---

## What the dashboard does with it

| | Minimal UI (`/`) | Classic UI (`/classic`) |
|---|---|---|
| Detects format | `schemas.py`, by column count | same server, same detection |
| Plots | 4 charts + 3D cube + tiles | 4 charts + 3D cube + tiles |
| Records CSV | ✓ | ✓ |
| Commands the cube's mode | ✓ `m0`/`m1` | ✗ — its EDGE/CORNER buttons only override parsing |

---

## Wheel speed units

Firmware reports **rad/s**. The UI tiles show **RPM**:

```
RPM = rad/s × 60 / (2π) ≈ rad/s × 9.5493
```

---

## Reading a saved CSV

Recordings land in `CubliUI/telemetry/plot/` (dashboard and live plotters) and
`CubliUI/telemetry/serial/` (`terminal_wifi.py` logs).

```powershell
cd "C:\Users\pablo\OneDrive\Documentos\GitHub\Cubli-Space-challenge-2026\firmware\CubliUI\tools\analysis"
python plot_session_csv.py <path-to-csv>
```

Run it with no arguments and press `d` to open the newest recording.

Older logs live in `firmware/3D model/Gam/FINAL/telemetry/` and
`.../PreFINAL/telemetry/`.

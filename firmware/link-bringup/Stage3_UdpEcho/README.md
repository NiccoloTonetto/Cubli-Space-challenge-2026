# Stage 3 — Laptop ↔ XIAO over UDP, both directions, under load

**Power:** cube power **OFF**, XIAO USB only. Still no Teensy, still no wires.
**Can anything move?** No.

## Proves

This is the stage that decides **whether WiFi is your problem at all**. It
separates the two directions, which a silent telemetry window never can:

| Direction | Evidence |
|---|---|
| laptop → XIAO | the board's own `udp_in` counter, printed over its USB. Nothing on the network can fake this number. |
| XIAO → laptop | `ECHO:` replies and unprompted `BEAT:` packets arriving at the script. |

It also **loads the link** at 250 packets/s. A link that answers one ping
politely and collapses under load passes every casual test and still gives you
a dead plot window at Stage 4.

## Run

Two windows, side by side — you need both:

1. Flash `xiao_udp_echo/xiao_udp_echo.ino` (same five config values as Stage
   2). Serial Monitor at 115200 → this shows `udp_in`.
2. `python stage3_udp_echo.py`

```bash
python stage3_udp_echo.py
python stage3_udp_echo.py --ip 192.168.1.42
python stage3_udp_echo.py --rate 500 --seconds 20    # edge build's rate
```

No dependencies beyond the standard library.

## Pass

```
1. Round trip:  rtt min=2.1 med=4.5 p95=9.0 max=22 ms, loss 0/50
2. BEAT:        6 packets (expected about 6)
3. Load:        sent 2500 (250/s), echoed back 2490 (249/s)
```

Median RTT under ~50 ms, loss under 2 %, and ≥ 240 round trips/s.

## If it fails — read both windows together

| Script says | XIAO's `udp_in` | Verdict |
|---|---|---|
| nothing back | `0/s` | Your packets never arrive. **Client isolation** on the AP, wrong `XIAO_IP`, or the laptop is on a different network. No firmware change fixes isolation — use a phone hotspot. |
| nothing back | climbing | Laptop-side inbound block: **Windows Firewall** on `python.exe`, or a VPN capturing the route. |
| echoes trickle back ~2 s apart | climbing | The XIAO's loop is stalled — almost always its USB cable attached with **nothing reading the port**. See [the CDC stall](../README.md#the-cdc-stall-the-one-that-wastes-the-most-time). |
| min RTT single-digit, median ~75 ms, ceiling ~100 ms | climbing | **WiFi modem sleep** — not the network. `WiFi.setSleep(false)`, already applied here and in the bridge; reflash the XIAO. [Full explanation](../README.md#wifi-modem-sleep-found-at-stage-3-2026-08-17). |
| echoes fine, load test poor | climbing | Weak RF. Check RSSI on the `[status]` line; move the AP closer. |
| echoes fine, load test fine | climbing | **Done.** Everything from here on is Teensy/Serial1, not WiFi. |

### Windows Firewall

Windows prompts the first time this script binds. Allow it on **Private**
networks. A phone hotspot is often classified **Public** — if you clicked the
wrong box once, no prompt ever appears again and everything silently fails.
Fix at *Windows Security → Firewall & network protection → Allow an app*.

→ Then [Stage 4](../Stage4_BridgeThroughput/).

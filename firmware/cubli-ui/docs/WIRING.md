# Wiring and hardware setup

Two boards, one cable pair, one ground. Edge and corner balance are the same
firmware on the same hardware — nothing here changes when you switch mode.

---

## Teensy 4.1 ↔ XIAO ESP32-C6

| Teensy 4.1 | direction | XIAO ESP32-C6 |
|---|---|---|
| **TX1** — pin 1 | → | **D7** (RX) |
| **RX1** — pin 0 | ← | **D6** (TX) |
| **GND** | — | **GND** |

**Crossed, not straight.** TX goes to RX on the other board.

A common ground is mandatory. Without it the link looks dead or produces garbage characters.

```
   TEENSY 4.1                      XIAO ESP32-C6
   ┌──────────┐                    ┌──────────┐
   │  pin 1 TX├───────────────────►┤D7 RX     │
   │  pin 0 RX│◄───────────────────┤D6 TX     │
   │      GND ├────────────────────┤ GND      │
   └──────────┘                    └──────────┘
```

---

## Link speed

```
1 000 000 baud   (1 Mbaud)
```

This value appears in **two** places and they must match:

| File | Constant |
|---|---|
| `xiao/xiao_teensy_bridge/xiao_teensy_bridge.ino` | `TEENSY_LINK_BAUD` |
| `teensy/CubliBalance/CubliBalance.ino` | `kLinkBaud` |

**A mismatch is silent.** The symptom looks exactly like a dead cable: no telemetry, a dead link pill.

### Why not 115200?

Corner mode sends ~200 bytes per line at 250 Hz ≈ 400 kbit/s. That does not fit in 115200. Writes would block inside the 2 ms control loop and the balance would fail.

The real ceiling is not the UART, though — it is the UDP hop, measured at ~105 lines/s in the link bring-up. Use `d<N>` if you need a readable rate rather than a captured one.

---

## Motors (moteus over CAN)

| Controller ID | Body axis |
|---|---|
| 2 | X |
| 3 | Y |
| 1 | Z |

CAN bus: **CAN3**, FD, 1 Mbit.

**Corner mode drives all three. Edge mode drives one** — `gEdgeAxis`, which
defaults to **Y (id 3)**. On every mode entry the firmware `SetStop()`s all
three, so in edge mode the other two are explicitly stopped rather than left to
time out on their own watchdogs.

---

## IMU

BMI270 over SPI, chip select on **pin 10**, 4 MHz.

---

## Power

- Battery: **6S** pack (25.2 V full, 22.2 V nominal, **18.0 V floor**)
- Below 18 V, stop and recharge.
- The XIAO can run from USB during bench work, but for untethered runs it must be powered from the cube.

---

## Wi-Fi

| Setting | Default |
|---|---|
| SSID | `cubli1` |
| XIAO IP | `172.20.10.14` |
| Gateway | `172.20.10.1` |
| Subnet | `255.255.255.240` |

These match an **iPhone hotspot**, which hands out a `/28` (`172.20.10.1` … `.14`).

If you use a different network, edit the IP block in the XIAO sketch, or switch it to DHCP and read the address from the USB serial monitor at boot.

---

## Before every session

- [ ] TX/RX crossed, GND shared
- [ ] Battery above 18 V
- [ ] Hotspot on, XIAO joined
- [ ] Clear space around the cube
- [ ] E-STOP reachable (browser open, or spacebar)
- [ ] Cube on a flat, stable surface
- [ ] **Right mode staged** (`m0` edge / `m1` corner) and the pivot re-resolved
- [ ] **Cube placed near equilibrium** — corner mode will arm at any tilt

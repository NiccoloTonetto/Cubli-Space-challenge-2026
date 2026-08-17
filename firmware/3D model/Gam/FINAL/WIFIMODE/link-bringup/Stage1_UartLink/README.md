# Stage 1 — The Teensy ↔ XIAO wire, at 1 Mbaud, no WiFi

**Power:** cube power **OFF**. Both boards on their own USB cable, both into
**the same laptop**.
**Can anything move?** No. No CAN traffic exists in either sketch.

> **Two cables here are for the two Serial Monitors, not for power** — one
> cable already powers both boards through the commoned 5 V node. Plugging
> both therefore ties two USB ports' VBUS together: tolerated on one laptop
> (one internal 5 V rail, common ground), **never across two hosts** — that
> gives you a ground offset across your jumpers as well as two fighting
> supplies. See [the 5 V note](../README.md#the-5-v-pins-are-commoned--what-that-changes).

## Proves

- The three jumpers are right — **TX/RX crossed**, common GND.
- 1 Mbaud is clean over your actual wire.
- **Each direction, separately.** This is the whole reason the stage exists:
  from the laptop, a broken outbound wire and a broken return wire look
  identical, and you can burn an afternoon on the wrong one.

## Wiring

```
Teensy Serial1 TX1 (pin 1)  ---->  XIAO D7 (RX)
Teensy Serial1 RX1 (pin 0)  <----  XIAO D6 (TX)
Teensy GND                  -----  XIAO GND      <- not optional
```

Both boards are 3.3 V logic — direct connection, no level shifter. **Never put
5 V on D6/D7.** All power off before touching a jumper: hot-plugging a signal
wire back-feeds an unpowered board through its clamp diodes. GND connects
first and disconnects last.

## Run

1. Flash `xiao_uart_echo/` **first** (so nothing transmits into a board that
   is mid-upload), then `teensy_uart_ping/`.
2. Open **both** Serial Monitors at 115200 and watch them side by side.

The Teensy sends `PING <n>` every 100 ms; the XIAO answers `ACK <n>`.

## Pass

```
Teensy:  [link] sent=10 acks=10 loss=0.0% rtt_avg=0.4ms garbage=0
XIAO:    [uart] lines=10 pings=10 acks_sent=10 garbage=0 last="PING 41"
```

**Both** sides must agree. One side alone only proves one direction.

## If it fails

| Teensy monitor | XIAO monitor | Broken thing |
|---|---|---|
| `acks=0` | no pings | Teensy pin 1 → XIAO D7, **or** no common GND, or a board is not running (redo Stage 0) |
| `acks=0` | pings arriving | **only the return wire**: XIAO D6 → Teensy pin 0. This is what a swapped pair usually looks like |
| `garbage>0` | mojibake | baud mismatch, or jumpers too long/noisy |
| both silent | both silent | GND, or both boards are simply not running |

`garbage` that only clears at a lower baud means bad wiring — **shorten the
jumpers, do not lower the baud.** The corner build's telemetry budget assumes
1 Mbaud (21 fields × 250 Hz ≈ 42 % utilisation); dropping to 115200 makes it
mathematically impossible.

→ Then [Stage 2](../Stage2_WiFiJoin/).

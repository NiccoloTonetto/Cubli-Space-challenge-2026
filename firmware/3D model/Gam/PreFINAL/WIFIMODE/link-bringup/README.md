# link-bringup — find out exactly where the WiFi link is broken

Six stages, each one hop longer than the last. Every stage isolates **one**
thing and gives a number that either passes or names the fault. Run them in
order. The first one that fails is your answer — and you never have to guess
between "the WiFi is broken" and "the wire is broken" again.

This exists because the failure you are chasing —  *no telemetry arrives* —
has at least nine distinct causes spread across three boards, two power
schemes and a firewall, and from the plot window they all look identical.

> **Nothing in Stages 0–5 can move a wheel.** No sketch in this folder opens
> CAN. Stage 6 flashes the real firmware and still keeps the motor bus dead.

## The ladder

| Stage | Powered by | Boards | Proves | Pass |
|---|---|---|---|---|
| [0 — Boards alive](Stage0_BoardsAlive/) | USB | either, alone | toolchain, COM ports, **USB CDC On Boot** | `[alive]` once a second on both |
| [1 — UART link](Stage1_UartLink/) | USB ×2 | both | jumpers crossed, common GND, 1 Mbaud clean, **each direction separately** | `acks=10 loss=0.0% garbage=0` on both monitors |
| [2 — WiFi join](Stage2_WiFiJoin/) | USB | XIAO | SSID, password, static IP on the right subnet | `[wifi] up` for 60 s with no drop |
| [3 — UDP echo](Stage3_UdpEcho/) | USB | XIAO | laptop↔XIAO **both directions**, under 250/s load | RTT < 50 ms, loss < 2 %, ≥ 240 round trips/s |
| [4 — Bridge throughput](Stage4_BridgeThroughput/) | USB ×2 | both | the **whole pipeline** at 250 lines/s, real grammar, no control law | `rate 249/s loss < 1%` |
| [5 — Rail power](Stage5_RailPower/) | **bench supply** | both | the LM2596 rail carries it, no USB anywhere | same numbers as Stage 4, held 2 min |
| [6 — Real firmware](Stage6_RealFirmware/) | bench supply | both | the actual build, **motor bus dead** | boot diagnostics + live telemetry + commands answered |

Stages 0–5 are all bench work with the cube power off or the motors dead.
Arming is deliberately outside this folder — see
[`../README.md`](../README.md) § *Bring-up sequence*.

## Start here if you are already stuck

You do not have to start at Stage 0 if you have a specific symptom:

| What you see | Go to |
|---|---|
| Nothing at all, no idea where to begin | [Stage 0](Stage0_BoardsAlive/) — 5 minutes, and it rules out two whole categories |
| XIAO's COM port shows no text | [Stage 0](Stage0_BoardsAlive/) — *USB CDC On Boot* |
| XIAO says `serial1_lines=0` | [Stage 1](Stage1_UartLink/) — it is the wire or GND |
| XIAO says `wifi=DOWN`, or never connects | [Stage 2](Stage2_WiFiJoin/) — the scan output names the reason |
| `wifi=up` but nothing reaches the laptop | [Stage 3](Stage3_UdpEcho/) — separates firewall from client isolation |
| Telemetry arrives but is choppy / slow / dies | [Stage 4](Stage4_BridgeThroughput/) — measures exact loss with no control law involved |
| It worked, then the XIAO **vanished** — no ping, no ARP entry | Power, not network. The board browned out or reset: see [the 5 V note](#the-5-v-pins-are-commoned--what-that-changes). Read its Serial Monitor to tell a reboot from a WiFi drop. |
| Works on USB power, not on the supply | [Stage 5](Stage5_RailPower/) — it is the rail, not the network |
| Link fine, firmware misbehaves | [Stage 6](Stage6_RealFirmware/) |

Already-flashed bridge and just want a verdict?
[`../link_check.py`](../link_check.py) walks the whole pipeline in one command.
This folder is what you use when `link_check.py` says "broken" and you need to
know **which part**.

## Power rules (read once, then never improvise)

**Never have cube power and USB connected at the same time.**

Both `Teensy VIN` and the XIAO's `5V` pin are tied to their board's USB 5 V
internally. With the rail live, plugging in USB puts the **laptop's 5 V
directly in parallel with the LM2596's 5 V** — two stiff sources, neither
current-limited toward the other, whichever is higher back-feeding the other.
That can damage the laptop's USB port, the LM2596, or a board.

On this build the supply feeds **both** boards and you cannot power one
without the other. **This ladder is arranged so that never matters:**

| | Rail | XIAO USB | Teensy USB |
|---|---|---|---|
| Stages 0–4 | **off** | yes | yes |
| Stage 5–6 | **on** | **no** | **no** |

Stages 0–4 run on USB power with the rail dead. Stage 5–6 run on the rail with
no cable anywhere. You never need both at once.

### The 5 V pins are commoned — what that changes

On this build the XIAO's `5V` pin and the Teensy's `VIN` are the **same node**,
both fed from the LM2596. With the rail off and one USB cable plugged in, that
cable powers **both boards** — plus anything else on the 5 V rail.

That is electrically fine (one source, and back-feeding the LM2596's *output*
has no path to its input through a non-synchronous buck), but two things
follow:

- **"One board at a time" is not achievable.** Plugging in the XIAO boots the
  Teensy too. Stage 0 and Stage 1 are written accordingly.
- **All of it flows through one USB connector.** The Teensy's ~100 mA idle
  travels through the XIAO's USB port, its 5 V trace and its `5V` pin. Against
  a 500 mA USB 2.0 port:

  | | |
  |---|---|
  | XIAO continuous (with `setSleep(false)`) | ~100 mA |
  | XIAO WiFi TX peaks | up to 300 mA |
  | Teensy 4.1 idle | ~100 mA |
  | **peak through one port** | **~400 mA+** |

  Use a **USB 3.0 port (900 mA) or a powered hub** for the XIAO. A port that
  current-limits browns out the XIAO, which drops off WiFi and vanishes from
  the ARP table — a failure that looks like a network fault and is not one.
  Symptom seen on this hardware: Stage 3 passing twice, then the board
  disappearing entirely between runs.

Plugging **both** USB cables ties two ports' VBUS together. On the same
laptop that is tolerated — one internal 5 V rail, common ground. **Never
across two different hosts.**

The permanent fix is the same jumper from
[Stage 5](Stage5_RailPower/#if-you-want-usb-and-rail-power-anyway): put a
2-pin header in the XIAO's 5 V feed. Then USB powers only the XIAO, the rail
powers only what you want, and the whole class of problem goes away.

**Order, every time:** power up → cube power **OFF**, plug USB, flash, **unplug
USB**, cube power **ON**. Power down → cube power **OFF** first, then USB.
Wiring → all power off before touching D6/D7/GND; GND connects first and
disconnects last.

If you do want both simultaneously — mainly so you can run
`link_check.py --serial`, the only conclusive directional test — the options
and their trade-offs are in [Stage 5](Stage5_RailPower/#if-you-want-usb-and-rail-power-anyway).
The short version: **put a jumper in the XIAO's 5 V feed.** Ten minutes, and
the constraint goes away permanently.

## The CDC stall (the one that wastes the most time)

Leaving the XIAO's USB cable attached with **no application reading the port**
collapses the bridge. Measured on this hardware, not theorised:

```
loop() rate, cable attached, monitor closed:   0.49 iterations/s
```

`pollUdpIn()` and `pollSerial1Out()` each handle exactly **one item per
`loop()` iteration**, so the bridge then relays **0.5 lines/s** — against the
250/s the corner build emits. Telemetry appears completely dead, for a reason
that looks nothing like the actual cause.

Why: on this chip `if (Serial)` is true whenever the cable is plugged in, even
with nothing draining the buffer, and `printf()` then blocks ~2 s. The 1 s gate
on the status print has always expired by the time it returns, so it re-blocks
every iteration.

**Three ways to be safe, in order:**

1. **Unplug the XIAO's USB and power it from the 5 V rail** — the normal
   operating configuration anyway.
2. Keep a serial monitor **open and reading** — that drains the buffer. This
   is what Stages 0–4 assume, and why they tell you to keep the monitors open.
3. Set `ENABLE_LINK_STATUS 0` in the bridge sketch if the cable must stay
   attached unattended.

Every XIAO sketch in this folder carries the same `availableForWrite()` guard
the bridge does, so it skips its print instead of stalling. Belt and braces —
rule 1 is still the one to follow.

**Signature:** Stage 3 echoes trickling back ~2 s apart, or Stage 4 reporting a
rate near 0.5/s.

## WiFi modem sleep (found at Stage 3, 2026-08-17)

The esp32 core defaults a station to `WIFI_PS_MIN_MODEM`: the radio parks
between the AP's beacons and only wakes to collect buffered downlink traffic.
Measured here at **−32 dBm**, which is as strong as a link gets:

```
rtt  min=3.9   med=75.5   p95=93.5   max=93.5 ms
```

The 3.9 ms is what the RF path actually costs. The rest is sleep, and the
~100 ms ceiling is the iPhone hotspot's beacon interval. The tell is the
**shape**: a fast best case with a median an order of magnitude worse and a
hard ceiling. Weak signal and congestion both look nothing like this.

It matters more than the latency number suggests:

- every command, including `a0`, is delayed by up to a beacon interval;
- the PC keepalive arrives in bursts, so the Teensy's **300 ms link watchdog
  can time out and auto-disarm mid-balance** for no real reason — a phantom
  fault indistinguishable from a genuine link failure;
- packets buffered for a sleeping station are dropped once the AP's queue for
  it fills, which is routine at 250 lines/s.

**Fix:** `WiFi.setSleep(false)` — now in
[`../xiao_teensy_bridge/`](../xiao_teensy_bridge/xiao_teensy_bridge.ino),
[`firmware/XIAO/xiao_teensy_bridge/`](../../../../../XIAO/xiao_teensy_bridge/xiao_teensy_bridge.ino)
and both XIAO sketches in this ladder. Costs ~100 mA continuous, already
inside the ESP32-C6 budget in
[`Electrical-Design-Guide.md`](../../../../../../docs/electronics/Electrical-Design-Guide.md)
§2.2. **Reflash the XIAO to pick it up.**

`stage3_udp_echo.py` now names this pattern when it sees it, and separates
**late** replies from **lost** ones — with modem sleep on, replies routinely
arrive after the next ping goes out, which an earlier version of the script
scored as 13 % loss when almost none of it was loss.

## Keep these five values identical everywhere

Every XIAO sketch in this ladder and
[`../xiao_teensy_bridge/xiao_teensy_bridge.ino`](../xiao_teensy_bridge/xiao_teensy_bridge.ino)
must agree on:

```cpp
WIFI_SSID    WIFI_PASSWORD    XIAO_IP    XIAO_GATEWAY    XIAO_SUBNET
```

and the Python scripts' `XIAO_IP` must match too (or pass `--ip`). A test
sketch that passes with different settings than the bridge that "doesn't work"
is a trap this project has already fallen into.

Two more that must match across boards: **1000000 baud** (`kLinkBaud` on the
Teensy, `TEENSY_LINK_BAUD` on the XIAO) and **UDP port 4210**.

### Network choice

Use a **phone hotspot or a network you control**. Guest/campus networks
(`ES-Guest` here) run **client isolation**: the laptop and the XIAO both reach
the internet and never each other. Stage 2 passes perfectly, Stage 3 can never
pass, and no firmware change fixes it.

Keep the SSID **plain ASCII**. The default iPhone name contains U+2019, a
curly apostrophe — typing a normal `'` never matches and the join fails
silently. This repo uses `cubli1`.

## Wiring, once, for all stages

```
Teensy Serial1 TX1 (pin 1)  ---->  XIAO D7 (RX)
Teensy Serial1 RX1 (pin 0)  <----  XIAO D6 (TX)
Teensy GND                  -----  XIAO GND      <- not optional
```

Crossed, and 3.3 V logic on both sides — no level shifter, and never 5 V on
D6/D7.

## What is in here

```
Stage0_BoardsAlive/     xiao_hello/          teensy_hello/
Stage1_UartLink/        xiao_uart_echo/      teensy_uart_ping/
Stage2_WiFiJoin/        xiao_wifi_join/
Stage3_UdpEcho/         xiao_udp_echo/       stage3_udp_echo.py
Stage4_BridgeThroughput/ teensy_csv_faker/   stage4_rate_check.py
Stage5_RailPower/       (README only — reuses Stage 4)
Stage6_RealFirmware/    (README only — flashes ../CornerBalance_WiFi/)
```

Python needs only the standard library. `matplotlib` is required by the real
telemetry scripts one level up, not by anything here.

Nothing in this folder modifies the flight builds. The bridge sketch used from
Stage 4 on is the existing
[`../xiao_teensy_bridge/`](../xiao_teensy_bridge/xiao_teensy_bridge.ino),
unchanged.

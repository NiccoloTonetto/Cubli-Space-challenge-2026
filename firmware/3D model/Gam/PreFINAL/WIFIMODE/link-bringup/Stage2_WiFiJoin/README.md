# Stage 2 — The XIAO joins the network

**Power:** cube power **OFF**, XIAO USB only. The Teensy is irrelevant here.
**Can anything move?** No.

## Proves

The board associates with your AP and holds the **static address** you gave
it. Nothing more — reachability is Stage 3's job, and the two failures look
identical from the laptop.

## Run

1. In `xiao_wifi_join/xiao_wifi_join.ino` set `WIFI_SSID`, `WIFI_PASSWORD`,
   `XIAO_IP` / `XIAO_GATEWAY` / `XIAO_SUBNET`.
2. Upload. Serial Monitor at 115200.

> **Keep those five values byte-identical** here, in Stage 3's sketch, and in
> [`xiao_teensy_bridge.ino`](../../xiao_teensy_bridge/xiao_teensy_bridge.ino).
> A test sketch that passes with different settings than the bridge that
> "doesn't work" is a trap this project has already fallen into once.

## Pass

```
 connected!
  IP:      172.20.10.14
  gateway: 172.20.10.1
  RSSI:    -52 dBm
[wifi] up ip=172.20.10.14 rssi=-52dBm uptime=12s
```

The `[wifi] up` line must keep printing for **60 s without a single `link
DROPPED`**. A join that flaps every few seconds shows up three stages later as
"random telemetry dropouts" and is very hard to recognise there.

Then, from the laptop: `ping 172.20.10.14`. A reply is good news but **not**
proof — some APs pass ICMP and still block client-to-client UDP. Stage 3
decides.

## If it fails

The sketch scans on failure and lists every SSID it can hear, with RSSI and a
flag on any name containing **non-ASCII characters**.

| Reported status | Meaning |
|---|---|
| `NO_SSID_AVAIL` + your SSID absent from the scan | Name mismatch, out of range, or the AP is **5 GHz-only** (this radio is 2.4 GHz here and will never see it) |
| `NO_SSID_AVAIL` + your SSID **present** in the scan | The name you typed is not the name that is broadcast — compare character by character |
| `CONNECT_FAILED` | Wrong password, WPA2-Enterprise (unsupported), or a captive portal |
| Connected, but IP ≠ `XIAO_IP` | `WiFi.config()` was overridden. Use the address it actually printed |
| `!! XIAO_IP and XIAO_GATEWAY are NOT in the same subnet` | Fix the addresses. This produces a board that reports healthy and is unreachable — the most confusing failure in the ladder |

### The curly-apostrophe trap

The default iPhone hotspot name contains **U+2019**, a curly apostrophe that
is three UTF-8 bytes. Typing a normal `'` in `WIFI_SSID` never matches and the
join fails with no useful error. Rename the phone to plain ASCII — this repo
uses `cubli1`. The scan output flags any SSID with non-ASCII bytes for exactly
this reason.

### Do not use a guest / campus network

`ES-Guest` was abandoned for this project: it runs **client isolation**, so
the laptop and the XIAO both reach the internet and never each other. Stage 2
passes perfectly and Stage 3 can never pass. No firmware change fixes it.

→ Then [Stage 3](../Stage3_UdpEcho/).

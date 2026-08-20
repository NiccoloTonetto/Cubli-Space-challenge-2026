# Stage 0 — Both boards alive over USB

**Power:** cube power **OFF**, one USB cable at a time.
**Wires between the boards:** none needed. Leave them connected or not.
**Can anything move?** No. Nothing here touches CAN.

> **Both boards will power up from one cable.** The XIAO's `5V` pin and the
> Teensy's `VIN` are the same node on this build, so whichever board you plug
> in also feeds the other. That is fine with the rail off — but it means you
> cannot test one board in isolation, and all the current goes through one USB
> connector. Prefer a **USB 3.0 port or a powered hub**; see
> [the 5 V note](../README.md#the-5-v-pins-are-commoned--what-that-changes).

## Proves

- The toolchains are installed and the right board is selected.
- Each board enumerates, and **you know which COM port is which**.
- On the XIAO: *Tools → USB CDC On Boot* is **Enabled**. With it disabled you
  get a COM port that never emits a single character — the most common wasted
  hour on this board.

## Run

| | |
|---|---|
| 1 | Open `xiao_hello/xiao_hello.ino`, Board = **XIAO_ESP32C6**, upload, Serial Monitor **115200**. |
| 2 | Write down the **MAC address** it prints. Stage 2/3 use it to find the board in the hotspot's client list and in `arp -a`. |
| 3 | Open `teensy_hello/teensy_hello.ino`, Board = **Teensy 4.1**, upload, Serial Monitor. |
| 4 | Note both COM port numbers. Keep them somewhere; every later stage asks. |

## Pass

Both boards print `[alive] seq=…` once a second. XIAO LED and Teensy pin-13
LED blink.

## If it fails

| Symptom | Cause |
|---|---|
| XIAO: COM port exists, no text ever | *USB CDC On Boot* is disabled. Enable it and reflash. |
| XIAO: no COM port at all | Cable is charge-only — a very common bad cable. Try another. Or enter bootloader: hold **BOOT**, tap **RESET**, release BOOT. |
| XIAO: upload fails midway | Same bootloader trick, then upload while it sits in bootloader mode. |
| Teensy: no port | Press the physical program button on the board during upload. |
| Teensy: port but no text | Wrong board selected in *Tools → Board*. |
| Either board: nothing at all, ever | Check it is not being powered from the LM2596 rail at the same time — see [the power rules](../README.md#power-rules-read-once-then-never-improvise). |

→ Then [Stage 1](../Stage1_UartLink/).

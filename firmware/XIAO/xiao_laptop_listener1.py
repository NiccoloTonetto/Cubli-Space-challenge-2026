#!/usr/bin/env python3
"""
xiao_laptop_listener.py
------------------------
Companion script for xiao_esp32c6_wifi_test.ino.

Run this on your laptop BEFORE powering on / resetting the XIAO ESP32C6.
It does two things:
  1. Responds to "PING:<id>" packets with "PONG:<id>" (used by the board's
     startup latency calibration).
  2. Prints incoming telemetry JSON packets as they arrive, with a running
     packet count and a rate estimate, so you can visually confirm the
     WiFi link is stable.

No third-party dependencies — just the standard library.

Usage:
    python3 xiao_laptop_listener.py
    (Ctrl+C to stop)
"""

import socket
import json
import time

UDP_PORT = 4210
BUFFER_SIZE = 1024

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)

    print(f"Listening for the XIAO ESP32C6 on UDP port {UDP_PORT}...")
    print("(Leave this running, then power on / reset the board.)\n")

    telemetry_count = 0
    last_seq = None
    dropped = 0
    start_time = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue

            text = data.decode(errors="replace").strip()

            if text.startswith("PING:"):
                ping_id = text.split(":", 1)[1]
                reply = f"PONG:{ping_id}".encode()
                sock.sendto(reply, addr)
                print(f"[calibration] ping {ping_id} from {addr[0]} -> pong sent")
                continue

            # Otherwise, expect telemetry JSON
            try:
                msg = json.loads(text)
            except json.JSONDecodeError:
                print(f"[unrecognized] {addr[0]}: {text}")
                continue

            if msg.get("type") == "telemetry":
                telemetry_count += 1
                seq = msg.get("seq")
                if last_seq is not None and seq is not None and seq != last_seq + 1:
                    dropped += max(0, seq - last_seq - 1)
                last_seq = seq

                elapsed = time.time() - start_time
                rate = telemetry_count / elapsed if elapsed > 0 else 0

                print(
                    f"[telemetry] seq={seq:<5} "
                    f"uptime={msg.get('uptime_ms'):>8}ms "
                    f"rssi={msg.get('rssi_dbm'):>4}dBm "
                    f"heap={msg.get('free_heap'):>7}B "
                    f"| received={telemetry_count} dropped~{dropped} "
                    f"rate={rate:.1f}/s"
                )

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()

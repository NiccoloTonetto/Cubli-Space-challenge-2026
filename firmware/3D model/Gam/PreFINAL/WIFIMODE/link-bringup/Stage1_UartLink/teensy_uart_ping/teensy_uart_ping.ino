/*
  Stage 1a -- Teensy side of the Serial1 <-> XIAO wire test.  NO WiFi.
  ---------------------------------------------------------------------------
  Pair with Stage1_UartLink/xiao_uart_echo/xiao_uart_echo.ino. Both boards
  powered by their OWN USB cable, cube power OFF, both Serial Monitors open.

  What this proves, and nothing else:
    * the three jumpers are right (TX/RX CROSSED, common GND)
    * 1 Mbaud is clean over your actual wire length
    * each DIRECTION works, tested independently -- which is the whole point.
      A silent bridge later is almost always one of these two wires, and from
      the laptop the two failures look identical.

  Wiring -- unplug everything before touching it:

      Teensy Serial1 TX1 (pin 1)  ---->  XIAO D7 (RX)
      Teensy Serial1 RX1 (pin 0)  <----  XIAO D6 (TX)
      Teensy GND                  -----  XIAO GND      (NOT optional)

  Both boards are 3.3 V logic -- direct connection, no level shifter. Never
  put 5 V on D6/D7.

  PROTOCOL: this board sends "PING <n>" every 100 ms and expects "ACK <n>"
  back. Once a second it prints a summary to USB:

      [link] sent=10 acks=10 loss=0.0% rtt_avg=0.4ms garbage=0

  READING THE RESULT (this is the diagnosis, do not skip it):

    acks == sent, garbage == 0
        Both directions work. Go to Stage 2.

    acks == 0, and the XIAO's monitor shows NO pings either
        Teensy TX1 (pin 1) -> XIAO D7 is broken, or there is no common GND,
        or one board is not actually running (redo Stage 0).

    acks == 0, but the XIAO's monitor DOES show the pings arriving
        Only the return wire is broken: XIAO D6 -> Teensy RX1 (pin 0).
        This is the half that a swapped TX/RX pair produces most often.

    garbage > 0, or the XIAO prints mojibake
        Baud mismatch or a bad wire. kLinkBaud here must equal the XIAO's
        Serial1.begin() rate, and both must equal kLinkBaud in
        CornerBalance_WiFi.ino (1000000). If it only cleans up at a lower
        baud, your jumpers are too long/unshielded -- shorten them; do NOT
        "fix" it by lowering the baud, the corner build's telemetry budget
        assumes 1 Mbaud (see ../../README.md).
*/

static const uint32_t kLinkBaud   = 1000000;   // MUST match the XIAO sketch
static const uint32_t kPingPeriod = 100;       // ms

static uint32_t gNextPingMs   = 0;
static uint32_t gLastReportMs = 0;
static uint32_t gSeq          = 0;

static uint32_t gSent = 0, gAcks = 0, gGarbage = 0;
static uint32_t gRttSumUs = 0;
static uint32_t gPingSentUs = 0;
static int32_t  gLastAckSeq = -1;

// Non-blocking line accumulator -- Serial1.readStringUntil() would block up to
// its timeout, which is the exact behaviour the real WiFi build's LineReader
// exists to avoid. Same shape here so the test matches the real link layer.
static char gBuf[128];
static uint8_t gLen = 0;

void handleLine(const char* line) {
  if (strncmp(line, "ACK ", 4) == 0) {
    const long n = atol(line + 4);
    gAcks++;
    gLastAckSeq = (int32_t)n;
    gRttSumUs += (uint32_t)(micros() - gPingSentUs);
    Serial.printf("  <- %s\n", line);
    return;
  }
  // Anything that is not an ACK: show it, and show it as escaped bytes so a
  // baud mismatch reads as \xNN garbage instead of invisible whitespace.
  gGarbage++;
  Serial.print("  <- [unexpected] ");
  for (const char* p = line; *p; ++p) {
    if (*p >= 32 && *p < 127) { Serial.print(*p); }
    else                      { Serial.printf("\\x%02X", (uint8_t)*p); }
  }
  Serial.println();
}

void pollLink() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (gLen == 0) { continue; }
      gBuf[gLen] = '\0';
      handleLine(gBuf);
      gLen = 0;
      continue;
    }
    if (gLen < sizeof(gBuf) - 1) { gBuf[gLen++] = c; }
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 2000) { delay(10); }

  Serial1.begin(kLinkBaud);   // Teensy 4.1 Serial1: RX1 = pin 0, TX1 = pin 1

  Serial.println();
  Serial.println("=== Stage 1a: Teensy -> XIAO UART ping ===");
  Serial.printf("  Serial1 @ %lu baud, RX1=pin0, TX1=pin1\n",
                (unsigned long)kLinkBaud);
  Serial.println("  Sending \"PING <n>\" every 100 ms, expecting \"ACK <n>\".");
  Serial.println();
}

void loop() {
  const uint32_t now = millis();

  if ((int32_t)(now - gNextPingMs) >= 0) {
    gNextPingMs = now + kPingPeriod;
    Serial1.printf("PING %lu\n", (unsigned long)gSeq);
    gPingSentUs = micros();
    gSeq++;
    gSent++;
  }

  pollLink();

  if (now - gLastReportMs >= 1000) {
    gLastReportMs = now;
    const float loss = gSent ? 100.0f * (float)(gSent - gAcks) / (float)gSent : 0.0f;
    const float rtt  = gAcks ? (float)gRttSumUs / (float)gAcks / 1000.0f : 0.0f;
    Serial.printf("[link] sent=%lu acks=%lu loss=%.1f%% rtt_avg=%.2fms "
                  "garbage=%lu last_ack=%ld\n",
                  (unsigned long)gSent, (unsigned long)gAcks, loss, rtt,
                  (unsigned long)gGarbage, (long)gLastAckSeq);
    gSent = gAcks = gGarbage = 0;
    gRttSumUs = 0;
  }
}

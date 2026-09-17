/*
  Stage 1b -- XIAO side of the Serial1 <-> Teensy wire test.  NO WiFi.
  ---------------------------------------------------------------------------
  Pair with Stage1_UartLink/teensy_uart_ping/teensy_uart_ping.ino. Flash this
  one FIRST, then the Teensy, so nothing is transmitting into a board that is
  still being programmed.

  Both boards on their OWN USB cable, cube power OFF, both Serial Monitors
  open at 115200 (the XIAO's monitor MUST be open -- see the note on
  Serial.availableForWrite() below).

  Wiring, identical to the real bridge:

      Teensy Serial1 TX1 (pin 1)  ---->  XIAO D7 (RX)
      Teensy Serial1 RX1 (pin 0)  <----  XIAO D6 (TX)
      Teensy GND                  -----  XIAO GND

  This board answers every "PING <n>" with "ACK <n>" on the same wire and
  prints a summary once a second:

      [uart] lines=10 pings=10 acks_sent=10 garbage=0 last="PING 41"

  PASS: pings == acks_sent == 10/s, garbage == 0, AND the Teensy's own
  monitor reports acks=10 loss=0.0%. Both sides must agree -- one side alone
  only proves one direction. The full diagnosis table is in
  ../teensy_uart_ping/teensy_uart_ping.ino's header.
*/

static const uint32_t kLinkBaud = 1000000;   // MUST match the Teensy sketch
static const int8_t   kRxPin    = D7;        // <- Teensy TX1 (pin 1)
static const int8_t   kTxPin    = D6;        // -> Teensy RX1 (pin 0)

static uint32_t gLastReportMs = 0;
static uint32_t gLines = 0, gPings = 0, gAcks = 0, gGarbage = 0;

static char gBuf[128];
static uint8_t gLen = 0;
static char gLast[128] = "(nothing yet)";

void handleLine(const char* line) {
  gLines++;
  strncpy(gLast, line, sizeof(gLast) - 1);
  gLast[sizeof(gLast) - 1] = '\0';

  if (strncmp(line, "PING ", 5) == 0) {
    gPings++;
    Serial1.printf("ACK %s\n", line + 5);
    gAcks++;
    return;
  }
  gGarbage++;
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
  while (!Serial && millis() - start < 3000) { delay(10); }

  Serial1.begin(kLinkBaud, SERIAL_8N1, kRxPin, kTxPin);

  if (Serial) {
    Serial.println();
    Serial.println("=== Stage 1b: XIAO UART echo (no WiFi) ===");
    Serial.printf("  Serial1 @ %lu baud, RX=D7, TX=D6\n",
                  (unsigned long)kLinkBaud);
    Serial.println("  Answering \"PING <n>\" with \"ACK <n>\".");
    Serial.println();
  }
}

void loop() {
  pollLink();

  const uint32_t now = millis();
  if (now - gLastReportMs < 1000) { return; }
  gLastReportMs = now;

  // availableForWrite() guard, same as xiao_teensy_bridge.ino: on this chip
  // `if (Serial)` is true whenever the cable is plugged in even with nothing
  // draining the buffer, and printf() then blocks for seconds. Here that would
  // stall the ACKs and make the Teensy report loss that is not real.
  if (Serial && Serial.availableForWrite() >= 120) {
    Serial.printf("[uart] lines=%lu pings=%lu acks_sent=%lu garbage=%lu last=\"%s\"\n",
                  (unsigned long)gLines, (unsigned long)gPings,
                  (unsigned long)gAcks, (unsigned long)gGarbage, gLast);
  }
  gLines = gPings = gAcks = gGarbage = 0;
}

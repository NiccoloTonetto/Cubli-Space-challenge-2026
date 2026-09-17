/*
  Stage 0b -- Teensy 4.1 alive over USB.  No CAN, no IMU, no Serial1, no wires.
  ---------------------------------------------------------------------------
  Proves only: Teensyduino is installed, the board is selected as "Teensy 4.1",
  the upload works, you know which COM port is the Teensy (as opposed to the
  XIAO -- with both plugged in they are easy to mix up, and uploading the wrong
  sketch to the wrong board wastes far more time than this test costs).

  Unlike the XIAO, the Teensy's USB serial NEVER blocks when no host is
  reading, so nothing here can stall. That difference matters later: it is why
  the XIAO sketches guard their prints and this one does not.

  RUN IT WITH THE CUBE POWER OFF and only the Teensy's USB attached.

  Serial Monitor: baud is ignored by Teensy USB serial, any setting works.
  Expect one line per second and the on-board LED (pin 13) blinking.

  PASS: lines appear, LED blinks.
  FAIL: no port -> press the physical program button on the Teensy while
        uploading. Port but no text -> wrong board selected in Tools > Board.
*/

const uint8_t kLedPin = 13;

static uint32_t gLastPrintMs = 0;
static uint32_t gSeq = 0;

void setup() {
  Serial.begin(115200);          // value ignored on Teensy USB CDC
  pinMode(kLedPin, OUTPUT);

  const uint32_t start = millis();
  while (!Serial && millis() - start < 2000) { delay(10); }

  Serial.println();
  Serial.println("=== Stage 0b: Teensy 4.1 alive ===");
  Serial.printf("  F_CPU: %lu MHz\n", (unsigned long)(F_CPU / 1000000UL));
  Serial.println("  This is the TEENSY. Note its COM port number.");
  Serial.println();
}

void loop() {
  const uint32_t now = millis();
  if (now - gLastPrintMs < 1000) { return; }
  gLastPrintMs = now;

  digitalWrite(kLedPin, (gSeq & 1) ? HIGH : LOW);
  Serial.printf("[alive] seq=%lu uptime=%lus\n",
                (unsigned long)gSeq++, (unsigned long)(now / 1000));
}

/*
  Stage 4 -- the WHOLE pipeline at the real data rate, with NO control law.
  ---------------------------------------------------------------------------
  Teensy -> Serial1 @ 1 Mbaud -> XIAO -> WiFi/UDP -> laptop, exercised exactly
  as CornerBalance_WiFi.ino exercises it: 21 CSV fields, 250 lines/s, the same
  command grammar, the same '#' console lines.

  What is DELIBERATELY absent: moteus/CAN, the BMI270, the estimator, the
  control law, arming. Nothing here can move a wheel -- there is no CAN traffic
  at all. That is the point. When this passes you know the transport is sound,
  so the first time you run the real firmware, any problem you see is in the
  firmware and not in the link. If you skip this stage you will be debugging
  both at once, on a cube that can throw itself off the bench.

  The XIAO runs the REAL bridge for this stage:
      ../../../xiao_teensy_bridge/xiao_teensy_bridge.ino
  Flash it now -- it is what will be running for the rest of the project's
  life, and Stage 4 is the last chance to test it against a source that cannot
  hurt anybody.

  POWER: both boards on their OWN USB cable, cube power OFF. Both boards get
  their 5 V from the laptop, the D6/D7/GND jumpers stay connected, and nothing
  touches the LM2596 rail. Stage 5 repeats this test on rail power.

  ---------------------------------------------------------------------------
  WIRE FORMAT -- byte-compatible with the real corner build:

    t_ms, phi_x,phi_y,phi_z, om_x,om_y,om_z, rho_x,rho_y,rho_z,
    rho_x_lp,rho_y_lp,rho_z_lp, tau_x,tau_y,tau_z,
    tau_cmd_x,tau_cmd_y,tau_cmd_z, armed, gain_scale          = 21 fields

  t_ms is EXACTLY seq*4 (250 Hz), never wall-clock millis(). That makes the
  sequence number recoverable on the laptop as t_ms/4, so stage4_rate_check.py
  can count dropped lines exactly instead of estimating a rate. The values are
  slow sinusoids so ../../../telemetry_python_wifi_corner.py also draws
  something recognisable if you point it here -- a good way to prove the
  plotter itself works before the cube depends on it.

  COMMAND GRAMMAR -- same as CornerBalance_WiFi.ino, so muscle memory built
  here transfers:
    h / k     no-op keepalive (the PC scripts send 'h' every 100 ms)
    a1 / a0   flips the `armed` COLUMN only. No motor exists. It is here so
              you can verify the command path end to end, and see the
              telemetry column change in response.
    p1 / p0   pause / resume the stream (p1 = silence, to check the watchdog
              and to see what "dead link" looks like on purpose)
    c         answers "# faker: corner candidate 3 (synthetic)"
    z1 / z0   answers a tare acknowledgement
    g<0..1>   sets the gain_scale column
    t0 / t1   move the stream to USB / Serial1, exactly like the real build
    (x0/x1 never arrive here -- the XIAO consumes them.  That is expected and
     is itself worth confirming: send x0 and this board must NOT react.)
*/

static const uint32_t kLinkBaud  = 1000000;   // must match the XIAO bridge
static const uint32_t kPeriodUs  = 4000;      // 250 Hz -> 4 ms
static const uint32_t kStatusMs  = 1000;

// Teensy 4.x gives each HardwareSerial a 40-byte TX buffer by default. A
// 21-field line is ~170 B typical and ~200 B worst case, so availableForWrite()
// can NEVER reach n and emitLine()'s room check below rejects every single
// line: lines_out=0, tx_skipped=250/s, and not one byte on the wire. The
// symptom is indistinguishable from a dead link if you only look at the laptop.
//
// 1 KiB is ~5 worst-case lines. At 250 Hz x 200 B = 50 kB/s against the
// ~100 kB/s a 1 Mbaud link carries, the buffer sits near empty in steady state;
// the depth is headroom for the '#' console bursts, not a queue we intend to
// fill. It drains in ~10 ms if it ever does back up, so the room check stays a
// real non-blocking guarantee rather than a permanent veto.
static uint8_t gSerial1TxBuffer[1024];

enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // same boot default as the real build

static bool     gArmed   = false;
static bool     gHalted  = false;
static float    gGain    = 1.0f;
static uint32_t gSeq     = 0;

static uint32_t gNextSendUs   = 0;
static uint32_t gLastStatusMs = 0;
static uint32_t gLinesOut = 0, gTxSkipped = 0, gCmdsIn = 0;

// Non-blocking reader, same idea as the real build's LineReader: never let a
// partial line block the emit schedule.
struct LineReader {
  char buf[96];
  uint8_t len = 0;
  bool feed(Stream& s, char* out, size_t outSize) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) { continue; }
        buf[len] = '\0';
        strncpy(out, buf, outSize - 1);
        out[outSize - 1] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) { buf[len++] = c; }
    }
    return false;
  }
};
static LineReader gUsbReader, gLinkReader;

Stream& outStream() {
  return (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
}

void console(const char* msg) {
  // '#' prefix: telemetry_python_wifi_corner.py prints these to the terminal
  // instead of parsing them, so command answers stay visible in PLOTMODE
  // without corrupting the CSV.
  outStream().printf("# %s\n", msg);
}

void handleCommand(const char* line) {
  gCmdsIn++;
  switch (line[0]) {
    case 'h': case 'k':
      return;                                   // keepalive -- MUST be a no-op
    case 'a':
      gArmed = (line[1] == '1');
      console(gArmed ? "faker: armed=1 (column only, no motors exist)"
                     : "faker: armed=0");
      return;
    case 'p':
      gHalted = (line[1] == '1');
      console(gHalted ? "faker: HALTED -- stream stopped on purpose"
                      : "faker: resumed");
      return;
    case 'c':
      console("faker: corner candidate 3 (synthetic)");
      return;
    case 'z':
      console(line[1] == '1' ? "faker: phi offset tared (synthetic)"
                             : "faker: phi offset cleared");
      return;
    case 'g': {
      const float g = atof(line + 1);
      if (g >= 0.0f && g <= 1.0f) { gGain = g; console("faker: gain set"); }
      else                        { console("faker: gain out of range 0..1"); }
      return;
    }
    case 't':
      gLinkMode = (line[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      console(gLinkMode == LinkMode::WIFI ? "faker: link=WIFI (Serial1)"
                                          : "faker: link=USB (Serial)");
      return;
    case 'x':
      // Should be unreachable over WiFi: xiao_teensy_bridge.ino eats 'x'.
      // If you DO see this over the WiFi path, the bridge is not the sketch
      // you think it is -- reflash it.
      console("faker: got an 'x' -- the XIAO should have consumed it!");
      return;
    default:
      console("faker: unknown command");
      return;
  }
}

void pollCommands() {
  char line[96];
  // Both streams every iteration, like the real build, so a mode switch can
  // never be missed on whichever channel happens to be reachable.
  if (gUsbReader.feed(Serial, line, sizeof(line)))  { handleCommand(line); }
  if (gLinkReader.feed(Serial1, line, sizeof(line))) { handleCommand(line); }
}

void emitLine() {
  const uint32_t tMs = gSeq * 4;               // exact: seq = t_ms / 4
  const float ph = (float)gSeq * 0.01f;
  const float s1 = sinf(ph), s2 = sinf(ph * 0.7f + 1.0f), s3 = sinf(ph * 1.3f + 2.0f);

  char buf[224];
  const int n = snprintf(buf, sizeof(buf),
      "%lu,"
      "%.3f,%.3f,%.3f,"          // phi x/y/z  deg
      "%.2f,%.2f,%.2f,"          // om  x/y/z  dps
      "%.2f,%.2f,%.2f,"          // rho x/y/z
      "%.2f,%.2f,%.2f,"          // rho_lp
      "%.4f,%.4f,%.4f,"          // tau
      "%.4f,%.4f,%.4f,"          // tau_cmd
      "%d,%.2f\n",
      (unsigned long)tMs,
      2.0f * s1, 2.0f * s2, 2.0f * s3,
      30.0f * s2, 30.0f * s3, 30.0f * s1,
      100.0f * s1, 100.0f * s2, 100.0f * s3,
      60.0f * s1, 60.0f * s2, 60.0f * s3,
      0.15f * s1, 0.15f * s2, 0.15f * s3,
      0.15f * s1, 0.15f * s2, 0.15f * s3,
      gArmed ? 1 : 0, gGain);

  if (n < 0 || n >= (int)sizeof(buf)) { gTxSkipped++; gSeq++; return; }   // truncated

  // Never block inside the emit slot. If the TX buffer cannot take the whole
  // line, DROP it and count the drop -- a nonzero tx_skipped means the TEENSY
  // is the bottleneck, which is a completely different fault from the laptop
  // seeing gaps because the XIAO or the air link dropped them. Without this
  // counter those two are indistinguishable from the PC end.
  const bool wifi = (gLinkMode == LinkMode::WIFI);
  const int room = wifi ? Serial1.availableForWrite() : Serial.availableForWrite();
  if (room >= n) {
    if (wifi) { Serial1.write((const uint8_t*)buf, n); }
    else      { Serial.write((const uint8_t*)buf, n); }
    gLinesOut++;
  } else {
    gTxSkipped++;
  }

  gSeq++;
}

void printStatus() {
  const uint32_t now = millis();
  if (now - gLastStatusMs < kStatusMs) { return; }
  gLastStatusMs = now;

  // To USB, always -- this is the Teensy's own account of what it sent, and
  // it must stay readable even when the stream is going out over Serial1.
  Serial.printf("[faker] lines_out=%lu/s tx_skipped=%lu cmds_in=%lu armed=%d "
                "halted=%d link=%s seq=%lu\n",
                (unsigned long)gLinesOut, (unsigned long)gTxSkipped,
                (unsigned long)gCmdsIn, gArmed ? 1 : 0, gHalted ? 1 : 0,
                gLinkMode == LinkMode::WIFI ? "WIFI" : "USB",
                (unsigned long)gSeq);
  gLinesOut = gTxSkipped = gCmdsIn = 0;
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 2000) { delay(10); }

  Serial1.begin(kLinkBaud);      // RX1 = pin 0, TX1 = pin 1
  Serial1.addMemoryForWrite(gSerial1TxBuffer, sizeof(gSerial1TxBuffer));

  Serial.println();
  Serial.println("=== Stage 4: Teensy CSV faker (no CAN, no IMU, no motors) ===");
  Serial.printf("  Serial1 TX buffer: %u B (default 40 B cannot hold one line)\n",
                (unsigned)sizeof(gSerial1TxBuffer));
  Serial.printf("  21 fields at %lu Hz on Serial1 @ %lu baud\n",
                (unsigned long)(1000000UL / kPeriodUs), (unsigned long)kLinkBaud);
  Serial.println("  t_ms == seq*4, so the laptop can count dropped lines exactly.");
  Serial.println("  Watch tx_skipped: nonzero means the TEENSY is the bottleneck.");
  Serial.println();

  gNextSendUs = micros();
}

void loop() {
  pollCommands();

  const uint32_t now = micros();
  if (!gHalted && (int32_t)(now - gNextSendUs) >= 0) {
    gNextSendUs += kPeriodUs;
    // If we ever fall a whole period behind (should not happen with no CAN or
    // IMU work here), resynchronise rather than emitting a burst to catch up.
    if ((int32_t)(now - gNextSendUs) > (int32_t)kPeriodUs) { gNextSendUs = now + kPeriodUs; }
    emitLine();
  }

  printStatus();
}

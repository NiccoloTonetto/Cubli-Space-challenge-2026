// ============================================================================
// TEENSY 4.1 + 1x moteus-n1 (CAN3) — PHASE 0.2: SINGLE MOTEUS QUERY/REPLY
// ============================================================================
// First cube-hardware test in the bring-up sequence. No IMU, no other
// wheels, no torque above zero -- this isolates the CAN-FD link itself
// before anything else touches it, same philosophy as the panel's Stage 0
// (see "Firmware Lessons -- 2D Panel to 3D Cube.md" S1, S7).
//
// Driver commissioning (encoder offset, phase order, v_per_hz) is assumed
// already done via moteus_tool/tview -- this sketch does NOT calibrate the
// motor. It tests the APPLICATION layer: our CAN traffic pattern, our
// SetPosition Format contract, and clean query/reply over the actual cube
// wiring (not the panel's).
//
// PASS CRITERION (Phase 0.2): clean query/reply for 60 s, zero errors.
// Exercises all three required behaviors in sequence:
//   Phase A (5 s)  -- repeated SetStop(), verifies stop + query together.
//   Phase B (5 s)  -- repeated bare SetQuery(), verifies query alone.
//   Phase C (50 s) -- repeated zero feedforward_torque via the Format-fixed
//                     PositionMode command, verifies "send zero torque" is
//                     actually reaching the wire (Firmware Lessons S2).
//
// CHANGE kMoteusId BELOW to test each wheel in turn -- do not assume the
// link is clean on all three because it was clean on one.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

static const int8_t kMoteusId = 1;   // TODO: re-run with 2, then 3.

Moteus moteus1(canBus, []() {
  Moteus::Options options;
  options.id = kMoteusId;
  // Default is 2000 us. Widened so a slow-but-real reply is MEASURED as a
  // large latency instead of silently folding into "no reply" at exactly
  // the pass/fail boundary -- we want the actual number, not a coin flip
  // at 2.0 ms. Stage 0b's histogram is where the real pass/fail lives.
  options.min_rcv_wait_us = 5000;
  return options;
}());

static const uint32_t kPeriodUs = 2500;   // 400 Hz, matches FS_HZ in cubli_gains.h
static const uint32_t kPhaseA_Us = 5000000UL;    // 5 s  -- SetStop()
static const uint32_t kPhaseB_Us = 5000000UL;    // 5 s  -- SetQuery()
static const uint32_t kPhaseC_Us = 50000000UL;   // 50 s -- zero-torque PositionMode
static const uint32_t kTotal_Us  = kPhaseA_Us + kPhaseB_Us + kPhaseC_Us;   // 60 s

// Same Format-fixing pattern as the panel's Stage1_OpenLoopTorque.ino and
// Gam/Skeleton_3Axis.ino -- SetPosition()'s DEFAULT wire format only
// transmits position/velocity; every other Command field is silently
// dropped unless this Format explicitly turns each one on. Confirmed
// against mjbots/moteus-arduino's moteus_protocol.h; see
// "Firmware Lessons -- 2D Panel to 3D Cube.md" S2.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque    = Moteus::kFloat;
  f.kp_scale               = Moteus::kFloat;
  f.kd_scale                = Moteus::kFloat;
  f.maximum_torque          = Moteus::kFloat;
  f.watchdog_timeout        = Moteus::kFloat;
  f.ignore_position_bounds  = Moteus::kFloat;
  return f;
}();

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: STATS
// ----------------------------------------------------------------------------

struct PhaseStats {
  uint32_t cycles       = 0;
  uint32_t replies      = 0;
  uint32_t noReply      = 0;    // SetStop()/SetQuery()/SetPosition() returned false
  uint32_t faultNonzero = 0;    // reply received but v.fault != 0
  uint32_t rtMinUs      = 0xFFFFFFFFUL;
  uint32_t rtMaxUs      = 0;
  uint64_t rtSumUs      = 0;    // for a mean; 64-bit, this will not overflow at 400 Hz for 60 s
};

PhaseStats gStatsA, gStatsB, gStatsC;

void recordReply(PhaseStats& s, bool ok, uint32_t rtUs, int8_t fault) {
  s.cycles++;
  if (!ok) { s.noReply++; return; }
  s.replies++;
  if (fault != 0) { s.faultNonzero++; }
  if (rtUs < s.rtMinUs) { s.rtMinUs = rtUs; }
  if (rtUs > s.rtMaxUs) { s.rtMaxUs = rtUs; }
  s.rtSumUs += rtUs;
}

void printPhaseStats(const char* name, const PhaseStats& s) {
  Serial.print("# "); Serial.print(name);
  Serial.print(": cycles="); Serial.print(s.cycles);
  Serial.print(" replies="); Serial.print(s.replies);
  Serial.print(" noReply="); Serial.print(s.noReply);
  Serial.print(" faultNonzero="); Serial.print(s.faultNonzero);
  if (s.replies > 0) {
    Serial.print(" rt_us[min="); Serial.print(s.rtMinUs);
    Serial.print(" mean="); Serial.print((uint32_t)(s.rtSumUs / s.replies));
    Serial.print(" max="); Serial.print(s.rtMaxUs);
    Serial.print("]");
  }
  Serial.println();
}


// ----------------------------------------------------------------------------
// SECTION 2c: SERIAL COMMANDS
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  const String line = Serial.readStringUntil('\n');
  if (line.length() == 0) { return; }
  if (line[0] == 'h') {
    gHalted = !gHalted;
    Serial.print("# gHalted = "); Serial.println(gHalted ? "TRUE" : "FALSE");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - PHASE 0.2: SINGLE MOTEUS QUERY/REPLY");
  Serial.print("# testing moteus id "); Serial.println(kMoteusId);

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  const bool stopOk = moteus1.SetStop();
  Serial.print("# initial SetStop() reply: "); Serial.println(stopOk ? "OK" : "NO REPLY");

  Serial.println("# Phase A (5s): SetStop() at 400 Hz");
  Serial.println("# send h to halt/resume, otherwise just watch");
}


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();
  if (gHalted) { return; }

  static uint32_t nextCycleUs = micros();
  static uint32_t startUs = micros();
  static bool announcedB = false, announcedC = false, done = false;

  if (done) { return; }

  const uint32_t now = micros();
  if ((int32_t)(now - nextCycleUs) < 0) { return; }
  nextCycleUs += kPeriodUs;

  const uint32_t elapsed = now - startUs;

  if (elapsed < kPhaseA_Us) {
    // --- Phase A: SetStop() ---
    const uint32_t t0 = micros();
    const bool ok = moteus1.SetStop();
    const uint32_t rt = micros() - t0;
    const auto& v = moteus1.last_result().values;
    recordReply(gStatsA, ok, rt, (int8_t)v.fault);

  } else if (elapsed < kPhaseA_Us + kPhaseB_Us) {
    // --- Phase B: bare query ---
    if (!announcedB) {
      announcedB = true;
      printPhaseStats("Phase A (SetStop)", gStatsA);
      Serial.println("# Phase B (5s): SetQuery() at 400 Hz");
    }
    const uint32_t t0 = micros();
    const bool ok = moteus1.SetQuery();
    const uint32_t rt = micros() - t0;
    const auto& v = moteus1.last_result().values;
    recordReply(gStatsB, ok, rt, (int8_t)v.fault);

  } else if (elapsed < kTotal_Us) {
    // --- Phase C: zero feedforward torque, Format explicitly turned on ---
    if (!announcedC) {
      announcedC = true;
      printPhaseStats("Phase B (SetQuery)", gStatsB);
      Serial.println("# Phase C (50s): zero-torque PositionMode at 400 Hz");
    }
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = 0.0f;
    cmd.maximum_torque           = 0.12f;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;

    const uint32_t t0 = micros();
    const bool ok = moteus1.SetPosition(cmd, &kTorqueFormat);
    const uint32_t rt = micros() - t0;
    const auto& v = moteus1.last_result().values;
    recordReply(gStatsC, ok, rt, (int8_t)v.fault);

  } else {
    done = true;
    printPhaseStats("Phase C (zero torque)", gStatsC);

    const uint32_t totalErrors = gStatsA.noReply + gStatsA.faultNonzero
                                + gStatsB.noReply + gStatsB.faultNonzero
                                + gStatsC.noReply + gStatsC.faultNonzero;
    Serial.println();
    Serial.print("# ================ RESULT: ");
    Serial.print(totalErrors == 0 ? "PASS" : "FAIL");
    Serial.print(" (");
    Serial.print(totalErrors);
    Serial.println(" total errors) ================");
    Serial.println("# send h to halt, or reset the board to re-run");
  }
}


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Re-run with kMoteusId = 2 and 3 before moving to Stage0b -- a clean link
// on one wheel says nothing about the other two (different CAN node, but
// also physically different wiring run to that corner of the cube).
//
// If Phase C fails but A and B pass: SetStop()/SetQuery() don't exercise
// the Format-augmented Command path at all, so a Phase-C-only failure
// means the problem is specific to the torque-mode command construction
// (kTorqueFormat), not the link.
// ============================================================================

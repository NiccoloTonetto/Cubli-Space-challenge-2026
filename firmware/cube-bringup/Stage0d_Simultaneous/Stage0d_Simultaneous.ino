// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) — PHASE 0.3b: SIMULTANEOUS POWERED SPIN
// ============================================================================
// Run this after Stage0_SingleMoteusQuery has passed on all 3 wheel IDs
// individually AND Stage0b_ThreeMoteusLatency has passed. Those two prove
// the CAN-FD link is clean; neither one proves the wheels can actually be
// COMMANDED to move -- Stage0_SingleMoteusQuery only ever sends zero
// feedforward_torque, and Stage0b's 10-minute loop does the same on all
// three. A moteus can query/reply/accept-zero-torque perfectly fine even
// if its motor phases are unplugged or its bus supply isn't actually
// connected -- that failure mode is invisible to both prior stages.
//
// This sketch is the simplest possible closed loop: command all three
// wheels to the SAME constant, modest velocity at the same time, using
// moteus's velocity-via-PositionMode trick (kp_scale=0 kills the position
// term, kd_scale=1 makes the servo track the velocity target -- position
// itself stays NaN/ignored, same Format-fixing pattern as every other
// stage in this repo).
//
// ARM/STOP, MANUAL, NO AUTO-STOP:
//   The rig starts DISARMED and stays that way until you tell it
//   otherwise -- send "a" over Serial to arm (all 3 wheels ramp to
//   gSpinVelocityRevS and hold there, indefinitely, no time limit), send
//   "s" to stop (SetStop() on all 3, indefinitely, until armed again).
//   Send "v<revs>" any time (armed or not) to change the target speed --
//   e.g. "v2.5" -- clamped to +/-kMaxSpinVelocityRevS, takes effect on the
//   very next command cycle if already armed. There is no automatic phase
//   timeout here on purpose -- unlike this repo's other Stage0 files, this
//   one is meant to be left running for as long as you want to watch/
//   measure it, not to self-terminate after a fixed window. YOU decide
//   when it stops.
//
// CUBE SHOULD BE RESTING FREE ON A BENCH/TABLE, NOT HAND-HELD. At constant
// velocity a wheel imparts ~zero net reaction torque on the body; the only
// torque is during the brief torque-limited ramp up to speed, capped by
// kTauMax same as every other stage. This is NOT a balance test and reads
// no IMU -- edge-bringup/Stage1_WheelSignCheck.ino is where per-wheel
// reaction-torque sign gets checked, hand-held, after this passes.
//
// PASS CRITERION, per armed session: zero noReply, zero non-zero fault,
// each wheel's velocity settles within kVelTolRevS of gSpinVelocityRevS.
// Printed as a summary the moment you send "s" -- see Section 2b.
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

static Moteus::Options MakeOptions(int8_t id) {
  Moteus::Options options;
  options.id = id;
  options.min_rcv_wait_us = 5000;   // widened, same reasoning as Stage0/Stage0b --
                                      // want a slow reply MEASURED as noReply's
                                      // opposite, not silently folded in either way.
  return options;
}

// Confirmed on bench, physically verified per wheel via
// Stage0_SingleMoteusQuery: id 2 -> X, id 3 -> Y, id 1 -> Z.
Moteus moteusX(canBus, MakeOptions(2));
Moteus moteusY(canBus, MakeOptions(3));
Moteus moteusZ(canBus, MakeOptions(1));
Moteus* const gWheels[3] = { &moteusX, &moteusY, &moteusZ };
static const char* kWheelName[3] = { "X", "Y", "Z" };

static const uint32_t kPeriodUs      = 2500;      // 400 Hz, matches FS_HZ in cubli_gains.h
static const uint32_t kStatusEveryUs = 1000000UL; // live status line every 1 s

// Same target for all three -- "constant speed" for this check means
// literally the same commanded velocity, same sign, on every wheel at
// once. Modest default: 1.5 rev/s (~9.4 rad/s), live-settable below via
// "v<revs>" over Serial. Clamped to kMaxSpinVelocityRevS, comfortably
// under OMEGA_CAP (40 rad/s = ~6.37 rev/s) from cubli_gains.h, with no
// balance loop running to need headroom above it.
static float gSpinVelocityRevS = 1.5f;
static const float kMaxSpinVelocityRevS = 60.0f;  // rev/s, clamp for "v<revs>"
static const float kTauMax = 0.12f;              // N*m, TAU_MAX from cubli_gains.h
static const float kVelTolRevS = 0.3f;           // convergence tolerance for the summary

// Same Format-fixing pattern as every other stage: SetPosition()'s DEFAULT
// wire format only transmits position/velocity; every other Command field
// is silently dropped unless explicitly turned on here.
static Moteus::PositionMode::Format kSpinFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque    = Moteus::kFloat;
  f.kp_scale               = Moteus::kFloat;
  f.kd_scale                = Moteus::kFloat;
  f.maximum_torque          = Moteus::kFloat;
  f.watchdog_timeout        = Moteus::kFloat;
  f.ignore_position_bounds  = Moteus::kFloat;
  return f;
}();

static bool gArmed = false;   // starts DISARMED -- must send "a" to spin.


// ----------------------------------------------------------------------------
// SECTION 2b: PER-WHEEL STATS -- reset every time "a" is sent, reported
// when "s" is sent, so each armed session gets its own clean summary.
// ----------------------------------------------------------------------------

struct WheelStats {
  uint32_t cycles       = 0;
  uint32_t replies      = 0;
  uint32_t noReply      = 0;    // SetPosition() returned false
  uint32_t faultNonzero = 0;    // reply received but v.fault != 0
  float    lastVelocity = 0.0f; // rev/s, most recent successful reading
  float    lastTorque   = 0.0f; // N*m, most recent successful reading
  float    emaVelocity  = 0.0f; // exponential moving average, ~2.5s time
                                  // constant -- a "has it settled" number
                                  // that doesn't need a fixed window,
                                  // since there's no fixed run length here.
  bool     emaInit      = false;
};

WheelStats gStats[3];

// Explicit prototypes, placed here (after WheelStats is defined) so the
// Arduino IDE's auto-prototype pass -- which otherwise inserts its own
// prototypes right after the last #include, BEFORE this struct exists --
// finds these already-visible ones and skips generating the broken kind.
// Same fix needed anywhere a .ino-local struct is passed to a function.
void recordWheel(WheelStats& s, bool ok, float vel, float torque, int8_t fault);
void printSessionSummary();

static const float kEmaAlpha = 0.01f;   // ~2.5 s time constant at 400 Hz

void recordWheel(WheelStats& s, bool ok, float vel, float torque, int8_t fault) {
  s.cycles++;
  if (!ok) { s.noReply++; return; }
  s.replies++;
  if (fault != 0) { s.faultNonzero++; }
  s.lastVelocity = vel;
  s.lastTorque = torque;
  if (!s.emaInit) { s.emaVelocity = vel; s.emaInit = true; }
  else { s.emaVelocity += kEmaAlpha * (vel - s.emaVelocity); }
}

void resetStats() {
  for (int i = 0; i < 3; ++i) { gStats[i] = WheelStats(); }
}

void printSessionSummary() {
  Serial.println("# ------------ armed-session summary ------------");
  uint32_t totalErrors = 0;
  bool allConverged = true;
  for (int i = 0; i < 3; ++i) {
    const WheelStats& s = gStats[i];
    const uint32_t wheelErrors = s.noReply + s.faultNonzero;
    totalErrors += wheelErrors;
    const float velErr = fabsf(s.emaVelocity - gSpinVelocityRevS);
    const bool converged = s.emaInit && (velErr <= kVelTolRevS);
    allConverged &= converged;
    Serial.print("#   "); Serial.print(kWheelName[i]);
    Serial.print(": cycles="); Serial.print(s.cycles);
    Serial.print(" replies="); Serial.print(s.replies);
    Serial.print(" noReply="); Serial.print(s.noReply);
    Serial.print(" faultNonzero="); Serial.print(s.faultNonzero);
    Serial.print(" settledVel_revs="); Serial.print(s.emaVelocity, 3);
    Serial.print(" (target "); Serial.print(gSpinVelocityRevS, 2);
    Serial.print(") -> "); Serial.println(converged ? "CONVERGED" : "DID NOT CONVERGE");
  }
  const bool pass = (totalErrors == 0) && allConverged;
  Serial.print("# RESULT: "); Serial.print(pass ? "PASS" : "FAIL");
  Serial.print(" ("); Serial.print(totalErrors);
  Serial.println(" total comms errors)");
  Serial.println("# -------------------------------------------------");
}


// ----------------------------------------------------------------------------
// SECTION 2c: SERIAL COMMANDS -- "a" arm (spin, perpetual), "s" stop
// (SetStop() on all 3, perpetual, this is also the boot-default state).
// No automatic transition either direction -- this stays exactly as
// armed/disarmed as you last left it until you send the other command.
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  const String line = Serial.readStringUntil('\n');
  if (line.length() == 0) { return; }
  const char cmd = line.charAt(0);

  if (cmd == 'a') {
    if (gArmed) {
      Serial.println("# already armed, ignored");
    } else {
      gArmed = true;
      resetStats();
      Serial.print("# ARMED -- all 3 wheels -> "); Serial.print(gSpinVelocityRevS, 2);
      Serial.println(" rev/s, holding until \"s\" is sent");
    }
  } else if (cmd == 's') {
    if (!gArmed) {
      Serial.println("# already stopped, ignored");
    } else {
      gArmed = false;
      Serial.println("# STOP -- SetStop() on all 3");
      printSessionSummary();
    }
  } else if (cmd == 'v') {
    const float val = line.substring(1).toFloat();
    gSpinVelocityRevS = val >  kMaxSpinVelocityRevS ?  kMaxSpinVelocityRevS
                       : val < -kMaxSpinVelocityRevS ? -kMaxSpinVelocityRevS : val;
    Serial.print("# gSpinVelocityRevS = "); Serial.print(gSpinVelocityRevS, 3);
    Serial.println(" rev/s (takes effect immediately if armed)");
  } else {
    Serial.println("# unknown. use: a (arm, spin all 3)  s (stop all 3)  v<revs> (set speed)");
  }
}


// ----------------------------------------------------------------------------
// SECTION 2d: CAN BUS DIAGNOSTICS
// ----------------------------------------------------------------------------
// Same as Stage0/Stage0b -- kBusOff/climbing error counters means
// electrical contention; staying ACTIVE with 0 errors while noReply is
// nonzero on exactly one wheel points at that wheel specifically (power,
// wiring, or ID), not the bus.

const char* canStateName(tControllerState s) {
  switch (s) {
    case kActive:  return "ACTIVE";
    case kPassive: return "PASSIVE";
    case kBusOff:  return "BUS_OFF";
  }
  return "?";
}

void printCanDiagnostics() {
  Serial.print("# CAN state: "); Serial.print(canStateName(ACAN_T4::can3.controllerState()));
  Serial.print("  rxErr="); Serial.print(ACAN_T4::can3.receiveErrorCounter());
  Serial.print("  txErr="); Serial.println(ACAN_T4::can3.transmitErrorCounter());
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - PHASE 0.3b: SIMULTANEOUS POWERED SPIN (manual arm/stop)");
  Serial.println("# cube resting free on a bench/table -- NOT hand-held.");
  Serial.print("# armed target: all 3 wheels to "); Serial.print(gSpinVelocityRevS, 2);
  Serial.println(" rev/s at the same time, held until you send \"s\"");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  Serial.println("# CAN-FD peripheral initialized locally -- this does NOT confirm a live bus/counterpart:");
  printCanDiagnostics();

  for (int i = 0; i < 3; ++i) {
    const bool ok = gWheels[i]->SetStop();
    Serial.print("# initial SetStop() "); Serial.print(kWheelName[i]);
    Serial.print(": "); Serial.println(ok ? "OK" : "NO REPLY");
  }
  printCanDiagnostics();

  Serial.println("# DISARMED -- send \"a\" to arm (spin all 3), \"s\" to stop, \"v<revs>\" to set speed");
}


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();

  static uint32_t nextCycleUs  = micros();
  static uint32_t startUs      = micros();
  static uint32_t nextStatusUs = micros() + kStatusEveryUs;

  const uint32_t now = micros();
  if ((int32_t)(now - nextCycleUs) < 0) { return; }
  nextCycleUs += kPeriodUs;

  for (int i = 0; i < 3; ++i) {
    bool ok;
    if (gArmed) {
      Moteus::PositionMode::Command cmd;
      cmd.position               = NaN;
      cmd.velocity               = gSpinVelocityRevS;
      cmd.kp_scale                = 0.0f;   // no position term
      cmd.kd_scale                 = 1.0f;   // full velocity-tracking gain
      cmd.feedforward_torque       = 0.0f;
      cmd.maximum_torque           = kTauMax;
      cmd.watchdog_timeout          = 0.10f;
      cmd.ignore_position_bounds    = 1.0f;
      ok = gWheels[i]->SetPosition(cmd, &kSpinFormat);
    } else {
      ok = gWheels[i]->SetStop();
    }
    const auto& v = gWheels[i]->last_result().values;
    recordWheel(gStats[i], ok, v.velocity, v.torque, (int8_t)v.fault);
  }

  if ((int32_t)(now - nextStatusUs) >= 0) {
    nextStatusUs += kStatusEveryUs;
    const uint32_t elapsed = now - startUs;
    Serial.print("# t="); Serial.print(elapsed / 1000000UL);
    Serial.print("s  "); Serial.print(gArmed ? "ARMED" : "stopped");
    Serial.print("  vel[X,Y,Z]_revs=[");
    Serial.print(gWheels[0]->last_result().values.velocity, 2); Serial.print(", ");
    Serial.print(gWheels[1]->last_result().values.velocity, 2); Serial.print(", ");
    Serial.print(gWheels[2]->last_result().values.velocity, 2); Serial.print("]");
    Serial.print("  fault[X,Y,Z]=[");
    Serial.print((int)gWheels[0]->last_result().values.fault); Serial.print(", ");
    Serial.print((int)gWheels[1]->last_result().values.fault); Serial.print(", ");
    Serial.print((int)gWheels[2]->last_result().values.fault); Serial.println("]");
    printCanDiagnostics();
  }
}


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// If one wheel shows noReply/fault or never converges while the other two
// look clean: that isolates to that wheel specifically (power, phase
// wiring, or ID) with all three sharing the same bus and timing, which is
// the exact ambiguity a single-wheel-at-a-time test cannot resolve.
//
// If a wheel converges to roughly HALF or DOUBLE gSpinVelocityRevS, or the
// sign is flipped relative to the other two: that is a driver-commissioning
// issue (encoder scale, pole count, or phase order) that moteus_tool/tview
// calibration owns, not this application-layer test -- do not try to fix
// it here.
//
// Next: edge-bringup/Stage1_WheelSignCheck.ino -- cube hand-held, one wheel
// at a time, real torque pulses, checking reaction-torque SIGN against the
// estimator. That stage needs kAxisWheelSign verified per wheel; this one
// only proves all three can be powered and controlled together.
// ============================================================================

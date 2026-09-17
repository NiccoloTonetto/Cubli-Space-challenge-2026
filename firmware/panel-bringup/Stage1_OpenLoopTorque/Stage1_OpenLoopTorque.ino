// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 1: OPEN-LOOP TORQUE
// ============================================================================
// PANEL HELD FIRMLY BY HAND. No feedback loop yet — this is the first time
// this rig ever commands nonzero torque, and the first real test of
// torque mode (Bring-Up Plan §0, Option A: position mode with kp/kd scaled
// to zero, command carried entirely by feedforward_torque).
//
// From: Arduino Bring-Up Plan — Sections 2b and 2d, §2 "Stage 1".
//
// STAGE 1 CHECKLIST (~15 min) — hold the panel firmly by hand.
//   Send "p" over Serial to fire ONE pulse: 0.05 N*m for 1000 ms.
//   [ ] Wheel spins                          -> torque mode works
//   [ ] wheel_vel goes POSITIVE for a positive pulse -> sign check 3
//   [ ] Does the panel push toward NEGATIVE theta while the wheel speeds
//       up?                                  -> sign check 4
//
// 0.05 N*m, not 0.01: confirmed empirically on this rig -- 0.01 N*m sat
// below real wheel/bearing stiction and produced no visible motion even
// with torque mode working correctly end to end (moteus_mode/fault stayed
// 10/0 throughout, i.e. no fault, it just wasn't enough torque to move).
// If 0.05 still doesn't move it on a rebuild, use "t<Nm>" to step up
// further (0.08, then look hard at the moteus_torque/qcurrent columns).
//
// >>> SIGN CHECK 4 IS THE ONE THAT KILLS HARDWARE. <<<
// If a positive torque pushes the panel the WRONG way, the correct negative
// sign in the control law (Stage 2 onward) will actively drive the fall
// once the loop closes. Fix the estimator sign or the encoder convention —
// NEVER flip the minus in the control law to compensate. This is also the
// only check in the whole bring-up that cannot be done without spinning
// the motor, which is exactly why it's gated behind Stage 0.
//
// CONFIRMED FAILED on this hardware: a positive pulse pushed the panel
// toward increasing |theta| (toward the mechanical limit), not back
// toward vertical. Fixed below with kWheelSign -- a single, clearly-
// labeled physical mounting-convention constant applied where torque
// leaves and wheel speed comes back in. K1/K2/K3 are untouched; this is
// deliberately NOT "flip the minus in the control law."
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

Moteus moteus1(canBus, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;   // see Stage 0 for why
const uint32_t kPeriodMs = 2;                 // 500 Hz, see Stage 0 for why

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Software pause -- see SECTION 2e. Closing the Serial Monitor does NOT
// stop a Teensy sketch; loop() keeps running regardless. This is the real
// way to freeze IMU reads and CAN traffic without reflashing or touching
// the board. Halting also cancels any in-flight pulse (see handleSerial-
// Commands) so resuming never surprises you with leftover torque -- and
// even without that, cmd.watchdog_timeout = 0.10f below means the moteus
// faults itself within 100 ms of the command stream stopping anyway.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION (unchanged from Stage 0, minus the theta_acc
// debug output — Stage 1's checklist only needs the filtered theta)
// ----------------------------------------------------------------------------

struct IMUData {
  float ax, ay, az;
  float wx, wy, wz;
};

static const float kG0 = 9.80665f;
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };
static const float kTau = 1.00f;
static const float kAccelGateLo = 0.85f * kG0;
static const float kAccelGateHi = 1.15f * kG0;

// TODO: replace with the value you measured in Stage 0 (checklist item:
// balance by hand, send "o<deg>" until theta reads ~0, write it down).
// Kept live-settable here too in case the mounting shifted since then.
static float kThetaOffset = 0.0f;   // rad

IMUData readIMU(BMI270& sensor) {
  IMUData d;
  d.ax = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  d.ay = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  d.az = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  d.wx = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  d.wy = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  d.wz = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
  return d;
}

void calculateState(const IMUData& imuData, float& theta, float& theta_dot) {
  static uint32_t lastMicros = 0;
  static bool initialized = false;

  const uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0.0f || dt > 0.5f) { dt = kPeriodMs * 1e-3f; }

  theta_dot = imuData.wz;

  const float theta_acc = atan2f(imuData.ax - imuData.ay,
                                 imuData.ax + imuData.ay) - kThetaOffset;

  const float aMag = sqrtf(imuData.ax*imuData.ax +
                           imuData.ay*imuData.ay +
                           imuData.az*imuData.az);
  const bool accelTrusted = (aMag > kAccelGateLo && aMag < kAccelGateHi);

  if (!initialized) {
    theta = theta_acc;
    initialized = true;
    return;
  }

  const float theta_gyro = theta + theta_dot * dt;
  if (accelTrusted) {
    const float alpha = kTau / (kTau + dt);
    theta = alpha * theta_gyro + (1.0f - alpha) * theta_acc;
  } else {
    theta = theta_gyro;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------

// Columns: time_ms  theta_deg  theta_dot_dps  tau_cmd_Nm  pulse_active
//          wheel_pos  wheel_vel  moteus_mode  moteus_fault  moteus_torque
//          moteus_qcurrent
//
// moteus_mode: 0 stopped, 1 FAULT, 2-4 preparing, 10 position, 12 zero vel,
//              15 brake (full list: mjbots.github.io/moteus/protocol/registers,
//              register 0x000).
// moteus_fault: only meaningful when mode==1. 39 = "outside limit" (position
//              bounds), 36 = "motor not configured" (never calibrated via
//              moteus_tool), 40 = under voltage, 34 = over voltage. Full
//              list: register 0x00f.
// moteus_torque / moteus_qcurrent: the servo's OWN measured output torque
//              (N*m) and q-axis current (A) -- i.e. what it thinks it's
//              actually doing, not what we asked for. With mode==10 and
//              fault==0 but the wheel not spinning, this is the field that
//              tells us apart two very different problems:
//                - reads near tau_cmd_Nm / clearly nonzero -> the servo IS
//                  applying torque; something mechanical (friction/stuck
//                  wheel) is the remaining question, not the command path.
//                - reads ~0 regardless of what's commanded -> feedforward
//                  torque still isn't reaching the current loop despite
//                  mode==10, fault==0 -- a deeper protocol issue.
//
// v.fault is int8_t -- Serial.print() has a print(char) overload that
// int8_t can bind to, which would print fault code 39 as the character
// "'" instead of the digits "39". Cast to (int) to force the numeric
// overload; same caution applies to v.mode's enum type.
void printState(uint32_t t_ms, float theta, float theta_dot, float tau,
                bool pulseActive, const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(theta     * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(theta_dot * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(pulseActive ? 1 : 0);
  Serial.print('\t'); Serial.print(v.position, 3);
  Serial.print('\t'); Serial.print(v.velocity, 3);
  Serial.print('\t'); Serial.print((int)v.mode);
  Serial.print('\t'); Serial.print((int)v.fault);
  Serial.print('\t'); Serial.print(v.torque, 4);
  Serial.print('\t'); Serial.println(v.q_current, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — single-shot open-loop pulse, no gains involved
// ----------------------------------------------------------------------------

static float    gTauPulse          = 0.05f;   // N*m — live-settable, see "t<Nm>".
                                               // 0.01 sat below real stiction on
                                               // this rig and produced no motion
                                               // even with a fully working command
                                               // path; 0.05 is the confirmed value.
static const uint32_t kPulseDurationMs = 1000;
static const float kTauMax         = 0.12f;   // N*m, continuous limit — hard
                                               // clamp even on a constant pulse

// Physical wheel-mounting sign convention. tau here is defined so that a
// positive value SHOULD push the panel toward decreasing |theta| (the LQR
// model's convention: theta_ddot = ... - tau/Tbar). Confirmed backwards on
// this hardware -- sign check 4 above. -1.0f corrects it at the hardware
// boundary, not by touching the control law. Applied only to the outgoing
// command in this file: Stage 1 has no control-law dependency on
// wheel_omega's sign (no taper, no k3 term, no feedback loop at all), so
// telemetry here (wheel_pos/wheel_vel/moteus_torque/moteus_qcurrent)
// intentionally still shows the RAW, unflipped moteus convention -- Stages
// 2-5 flip both sides, since they actually feed wheel_omega back into the
// law.
static const float kWheelSign = -1.0f;

// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field (feedforward_torque, kp_scale, kd_scale,
// maximum_torque, watchdog_timeout) defaults to Resolution::kIgnore and is
// silently DROPPED before it reaches the CAN bus unless this Format
// explicitly turns each one on. Without it, everything set on `cmd` below
// is real in this C++ struct and never leaves the Teensy: the moteus falls
// back to its own onboard kp_scale=1/kd_scale=1 and just holds velocity=0
// -- plain velocity mode, not torque mode, regardless of what commandWheel()
// computes. Confirmed against mjbots/moteus-arduino's moteus_protocol.h.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque    = Moteus::kFloat;
  f.kp_scale               = Moteus::kFloat;
  f.kd_scale                = Moteus::kFloat;
  f.maximum_torque          = Moteus::kFloat;
  f.watchdog_timeout        = Moteus::kFloat;
  f.ignore_position_bounds  = Moteus::kFloat;   // see cmd.ignore_position_bounds below
  return f;
}();

static bool     gPulseActive  = false;
static uint32_t gPulseStartMs = 0;
static float    gLastTau      = 0.0f;   // for telemetry

// Runs the open-loop pulse state machine and issues the resulting torque.
// This is the FIRST use of the moteus torque-mode command structure
// (Bring-Up Plan §0, Option A) anywhere in this bring-up sequence.
void commandWheel() {

  float tau = 0.0f;
  if (gPulseActive) {
    if (millis() - gPulseStartMs < kPulseDurationMs) {
      tau = gTauPulse;
    } else {
      gPulseActive = false;
      Serial.println("# PULSE END");
    }
  }

  // --- NaN guard before the clamp (Section 3 safety additions) ---
  // Every comparison against NaN is false, so a clamp lets a NaN straight
  // through if this check isn't first. tau can't actually be NaN from the
  // state machine above, but the pattern needs to be in place from the
  // first real torque command onward, not bolted on later.
  if (!isfinite(tau)) { tau = 0.0f; gPulseActive = false; }

  // --- hard clamp ---
  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  // --- torque mode: position loop and rate loop both disabled, command
  //     carried entirely by feedforward_torque ---
  Moteus::PositionMode::Command cmd;
  cmd.position              = NaN;
  cmd.velocity              = 0.0f;
  cmd.kp_scale              = 0.0f;
  cmd.kd_scale              = 0.0f;
  cmd.feedforward_torque    = kWheelSign * tau;
  cmd.maximum_torque        = kTauMax;   // hardware-side clamp, belt-and-suspenders
  cmd.watchdog_timeout      = 0.10f;     // s — if CAN stalls for 100 ms the servo
                                          //     enters its own Timeout state
                                          //     instead of holding stale torque
  cmd.ignore_position_bounds = 1.0f;     // a reaction wheel has no natural travel
                                          // limit. Without this, entering Position
                                          // mode while the current position is
                                          // outside servopos.position_min/_max
                                          // (moteus defaults assume a bounded
                                          // joint) raises FAULT 39 "outside limit"
                                          // and the servo stays faulted -- still
                                          // replying to queries (telemetry looks
                                          // fine) but applying zero torque forever
                                          // after. This is the leading suspect for
                                          // "manual spin reads correctly, commanded
                                          // torque does nothing."
  moteus1.SetPosition(cmd, &kTorqueFormat);

  gLastTau = tau;
}


// ----------------------------------------------------------------------------
// SECTION 2e: SERIAL COMMANDS
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) { return; }

  const char cmd = line.charAt(0);

  if (cmd == 'p') {
    if (gPulseActive) {
      Serial.println("# pulse already running, ignored");
    } else {
      gPulseActive  = true;
      gPulseStartMs = millis();
      Serial.print("# PULSE START: "); Serial.print(gTauPulse, 4);
      Serial.println(" N*m for 1000 ms");
    }
  } else if (cmd == 't') {
    const float val = line.substring(1).toFloat();
    gTauPulse = val >  kTauMax ?  kTauMax
              : val < -kTauMax ? -kTauMax : val;
    Serial.print("# gTauPulse = "); Serial.print(gTauPulse, 4); Serial.println(" N*m");
  } else if (cmd == 'o') {
    const float val = line.substring(1).toFloat();
    kThetaOffset = val * (float)DEG_TO_RAD;
    Serial.print("# kThetaOffset = "); Serial.print(val, 4); Serial.println(" deg");
  } else if (cmd == 'h') {
    const float val = line.substring(1).toFloat();
    gHalted = (val != 0.0f);
    if (gHalted && gPulseActive) {
      gPulseActive = false;
      Serial.println("# pulse cancelled by halt");
    }
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed)");
  } else {
    Serial.println("# unknown. use: p (fire pulse)  t<Nm> (pulse size)  "
                    "o<deg> (mount offset)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - STAGE 1: OPEN-LOOP TORQUE (panel held by hand)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  moteus1.SetStop();
  Serial.println("all stopped");

  pinMode(imuChipSelectPin, OUTPUT);
  digitalWrite(imuChipSelectPin, HIGH);
  SPI.begin();

  while (imu.beginSPI(imuChipSelectPin, imuClockFrequency) != BMI2_OK) {
    Serial.println("Error: BMI270 not connected, check wiring and CS pin!");
    delay(1000);
  }
  Serial.println("BMI270 connected!");

  if (imu.setAccelODR(BMI2_ACC_ODR_400HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_400HZ)  != BMI2_OK) {
    Serial.println("Warning: could not raise BMI270 ODR to 400 Hz");
  }

  Serial.println("t_ms\ttheta_deg\ttheta_dot_dps\ttau_cmd_Nm\tpulse_active\t"
                  "wheel_pos\twheel_vel\tmoteus_mode\tmoteus_fault\t"
                  "moteus_torque\tmoteus_qcurrent");
  Serial.println("# moteus_mode 1 = FAULT. moteus_fault 39 = outside position "
                  "bounds (see cmd.ignore_position_bounds), 36 = never calibrated.");
  Serial.println("# moteus_torque/qcurrent near 0 despite tau_cmd_Nm nonzero =");
  Serial.println("# torque still not reaching the current loop. Near tau_cmd_Nm =");
  Serial.println("# servo IS applying it -- look for a mechanical cause instead.");
  Serial.println("# send p to fire a pulse, t<Nm> to change its size (try bigger,");
  Serial.println("# e.g. t0.05 or t0.08, well under tau_cont=0.12)");
  Serial.println("# send h1 to halt (idle, no CAN/IMU traffic), h0 to resume");
  Serial.println("# HOLD THE PANEL FIRMLY BEFORE SENDING p");

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  handleSerialCommands();

  // Halted: skip IMU reads, CAN traffic, and telemetry entirely. Serial
  // commands above still run every pass, so "h0" always gets through.
  if (gHalted) { return; }

  if (static_cast<int32_t>(millis() - gNextSendMillis) < 0) { return; }
  gNextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  imu.getSensorData();

  static float theta = 0.0f;
  static float theta_dot = 0.0f;
  calculateState(readIMU(imu), theta, theta_dot);

  commandWheel();
  const auto& v = moteus1.last_result().values;

  printState(time, theta, theta_dot, gLastTau, gPulseActive, v);

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage2_DampingOnly/Stage2_DampingOnly.ino — first closed-loop term
// (rate/damping only), still hand-held. This is also where the latching
// safety trips and the wheel taper are introduced (Order of work step 5),
// before any position feedback exists that could reinforce a sign error.
// ============================================================================

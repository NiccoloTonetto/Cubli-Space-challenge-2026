// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 3: POSITION + DAMPING
// ============================================================================
// PANEL STILL HELD BY HAND. Adds k1 (position) on top of k2 (damping);
// k3 = 0 still. This is the first stage where a sign error CAN reinforce
// into a runaway, which is why gGainScale starts low and is ramped up
// manually between runs rather than jumping straight to full authority.
//
// From: Arduino Bring-Up Plan — Sections 2b and 2d, §2 "Stage 3".
//
// STAGE 3 CHECKLIST — panel held by hand, one run per gain step:
//   Send "a1" to arm, then step gGainScale with "g0.1", "g0.3", "g0.6",
//   "g1.0" -- ONE STEP PER RUN. Disarm ("a0") between steps if you want to
//   reset and re-brace.
//   [ ] At g=0.1: a gentle push back toward vertical
//   [ ] At g=1.0: distinctly stiff
//
// The wheel WILL spin up steadily without k3 -- that is expected. It is
// exactly why Stage 4 exists; do not chase it here.
//
// kMaxTilt is wide (40 deg default, live-settable with "m<deg>") and NOT
// tied to the Build Guide's recovery-envelope numbers the way it is in
// Stage 4/5 -- here it exists only so you can physically reach angles that
// actually produce enough torque to feel. At low gGainScale the position
// term alone doesn't clear real wheel stiction (~0.05 N*m, confirmed in
// Stage 1) until surprisingly large angles:
//     g=0.1 -> needs ~26 deg for tau to exceed stiction
//     g=0.3 -> needs ~9 deg
//     g=0.6 -> needs ~4 deg
//     g=1.0 -> needs ~3 deg
// The OLD 8 deg trip was cutting g=0.1 and g=0.3 off before they ever
// produced a torque big enough to notice -- not because the law was wrong.
// Note the wheel not visibly spinning and the panel not feeling a reaction
// are two different things: the motor's stator reaction hits the frame
// the instant current flows, whether or not the rotor overcomes friction
// and actually turns. If you feel a push but no wheel motion at low gain,
// that is consistent, not a fault.
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
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Software pause -- see SECTION 2e. Closing the Serial Monitor does NOT
// stop a Teensy sketch; loop() keeps running regardless, torque included.
// Halting here also force-disarms (gArmed = false) -- it's a strict
// superset of "a0", not an alternative to it: torque goes to zero AND the
// IMU/CAN/telemetry loop itself freezes. Resuming ("h0") does NOT
// re-arm -- you still need a fresh "a1", same as after any other trip.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION (unchanged)
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

// TODO: carry forward the value measured in Stage 0.
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

void printState(uint32_t t_ms, float theta, float theta_dot, float tau,
                float tauCmd, bool armed, float gainScale,
                const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(theta     * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(theta_dot * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(tauCmd, 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  Serial.print('\t'); Serial.print(v.position, 3);
  Serial.print('\t'); Serial.println(v.velocity, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — k1 (position) + k2 (damping), k3 still zero
// ----------------------------------------------------------------------------

static const float kK1 = -1.0998f;    // N*m / rad        -- ACTIVE
static const float kK2 = -0.1232f;    // N*m / (rad/s)     -- ACTIVE
static const float kK3 = -0.001732f;  // N*m / (rad/s)     -- NOT USED this stage

static bool  gArmed     = false;
static float gGainScale = 0.1f;    // START LOW. Ramp by hand: 0.1 -> 0.3 ->
                                     // 0.6 -> 1.0, one run per step.

// NOT const, and NOT the Stage 4/5 value: see the header comment. Default
// 40 deg -- wide enough that low-gGainScale steps can actually reach a
// commanded torque above real stiction before tripping. Live-settable
// with "m<deg>" if 40 still isn't enough, or you want it tighter.
static float kMaxTilt    = 0.6981f;   // 40 deg
static const float kMaxOmega   = 40.0f;
static const float kTauMax     = 0.12f;
static const float kTaperStart = 36.0f;

// Physical wheel-mounting sign convention -- CONFIRMED backwards on this
// hardware in Stage 1 (sign check 4: a positive pulse pushed the panel
// toward increasing |theta| instead of back toward vertical). Applied to
// BOTH the outgoing torque and the incoming wheel speed, together, so tau
// and wheel_omega stay mutually self-consistent (the taper's spinning_up
// check depends on that) while the panel-reaction direction gets
// corrected. K1/K2/K3 are untouched -- this is a hardware convention
// fix, not a control-law change. See Stage 1 for the full writeup.
static const float kWheelSign = -1.0f;

// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field (feedforward_torque, kp_scale, kd_scale,
// maximum_torque, watchdog_timeout) defaults to Resolution::kIgnore and is
// silently DROPPED before it reaches the CAN bus unless this Format
// explicitly turns each one on. Without it, everything set on `cmd` below
// is real in this C++ struct and never leaves the Teensy: the moteus falls
// back to its own onboard kp_scale=1/kd_scale=1 and just holds velocity=0
// -- plain velocity mode, not torque mode. Confirmed against
// mjbots/moteus-arduino's moteus_protocol.h.
static Moteus::PositionMode::Format kTorqueFormat = []() {
  Moteus::PositionMode::Format f;
  f.feedforward_torque = Moteus::kFloat;
  f.kp_scale            = Moteus::kFloat;
  f.kd_scale             = Moteus::kFloat;
  f.maximum_torque       = Moteus::kFloat;
  f.watchdog_timeout     = Moteus::kFloat;
  f.ignore_position_bounds = Moteus::kFloat;   // see cmd.ignore_position_bounds
  return f;
}();

static float gLastTau    = 0.0f;
static float gLastTauCmd = 0.0f;

void commandWheel(float theta, float theta_dot, float wheel_omega) {

  // k3 held at zero this stage -- position + damping only.
  float tau = -(kK1 * theta + kK2 * theta_dot) * gGainScale;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  if (fabsf(theta) > kMaxTilt)        { gArmed = false; }
  if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
  if (!isfinite(theta) || !isfinite(theta_dot)) { gArmed = false; }

  const float tau_cmd = gArmed ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position           = NaN;
  cmd.velocity           = 0.0f;
  cmd.kp_scale           = 0.0f;
  cmd.kd_scale           = 0.0f;
  cmd.feedforward_torque = kWheelSign * tau_cmd;
  cmd.maximum_torque     = kTauMax;
  cmd.watchdog_timeout   = 0.10f;
  cmd.ignore_position_bounds = 1.0f;  // a reaction wheel has no travel limit --
                                       // moteus defaults assume a bounded joint
                                       // and can fault (39, "outside limit")
                                       // entering Position mode outside
                                       // servopos.position_min/_max otherwise.
                                       // See Stage 1 for the full writeup.
  moteus1.SetPosition(cmd, &kTorqueFormat);

  gLastTau    = tau;
  gLastTauCmd = tau_cmd;
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
  const float val = line.substring(1).toFloat();

  if (cmd == 'a') {
    gArmed = (val != 0.0f);
    Serial.print("# gArmed = "); Serial.println(gArmed ? "TRUE" : "FALSE");
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
  } else if (cmd == 'o') {
    kThetaOffset = val * (float)DEG_TO_RAD;
    Serial.print("# kThetaOffset = "); Serial.print(val, 4); Serial.println(" deg");
  } else if (cmd == 'm') {
    kMaxTilt = val * (float)DEG_TO_RAD;
    Serial.print("# kMaxTilt = "); Serial.print(val, 2); Serial.println(" deg");
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
      Serial.println("# disarmed by halt");
    }
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed, still DISARMED -- send a1)");
  } else {
    Serial.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  m<deg>  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - STAGE 3: POSITION + DAMPING (panel held by hand)");

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

  Serial.println("t_ms\ttheta_deg\ttheta_dot_dps\ttau_Nm\ttau_cmd_Nm\tarmed\t"
                  "gain_scale\twheel_pos\twheel_vel");
  Serial.println("# STARTS DISARMED at gGainScale=0.1. Send a1 to arm.");
  Serial.println("# g<0..1> steps the gain, o<deg> sets mount offset.");
  Serial.println("# m<deg> sets the tilt trip (default 40 deg -- wide on");
  Serial.println("# purpose, see header comment for why).");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");

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

  // --- wheel speed from the previous cycle's moteus reply ---
  const auto& v = moteus1.last_result().values;
  const float wheel_omega = kWheelSign * v.velocity * 2.0f * (float)PI;   // rev/s -> rad/s

  // --- control + command ---
  commandWheel(theta, theta_dot, wheel_omega);

  // --- telemetry ---
  printState(time, theta, theta_dot, gLastTau, gLastTauCmd, gArmed,
             gGainScale, v);

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage4_FullLaw/Stage4_FullLaw.ino — adds k3. The wheel should now
// UNWIND after each correction instead of accumulating. Watch the standing
// wheel speed; the 180-deg flip test there separates a COM offset from a
// gyro bias in about fifteen seconds.
// ============================================================================

// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 5: RELEASE
// ============================================================================
// RAILS IN PLACE. E-STOP IN HAND. This is the first stage where the panel
// is NOT held by hand -- everything before this was rehearsal. The control
// law is identical to Stage 4; nothing new is being tested about the law
// itself, only about the full closed loop running unsupported.
//
// From: Arduino Bring-Up Plan — Sections 2b and 2d, §2 "Stage 5".
//
// Same trip logic as Stage 4, plus a latched trip-reason readout (tilt /
// omega / NaN) printed once at the moment it fires -- worth the extra
// visibility here specifically, since this is the one stage where nobody's
// hand is on the panel to feel a problem before the telemetry shows it.
//
// EXPECTED RESULT (Simscape Panel Model Build Guide, "Hardware predictions"):
//   Recoverable tilt   : 7-9 deg on hardware (11-12 deg was the ideal-sim
//                        figure; quantisation, noise, delay and friction
//                        all eat into it on real hardware)
//   Settling from 3 deg: ~0.75 s
//   Standing wheel speed (healthy): < 1 rad/s
//
// STAGE 5 PROCEDURE:
//   1. Panel resting against the rail, near vertical.
//   2. Send "a1" to arm.
//   3. Let go / give it a small push. Watch it recover, or watch it trip.
//   4. If it trips: check trip_reason in telemetry before re-arming. A
//      trip is not a bug to route around by disarming and re-arming
//      quickly -- read the number first.
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
// Deliberately does NOT touch gTripReason -- halting is an operator pause,
// not a trip, so if a real trip happened before you halted, that record
// stays visible until you actually re-arm.


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

// TODO: carry forward the value validated through Stages 0-4.
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

// trip_reason: 0 none, 1 tilt (|theta| > kMaxTilt), 2 omega (|wheel_omega| >
// kMaxOmega), 3 nan (non-finite theta/theta_dot/tau). Latched until the
// next "a1" — read it before re-arming, don't just re-arm and hope.
void printState(uint32_t t_ms, float theta, float theta_dot, float tau,
                float tauCmd, bool armed, int tripReason, float wheelOmegaLp,
                const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(theta     * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(theta_dot * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(tauCmd, 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(tripReason);
  Serial.print('\t'); Serial.print(wheelOmegaLp, 3);
  Serial.print('\t'); Serial.print(v.position, 3);
  Serial.print('\t'); Serial.println(v.velocity, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — full law: k1 + k2 + k3 (identical to Stage 4)
// ----------------------------------------------------------------------------

static const float kK1 = -1.0998f;    // N*m / rad
static const float kK2 = -0.1232f;    // N*m / (rad/s)
static const float kK3 = -0.001732f;  // N*m / (rad/s)

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 4

static const float kMaxTilt    = 0.14f;    // rad, 8 deg
static const float kMaxOmega   = 40.0f;    // rad/s
static const float kTauMax     = 0.12f;    // N*m
static const float kTaperStart = 36.0f;    // rad/s

// Physical wheel-mounting sign convention -- CONFIRMED backwards on this
// hardware in Stage 1 (sign check 4: a positive pulse pushed the panel
// toward increasing |theta| instead of back toward vertical). Applied to
// BOTH the outgoing torque and the incoming wheel speed, together, so tau
// and wheel_omega stay mutually self-consistent (the taper's spinning_up
// check AND the k3 momentum term both depend on that) while the panel-
// reaction direction gets corrected. K1/K2/K3 are untouched -- this is a
// hardware convention fix, not a control-law change. See Stage 1 for the
// full writeup.
static const float kWheelSign = -1.0f;

enum TripReason { TRIP_NONE = 0, TRIP_TILT = 1, TRIP_OMEGA = 2, TRIP_NAN = 3 };
static int gTripReason = TRIP_NONE;

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

static float gLastTau      = 0.0f;
static float gLastTauCmd   = 0.0f;
static float gWheelOmegaLp = 0.0f;   // standing wheel speed, tau = 5 s

void commandWheel(float theta, float theta_dot, float wheel_omega) {

  float tau = -(kK1 * theta + kK2 * theta_dot + kK3 * wheel_omega) * gGainScale;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  // --- latching trips: record WHY, print once, and never auto-unlatch ---
  if (gArmed && fabsf(theta) > kMaxTilt) {
    gArmed = false; gTripReason = TRIP_TILT;
    Serial.println("# TRIP: tilt limit");
  }
  if (gArmed && fabsf(wheel_omega) > kMaxOmega) {
    gArmed = false; gTripReason = TRIP_OMEGA;
    Serial.println("# TRIP: wheel speed limit");
  }
  if (gArmed && (!isfinite(theta) || !isfinite(theta_dot) || !isfinite(tau))) {
    gArmed = false; gTripReason = TRIP_NAN;
    Serial.println("# TRIP: non-finite state or torque");
  }

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

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);
  gWheelOmegaLp += alphaLp * (wheel_omega - gWheelOmegaLp);

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
    if (gArmed) { gTripReason = TRIP_NONE; }   // manual re-arm clears the latch
    Serial.print("# gArmed = "); Serial.println(gArmed ? "TRUE" : "FALSE");
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
  } else if (cmd == 'o') {
    kThetaOffset = val * (float)DEG_TO_RAD;
    Serial.print("# kThetaOffset = "); Serial.print(val, 4); Serial.println(" deg");
  } else if (cmd == 'r') {
    // Bookmark only -- no effect on control. Send this the instant you let
    // go of the panel at a held test angle; <deg> is just a label for the
    // log (what you INTENDED to release from), not fed into anything. The
    // telemetry row immediately following this line is your real t=0 --
    // this line just makes it easy to find in a long capture. For actual
    // time alignment in post-processing, prefer detecting the moment
    // theta_deg starts moving on its own in the data over trusting this
    // timestamp precisely -- human release timing has more jitter than
    // the control loop does.
    Serial.print("# RECORD_START t_ms="); Serial.print(millis());
    Serial.print(" theta_target_deg="); Serial.println(val, 2);
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
    Serial.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  r<deg> (log marker)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - STAGE 5: RELEASE (rails + e-stop, panel free)");

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
                  "trip_reason\twheel_omega_lp\twheel_pos\twheel_vel");
  Serial.println("# STARTS DISARMED. Rails + e-stop ready BEFORE sending a1.");
  Serial.println("# trip_reason: 0 none  1 tilt  2 omega  3 nan");
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
             gTripReason, gWheelOmegaLp, v);

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This is the end of the staged bring-up. From here, retuning follows
// Panel Controller Workflow's step 4: re-measure Theta from a free-swing
// test on THIS hardware, update the plant, re-run lqr(), and re-flash
// updated kK1/kK2/kK3 into this file.
// ============================================================================

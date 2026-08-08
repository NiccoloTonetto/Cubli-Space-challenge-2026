// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 2: DAMPING ONLY
// ============================================================================
// PANEL STILL HELD BY HAND. First closed-loop term: k2 (rate/damping) only,
// k1 = k3 = 0. No position feedback yet, so there is no term that can
// reinforce a sign error into a runaway — cheapest closed-loop confidence
// available.
//
// This file also introduces the full safety scaffold (latching trips +
// wheel taper, Bring-Up Plan §3 / Order of work step 5) — everything from
// here onward runs with it active.
//
// From: Arduino Bring-Up Plan — Sections 2b and 2d, §2 "Stage 2".
//
// STAGE 2 CHECKLIST (~20 min) — panel held by hand.
//   Send "a1" to arm (gGainScale fixed at 1.0 this stage — see doc).
//   [ ] Panel feels VISCOUS: resists rotation in either direction, with
//       no tendency to hold a position.
//   [ ] If it fights you, or feels like it's helping the fall, theta_dot's
//       sign is wrong — go back to Stage 0, do not proceed to Stage 3.
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
// SECTION 2b: STATE ESTIMATION (unchanged from Stage 0/1)
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

// Fixed-width columns instead of tabs. Tabs looked aligned in the source
// file but drift in a real terminal/Serial Monitor because tab-stop width
// isn't guaranteed and every field prints a different number of digits.
//
// Deliberately NOT Serial.printf("%...f", ...): Teensy 4's default
// toolchain links the nano-libc printf, which silently drops float
// support unless the IDE's Optimize setting is specifically not
// "Smallest Code" -- not something to depend on here. Serial.print(value,
// decimals) is already proven working throughout this file; pad() just
// left-justifies whatever it wrote, using the byte count Serial.print()
// returns. Left- rather than right-justified because Serial.print()
// writes straight to the stream -- there's no way to know a value's
// printed width and pad BEFORE it, only after.
//
// Trade-off worth knowing: this is no longer strict TSV. Redirecting to a
// file for pandas/numpy now needs sep=r'\s+' instead of sep='\t'.
void pad(size_t printed, uint8_t width) {
  for (size_t i = printed; i < width; i++) { Serial.print(' '); }
}

// Column widths -- shared by the header and every data row, so alignment
// holds regardless of value magnitude. Widened a bit past each label's own
// length to leave room for the numbers under it.
static const uint8_t kW_t     = 8;    // t_ms
static const uint8_t kW_theta = 11;   // theta_deg
static const uint8_t kW_thdot = 15;   // theta_dot_dps
static const uint8_t kW_tau   = 9;    // tau_Nm
static const uint8_t kW_tauc  = 12;   // tau_cmd_Nm
static const uint8_t kW_armed = 7;    // armed
static const uint8_t kW_gain  = 12;   // gain_scale
static const uint8_t kW_pos   = 11;   // wheel_pos

// Called once from setup(). NOT from printState() -- printing a header
// every call would mean one every 2 ms, which defeats the point.
void printHeader() {
  pad(Serial.print("t_ms"),          kW_t);
  pad(Serial.print("theta_deg"),     kW_theta);
  pad(Serial.print("theta_dot_dps"), kW_thdot);
  pad(Serial.print("tau_Nm"),        kW_tau);
  pad(Serial.print("tau_cmd_Nm"),    kW_tauc);
  pad(Serial.print("armed"),         kW_armed);
  pad(Serial.print("gain_scale"),    kW_gain);
  pad(Serial.print("wheel_pos"),     kW_pos);
  Serial.println("wheel_vel");
}

void printState(uint32_t t_ms, float theta, float theta_dot, float tau,
                float tauCmd, bool armed, float gainScale,
                const Moteus::Query::Result& v) {
  pad(Serial.print(t_ms),                             kW_t);
  pad(Serial.print(theta     * (float)RAD_TO_DEG, 2),  kW_theta);
  pad(Serial.print(theta_dot * (float)RAD_TO_DEG, 2),  kW_thdot);
  pad(Serial.print(tau, 4),                            kW_tau);
  pad(Serial.print(tauCmd, 4),                         kW_tauc);
  pad(Serial.print(armed ? 1 : 0),                     kW_armed);
  pad(Serial.print(gainScale, 2),                      kW_gain);
  pad(Serial.print(v.position, 3),                     kW_pos);
  Serial.println(v.velocity, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — k2 (damping) only
// ----------------------------------------------------------------------------

// Gains from LQR on the measured plant (Simscape Panel Model Build Guide,
// Gate 7). Closed-loop poles for the FULL law: approx -12.5, -9.7, -6.3
// rad/s (all real). Only k2 is active this stage.
static const float kK1 = -1.0998f;    // N*m / rad        -- NOT USED this stage
static const float kK2 = -0.1232f;    // N*m / (rad/s)     -- ACTIVE
static const float kK3 = -0.001732f;  // N*m / (rad/s)     -- NOT USED this stage

// ---- globals that gate everything (Bring-Up Plan §2.0) --------------------
static bool  gArmed     = false;   // false => torque forced to zero at output
static float gGainScale = 1.0f;    // 0..1 -- fixed at 1.0 this stage (doc: no
                                     // ramp needed for a single damping term)

static const float kMaxTilt    = 0.14f;    // rad, 8 deg -- trip
static const float kMaxOmega   = 40.0f;    // rad/s      -- trip
static const float kTauMax     = 0.12f;    // N*m        -- saturation
static const float kTaperStart = 36.0f;    // rad/s, 90 % of cap

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

static float gLastTau    = 0.0f;   // would-be torque, for telemetry
static float gLastTauCmd = 0.0f;   // actually-sent torque, for telemetry

// Runs the control law and issues the resulting command to the moteus.
void commandWheel(float theta, float theta_dot, float wheel_omega) {

  // k1 and k3 held at zero this stage -- damping only.
  float tau = -(kK2 * theta_dot) * gGainScale;

  // --- wheel taper (Bring-Up Plan §3): fade SPIN-UP torque only, as the
  //     wheel approaches the cap. Braking is always allowed unfaded -- it
  //     is the recovery action the taper exists to preserve. ---
  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  // --- NaN guard BEFORE the clamp: every comparison against NaN is false,
  //     so a clamp alone lets a NaN pass straight through. ---
  if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }

  // --- hard clamp ---
  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  // --- latching trips. Do NOT auto-unlatch -- re-arming is a deliberate
  //     human act (send "a1" again), otherwise a marginal rig trips
  //     repeatedly while you try to work out what happened. ---
  if (fabsf(theta) > kMaxTilt)        { gArmed = false; }
  if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
  if (!isfinite(theta) || !isfinite(theta_dot)) { gArmed = false; }

  // --- gate: zero the OUTPUT unless armed. tau above still reflects what
  //     the law wants, for telemetry. ---
  const float tau_cmd = gArmed ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position           = NaN;
  cmd.velocity           = 0.0f;
  cmd.kp_scale           = 0.0f;
  cmd.kd_scale           = 0.0f;
  cmd.feedforward_torque = kWheelSign * tau_cmd;
  cmd.maximum_torque     = kTauMax;   // hardware-side clamp, belt-and-suspenders
  cmd.watchdog_timeout   = 0.10f;     // s -- stale CAN link faults the servo
                                       //      instead of holding stale torque
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
    Serial.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - STAGE 2: DAMPING ONLY (panel held by hand)");

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

  printHeader();
  Serial.println("# STARTS DISARMED. Send a1 to arm, a0 to disarm.");
  Serial.println("# o<deg> sets mount offset. g<0..1> sets gain scale.");
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
// Next: Stage3_PositionDamping/Stage3_PositionDamping.ino — adds k1
// (position), still k3 = 0. Ramp gGainScale 0.1 -> 0.3 -> 0.6 -> 1.0, one
// step per run, still hand-held. The wheel WILL spin up steadily without
// k3 -- that's expected, and exactly why Stage 3 isn't the last one.
// ============================================================================

// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — STAGE 4: FULL LAW
// ============================================================================
// PANEL STILL HELD BY HAND. Adds k3 (wheel momentum management) on top of
// k1 + k2. The wheel should now UNWIND after each correction instead of
// spinning up steadily the way it did in Stage 3.
//
// From: Arduino Bring-Up Plan — Sections 2b and 2d, §2 "Stage 4".
//
// STAGE 4 CHECKLIST — panel held by hand, gGainScale = 1.0:
//   Send "a1" to arm.
//   [ ] Wheel unwinds after each correction (watch wheel_omega_lp below —
//       it's wheel_omega through a ~5 s low-pass, i.e. the STANDING speed,
//       not the instantaneous one).
//   [ ] wheel_omega_lp near zero -> healthy.
//   [ ] wheel_omega_lp a few rad/s -> gyro bias or a COM offset. Run the
//       180-DEG FLIP TEST to tell them apart: rotate the panel 180 deg
//       about the pivot axis and re-run.
//         - Standing speed REVERSES SIGN  -> COM offset (re-measure
//           kThetaOffset in Stage 0, or the physical mount).
//         - Standing speed DOES NOT reverse -> gyro bias (re-run the IMU
//           calibration, or accept it and watch it in Stage 5).
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <MoteusTeensy.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 1b: TELEMETRY MODE SELECTOR
// ----------------------------------------------------------------------------
// TELEMETRY_SERIAL: human-readable, tab-delimited, with header + "#" status
//                    lines -- for reading directly in the Arduino Serial
//                    Monitor.
// TELEMETRY_MATLAB:  plain CSV, one line per control cycle, no header/
//                     comment lines -- for
//                     firmware/2D model/Validation/telemetry_matlab.m's
//                     live scrolling plot + logging.
// Change the line below and re-upload to switch modes.
#define TELEMETRY_SERIAL 0
#define TELEMETRY_MATLAB 1
#define TELEMETRY_MODE TELEMETRY_SERIAL


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

// TODO: carry forward the value measured in Stage 0. This is also the
// value the 180-deg flip test checklist item above may send you back to.
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

// wheel_omega_lp: wheel_omega through a tau=5s low-pass -- the STANDING
// wheel speed the Stage 4/5 checklists ask you to watch, with the
// per-cycle noise/ripple averaged out.
void printState(uint32_t t_ms, float theta, float theta_dot, float tau,
                float tauCmd, bool armed, float gainScale, float wheelOmegaLp,
                const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(theta     * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(theta_dot * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(tauCmd, 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  Serial.print('\t'); Serial.print(wheelOmegaLp, 3);
  Serial.print('\t'); Serial.print(v.position, 3);
  Serial.print('\t'); Serial.println(v.velocity, 3);
}

// Same fields as printState() above, same order, but plain comma-separated
// with no header/comment lines -- one clean line per cycle for
// telemetry_matlab.m to parse with a simple split-on-comma.
void telemetryMatlab(uint32_t t_ms, float theta, float theta_dot, float tau,
                     float tauCmd, bool armed, float gainScale,
                     float wheelOmegaLp, const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print(','); Serial.print(theta     * (float)RAD_TO_DEG, 4);
  Serial.print(','); Serial.print(theta_dot * (float)RAD_TO_DEG, 4);
  Serial.print(','); Serial.print(tau, 6);
  Serial.print(','); Serial.print(tauCmd, 6);
  Serial.print(','); Serial.print(armed ? 1 : 0);
  Serial.print(','); Serial.print(gainScale, 4);
  Serial.print(','); Serial.print(wheelOmegaLp, 6);
  Serial.print(','); Serial.print(v.position, 6);
  Serial.print(','); Serial.println(v.velocity, 6);
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL — full law: k1 + k2 + k3
// ----------------------------------------------------------------------------

static const float kK1 = -1.0998f;    // N*m / rad
static const float kK2 = -0.1232f;    // N*m / (rad/s)
static const float kK3 = -0.001732f;  // N*m / (rad/s)

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated by the Stage 3 ramp

static const float kMaxTilt    = 0.52f;   // widened from 0.14 (8 deg) -- full
                                           // envelope test, confirmed working
static const float kMaxOmega   = 600.0f;  // widened from 40 (firmware policy
                                           // cap) -- see kTaperStart below,
                                           // still well under omega_max=883
static const float kTauMax     = 0.12f;
static const float kTaperStart = 36.0f;

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

  // full law -- all three terms active.
  float tau = -(kK1 * theta + kK2 * theta_dot + kK3 * wheel_omega) * gGainScale;

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

  // Standing wheel speed: low-pass, tau = 5 s. Nominal dt (kPeriodMs) is
  // fine here -- this filter only needs to be accurate to within seconds,
  // not milliseconds.
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

  // Command handling itself always runs, in both telemetry modes -- only
  // the "#" status echoes below are Serial-mode only, so a MATLAB-mode
  // stream stays clean CSV even while you arm/disarm from the terminal.
  if (cmd == 'a') {
    gArmed = (val != 0.0f);
#if TELEMETRY_MODE == TELEMETRY_SERIAL
    Serial.print("# gArmed = "); Serial.println(gArmed ? "TRUE" : "FALSE");
#endif
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
#if TELEMETRY_MODE == TELEMETRY_SERIAL
    Serial.print("# gGainScale = "); Serial.println(gGainScale, 3);
#endif
  } else if (cmd == 'o') {
    kThetaOffset = val * (float)DEG_TO_RAD;
#if TELEMETRY_MODE == TELEMETRY_SERIAL
    Serial.print("# kThetaOffset = "); Serial.print(val, 4); Serial.println(" deg");
#endif
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
#if TELEMETRY_MODE == TELEMETRY_SERIAL
      Serial.println("# disarmed by halt");
#endif
    }
#if TELEMETRY_MODE == TELEMETRY_SERIAL
    Serial.print("# gHalted = ");
    Serial.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                            : "FALSE (resumed, still DISARMED -- send a1)");
#endif
#if TELEMETRY_MODE == TELEMETRY_SERIAL
  } else {
    Serial.println("# unknown. use: a<0/1>  g<0..1>  o<deg>  h<0/1>");
#endif
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - STAGE 4: FULL LAW (panel held by hand)");

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

#if TELEMETRY_MODE == TELEMETRY_SERIAL
  Serial.println("t_ms\ttheta_deg\ttheta_dot_dps\ttau_Nm\ttau_cmd_Nm\tarmed\t"
                  "gain_scale\twheel_omega_lp\twheel_pos\twheel_vel");
  Serial.println("# STARTS DISARMED. Send a1 to arm. o<deg> sets mount offset.");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
#endif

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
#if TELEMETRY_MODE == TELEMETRY_MATLAB
  telemetryMatlab(time, theta, theta_dot, gLastTau, gLastTauCmd, gArmed,
                  gGainScale, gWheelOmegaLp, v);
#else
  printState(time, theta, theta_dot, gLastTau, gLastTauCmd, gArmed,
             gGainScale, gWheelOmegaLp, v);
#endif

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage5_Release/Stage5_Release.ino -- same control law, panel let go
// with rails and an e-stop in place. Expect recovery from 7-9 deg (below
// the 11-12 deg ideal in the Build Guide -- quantisation, noise, delay and
// friction all consume margin on real hardware).
// ============================================================================

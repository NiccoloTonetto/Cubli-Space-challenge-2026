// ============================================================================
// TEENSY 4.1 + 3x moteus-n1 (CAN3) + BMI270 IMU (SPI) — CORNER STAGE 4:
// FULL LAW -- ARCHITECTURE SKELETON, CONTROL LAW LEFT BLANK
// ============================================================================
// Sibling of edge-bringup/Stage4_FullLaw.ino, generalized from ONE wheel
// (edge, single K[3] gain vector) to THREE wheels (corner, cube balanced on
// the [1,1,1] vertex, id 2 -> X, id 3 -> Y, id 1 -> Z -- same mapping as
// Skeleton_3Axis.ino and cube-bringup/Stage0d_Simultaneous.ino).
//
// WHAT IS WIRED AND LIVE:
//   - CAN/IMU boilerplate, telemetry-mode selector, serial-command interface
//     (same pattern as every other stage in this repo).
//   - SECTION 2b, STATE ESTIMATION: the subprograms that turn raw BMI270
//     samples into a body-frame acceleration/rate the control law can use --
//     readIMURaw() (sensor-frame calibration), rotateToBodyFrame() (mount
//     DCM), calibrateGyroBias(), updateBodyFrameState() (per-cycle driver).
//     This is IMU-to-control plumbing only, no attitude filter.
//   - SECTION 2d, DEFAULT CONTROL: a plain proportional controller, torque
//     proportional to the body-frame accelerometer error against the
//     corner-equilibrium reference (kARef[3]). No rate term, no wheel-
//     momentum term, no LQR -- see the TODO block right above
//     commandWheelsDefault() for what belongs there instead once ready.
//
// WHAT IS DELIBERATELY LEFT BLANK:
//   - The real corner control law (LQR / full state feedback over
//     g_hat/w_b/wheel-momentum, mirroring edge-bringup Stage4's
//     tau = -(K[0]*phi + K[1]*om + K[2]*wheel_omega)) is NOT implemented.
//     commandWheels() below calls commandWheelsDefault() (the P controller)
//     -- swap that call for the real law when it's ready, do not delete the
//     default, it stays useful as a low-authority sanity check.
//
// CUBE HELD BY HAND until the default P controller has been bench-verified
// (correct sign per wheel, sane torque at a few degrees of hand-tilt) --
// same staged-bringup discipline as every other Stage4 in this repo.
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

#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE SERIALMONITORMODE


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

ACAN_T4FD_Settings canSettings(1000000, DataBitRateFactor::x1);
MoteusTeensyCanFD canBus(ACAN_T4::can3, canSettings);

// Confirmed on bench, physically verified per wheel via
// cube-bringup/Stage0_SingleMoteusQuery: id 2 -> X, id 3 -> Y, id 1 -> Z.
Moteus moteusX(canBus, []() { Moteus::Options o; o.id = 2; return o; }());
Moteus moteusY(canBus, []() { Moteus::Options o; o.id = 3; return o; }());
Moteus moteusZ(canBus, []() { Moteus::Options o; o.id = 1; return o; }());
Moteus* const gWheels[3] = { &moteusX, &moteusY, &moteusZ };
static const char* kWheelName[3] = { "X", "Y", "Z" };

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- IMU-to-control plumbing (no attitude
// filter here on purpose, see file header: the default controller below
// consumes aBody directly).
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: recalibrate for this rig's actual IMU mount (run
// firmware/3D model/IMU_Calibration/IMU_Calibration.ino), same as
// Skeleton_3Axis.ino -- these are placeholder values.
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

// Mounting sequence intrinsic Z->X->Z, same closed-form as
// Skeleton_3Axis.ino / edge-bringup Stage4 -- do not multiply elementary
// matrices at runtime, checked instead by checkMountingDCMValid().
float theta1_deg = -30.0f;
float theta2_deg = 54.74f;
float theta3_deg = 45.0f;
float gMountDCM[3][3];

void updateMountingDCM() {
  const float k  = (float)DEG_TO_RAD;
  const float c1 = cosf(theta1_deg * k), s1 = sinf(theta1_deg * k);
  const float c2 = cosf(theta2_deg * k), s2 = sinf(theta2_deg * k);
  const float c3 = cosf(theta3_deg * k), s3 = sinf(theta3_deg * k);
  gMountDCM[0][0] =  c1*c3 - c2*s1*s3;
  gMountDCM[0][1] =  c3*s1 + c1*c2*s3;
  gMountDCM[0][2] =  s2*s3;
  gMountDCM[1][0] = -c1*s3 - c2*c3*s1;
  gMountDCM[1][1] =  c1*c2*c3 - s1*s3;
  gMountDCM[1][2] =  c3*s2;
  gMountDCM[2][0] =  s1*s2;
  gMountDCM[2][1] = -c1*s2;
  gMountDCM[2][2] =  c2;
}

void checkMountingDCMValid() {
  const float (&C)[3][3] = gMountDCM;
  const float det = C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1])
                   - C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0])
                   + C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
  Serial.print("# mount DCM check: det="); Serial.print(det, 4);
  for (int i = 0; i < 3; ++i) {
    const float len = sqrtf(C[i][0]*C[i][0] + C[i][1]*C[i][1] + C[i][2]*C[i][2]);
    Serial.print("  |row"); Serial.print(i); Serial.print("|="); Serial.print(len, 4);
  }
  Serial.println();
}

// Sensor-frame unit conversion + calibration, done once at the driver
// boundary -- rotation to body frame happens separately, below.
void readIMURaw(BMI270& sensor, float aImu[3], float wImu[3]) {
  aImu[0] = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  aImu[1] = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  aImu[2] = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  wImu[0] = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  wImu[1] = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  wImu[2] = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
}

// v_body = C * v_imu, applied to both accel and gyro.
void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

static float gGyroBiasBody[3] = { 0.0f, 0.0f, 0.0f };

// Residual body-frame gyro bias, measured fresh at each power-on -- call
// from setup(), after updateMountingDCM() and IMU init, cube stationary.
void calibrateGyroBias(uint16_t n_samples) {
  float sum[3] = { 0.0f, 0.0f, 0.0f };
  for (uint16_t i = 0; i < n_samples; ++i) {
    imu.getSensorData();
    float aImu[3], wImu[3], aBody[3], wBody[3];
    readIMURaw(imu, aImu, wImu);
    rotateToBodyFrame(aImu, wImu, aBody, wBody);
    sum[0] += wBody[0]; sum[1] += wBody[1]; sum[2] += wBody[2];
    delay(kPeriodMs);
  }
  gGyroBiasBody[0] = sum[0] / n_samples;
  gGyroBiasBody[1] = sum[1] / n_samples;
  gGyroBiasBody[2] = sum[2] / n_samples;
}

static inline float norm3(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Live state -- the values every downstream consumer (telemetry AND the
// default controller) reads. Updated once per cycle by updateBodyFrameState().
float aBody[3] = { 0.0f, 0.0f, 0.0f };   // m/s^2, body frame, calibrated
float wBody[3] = { 0.0f, 0.0f, 0.0f };   // rad/s, body frame, bias-corrected
float dt_s     = 0.0f;                    // last dt, telemetry only

// This is the IMU<->control link: raw sample in, aBody/wBody out. No
// attitude filter -- if/when the real control law needs a filtered gravity
// direction (g_hat) instead of raw aBody, add the complementary-filter step
// here (see Skeleton_3Axis.ino's attitudeUpdate() for that pattern) rather
// than inside the controller itself.
void updateBodyFrameState(float dt) {
  dt_s = dt;
  float aImu[3], wImuRaw[3];
  readIMURaw(imu, aImu, wImuRaw);
  float wImu[3];
  rotateToBodyFrame(aImu, wImuRaw, aBody, wImu);
  wBody[0] = wImu[0] - gGyroBiasBody[0];
  wBody[1] = wImu[1] - gGyroBiasBody[1];
  wBody[2] = wImu[2] - gGyroBiasBody[2];
}

// Corner-equilibrium reference: at the balanced [1,1,1] vertex the
// accelerometer's body-frame reading should sit at kG0 along the corner
// diagonal (same G_CORNER direction as Skeleton_3Axis.ino's BalanceMode).
static const float kCornerUnit[3] = { 0.57735027f, 0.57735027f, 0.57735027f };
static const float kARef[3] = { kG0 * kCornerUnit[0], kG0 * kCornerUnit[1], kG0 * kCornerUnit[2] };


// ----------------------------------------------------------------------------
// SECTION 2c: TELEMETRY
// ----------------------------------------------------------------------------

void printState(uint32_t t_ms, const float tauCmd[3], const float wheelVel[3],
                bool armed, float gainScale) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(aBody[0], 3);
  Serial.print('\t'); Serial.print(aBody[1], 3);
  Serial.print('\t'); Serial.print(aBody[2], 3);
  Serial.print('\t'); Serial.print(wBody[0] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(wBody[1] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(wBody[2] * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tauCmd[0], 4);
  Serial.print('\t'); Serial.print(tauCmd[1], 4);
  Serial.print('\t'); Serial.print(tauCmd[2], 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  Serial.print('\t'); Serial.print(wheelVel[0], 3);
  Serial.print('\t'); Serial.print(wheelVel[1], 3);
  Serial.print('\t'); Serial.println(wheelVel[2], 3);
}

void telemetryPlot(uint32_t t_ms, const float tauCmd[3], const float wheelVel[3],
                   bool armed, float gainScale) {
  Serial.print(t_ms);
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(aBody[i], 6); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(wBody[i] * (float)RAD_TO_DEG, 4); }
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(tauCmd[i], 6); }
  Serial.print(','); Serial.print(armed ? 1 : 0);
  Serial.print(','); Serial.print(gainScale, 4);
  for (int i = 0; i < 3; ++i) { Serial.print(','); Serial.print(wheelVel[i], 6); }
  Serial.println();
}


// ----------------------------------------------------------------------------
// SECTION 2d: CONTROL
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 0.1f;   // START LOW, same convention as every
                                    // other Stage4 -- ramp by hand ('g0.3',
                                    // 'g0.6', 'g1') once sign/scale look sane.

static const float kTauMax   = 0.12f;      // N*m, TAU_MAX placeholder -- re-derive
static const float kMaxAccelErr = 6.0f;    // m/s^2, disarm trip on the default P term

// TODO: per-wheel sign, NOT calibrated here -- verify each axis independently
// (Phase 1.3 isolated-pulse method, same as edge-bringup Stage1) before
// trusting this in closed loop.
static const float kWheelSign[3] = { 1.0f, 1.0f, 1.0f };

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

void sendWheelTorque(Moteus& wheel, float tau) {
  Moteus::PositionMode::Command cmd;
  cmd.position               = NaN;
  cmd.velocity               = 0.0f;
  cmd.kp_scale                = 0.0f;
  cmd.kd_scale                 = 0.0f;
  cmd.feedforward_torque       = tau;
  cmd.maximum_torque           = kTauMax;
  cmd.watchdog_timeout          = 0.10f;
  cmd.ignore_position_bounds    = 1.0f;
  wheel.SetPosition(cmd, &kTorqueFormat);
}

// ----------------------------------------------------------------------------
// REAL CONTROL LAW -- INTENTIONALLY BLANK.
// ----------------------------------------------------------------------------
// TODO: state-feedback / LQR corner control, mirroring edge-bringup
// Stage4's commandWheel() (tau = -(K_phi*phi + K_om*om + K_rho*wheel_omega),
// gain-scaled, friction feedforward, arm gate, taper/saturate/trip),
// generalized to 3 axes and a 9-ish-state gain matrix (tilt x2, rate x2/x3,
// wheel momentum x3). Needs a filtered attitude estimate (g_hat, see
// Skeleton_3Axis.ino's attitudeUpdate()) as input, which this file does not
// compute -- add that alongside this function when it's implemented, do not
// wire an LQR gain matrix to raw aBody.
//
// void commandWheelsLQR(const float wheelOmega[3], float tauCmd[3]) { ... }
// ----------------------------------------------------------------------------

// DEFAULT CONTROLLER -- plain proportional, driven directly by body-frame
// acceleration error against the corner reference. No rate/momentum term.
// This is a bench sanity-check law (correct sign? sane torque at a few deg
// of hand-tilt?), not the real corner control law -- replace the call in
// commandWheels() with commandWheelsLQR() once that exists.
static float kKp = 0.01f;   // N*m / (m/s^2) of accel error -- TODO tune live via 'p<val>'

void commandWheelsDefault(float tauCmd[3]) {
  const float err[3] = {
    aBody[0] - kARef[0],
    aBody[1] - kARef[1],
    aBody[2] - kARef[2],
  };

  for (int i = 0; i < 3; ++i) {
    float tau = -kKp * err[i] * gGainScale;
    tau = tau >  kTauMax ?  kTauMax : tau;
    tau = tau < -kTauMax ? -kTauMax : tau;
    tauCmd[i] = tau;
  }

  if (norm3(err) > kMaxAccelErr)                 { gArmed = false; }
  if (!isfinite(err[0]) || !isfinite(err[1]) || !isfinite(err[2])) { gArmed = false; }
}

void commandWheels(const float wheelOmega[3], float tauCmd[3]) {
  (void)wheelOmega;   // unused by the default P controller -- consumed once
                        // the real (LQR / momentum-aware) law replaces this call.
  commandWheelsDefault(tauCmd);
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
  } else if (cmd == 'p') {
    kKp = val < 0.0f ? 0.0f : val;
    Serial.print("# kKp = "); Serial.println(kKp, 5);
#if TELEMETRY_MODE == SERIALMONITORMODE
  } else {
    Serial.println("# unknown. use: a<0/1>  g<0..1>  p<kp>");
#endif
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - CORNER STAGE 4: FULL LAW skeleton (default P controller, LQR not implemented)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  for (int i = 0; i < 3; ++i) { gWheels[i]->SetStop(); }
  Serial.println("all stopped");

  updateMountingDCM();
  checkMountingDCMValid();

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

  Serial.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);

#if TELEMETRY_MODE == SERIALMONITORMODE
  Serial.println("t_ms\taccX\taccY\taccZ\tgyrX_dps\tgyrY_dps\tgyrZ_dps\t"
                  "tauX_Nm\ttauY_Nm\ttauZ_Nm\tarmed\tgain_scale\tvelX_rps\tvelY_rps\tvelZ_rps");
  Serial.println("# STARTS DISARMED. Send a1 to arm.");
  Serial.println("# DEFAULT controller only -- plain P on body-frame accel error, no LQR.");
#endif

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();

  if (static_cast<int32_t>(millis() - gNextSendMillis) < 0) { return; }
  gNextSendMillis += kPeriodMs;

  const uint32_t time = millis();

  static uint32_t lastMicros = 0;
  static bool dtInitialized = false;
  const uint32_t nowMicros = micros();
  float dt = dtInitialized ? (nowMicros - lastMicros) * 1e-6f : kPeriodMs * 1e-3f;
  lastMicros = nowMicros;
  dtInitialized = true;
  if (dt <= 0.0f || dt > 0.5f) { dt = kPeriodMs * 1e-3f; }

  imu.getSensorData();
  updateBodyFrameState(dt);

  float wheelVel[3];
  float wheelOmega[3];
  for (int i = 0; i < 3; ++i) {
    const auto& v = gWheels[i]->last_result().values;
    wheelVel[i] = v.velocity;
    wheelOmega[i] = kWheelSign[i] * v.velocity * 2.0f * (float)PI;
  }

  float tauCmd[3];
  commandWheels(wheelOmega, tauCmd);

  for (int i = 0; i < 3; ++i) {
    const float tau_cmd = gArmed ? tauCmd[i] : 0.0f;
    sendWheelTorque(*gWheels[i], kWheelSign[i] * tau_cmd);
  }

#if TELEMETRY_MODE == PLOTMODE
  telemetryPlot(time, tauCmd, wheelVel, gArmed, gGainScale);
#else
  printState(time, tauCmd, wheelVel, gArmed, gGainScale);
#endif
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Estimator plumbing (readIMURaw/rotateToBodyFrame/calibrateGyroBias/
// updateBodyFrameState -> aBody/wBody) is wired and live. The DEFAULT
// controller (commandWheelsDefault(), plain P on accel error) is wired and
// live for bench sanity-checking. The REAL corner control law is NOT
// implemented -- before trusting this past a hand-held bench check:
//   1. Recalibrate kGyroBias/kAccelOffset/kAccelScale for this rig's mount.
//   2. Confirm theta1_deg/theta2_deg/theta3_deg against the built mount.
//   3. Calibrate kWheelSign[3] per axis (isolated-pulse method).
//   4. Replace commandWheelsDefault() with the real law (state feedback /
//      LQR over a filtered g_hat, wheel momentum management, friction
//      feedforward, arm gate) -- see the blank block in SECTION 2d.
// ============================================================================

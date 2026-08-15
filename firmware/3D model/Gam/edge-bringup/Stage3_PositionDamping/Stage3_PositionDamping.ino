// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — EDGE STAGE 3:
// POSITION (phi_edge) + RATE -- AXIS-SELECTABLE
// ============================================================================
// >>> SET kAxis BELOW TO MATCH THE WHEEL YOU JUST RAN STAGE 1/2 ON. <<<
//
// CUBE STILL HELD BY HAND. Adds K[0] (phi_edge / position) on top of K[1]
// (rate); K[2] = 0 still. This is the first stage where a sign error CAN
// reinforce into a runaway, which is why gGainScale starts low and is
// ramped up manually between runs rather than jumping to full authority --
// same reasoning as the panel's Stage 3.
//
// STAGE 3 CHECKLIST — cube held by hand, one run per gain step:
//   Send "a1" to arm, then step gGainScale with "g0.1", "g0.3", "g0.6",
//   "g1.0" -- ONE STEP PER RUN. Disarm ("a0") between steps to reset/re-brace.
//   [ ] At g=0.1: a gentle push back toward the edge equilibrium
//   [ ] At g=1.0: distinctly stiff
//
// The wheel WILL spin up steadily without K[2] -- expected, exactly why
// Stage 4 exists; do not chase it here.
//
// kMaxTilt is wide (live-settable with "m<deg>") and NOT the Stage 4/5
// value -- same reasoning as the panel's Stage 3: at low gGainScale the
// position term alone doesn't clear real wheel stiction until larger
// angles, and the tight trip would cut those steps off before they ever
// produce a torque big enough to notice.
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

// Confirmed on bench, physically verified (Gam/Skeleton_3Axis.ino's mapping
// comment): id 2 -> X, id 3 -> Y, id 1 -> Z.
enum Axis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
static const Axis kAxis = AXIS_Y;   // <<< CHANGE THIS to test X or Z

static const int8_t kAxisMoteusId[3] = { 2, 3, 1 };
static const char*  kAxisName[3]     = { "X", "Y", "Z" };

Moteus moteusActive(canBus, []() {
  Moteus::Options options;
  options.id = kAxisMoteusId[kAxis];
  return options;
}());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1/2)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

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

void readIMURaw(BMI270& sensor, float aImu[3], float wImu[3]) {
  aImu[0] = (sensor.data.accelX * kG0 - kAccelOffset[0]) / kAccelScale[0];
  aImu[1] = (sensor.data.accelY * kG0 - kAccelOffset[1]) / kAccelScale[1];
  aImu[2] = (sensor.data.accelZ * kG0 - kAccelOffset[2]) / kAccelScale[2];
  wImu[0] = sensor.data.gyroX * (float)DEG_TO_RAD - kGyroBias[0];
  wImu[1] = sensor.data.gyroY * (float)DEG_TO_RAD - kGyroBias[1];
  wImu[2] = sensor.data.gyroZ * (float)DEG_TO_RAD - kGyroBias[2];
}

void rotateToBodyFrame(const float aImu[3], const float wImu[3],
                       float aBody[3], float wBody[3]) {
  for (int i = 0; i < 3; ++i) {
    wBody[i] = gMountDCM[i][0]*wImu[0] + gMountDCM[i][1]*wImu[1] + gMountDCM[i][2]*wImu[2];
    aBody[i] = gMountDCM[i][0]*aImu[0] + gMountDCM[i][1]*aImu[1] + gMountDCM[i][2]*aImu[2];
  }
}

static inline float norm3(const float v[3]) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
static inline void normalize3(float v[3]) {
  const float n = norm3(v);
  if (n > 1e-9f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}
static inline void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}
static inline float dot3(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float ghat[3] = { 0.0f, 0.0f, 1.0f };
float w_b[3]  = { 0.0f, 0.0f, 0.0f };
static float gGyroBiasBody[3] = { 0.0f, 0.0f, 0.0f };

void calibrateGyroBias(uint16_t n_samples) {
  float sum[3] = { 0.0f, 0.0f, 0.0f };
  for (uint16_t i = 0; i < n_samples; ++i) {
    imu.getSensorData();
    float aImu[3], wImu[3], aBody[3], wBody[3];
    readIMURaw(imu, aImu, wImu);
    rotateToBodyFrame(aImu, wImu, aBody, wBody);
    sum[0] += wBody[0]; sum[1] += wBody[1]; sum[2] += wBody[2];
    delay(2);
  }
  gGyroBiasBody[0] = sum[0]/n_samples;
  gGyroBiasBody[1] = sum[1]/n_samples;
  gGyroBiasBody[2] = sum[2]/n_samples;
}

static const float kP_FILT = 4.0f;
static const float kI_FILT = 0.5f;
static float bhat[3] = { 0.0f, 0.0f, 0.0f };
static bool gEstimatorInit = false;

void attitudeUpdate(const float a_imu[3], const float w_imu[3], float dt) {
  float aBody[3], wBodyRaw[3];
  rotateToBodyFrame(a_imu, w_imu, aBody, wBodyRaw);
  const float wRaw[3] = {
    wBodyRaw[0] - gGyroBiasBody[0],
    wBodyRaw[1] - gGyroBiasBody[1],
    wBodyRaw[2] - gGyroBiasBody[2],
  };

  float ga[3] = { aBody[0], aBody[1], aBody[2] };
  normalize3(ga);

  if (!gEstimatorInit) {
    ghat[0] = ga[0]; ghat[1] = ga[1]; ghat[2] = ga[2];
    gEstimatorInit = true;
    return;
  }

  float e[3];
  cross3(ghat, ga, e);
  const float wc[3] = { wRaw[0]-bhat[0], wRaw[1]-bhat[1], wRaw[2]-bhat[2] };
  float wxg[3]; cross3(wc, ghat, wxg);
  float exg[3]; cross3(e, ghat, exg);
  for (int i = 0; i < 3; ++i) { ghat[i] += dt * (-wxg[i] + kP_FILT * exg[i]); }
  normalize3(ghat);
  for (int i = 0; i < 3; ++i) { bhat[i] += kI_FILT * e[i] * dt; }
  for (int i = 0; i < 3; ++i) { w_b[i] = wRaw[i] - bhat[i]; }

  static int badCount = 0;
  if (dot3(ghat, ga) < 0.0f) {
    badCount++;
    if (badCount > 20) {
      ghat[0] = ga[0]; ghat[1] = ga[1]; ghat[2] = ga[2];
      bhat[0] = bhat[1] = bhat[2] = 0.0f;
      badCount = 0;
    }
  } else {
    badCount = 0;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2c: EDGE CANDIDATE (resolved fresh here too, per axis)
// ----------------------------------------------------------------------------

struct EdgeCandidate {
  const char* name;
  float e[3];
  float gB[3];
  float K[3];
  float placeOffsetDeg;
};

static const EdgeCandidate kCandidates[3][2] = {
  // AXIS_X (id 2) -- X[+1,+1] and X[-1,-1], offsets 0.758/0.837 deg
  { { "X[+1,+1] (+Y,+Z up)", { 1.0f, 0.0f, 0.0f }, { -0.0f, 0.697691619f, 0.716398239f },
      { -9.22453213f, -1.09680569f, -0.00692820316f }, 0.758f },
    { "X[-1,-1] (-Y,-Z up)", { 1.0f, 0.0f, 0.0f }, { -0.0f, -0.717363894f, -0.696698666f },
      { -8.2052536f, -0.948087275f, -0.00692820316f }, 0.837f } },
  // AXIS_Y (id 3) -- Y[+1,+1] and Y[-1,-1], offsets 0.006/0.007 deg (best on the cube)
  { { "Y[+1,+1] (+X,+Z up)", { 0.0f, 1.0f, 0.0f }, { 0.707182407f, -0.0f, 0.70703119f },
      { -9.33804893f, -1.10871983f, -0.00692820316f }, 0.006f },
    { "Y[-1,-1] (-X,-Z up)", { 0.0f, 1.0f, 0.0f }, { -0.707020879f, -0.0f, -0.707192659f },
      { -8.03074265f, -0.918068051f, -0.00692820316f }, 0.007f } },
  // AXIS_Z (id 1) -- Z[+1,+1] and Z[-1,-1], offsets 0.764/0.844 deg
  { { "Z[+1,+1] (+X,+Y up)", { 0.0f, 0.0f, 1.0f }, { 0.716472805f, 0.697615027f, -0.0f },
      { -9.22558498f, -1.09693909f, -0.00692820316f }, 0.764f },
    { "Z[-1,-1] (-X,-Y up)", { 0.0f, 0.0f, 1.0f }, { -0.696611583f, -0.717448473f, -0.0f },
      { -8.20397282f, -0.947880447f, -0.00692820316f }, 0.844f } },
};

int gEdgeIdx = 0;

void resolveEdgeCandidate() {
  const float d0 = dot3(ghat, kCandidates[kAxis][0].gB);
  const float d1 = dot3(ghat, kCandidates[kAxis][1].gB);
  gEdgeIdx = (d0 >= d1) ? 0 : 1;
  Serial.print("# axis="); Serial.print(kAxisName[kAxis]);
  Serial.print("  edge candidate resolved: "); Serial.print(kCandidates[kAxis][gEdgeIdx].name);
  Serial.print("  K_phi="); Serial.print(kCandidates[kAxis][gEdgeIdx].K[0], 5);
  Serial.print("  K_om="); Serial.print(kCandidates[kAxis][gEdgeIdx].K[1], 5);
  Serial.print("  (dot0="); Serial.print(d0, 4);
  Serial.print(" dot1="); Serial.print(d1, 4); Serial.println(")");
  if (fabsf(d0 - d1) < 0.2f) {
    Serial.println("# WARNING: dot0 and dot1 are close -- verify before arming.");
  }
}

float phi_edge = 0.0f;
float om_edge  = 0.0f;

void updateEdgeProjection() {
  const EdgeCandidate& c = kCandidates[kAxis][gEdgeIdx];
  float phi[3];
  { float t[3]; cross3(c.gB, ghat, t); phi[0]=-t[0]; phi[1]=-t[1]; phi[2]=-t[2]; }
  phi_edge = dot3(c.e, phi);
  om_edge  = dot3(c.e, w_b);
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------

void printState(uint32_t t_ms, float tau, float tauCmd, bool armed,
                float gainScale, const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(phi_edge * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(om_edge  * (float)RAD_TO_DEG, 2);
  Serial.print('\t'); Serial.print(tau, 4);
  Serial.print('\t'); Serial.print(tauCmd, 4);
  Serial.print('\t'); Serial.print(armed ? 1 : 0);
  Serial.print('\t'); Serial.print(gainScale, 2);
  Serial.print('\t'); Serial.print(v.position, 3);
  Serial.print('\t'); Serial.println(v.velocity, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — K_phi (position) + K_om (rate), K_rho still zero
// ----------------------------------------------------------------------------

static bool  gArmed     = false;
static float gGainScale = 0.1f;   // START LOW. Ramp by hand: 0.1 -> 0.3 -> 0.6 -> 1.0.

// NOT const, live-settable with "m<deg>" -- same reasoning as the panel's
// Stage 3: wide enough that low-gGainScale steps can reach a commanded
// torque above real stiction before tripping. NOT the Stage 4/5 value.
static float kMaxTilt    = 0.6981f;   // 40 deg
static const float kMaxOmega   = 40.0f;
static const float kTauMax     = 0.12f;
static const float kTaperStart = 36.0f;

// Per-axis: Y confirmed via Stage 1's pulse test, X/Z still placeholders
// pending their own Stage 1 run -- do NOT assume one wheel's sign for
// another (Firmware Lessons S4).
static const float kAxisWheelSign[3] = {
  1.0f,   // X -- PLACEHOLDER
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- PLACEHOLDER
};
static const float kWheelSign = kAxisWheelSign[kAxis];

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

static float gLastTau    = 0.0f;
static float gLastTauCmd = 0.0f;

void commandWheel(float wheel_omega) {
  // K[2] held at zero this stage -- position + rate only.
  const EdgeCandidate& c = kCandidates[kAxis][gEdgeIdx];
  float tau = -(c.K[0] * phi_edge + c.K[1] * om_edge) * gGainScale;

  const bool spinning_up = (tau >= 0.0f) == (wheel_omega >= 0.0f);
  if (spinning_up) {
    float s = (kMaxOmega - fabsf(wheel_omega)) / (kMaxOmega - kTaperStart);
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tau *= s;
  }

  if (!isfinite(tau)) { tau = 0.0f; gArmed = false; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  if (fabsf(phi_edge) > kMaxTilt)     { gArmed = false; }
  if (fabsf(wheel_omega) > kMaxOmega) { gArmed = false; }
  if (!isfinite(phi_edge) || !isfinite(om_edge)) { gArmed = false; }

  const float tau_cmd = gArmed ? tau : 0.0f;

  Moteus::PositionMode::Command cmd;
  cmd.position               = NaN;
  cmd.velocity               = 0.0f;
  cmd.kp_scale                = 0.0f;
  cmd.kd_scale                 = 0.0f;
  cmd.feedforward_torque       = kWheelSign * tau_cmd;
  cmd.maximum_torque           = kTauMax;
  cmd.watchdog_timeout          = 0.10f;
  cmd.ignore_position_bounds    = 1.0f;
  moteusActive.SetPosition(cmd, &kTorqueFormat);

  gLastTau    = tau;
  gLastTauCmd = tau_cmd;
}


// ----------------------------------------------------------------------------
// SECTION 2f: SERIAL COMMANDS
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
  } else if (cmd == 'm') {
    kMaxTilt = val * (float)DEG_TO_RAD;
    Serial.print("# kMaxTilt = "); Serial.print(val, 2); Serial.println(" deg");
  } else if (cmd == 'e') {
    resolveEdgeCandidate();
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
    Serial.println("# unknown. use: a<0/1>  g<0..1>  m<deg>  e (re-resolve)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - EDGE STAGE 3: POSITION + RATE (cube held by hand)");
  Serial.print("# ACTIVE AXIS: "); Serial.print(kAxisName[kAxis]);
  Serial.print("  (moteus id "); Serial.print(kAxisMoteusId[kAxis]);
  Serial.println(") -- confirm this matches your Stage 1/2 runs.");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    Serial.print("CAN error 0x");
    Serial.println(errorCode, HEX);
    delay(1000);
  }

  moteusActive.SetStop();
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

  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveEdgeCandidate();

  Serial.println("t_ms\tphi_edge_deg\tom_edge_dps\ttau_Nm\ttau_cmd_Nm\tarmed\t"
                  "gain_scale\twheel_pos\twheel_vel");
  Serial.println("# STARTS DISARMED at gGainScale=0.1. Send a1 to arm.");
  Serial.println("# g<0..1> steps the gain. m<deg> sets the tilt trip");
  Serial.println("# (default 40 deg -- wide on purpose, see header comment).");
  Serial.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();
  if (gHalted) { return; }

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

  float aImu[3], wImu[3];
  imu.getSensorData();
  readIMURaw(imu, aImu, wImu);
  attitudeUpdate(aImu, wImu, dt);
  updateEdgeProjection();

  const auto& v = moteusActive.last_result().values;
  const float wheel_omega = kWheelSign * v.velocity * 2.0f * (float)PI;

  commandWheel(wheel_omega);

  printState(time, gLastTau, gLastTauCmd, gArmed, gGainScale, v);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage4_FullLaw/Stage4_FullLaw.ino -- adds K[2] (wheel momentum
// management) plus friction feedforward, the real ARM_GATE (0.5 deg), and
// the Stage 4/5 tilt/omega trip values. The wheel should now UNWIND after
// each correction instead of accumulating.
// ============================================================================

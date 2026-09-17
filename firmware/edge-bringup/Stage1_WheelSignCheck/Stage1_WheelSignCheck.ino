// ============================================================================
// TEENSY 4.1 + moteus-n1 (CAN3) + BMI270 IMU (SPI) — EDGE STAGE 1:
// WHEEL SIGN CHECK -- AXIS-SELECTABLE
// ============================================================================
// CUBE HELD FIRMLY BY HAND. No feedback loop yet -- first real torque
// command to whichever wheel kAxis (below) selects, and the first real
// test of torque mode on THAT wheel specifically. Direct analogue of the
// panel's Stage1_OpenLoopTorque.ino (same method, same checklist shape) --
// see Firmware Lessons S4 for why kWheelSign must be verified per wheel
// and never assumed from another wheel's result, and S7 for why this
// stays an isolated, feedback-free test.
//
// >>> SET kAxis BELOW TO THE WHEEL YOU'RE TESTING AND RE-FLASH. <<<
// Y (id 3) already passed Stages 1-5 on real hardware. X (id 2) and Z
// (id 1) are UNTESTED -- each needs its own Stage 1 run before Stage 2
// onward can be trusted for that wheel. A sign or edge-candidate result
// from one wheel tells you NOTHING about another (Firmware Lessons S4) --
// this file cannot skip that just because Y already worked.
//
// STAGE 1 CHECKLIST (~15 min) — cube held firmly by hand.
//   Send "p" over Serial to fire ONE pulse: 0.05 N*m for 1000 ms.
//   [ ] Wheel spins                          -> torque mode works
//   [ ] wheel_vel goes POSITIVE for a positive pulse -> sign check 3
//   [ ] Does the cube push toward DECREASING |phi_edge| (back toward
//       vertical) while the wheel speeds up?  -> sign check 4
//
// 0.05 N*m, not 0.01: the panel's stiction was measured at ~0.05 N*m: use
// the same starting point, confirm/adjust for this rig with "t<Nm>".
//
// >>> SIGN CHECK 4 IS THE ONE THAT KILLS HARDWARE. <<<
// If a positive torque pushes the cube the WRONG way, the correct negative
// sign in Stage 2's control law will actively drive the fall once the loop
// closes. Fix kWheelSign here -- NEVER flip a minus sign in the control law
// to compensate. See Firmware Lessons S4 for the full reasoning.
//
// ---------------------------- WHY gam HERE ---------------------------------
// The panel's atan2 trick was 2D/45deg-mount specific (Firmware Lessons S5,
// S10) and does not apply -- the cube tips in a genuinely 3D way even
// though only the Y wheel actuates. This stage carries the SAME reduced-
// attitude estimator Gam/Skeleton_3Axis.ino uses (ghat/w_b), projected onto
// the single edge direction (phi_edge, om_edge) rather than a different
// filter. It is read-only telemetry this stage -- no feedback yet -- but
// getting the estimator right here, before any gain touches it, is the
// whole point of doing this before Stage 2.
//
// ONE DELIBERATE DIFFERENCE from Gam/Skeleton_3Axis.ino as it stands today:
// that file still runs the LERP-blend complementary filter (flagged
// elsewhere as needing replacement before any closed loop -- it is past
// the measured kP cliff, just never triggered because commandWheels() is a
// zero-torque stub there). Edge balance IS a real closed loop, so this
// file uses the validated kP=4/kI=0.5 cross-product filter from
// Attitude representation for the firmware.md / cubli_gains.h directly,
// not the blend. Gam/Skeleton_3Axis.ino needs the same replacement
// separately -- not done here, that file isn't touched by this one.
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
static const Axis kAxis = AXIS_X;   // <<< CHANGE THIS to test X or Z

static const int8_t kAxisMoteusId[3] = { 2, 3, 1 };   // indexed by Axis
static const char*  kAxisName[3]     = { "X", "Y", "Z" };

Moteus moteusActive(canBus, []() {
  Moteus::Options options;
  options.id = kAxisMoteusId[kAxis];
  return options;
}());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz -- matches the panel's bring-up rate.
                                 // Gains here are continuous-time (see the
                                 // 500Hz-vs-DT correction earlier), fine to
                                 // differ from the eventual 400Hz cube rate.

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Same semantics as every panel stage: loop() keeps running regardless of
// the Serial Monitor window; halting freezes IMU/CAN/telemetry and cancels
// any in-flight pulse.


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam, ported from Gam/Skeleton_3Axis.ino
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: still the 2D-panel's mount constants. Replace with this rig's own
// numbers once IMU_Calibration.ino (or pablo's face-rest wizard, once
// kFacePoses is filled in) has been re-run for THIS mount -- flagged, not
// fixed, here. Telemetry-only in this stage so it doesn't block the pulse
// test, but don't trust phi_edge/om_edge numerically until this is real.
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

// Mount rotation, sensor -> body. Copied verbatim from Gam/Skeleton_3Axis.ino
// -- keep byte-for-byte identical if that file's measured angles ever change.
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

// Reduced-attitude complementary filter, kP=4/kI=0.5 -- the validated form,
// not the LERP blend. e = ghat x ga (filter innovation, magnitude ~=
// sin(error)), correction is kP*(e x ghat), bias integral is PLUS
// kI*e*dt (Attitude representation.md: the opposite sign converges to 5x
// the true bias).
static const float kP_FILT = 4.0f;   // HARD CLIFF between 6 and 7 -- do not raise
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
  for (int i = 0; i < 3; ++i) { bhat[i] += kI_FILT * e[i] * dt; }   // PLUS
  for (int i = 0; i < 3; ++i) { w_b[i] = wRaw[i] - bhat[i]; }

  // Antipode reset (Attitude representation.md's numerical caution): e's
  // magnitude is sin(error), zero again at 180 deg -- if the cube gets
  // picked up and inverted, ghat can lock onto the antipode.
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
// SECTION 2c: EDGE CANDIDATE -- resolved by measurement, not memory
// ----------------------------------------------------------------------------
// Each axis has TWO best-placement edges in cubli_gains.h (smallest
// placement offset = best balancing feature for that axis -- the same
// selection logic that picked Y[+1,+1]/Y[-1,-1] originally), same edge
// direction within a pair but opposite-ish gB (which pair of transverse
// corners is up). Verbal description flipped twice for Y this session
// (see the id<->axis mapping saga) -- not doing that again for X/Z. This
// picks whichever candidate's gB the MEASURED ghat actually agrees with,
// at startup, and prints the answer instead of assuming it.

struct EdgeCandidate {
  const char* name;
  float e[3];             // edge direction
  float gB[3];
  float K[3];             // [K_phi, K_om, K_rho] -- only K[1] used until Stage 2
  float placeOffsetDeg;
};

// kCandidates[axis][0..1], both entries from cubli_gains.h's EDGE table.
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
  Serial.print("  place_offset="); Serial.print(kCandidates[kAxis][gEdgeIdx].placeOffsetDeg, 3);
  Serial.print(" deg  (dot0="); Serial.print(d0, 4);
  Serial.print(" dot1="); Serial.print(d1, 4); Serial.println(")");
  if (fabsf(d0 - d1) < 0.2f) {
    Serial.println("# WARNING: dot0 and dot1 are close -- cube may not be resting");
    Serial.println("#   stably on either candidate edge yet. Recheck before Stage 2.");
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

void printState(uint32_t t_ms, float tau, bool pulseActive, const Moteus::Query::Result& v) {
  Serial.print(t_ms);
  Serial.print('\t'); Serial.print(phi_edge * (float)RAD_TO_DEG, 3);
  Serial.print('\t'); Serial.print(om_edge  * (float)RAD_TO_DEG, 2);
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
// SECTION 2e: CONTROL -- single-shot open-loop pulse, no gains involved
// ----------------------------------------------------------------------------

static float    gTauPulse       = 0.05f;   // N*m -- live-settable, see "t<Nm>".
static const uint32_t kPulseDurationMs = 1000;
static const float kTauMax      = 0.12f;   // N*m, TAU_MAX from cubli_gains.h

// Confirmed for all three axes via their own run of this stage -- +1.0f
// across the board, no sign flip needed on this build. Re-verify per axis
// rather than assuming if the mount or wiring changes; do not copy another
// wheel's confirmed value onto a rebuild without re-running this stage.
// Applied to the OUTGOING command only in this stage -- telemetry
// (wheel_pos/wheel_vel/moteus_torque/qcurrent) intentionally still shows
// the raw, unflipped moteus convention here, same as the panel's Stage 1.
static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED on real hardware
  1.0f,   // Y -- CONFIRMED on real hardware
  1.0f,   // Z -- CONFIRMED on real hardware
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

static bool     gPulseActive  = false;
static uint32_t gPulseStartMs = 0;
static float    gLastTau      = 0.0f;

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

  if (!isfinite(tau)) { tau = 0.0f; gPulseActive = false; }

  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  Moteus::PositionMode::Command cmd;
  cmd.position               = NaN;
  cmd.velocity               = 0.0f;
  cmd.kp_scale                = 0.0f;
  cmd.kd_scale                 = 0.0f;
  cmd.feedforward_torque       = kWheelSign * tau;
  cmd.maximum_torque           = kTauMax;
  cmd.watchdog_timeout          = 0.10f;
  cmd.ignore_position_bounds    = 1.0f;
  moteusActive.SetPosition(cmd, &kTorqueFormat);

  gLastTau = tau;
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
    gTauPulse = val >  kTauMax ?  kTauMax : val < -kTauMax ? -kTauMax : val;
    Serial.print("# gTauPulse = "); Serial.print(gTauPulse, 4); Serial.println(" N*m");
  } else if (cmd == 'e') {
    resolveEdgeCandidate();
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
                    "e (re-resolve edge candidate)  h<0/1>");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - EDGE STAGE 1: WHEEL SIGN CHECK (cube held by hand)");
  Serial.print("# ACTIVE AXIS: "); Serial.print(kAxisName[kAxis]);
  Serial.print("  (moteus id "); Serial.print(kAxisMoteusId[kAxis]);
  Serial.println(") -- confirm this is the wheel you mean to pulse.");

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
  Serial.print("# gyro bias (body, rad/s): ");
  Serial.print(gGyroBiasBody[0], 6); Serial.print('\t');
  Serial.print(gGyroBiasBody[1], 6); Serial.print('\t');
  Serial.println(gGyroBiasBody[2], 6);

  // Prime ghat with a few real cycles before resolving the edge candidate --
  // the very first attitudeUpdate() call only initializes ghat from a
  // single raw sample, no filtering yet.
  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveEdgeCandidate();

  Serial.println("t_ms\tphi_edge_deg\tom_edge_dps\ttau_Nm\tpulse_active\t"
                  "wheel_pos\twheel_vel\tmoteus_mode\tmoteus_fault\t"
                  "moteus_torque\tmoteus_qcurrent");
  Serial.println("# moteus_mode 1 = FAULT. moteus_fault 39 = outside position");
  Serial.println("# bounds, 36 = never calibrated. See Firmware Lessons S3.");
  Serial.println("# send p to fire a pulse, t<Nm> to change its size (try t0.08");
  Serial.println("# if 0.05 doesn't move it), e to re-resolve the edge candidate");
  Serial.println("# HOLD THE CUBE FIRMLY BEFORE SENDING p");

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

  commandWheel();
  const auto& v = moteusActive.last_result().values;

  printState(time, gLastTau, gPulseActive, v);
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: Stage2_RateOnly/Stage2_RateOnly.ino -- first closed-loop term
// (om_edge/rate damping only), still hand-held. Set kAxis there to MATCH
// what you just ran here, and fix kAxisWheelSign[kAxis] with this stage's
// result FIRST -- do not proceed with a placeholder for whichever axis
// you're testing. This is also where the safety scaffold (latching trips)
// gets introduced, same shape as the panel's Stage 2.
// ============================================================================

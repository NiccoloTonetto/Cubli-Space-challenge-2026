// ============================================================================
// TEENSY 4.1 + BMI270 IMU (SPI) — IMU CALIBRATION WIZARD (3D rig, face-rest)
// ============================================================================
// One-shot bench utility: measures gyro zero-rate bias and the accelerometer
// offset/scale, then prints the three constants ready to paste verbatim into
// kGyroBias / kAccelOffset / kAccelScale in Skeleton_3Axis.ino (and
// Skeleton_3Axis_WiFi.ino) -- same convention as the 2D panel's calibration
// block (see "firmware/2D model/panel-bringup/test_control.txt" SECTION 2b
// and "Bring-Up Stages — Implementation Notes.md" S2.3): applied as
// corrected = (raw_SI - offset) / scale for accel, raw_SI - bias for gyro.
//
// WHY THIS VERSION DOES NOT USE THE CLASSIC "SILKSCREEN AXIS UP" TRICK:
// the IMU is mounted inside the assembled cube at the corner-balance mount
// angle (theta1=-30, theta2=54.74, theta3=45 -- see updateMountingDCM()
// below, copied verbatim from Skeleton_3Axis.ino). That mount is a compound
// rotation, so no face-rest position of the assembled cube puts a single
// BMI270 silkscreen axis purely vertical -- there is no orientation you can
// set the cube down in, resting on a face, that reads +-1g on exactly one
// raw channel and 0 on the other two.
//
// What the cube CAN do is rest stably on each of its 6 faces, and each of
// those 6 positions puts exactly one BODY axis vertical (not one SENSOR
// axis). Since the mount rotation is known and fixed, the expected
// sensor-frame gravity vector for each face-rest position can be computed
// from it directly (expectedSensorVec() below). This wizard captures the
// 6 face-rest means and fits offset+scale per sensor channel by least
// squares against those 6 known, generally NON-axis-aligned vectors,
// instead of the old opposite-pair average. This is a strict
// generalization: with an identity mount it reduces to the exact same
// arithmetic the previous revision used (up+down)/2 and (up-down)/2.
//
// !! kFacePoses BELOW MUST BE EDITED BEFORE USE !! -- this sketch cannot
// know, on its own, which physical face of your cube is "+X body up" vs
// "-Y body up" etc. Fill it in against whatever body-frame convention
// Skeleton_3Axis.ino's wheel torque axes / your CAD assembly already use.
// Getting an axis or sign wrong here will NOT throw an error -- it will
// silently produce a self-consistent-looking but wrong calibration. The
// STEP 7 verification (expects |a| ~= 9.8066 m/s^2 on an independent
// face-rest) is a necessary but not sufficient check -- a consistent
// sign/axis mixup can still pass it. Double check kFacePoses by eye once
// before trusting results from it.
//
// PROCEDURE (wizard runs automatically once, at power-up):
//   Step 0    -- gyro bias: set the cube down (any orientation), don't
//                touch it, ~9 s. Orientation-independent, unaffected by
//                the mount angle.
//   Steps 1-6 -- accel calibration: rest the cube flat on each of its 6
//                faces in turn, per kFacePoses' instructions, still, ~3 s
//                each.
//   Step 7    -- verification: rest on any face (does not need to be one
//                of the 6 above): checks the computed calibration
//                reconstructs |a| ~= 9.8066 m/s^2.
// Every step reports mean + std dev and warns (and lets you redo) if it
// looks like the cube moved. The final paste-ready block prints once at
// the end. Send 'r' + Enter any time afterwards to rerun the whole thing,
// or 'p' + Enter to reprint the last result without rerunning.
//
// Motor/CAN bus is never touched by this sketch -- it exists only to
// calibrate the IMU, so there is no moteus setup here at all.
// ============================================================================


// ----------------------------------------------------------------------------
// SECTION 1: LIBRARY INCLUDES
// ----------------------------------------------------------------------------

#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"


// ----------------------------------------------------------------------------
// SECTION 2: OBJECTS AND SETTINGS
// ----------------------------------------------------------------------------

BMI270 imu;
const uint8_t  imuChipSelectPin  = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kSampleSpacingMs  = 3;    // > 2.5 ms ODR period @ 400 Hz -- avoids re-reading a stale sample

const uint16_t kGyroSamples  = 3000;     // ~9 s
const uint16_t kAccelSamples = 1000;     // ~3 s

// Motion-gate thresholds on the per-step std dev -- heuristic, well above
// the BMI270's own noise floor, so these only fire on an actual bump/
// vibration/drift, not sensor noise.
const float kAccelStdGateG   = 0.01f;    // g
const float kGyroStdGateDps  = 0.30f;    // deg/s

static const float kG0 = 9.80665f;   // g -> m/s^2, same constant as the flight sketch


// ----------------------------------------------------------------------------
// SECTION 2b: MOUNT GEOMETRY (sensor frame -> body frame)
// ----------------------------------------------------------------------------
// Copied verbatim from Skeleton_3Axis.ino's updateMountingDCM(). Keep these
// three angles byte-for-byte identical to the flight sketch -- if that
// sketch's measured mount angles ever change, change them here too, or the
// "expected" vectors below stop matching reality and the fit silently goes
// wrong.

const float kTheta1Deg = -30.0f;
const float kTheta2Deg = 54.74f;
const float kTheta3Deg = 45.0f;

float gMountDCM[3][3];   // aBody = gMountDCM * aSensor

void updateMountingDCM() {
  const float k  = (float)DEG_TO_RAD;
  const float c1 = cosf(kTheta1Deg * k), s1 = sinf(kTheta1Deg * k);
  const float c2 = cosf(kTheta2Deg * k), s2 = sinf(kTheta2Deg * k);
  const float c3 = cosf(kTheta3Deg * k), s3 = sinf(kTheta3Deg * k);

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

// Same test as Skeleton_3Axis.ino's checkMountingDCMValid(): det ~= +1 and
// unit row norms. Catches a typo'd angle above before it poisons the fit.
bool mountDCMLooksValid() {
  const float (&C)[3][3] = gMountDCM;
  const float det =
      C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1]) -
      C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0]) +
      C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
  bool ok = fabsf(det - 1.0f) < 1e-3f;
  for (int i = 0; i < 3; ++i) {
    const float rowNorm = sqrtf(C[i][0]*C[i][0] + C[i][1]*C[i][1] + C[i][2]*C[i][2]);
    ok = ok && (fabsf(rowNorm - 1.0f) < 1e-3f);
  }
  return ok;
}


// ----------------------------------------------------------------------------
// SECTION 2c: FACE-REST POSITIONS  <-- EDIT THIS FOR YOUR BUILD BEFORE USE
// ----------------------------------------------------------------------------
// Two faces per body axis (opposite signs), matching however the cube
// physically rests on a table. "axis"/"sign" say which BODY axis points
// straight UP (away from the table) in that pose -- e.g. { AXIS_X, +1.0f }
// means "in this pose, +X (body) points up", regardless of which sensor
// channel that maps to.

enum BodyAxis { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

struct FacePose {
  const char* instructions;  // shown verbatim in the step prompt
  BodyAxis    axis;          // which BODY axis is vertical in this pose
  float       sign;          // +1.0f if +axis points up, -1.0f if -axis points up
};

// TODO(you): replace every "FIXME" string, and double-check axis/sign
// against the body frame Skeleton_3Axis.ino's wheel torques act about (or
// your CAD assembly's coordinate axes). Order doesn't matter.
const FacePose kFacePoses[6] = {
  { "FIXME: rest the cube on the face that makes +X (body) point UP", AXIS_X, +1.0f },
  { "FIXME: rest the cube on the face that makes -X (body) point UP", AXIS_X, -1.0f },
  { "FIXME: rest the cube on the face that makes +Y (body) point UP", AXIS_Y, +1.0f },
  { "FIXME: rest the cube on the face that makes -Y (body) point UP", AXIS_Y, -1.0f },
  { "FIXME: rest the cube on the face that makes +Z (body) point UP", AXIS_Z, +1.0f },
  { "FIXME: rest the cube on the face that makes -Z (body) point UP", AXIS_Z, -1.0f },
};

// Expected (noise-free) sensor-frame specific-force vector, in m/s^2, for a
// given face pose: trueBody = sign * g * e_axis, trueSensor = C^T*trueBody,
// and since C is orthogonal, C^T*e_axis is just row 'axis' of C.
void expectedSensorVec(const FacePose& pose, float trueSensor[3]) {
  for (int c = 0; c < 3; ++c) {
    trueSensor[c] = pose.sign * kG0 * gMountDCM[pose.axis][c];
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: SAMPLING ENGINE
// ----------------------------------------------------------------------------

struct Stats6 {
  float mean[6];   // ax, ay, az (g), gx, gy, gz (dps) -- raw SparkFun units
  float sd[6];      // population std dev, same order/units
};

// Blocking -- collects n samples spaced kSampleSpacingMs apart, computes
// per-channel mean and std dev in one pass (Welford's algorithm, so no
// per-sample storage is needed).
void collectStats(uint16_t n, Stats6& out) {
  double mean[6] = {0};
  double m2[6]   = {0};

  for (uint16_t i = 0; i < n; ++i) {
    imu.getSensorData();
    const double x[6] = {
      imu.data.accelX, imu.data.accelY, imu.data.accelZ,
      imu.data.gyroX,  imu.data.gyroY,  imu.data.gyroZ,
    };
    for (int c = 0; c < 6; ++c) {
      const double delta = x[c] - mean[c];
      mean[c] += delta / (double)(i + 1);
      m2[c]   += delta * (x[c] - mean[c]);
    }
    delay(kSampleSpacingMs);
  }

  for (int c = 0; c < 6; ++c) {
    out.mean[c] = (float)mean[c];
    out.sd[c]   = (float)sqrt(m2[c] / (double)n);
  }
}

// Ordinary least squares, y = slope*x + intercept, over exactly 6 points
// (n hardcoded since that's the fixed number of face poses / sensor
// channel this is ever called with).
void linregSlopeIntercept(const float x[6], const float y[6], float& slope, float& intercept) {
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int k = 0; k < 6; ++k) {
    sx  += x[k];
    sy  += y[k];
    sxx += x[k] * x[k];
    sxy += x[k] * y[k];
  }
  const float n     = 6.0f;
  const float denom = n * sxx - sx * sx;
  slope     = (n * sxy - sx * sy) / denom;
  intercept = (sy - slope * sx) / n;
}


// ----------------------------------------------------------------------------
// SECTION 4: WIZARD STEPS
// ----------------------------------------------------------------------------

void flushSerialInput() {
  while (Serial.available()) { Serial.read(); }
}

void waitForEnter() {
  flushSerialInput();
  while (!Serial.available()) { /* spin -- blocking by design, bench tool */ }
  flushSerialInput();
}

// Runs one capture, prints a report, and lets the user accept ('Enter') or
// redo ('r' + Enter) before returning -- so a bumped board or a
// not-quite-level surface doesn't silently poison the final numbers.
void runStep(const char* instructions, uint16_t nSamples, Stats6& out) {
  for (;;) {
    Serial.println();
    Serial.println(instructions);
    Serial.println("  (press Enter when the cube is positioned and still)");
    waitForEnter();

    Serial.println("  capturing...");
    collectStats(nSamples, out);

    Serial.print("  accel mean (g):   ");
    Serial.print(out.mean[0], 5); Serial.print(", ");
    Serial.print(out.mean[1], 5); Serial.print(", ");
    Serial.println(out.mean[2], 5);
    Serial.print("  accel std  (g):   ");
    Serial.print(out.sd[0], 5); Serial.print(", ");
    Serial.print(out.sd[1], 5); Serial.print(", ");
    Serial.println(out.sd[2], 5);
    Serial.print("  gyro  mean (dps): ");
    Serial.print(out.mean[3], 4); Serial.print(", ");
    Serial.print(out.mean[4], 4); Serial.print(", ");
    Serial.println(out.mean[5], 4);
    Serial.print("  gyro  std  (dps): ");
    Serial.print(out.sd[3], 4); Serial.print(", ");
    Serial.print(out.sd[4], 4); Serial.print(", ");
    Serial.println(out.sd[5], 4);

    const float accelSdMax = fmaxf(out.sd[0], fmaxf(out.sd[1], out.sd[2]));
    const float gyroSdMax  = fmaxf(out.sd[3], fmaxf(out.sd[4], out.sd[5]));
    bool suspect = false;
    if (accelSdMax > kAccelStdGateG) {
      Serial.println("  WARNING: accel std dev is high -- cube likely moved or vibrated.");
      suspect = true;
    }
    if (gyroSdMax > kGyroStdGateDps) {
      Serial.println("  WARNING: gyro std dev is high -- cube likely moved or vibrated.");
      suspect = true;
    }
    Serial.println(suspect
      ? "  Type 'r' + Enter to redo this step, or Enter to accept anyway."
      : "  looks stable. Press Enter to accept, or 'r' + Enter to redo.");

    flushSerialInput();
    while (!Serial.available()) { /* spin */ }
    const String line = Serial.readStringUntil('\n');
    flushSerialInput();
    if (line.length() > 0 && (line[0] == 'r' || line[0] == 'R')) {
      continue;   // redo this step
    }
    return;   // accepted
  }
}

// Prints one "static const float NAME = { ... };" line, formatted to match
// the declarations it's meant to replace in Skeleton_3Axis.ino. paddedName
// must already include trailing padding spaces so every "=" lands in the
// same column (see the aligned declarations there).
void printResultLine(const char* paddedName, const float v[3], const char* comment) {
  Serial.print("static const float ");
  Serial.print(paddedName);
  Serial.print("= { ");
  for (int i = 0; i < 3; ++i) {
    if (v[i] >= 0.0f) { Serial.print('+'); }
    Serial.print(v[i], 6);
    Serial.print('f');
    if (i < 2) { Serial.print(", "); }
  }
  Serial.print(" };");
  if (comment != nullptr) {
    Serial.print("  // ");
    Serial.print(comment);
  }
  Serial.println();
}

static float gGyroBias[3]    = { 0.0f, 0.0f, 0.0f };
static float gAccelOffset[3] = { 0.0f, 0.0f, 0.0f };
static float gAccelScale[3]  = { 1.0f, 1.0f, 1.0f };
static bool  gHaveResults    = false;

void printFinalBlock() {
  if (!gHaveResults) {
    Serial.println("# no results yet -- run the wizard first ('r' + Enter).");
    return;
  }
  Serial.println();
  Serial.println("==================== COPY INTO Skeleton_3Axis.ino (and _WiFi) ====================");
  printResultLine("kGyroBias[3]    ", gGyroBias, "rad/s");
  printResultLine("kAccelOffset[3] ", gAccelOffset, "m/s^2");
  printResultLine("kAccelScale[3]  ", gAccelScale, nullptr);
  Serial.println("====================================================================================");
}

void runCalibrationWizard() {
  Serial.println();
  Serial.println("==================== IMU CALIBRATION WIZARD (face-rest) ====================");

  Stats6 gyroStill;
  runStep("STEP 0/7 -- GYRO BIAS. Set the cube down (any orientation), do not "
          "touch or breathe on it, let it settle.", kGyroSamples, gyroStill);

  Stats6 s;
  float rawAccelMean[6][3];   // g, sensor frame, one row per kFacePoses entry

  for (int k = 0; k < 6; ++k) {
    char header[160];
    snprintf(header, sizeof(header), "STEP %d/7 -- FACE-REST POSE %d/6. %s.",
              k + 1, k + 1, kFacePoses[k].instructions);
    runStep(header, kAccelSamples, s);
    rawAccelMean[k][0] = s.mean[0];
    rawAccelMean[k][1] = s.mean[1];
    rawAccelMean[k][2] = s.mean[2];
  }

  // --- solve for offset/scale per axis by least squares ---
  // Model: corrected = (raw_g * kG0 - offset) / scale, and for face pose k
  // we want corrected == expectedSensorVec(kFacePoses[k]) (a known,
  // generally non-axis-aligned vector once the mount rotation is compound).
  // Equivalently: raw_SI(k) = offset + scale * expected(k) -- linear in
  // (offset, scale) for each sensor channel independently, fit over the 6
  // face poses by ordinary least squares.
  for (int c = 0; c < 3; ++c) {
    float trueVec[6], rawSI[6];
    for (int k = 0; k < 6; ++k) {
      float t[3];
      expectedSensorVec(kFacePoses[k], t);
      trueVec[k] = t[c];
      rawSI[k]   = rawAccelMean[k][c] * kG0;
    }
    float slope, intercept;
    linregSlopeIntercept(trueVec, rawSI, slope, intercept);
    gAccelScale[c]  = slope;
    gAccelOffset[c] = intercept;
  }
  for (int i = 0; i < 3; ++i) {
    gGyroBias[i] = gyroStill.mean[3 + i] * (float)DEG_TO_RAD;
  }
  gHaveResults = true;

  // --- verification ---
  Stats6 verify;
  runStep("STEP 7/7 -- VERIFY. Rest the cube on any face, still (does not "
          "need to match one of the 6 poses above).", kAccelSamples, verify);

  float aCorr[3], wCorr[3];
  for (int i = 0; i < 3; ++i) {
    aCorr[i] = (verify.mean[i] * kG0 - gAccelOffset[i]) / gAccelScale[i];
    wCorr[i] = verify.mean[3 + i] * (float)DEG_TO_RAD - gGyroBias[i];
  }
  const float aMag = sqrtf(aCorr[0]*aCorr[0] + aCorr[1]*aCorr[1] + aCorr[2]*aCorr[2]);

  Serial.println();
  Serial.print("  verification: |a_corrected| = "); Serial.print(aMag, 4);
  Serial.print(" m/s^2  (expect ~"); Serial.print(kG0, 4); Serial.println(")");
  Serial.print("  verification: w_corrected   = ");
  Serial.print(wCorr[0] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(wCorr[1] * (float)RAD_TO_DEG, 4); Serial.print(", ");
  Serial.print(wCorr[2] * (float)RAD_TO_DEG, 4);
  Serial.println(" dps  (expect ~0,0,0 if the cube was truly still)");
  if (fabsf(aMag - kG0) > 0.3f) {
    Serial.println("  WARNING: |a_corrected| is off from 1g by >0.3 m/s^2 -- recheck kFacePoses "
                    "(axis/sign typo) and the six face-rest steps.");
  }

  printFinalBlock();
  Serial.println("# send 'r' + Enter to rerun the whole wizard, 'p' + Enter to reprint this block.");
}


// ----------------------------------------------------------------------------
// SECTION 5: setup() / loop()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - IMU CALIBRATION WIZARD, face-rest (no CAN/moteus setup in this sketch)");

  updateMountingDCM();
  if (!mountDCMLooksValid()) {
    Serial.println("FATAL: gMountDCM failed the det~=1 / unit-row-norm sanity check -- "
                    "check kTheta1Deg/kTheta2Deg/kTheta3Deg above before continuing.");
    while (true) { delay(1000); }
  }

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

  runCalibrationWizard();
}

void loop() {
  if (!Serial.available()) { return; }
  const String line = Serial.readStringUntil('\n');
  flushSerialInput();
  if (line.length() == 0) { return; }

  if (line[0] == 'r' || line[0] == 'R') {
    runCalibrationWizard();
  } else if (line[0] == 'p' || line[0] == 'P') {
    printFinalBlock();
  } else {
    Serial.println("# send 'r' + Enter to rerun the wizard, 'p' + Enter to reprint the last result.");
  }
}


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// After copying the printed block into Skeleton_3Axis.ino, also copy the
// same three lines into Skeleton_3Axis_WiFi.ino (kept in sync by hand, same
// reasoning as the 2D bring-up files -- see "Bring-Up Stages —
// Implementation Notes.md" S1).
//
// If Skeleton_3Axis.ino's theta1_deg/theta2_deg/theta3_deg ever change
// (re-measured mount geometry), update kTheta1Deg/kTheta2Deg/kTheta3Deg
// above to match before rerunning this wizard -- a stale mount angle here
// silently biases the fit, it will not error out.
//
// This wizard does not touch kWheelSign, kThetaOffset-equivalents, or the
// mount angles themselves -- those are separate calibrations (see the TODOs
// in Skeleton_3Axis.ino SECTION 2b/2d).
// ============================================================================

// ============================================================================
// TEENSY 4.1 + BMI270 IMU (SPI) — IMU CALIBRATION WIZARD (3D rig)
// ============================================================================
// One-shot bench utility: measures gyro zero-rate bias and the 6-position
// accelerometer offset/scale, then prints the three constants ready to
// paste verbatim into kGyroBias / kAccelOffset / kAccelScale in
// Skeleton_3Axis.ino (and Skeleton_3Axis_WiFi.ino) -- same convention as the
// 2D panel's calibration block (see
// "firmware/2D model/panel-bringup/test_control.txt" SECTION 2b and
// "Bring-Up Stages — Implementation Notes.md" S2.3): applied as
// corrected = (raw_SI - offset) / scale for accel, raw_SI - bias for gyro.
//
// IMPORTANT -- these numbers are SENSOR-FRAME constants. In readIMU() in
// the flight sketch they are applied to the RAW accelX/Y/Z, gyroX/Y/Z
// BEFORE gMountDCM rotates anything into body axes. So this wizard has to
// be run against the BMI270 BOARD's own silkscreen X/Y/Z axes -- NOT the
// cube's assembled body axes, and NOT the 54.7/45 deg mount geometry.
// Either calibrate the bare board before it goes into the mount, or (if
// already mounted) work out from the mount geometry which cube face puts
// the board's own +X/-X/+Y/-Y/+Z/-Z arrow straight up for each step below
// -- do not eyeball the cube's own axes for this.
//
// PROCEDURE (wizard runs automatically once, at power-up):
//   Step 0    -- gyro bias: set the board down, don't touch it, ~9 s.
//   Steps 1-6 -- accel 6-position: one silkscreen axis arrow straight up
//                at a time, board flat and still, ~3 s each.
//   Step 7    -- verification: any still orientation: checks the computed
//                calibration reconstructs |a| ~= 9.8066 m/s^2.
// Every step reports mean + std dev and warns (and lets you redo) if it
// looks like the board moved. The final paste-ready block prints once at
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
    Serial.println("  (press Enter when the board is positioned and still)");
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
      Serial.println("  WARNING: accel std dev is high -- board likely moved or vibrated.");
      suspect = true;
    }
    if (gyroSdMax > kGyroStdGateDps) {
      Serial.println("  WARNING: gyro std dev is high -- board likely moved or vibrated.");
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
  Serial.println("==================== IMU CALIBRATION WIZARD ====================");

  Stats6 gyroStill;
  runStep("STEP 0/7 -- GYRO BIAS. Set the board on a stable surface, do not "
          "touch or breathe on it, let it settle.", kGyroSamples, gyroStill);

  Stats6 s;
  float accelUp[3], accelDown[3];

  runStep("STEP 1/7 -- ACCEL +X UP. Board flat, silkscreen +X arrow "
          "pointing straight UP (away from the table).", kAccelSamples, s);
  accelUp[0] = s.mean[0];

  runStep("STEP 2/7 -- ACCEL -X UP. Flip the board: silkscreen +X arrow "
          "pointing straight DOWN.", kAccelSamples, s);
  accelDown[0] = s.mean[0];

  runStep("STEP 3/7 -- ACCEL +Y UP. Silkscreen +Y arrow pointing straight "
          "UP.", kAccelSamples, s);
  accelUp[1] = s.mean[1];

  runStep("STEP 4/7 -- ACCEL -Y UP. Silkscreen +Y arrow pointing straight "
          "DOWN.", kAccelSamples, s);
  accelDown[1] = s.mean[1];

  runStep("STEP 5/7 -- ACCEL +Z UP. Silkscreen +Z arrow (chip face) "
          "pointing straight UP.", kAccelSamples, s);
  accelUp[2] = s.mean[2];

  runStep("STEP 6/7 -- ACCEL -Z UP. Silkscreen +Z arrow (chip face) "
          "pointing straight DOWN.", kAccelSamples, s);
  accelDown[2] = s.mean[2];

  // --- solve for offset/scale per axis ---
  // Model: corrected = (raw_g * kG0 - offset) / scale, and we want
  // corrected = +kG0 when that axis's arrow was up, -kG0 when down:
  //   offset = kG0 * (up + down) / 2
  //   scale  = (up - down) / 2         (dimensionless, should end up ~1)
  for (int i = 0; i < 3; ++i) {
    gGyroBias[i]    = gyroStill.mean[3 + i] * (float)DEG_TO_RAD;
    gAccelOffset[i] = kG0 * (accelUp[i] + accelDown[i]) * 0.5f;
    gAccelScale[i]  = (accelUp[i] - accelDown[i]) * 0.5f;
  }
  gHaveResults = true;

  // --- verification ---
  Stats6 verify;
  runStep("STEP 7/7 -- VERIFY. Set the board still in ANY orientation (does "
          "not need to be axis-aligned).", kAccelSamples, verify);

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
  Serial.println(" dps  (expect ~0,0,0 if the board was truly still)");
  if (fabsf(aMag - kG0) > 0.3f) {
    Serial.println("  WARNING: |a_corrected| is off from 1g by >0.3 m/s^2 -- recheck the six accel steps.");
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
  Serial.println("started - IMU CALIBRATION WIZARD (no CAN/moteus setup in this sketch)");

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
// This wizard does not touch kWheelSign, kThetaOffset-equivalents, or the
// mount angles (gTheta1Deg/gTheta2Deg/gTheta3Deg) -- those are separate
// calibrations (see the TODOs in Skeleton_3Axis.ino SECTION 2b/2d).
// ============================================================================

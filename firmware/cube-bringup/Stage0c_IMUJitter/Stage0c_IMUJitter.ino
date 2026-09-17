// ============================================================================
// TEENSY 4.1 + BMI270 IMU (SPI) — PHASE 0.4: IMU ODR/LPF, POLL JITTER
// ============================================================================
// No CAN, no moteus -- isolates SPI/IMU timing from CAN-FD interrupt load
// (Stage0b covers that side). Raises ODR from the 400 Hz used everywhere
// else in the repo so far to 800 Hz, matching the spec's anti-aliasing
// requirement against the 18-125 Hz wheel unbalance band (independent of
// control loop rate -- see the correction on this point: Kp is
// rate-independent, ODR is not, they are unrelated numbers that happen to
// both be "rate" in the name).
//
// WHAT THIS MEASURES, PRECISELY:
//   The real control loop polls the IMU unconditionally every cycle at a
//   fixed rate (imu.getSensorData() in Gam/Skeleton_3Axis.ino, no
//   data-ready interrupt) -- so the question that matters operationally
//   isn't abstract "sensor jitter", it's: at 400 Hz polling with ODR at
//   800 Hz, does every poll return FRESH data, or does polling
//   occasionally out-race the sensor and read a stale/repeated sample?
//   That's measured directly here (bit-exact repeat detection). The
//   POLL LOOP's own scheduling jitter (Teensy timing under SPI load) is
//   measured separately and is a different number from sensor jitter.
//
// WHAT THIS DOES NOT MEASURE:
//   Group delay of the internal digital filter (BWP setting below) is a
//   function of the (ODR, BWP) pair per the BMI270 datasheet's filter
//   response table -- it is not computable from firmware behavior, only
//   looked up. bwp/filter_perf are left at the library default here
//   deliberately rather than guessing a specific BMI2_ACC_*/BMI2_GYR_*
//   constant for a "~100 Hz cutoff" claim that hasn't been checked
//   against that table yet. TODO below is the actual next step.
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

static const uint32_t kPollPeriodUs = 2500;          // 400 Hz -- matches the real
                                                       // control loop's poll rate,
                                                       // NOT the ODR. This is the
                                                       // rate we're testing ODR=800
                                                       // has headroom against.
static const uint32_t kDurationUs   = 60000000UL;     // 60 s
static const uint32_t kStatusEveryUs = 5000000UL;

static bool gHalted = false;


// ----------------------------------------------------------------------------
// SECTION 2b: STATS
// ----------------------------------------------------------------------------

static uint32_t gPolls        = 0;
static uint32_t gStaleSamples = 0;   // bit-identical to the previous poll -> ODR
                                       // didn't produce fresh data in time
static uint32_t gPollJitterMinUs = 0xFFFFFFFFUL;
static uint32_t gPollJitterMaxUs = 0;
static uint64_t gPollIntervalSumUs = 0;

static float gPrevAx = NAN, gPrevAy = NAN, gPrevAz = NAN;
static float gPrevGx = NAN, gPrevGy = NAN, gPrevGz = NAN;


// ----------------------------------------------------------------------------
// SECTION 2c: SERIAL COMMANDS
// ----------------------------------------------------------------------------

void handleSerialCommands() {
  if (!Serial.available()) { return; }
  const String line = Serial.readStringUntil('\n');
  if (line.length() == 0) { return; }
  if (line[0] == 'h') {
    gHalted = !gHalted;
    Serial.print("# gHalted = "); Serial.println(gHalted ? "TRUE" : "FALSE");
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("started - PHASE 0.4: IMU ODR/LPF, POLL JITTER (60s)");

  pinMode(imuChipSelectPin, OUTPUT);
  digitalWrite(imuChipSelectPin, HIGH);
  SPI.begin();

  while (imu.beginSPI(imuChipSelectPin, imuClockFrequency) != BMI2_OK) {
    Serial.println("Error: BMI270 not connected, check wiring and CS pin!");
    delay(1000);
  }
  Serial.println("BMI270 connected!");

  int8_t err = BMI2_OK;
  err |= imu.setAccelODR(BMI2_ACC_ODR_800HZ);
  err |= imu.setGyroODR(BMI2_GYR_ODR_800HZ);
  // Performance-mode filtering (less noise, more power) rather than the
  // power-saving averaging mode -- matches Example04's pattern. bwp itself
  // (Normal/OSR2/OSR4/CIC) is intentionally NOT set here.
  //
  // TODO before trusting this for real: cross-reference the BMI270
  // datasheet's filter response / group-delay table for ODR=800Hz and
  // pick the bwp that lands the cutoff near 100 Hz, then call
  // setAccelFilterBandwidth(...)/setGyroFilterBandwidth(...) explicitly
  // with that constant and record the datasheet's stated group delay for
  // it -- that number, not anything this sketch prints, is what goes into
  // the loop-delay budget.
  err |= imu.setAccelPowerMode(BMI2_PERF_OPT_MODE);
  err |= imu.setGyroPowerMode(BMI2_PERF_OPT_MODE, BMI2_PERF_OPT_MODE);

  if (err != BMI2_OK) {
    Serial.print("Warning: BMI270 config call(s) returned error code ");
    Serial.println(err);
  }
  Serial.println("# ODR set to 800 Hz on both accel and gyro (bwp left at default -- see TODO above)");
  Serial.println("# polling at 400 Hz, checking every sample is fresh");
}


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  handleSerialCommands();
  if (gHalted) { return; }

  static uint32_t nextPollUs   = micros();
  static uint32_t lastPollUs   = micros();
  static uint32_t startUs      = micros();
  static uint32_t nextStatusUs = micros() + kStatusEveryUs;
  static bool firstPoll = true;
  static bool done = false;

  if (done) { return; }

  const uint32_t now = micros();
  if ((int32_t)(now - nextPollUs) < 0) { return; }
  nextPollUs += kPollPeriodUs;

  // --- poll-loop scheduling jitter: actual interval vs nominal 2500 us ---
  if (!firstPoll) {
    const uint32_t interval = now - lastPollUs;
    const uint32_t dev = (interval > kPollPeriodUs)
                        ? (interval - kPollPeriodUs)
                        : (kPollPeriodUs - interval);
    if (dev < gPollJitterMinUs) { gPollJitterMinUs = dev; }
    if (dev > gPollJitterMaxUs) { gPollJitterMaxUs = dev; }
    gPollIntervalSumUs += interval;
  }
  lastPollUs = now;

  // --- read, check freshness ---
  imu.getSensorData();
  const float ax = imu.data.accelX, ay = imu.data.accelY, az = imu.data.accelZ;
  const float gx = imu.data.gyroX,  gy = imu.data.gyroY,  gz = imu.data.gyroZ;

  gPolls++;
  if (!firstPoll &&
      ax == gPrevAx && ay == gPrevAy && az == gPrevAz &&
      gx == gPrevGx && gy == gPrevGy && gz == gPrevGz) {
    gStaleSamples++;
  }
  gPrevAx = ax; gPrevAy = ay; gPrevAz = az;
  gPrevGx = gx; gPrevGy = gy; gPrevGz = gz;
  firstPoll = false;

  const uint32_t elapsed = now - startUs;

  if ((int32_t)(now - nextStatusUs) >= 0) {
    nextStatusUs += kStatusEveryUs;
    Serial.print("# t="); Serial.print(elapsed / 1000000UL);
    Serial.print("s polls="); Serial.print(gPolls);
    Serial.print(" stale="); Serial.print(gStaleSamples);
    Serial.print(" jitter_us[min="); Serial.print(gPollJitterMinUs == 0xFFFFFFFFUL ? 0 : gPollJitterMinUs);
    Serial.print(" max="); Serial.print(gPollJitterMaxUs);
    Serial.println("]");
  }

  if (elapsed >= kDurationUs) {
    done = true;
    Serial.println();
    Serial.println("# ================ 60 SECONDS COMPLETE ================");
    Serial.print("# total polls: "); Serial.println(gPolls);
    Serial.print("# stale samples (ODR outrun by 400 Hz poll): "); Serial.println(gStaleSamples);
    Serial.print("# stale rate: "); Serial.print(100.0f * gStaleSamples / gPolls, 4); Serial.println(" %");
    Serial.print("# poll interval mean_us: "); Serial.println((uint32_t)(gPollIntervalSumUs / (gPolls - 1)));
    Serial.print("# poll jitter (|actual-2500us|) min_us: "); Serial.println(gPollJitterMinUs);
    Serial.print("# poll jitter (|actual-2500us|) max_us: "); Serial.println(gPollJitterMaxUs);
    Serial.println("#");
    Serial.println("# REMINDER: filter group delay is NOT measured above -- pull it from");
    Serial.println("# the BMI270 datasheet's table for whatever (ODR, bwp) you actually");
    Serial.println("# ship, per the TODO in setup().");
    const bool pass = (gStaleSamples == 0);
    Serial.print("# ================ RESULT: ");
    Serial.print(pass ? "PASS (zero stale samples)" : "FAIL (stale samples present)");
    Serial.println(" ================");
  }
}

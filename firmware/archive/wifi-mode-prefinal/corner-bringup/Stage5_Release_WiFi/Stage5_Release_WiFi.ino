// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 5: RELEASE  [WiFi build]
// ============================================================================
// WiFi variant of ../../../../corner-bringup/Stage5_Release/. The control
// path -- corner table, estimator, full law, friction feedforward, arm gate,
// and the REAL trip policy -- is copied VERBATIM and must not be retuned
// here. Retune in the USB sketch and re-copy. Everything this file adds is
// transport; see ../Stage1_CornerIDAndPulse_WiFi/ for the full rationale of
// the link layer, which is identical in all four corner bring-up WiFi stages.
//
// ---------------------------- COMMAND GRAMMAR -------------------------------
// This stage's own commands are UNCHANGED from the USB build:
//
//   a<0/1>    arm / disarm (a1 clears the trip latch, and is gated)
//   c         re-resolve corner
//   r<val>    log marker
//   h<0/1>    halt
//
// WiFi-only controls live on letters no corner stage uses:
//
//   k         keepalive no-op (feeds the link watchdog)
//   l<0/1>    link mode: l0 = telemetry on USB, l1 = telemetry on Serial1
//   d<N>      telemetry decimation, 1..100 (d2 = 250 Hz, d20 = 25 Hz)
//
// ------------------------- LINK LOSS IS NOT A TRIP --------------------------
// The watchdog disarms on link loss but deliberately does NOT touch
// gTripReason. The four trip reasons describe what the CUBE did; losing the
// radio is not one of them, and overwriting a latched tilt/omega/nan with a
// bogus code would destroy exactly the evidence step 4 of the procedure below
// tells you to read. A link-loss disarm can only happen while gArmed, i.e.
// while gTripReason is already TRIP_NONE, so it stays TRIP_NONE and the
// console line is the only record.
//
// >>> THIS IS THE STAGE WHERE NOBODY'S HAND IS ON THE CUBE, AND NOW THE
// OPERATOR IS ALSO ON THE FAR END OF A RADIO LINK. The physical stop/catch
// and the e-stop are the safety net -- NOT the watchdog, NOT "a0". Prove the
// watchdog works (kill terminal_wifi.py while armed, confirm it disarms
// within 300 ms) BEFORE the first real release. <<<
//
// ------------------------------- WIRING -------------------------------------
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   common GND
//
// ============================================================================
// PHYSICAL STOP/CATCH IN PLACE. E-STOP IN HAND. This is the first stage
// where the cube is NOT held by hand -- everything before this was
// rehearsal. The control law is identical to Stage 4; nothing new is being
// tested about the law itself, only about the full closed loop running
// unsupported, on a corner, for real, with all three wheels live at once.
//
// Trips are now the REAL cube-wide policy from cubli_gains.h (DISARM =
// 15 deg on norm3(phi), OMEGA_CAP = 40 rad/s per wheel) -- Stage 4's loose
// values were for hand-held observation only. Latched trip-reason readout
// (tilt / omega / nan), printed once at the moment it fires.
//
// STAGE 5 PROCEDURE:
//   1. Cube resting on the corner Stage 1 identified, near the resolved
//      equilibrium (arm gate needs this).
//   2. Send "a1" to arm -- refused if not close enough.
//   3. Let go / give it a small push. Watch it recover, or watch it trip.
//   4. If it trips: check trip_reason in telemetry before re-arming. A
//      trip is not a bug to route around by disarming and re-arming
//      quickly -- read the number first, and check WHICH wheel's rho or
//      which axis of phi drove it before assuming it's the same failure
//      mode you saw last time.
//
// Still using the bench power supply, not the flight battery. If it's stable
// but sluggish, or the resting equilibrium is measurably off from
// cubli_gains.h's per-corner theta_eq comments, that's consistent with a
// plant mismatch, not necessarily a sign/wiring bug -- re-derive gains once
// the final mass is on (measure -> derive -> re-tune).
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

Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 2; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 1; return options; }());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz control loop -- NOT the telemetry rate

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Halting force-disarms (strict superset of "a0"), resuming does NOT re-arm.

// ---------------- Link mode + command channels ----------------
// Identical in all four corner bring-up WiFi stages -- fix a bug here and fix
// it in all of them.

// Emit telemetry every Nth control cycle. Non-const: "d<N>" writes it.
// 21 fields at 500 Hz is ~85 kB/s, marginal against the 1 Mbaud link, and the
// UDP hop is the real ceiling (~105 lines/s measured in link-bringup Stage 6).
static uint8_t gTelemetryDecim = 2;   // 2 -> 250 Hz
static uint8_t gTelemetryCount = 0;
static const uint8_t kDecimMin = 1;
static const uint8_t kDecimMax = 100;

enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default

const uint32_t kLinkBaud      = 1000000;   // must match TEENSY_LINK_BAUD in
                                            // the XIAO sketch
const uint32_t kLinkTimeoutMs = 300;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
static uint32_t gLastSerial1RxMillis = 0;
static bool     gLinkAlive           = false;   // false at boot so the first
                                                 // packet from the laptop
                                                 // re-prints the header

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet. Replaces the USB build's blocking readStringUntil('\n').
struct LineReader {
  char buf[64];
  uint8_t len = 0;

  bool poll(Stream& s, char* outLine, size_t outSize) {
    while (s.available()) {
      const char c = (char)s.read();
      if (c == '\n' || c == '\r') {
        if (len == 0) { continue; }   // swallow CRLF / blank artifacts
        buf[len] = '\0';
        const uint8_t copyLen = (len < outSize - 1) ? len : (uint8_t)(outSize - 1);
        memcpy(outLine, buf, copyLen);
        outLine[copyLen] = '\0';
        len = 0;
        return true;
      }
      if (len < sizeof(buf) - 1) { buf[len++] = c; }
      // else: line too long -- drop the overflow char, keep accumulating
      // until the terminator so we resync on the next line.
    }
    return false;
  }
};

static LineReader gUsbReader;
static LineReader gLinkReader;

// Boot/diagnostic sink: writes to USB and to the XIAO link at the same time.
// Only used outside the control path -- per-cycle telemetry goes to exactly
// one stream, chosen by gLinkMode.
class DualPrint : public Print {
 public:
  using Print::write;   // keep the base write(const char*) overload visible
  size_t write(uint8_t c) override {
    Serial.write(c);
    Serial1.write(c);
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
    Serial1.write(b, n);
    return n;
  }
};
static DualPrint gBoot;

// Where asynchronous console messages go -- crucially the TRIP lines, which
// must reach whoever is watching. The channel the operator is actually on.
static inline Stream& activeStream() {
  return (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
}


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from Stage 1-3)
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: still the 2D-panel's mount constants -- see Stage 1's header.
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

void checkMountingDCMValid(Print& out) {
  const float (&C)[3][3] = gMountDCM;
  const float det = C[0][0]*(C[1][1]*C[2][2] - C[1][2]*C[2][1])
                   - C[0][1]*(C[1][0]*C[2][2] - C[1][2]*C[2][0])
                   + C[0][2]*(C[1][0]*C[2][1] - C[1][1]*C[2][0]);
  out.print("# mount DCM check: det="); out.print(det, 4);
  for (int i = 0; i < 3; ++i) {
    const float len = sqrtf(C[i][0]*C[i][0] + C[i][1]*C[i][1] + C[i][2]*C[i][2]);
    out.print("  |row"); out.print(i); out.print("|="); out.print(len, 4);
  }
  out.println();
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

// Reduced-attitude complementary filter, kP=4/kI=0.5 -- the validated form.
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

  // Antipode reset: e's magnitude is sin(error), zero again at 180 deg.
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
// SECTION 2c: CORNER CANDIDATE -- resolved by measurement, not memory
// ----------------------------------------------------------------------------
// All 8 corners from cubli_gains.h's CORNER table, verbatim -- byte-identical
// to the table in every other corner-bringup file.

struct CornerCandidate {
  const char* name;
  float gB[3];
  float Kp[3][9];   // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
  float placeOffsetDeg;
};

static const CornerCandidate kCorners[8] = {
  { "[-1,-1,-1]"  // lean 0.797 deg vs body diagonal, ell 122.84 mm, Sg 1.8875, lambda 8.2572
    , { -0.571549475f, -0.588645577f, -0.571688354f }
    , { { -6.9157443f, 3.39226651f, 3.42117763f, -0.834802926f, 0.420730621f, 0.42704013f, -0.000700236589f, 0.00623514736f, 0.00621826435f },  // wheel X
        { 3.51941299f, -6.8296299f, 3.51364994f, 0.412131369f, -0.848057508f, 0.411711216f, 0.00382628222f, -0.00316504063f, 0.00381609239f },  // wheel Y
        { 3.42913675f, 3.39467001f, -6.92366505f, 0.426502436f, 0.419758797f, -0.837698817f, 0.00606757682f, 0.00607034843f, -0.000870343472f } } // wheel Z
    , 0.797f },
  { "[-1,-1,+1]"  // lean 3.170 deg vs body diagonal, ell 128.54 mm, Sg 1.9750, lambda 8.1212
    , { -0.546220303f, -0.562558711f, 0.620621562f }
    , { { -7.58488846f, 3.33574939f, -3.65192771f, -0.921932399f, 0.430031687f, -0.480414748f, 0.000473975349f, 0.00748626003f, -0.00814931281f },  // wheel X
        { 3.43748641f, -7.47500134f, -3.7502768f, 0.421860248f, -0.928083122f, -0.467502952f, 0.00511831883f, -0.00179392833f, -0.00555475708f },  // wheel Y
        { -3.84645557f, -3.83494377f, -6.8614974f, -0.464526713f, -0.460435808f, -0.881195128f, -0.00357620185f, -0.00353527209f, -0.00314604398f } } // wheel Z
    , 3.170f },
  { "[-1,+1,-1]"  // lean 2.773 deg vs body diagonal, ell 126.08 mm, Sg 1.9373, lambda 8.1591
    , { -0.556852818f, 0.616181135f, -0.55698812f }
    , { { -7.26747227f, -3.47609496f, 3.4201951f, -0.874630272f, -0.451983064f, 0.442295492f, 0.000827456824f, -0.00813719723f, 0.00776857371f },  // wheel X
        { -3.79863954f, -6.87362671f, -3.8063941f, -0.426038593f, -0.899968863f, -0.426554501f, -0.000622705789f, -0.00668813102f, -0.000633999531f },  // wheel Y
        { 3.41229653f, -3.47500682f, -7.25577497f, 0.442907125f, -0.45318532f, -0.871017814f, 0.00794851314f, -0.0083494084f, 0.00103329937f } } // wheel Z
    , 2.773f },
  { "[-1,+1,+1]"  // lean 3.097 deg vs body diagonal, ell 131.64 mm, Sg 2.0226, lambda 8.0480
    , { -0.533349514f, 0.590173781f, 0.605997622f }
    , { { -8.23398495f, -3.56334782f, -3.77657819f, -1.06839609f, -0.416523129f, -0.442939252f, -0.00582132814f, -0.00140620384f, -0.00158187468f },  // wheel X
        { -3.41282964f, -7.33259392f, 4.13743114f, -0.429089457f, -0.917372108f, 0.514714897f, -0.00497917458f, -0.00120868487f, 0.00603242731f },  // wheel Y
        { -3.50851822f, 4.01196194f, -6.99511194f, -0.465059072f, 0.524895251f, -0.847355127f, -0.00789372995f, 0.00895721f, 0.00243827817f } } // wheel Z
    , 3.097f },
  { "[+1,-1,-1]"  // lean 3.171 deg vs body diagonal, ell 128.56 mm, Sg 1.9753, lambda 8.1180
    , { 0.620658159f, -0.562471628f, -0.546268404f }
    , { { -6.85864925f, -3.8295908f, -3.8494637f, -0.880151272f, -0.460793495f, -0.465634525f, -0.00304374471f, -0.00361368596f, -0.00366454106f },  // wheel X
        { -3.75562048f, -7.4855423f, 3.44052219f, -0.467026591f, -0.931139171f, 0.421110034f, -0.00539576123f, -0.00195194408f, 0.0049761422f },  // wheel Y
        { -3.65655398f, 3.33088136f, -7.58417654f, -0.481462985f, 0.429957539f, -0.921488523f, -0.00823030341f, 0.00754544372f, 0.000543692615f } } // wheel Z
    , 3.171f },
  { "[+1,-1,+1]"  // lean 2.609 deg vs body diagonal, ell 134.01 mm, Sg 2.0591, lambda 7.9804
    , { 0.595400333f, -0.539581716f, 0.595273018f }
    , { { -7.30901575f, -3.4263885f, 4.20474958f, -0.895961821f, -0.451509207f, 0.553766072f, 0.00180089788f, -0.00741348462f, 0.00873730052f },  // wheel X
        { -3.81644511f, -8.42617893f, -3.82059836f, -0.41949293f, -1.11799276f, -0.419535488f, 0.00164802792f, -0.00882489327f, 0.00163950643f },  // wheel Y
        { 4.19695139f, -3.42353535f, -7.30109262f, 0.554382324f, -0.452106625f, -0.892860353f, 0.00891439989f, -0.00758054852f, 0.00199437374f } } // wheel Z
    , 2.609f },
  { "[+1,+1,-1]"  // lean 3.095 deg vs body diagonal, ell 131.66 mm, Sg 2.0229, lambda 8.0501
    , { 0.606037736f, 0.590086639f, -0.533400357f }
    , { { -6.99898767f, 4.01700401f, -3.50819087f, -0.848652959f, 0.524742544f, -0.464503765f, 0.00232795905f, 0.00885933265f, -0.00779828522f },  // wheel X
        { 4.13386154f, -7.32801533f, -3.40998602f, 0.515303075f, -0.915354431f, -0.429458082f, 0.00615369575f, -0.00108131079f, -0.00508674001f },  // wheel Y
        { -3.77546f, -3.56726503f, -8.23596478f, -0.442555219f, -0.416409701f, -1.06887817f, -0.00154884392f, -0.00138305803f, -0.00584927434f } } // wheel Z
    , 3.095f },
  { "[+1,+1,+1]"  // lean 0.714 deg vs body diagonal, ell 136.99 mm, Sg 2.1048, lambda 7.9341
    , { 0.582457066f, 0.567126632f, 0.582332492f }
    , { { -7.76383209f, 3.81748652f, 4.04768848f, -0.975789487f, 0.490672857f, 0.522951066f, -0.000524852891f, 0.00608501304f, 0.00639130361f },  // wheel X
        { 3.93991017f, -8.08397388f, 3.93213129f, 0.481777757f, -1.03972197f, 0.481087863f, 0.00368249253f, -0.00348382187f, 0.00367164146f },  // wheel Y
        { 4.05641413f, 3.81822968f, -7.77580976f, 0.522317231f, 0.489363909f, -0.97946316f, 0.00622009346f, 0.00590694463f, -0.000720091164f } } // wheel Z
    , 0.714f },
};

int gCornerIdx = 0;

void resolveCornerCandidate(Print& echo) {
  int bestIdx = 0, secondIdx = 0;
  float bestDot = -2.0f, secondDot = -2.0f;
  for (int i = 0; i < 8; ++i) {
    const float d = dot3(ghat, kCorners[i].gB);
    if (d > bestDot) { secondDot = bestDot; secondIdx = bestIdx; bestDot = d; bestIdx = i; }
    else if (d > secondDot) { secondDot = d; secondIdx = i; }
  }
  gCornerIdx = bestIdx;
  echo.print("# corner resolved: "); echo.print(kCorners[gCornerIdx].name);
  echo.print("  place_offset="); echo.print(kCorners[gCornerIdx].placeOffsetDeg, 3);
  echo.print(" deg  (best_dot="); echo.print(bestDot, 4);
  echo.print(" runner_up="); echo.print(kCorners[secondIdx].name);
  echo.print(" dot="); echo.print(secondDot, 4); echo.println(")");
  if (bestDot - secondDot < 0.2f) {
    echo.println("# WARNING: best and runner-up corners are close -- verify before arming.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0]; phi[1] = -t[1]; phi[2] = -t[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------
// Tab-delimited, 21 fields, byte-for-byte the USB build's format -- the whole
// point of this build is that ../../terminal_wifi.py reproduces the Arduino
// Serial Monitor exactly. There is no PLOTMODE here: bring-up stages are read,
// not plotted.
//
//   t_ms  phi_x_deg phi_y_deg phi_z_deg  om_x_dps om_y_dps om_z_dps
//   rho_x rho_y rho_z  rho_x_lp rho_y_lp rho_z_lp
//   tau_x tau_y tau_z  tau_cmd_x tau_cmd_y tau_cmd_z  armed trip_reason
//
// NOTE the last column is trip_reason, NOT gain_scale -- that is the one
// difference from Stage 4's otherwise identical 21-field layout, and it is
// why ../../plot_session_csv.py's 21-column mode labels the last trace
// wrongly for a Stage 5 log. The numbers are still right.
//
// trip_reason: 0 none, 1 tilt (norm3(phi) > DISARM), 2 omega (any
// |rho[i]| > OMEGA_CAP), 3 nan. Latched until the next "a1" -- read it
// before re-arming, don't just re-arm and hope.

void printTelemetryHeader(Print& out) {
  out.println("t_ms\tphi_x_deg\tphi_y_deg\tphi_z_deg\t"
               "om_x_dps\tom_y_dps\tom_z_dps\trho_x\trho_y\trho_z\t"
               "rho_x_lp\trho_y_lp\trho_z_lp\t"
               "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
               "armed\ttrip_reason");
}

void printState(Stream& out, uint32_t t_ms, const float rho[3], const float rhoLp[3],
                const float tau[3], const float tauCmd[3], bool armed,
                int tripReason) {
  out.print(t_ms);
  out.print('\t'); out.print(phi[0] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(phi[1] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(phi[2] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(w_b[0] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[1] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[2] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(rho[0], 3);
  out.print('\t'); out.print(rho[1], 3);
  out.print('\t'); out.print(rho[2], 3);
  out.print('\t'); out.print(rhoLp[0], 3);
  out.print('\t'); out.print(rhoLp[1], 3);
  out.print('\t'); out.print(rhoLp[2], 3);
  out.print('\t'); out.print(tau[0], 4);
  out.print('\t'); out.print(tau[1], 4);
  out.print('\t'); out.print(tau[2], 4);
  out.print('\t'); out.print(tauCmd[0], 4);
  out.print('\t'); out.print(tauCmd[1], 4);
  out.print('\t'); out.print(tauCmd[2], 4);
  out.print('\t'); out.print(armed ? 1 : 0);
  out.print('\t'); out.println(tripReason);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — full law (identical to Stage 4), real trip policy
// ----------------------------------------------------------------------------
// Copied verbatim from the USB build. Do not retune here.

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 4

// Real cube-wide policy from cubli_gains.h -- NOT Stage 4's loose values.
static const float kMaxTilt    = 0.261799395f;   // rad, 15 deg -- DISARM, vs norm3(phi)
static const float kMaxOmega   = 40.0f;          // rad/s -- OMEGA_CAP, per wheel
static const float kTauMax     = 0.12f;          // N*m -- TAU_MAX
static const float kTaperStart = 36.0f;          // rad/s

static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s

static const float kArmGate = 0.00872664619f;   // rad, 0.5 deg

static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
};

enum TripReason { TRIP_NONE = 0, TRIP_TILT = 1, TRIP_OMEGA = 2, TRIP_NAN = 3 };
static int gTripReason = TRIP_NONE;

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

static float gLastTau[3]    = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3] = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]      = { 0.0f, 0.0f, 0.0f };

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels(const float rho[3], Stream& echo) {
  const float xVec[9] = {
    phi[0], phi[1], phi[2],
    w_b[0], w_b[1], w_b[2],
    rho[0], rho[1], rho[2],
  };

  float tau[3];
  for (int i = 0; i < 3; ++i) {
    const float* row = kCorners[gCornerIdx].Kp[i];
    float u = 0.0f;
    for (int j = 0; j < 9; ++j) { u -= row[j] * xVec[j]; }
    u *= gGainScale;
    u += kTauCw * tanhf(rho[i] / kEpsFf) + kBw * rho[i];

    const bool spinning_up = (u >= 0.0f) == (rho[i] >= 0.0f);
    if (spinning_up) {
      float s = (kMaxOmega - fabsf(rho[i])) / (kMaxOmega - kTaperStart);
      s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
      u *= s;
    }

    if (!isfinite(u)) { u = 0.0f; }
    u = u >  kTauMax ?  kTauMax : u;
    u = u < -kTauMax ? -kTauMax : u;
    tau[i] = u;
  }

  // --- latching trips: record WHY, print once, never auto-unlatch ---
  if (gArmed && norm3(phi) > kMaxTilt) {
    gArmed = false; gTripReason = TRIP_TILT;
    echo.println("# TRIP: tilt limit");
  }
  for (int i = 0; i < 3; ++i) {
    if (gArmed && fabsf(rho[i]) > kMaxOmega) {
      gArmed = false; gTripReason = TRIP_OMEGA;
      echo.print("# TRIP: wheel speed limit, wheel ");
      echo.println(i == 0 ? "X" : i == 1 ? "Y" : "Z");
    }
  }
  if (gArmed && (!isfinite(phi[0]) || !isfinite(phi[1]) || !isfinite(phi[2]) ||
                 !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2]) ||
                 !isfinite(tau[0]) || !isfinite(tau[1]) || !isfinite(tau[2]))) {
    gArmed = false; gTripReason = TRIP_NAN;
    echo.println("# TRIP: non-finite state or torque");
  }

  static const float kLpTau = 5.0f;
  const float dtNom = kPeriodMs * 1e-3f;
  const float alphaLp = dtNom / (kLpTau + dtNom);

  for (int i = 0; i < 3; ++i) {
    const float tau_cmd = gArmed ? tau[i] : 0.0f;
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = kAxisWheelSign[i] * tau_cmd;
    cmd.maximum_torque           = kTauMax;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;
    wheelObj(i).SetPosition(cmd, &kTorqueFormat);

    gRhoLp[i] += alphaLp * (rho[i] - gRhoLp[i]);
    gLastTau[i]    = tau[i];
    gLastTauCmd[i] = tau_cmd;
  }
}


// ----------------------------------------------------------------------------
// SECTION 2f: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// The USB build's a/c/r/h grammar, UNCHANGED -- the link controls live on
// k/l/d instead. `echo` is whichever stream the line arrived on, so an ack
// goes back down the channel it came in on.

void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'a') {
    if (val != 0.0f) {
      if (norm3(phi) < kArmGate) {
        gArmed = true;
        gTripReason = TRIP_NONE;   // manual re-arm clears the latch
        echo.println("# gArmed = TRUE");
      } else {
        gArmed = false;
        echo.print("# ARM REFUSED: |phi|="); echo.print(norm3(phi) * (float)RAD_TO_DEG, 3);
        echo.print(" deg exceeds ARM_GATE="); echo.print(kArmGate * (float)RAD_TO_DEG, 2);
        echo.println(" deg. Get closer to the resolved equilibrium and retry.");
      }
    } else {
      gArmed = false;
      echo.println("# gArmed = FALSE");
    }
  } else if (cmd == 'c') {
    resolveCornerCandidate(echo);
  } else if (cmd == 'r') {
    // Bookmark only -- no effect on control. Send this the instant you let
    // go; val is just a log label. For real time alignment, prefer detecting
    // where phi starts moving on its own in the data over trusting this
    // timestamp -- human release timing has more jitter than the control loop
    // does, and over WiFi it now also carries the UDP hop's latency.
    echo.print("# RECORD_START t_ms="); echo.print(millis());
    echo.print(" marker="); echo.println(val, 2);
  } else if (cmd == 'h') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
      echo.println("# disarmed by halt");
    }
    echo.print("# gHalted = ");
    echo.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                          : "FALSE (resumed, still DISARMED -- send a1)");
  } else if (cmd == 'd') {
    const int n = atoi(line + 1);
    gTelemetryDecim = (uint8_t)(n < kDecimMin ? kDecimMin
                                : (n > kDecimMax ? kDecimMax : n));
    gTelemetryCount = 0;
    echo.print("# gTelemetryDecim = "); echo.print(gTelemetryDecim);
    echo.print("  ("); echo.print(500.0f / (float)gTelemetryDecim, 1);
    echo.println(" Hz)");
    printTelemetryHeader(echo);   // rate changed -- restate what the columns are
  } else if (cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
  } else {
    echo.println("# unknown. use: a<0/1>  c (re-resolve)  r<val> (log marker)  h<0/1>  "
                  "d<N> (telem decim)  l<0/1> (link)  k (keepalive)");
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see LineReader
// above for why this can never block. 'l'-prefixed lines switch gLinkMode
// locally and are NOT passed to handleCommandLine().
void pollCommands() {
  static char lineBuf[64];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (lineBuf[0] == 'l') {
      const LinkMode newMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
      if (newMode == LinkMode::WIFI && gLinkMode != LinkMode::WIFI) {
        gLastSerial1RxMillis = millis();   // fresh grace period on entry
      }
      gLinkMode = newMode;
    } else {
      handleCommandLine(lineBuf, Serial);
    }
  }

  if (gLinkReader.poll(Serial1, lineBuf, sizeof(lineBuf))) {
    gLastSerial1RxMillis = millis();   // any line at all counts as link-alive
    if (!gLinkAlive) {
      // The laptop just showed up -- it missed the boot header, so restate it.
      gLinkAlive = true;
      printTelemetryHeader(Serial1);
    }
    if (lineBuf[0] == 'l') {
      gLinkMode = (lineBuf[1] != '0') ? LinkMode::WIFI : LinkMode::USB;
    } else {
      handleCommandLine(lineBuf, Serial1);
    }
  }
}


// ----------------------------------------------------------------------------
// SECTION 3: setup()
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // Bounded wait, NOT while(!Serial){} -- this build must still boot into
  // WIFI mode with nothing attached to USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  gBoot.println("\nstarted - CORNER STAGE 5: RELEASE (WiFi build, stop/catch + e-stop REQUIRED)");

  const uint32_t errorCode = ACAN_T4::can3.beginFD(canSettings);
  while (errorCode != 0) {
    gBoot.print("CAN error 0x");
    gBoot.println(errorCode, HEX);
    delay(1000);
  }

  moteusX.SetStop();
  moteusY.SetStop();
  moteusZ.SetStop();
  gBoot.println("all stopped");

  updateMountingDCM();
  checkMountingDCMValid(gBoot);

  pinMode(imuChipSelectPin, OUTPUT);
  digitalWrite(imuChipSelectPin, HIGH);
  SPI.begin();

  while (imu.beginSPI(imuChipSelectPin, imuClockFrequency) != BMI2_OK) {
    gBoot.println("Error: BMI270 not connected, check wiring and CS pin!");
    delay(1000);
  }
  gBoot.println("BMI270 connected!");

  if (imu.setAccelODR(BMI2_ACC_ODR_400HZ) != BMI2_OK ||
      imu.setGyroODR(BMI2_GYR_ODR_400HZ)  != BMI2_OK) {
    gBoot.println("Warning: could not raise BMI270 ODR to 400 Hz");
  }

  gBoot.println("# calibrating gyro bias -- keep the cube PERFECTLY STILL (~2s)");
  calibrateGyroBias(1000);

  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveCornerCandidate(gBoot);

  printTelemetryHeader(gBoot);
  gBoot.println("# STARTS DISARMED. Stop/catch + e-stop ready BEFORE sending a1.");
  gBoot.println("# trip_reason: 0 none  1 tilt  2 omega  3 nan");
  gBoot.println("# h1 halts (idle + disarm), h0 resumes (still disarmed).");
  gBoot.println("# WiFi build: d<N> sets telemetry decimation (d2 = 250 Hz,");
  gBoot.println("#   d20 = 25 Hz readable), l<0/1> picks USB/WiFi telemetry,");
  gBoot.println("#   k is the link keepalive. AUTO-DISARMS if no line arrives");
  gBoot.println("#   on Serial1 for 300 ms -- that is NOT logged as a trip.");
  gBoot.println("# PROVE THE WATCHDOG (kill the laptop script while armed)");
  gBoot.println("#   BEFORE the first real release.");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  pollCommands();

  // Link watchdog -- the laptop is the operator's only disarm button in WiFi
  // mode. gTripReason is deliberately NOT written here: losing the radio is
  // not something the cube did, and this branch can only fire while gArmed,
  // i.e. while the latch is already TRIP_NONE. See the header.
  // Enforced whenever WIFI mode is selected, even if the laptop has never
  // spoken (gLinkAlive still false) -- otherwise a USB "a1" with no laptop
  // would leave the law running. Matches CornerBalance_WiFi.ino:903-906.
  if (gLinkMode == LinkMode::WIFI &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    if (gArmed) {
      gArmed = false;
      gBoot.println("# DISARMED: link lost (not a trip -- trip_reason unchanged)");
    }
    gLinkAlive = false;
  }

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
  updateCornerProjection();

  const float rho[3] = {
    kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI,
    kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI,
  };

  commandWheels(rho, activeStream());

  // --- telemetry: decimated, and to Serial or Serial1 per gLinkMode ---
  if (++gTelemetryCount >= gTelemetryDecim) {
    gTelemetryCount = 0;
    printState(activeStream(), time, rho, gRhoLp, gLastTau, gLastTauCmd,
               gArmed, gTripReason);
  }
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// This is the end of the corner staged bring-up. From here: re-derive Kp
// once the battery cable/DC-DC (and eventually the flight battery itself)
// are actually mounted (measure -> derive -> re-tune), then repeat the
// entire Stage 1->5 sequence for the OTHER SEVEN corners before trusting any
// of them -- a result on one corner says nothing about another. Note that
// only [+1,+1,+1] and [-1,-1,-1] are currently reachable at all; the other
// six foul the frame (see ../../../corner-bringup/README.md).
//
// Once multiple corners are validated, corner-to-corner transitions are where
// the quaternion/MEKF estimator becomes load-bearing (Attitude
// representation for the firmware.md section 4) -- this reduced-attitude
// gam estimator is deliberately NOT that.
// ============================================================================

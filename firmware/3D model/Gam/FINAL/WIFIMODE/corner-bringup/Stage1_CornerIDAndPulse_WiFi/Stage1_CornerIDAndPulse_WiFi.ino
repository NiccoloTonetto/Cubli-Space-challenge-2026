// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER STAGE 1: CORNER ID + PER-WHEEL PULSE CHECK  [WiFi build]
// ============================================================================
// WiFi variant of ../../../../corner-bringup/Stage1_CornerIDAndPulse/. The
// control path -- corner table, estimator, pulse logic, kAxisWheelSign,
// kTauMax -- is copied VERBATIM and must not be retuned here. Retune in the
// USB sketch and re-copy. Everything this file adds is transport:
//
//   - a second telemetry/command channel on Serial1, wired to a XIAO ESP32C6
//     that relays it to UDP :4210 (see ../../xiao_teensy_bridge/)
//   - a non-blocking LineReader on BOTH channels, replacing the USB build's
//     String + Serial.readStringUntil('\n') -- that call BLOCKS up to its
//     stream timeout, which is exactly the wrong thing to do inside a 2 ms
//     control cycle once a second stream can go quiet mid-line
//   - a link watchdog that cancels an in-flight pulse if the laptop vanishes
//   - runtime telemetry decimation, so the same build can feed a capture at
//     250 Hz or a human reading the terminal at 25 Hz
//
// ---------------------------- COMMAND GRAMMAR -------------------------------
// Every command letter this stage already had means EXACTLY what it means in
// the USB build:
//
//   w<0/1/2>  select pulse wheel      p         fire pulse
//   t<Nm>     pulse size              c         re-resolve corner
//   h<0/1>    halt
//
// The WiFi-only controls therefore had to move off 'h', 'p' and 't', which
// ../../CornerBalance_WiFi/ had used for keepalive, halt and link mode. The
// five corner stages between them use a c g h m p r t w z, so the link layer
// lives on the free letters:
//
//   k         keepalive no-op (feeds the link watchdog)
//   l<0/1>    link mode: l0 = telemetry on USB, l1 = telemetry on Serial1
//   d<N>      telemetry decimation, 1..100
//
// 'x' is deliberately NOT used: xiao_teensy_bridge.ino consumes x0/x1 to
// switch its own mode and never relays them to this board.
//
// NOTE the legacy difference: ../../CornerBalance_WiFi/ (Stage 4) still uses
// h=keepalive, p=halt, t=link. It is the validated build with recorded
// sessions and was left alone. It also accepts 'k' as a keepalive alias,
// which is why ../../terminal_wifi.py drives both it and this file.
//
// ------------------------------ TELEMETRY RATE ------------------------------
// The control loop stays at 500 Hz. Telemetry is emitted every
// gTelemetryDecim-th cycle, default 2 -> 250 Hz, matching the validated
// CornerBalance_WiFi. This stage's line is 13 fields (~90 B), so 500 Hz
// would fit the 1 Mbaud link on paper, but the UDP hop does not: Stage 6 of
// the link ladder measured ~105 lines/s actually reaching the laptop. Send
// d20 (25 Hz) when you want to READ the terminal rather than capture it.
//
// ------------------------------- WIRING -------------------------------------
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   common GND
//
// ============================================================================
// CUBE HELD FIRMLY BY HAND (or braced), resting on ONE corner. No feedback
// loop yet -- all three wheels are commanded every cycle (SetPosition sent
// each 2 ms tick, watchdog-safe), but only ONE wheel at a time carries
// nonzero torque, selected with "w<0/1/2>" and fired with "p", same method
// as edge-bringup's Stage 1.
//
// >>> SIGN CHECK 4 IS THE ONE THAT KILLS HARDWARE. <<<
// If a positive torque on a wheel pushes the cube the WRONG way (phi should
// shrink, not grow, while that wheel's rho increases), the correct-looking
// negative sign in Stage 2's Kp matrix will actively drive the fall once
// the loop closes. Fix kAxisWheelSign here -- NEVER flip a sign inside Kp
// to compensate.
//
// STAGE 1 CHECKLIST (~20 min) — cube held/braced on one corner.
//   Send "c" to resolve which of the 8 corners is currently down -- confirm
//   it matches the corner you think you're resting on before doing anything
//   else. Then for EACH wheel (w0=X, w1=Y, w2=Z):
//   [ ] "w<i>" to select it, "p" to fire ONE pulse (0.05 N*m, 1000 ms)
//   [ ] that wheel's rho[i] goes POSITIVE for a positive pulse
//   [ ] the OTHER two wheels' rho stay ~0
//   [ ] cube pushes toward DECREASING |phi| while the pulsed wheel speeds up
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

// Confirmed on bench, physically verified: id 2 -> X, id 3 -> Y, id 1 -> Z.
Moteus moteusX(canBus, []() { Moteus::Options options; options.id = 2; return options; }());
Moteus moteusY(canBus, []() { Moteus::Options options; options.id = 3; return options; }());
Moteus moteusZ(canBus, []() { Moteus::Options options; options.id = 1; return options; }());

BMI270 imu;
const uint8_t imuChipSelectPin = 10;
const uint32_t imuClockFrequency = 4000000;
const uint32_t kPeriodMs = 2;   // 500 Hz control loop -- NOT the telemetry rate

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;
// Same semantics as every panel/edge/corner stage: loop() keeps running
// regardless of the terminal; halting freezes IMU/CAN/telemetry and cancels
// any in-flight pulse.

// ---------------- Link mode + command channels ----------------
// Lifted from ../../CornerBalance_WiFi/CornerBalance_WiFi.ino. Shared by all
// four corner bring-up WiFi stages -- fix a bug here and fix it in all of
// them.

// Emit telemetry every Nth control cycle. Non-const: "d<N>" writes it.
static uint8_t gTelemetryDecim = 2;   // 2 -> 250 Hz
static uint8_t gTelemetryCount = 0;
static const uint8_t kDecimMin = 1;
static const uint8_t kDecimMax = 100;

enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default -- this build
                                               // must be useful with nothing
                                               // attached to USB at all

const uint32_t kLinkBaud      = 1000000;   // Serial1 <-> XIAO. Must match
                                            // TEENSY_LINK_BAUD in the XIAO
                                            // sketch.
const uint32_t kLinkTimeoutMs = 300;       // pulse cancelled if WIFI mode and
                                            // no Serial1 line in this long
                                            // (terminal_wifi.py sends a
                                            // keepalive every 100 ms)
static uint32_t gLastSerial1RxMillis = 0;
static bool     gLinkAlive           = false;   // false at boot so the first
                                                 // packet from the laptop
                                                 // re-prints the header

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet. Returns true (outLine filled + NUL-terminated) at most once
// per call, i.e. bounded work per loop() iteration; queued lines are picked
// up on the following iteration(s). This is what replaces the USB build's
// blocking Serial.readStringUntil('\n').
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

// Boot/diagnostic sink: writes to USB and to the XIAO link at the same time,
// so boot diagnostics are visible whichever channel you happen to be on.
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

// Where asynchronous console messages go (pulse end, watchdog, trips): the
// channel the operator is actually watching.
static inline Stream& activeStream() {
  return (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
}


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam, identical to the USB build
// ----------------------------------------------------------------------------

static const float kG0 = 9.80665f;
// TODO: still the 2D-panel's mount constants. Replace with this rig's own
// numbers once IMU_Calibration.ino has been re-run for THIS mount.
static const float kGyroBias[3]    = { +0.002348f, -0.001140f, -0.000762f };
static const float kAccelOffset[3] = { -0.092282f, -0.196510f, +0.050664f };
static const float kAccelScale[3]  = {  0.991085f,  0.991035f,  1.003787f };

// Mount rotation, sensor -> body.
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

  // Antipode reset: e's magnitude is sin(error), zero again at 180 deg -- if
  // the cube gets picked up and inverted, ghat can lock onto the antipode.
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
    echo.println("# WARNING: best and runner-up corners are close -- cube may not");
    echo.println("#   be resting stably on a single corner yet. Recheck before Stage 2.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };
// om is just w_b, no separate variable needed -- read directly at use.
float rho[3] = { 0.0f, 0.0f, 0.0f };

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0]; phi[1] = -t[1]; phi[2] = -t[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------
// Tab-delimited, 13 fields, byte-for-byte the USB build's format -- the whole
// point of this build is that ../../terminal_wifi.py reproduces the Arduino
// Serial Monitor exactly. There is no PLOTMODE here: bring-up stages are read,
// not plotted. (Stage 4 keeps both -- see ../../CornerBalance_WiFi/.)
//
//   t_ms  pulse_wheel  tau_Nm  pulse_active
//   phi_x_deg phi_y_deg phi_z_deg  om_x_dps om_y_dps om_z_dps
//   rho_x rho_y rho_z

void printTelemetryHeader(Print& out) {
  out.println("t_ms\tpulse_wheel\ttau_Nm\tpulse_active\t"
               "phi_x_deg\tphi_y_deg\tphi_z_deg\t"
               "om_x_dps\tom_y_dps\tom_z_dps\t"
               "rho_x\trho_y\trho_z");
}

void printState(Stream& out, uint32_t t_ms, int pulseWheel, float tau, bool pulseActive) {
  out.print(t_ms);
  out.print('\t'); out.print(pulseWheel);
  out.print('\t'); out.print(tau, 4);
  out.print('\t'); out.print(pulseActive ? 1 : 0);
  out.print('\t'); out.print(phi[0] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(phi[1] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(phi[2] * (float)RAD_TO_DEG, 3);
  out.print('\t'); out.print(w_b[0] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[1] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(w_b[2] * (float)RAD_TO_DEG, 2);
  out.print('\t'); out.print(rho[0], 3);
  out.print('\t'); out.print(rho[1], 3);
  out.print('\t'); out.println(rho[2], 3);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL -- single-shot open-loop pulse on ONE selected wheel
// ----------------------------------------------------------------------------
// Copied verbatim from the USB build. No gains involved. The other two wheels
// always get a zero-torque command (same Format/watchdog contract) so all
// three stay in torque mode together.

static float    gTauPulse       = 0.05f;   // N*m -- live-settable, see "t<Nm>".
static const uint32_t kPulseDurationMs = 1000;
static const float kTauMax      = 0.12f;   // N*m, TAU_MAX from cubli_gains.h

// Carried forward from edge-bringup: all three CONFIRMED +1.0f on real
// hardware there (Stage1_WheelSignCheck.ino).
static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED (edge-bringup)
  1.0f,   // Y -- CONFIRMED (edge-bringup)
  1.0f,   // Z -- CONFIRMED (edge-bringup)
};

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

static int      gPulseWheel   = 0;       // 0=X 1=Y 2=Z -- select with "w<i>"
static bool     gPulseActive  = false;
static uint32_t gPulseStartMs = 0;
static float    gLastTau      = 0.0f;

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels(Stream& echo) {
  float tau = 0.0f;
  if (gPulseActive) {
    if (millis() - gPulseStartMs < kPulseDurationMs) {
      tau = gTauPulse;
    } else {
      gPulseActive = false;
      echo.println("# PULSE END");
    }
  }

  if (!isfinite(tau)) { tau = 0.0f; gPulseActive = false; }
  tau = tau >  kTauMax ?  kTauMax : tau;
  tau = tau < -kTauMax ? -kTauMax : tau;

  for (int i = 0; i < 3; ++i) {
    const float tau_i = (i == gPulseWheel) ? tau : 0.0f;
    Moteus::PositionMode::Command cmd;
    cmd.position               = NaN;
    cmd.velocity               = 0.0f;
    cmd.kp_scale                = 0.0f;
    cmd.kd_scale                 = 0.0f;
    cmd.feedforward_torque       = kAxisWheelSign[i] * tau_i;
    cmd.maximum_torque           = kTauMax;
    cmd.watchdog_timeout          = 0.10f;
    cmd.ignore_position_bounds    = 1.0f;
    wheelObj(i).SetPosition(cmd, &kTorqueFormat);
  }

  gLastTau = tau;
}


// ----------------------------------------------------------------------------
// SECTION 2f: COMMANDS (from either Serial or Serial1)
// ----------------------------------------------------------------------------
// The USB build's w/p/t/c/h grammar, UNCHANGED -- see the header for why the
// link controls had to move to k/l/d instead. `echo` is whichever stream the
// line arrived on, so an ack goes back down the channel it came in on.

void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];

  if (cmd == 'w') {
    const int val = atoi(line + 1);
    if (val >= 0 && val <= 2) {
      gPulseWheel = val;
      echo.print("# pulse wheel = "); echo.println(val == 0 ? "X" : val == 1 ? "Y" : "Z");
    } else {
      echo.println("# invalid, use w0 (X) w1 (Y) w2 (Z)");
    }
  } else if (cmd == 'p') {
    if (gPulseActive) {
      echo.println("# pulse already running, ignored");
    } else {
      gPulseActive  = true;
      gPulseStartMs = millis();
      echo.print("# PULSE START on wheel ");
      echo.print(gPulseWheel == 0 ? "X" : gPulseWheel == 1 ? "Y" : "Z");
      echo.print(": "); echo.print(gTauPulse, 4);
      echo.println(" N*m for 1000 ms");
    }
  } else if (cmd == 't') {
    const float val = atof(line + 1);
    gTauPulse = val >  kTauMax ?  kTauMax : val < -kTauMax ? -kTauMax : val;
    echo.print("# gTauPulse = "); echo.print(gTauPulse, 4); echo.println(" N*m");
  } else if (cmd == 'c') {
    resolveCornerCandidate(echo);
  } else if (cmd == 'h') {
    const float val = atof(line + 1);
    gHalted = (val != 0.0f);
    if (gHalted && gPulseActive) {
      gPulseActive = false;
      echo.println("# pulse cancelled by halt");
    }
    echo.print("# gHalted = ");
    echo.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                          : "FALSE (resumed)");
  } else if (cmd == 'd') {
    const int val = atoi(line + 1);
    gTelemetryDecim = (uint8_t)(val < kDecimMin ? kDecimMin
                                : (val > kDecimMax ? kDecimMax : val));
    gTelemetryCount = 0;
    echo.print("# gTelemetryDecim = "); echo.print(gTelemetryDecim);
    echo.print("  ("); echo.print(500.0f / (float)gTelemetryDecim, 1);
    echo.println(" Hz)");
    printTelemetryHeader(echo);   // rate changed -- restate what the columns are
  } else if (cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
  } else {
    echo.println("# unknown. use: w<0/1/2> (select wheel)  p (fire pulse)  "
                  "t<Nm> (pulse size)  c (re-resolve corner)  h<0/1>  "
                  "d<N> (telem decim)  l<0/1> (link)  k (keepalive)");
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see LineReader
// above for why this can never block. 'l'-prefixed lines switch gLinkMode
// locally and are NOT passed to handleCommandLine(); everything else is,
// regardless of which stream it arrived on.
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

  gBoot.println("\nstarted - CORNER STAGE 1: CORNER ID + PULSE CHECK (WiFi build, cube held/braced)");

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
  gBoot.print("# gyro bias (body, rad/s): ");
  gBoot.print(gGyroBiasBody[0], 6); gBoot.print('\t');
  gBoot.print(gGyroBiasBody[1], 6); gBoot.print('\t');
  gBoot.println(gGyroBiasBody[2], 6);

  // Prime ghat with a few real cycles before resolving the corner candidate
  // -- the very first attitudeUpdate() call only initializes ghat from a
  // single raw sample, no filtering yet.
  for (int i = 0; i < 50; ++i) {
    float aImu[3], wImu[3];
    imu.getSensorData();
    readIMURaw(imu, aImu, wImu);
    attitudeUpdate(aImu, wImu, 0.002f);
    delay(2);
  }
  resolveCornerCandidate(gBoot);

  printTelemetryHeader(gBoot);
  gBoot.println("# send c to re-resolve the corner candidate, w<0/1/2> to pick");
  gBoot.println("# a wheel (0=X 1=Y 2=Z), p to fire a pulse, t<Nm> to change its");
  gBoot.println("# size (try t0.08 if 0.05 doesn't move it)");
  gBoot.println("# HOLD/BRACE THE CUBE FIRMLY BEFORE SENDING p");
  gBoot.println("# WiFi build: d<N> sets telemetry decimation (d2 = 250 Hz,");
  gBoot.println("#   d20 = 25 Hz readable), l<0/1> picks USB/WiFi telemetry,");
  gBoot.println("#   k is the link keepalive. An in-flight pulse is CANCELLED");
  gBoot.println("#   if no line arrives on Serial1 for 300 ms.");

  gNextSendMillis = millis();
}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {
  pollCommands();

  // Link watchdog. Stage 1 has no gArmed to drop -- the hazard here is an
  // open-loop torque pulse still running with the operator's terminal gone,
  // so that is what gets cancelled. Firing again needs a fresh "p".
  // Enforced whenever WIFI mode is selected, even if the laptop has never
  // spoken (gLinkAlive still false) -- otherwise a USB "p" with no laptop
  // would run the pulse blind. Matches CornerBalance_WiFi.ino:903-906.
  if (gLinkMode == LinkMode::WIFI &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    if (gPulseActive) {
      gPulseActive = false;
      gBoot.println("# PULSE CANCELLED: link lost");
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

  commandWheels(activeStream());

  rho[0] = kAxisWheelSign[0] * moteusX.last_result().values.velocity * 2.0f * (float)PI;
  rho[1] = kAxisWheelSign[1] * moteusY.last_result().values.velocity * 2.0f * (float)PI;
  rho[2] = kAxisWheelSign[2] * moteusZ.last_result().values.velocity * 2.0f * (float)PI;

  // --- telemetry: decimated, and to Serial or Serial1 per gLinkMode ---
  if (++gTelemetryCount >= gTelemetryDecim) {
    gTelemetryCount = 0;
    printState(activeStream(), time, gPulseWheel, gLastTau, gPulseActive);
  }
}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Next: ../Stage2_RateOnly_WiFi/ -- first closed-loop term (the om/rate block
// of each wheel's Kp row only, phi and rho columns masked to zero), still
// hand-held. That is also where the safety scaffold (arm gate, trips) gets
// introduced, and where the link watchdog starts disarming rather than
// cancelling a pulse.
//
// Do not proceed if any of this stage's three per-wheel sign checks failed,
// or if the corner resolution warning fired and wasn't resolved.
// ============================================================================

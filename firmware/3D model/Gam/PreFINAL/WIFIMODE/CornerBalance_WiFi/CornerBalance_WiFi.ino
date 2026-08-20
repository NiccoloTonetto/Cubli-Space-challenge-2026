// ============================================================================
// TEENSY 4.1 + moteus-n1 x3 (CAN3, ids 1/2/3) + BMI270 IMU (SPI) —
// CORNER BALANCE, FULL LAW — WiFi BUILD
// ============================================================================
// Same physics, estimator, corner-candidate table, control law, gains, trips,
// friction feedforward, phi-tare and arm gate as
// ../../USBMODE/CornerBalance/CornerBalance.ino (which is itself a
// byte-identical copy of Gam/corner-bringup/Stage4_FullLaw/Stage4_FullLaw.ino,
// the USB reference build). NOTHING in the control path is changed here.
//
// The link layer below is transplanted from
// ../EdgeBalance_WiFi/EdgeBalance_WiFi.ino — same LineReader, same DualPrint,
// same watchdog, same t0/t1 mode switch — so the two WiFi builds behave
// identically on the wire and share one XIAO firmware.
//
// !! NEITHER WiFi BUILD HAS BEEN VALIDATED ON HARDWARE YET. !!
// The USB corner build has flown; this one has not. Treat the first run as a
// bring-up, not a repeat: hand-held, one hand on the cube, a0 ready.
//
// What this file adds on top of the USB build, and only this:
//   - a second telemetry/command channel on Serial1, wired to a XIAO ESP32C6
//     running ../xiao_teensy_bridge/xiao_teensy_bridge.ino, so the cube can
//     balance on its corner untethered from USB;
//   - a link watchdog that auto-disarms if the WiFi link goes quiet;
//   - non-blocking line reading on BOTH streams (the USB build's
//     Serial.readStringUntil() can block up to its timeout on a partial line,
//     which is not acceptable inside a 2 ms control cycle);
//   - a PLOTMODE CSV telemetry format (the USB build is tab-delimited only);
//   - boot diagnostics mirrored to both streams, so an untethered bring-up
//     still shows the CAN check, IMU status, the "hold still" gyro-calibration
//     prompt and the resolved corner in the WiFi console.
//
// --------------------------- TELEMETRY RATE ---------------------------
// The control loop runs at 500 Hz, unchanged. TELEMETRY is emitted every
// kTelemetryDecim-th cycle (2 -> 250 Hz), and that is deliberate, not a
// convenience:
//
//   A 21-field corner line is ~170 B typical / ~200 B worst case. At 500 Hz
//   that is ~850 kbit/s of payload against ~800 kbit/s usable on the 1 Mbaud
//   Serial1 link -- over budget. The Teensy's TX buffer would back up and
//   Serial1 writes would start BLOCKING inside the 2 ms control cycle, which
//   is exactly the failure the non-blocking LineReader exists to prevent.
//   At 250 Hz it is ~340 kbit/s: ~42% utilization, with real margin for the
//   '#' diagnostic bursts and boot text.
//
//   250 Hz is still 12x the PC-side plotter's 20 fps redraw, and 2x the
//   fastest closed-loop transient this law produces. Do NOT raise this to
//   emit every cycle without also raising kLinkBaud here AND
//   TEENSY_LINK_BAUD in the XIAO sketch.
//
// --------------------------- LINK MODE ---------------------------
// gLinkMode selects which stream (Serial = USB, Serial1 = XIAO) telemetry
// goes OUT on and which one the watchdog watches. It does NOT gate which
// stream commands are read FROM -- both Serial and Serial1 are polled every
// loop() iteration, so a mode-switch command is never missed no matter what
// the board is currently doing. Switch it live with:
//   t0   -> USB mode   (telemetry/watchdog on Serial, no auto-disarm)
//   t1   -> WiFi mode  (telemetry/watchdog on Serial1)      [boot default]
// Recognized on EITHER stream, so you can always flip it from whichever
// channel is currently reachable.
//
// --------------------------- COMMAND GRAMMAR ---------------------------
//   a<0/1>   arm / disarm      (a1 still passes through the 0.5 deg arm gate)
//   g<0..1>  gain scale
//   c        re-resolve the corner candidate, answer echoed back
//   z<0/1>   z1 tares gPhiOffset to the CURRENT phi, z0 clears it
//   p<0/1>   HALT / resume     (idle: no IMU reads, no CAN traffic)
//   t<0/1>   link mode (USB / WiFi)
//   h  or  k no-op keepalive
//
// TWO GRAMMAR CHANGES vs the USB build, both forced by the transport:
//
//   1. HALT MOVED FROM 'h' TO 'p'. The USB build uses h1/h0 for halt, but
//      matlab/2Dmodel/Validation/telemetry_python_wifi.py sends a bare "h"
//      every 100 ms as its link keepalive. Flashing the USB grammar here
//      would halt the cube ten times a second. So 'h' (and 'k') are no-op
//      keepalives here, and halt is 'p' for pause.
//
//   2. HALT IS NOT 'x'. 'x' packets are consumed by the XIAO bridge itself
//      (x0 = WIFI_TEST, x1 = TEENSY_BRIDGE) and are NEVER relayed to the
//      Teensy -- see xiao_teensy_bridge.ino's pollUdpIn(). A halt bound to
//      'x' would be silently unreachable over WiFi while still working over
//      USB, which is the worst possible failure mode for a safety command.
//      Every letter other than 'x' relays verbatim.
//
// The USB build in ../../USBMODE/CornerBalance/ keeps h1/h0 unchanged. If you
// are switching between the two builds on the bench, that difference is the
// one thing to keep straight.
//
// Wiring to the XIAO (see xiao_teensy_bridge.ino):
//   Teensy Serial1 TX1 (pin 1)  -> XIAO D7 (RX)
//   Teensy Serial1 RX1 (pin 0)  <- XIAO D6 (TX)
//   Common GND between the two boards.
//   1 Mbaud. See TELEMETRY RATE above for the budget this implies.
//
// CHECKLIST — cube held by hand, gGainScale = 1.0 (unchanged):
//   Position near the resolved corner's equilibrium BEFORE sending "a1" --
//   if norm3(phi) doesn't settle under ARM_GATE (1.0 deg) even holding the
//   cube still at its natural rest point, that's the COM offset the USB
//   build's header describes, not a bad hold -- tare it with "z1" first,
//   then arm.
//   [ ] All three wheels unwind after each correction (watch the rho_lp
//       columns -- rho through a ~5s low-pass).
//   [ ] Standing speed near zero on all three -> healthy.
//   [ ] Standing speed nonzero and not settling on ONE wheel -> re-check
//       that wheel's gyro bias/sign before suspecting the coupled law.
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
// SERIALMONITORMODE: human-readable, tab-delimited, with header + "#" status
//                     lines -- for reading directly in a Serial Monitor (USB,
//                     or whatever terminal the XIAO's UDP bridge feeds).
//                     Byte-for-byte the USB build's format.
// PLOTMODE:           plain CSV, one line per emitted cycle -- for
//                      telemetry_python_wifi_corner.py. SAME 21 fields, in
//                      the SAME order as the tab-delimited header, so a
//                      PLOTMODE log and a SERIALMONITORMODE log carry
//                      identical information.
// Defaults to PLOTMODE: the whole point of the WiFi build is running with
// the PC-side plotter.
#define SERIALMONITORMODE 0
#define PLOTMODE 1
#define TELEMETRY_MODE PLOTMODE


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

// Emit telemetry every Nth control cycle. 2 -> 250 Hz. See the TELEMETRY RATE
// note in the header before changing this.
static const uint8_t kTelemetryDecim = 2;
static uint8_t gTelemetryCount = 0;

static uint32_t gNextSendMillis = 0;

static bool gHalted = false;

// ---------------- Link mode + command channels ----------------
enum class LinkMode : uint8_t { USB = 0, WIFI = 1 };
static LinkMode gLinkMode = LinkMode::WIFI;   // boot default -- see header

const uint32_t kLinkBaud      = 1000000;   // Serial1 <-> XIAO. Must match
                                            // TEENSY_LINK_BAUD in the XIAO
                                            // sketch.
const uint32_t kLinkTimeoutMs = 300;       // auto-disarm if WIFI mode and no
                                            // Serial1 line in this long
                                            // (telemetry_python_wifi_corner.py
                                            // sends a keepalive every 100 ms,
                                            // well under this)
static uint32_t gLastSerial1RxMillis = 0;

// Small non-blocking line accumulator -- used for BOTH Serial and Serial1 so
// neither channel can ever stall loop() waiting on bytes that haven't
// arrived yet. Returns true (outLine filled + NUL-terminated) at most once
// per call, i.e. bounded work per loop() iteration; queued lines are picked
// up on the following iteration(s).
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
// Only used outside the control path (setup(), and command echoes) -- per-
// cycle telemetry still goes to exactly one stream, chosen by gLinkMode, so
// the PC-side CSV parser never sees doubled data lines.
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


// ----------------------------------------------------------------------------
// SECTION 2b: STATE ESTIMATION -- gam (unchanged from the USB build)
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

// Takes the sink explicitly so the same check is readable over WiFi at boot.
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
// SECTION 2c: CORNER CANDIDATE (unchanged from the USB build)
// ----------------------------------------------------------------------------

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

// Same measurement-based resolution as the USB build; only the print sink is
// parameterised, so the 'c' command's answer comes back on whichever channel
// asked for it.
void resolveCornerCandidate(Print& out) {
  int bestIdx = 0, secondIdx = 0;
  float bestDot = -2.0f, secondDot = -2.0f;
  for (int i = 0; i < 8; ++i) {
    const float d = dot3(ghat, kCorners[i].gB);
    if (d > bestDot) { secondDot = bestDot; secondIdx = bestIdx; bestDot = d; bestIdx = i; }
    else if (d > secondDot) { secondDot = d; secondIdx = i; }
  }
  gCornerIdx = bestIdx;
  out.print("# corner resolved: "); out.print(kCorners[gCornerIdx].name);
  out.print("  place_offset="); out.print(kCorners[gCornerIdx].placeOffsetDeg, 3);
  out.print(" deg  (best_dot="); out.print(bestDot, 4);
  out.print(" runner_up="); out.print(kCorners[secondIdx].name);
  out.print(" dot="); out.print(secondDot, 4); out.println(")");
  if (bestDot - secondDot < 0.2f) {
    out.println("# WARNING: best and runner-up corners are close -- verify before arming.");
  }
}

float phi[3] = { 0.0f, 0.0f, 0.0f };

// Live-settable via "z1" (tare) / "z0" (clear) -- see the USB build's header
// for the full reasoning. Subtracted from the raw measurement below, so
// EVERYWHERE phi is used from here down (control, arm gate, trips,
// telemetry) already sees the corrected value.
static float gPhiOffset[3] = { 0.0f, 0.0f, 0.0f };

void updateCornerProjection() {
  float t[3];
  cross3(kCorners[gCornerIdx].gB, ghat, t);
  phi[0] = -t[0] - gPhiOffset[0];
  phi[1] = -t[1] - gPhiOffset[1];
  phi[2] = -t[2] - gPhiOffset[2];
}


// ----------------------------------------------------------------------------
// SECTION 2d: TELEMETRY
// ----------------------------------------------------------------------------
// Both functions take the destination Stream& explicitly -- loop() picks
// Serial or Serial1 each emitted cycle from gLinkMode. Field content and
// order are IDENTICAL between the two modes and identical to the USB build's
// tab-delimited header, so a PLOTMODE CSV and a SERIALMONITORMODE capture
// carry exactly the same 21 quantities:
//
//   t_ms,
//   phi_x_deg, phi_y_deg, phi_z_deg,
//   om_x_dps, om_y_dps, om_z_dps,
//   rho_x, rho_y, rho_z,
//   rho_x_lp, rho_y_lp, rho_z_lp,
//   tau_x, tau_y, tau_z,
//   tau_cmd_x, tau_cmd_y, tau_cmd_z,
//   armed, gain_scale
//
// telemetry_python_wifi_corner.py parses the PLOTMODE form (NUM_COLS = 21)
// and prints any '#'-prefixed line straight to its terminal as firmware
// console output, so command echoes and warnings stay visible without
// corrupting the data stream.

void printState(Stream& out, uint32_t t_ms, const float rho[3], const float rhoLp[3],
                const float tau[3], const float tauCmd[3], bool armed,
                float gainScale) {
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
  out.print('\t'); out.println(gainScale, 2);
}

void telemetryPlot(Stream& out, uint32_t t_ms, const float rho[3], const float rhoLp[3],
                   const float tau[3], const float tauCmd[3], bool armed,
                   float gainScale) {
  out.print(t_ms);
  out.print(','); out.print(phi[0] * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(phi[1] * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(phi[2] * (float)RAD_TO_DEG, 4);
  out.print(','); out.print(w_b[0] * (float)RAD_TO_DEG, 3);
  out.print(','); out.print(w_b[1] * (float)RAD_TO_DEG, 3);
  out.print(','); out.print(w_b[2] * (float)RAD_TO_DEG, 3);
  out.print(','); out.print(rho[0], 3);
  out.print(','); out.print(rho[1], 3);
  out.print(','); out.print(rho[2], 3);
  out.print(','); out.print(rhoLp[0], 3);
  out.print(','); out.print(rhoLp[1], 3);
  out.print(','); out.print(rhoLp[2], 3);
  out.print(','); out.print(tau[0], 5);
  out.print(','); out.print(tau[1], 5);
  out.print(','); out.print(tau[2], 5);
  out.print(','); out.print(tauCmd[0], 5);
  out.print(','); out.print(tauCmd[1], 5);
  out.print(','); out.print(tauCmd[2], 5);
  out.print(','); out.print(armed ? 1 : 0);
  out.print(','); out.println(gainScale, 3);
}


// ----------------------------------------------------------------------------
// SECTION 2e: CONTROL — full law: phi + om + rho, plus friction FF
// ----------------------------------------------------------------------------
// Byte-for-byte the same law, gains, trips and saturation as the USB build in
// ../../USBMODE/CornerBalance/. Do not retune here -- retune there and
// re-copy.

static bool  gArmed     = false;
static float gGainScale = 1.0f;   // already validated through Stage 3's ramp

// Loose this stage -- hand-held, watching combined-term behavior. Stage 5
// tightens to the real DISARM/OMEGA_CAP policy from cubli_gains.h.
static const float kMaxTilt    = 0.4363f;   // rad, 25 deg, vs norm3(phi)
// Velocity cap loosened from the 40 rad/s policy value, but not literally
// removed -- set to 2000 RPM, the motor's real mechanical speed rating, so
// there's still a genuine hardware ceiling behind it rather than
// "effectively infinite." TAU_MAX below is untouched and is still the real
// physical torque saturation.
static const float kMaxOmega   = 209.43951f;    // rad/s (2000 RPM, motor rating)
static const float kTauMax     = 0.12f;     // N*m, TAU_MAX
static const float kTaperStart = 36.0f;     // rad/s -- irrelevant now: with
                                              // kMaxOmega this large, the
                                              // taper's fade factor stays
                                              // ~1.0 for any real wheel speed

// Friction feedforward -- Firmware Lessons: "not optional". PLACEHOLDER
// values from cubli_gains.h pending the real spin-down/breakaway test.
static const float kTauCw  = 0.008f;   // N*m, PLACEHOLDER
static const float kBw     = 0.0f;     // N*m*s, PLACEHOLDER
static const float kEpsFf  = 0.05f;    // rad/s, tanh width

// Arm gate (cubli_gains.h's contract): refuse "a1" unless already near the
// resolved corner's equilibrium. Checked at the moment of arming, not
// continuously. Compared against norm3(phi).
static const float kArmGate = 0.01745329252f;   // rad, 1.0 deg -- widened from
                                                  // 0.5 deg for hand-held
                                                  // bring-up. DO NOT widen
                                                  // further: with TAU_MAX =
                                                  // 0.12 N*m and diagonal phi
                                                  // gains of ~8 N*m/rad, the
                                                  // phi term alone saturates
                                                  // the wheels at 0.015 rad
                                                  // (0.86 deg), so arming much
                                                  // past 1 deg starts the law
                                                  // already clipped -- no
                                                  // proportional authority
                                                  // left, and outside the
                                                  // linear regime the LQR
                                                  // gains were solved for.
                                                  // "z1" (tare) is still the
                                                  // fix for a COM offset; the
                                                  // gate is not the knob for
                                                  // a cube that rests degrees
                                                  // off equilibrium.

static const float kAxisWheelSign[3] = {
  1.0f,   // X -- CONFIRMED
  1.0f,   // Y -- CONFIRMED
  1.0f,   // Z -- CONFIRMED
};

// SetPosition()'s DEFAULT wire format only transmits position/velocity --
// every other Command field defaults to Resolution::kIgnore and is silently
// DROPPED before it reaches the CAN bus unless this Format turns it on.
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

static float gLastTau[3]      = { 0.0f, 0.0f, 0.0f };
static float gLastTauCmd[3]   = { 0.0f, 0.0f, 0.0f };
static float gRhoLp[3]        = { 0.0f, 0.0f, 0.0f };   // standing speed, tau = 5 s

Moteus& wheelObj(int i) {
  return i == 0 ? moteusX : (i == 1 ? moteusY : moteusZ);
}

void commandWheels(const float rho[3]) {
  // x = [phi(3); om(3); rho(3)] -- full state, all nine columns of each
  // wheel's Kp row now contribute.
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

    if (!isfinite(u)) { u = 0.0f; gArmed = false; }
    u = u >  kTauMax ?  kTauMax : u;
    u = u < -kTauMax ? -kTauMax : u;
    tau[i] = u;
  }

  if (norm3(phi) > kMaxTilt) { gArmed = false; }
  for (int i = 0; i < 3; ++i) {
    if (fabsf(rho[i]) > kMaxOmega) { gArmed = false; }
  }
  if (!isfinite(phi[0]) || !isfinite(phi[1]) || !isfinite(phi[2]) ||
      !isfinite(w_b[0]) || !isfinite(w_b[1]) || !isfinite(w_b[2])) {
    gArmed = false;
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
// Same a/g/c/z grammar as the USB build, with halt moved from 'h' to 'p'
// (see the header: 'h' must be a keepalive here, and 'x' never reaches this
// board), plus 'h'/'k' (keepalive no-op) and 't' (link mode -- handled in
// pollCommands() before this is called). `echo` is whichever stream the line
// arrived on, so an ack goes back down the channel it came in on.
//
// Echoes are '#'-prefixed and printed in BOTH telemetry modes, deliberately.
// telemetry_python_wifi_corner.py prints '#' lines to its terminal instead of
// trying to parse them, so a tare readback or an arm refusal is visible in
// PLOTMODE without polluting the CSV. (Note this differs from
// EdgeBalance_WiFi.ino, whose 10-column script has no such handling and so
// suppresses most echoes in PLOTMODE.)
void handleCommandLine(const char* line, Stream& echo) {
  if (line[0] == '\0') { return; }

  const char cmd = line[0];
  const float val = atof(line + 1);

  if (cmd == 'a') {
    if (val != 0.0f) {
      if (norm3(phi) < kArmGate) {
        gArmed = true;
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
  } else if (cmd == 'g') {
    gGainScale = val < 0.0f ? 0.0f : (val > 1.0f ? 1.0f : val);
    echo.print("# gGainScale = "); echo.println(gGainScale, 3);
  } else if (cmd == 'c') {
    resolveCornerCandidate(echo);
  } else if (cmd == 'z') {
    if (val != 0.0f) {
      gPhiOffset[0] += phi[0];
      gPhiOffset[1] += phi[1];
      gPhiOffset[2] += phi[2];
      echo.print("# gPhiOffset TARED to: ");
      echo.print(gPhiOffset[0] * (float)RAD_TO_DEG, 3); echo.print(",");
      echo.print(gPhiOffset[1] * (float)RAD_TO_DEG, 3); echo.print(",");
      echo.print(gPhiOffset[2] * (float)RAD_TO_DEG, 3); echo.println(" deg");
      if (norm3(gPhiOffset) > 0.1745329f) {   // ~10 deg
        echo.println("# NOTE: that's a large offset (>10 deg) -- expected for");
        echo.println("#   a missing-mass COM shift, but confirm this is really");
        echo.println("#   the cube resting naturally and not just held crooked.");
      }
    } else {
      gPhiOffset[0] = gPhiOffset[1] = gPhiOffset[2] = 0.0f;
      echo.println("# gPhiOffset cleared to 0,0,0");
    }
  } else if (cmd == 'p') {
    gHalted = (val != 0.0f);
    if (gHalted && gArmed) {
      gArmed = false;
      echo.println("# disarmed by halt");
    }
    echo.print("# gHalted = ");
    echo.println(gHalted ? "TRUE (idle -- no IMU reads, no CAN traffic)"
                          : "FALSE (resumed, still DISARMED -- send a1)");
  } else if (cmd == 'h' || cmd == 'k') {
    // keepalive / no-op -- receipt alone is enough (see pollCommands()).
    // Both letters accepted: 'h' keeps the python script working unmodified,
    // 'k' matches the wider WiFi-build grammar. NOTE: 'h' is HALT in the USB
    // build -- that is the one grammar difference between the two.
  } else {
    echo.println("# unknown. use: a<0/1>  g<0..1>  c (re-resolve)  "
                  "z<0/1> (clear/tare phi offset)  p<0/1> (halt)  "
                  "t<0/1> (link)  h|k (keepalive)");
  }
}

// Polls BOTH Serial and Serial1 every loop() iteration -- see LineReader
// above for why this can never block. 't'-prefixed lines switch gLinkMode
// locally and are NOT passed to handleCommandLine(); everything else
// (a/g/c/z/p/h/k) is, regardless of which stream it arrived on.
void pollCommands() {
  static char lineBuf[64];

  if (gUsbReader.poll(Serial, lineBuf, sizeof(lineBuf))) {
    if (lineBuf[0] == 't') {
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
    if (lineBuf[0] == 't') {
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
  // Bounded wait, NOT while(!Serial){} -- the USB build can afford to hang
  // there since a laptop is always attached; this build must still boot into
  // WIFI mode with nothing on USB at all.
  uint32_t usbWaitStart = millis();
  while (!Serial && millis() - usbWaitStart < 3000) { delay(10); }

  Serial1.begin(kLinkBaud);
  gLastSerial1RxMillis = millis();

  gBoot.println("\nstarted - CORNER BALANCE: FULL LAW (WiFi build, cube held by hand)");

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

#if TELEMETRY_MODE == SERIALMONITORMODE
  gBoot.println("t_ms\tphi_x_deg\tphi_y_deg\tphi_z_deg\t"
                 "om_x_dps\tom_y_dps\tom_z_dps\trho_x\trho_y\trho_z\t"
                 "rho_x_lp\trho_y_lp\trho_z_lp\t"
                 "tau_x\ttau_y\ttau_z\ttau_cmd_x\ttau_cmd_y\ttau_cmd_z\t"
                 "armed\tgain_scale");
#endif
  gBoot.println("# STARTS DISARMED. a1 refused unless norm3(phi) < ARM_GATE");
  gBoot.println("# (1.0 deg) -- get close to the resolved corner first. If it");
  gBoot.println("# won't settle under 1.0 deg even resting naturally (COM offset");
  gBoot.println("# without the battery/DC-DC mounted), send z1 to tare, z0 to clear.");
  gBoot.println("# Velocity cap loosened to 2000 RPM this stage -- a0 (disarm) is");
  gBoot.println("# the real safety net now, not a speed limit.");
  gBoot.println("# p1 halts (idle + disarm), p0 resumes. NOTE: halt is 'p' here,");
  gBoot.println("# not 'h' -- 'h'/'k' are the WiFi keepalive on this build.");
  gBoot.println("# t0 = USB link mode, t1 = WiFi link mode (default t1).");
  gBoot.println("# WiFi mode auto-disarms if no line arrives on Serial1 for 300 ms.");
  gBoot.println("# telemetry decimated to 250 Hz; control loop still runs at 500 Hz.");

  gNextSendMillis = millis();

}   // end of setup()


// ----------------------------------------------------------------------------
// SECTION 4: loop()
// ----------------------------------------------------------------------------

void loop() {

  pollCommands();

  // WiFi-mode link watchdog -- only enforced in WIFI mode, does exactly what
  // an explicit a0 does. USB mode never auto-disarms from this (matches the
  // USB build: only a0 or the control law's own trips do). Deliberately
  // checked BEFORE the halt return below, so a halted board still drops the
  // arm flag if the link dies while it is idle.
  if (gLinkMode == LinkMode::WIFI && gArmed &&
      millis() - gLastSerial1RxMillis > kLinkTimeoutMs) {
    gArmed = false;
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

  commandWheels(rho);

  // --- telemetry: decimated, and to Serial or Serial1 per gLinkMode ---
  // Every control cycle above ran in full; only the emission below is
  // throttled. See the TELEMETRY RATE note in the header for why.
  if (++gTelemetryCount >= kTelemetryDecim) {
    gTelemetryCount = 0;
    Stream& out = (gLinkMode == LinkMode::WIFI) ? (Stream&)Serial1 : (Stream&)Serial;
#if TELEMETRY_MODE == PLOTMODE
    telemetryPlot(out, time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gGainScale);
#else
    printState(out, time, rho, gRhoLp, gLastTau, gLastTauCmd, gArmed, gGainScale);
#endif
  }

}   // end of loop()


// ============================================================================
// NOTES
// ----------------------------------------------------------------------------
// Companion files:
//   ../../USBMODE/CornerBalance/CornerBalance.ino  -- the USB-tethered
//     reference build this is derived from, itself a byte-identical copy of
//     Gam/corner-bringup/Stage4_FullLaw/. Retune THERE, then re-copy the
//     changed constants here.
//   ../xiao_teensy_bridge/xiao_teensy_bridge.ino  -- the XIAO ESP32C6 sketch
//     that relays Serial1 <-> UDP. Shared with EdgeBalance_WiFi.ino, so only
//     ONE of the two WiFi builds can be on the link at a time.
//   ../telemetry_python_wifi_corner.py  -- the PC-side viewer/logger/command
//     console for this build's 21-column PLOTMODE CSV.
//   ../../plot_session_csv.py  -- offline plotter for the saved session CSV.
//
// Next: Gam/corner-bringup/Stage5_Release/ is the unsupported release
// attempt -- same law, trip policy tightened to the real cubli_gains.h values
// (DISARM = 15 deg on norm3(phi), OMEGA_CAP = 40 rad/s per wheel) with a
// latched trip reason. This folder deliberately stops at hand-held. A WiFi
// variant of Stage 5 is not written yet; when it is, it should reuse this
// file's link/watchdog/command layer verbatim.
//
// See ../README.md for the end-to-end flashing procedure.
// ============================================================================

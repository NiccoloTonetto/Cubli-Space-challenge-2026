// cubli_gains.h  -  GENERATED FILE, DO NOT EDIT BY HAND
//
// Checked into the repo so it's the single source of truth every stage file
// (edge-bringup, corner-bringup) draws its literal K/gB/ell/Sg/lambda values
// from, instead of each one hand-copying numbers. `cubli_export_gains.m`
// itself isn't in this repo yet (matlab/3Dmodel only has Validation/ so
// far) -- once it lands, regenerate this file from it rather than hand-
// editing, per the header below.
//
// Produced by cubli_export_gains.m on 2026-08-14 15:03
// Plant  : cubli_cube_params, mode measured, rho_scale 0.84, m_total 1.5668 kg
// LQR    : Bryson qa 0.200 rad, qr 2.000 rad/s, qw 10 rad/s, rt 0.240 N.m
// Filter : kP 4, kI 0.5
//
// STATE ORDER IS [phi(3); om(3); rho(3)], BODY AXES X,Y,Z, IN THAT ORDER.
// Build it directly. Do NOT apply the [1 4 2 5 3 6 7 8 9] permutation from
// the Simscape work - that is a model wiring artefact, not a firmware one.
//
// phi = -cross(gB, ghat).  Never feed the raw accelerometer to the gain:
// with the IMU 130 mm from the corner the lever-arm term reaches 88 % of
// the signal and the loop falls over every time.

#pragma once
#include <stdint.h>

namespace cubli {

// ---- loop, actuator and filter constants -----------------------------
static constexpr float FS_HZ      = 400.0f;   // control rate; 1 kHz was tested and buys nothing
static constexpr float DT         = 0.00249999994f;
static constexpr float KT         = 0.02513f;   // N.m/A
static constexpr float TAU_MAX    = 0.119999997f;   // N.m   ESTIMATE - thermal test pending
static constexpr float OMEGA_CAP  = 40.0f;   // rad/s FIRMWARE POLICY, not hardware
static constexpr float IS_WHEEL   = 0.00048687958f;   // kg.m^2 wheel about its spin axis
static constexpr float TAU_CW     = 0.00800000038f;   // N.m   PLACEHOLDER - spin-down test
static constexpr float B_W        = 0.0f;   // N.m.s PLACEHOLDER - spin-down test
static constexpr float EPS_FF     = 0.0500000007f;   // rad/s tanh width for the feedforward
static constexpr float KP_FILT    = 4.0f;   // HARD CLIFF between 6 and 7 - do not raise
static constexpr float KI_FILT    = 0.5f;   // bias update is  bhat += KI*e*dt  (PLUS)
static constexpr float ARM_GATE   = 0.00872664619f;   // rad, 0.5 deg: engage below this |phi|
static constexpr float DISARM     = 0.261799395f;   // rad, 15 deg: past recovery range
static constexpr float ID_SEP_MIN = 1.17295325f;   // rad, 67.2 deg between corner gB vectors

static constexpr int N_CORNERS = 8;
static constexpr int N_EDGES   = 12;

struct CornerGains {
    int8_t sign[3];     // which corner, e.g. {+1,+1,+1}
    float  gB[3];       // body-frame gravity direction at balance
    float  Kp[3][9];    // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]
    float  ell;         // m,     contact-to-COM lever arm
    float  Sg;          // N.m,   m*g*ell
    float  lambda;      // rad/s, open-loop unstable pole
    float  theta_eq;    // rad,   PLACEMENT lean vs the body diagonal
};

struct EdgeGains {
    int8_t axis;        // 0=X 1=Y 2=Z. ALSO THE INDEX OF THE ONLY ACTIVE WHEEL.
    int8_t sign[2];     // signs of the two transverse coords at the contact line
    float  e[3];        // edge direction, body frame
    float  gB[3];
    float  K[3];        // [phi_e, om_e, rho_axis]
                        // phi_e = dot(e, -cross(gB, ghat)),  om_e = dot(e, om)
    float  ell, Sg, lambda;
    float  place_offset;// rad, lean vs the face diagonal. Compare to recovery!
};

// ---- CORNERS ---------------------------------------------------------
static constexpr CornerGains CORNER[N_CORNERS] = {
  { // [-1,-1,-1]  ell 122.84 mm  Sg 1.8875  lambda +8.2572  lean 0.797 deg  Ms 0.999999  slowest -7.817
    { -1, -1, -1 },
    { -0.571549475f, -0.588645577f, -0.571688354f },
    {
      { -6.9157443f, 3.39226651f, 3.42117763f, -0.834802926f, 0.420730621f, 0.42704013f, -0.000700236589f, 0.00623514736f, 0.00621826435f },  // wheel X
      { 3.51941299f, -6.8296299f, 3.51364994f, 0.412131369f, -0.848057508f, 0.411711216f, 0.00382628222f, -0.00316504063f, 0.00381609239f },  // wheel Y
      { 3.42913675f, 3.39467001f, -6.92366505f, 0.426502436f, 0.419758797f, -0.837698817f, 0.00606757682f, 0.00607034843f, -0.000870343472f }   // wheel Z
    },
    0.12284115f, 1.88746154f, 8.2572031f, 0.0139029883f
  },
  { // [-1,-1,+1]  ell 128.54 mm  Sg 1.9750  lambda +8.1212  lean 3.170 deg  Ms 0.999999  slowest -7.629
    { -1, -1, 1 },
    { -0.546220303f, -0.562558711f, 0.620621562f },
    {
      { -7.58488846f, 3.33574939f, -3.65192771f, -0.921932399f, 0.430031687f, -0.480414748f, 0.000473975349f, 0.00748626003f, -0.00814931281f },  // wheel X
      { 3.43748641f, -7.47500134f, -3.7502768f, 0.421860248f, -0.928083122f, -0.467502952f, 0.00511831883f, -0.00179392833f, -0.00555475708f },  // wheel Y
      { -3.84645557f, -3.83494377f, -6.8614974f, -0.464526713f, -0.460435808f, -0.881195128f, -0.00357620185f, -0.00353527209f, -0.00314604398f }   // wheel Z
    },
    0.128537506f, 1.97498643f, 8.12116909f, 0.0553267486f
  },
  { // [-1,+1,-1]  ell 126.08 mm  Sg 1.9373  lambda +8.1591  lean 2.773 deg  Ms 0.999999  slowest -7.679
    { -1, 1, -1 },
    { -0.556852818f, 0.616181135f, -0.55698812f },
    {
      { -7.26747227f, -3.47609496f, 3.4201951f, -0.874630272f, -0.451983064f, 0.442295492f, 0.000827456824f, -0.00813719723f, 0.00776857371f },  // wheel X
      { -3.79863954f, -6.87362671f, -3.8063941f, -0.426038593f, -0.899968863f, -0.426554501f, -0.000622705789f, -0.00668813102f, -0.000633999531f },  // wheel Y
      { 3.41229653f, -3.47500682f, -7.25577497f, 0.442907125f, -0.45318532f, -0.871017814f, 0.00794851314f, -0.0083494084f, 0.00103329937f }   // wheel Z
    },
    0.126083225f, 1.93727612f, 8.15909767f, 0.0484051518f
  },
  { // [-1,+1,+1]  ell 131.64 mm  Sg 2.0226  lambda +8.0480  lean 3.097 deg  Ms 0.999999  slowest -7.580
    { -1, 1, 1 },
    { -0.533349514f, 0.590173781f, 0.605997622f },
    {
      { -8.23398495f, -3.56334782f, -3.77657819f, -1.06839609f, -0.416523129f, -0.442939252f, -0.00582132814f, -0.00140620384f, -0.00158187468f },  // wheel X
      { -3.41282964f, -7.33259392f, 4.13743114f, -0.429089457f, -0.917372108f, 0.514714897f, -0.00497917458f, -0.00120868487f, 0.00603242731f },  // wheel Y
      { -3.50851822f, 4.01196194f, -6.99511194f, -0.465059072f, 0.524895251f, -0.847355127f, -0.00789372995f, 0.00895721f, 0.00243827817f }   // wheel Z
    },
    0.131639361f, 2.02264667f, 8.04803944f, 0.054054521f
  },
  { // [+1,-1,-1]  ell 128.56 mm  Sg 1.9753  lambda +8.1180  lean 3.171 deg  Ms 0.999999  slowest -7.628
    { 1, -1, -1 },
    { 0.620658159f, -0.562471628f, -0.546268404f },
    {
      { -6.85864925f, -3.8295908f, -3.8494637f, -0.880151272f, -0.460793495f, -0.465634525f, -0.00304374471f, -0.00361368596f, -0.00366454106f },  // wheel X
      { -3.75562048f, -7.4855423f, 3.44052219f, -0.467026591f, -0.931139171f, 0.421110034f, -0.00539576123f, -0.00195194408f, 0.0049761422f },  // wheel Y
      { -3.65655398f, 3.33088136f, -7.58417654f, -0.481462985f, 0.429957539f, -0.921488523f, -0.00823030341f, 0.00754544372f, 0.000543692615f }   // wheel Z
    },
    0.128557414f, 1.97529233f, 8.11799717f, 0.0553516969f
  },
  { // [+1,-1,+1]  ell 134.01 mm  Sg 2.0591  lambda +7.9804  lean 2.609 deg  Ms 0.999999  slowest -7.552
    { 1, -1, 1 },
    { 0.595400333f, -0.539581716f, 0.595273018f },
    {
      { -7.30901575f, -3.4263885f, 4.20474958f, -0.895961821f, -0.451509207f, 0.553766072f, 0.00180089788f, -0.00741348462f, 0.00873730052f },  // wheel X
      { -3.81644511f, -8.42617893f, -3.82059836f, -0.41949293f, -1.11799276f, -0.419535488f, 0.00164802792f, -0.00882489327f, 0.00163950643f },  // wheel Y
      { 4.19695139f, -3.42353535f, -7.30109262f, 0.554382324f, -0.452106625f, -0.892860353f, 0.00891439989f, -0.00758054852f, 0.00199437374f }   // wheel Z
    },
    0.134011015f, 2.05908728f, 7.98038483f, 0.0455395654f
  },
  { // [+1,+1,-1]  ell 131.66 mm  Sg 2.0229  lambda +8.0501  lean 3.095 deg  Ms 0.999999  slowest -7.580
    { 1, 1, -1 },
    { 0.606037736f, 0.590086639f, -0.533400357f },
    {
      { -6.99898767f, 4.01700401f, -3.50819087f, -0.848652959f, 0.524742544f, -0.464503765f, 0.00232795905f, 0.00885933265f, -0.00779828522f },  // wheel X
      { 4.13386154f, -7.32801533f, -3.40998602f, 0.515303075f, -0.915354431f, -0.429458082f, 0.00615369575f, -0.00108131079f, -0.00508674001f },  // wheel Y
      { -3.77546f, -3.56726503f, -8.23596478f, -0.442555219f, -0.416409701f, -1.06887817f, -0.00154884392f, -0.00138305803f, -0.00584927434f }   // wheel Z
    },
    0.131658807f, 2.0229454f, 8.05012703f, 0.0540137999f
  },
  { // [+1,+1,+1]  ell 136.99 mm  Sg 2.1048  lambda +7.9341  lean 0.714 deg  Ms 0.999999  slowest -7.564
    { 1, 1, 1 },
    { 0.582457066f, 0.567126632f, 0.582332492f },
    {
      { -7.76383209f, 3.81748652f, 4.04768848f, -0.975789487f, 0.490672857f, 0.522951066f, -0.000524852891f, 0.00608501304f, 0.00639130361f },  // wheel X
      { 3.93991017f, -8.08397388f, 3.93213129f, 0.481777757f, -1.03972197f, 0.481087863f, 0.00368249253f, -0.00348382187f, 0.00367164146f },  // wheel Y
      { 4.05641413f, 3.81822968f, -7.77580976f, 0.522317231f, 0.489363909f, -0.97946316f, 0.00622009346f, 0.00590694463f, -0.000720091164f }   // wheel Z
    },
    0.136988997f, 2.10484409f, 7.93411636f, 0.0124670472f
  }
};

// ---- EDGES -----------------------------------------------------------
// SINGLE WHEEL. Only the wheel whose spin axis is PARALLEL to the edge makes
// torque about it; command the other two to zero. Three states, fully
// controllable - no yaw projection, no null-space design.
// Sorted by placement offset: the first entries are the easiest to set down.
// An edge whose place_offset exceeds its recovery angle WILL fall on release.
static constexpr EdgeGains EDGE[N_EDGES] = {
  { // Y[+1,+1]  ell 112.83 mm  Sg 1.7336  lambda 8.4739  placement offset 0.006 deg
    1, { 1, 1 },
    { 0.0f, 1.0f, 0.0f },
    { 0.707182407f, -0.0f, 0.70703119f },
    { -9.33804893f, -1.10871983f, -0.00692820316f },
    0.112828329f, 1.73361397f, 8.47385406f, 0.000106925152f
  },
  { // Y[-1,-1]  ell 99.30 mm  Sg 1.5258  lambda 8.8124  placement offset 0.007 deg
    1, { -1, -1 },
    { 0.0f, 1.0f, 0.0f },
    { -0.707020879f, -0.0f, -0.707192659f },
    { -8.03074265f, -0.918068051f, -0.00692820316f },
    0.0993037075f, 1.52580738f, 8.81237316f, 0.00012148777f
  },
  { // X[+1,+1]  ell 111.35 mm  Sg 1.7109  lambda 8.4623  placement offset 0.758 deg
    0, { 1, 1 },
    { 1.0f, 0.0f, 0.0f },
    { -0.0f, 0.697691619f, 0.716398239f },
    { -9.22453213f, -1.09680569f, -0.00692820316f },
    0.11135307f, 1.71094656f, 8.46230125f, 0.013227975f
  },
  { // Z[+1,+1]  ell 111.37 mm  Sg 1.7111  lambda 8.4622  placement offset 0.764 deg
    2, { 1, 1 },
    { 0.0f, 0.0f, 1.0f },
    { 0.716472805f, 0.697615027f, -0.0f },
    { -9.22558498f, -1.09693909f, -0.00692820316f },
    0.111365296f, 1.71113443f, 8.46223164f, 0.013334862f
  },
  { // X[-1,-1]  ell 100.80 mm  Sg 1.5488  lambda 8.7166  placement offset 0.837 deg
    0, { -1, -1 },
    { 1.0f, 0.0f, 0.0f },
    { -0.0f, -0.717363894f, -0.696698666f },
    { -8.2052536f, -0.948087275f, -0.00692820316f },
    0.100799464f, 1.54878974f, 8.71661282f, 0.0146130249f
  },
  { // Z[-1,-1]  ell 100.79 mm  Sg 1.5486  lambda 8.7172  placement offset 0.844 deg
    2, { -1, -1 },
    { 0.0f, 0.0f, 1.0f },
    { -0.696611583f, -0.717448473f, -0.0f },
    { -8.20397282f, -0.947880447f, -0.00692820316f },
    0.10078758f, 1.54860711f, 8.71716881f, 0.01473446f
  },
  { // X[-1,+1]  ell 107.67 mm  Sg 1.6543  lambda 8.5504  placement offset 2.809 deg
    0, { -1, 1 },
    { 1.0f, 0.0f, 0.0f },
    { -0.0f, -0.671598375f, 0.740915418f },
    { -8.86432362f, -1.04344738f, -0.00692820316f },
    0.107668363f, 1.65433073f, 8.55044651f, 0.0490341745f
  },
  { // Z[+1,-1]  ell 107.68 mm  Sg 1.6545  lambda 8.5504  placement offset 2.816 deg
    2, { 1, -1 },
    { 0.0f, 0.0f, 1.0f },
    { 0.740986824f, -0.671519518f, -0.0f },
    { -8.86540031f, -1.04358149f, -0.00692820316f },
    0.107680999f, 1.65452492f, 8.5503788f, 0.0491405837f
  },
  { // X[+1,-1]  ell 104.73 mm  Sg 1.6091  lambda 8.6214  placement offset 2.888 deg
    0, { 1, -1 },
    { 1.0f, 0.0f, 0.0f },
    { -0.0f, 0.741840661f, -0.670576215f },
    { -8.58001709f, -1.00194442f, -0.00692820316f },
    0.104726136f, 1.60912323f, 8.62139606f, 0.0504129156f
  },
  { // Z[-1,+1]  ell 104.71 mm  Sg 1.6089  lambda 8.6219  placement offset 2.895 deg
    2, { -1, 1 },
    { 0.0f, 0.0f, 1.0f },
    { -0.670486569f, 0.741921723f, -0.0f },
    { -8.57876492f, -1.00173855f, -0.00692820316f },
    0.104714692f, 1.60894752f, 8.62192249f, 0.0505337864f
  },
  { // Y[+1,-1]  ell 106.29 mm  Sg 1.6332  lambda 8.6358  placement offset 3.648 deg
    1, { 1, -1 },
    { 0.0f, 1.0f, 0.0f },
    { 0.750660002f, -0.0f, -0.660688698f },
    { -8.69853497f, -1.01400948f, -0.00692820316f },
    0.106293403f, 1.63320446f, 8.63583469f, 0.0636622831f
  },
  { // Y[-1,+1]  ell 106.27 mm  Sg 1.6328  lambda 8.6364  placement offset 3.648 deg
    1, { -1, 1 },
    { 0.0f, 1.0f, 0.0f },
    { -0.66067791f, -0.0f, 0.750669539f },
    { -8.69620609f, -1.01366949f, -0.00692820316f },
    0.10626933f, 1.63283455f, 8.63643742f, 0.0636767223f
  }
};

} // namespace cubli

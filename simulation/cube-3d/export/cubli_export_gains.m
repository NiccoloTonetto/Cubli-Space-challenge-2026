function cubli_export_gains(outfile, qa, qr, qw, rt, kPf, kIf)
%CUBLI_EXPORT_GAINS  Generate cubli_gains.h for the Teensy firmware.
%
%   cubli_export_gains                        defaults, writes ./cubli_gains.h
%   cubli_export_gains('firmware/include/cubli_gains.h')
%   cubli_export_gains(file, 0.20, 2.0, 10, 0.24, 4, 0.5)
%
%  Emits, for all 8 corners and all 12 edges:
%    gB   body-frame gravity direction at that equilibrium
%    Kp   3x9 corner gain, state order [phi(3); om(3); rho(3)], body X,Y,Z
%    K    1x3 edge gain,   state order [phi_e; om_e; rho_k], k = edge axis
%  plus geometry, placement offsets and the loop constants.
%
%  Ms is asserted to 1.000000 on every corner before anything is written, and
%  rank(ctrb) on every edge. If this refuses to run, fix the design - never
%  hand-edit the generated header.

if nargin<1||isempty(outfile), outfile='cubli_gains.h'; end
if nargin<2||isempty(qa),  qa=0.20; end
if nargin<3||isempty(qr),  qr=2.0;  end
if nargin<4||isempty(qw),  qw=10;   end
if nargin<5||isempty(rt),  rt=0.24; end
if nargin<6||isempty(kPf), kPf=4;   end
if nargin<7||isempty(kIf), kIf=0.5; end
AX = 'XYZ';                      % note: 'XYZ'(k) is Octave, not MATLAB
p  = cubli_cube_params;

% ---------------------------------------------------------------- corners
C = struct([]);
for n = 1:8
    c = cubli_corner_plant(p, n);
    [Kp,~,gi] = cubli_gains(p, c, qa, qr, qw, rt);
    assert(abs(gi.Ms-1) < 1e-5, ...
        'corner %d: Ms = %.6f, not 1. Implementation bug - not exporting.', n, gi.Ms);
    assert(gi.marginal == 2, ...
        'corner %d: %d marginal modes, expected 2 (yaw momentum + yaw angle).', n, gi.marginal);
    C(n).sign=c.sign; C(n).gB=c.gB; C(n).Kp=Kp; C(n).ell=c.ell; C(n).Sg=c.Sg;
    C(n).lambda=c.lambda(1); C(n).theta_eq=c.theta_eq; C(n).Ms=gi.Ms; C(n).slow=gi.slowest;
end

% ------------------------------------------------------------------ edges
E = struct([]); m = 0; I3 = eye(3);
for a = 1:3
  for s1 = [-1 1]
    for s2 = [-1 1]
        m  = m + 1;  e = I3(:,a);
        ed = cubli_edge_plant(p, e, [s1 s2]);
        assert(ed.rank_ctrb == 3, 'edge %d: rank(ctrb) = %d, expected 3', m, ed.rank_ctrb);
        K  = lqr(ed.A, ed.B, diag([1/qa^2 1/qr^2 1/qw^2]), 12/rt^2);
        t  = setdiff(1:3, ed.k);  sym = zeros(3,1);  sym(t) = [s1;s2]/sqrt(2);
        E(m).axis  = ed.k - 1;        % 0-based; ALSO the index of the active wheel
        E(m).sign  = [s1 s2];  E(m).e = e;  E(m).gB = ed.gB;  E(m).K = K(:).';
        E(m).ell   = ed.ell;   E(m).Sg = ed.Sg;  E(m).lambda = ed.lambda;
        E(m).place = acos(max(-1, min(1, ed.gB.'*sym)));
    end
  end
end

sep = inf;                        % corner-ID margin
for i=1:8, for j=i+1:8
    sep = min(sep, acos(max(-1, min(1, C(i).gB.'*C(j).gB))));
end, end

% ------------------------------------------------------------------ write
f = fopen(outfile,'w');  assert(f>0, 'cannot open %s', outfile);
w = @(varargin) fprintf(f, varargin{:});
w('// cubli_gains.h  -  GENERATED FILE, DO NOT EDIT BY HAND\n//\n');
w('// Produced by cubli_export_gains.m on %s\n', datestr(now,'yyyy-mm-dd HH:MM'));
w('// Plant  : cubli_cube_params, mode %s, rho_scale %.2f, m_total %.4f kg\n', ...
    p.mode, p.rho_scale, p.m_total);
w('// LQR    : Bryson qa %.3f rad, qr %.3f rad/s, qw %g rad/s, rt %.3f N.m\n', qa,qr,qw,rt);
w('// Filter : kP %g, kI %g\n//\n', kPf, kIf);
w('// STATE ORDER IS [phi(3); om(3); rho(3)], BODY AXES X,Y,Z, IN THAT ORDER.\n');
w('// Build it directly. Do NOT apply the [1 4 2 5 3 6 7 8 9] permutation from\n');
w('// the Simscape work - that is a model wiring artefact, not a firmware one.\n//\n');
w('// phi = -cross(gB, ghat).  Never feed the raw accelerometer to the gain:\n');
w('// with the IMU 130 mm from the corner the lever-arm term reaches 88 %% of\n');
w('// the signal and the loop falls over every time.\n');
w('\n#pragma once\n#include <stdint.h>\n\nnamespace cubli {\n\n');

w('// ---- loop, actuator and filter constants -----------------------------\n');
w('static constexpr float FS_HZ      = %sf;   // control rate; 1 kHz was tested and buys nothing\n', fl(p.f_outer));
w('static constexpr float DT         = %sf;\n',  fl(1/p.f_outer));
w('static constexpr float KT         = %sf;   // N.m/A\n', fl(p.Kt));
w('static constexpr float TAU_MAX    = %sf;   // N.m   ESTIMATE - thermal test pending\n', fl(p.tau_cont));
w('static constexpr float OMEGA_CAP  = %sf;   // rad/s FIRMWARE POLICY, not hardware\n', fl(p.omega_cap));
w('static constexpr float IS_WHEEL   = %sf;   // kg.m^2 wheel about its spin axis\n', fl(p.Is));
w('static constexpr float TAU_CW     = %sf;   // N.m   PLACEHOLDER - spin-down test\n', fl(p.tau_cw));
w('static constexpr float B_W        = %sf;   // N.m.s PLACEHOLDER - spin-down test\n', fl(p.b_w));
w('static constexpr float EPS_FF     = %sf;   // rad/s tanh width for the feedforward\n', fl(p.eps_ff));
w('static constexpr float KP_FILT    = %sf;   // HARD CLIFF between 6 and 7 - do not raise\n', fl(kPf));
w('static constexpr float KI_FILT    = %sf;   // bias update is  bhat += KI*e*dt  (PLUS)\n', fl(kIf));
w('static constexpr float ARM_GATE   = %sf;   // rad, 0.5 deg: engage below this |phi|\n', fl(deg2rad(0.5)));
w('static constexpr float DISARM     = %sf;   // rad, 15 deg: past recovery range\n', fl(deg2rad(15)));
w('static constexpr float ID_SEP_MIN = %sf;   // rad, %.1f deg between corner gB vectors\n', fl(sep), rad2deg(sep));
w('\nstatic constexpr int N_CORNERS = 8;\nstatic constexpr int N_EDGES   = 12;\n\n');

w('struct CornerGains {\n');
w('    int8_t sign[3];     // which corner, e.g. {+1,+1,+1}\n');
w('    float  gB[3];       // body-frame gravity direction at balance\n');
w('    float  Kp[3][9];    // rows = wheel X,Y,Z; cols = [phi(3) om(3) rho(3)]\n');
w('    float  ell;         // m,     contact-to-COM lever arm\n');
w('    float  Sg;          // N.m,   m*g*ell\n');
w('    float  lambda;      // rad/s, open-loop unstable pole\n');
w('    float  theta_eq;    // rad,   PLACEMENT lean vs the body diagonal\n');
w('};\n\n');
w('struct EdgeGains {\n');
w('    int8_t axis;        // 0=X 1=Y 2=Z. ALSO THE INDEX OF THE ONLY ACTIVE WHEEL.\n');
w('    int8_t sign[2];     // signs of the two transverse coords at the contact line\n');
w('    float  e[3];        // edge direction, body frame\n');
w('    float  gB[3];\n');
w('    float  K[3];        // [phi_e, om_e, rho_axis]\n');
w('                        // phi_e = dot(e, -cross(gB, ghat)),  om_e = dot(e, om)\n');
w('    float  ell, Sg, lambda;\n');
w('    float  place_offset;// rad, lean vs the face diagonal. Compare to recovery!\n');
w('};\n\n');

w('// ---- CORNERS ---------------------------------------------------------\n');
w('static constexpr CornerGains CORNER[N_CORNERS] = {\n');
for n = 1:8
    w('  { // [%+d,%+d,%+d]  ell %.2f mm  Sg %.4f  lambda %+.4f  lean %.3f deg  Ms %.6f  slowest %+.3f\n', ...
        C(n).sign, C(n).ell*1e3, C(n).Sg, C(n).lambda, rad2deg(C(n).theta_eq), C(n).Ms, C(n).slow);
    w('    { %d, %d, %d },\n', C(n).sign);
    w('    { %sf, %sf, %sf },\n', fl(C(n).gB(1)), fl(C(n).gB(2)), fl(C(n).gB(3)));
    w('    {\n');
    for i = 1:3
        row = strjoin(arrayfun(@(v) [fl(v) 'f'], C(n).Kp(i,:), 'UniformOutput', false), ', ');
        if i < 3, tail = ','; else, tail = ' '; end
        w('      { %s }%s  // wheel %c\n', row, tail, AX(i));
    end
    w('    },\n');
    w('    %sf, %sf, %sf, %sf\n  }%s\n', fl(C(n).ell), fl(C(n).Sg), fl(C(n).lambda), ...
        fl(C(n).theta_eq), comma(n,8));
end
w('};\n\n');

w('// ---- EDGES -----------------------------------------------------------\n');
w('// SINGLE WHEEL. Only the wheel whose spin axis is PARALLEL to the edge makes\n');
w('// torque about it; command the other two to zero. Three states, fully\n');
w('// controllable - no yaw projection, no null-space design.\n');
w('// Sorted by placement offset: the first entries are the easiest to set down.\n');
w('// An edge whose place_offset exceeds its recovery angle WILL fall on release.\n');
w('static constexpr EdgeGains EDGE[N_EDGES] = {\n');
[~,ord] = sort([E.place]);  q = 0;
for m = ord
    q = q + 1;
    w('  { // %c[%+d,%+d]  ell %.2f mm  Sg %.4f  lambda %.4f  placement offset %.3f deg\n', ...
        AX(E(m).axis+1), E(m).sign, E(m).ell*1e3, E(m).Sg, E(m).lambda, rad2deg(E(m).place));
    w('    %d, { %d, %d },\n', E(m).axis, E(m).sign);
    w('    { %sf, %sf, %sf },\n', fl(E(m).e(1)),  fl(E(m).e(2)),  fl(E(m).e(3)));
    w('    { %sf, %sf, %sf },\n', fl(E(m).gB(1)), fl(E(m).gB(2)), fl(E(m).gB(3)));
    w('    { %sf, %sf, %sf },\n', fl(E(m).K(1)),  fl(E(m).K(2)),  fl(E(m).K(3)));
    w('    %sf, %sf, %sf, %sf\n  }%s\n', fl(E(m).ell), fl(E(m).Sg), fl(E(m).lambda), ...
        fl(E(m).place), comma(q,12));
end
w('};\n\n} // namespace cubli\n');
fclose(f);

fprintf('wrote %s\n', outfile);
fprintf('  8 corners : Ms = 1.000000 on all, 2 marginal modes each\n');
fprintf('  12 edges  : rank(ctrb) = 3 on all\n');
fprintf('  %d floats of gain data\n', 8*(3+27) + 12*(3+3+3));
fprintf('  easiest edge to place : %c[%+d,%+d] at %.3f deg\n', ...
    AX(E(ord(1)).axis+1), E(ord(1)).sign, rad2deg(E(ord(1)).place));
[~,iw] = max([C.theta_eq]);
fprintf('  worst corner to place : [%+d,%+d,%+d], lean %.3f deg\n', ...
    C(iw).sign, rad2deg(C(iw).theta_eq));
end

function s = fl(v)
%FL  float literal with enough digits to round-trip single precision
s = sprintf('%.9g', double(single(v)));
if ~contains(s,'.') && ~contains(s,'e') && ~contains(s,'E'), s = [s '.0']; end
end

function s = comma(i,n)
if i < n, s = ','; else, s = ''; end
end

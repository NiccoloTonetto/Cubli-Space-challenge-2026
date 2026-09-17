function p = cubli_cube_params(rho_scale)
%CUBLI_CUBE_PARAMS  Plant parameters for the 3D corner-balancing cube.
%
%   p = cubli_cube_params()      measured build, default print scale 0.84
%   p = cubli_cube_params(0.90)  different assumed print density for the frame
%   p = cubli_cube_params('cad') CATIA nominal, for comparison
%
% =========================================================================
% MEASURED, 10/08/2026
%   wheel with ballast          216 g   (x3, all identical)
%   wheel without ballast        86 g   -> ballast = 130 g
%   whole build without battery, 3 moteus, Teensy, DC-DC:  1257 g
%   inner cage ("power house") without electronics:  ~186 g
%
%   The WHEEL is fully determined by the two weighings.
%   The FRAME total is pinned at 609 g (= 1257 - 3*216) for the printed
%   structure + bought parts + unmodelled fasteners; only the SPLIT between
%   printed plastic and fasteners is assumed, via rho_scale.
% =========================================================================
%
% WHAT THE MEASUREMENTS REVEALED
%
%  1. The printed wheel is at 1087 kg/m^3 = 84 % of PETG-CF nominal, i.e.
%     nearly solid. Sensible for a reaction wheel - mass at radius is the
%     point - but very different from the panel, which came out at 24-34 %.
%     The frame is assumed to print at the same 84 %.
%
%  2. The ballast measures 130 g over a CAD volume of 19.83 cm^3, i.e.
%     6.56 g/cm^3 - BELOW steel, which is impossible for nuts and bolts.
%     CAD models threaded fasteners as PLAIN CYLINDERS; real threads remove
%     ~15 % of the volume. Correcting: 130/(0.85*19.83) = 7.71 g/cm^3.
%     The CSV's 6860 kg/m^3 was CATIA compensating for this in the wrong
%     place. ANY CAD fastener volume in this project is ~15 % overstated.
%
%  3. The frame side reconciles to +6.9 g against CAD at nominal density,
%     so the unmodelled fastener mass is modest - 63 g at rho_scale = 0.84.
%     lambda moves only 0.5 % across the whole plausible band, so this
%     assumption does not affect the gains.
%
% GEOMETRY  3D_assembly_FRAME.stpZ, 3D_assembly_WHEEL_X.stpZ (AP242, mm)
%           Origin at the CUBE GEOMETRIC CENTRE, cube spans -75..+75 mm.
%
% SIGN CONVENTION  CATIA reports P = +int(x*y)dm; this file and Simscape use
%           the standard tensor, off-diagonal = -int(x*y)dm. Recomputing the
%           frame from geometry reproduced CATIA's magnitudes with the
%           OPPOSITE sign, confirming it. Values below are already correct.
%
% CONVENTION (3D corner balance)
%   x = [phi(3); omega(3); wheel_rate(3)]                       9 states
%   Tb*omegadot = Sg*P*phi - u,   Is*(a_k.omegadot + phiddot_k) = u_k
%   Tb = Theta - Aw*Is*Aw',   P = I - gB*gB'

if nargin < 1 || isempty(rho_scale)
    rho_scale = 0.84;  p.mode = 'measured';
elseif ischar(rho_scale) || isstring(rho_scale)
    rho_scale = 1.00;  p.mode = 'cad';
else
    rho_scale = double(rho_scale);  p.mode = 'measured-swept';
end
p.rho_scale   = rho_scale;
p.rho_plastic = 1290 * rho_scale;

% ---------------------------------------------------------------- constants
p.g      = 9.80665;             % MUST match Mechanism Configuration
p.a      = 0.150;
%% --- BALANCING CORNER ------------------------------------------------
%% CHANGED 12/08/2026: moved to the corner BETWEEN THE WHEELS.
%% The 0.45 mm truncation on that corner has been restored to a point, so
%% the contact IS the exact geometric corner - no offset needed. (A 0.45 mm
%% edge cut is only 0.26 mm along the diagonal, theta_c = 0.081 deg, i.e.
%% below the 0.3 deg estimator noise floor. It was never contributing.)
%%
%% OLD, corner away from the wheels:
%% p.corner = [0.075; 0.075; 0.075];   %% (+1,+1,+1)  ell 136.99, lambda 7.934
p.corner = [-0.075; -0.075; -0.075];   %% (-1,-1,-1)  ell 122.84, lambda 8.257

% measured constraints
M_FRAME_SIDE = 0.609;           % = 1257 g - 3*216 g
M_WHEEL      = 0.216;
M_BALLAST    = 0.130;
M_WHEEL_BARE = 0.086;           % includes the shaft

% ------------------------------------------------- electronics, real masses
p.m_battery = 0.245;            % Tattu 6S 1550 R-Line  (CAD said 0.2541)
p.m_moteus  = 0.0438;           % 3 x 14.6 g
p.m_teensy  = 0.011;            % CAD said 7.1 g
p.m_dcdc    = 0.010;            % LM2596, NOT in the CAD

% =========================================================================
% GROUP DATA from the STEP at CSV densities.  mass kg, COM m in cube-centre
% coordinates, inertia kg m^2 about that group's own COM.
% =========================================================================

% FRAME - A: printed (CUBI, beams, corners, brackets, sleeves), V = 270.09 cm^3
A_M   = 0.348414;
A_COM = [0.001099; 0.001105; 0.001111];
A_I   = [1.5186837617e-03, 7.7048568716e-05, 7.6605978451e-05
         7.7048568716e-05, 1.5187585475e-03, 7.6831527707e-05
         7.6605978451e-05, 7.6831527707e-05, 1.5186957459e-03];

% FRAME - B: bought parts present in the 1257 g weighing
%            3 motors, 3 MA600, 3 magnets, 3 shafts, IMU, XIAO, perf, XT90
B_M   = 0.253700;
B_COM = [-0.003451; -0.010234; -0.005834];
B_I   = [4.4265588135e-04, 7.8025198222e-05, 1.6131955113e-05
         7.8025198222e-05, 5.2128453759e-04, 8.2179666862e-05
         1.6131955113e-05, 8.2179666862e-05, 4.8421019468e-04];

% FRAME - C: excluded from the weighing (battery, moteus, Teensy) + DC-DC
C_M0  = 0.305000;               % CAD mass, used only to scale C_I
C_COM = [0.010543; 0.026712; 0.012567];
C_I0  = [3.0381402879e-04, 1.8275451313e-05, 2.4511867411e-05
         1.8275451313e-05, 1.9402948999e-04, 1.8786191831e-05
         2.4511867411e-05, 1.8786191831e-05, 2.8171667326e-04];

% WHEEL - printed body, V = 76.424 cm^3
WPL_M   = 0.098587;
WPL_COM = [-0.064403; 0.007; 0.007];
WPL_I   = diag([2.001783e-04, 1.028595e-04, 1.028590e-04]);
% WHEEL - ballast, 16 nuts + 16 screws at r = 47.7 mm, V = 19.826 cm^3
WBA_M   = 0.136009;
WBA_COM = [-0.060100; 0.007; 0.007];
WBA_I   = diag([3.328681e-04, 1.672022e-04, 1.672022e-04]);
% WHEEL - shaft
WSH_M   = 0.002543;
WSH_COM = [-0.050250; 0.007; 0.007];
WSH_I   = diag([5.086113e-09, 1.869676e-07, 1.869676e-07]);

% ================================================= WHEEL - fully measured
s_shaft = 7850/6860;                                   % real steel
s_pl    = (M_WHEEL_BARE - WSH_M*s_shaft) / WPL_M;      % 0.8428 -> 1087 kg/m^3
s_ba    = M_BALLAST / WBA_M;                           % 0.9558, thread correction

[m_wx, c_wx, I_wx] = combine( ...
    {s_pl*WPL_M,    WPL_COM, s_pl*WPL_I}, ...
    {s_ba*WBA_M,    WBA_COM, s_ba*WBA_I}, ...
    {s_shaft*WSH_M, WSH_COM, s_shaft*WSH_I});

p.m_wheel    = m_wx;
p.axis_wheel = eye(3);
for k = 1:3                       % cyclic permutation x -> y -> z -> x
    R = circshift(eye(3), k-1, 1);
    p.com_wheel{k} = circshift(c_wx, k-1);
    p.I_wheel{k}   = R*I_wx*R.';
end
p.Is = p.I_wheel{1}(1,1);
p.It = p.I_wheel{1}(2,2);
p.rot_wx = +pi/2;                 % Rigid transform: about +Y, Z -> X
p.rot_wy = -pi/2;                 % about +X, Z -> Y   (WHEEL_Z needs none)

% --- axis position, for the T_W*_POS Rigid Transforms ----------------------
% A point ON wheel k's spin axis, in cube-centre coordinates. Numerically
% IDENTICAL to com_wheel{k}, because the wheel is balanced on its axis by
% design (CATIA puts its COM within 1e-8 mm of the axis).
%
% Kept as a SEPARATE name because the two are different quantities that
% happen to coincide:
%   p.r_wheel{k}   -> T_W*_POS translation           (a geometric location)
%   p.com_wheel{k} -> File Solid Custom Centre of Mass (a mass property)
% If a wheel ever becomes unbalanced, r_wheel stays on the axis and com_wheel
% moves. Using one for both would then silently misplace the joint.
%
% NOTE the axis is at (+7, +7) mm off the cube axes - almost certainly
% battery clearance on the body diagonal. It is NOT [d, 0, 0].
for k = 1:3
    p.r_wheel{k} = p.com_wheel{k};
end
p.off  = p.r_wheel{1}(2);         % +0.007 m, axis offset from the cube axes
p.d_ax = p.r_wheel{1}(1);         % -0.0616 m, position along the wheel's axis

% ================================================= FRAME
% Fasteners not present in the CAD, sized to close the measured 609 g.
p.m_extra = M_FRAME_SIDE - rho_scale*A_M - B_M;
if p.m_extra < 0
    warning('rho_scale %.2f implies negative fastener mass', rho_scale);
end
m_C = p.m_battery + p.m_moteus + p.m_teensy + p.m_dcdc;

[p.m_frame, p.com_frame, p.I_frame] = combine( ...
    {rho_scale*A_M, A_COM, rho_scale*A_I}, ...
    {B_M,           B_COM, B_I}, ...
    {m_C,           C_COM, C_I0*(m_C/C_M0)}, ...
    {p.m_extra,     A_COM, zeros(3)});      % extras at the printed centroid

% ================================================= support pole
% ADDED 19/08/2026. The frame bent measurably under load when balancing on
% the corner, so a strut was fitted from the balancing corner to the
% electronics housing. Whole-build mass went 1.5668 -> 1.6330 kg (+66.2 g),
% which is this member.
%
% It runs 4.49 deg off the balancing diagonal - i.e. essentially ALONG the
% load path - so it adds mass but SHORTENS the lever arm, and the two
% effects partly cancel: +4.2 %% mass, -1.4 %% ell, net +2.7 %% on Sg and
% only +0.4 %% on lambda.
%
% Modelled as a uniform slender rod. Endpoints ASSUMED: balancing corner to
% the electronics group centroid C_COM. If the real routing differs, change
% POLE_A / POLE_B - the mass is measured, the geometry is not.
p.m_pole = 1.6330 - (M_FRAME_SIDE + m_C + 3*p.m_wheel);   %% closes the weighing
POLE_A   = p.corner;                     %% balancing corner end
POLE_B   = C_COM;                        %% electronics housing end
p.pole_L = norm(POLE_B - POLE_A);
p.com_pole = 0.5*(POLE_A + POLE_B);
u_pole   = (POLE_B - POLE_A)/p.pole_L;
p.I_pole = (p.m_pole*p.pole_L^2/12)*(eye(3) - u_pole*u_pole.');  %% slender rod

% ================================================= assembled body
% FOLD THE STRUT INTO THE FRAME BODY.  21/08/2026
% The strut does not rotate, so it is part of the non-rotating structure.
% Carrying it as a SEPARATE body made p.m_total exceed p.m_frame+3*p.m_wheel
% by 66.2 g, and the Simscape model - which has only four solids and reads
% p.m_frame / p.m_wheel - therefore built a 1.5668 kg cube while the gains
% were designed for 1.6330 kg.  That 4 % mismatch pushed the two marginal
% modes (conserved yaw momentum, yaw angle) to +0.145 and +0.039 rad/s and
% the closed loop drifted away over ~20 s.
%
% OLD, strut as a fifth body:
%   bodies = {{p.m_frame, p.com_frame, p.I_frame}};
%   bodies{end+1} = {p.m_pole, p.com_pole, p.I_pole};
[p.m_frame, p.com_frame, p.I_frame] = combine( ...
    {p.m_frame, p.com_frame, p.I_frame}, ...
    {p.m_pole,  p.com_pole,  p.I_pole});
bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for k = 1:3
    bodies{end+1} = {p.m_wheel, p.com_wheel{k}, p.I_wheel{k}};   %#ok<AGROW>
end
p.m_total = 0;  mc = zeros(3,1);
for i = 1:numel(bodies)
    p.m_total = p.m_total + bodies{i}{1};
    mc = mc + bodies{i}{1}*bodies{i}{2};
end
p.com = mc / p.m_total;

p.Theta = zeros(3);               % about the CONTACT CORNER, bodies rigid
for i = 1:numel(bodies)
    d = bodies{i}{2} - p.corner;
    p.Theta = p.Theta + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
end

% ------------------------------------------------------- plant scalars
r        = p.com - p.corner;
p.ell    = norm(r);
p.gB     = -r/p.ell;
p.S      = p.m_total * p.ell;
p.Sg     = p.S * p.g;
p.Aw     = p.axis_wheel;
p.Is_mat = p.Is * eye(3);
p.Tb     = p.Theta - p.Aw*p.Is_mat*p.Aw.';
p.P      = eye(3) - p.gB*p.gB.';

% ------------------------------------------------------- mount rotation
tgt = [0; -1; 0];
ax  = cross(p.gB, tgt);
p.mount_axis  = ax / norm(ax);
p.mount_angle = acos(max(-1, min(1, dot(p.gB, tgt))));   % RAD

% ------------------------------------------------------- state space
Z = zeros(3);  I3 = eye(3);
G = p.Tb \ (p.Sg * p.P);
p.A = [ Z  I3  Z ;  G  Z  Z ; -G  Z  Z ];
p.B = [ Z ; -inv(p.Tb) ; inv(p.Is_mat) + inv(p.Tb) ];
p.C = eye(9);
p.D = zeros(9,3);
ev       = eig(p.A);
p.lambda = sort(real(ev(real(ev) > 1e-6)), 'descend');

% ------------------------------------------------------- actuation
p.Kt        = 0.02513;
p.tau_cont  = 0.12;               % N m  ESTIMATE - thermal, no airflow
p.tau_peak  = 0.40;
p.omega_max = 8436*2*pi/60;
p.omega_cap = 40;                 % rad/s  FIRMWARE POLICY, not hardware
p.h_cap     = p.Is * p.omega_cap;
p.tau_eff   = sqrt(2) * p.tau_cont;      % worst case over corner tilt axes
p.theta_static = asin(min(1, p.tau_eff/p.Sg));

% ------------------------------------------------------- friction
p.tau_cw = 8e-3;                  % N m   PLACEHOLDER - spin-down test
p.b_w    = 0;                     %       PLACEHOLDER - spin-down test
p.tau_cp = 0;                     %       PLACEHOLDER - breakaway test
p.eps_ff = 0.05;                  % rad/s tanh width for the feedforward

% ------------------------------------------------------- sim settings
p.f_outer  = 400;
p.enc_bits = 16;
p.enc_lsb  = 2*pi/2^p.enc_bits;

% ------------------------------------------- equilibria, all eight corners
n = 0;
for sx = [-1 1], for sy = [-1 1], for sz = [-1 1]
    n = n + 1;
    cc = [sx; sy; sz]*p.a/2;
    vv = p.com - cc;
    p.corners(n).sign     = [sx sy sz];
    p.corners(n).pos      = cc;
    p.corners(n).ell      = norm(vv);
    p.corners(n).gB       = -vv/norm(vv);
    u  = cc/norm(cc);
    da = dot(p.com, u);
    p.corners(n).theta_eq = atan(norm(p.com - da*u)/(sqrt(3)/2*p.a - da));
end, end, end

% =========================================================================
% EXPECTED VALUES at rho_scale = 0.84
%   m_wheel   0.216000 kg   Is 4.8688e-04   It 2.4804e-04   (CAD Is -8.7 %)
%   m_frame   0.918800 kg   extras 62.6 g
%   m_total   1.5668 kg          (CAD 1.6185, -3.2 %)
%   com       [-4.790, -2.690, -4.773] mm
%   ell       136.99 mm     Sg 2.1048 N m
%   lambda    +7.9152, +7.9341 rad/s      (uniform cube 0.99*sqrt(g/a)=8.01)
%   theta_eq  0.714 deg on (+1,+1,+1)
%   mount     124.5501 deg about [+0.707031, 0, -0.707182]
%   h_cap     0.019475 N m s at 40 rad/s
%   rank(ctrb) = 8 of 9   - yaw momentum is CONSERVED, see the decisions note
%
% STILL OPEN
%   - plastic/fastener split in the frame is assumed (rho_scale). lambda
%     moves 0.5 % across the band, Theta and the COM move more.
%     RESOLVE BY MEASUREMENT: plumb-line the COM and swing-test Theta.
%     Weighing the bare printed structure would also close it exactly.
%   - tau_cont = 0.12 N m is an ESTIMATE. Thermal test.
%   - tau_cw, b_w, tau_cp are placeholders. Spin-down and breakaway tests.
%   - motor rotors sit in the FRAME group and so do not rotate in the model.
%     Worth ~2-3 % on each wheel inertia.
%   - battery 245 g and DC-DC 10 g are estimates. Weigh them.
% =========================================================================
end

% -------------------------------------------------------------------------
function [M, C, I] = combine(varargin)
%COMBINE  Merge N groups, each {mass, com, inertia-about-own-com}.
    M = 0;  mc = zeros(3,1);
    for i = 1:nargin
        M  = M  + varargin{i}{1};
        mc = mc + varargin{i}{1}*varargin{i}{2};
    end
    C = mc / M;
    I = zeros(3);
    for i = 1:nargin
        d = varargin{i}{2} - C;
        I = I + varargin{i}{3} + varargin{i}{1}*((d.'*d)*eye(3) - d*d.');
    end
end
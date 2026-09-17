function p = cubli_cube_params3N(rho_scale)
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
    % CHANGED 18/08/2026: A_M is the AS-PRINTED mass, not CAD at nominal
    % density, so there is nothing left for rho_scale to correct.
    % OLD:  rho_scale = 0.84;
    rho_scale = 1.00;  p.mode = 'measured';
elseif ischar(rho_scale) || isstring(rho_scale)
    rho_scale = 1.00;  p.mode = 'cad';
else
    rho_scale = double(rho_scale);  p.mode = 'measured-swept';
end
p.rho_scale   = rho_scale;
p.rho_plastic = 1290 * rho_scale;

% ---------------------------------------------------------------- constants
p.g      = 9.80665;             % MUST match Mechanism Configuration
p.a      = 0.156;
% --- BALANCING CORNER ------------------------------------------------
% CHANGED 18/08/2026: the mass data moved to the CUBE-CENTRE datum but this
% line did not. In the centre datum [0.078;0;0] is the CENTRE OF THE +X FACE,
% not an edge midpoint. Symptoms: ell 87.48 mm (must be ~125), Theta diagonal
% [0.0064 0.0160 0.0161] instead of near-isotropic, and a 17x asymmetry in the
% K attitude diagonal because gB lands on +x and the x-wheel becomes the yaw
% wheel. Corners in this datum are [+-0.078; +-0.078; +-0.078].
% (-1,-1,-1) chosen: shortest lever arm -> smallest Sg -> largest envelope
% (3.66 deg vs 3.00 deg on (+1,+1,+1)).  An x-EDGE midpoint would be
% [0; +-0.078; +-0.078].
%
% OLD, face centre in the centre datum:
% p.corner = [0.156/2; 0; 0];    %% balancing corner (+1,+1,+1)
p.corner = [-0.078; -0.078; -0.078];   % (-1,-1,-1)  ell 124.78, lambda 7.83                 %%%%%%%%%%%%%%%%%%%%%%%%% NOTE - set to balance on x edge, so this coord is that edge's midpoint

% measured constraints                                                       
M_FRAME_SIDE = 0.328445 + 0.157691;           % = 1257 g - 3*216 g           %%%%%%%%%%%%%%%%%%%%%%%%% NOTE - this is the sum of inner and outer frames
M_WHEEL      = 0.036;                                                        %%%%%%%%%%%%%%%%%%%%%%%%% NOTE - this is mass of wheel with no nuts
M_BALLAST    = 0.113;                                               
M_WHEEL_BARE = 0.0369;           % includes the shaft

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
% Total Mass (kg)
A_M = 0.459168;

% Combined Center of Mass (m)
A_COM = [-0.00319159;
         -0.00255866;
         -0.00213565];

% Combined Inertia Tensor about the Combined Center of Mass (kg*m^2)
A_I = [2.58334469e-3, 9.94827599e-5, 8.04940562e-5;
       9.94827599e-5, 2.55685822e-3, 9.77123955e-5;
       8.04940562e-5, 9.77123955e-5, 2.60155255e-3];

% FRAME - B: bought parts present in the 1257 g weighing
%            3 motors, 3 MA600, 3 magnets, 3 shafts, IMU, XIAO, perf, XT90
% Combined physical properties (Mass in kg, COM in m, Inertia in kg*m^2)
B_M = 0.263423;

B_COM = [-0.02635955;
         -0.01509657;
         -0.00642959];

B_I = [1.21884784e-3, 9.42436814e-5, 5.76063683e-5;
       9.42436814e-5, 1.34114798e-3, 3.42784531e-5;
       5.76063683e-5, 3.42784531e-5, 1.31139459e-3];

% FRAME - C: excluded from the weighing (battery, moteus, Teensy) + DC-DC
% Combined physical properties (Mass in kg, COM in m, Inertia in kg*m^2)
C_M0 = 0.311454;

C_COM = [0.02316886;
         0.02505086;
         0.02404533];

C_I0 = [3.66699589e-4, -2.48283307e-5, -3.15589632e-5;
      -2.48283307e-5,  3.74304899e-4, -3.20815147e-5;
      -3.15589632e-5, -3.20815147e-5,  3.76635836e-4];

% WHEEL - printed body, V = 76.424 cm^3
WPL_M   = 0.036903;
WPL_COM = [-0.070748; 0.00; -4.217e-08];                                        %%%%%%%%%%%%%%%%%%%%%%%%% NOTE - this is centre of mass of the x axis wheel. the x axis wheel is heavier than the others rn. (only focusing on edge balance)
WPL_I   = [8.8922956e-05, 0, 0;
           0, 4.4652009e-05, 0;
           0, 0, 4.4657103e-05];
% WHEEL - ballast, 16 nuts + 16 screws at r = 47.7 mm, V = 19.826 cm^3
WBA_M   = 0.112623;

WBA_COM = [-0.07005166;
           -0.00282121;
           -0.00052959];

WBA_I   = [2.87405206e-4, 0, 0;
           0,  1.44026365e-4, 0;
          0, 0,  1.44031641e-4];
% WHEEL - shaft
WSH_M   = 0.002543;
WSH_COM = [-0.07005166;
           -0.00282121;
           -0.00052959];
 
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
% NOTE 18/08/2026: the printed body is ON axis but the BALLAST is not
% (WBA_COM y = -2.82 mm), so the assembled wheel COM is 2.22 mm OFF AXIS.
for k = 1:3
    p.r_wheel{k} = p.com_wheel{k};
end
p.off  = p.r_wheel{1}(2);         % +0.007 m, axis offset from the cube axes
p.d_ax = p.r_wheel{1}(1);         % -0.0616 m, position along the wheel's axis

% ================================================= FRAME
% Fasteners not present in the CAD, sized to close the measured 609 g.
% CHANGED 18/08/2026: the m_extra mechanism reconciles a CAD-at-nominal
% figure against a weighing. A_M is already reconciled, so it double-counts
% and returned -163 g. With rho_scale=1 and m_extra=0 the model totals
% 1.4821 kg against the MEASURED 1.480 kg - 2.1 g, 0.14 %.
% OLD:  p.m_extra = M_FRAME_SIDE - rho_scale*A_M - B_M;
p.m_extra = 0;
if p.m_extra < 0
    warning('rho_scale %.2f implies negative fastener mass', rho_scale);
end
m_C = p.m_battery + p.m_moteus + p.m_teensy + p.m_dcdc;

[p.m_frame, p.com_frame, p.I_frame] = combine( ...
    {rho_scale*A_M, A_COM, rho_scale*A_I}, ...
    {B_M,           B_COM, B_I}, ...
    {m_C,           C_COM, C_I0*(m_C/C_M0)}, ...
    {p.m_extra,     A_COM, zeros(3)});      % extras at the printed centroid

% ================================================= assembled body
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
% EXPECTED VALUES at rho_scale = 1.00   (regenerated 18/08/2026)
%   m_wheel   0.149900 kg   Is 3.7049e-04   It 1.8587e-04
%   m_frame   1.032391 kg   extras 0.0 g
%   m_total   1.4821 kg          MEASURED 1.480 kg, err +2.1 g (0.14 %)
%   com       [-8.194, -5.603, -4.141] mm
%   ell       124.78 mm     Sg 1.8136 N m       corner (-1,-1,-1)
%   lambda    +7.8275, +7.8065 rad/s      (uniform cube 0.99*sqrt(g/a)=7.86)
%   theta_eq  1.333 deg on (-1,-1,-1);  1.143 deg on (+1,+1,+1)
%   mount     54.5345 deg about [-0.726765, +0.000000, +0.686887]
%   h_cap     0.014820 N m s at 40 rad/s   theta_static 5.37 deg
%   envelope  3.66 deg, MOMENTUM limited (torque bound 5.37 deg)
%   rank(ctrb) = 8 of 9   - yaw momentum is CONSERVED
%   gains     use cubli_cube_gains3N:  qa .20 / qr 2.0 / qw 10 / rt .24 / rho 12
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
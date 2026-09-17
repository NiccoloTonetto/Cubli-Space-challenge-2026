function p = cubli_nemesis_params()
%CUBLI_NEMESIS_PARAMS  Plant for the NEMESIS 156 mm cube, CORNER balance.
%
% GEOMETRY   Cubli_Frame_Final.step, Cubli_[XYZ]wheel_Final.step
%            *** ORIGIN AT A CUBE CORNER. Cube spans 0..156 mm, centre at
%            (78,78,78). This is NOT the centre-origin convention of the
%            other cube - do not copy expressions across without checking. ***
%            Extracted with the OCC kernel, assembly transforms applied.
%            Bounding box check: 164 x 164 x 156 mm.
%
% MASSES     MEASURED 21/08/2026
%              assembled cube          1.615 kg
%              wheel with ballast      0.150 kg  (x3, assumed identical)
%              wheel without ballast   0.039 kg  -> ballast 0.111 kg
%
% CORNER     (0,0,0) - where the three wheels meet.
%
% NOTES ON THE MEASUREMENTS
%  1. Wheel plastic back-solves to 1315 kg/m^3 against a PETG-CF nominal of
%     1290, i.e. essentially SOLID. Either a denser filament or the 39 g
%     includes a shaft not in the CAD. The disc geometry is well modelled,
%     so this density with the CAD volume gives both the right mass and the
%     right distribution.
%  2. Ballast back-solves to 22753 kg/m^3 - 2.9x steel and 1.2x TUNGSTEN,
%     so physically impossible. The CAD ballast VOLUME is under-modelled;
%     the MASS is measured and correct. Resolution: keep the CAD geometric
%     distribution, scale to the measured mass. VERIFIED: rho_eff*Iv gives
%     3.3717e-04 against a thin ring m*r^2 = 3.3577e-04, agreement 0.42 %.
%     The ring really is at r = 55 mm.
%  3. Frame uses ONE effective density because the STEP export lost all part
%     names - all 31 solids share one label. Mass is right; the COM rests on
%     a uniform-density assumption. See STILL OPEN.
%  4. Motors are LUMPED INTO THE FRAME, so their rotors do not rotate in the
%     model. Worth roughly 2-3 % on each wheel inertia. Confirmed acceptable.
%
% CONVENTION
%   x = [phi(3); omega(3); wheel_rate(3)]                       9 states
%   Tb*omegadot = Sg*P*phi - u,   Is*(a_k.omegadot + phiddot_k) = u_k
%   Tb = Theta - Aw*Is*Aw^T,  P = I - gB*gB^T

p.g = 9.80665;
p.a = 0.156;                       % *** 156 mm, not 150 ***
p.corner = [0; 0; 0];              % balancing corner, CORNER-ORIGIN

% ------------------------------------------------------ measured masses
M_TOT     = 1.615;
M_WHEEL   = 0.150;
M_WH_BARE = 0.039;
M_BALLAST = M_WHEEL - M_WH_BARE;              % 0.111 kg
M_FRAME   = M_TOT - 3*M_WHEEL;                % 1.165 kg

% =======================================================================
% CAD GEOMETRY.  V in cm^3, C in m (corner origin), Iv in m^5.
% Iv is a VOLUME inertia about that group own COM: multiply by a density
% in kg/m^3 to get kg m^2.
% =======================================================================

% --- wheel plastic disc ------------------------------------------------
WPL_V  = 29.655358;
WPL_C  = [0.0073264; 0.0780002; 0.0780004];
WPL_Iv = [ 7.43908817e-08,  8.12644386e-14, -4.65103713e-14; ...
           8.12644386e-14,  3.73437291e-08, -2.86377418e-14; ...
          -4.65103713e-14, -2.86377418e-14,  3.73441188e-08];

% --- wheel ballast, 30 slugs at r = 55.0 mm ----------------------------
WBA_V  = 4.878512;
WBA_C  = [0.0055000; 0.0780289; 0.0780000];
WBA_Iv = [ 1.48188788e-08, 0, 0; ...
           0, 7.41960296e-09, 0; ...
           0, 0, 7.41960296e-09];

% --- frame, all 31 solids (structure + motors + drivers + battery) -----
FR_V  = 992.570037;
FR_C  = [0.0785128; 0.0785734; 0.0751641];
FR_Iv = [ 4.69956023e-06, -3.18183448e-08, -2.24561177e-08; ...
         -3.18183448e-08,  4.69488227e-06, -2.48123318e-08; ...
         -2.24561177e-08, -2.48123318e-08,  4.69387466e-06];

% ---------------------------------------- densities from the weighings
p.rho_plastic = M_WH_BARE / (WPL_V*1e-6);     % 1315  kg/m^3
p.rho_ballast = M_BALLAST / (WBA_V*1e-6);     % 22753 kg/m^3, see note 2
p.rho_frame   = M_FRAME   / (FR_V *1e-6);     % 1174  kg/m^3

% ================================================== WHEEL, X axis
[m_wx, c_wx, I_wx] = combine( ...
    {p.rho_plastic*WPL_V*1e-6, WPL_C, p.rho_plastic*WPL_Iv}, ...
    {p.rho_ballast*WBA_V*1e-6, WBA_C, p.rho_ballast*WBA_Iv});

p.m_wheel    = m_wx;
p.axis_wheel = eye(3);
for k = 1:3                        % cyclic permutation x -> y -> z -> x
    R = circshift(eye(3), k-1, 1);
    p.com_wheel{k} = circshift(c_wx, k-1);
    p.r_wheel{k}   = p.com_wheel{k};      % wheel is balanced on its axis
    p.I_wheel{k}   = R*I_wx*R.';
end
p.Is = p.I_wheel{1}(1,1);          % spin axis
p.It = p.I_wheel{1}(2,2);          % transverse
p.rot_wx = +pi/2;                  % Rigid transform: about +Y, joint Z -> X
p.rot_wy = -pi/2;                  % about +X, joint Z -> Y  (Z needs none)

% ================================================== FRAME
p.m_frame   = p.rho_frame*FR_V*1e-6;
p.com_frame = FR_C;
p.I_frame   = p.rho_frame*FR_Iv;

% ================================================== ASSEMBLY
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

p.Theta = zeros(3);                % about the CONTACT CORNER
for i = 1:numel(bodies)
    d = bodies{i}{2} - p.corner;
    p.Theta = p.Theta + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
end

% -------------------------------------------------- plant scalars
r        = p.com - p.corner;
p.ell    = norm(r);
p.gB     = -r/p.ell;
p.S      = p.m_total * p.ell;
p.Sg     = p.S * p.g;
p.Aw     = p.axis_wheel;
p.Is_mat = p.Is * eye(3);
p.Tb     = p.Theta - p.Aw*p.Is_mat*p.Aw.';
p.P      = eye(3) - p.gB*p.gB.';

% -------------------------------------------------- mount rotation
tgt = [0; -1; 0];
ax  = cross(p.gB, tgt);
p.mount_axis  = ax / norm(ax);
p.mount_angle = acos(max(-1, min(1, dot(p.gB, tgt))));   % RAD

% -------------------------------------------------- state space
Z = zeros(3);  I3 = eye(3);
G = p.Tb \ (p.Sg * p.P);
p.A = [ Z  I3  Z ;  G  Z  Z ; -G  Z  Z ];
p.B = [ Z ; -inv(p.Tb) ; inv(p.Is_mat) + inv(p.Tb) ];
p.C = eye(9);  p.D = zeros(9,3);
ev       = eig(p.A);
p.lambda = sort(real(ev(real(ev) > 1e-6)), 'descend');

% -------------------------------------------------- IMU
% *** PROBLEM POSITION. Reported at the centre of the TOP face, giving a
% lever arm of 191 mm from the contact corner, against 130 mm on the other
% cube. At the envelope edge (45 rad/s^2) the tangential error reaches 88 %
% of g and the self-cancellation leaves an 18 % residual, because atan()
% is no longer linear at that amplitude.
% MITIGATION, in order: model-based alpha correction (see the gains file),
% explicit centripetal correction, gate at 35-40 %% not 10 %%.
% CONFIRM THE ACTUAL POSITION - this is an estimator input.
p.r_imu = [0.078; 0.078; 0.156];
p.accel_gate = 0.35;               % NOT 0.10 - see above

% -------------------------------------------------- actuation
p.Kt        = 0.02513;
p.tau_cont  = 0.12;                % N m  ESTIMATE, thermal, no airflow
p.tau_peak  = 0.40;
p.omega_max = 8436*2*pi/60;
p.omega_cap = 40;                  % rad/s  FIRMWARE POLICY, not hardware
p.h_cap     = p.Is * p.omega_cap;
p.tau_eff   = sqrt(2) * p.tau_cont;
p.theta_static = asin(min(1, p.tau_eff/p.Sg));

% -------------------------------------------------- friction
p.tau_cw = 8e-3;                   % N m  PLACEHOLDER, spin-down test
p.b_w    = 0;                      %      PLACEHOLDER
p.tau_cp = 0;                      %      PLACEHOLDER
p.eps_ff = 0.05;

% -------------------------------------------------- sim settings
p.f_outer  = 400;
p.enc_bits = 16;
p.enc_lsb  = 2*pi/2^p.enc_bits;

% ------------------------------------- equilibria, all eight corners
% *** CORNER ORIGIN: corners are combinations of 0 and a, NOT +/- a/2. ***
n = 0;  ctr = [0.5;0.5;0.5]*p.a;
for sx = [0 1], for sy = [0 1], for sz = [0 1]
    n = n + 1;
    cc = [sx; sy; sz]*p.a;
    vv = p.com - cc;
    p.corners(n).sign = 2*[sx sy sz]-1;
    p.corners(n).pos  = cc;
    p.corners(n).ell  = norm(vv);
    p.corners(n).gB   = -vv/norm(vv);
    u  = (cc-ctr)/norm(cc-ctr);
    da = dot(p.com-ctr, u);
    p.corners(n).theta_eq = atan(norm((p.com-ctr) - da*u)/(sqrt(3)/2*p.a - da));
end, end, end

% =======================================================================
% EXPECTED VALUES
%   m_total  1.6150 kg   (identity m_frame + 3*m_wheel is EXACT)
%   m_wheel  0.1500 kg   Is 4.3500e-04   It 2.1802e-04   Is/It 1.9952
%   com      [71.682, 71.726, 69.267] mm, offset 12.47 mm from the centre
%   ell      122.80 mm   Sg 1.9449 N m
%   lambda   +7.8403, +7.8382 rad/s   (0.99*sqrt(g/a) = 7.8493, 0.1 %% off)
%   mount    54.2628 deg about [-0.69489, 0, +0.71912]
%   rank(ctrb) = 8 of 9   theta_static 5.01 deg
%
% STILL OPEN
%  - FRAME COM rests on a UNIFORM-DENSITY assumption. Motors, drivers and
%    battery are smeared with the printed structure. Mass is right, COM is
%    not guaranteed, and the COM sets the equilibrium attitude on every
%    corner. FIX: plumb-line from three corners, 15 minutes.
%  - IMU position ASSUMED. Confirm, and move it if at all possible - the
%    bottom face centre would give 110 mm instead of 191 mm.
%  - tau_cont carried over from the other cube. Thermal test.
%  - tau_cw, b_w, tau_cp are placeholders. Spin-down and breakaway.
%  - CAD ballast volume under-modelled 2.9x. Mass measured, so the inertia
%    is right, but trust no other CAD-volume-derived fastener number.
% =======================================================================
end

% -----------------------------------------------------------------------
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

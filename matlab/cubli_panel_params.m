function p = cubli_panel_params(mode)
%CUBLI_PANEL_PARAMS  Plant parameters for the 1D panel.
%
%   p = cubli_panel_params()            measured masses  <- USE THIS
%   p = cubli_panel_params('measured')  same
%   p = cubli_panel_params('cad')       old single-density CAD estimate
%   p = cubli_panel_params(rho)         CAD geometry at a given plastic density
%
% MEASURED MODE (03/08/2026, weighed by the mechanical team)
%   Frame w/o encoder           51 g   (frame piece 1 + moteus 14.6 g)
%   Frame holder                41 g   (frame piece 2, NON-rotating)
%   Wheel with nuts             66 g
%   Frame + encoder + wheel + motor    223 g
%   TOTAL without base, WITH frame screws   263 +/- 1.5 g   <- used here
%   The 37 g difference is frame-joining fasteners, not in the CAD.
%   Assumed distributed at the frame centroid. NOT measured - see below.
%
%   Back-solved effective print densities (15 % infill):
%     frame  447 kg/m^3  = 34 % of PETG-CF nominal
%     wheel  311 kg/m^3  = 24 % of nominal
%   They differ because the wheel is a flat disc where sparse infill dominates
%   the cross-section, while the ribbed frame carries proportionally more
%   perimeter. Do NOT collapse them to one number.
%
%   SCREW PLACEMENT IS ASSUMED, not measured. Bracketing cases give
%     perimeter  l = 101.7 mm, Theta = 3.398e-3   <- assumed
%     pivot end  l =  89.9 mm, Theta = 3.062e-3
%     centre     l = 103.1 mm, Theta = 3.473e-3
%   lambda varies only 0.6 % across all three, so the GAINS are safe.
%   The actuation bounds move ~4 deg. To close it: plumb-line the COM for l,
%   then Theta = S*g*(T/2pi)^2 from the clamped-wheel swing period.
%
% CONVENTION (ETH 1D):
%   x = [theta; theta_dot; phi_dot], theta from upright, phi_dot relative
%   Tb*theta_ddot = S*g*sin(theta) - u,   hdot = u,   Tb = Theta - Iw
% Pivot: corner of the 150 mm square, axis normal to the panel plane.
% Wheel axis and all electronics: panel centre, (75, 75) mm from the pivot.

if nargin < 1 || isempty(mode), mode = 'measured'; end

% ---------------------------------------------------------------- constants
p.g = 9.80665;                 % MUST match Mechanism Configuration in Simscape
rho_steel = 7850;

% CAD solid volumes, m^3 (exact - only the densities were ever uncertain)
V.frame2      = 103.532e-6;
V.frame2_0    =  69.740e-6;
V.bolt_din933 =   2.649e-6;    % not fitted, see header
V.nut_static  =   1.376e-6;    % not fitted
V.wheel_body  =  62.330e-6;
V.hub         =   1.795e-6;
V.nut_ballast =   5.161e-6;    % 15 M6 nuts at r = 57 mm
V.m3x20       =   0.707e-6;
V.motor       =  10.143e-6;
p.V = V;

% in-plane geometry, metres (pivot at the corner, origin of the static STEP)
p.a          = 0.150;
p.r_pivot    = [0.000; 0.000];
p.r_centre   = [0.075; 0.075];
p.com_frame  = [0.06788; 0.06794];   % CAD centroid of FRAME2 + FRAME2_0

% Izz about own COM at nominal 1300 kg/m^3, scales linearly with density
IZZ_FRAME_1300  = 1.3676e-3;
IZZ_WHEEL_1300  = 2.2650e-4;
IZZ_WHEEL_STEEL = 1.3164e-4;         % 15 nuts + 4 screws, fixed
IZZ_MOTOR       = 1.5000e-5;

% ------------------------------------------------------------------- modes
if (ischar(mode) || isstring(mode)) && strcmpi(mode,'measured')
    p.mode      = 'measured';
    p.rho_frame = 447;
    p.rho_wheel = 311;
    p.m_motor   = 66.0e-3;           % datasheet
    p.m_driver  = 14.6e-3;           % moteus n1
    p.m_encoder =  2.0e-3;           % MA600 breakout
    p.m_screws  = 37.0e-3;           % frame fasteners, at the frame centroid
    p.m_measured_total = 263.0e-3;   % without base
    fit_steel   = false;             % DIN933 + 4 nuts superseded by m_screws
elseif (ischar(mode) || isstring(mode)) && strcmpi(mode,'cad')
    p.mode      = 'cad';
    p.rho_frame = 1300;  p.rho_wheel = 1300;
    p.m_motor   = V.motor*6793;
    p.m_driver  = 0;     p.m_encoder = 0;  p.m_screws = 0;
    fit_steel   = true;
else
    p.mode      = 'cad-density';
    p.rho_frame = double(mode);  p.rho_wheel = double(mode);
    p.m_motor   = V.motor*6793;
    p.m_driver  = 0;     p.m_encoder = 0;  p.m_screws = 0;
    fit_steel   = true;
end

% ------------------------------------------------------------ body masses
m_frame_plastic = (V.frame2 + V.frame2_0)*p.rho_frame;
m_frame_steel   = fit_steel*(V.bolt_din933 + V.nut_static)*rho_steel;
m_wheel_plastic = (V.wheel_body + V.hub)*p.rho_wheel;
m_wheel_steel   = (V.nut_ballast + V.m3x20)*rho_steel;
m_electronics   = p.m_driver + p.m_encoder;

p.m_wheel = m_wheel_plastic + m_wheel_steel;
p.m_panel = m_frame_plastic + m_frame_steel + p.m_screws + p.m_motor + m_electronics;
p.m_total = p.m_panel + p.m_wheel;

% --------------------------------------------------------- inertia, kg m^2
p.Iw = IZZ_WHEEL_1300*(p.rho_wheel/1300) + IZZ_WHEEL_STEEL;
Izz_frame_com = IZZ_FRAME_1300*(p.rho_frame/1300);

% ------------------------------------------------------------------ COM
% Frame plastic (and its bolts) sit at the CAD frame centroid.
% Motor, driver, encoder and the wheel all sit on the panel centre.
m_at_centre = p.m_motor + m_electronics;
m_at_frame  = m_frame_plastic + m_frame_steel + p.m_screws;   % screws assumed
                                                              % at frame centroid

p.com_panel = (m_at_frame*p.com_frame + m_at_centre*p.r_centre)/p.m_panel;
p.com_wheel = p.r_centre;
p.com_total = (p.m_panel*p.com_panel + p.m_wheel*p.com_wheel)/p.m_total;

% ------------------------------------------------------- plant scalars
p.ell = norm(p.com_total - p.r_pivot);
p.S   = p.m_total * p.ell;
p.Sg  = p.S * p.g;

p.Izz_panel_com = Izz_frame_com + IZZ_MOTOR ...
    + m_at_frame *norm(p.com_frame - p.com_panel)^2 ...
    + m_at_centre*norm(p.r_centre  - p.com_panel)^2;

p.Theta = p.Izz_panel_com + p.m_panel*norm(p.com_panel - p.r_pivot)^2 ...
        + p.Iw            + p.m_wheel*norm(p.com_wheel - p.r_pivot)^2;
p.Tb    = p.Theta - p.Iw;

p.lambda    = sqrt(p.Sg/p.Tb);
p.T_locked  = 2*pi*sqrt(p.Theta/p.Sg);   % wheel clamped - measures Theta
p.T_free    = 2*pi*sqrt(p.Tb   /p.Sg);   % wheel free    - measures Tb
p.phi_mount = pi/2 - atan2(p.com_total(2), p.com_total(1));

% ------------------------------------------------------- state space
p.A = [0 1 0; p.Sg/p.Tb 0 0; -p.Sg/p.Tb 0 0];
p.B = [0; -1/p.Tb; 1/p.Iw + 1/p.Tb];
p.C = eye(3);
p.D = zeros(3,1);

% ------------------------------------------------------- actuation
p.Kt        = 0.02513;
p.tau_cont  = 0.12;                     % ESTIMATE - thermal, no airflow.
p.tau_peak  = 0.40;                     % Verify with a thermal test.
p.omega_max = 8436*2*pi/60;
p.h_max     = p.Iw * p.omega_max;
p.omega_cap = 40;                       % FIRMWARE POLICY, not a hardware limit
p.h_cap     = p.Iw * p.omega_cap;

p.torque_ratio = p.tau_cont/p.Sg;
p.theta_static = asin(min(1, p.torque_ratio));
p.X_cap        = p.h_cap^2/(2*p.Theta*p.Sg);
p.theta_mom    = acos(max(-1, 1 - min(2, p.X_cap)));

% ------------------------------------------------------- sim settings
p.f_outer  = 400;
p.enc_bits = 16;
p.enc_lsb  = 2*pi/2^p.enc_bits;

% NOTE: back-EMF at omega_cap is only 4.5 % of the 6S supply, so the taper in
% the ACTUATOR block is firmware policy, not a physical torque roll-off.
end

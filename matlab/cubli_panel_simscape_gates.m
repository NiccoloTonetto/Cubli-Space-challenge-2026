%CUBLI_PANEL_SIMSCAPE_GATES  Six validation gates for the 1D panel model.
%
% PREREQUISITES in the Simulink model:
%   Inport      'TorqueIn'   -> Simulink-PS -> WHEEL joint  t port
%   PS-Simulink x3 (PIVOT.q, PIVOT.w, WHEEL.w) -> Mux -> Outport 'StateOut'
%   To Workspace block on the Mux output: variable 'xlog', format Timeseries
%   Mux order MUST be [theta; theta_dot; phi_dot]
%
% Some gates need a manual dialog change first - each section says which.
% Run sections individually with Ctrl+Enter.

mdl = 'Cubli_sim';          % <- your model name
rho = 845;                    % <- measured effective plastic density

%% 1. Parameters (in cubli_panel_simscape_gates.m)
p = cubli_panel_params;        % <-- was cubli_panel_params(845)
p.Izz_panel_com = p.Theta - p.m_panel*norm(p.com_panel)^2 ...
                          - p.Iw     - p.m_wheel*norm(p.com_wheel)^2;
p.phi_mount     = pi/2 - atan2(p.com_total(2), p.com_total(1));

T_locked = 2*pi*sqrt(p.Theta/p.Sg);
T_free   = 2*pi*sqrt(p.Tb   /p.Sg);

fprintf('\n--- plant, rho = %.0f kg/m^3 ---\n', rho);
fprintf('  m = %.4f kg   l = %.2f mm   S = %.5f kg m\n', p.m_total, p.ell*1e3, p.S);
fprintf('  Theta = %.4e   Iw = %.4e   Tb = %.4e\n', p.Theta, p.Iw, p.Tb);
fprintf('  lambda = %.4f rad/s   (tau = %.0f ms)\n', p.lambda, 1000/p.lambda);
fprintf('  T_locked = %.4f s     T_free = %.4f s\n', T_locked, T_free);
fprintf('  phi_mount = %.5f rad (%.4f deg)\n\n', p.phi_mount, rad2deg(p.phi_mount));

if ~bdIsLoaded(mdl), load_system(mdl); end

%% GATE 1 - hang test
% MANUAL: PIVOT State Targets -> Position 0.3 rad, priority Low.
%         WHEEL Actuation -> Motion -> Provided by Input, fed Constant 0.
% Watch Mechanics Explorer. The panel must fall AWAY from upright and
% settle with the COM below the pivot. If it stays upright, gravity or
% phi_mount has the wrong sign - stop and fix before going further.

out  = sim(mdl, 'StopTime', '10');
xlog = out.xlog;
th   = squeeze(xlog.Data(:,1));

%% GATE 2 - free-swing period
% MANUAL: PIVOT Position target = pi - 0.15 rad (small swing about hanging).
%         Zero damping on BOTH joints (Internal Mechanics -> 0).
% Run twice: once with WHEEL locked (motion input, Constant 0) -> T_locked
%            once with WHEEL torque-actuated and TorqueIn = 0 -> T_free

out  = sim(mdl, 'StopTime', '6');
xlog = out.xlog;
t    = xlog.Time;
th   = squeeze(xlog.Data(:,1));

x = th - mean(th);
s = sign(x);
k = find(s(1:end-1) < 0 & s(2:end) >= 0);          % upward zero crossings
tc = t(k) - x(k).*(t(k+1)-t(k))./(x(k+1)-x(k));    % sub-step interpolation
T  = mean(diff(tc));

fprintf('GATE 2  measured T = %.4f s   (%d crossings)\n', T, numel(k));
fprintf('        swing = %.1f deg\n', rad2deg(max(th)-min(th)));
fprintf('        vs locked %.4f s (%+.2f%%) | free %.4f s (%+.2f%%)\n', ...
    T_locked, 100*(T-T_locked)/T_locked, T_free, 100*(T-T_free)/T_free);

% AFTER SIMULATIONS (Number corrected)
Tl = 0.7771/1.05548;      % locked, amplitude-corrected
Tf = 0.7498/1.04937;      % free,   amplitude-corrected
Theta_m = p.Sg*(Tl/(2*pi))^2;
Tb_m    = p.Sg*(Tf/(2*pi))^2;
fprintf('Iw from periods = %.4e  vs params %.4e  (%+.2f%%)\n', ...
        Theta_m-Tb_m, p.Iw, 100*((Theta_m-Tb_m)-p.Iw)/p.Iw);

%% GATE 3 - linearisation  ** the definitive gate **
% MANUAL: PIVOT Position target 0, priority High. WHEEL back to torque
%         actuation. Upright with zero torque is an exact equilibrium,
%         so no trimming is needed - linearise at t = 0.

io(1) = linio([mdl '/TorqueIn'], 1, 'openinput');
io(2) = linio([mdl '/StateOut'], 1, 'openoutput');
sys   = linearize(mdl, 0, io);
[Aq, Bq] = ssdata(sys);

if ~isequal(size(Aq), [3 3])
    error(['Aq is %dx%d, expected 3x3. Extra states usually mean input ' ...
           'filtering is ON in the Simulink-PS Converter.'], size(Aq,1), size(Aq,2));
end

eA = norm(Aq - p.A)/norm(p.A);
eB = norm(Bq - p.B)/norm(p.B);
fprintf('GATE 3  A err = %.3e   B err = %.3e\n', eA, eB);
if eA > 1e-6 || eB > 1e-6
    disp('  Aq ='); disp(Aq); disp('  p.A ='); disp(p.A);
    disp('  Bq ='); disp(Bq.'); disp('  p.B ='); disp(p.B.');
    warning(['MISMATCH. Scaled A(2,1) -> S or Tb. Wrong B(3) -> Iw (check ' ...
             'Inertia = Custom). Flipped B(2) -> reaction sign. Permuted ' ...
             'entries -> Mux order.']);
end

%% GATE 4 - pole match
lam = eig(Aq);
fprintf('GATE 4  poles: %s\n', sprintf('%+.4f  ', sort(real(lam))));
fprintf('        expect %+.4f, %+.4f, 0\n', -p.lambda, p.lambda);

%% GATE 5 - energy conservation
% MANUAL: zero damping, TorqueIn = 0, PIVOT position target 20 deg.
% E = 1/2 Tb thd^2 + 1/2 Iw (thd + phid)^2 + S g cos(theta)

out  = sim(mdl, 'StopTime', '10');
xlog = out.xlog;
th   = squeeze(xlog.Data(:,1));  thd = squeeze(xlog.Data(:,2));
phd = squeeze(xlog.Data(:,3));
E   = 0.5*p.Tb*thd.^2 + 0.5*p.Iw*(thd+phd).^2 + p.Sg*cos(th);
drift = 100*(max(E)-min(E))/mean(abs(E));
fprintf('GATE 5  energy drift = %.4f %%  (want < 0.1)\n', drift);

%% GATE 6 - momentum coupling
% MANUAL: Mechanism Configuration -> Gravity = [0 0 0].
%         PIVOT position target 0, TorqueIn = a short pulse on the wheel.
% Angular momentum about the pivot must stay zero:  L = Theta*thd + Iw*phid

out  = sim(mdl, 'StopTime', '10');
xlog = out.xlog;
thd = squeeze(xlog.Data(:,2));  phd = squeeze(xlog.Data(:,3));
L   = p.Theta*thd + p.Iw*phd;
fprintf('GATE 6  max |L| = %.3e N m s  (want ~0)\n', max(abs(L)));
fprintf('        ratio thd/phid = %+.5f   expect %+.5f\n', ...
    thd(end)/phd(end), -p.Iw/p.Theta);

% REMEMBER to restore gravity to [0 -9.81 0] before any other gate.

%% GATE 7
% --- LQR design (Bryson, designed against the capped wheel speed) ---
theta_max = 0.20;              % rad
rate_max  = 2.0;               % rad/s
omega_des = 0.5*p.omega_cap;   % 20 rad/s, NOT 0.5*omega_max
rho_lqr   = 12;

Q = diag([1/theta_max^2, 1/rate_max^2, 1/omega_des^2]);
R = rho_lqr / p.tau_cont^2;

K_lqr = lqr(p.A, p.B, Q, R);
fprintf('K = [%9.4f %9.4f %11.6f]\n', K_lqr);
fprintf('closed-loop poles: %s\n', sprintf('%.2f  ', eig(p.A - p.B*K_lqr)));

%%
out = sim(mdl,'StopTime','5');  xlog = out.xlog;
t = xlog.Time; x = squeeze(xlog.Data);

figure
subplot(3,1,1); plot(t, rad2deg(x(:,1))); ylabel('\theta [deg]'); grid on
subplot(3,1,2); plot(t, x(:,3)); ylabel('\phi dot [rad/s]'); grid on
subplot(3,1,3); plot(t, -x*K_lqr'); ylabel('u [N m]'); grid on; xlabel('t [s]')
yline(0.12,'r--'); yline(-0.12,'r--')

%% helper
function T = local_period(t, x)
    x = x - mean(x);
    s = sign(x);  k = find(s(1:end-1) < 0 & s(2:end) >= 0);
    if numel(k) < 2, T = NaN; return; end
    tc = t(k) - x(k).*(t(k+1)-t(k))./(x(k+1)-x(k));   % linear interp
    T  = mean(diff(tc));
end

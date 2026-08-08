%CUBLI_PANEL_SATURATION  Block A - recoverable-tilt envelope with a real actuator.
%
% Adds torque saturation and the wheel-speed cap to the validated linear
% closed loop, then finds the largest initial tilt the panel can actually
% recover from. Compare against the unsaturated bounds:
%   torque-limited (linear)  5.8 deg
%   momentum-limited (cap)  ~16   deg
% The saturated answer sits between them - temporary saturation is survivable,
% sustained saturation is not.
%
% MODEL PREREQUISITE
%   Between the LQR Gain and the Simulink-PS Converter, insert a MATLAB
%   Function block named 'ACTUATOR' with this body:
%
%       function u = actuator(u_cmd, phidot, tau_max, w_cap)
%       %#codegen
%       u = max(min(u_cmd, tau_max), -tau_max);
%       if (phidot >=  w_cap && u > 0) || (phidot <= -w_cap && u < 0)
%           u = 0;
%       end
%       end
%
%   Wire: u_cmd <- Gain output, phidot <- Mux element 3 (Selector, index 3),
%         tau_max <- Constant p.tau_cont, w_cap <- Constant p.omega_cap.
%   Add a bypass Manual Switch around it so the linear baseline stays
%   available for re-running Gate 3.
%
% Run cubli_panel_simscape_gates.m parameter section first (needs p, K_lqr).

mdl = 'Cubli_sim';

%% Parameters (run this first, always)
p = cubli_panel_params;        % defaults to measured
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

%% ---------------------------------------------------------------- setup
assert(exist('p','var')==1 && exist('K_lqr','var')==1, ...
    'Run the parameter/LQR section of cubli_panel_simscape_gates.m first.');
if ~bdIsLoaded(mdl), load_system(mdl); end

TSTOP      = 8;                    % s, long enough for the wheel to unwind
SETTLE_TOL = deg2rad(2);           % |theta| at t_end to count as recovered
PIVOT      = [mdl '/PIVOT'];

fprintf('\n=== BLOCK A: saturated recovery envelope ===\n');
fprintf('tau_cont  = %.3f N m\n', p.tau_cont);
fprintf('omega_cap = %.1f rad/s  (h_cap = %.4f N m s)\n', p.omega_cap, p.Iw*p.omega_cap);
fprintf('K         = [%.4f %.4f %.6f]\n\n', K_lqr);

%% ------------------------------------------------------- single-run helper
% (defined at the bottom of the file as local function RUNTILT)

%% ---------------------------------------------- 1. coarse sweep, for the shape
th_list = deg2rad([8 10 12 14 16 18]);
n = numel(th_list);
res = cell(1,n);

fprintf('%6s  %6s  %8s  %8s  %8s  %8s\n', ...
        'th0deg','ok','max|u|','%tau','max|w|','t_sat_s');
for i = 1:n
    r = runtilt(mdl, PIVOT, th_list(i), TSTOP, K_lqr, p, SETTLE_TOL);
    res{i} = r;
    fprintf('%6.1f  %6s  %8.4f  %8.0f  %8.1f  %8.3f\n', ...
        rad2deg(r.th0), string(r.ok), r.umax, 100*r.umax/p.tau_cont, r.wmax, r.tsat);
end

res = [res{:}];

%% --------------------------------------------------- 2. bisection on the edge
ok_all = [res.ok];
if all(ok_all)
    fprintf('\nAll coarse angles recovered - widen th_list before bisecting.\n');
    th_edge = NaN;
elseif ~any(ok_all)
    fprintf('\nNothing recovered - check the actuator wiring or gain sign.\n');
    th_edge = NaN;
else
    lo = max(th_list(ok_all));      % largest known-good
    hi = min(th_list(~ok_all));     % smallest known-bad
    for k = 1:10
        mid = 0.5*(lo+hi);
        r = runtilt(mdl, PIVOT, mid, TSTOP, K_lqr, p, SETTLE_TOL);
        if r.ok, lo = mid; else, hi = mid; end
    end
    th_edge = lo;
    fprintf('\n--- MAX RECOVERABLE TILT (saturated) = %.2f deg ---\n', rad2deg(th_edge));
    fprintf('    naive torque bound        %.1f deg\n', rad2deg(p.tau_cont/K_lqr(1)));
    fprintf('    momentum bound at cap     %.1f deg\n', rad2deg(p.theta_mom));
    fprintf('    static bound (mgl)        %.1f deg\n', rad2deg(p.theta_static));
end

%% ------------------------------------------------------- 3. which limit binds
% At the edge, check whether the run spent more time on the torque limit or
% at the wheel cap. That tells you which constraint to relax for more envelope.
if ~isnan(th_edge)
    r = runtilt(mdl, PIVOT, th_edge*0.98, TSTOP, K_lqr, p, SETTLE_TOL);
    tau_lim = p.tau_peak;      % or p.tau_cont for the conservative run
    f_tau = mean(abs(r.u) >= 0.999*tau_lim);
    f_cap = mean(abs(r.x(:,3)) >= 0.999*p.omega_cap);
    fprintf('\nAt 98%% of the edge angle:\n');
    fprintf('   %.1f%% of the run on the torque limit\n', 100*f_tau);
    fprintf('   %.1f%% of the run at the wheel cap\n',    100*f_cap);
    if f_tau > f_cap
        fprintf('   -> TORQUE binds first. More envelope needs lower rho or a bigger motor.\n');
    else
        fprintf('   -> MOMENTUM binds first. More envelope needs a higher cap or more Iw.\n');
    end
end

%% ------------------------------------------------------------- 4. plots
figure('Name','Block A - saturated recovery');
pick = [find(ok_all,1,'last'), find(~ok_all,1,'first')];   % last good, first bad
pick = pick(~isnan(pick) & pick>0);
lbl = {};
for j = 1:numel(pick)
    r = res(pick(j));
    subplot(3,1,1); hold on; grid on
    plot(r.t, rad2deg(r.x(:,1))); ylabel('\theta [deg]')
    subplot(3,1,2); hold on; grid on
    plot(r.t, r.x(:,3)); ylabel('$\dot\phi$ [rad/s]','Interpreter','latex')
    subplot(3,1,3); hold on; grid on
    plot(r.t, r.u); ylabel('u [N m]'); xlabel('t [s]')
    lbl{end+1} = sprintf('%.1f deg (%s)', rad2deg(r.th0), string(r.ok)); %#ok<SAGROW>
end
subplot(3,1,2); yline( p.omega_cap,'r--'); yline(-p.omega_cap,'r--');
subplot(3,1,3); yline( p.tau_cont, 'r--'); yline(-p.tau_cont, 'r--');
subplot(3,1,1); legend(lbl,'Location','best');

figure('Name','Block A - envelope');
plot(rad2deg([res.th0]), 100*[res.umax]/p.tau_cont, 'o-'); hold on; grid on
plot(rad2deg([res.th0]), 100*[res.wmax]/p.omega_cap, 's-')
if ~isnan(th_edge), xline(rad2deg(th_edge),'k--','edge'); end
yline(100,'r--'); xlabel('\theta_0 [deg]'); ylabel('% of limit')
legend('torque','wheel speed','Location','best')

%% ------------------------------------------------------------ local function
function r = runtilt(mdl, PIVOT, th0, tstop, K, p, tol)
    set_param(PIVOT, 'PositionTargetValue', num2str(th0));
    out = sim(mdl, 'StopTime', num2str(tstop), ...
                   'MaxStep', '1e-3', ...
                   'ZeroCrossControl', 'DisableAll');
    xl   = out.xlog;
    r.t  = xl.Time;
    r.x  = squeeze(xl.Data);
    ucmd = -r.x*K(:);
    % r.u  = max(min(ucmd, p.tau_cont), -p.tau_cont);        % as applied
    r.u = max(min(ucmd, p.tau_peak), -p.tau_peak);          % wo thermal
    w    = abs(r.x(:,3));
    s = max(min((p.omega_cap - abs(r.x(:,3)))/(0.1*p.omega_cap), 1), 0);
    same = sign(r.u) == sign(r.x(:,3));
    r.u(same) = r.u(same) .* s(same);

    r.th0  = th0;
    r.umax = max(abs(ucmd));                                % DEMANDED torque
    r.wmax = max(w);
    sat    = abs(ucmd) >= p.tau_cont;
    r.tsat = sum(sat)*mean(diff(r.t));                      % time in saturation
    r.ok   = abs(r.x(end,1)) < tol && all(isfinite(r.x(:)));
    idx    = find(abs(r.x(:,1)) > tol, 1, 'last');
    if isempty(idx), r.tset = 0; else, r.tset = r.t(min(idx+1,numel(r.t))); end
end

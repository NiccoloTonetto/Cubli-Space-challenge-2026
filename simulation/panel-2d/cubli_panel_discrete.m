%CUBLI_PANEL_DISCRETE  Block B - sample-and-hold + computational delay.
%
% Block A answered "how far can it tip". This answers "how slowly can the
% Teensy run". Those are independent questions: A is about actuator authority,
% B is about how much phase you can afford to throw away before the unstable
% pole outruns the loop.
%
% MODEL PREREQUISITE
%   Insert between the Mux and the LQR Gain:
%
%     Mux -> Zero-Order Hold (Ts) -> Unit Delay (Ts) -> Gain(-K_lqr) -> ACTUATOR
%
%   Both blocks: Sample time = Ts (the workspace variable, NOT a literal).
%   Add a Manual Switch bypassing BOTH so the continuous path stays available
%   for re-running Gate 3.
%
%   The Unit Delay is the one sample of latency between reading the sensors
%   and applying the torque. It is invisible in continuous design and is the
%   usual reason a controller that works in simulation fails on hardware.
%   Total loop delay is about 1.5*Ts: half a sample from the hold, one full
%   sample from the delay.
%
% Run the parameter and LQR cells of cubli_panel_simscape_gates.m first.

mdl = 'Cubli_sim';

%% ---------------------------------------------------------------- setup
assert(exist('p','var')==1 && exist('K_lqr','var')==1, ...
    'Run the parameter/LQR cells of cubli_panel_simscape_gates.m first.');
if ~bdIsLoaded(mdl), load_system(mdl); end

PIVOT   = [mdl '/PIVOT'];
TH0     = 0.05;            % rad, well inside the Block A envelope
TSTOP   = 5;
SET_TOL = deg2rad(2);

fprintf('\n=== BLOCK B: discrete loop ===\n');
fprintf('lambda = %.3f rad/s  ->  unstable time constant %.0f ms\n', ...
        p.lambda, 1000/p.lambda);
fprintf('wheel fundamental at omega_cap = %.1f Hz\n', p.omega_cap/(2*pi));
fprintf('wheel fundamental at omega_max = %.1f Hz\n\n', p.omega_max/(2*pi));

%% ------------------------------------------------- 1. rate sweep
f_list = [25 40 60 80 100 150 200 400 800];
nf = numel(f_list);
R = cell(1,nf);

fprintf('%7s %9s %13s %10s %10s %9s\n', ...
        'f [Hz]','Ts [ms]','theta_end deg','max|u| N m','max|w|','stable');
for i = 1:nf
    R{i} = runrate(mdl, PIVOT, f_list(i), TH0, TSTOP, K_lqr, p, SET_TOL);
    r = R{i};
    fprintf('%7d %9.2f %13.3f %10.3f %10.1f %9s\n', ...
        r.f, 1000/r.f, rad2deg(abs(r.theta_end)), r.umax, r.wmax, string(r.ok));
end
R = [R{:}];

%% ------------------------------------------------- 2. bisect the floor
ok = [R.ok];
if all(ok)
    fprintf('\nStable at every rate tested - lower f_list to find the floor.\n');
    f_min = NaN;
elseif ~any(ok)
    fprintf('\nUnstable everywhere - check the ZOH/Unit Delay sample times.\n');
    f_min = NaN;
else
    hi = min([R(ok).f]);        % lowest known-stable
    lo = max([R(~ok).f]);       % highest known-unstable
    for k = 1:8
        mid = 0.5*(lo+hi);
        r = runrate(mdl, PIVOT, mid, TH0, TSTOP, K_lqr, p, SET_TOL);
        if r.ok, hi = mid; else, lo = mid; end
    end
    f_min = hi;
    fprintf('\n--- MINIMUM STABLE LOOP RATE = %.0f Hz  (Ts = %.2f ms) ---\n', ...
            f_min, 1000/f_min);
    fprintf('    loop delay at the floor ~1.5*Ts = %.2f ms = %.2f%% of 1/lambda\n', ...
            1.5*1000/f_min, 100*1.5/f_min*p.lambda);
end

%% ------------------------------------------- 3. what actually sets the rate
% Two candidate constraints. Whichever is HIGHER is the real requirement.
f_aa_cap = p.omega_cap/(2*pi);      % wheel fundamental at the firmware cap
f_aa_max = p.omega_max/(2*pi);      % wheel fundamental at full speed
fprintf('\nCandidate constraints on f_outer:\n');
fprintf('   control stability floor        %6.0f Hz\n', f_min);
fprintf('   Nyquist on wheel fund. @cap    %6.0f Hz  (2x %.0f Hz)\n', 2*f_aa_cap, f_aa_cap);
fprintf('   Nyquist on wheel fund. @max    %6.0f Hz  (2x %.0f Hz)\n', 2*f_aa_max, f_aa_max);
fprintf('   with 5x hardware-filter relief %6.0f Hz\n', 2*f_aa_max/5);
if ~isnan(f_min)
    if 2*f_aa_max > f_min
        fprintf('   -> ANTI-ALIASING dominates. Loop rate is a filter-architecture\n');
        fprintf('      decision, not a control-bandwidth one.\n');
    else
        fprintf('   -> CONTROL STABILITY dominates. Loop rate is set by the loop.\n');
    end
end

%% ------------------------------------------- 4. discrete margin at f_outer
% Compare the continuous design against the discretised loop at the chosen
% rate. Ms == 1.0000 held exactly for continuous full-state LQR; the hold and
% delay are what destroy it.
Ts_n = 1/p.f_outer;
sysd = c2d(ss(p.A, p.B, eye(3), 0), Ts_n, 'zoh');
[Ad, Bd] = ssdata(sysd);
% augment with one sample of input delay
Aa = [Ad Bd; zeros(1,3) 0];
Ba = [zeros(3,1); 1];
Ka = [K_lqr 0];
ev = eig(Aa - Ba*Ka);
fprintf('\nDiscrete closed loop at f_outer = %d Hz (ZOH + 1 sample delay):\n', p.f_outer);
fprintf('   |z| = %s\n', sprintf('%.4f  ', abs(ev)));
fprintf('   max |z| = %.4f  (%s)\n', max(abs(ev)), ...
        string(max(abs(ev)) < 1) + " - inside unit circle means stable");

%% ------------------------------------------------------------- 5. plots
figure('Name','Block B - loop rate sweep');
subplot(2,1,1); hold on; grid on
for i = 1:numel(R)
    if R(i).f >= 40 && R(i).f <= 400
        plot(R(i).t, rad2deg(R(i).x(:,1)), 'DisplayName', sprintf('%d Hz', R(i).f));
    end
end
ylabel('\theta [deg]'); legend('Location','best'); title('recovery from 2.9 deg')
subplot(2,1,2); hold on; grid on
plot([R.f], rad2deg(abs([R.theta_end])), 'o-')
set(gca,'XScale','log'); yline(2,'r--','settle tol')
if ~isnan(f_min), xline(f_min,'k--','floor'); end
xlabel('loop rate [Hz]'); ylabel('|\theta| at t_{end} [deg]')

%% ------------------------------------------------------------ local function
function r = runrate(mdl, PIVOT, f, th0, tstop, K, p, tol)
    Ts = 1/f;
    assignin('base', 'Ts', Ts);
    set_param(PIVOT, 'PositionTargetValue', num2str(th0));
    set_param(mdl, 'SimulationCommand', 'update');
    try
        out = sim(mdl, 'StopTime', num2str(tstop), 'MaxStep', '1e-3');
        xl  = out.xlog;
        r.t = xl.Time;  r.x = squeeze(xl.Data);
        bad = false;
    catch
        r.t = 0;  r.x = nan(1,3);  bad = true;    % solver gave up = unstable
    end
    ucmd = -r.x*K(:);
    r.f         = f;
    r.Ts        = Ts;
    r.theta_end = r.x(end,1);
    r.umax      = max(abs(ucmd));
    r.wmax      = max(abs(r.x(:,3)));
    r.ok        = ~bad && abs(r.theta_end) < tol && all(isfinite(r.x(:)));
end

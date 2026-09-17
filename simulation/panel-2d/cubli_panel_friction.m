%CUBLI_PANEL_FRICTION  Block D - friction sensitivity and tolerance thresholds.
%
% We have NO friction measurements. So this script does not ask "what does our
% friction do" - it asks "how much friction can the loop tolerate before it
% matters", producing thresholds to measure against.
%
% THREE MECHANISMS, three different effects:
%
%   1. PIVOT COULOMB  tau_cp    external, on the panel.
%      Creates a STICK DEADBAND: the panel sits at a small angle without
%      moving, so the controller gets no feedback until it breaks free.
%      Deadband = asin(tau_cp/Sg). Causes stick-slip limit cycles.
%
%   2. WHEEL COULOMB  tau_cw    internal, wheel-to-panel bearing + cogging.
%      Worst at zero crossings: the wheel does not respond to small torque
%      commands, so fine control near equilibrium degrades. This is the one
%      that usually bites.
%
%   3. WHEEL VISCOUS  b_w       internal, speed-proportional.
%      Actually BENEFICIAL - a passive momentum sink that unwinds the wheel
%      with time constant Iw/b_w. Costs a little authority, buys drift immunity.
%
% EQUATIONS (tau_p external on panel, tau_w internal on wheel):
%   theta_dd = (Sg*sin(th) + tau_p - u - tau_w)/Tb
%   phi_dd   = (u + tau_w)/Iw - theta_dd
%
% Run the parameter and LQR cells of cubli_panel_simscape_gates.m first.

assert(exist('p','var')==1 && exist('K_lqr','var')==1, ...
    'Run the parameter/LQR cells of cubli_panel_simscape_gates.m first.');

K  = K_lqr(:).';
Ts = 1/p.f_outer;
T  = 20;                        % long, to catch slow limit cycles
EPS = 0.05;                     % rad/s, tanh regularisation width

fprintf('\n=== BLOCK D: friction sensitivity ===\n');
fprintf('Sg = %.4f N m, tau_cont = %.3f N m, Iw = %.3e\n\n', p.Sg, p.tau_cont, p.Iw);

%% ------------------------------------------- 1. pivot Coulomb sweep
fprintf('--- 1. PIVOT Coulomb (stick deadband) ---\n');
fprintf('%10s %10s %12s %14s %12s %8s\n', ...
    'tau_cp mNm','%tau_cont','deadband deg','|theta|_ss deg','phidot_ss','ok');
for tc = [0 0.0005 0.001 0.002 0.005 0.010 0.020]
    r = runfric(p, K, Ts, T, deg2rad(3), tc, 0, 0, EPS);
    fprintf('%10.1f %10.1f %12.2f %14.3f %12.2f %8s\n', 1000*tc, 100*tc/p.tau_cont, ...
        rad2deg(asin(min(1,tc/p.Sg))), rad2deg(r.th_ss), r.phid_ss, string(r.ok));
end

%% ------------------------------------------- 2. wheel Coulomb sweep
fprintf('\n--- 2. WHEEL Coulomb (stiction at zero crossing) ---\n');
fprintf('%10s %10s %14s %12s %12s %8s\n', ...
    'tau_cw mNm','%tau_cont','|theta|_ss deg','phidot_ss','LC amp deg','ok');
for tc = [0 0.001 0.002 0.005 0.010 0.020 0.040]
    r = runfric(p, K, Ts, T, deg2rad(3), 0, tc, 0, EPS);
    fprintf('%10.1f %10.1f %14.3f %12.2f %12.3f %8s\n', 1000*tc, 100*tc/p.tau_cont, ...
        rad2deg(r.th_ss), r.phid_ss, rad2deg(r.lc_amp), string(r.ok));
end

%% ------------------------------------------- 3. wheel viscous sweep
fprintf('\n--- 3. WHEEL viscous (passive momentum bleed) ---\n');
fprintf('%12s %14s %14s %14s %8s\n', ...
    'b Nms/rad','bleed tau s','drag@cap mNm','|theta|_ss deg','ok');
for b = [0 1e-6 1e-5 5e-5 2e-4 1e-3]
    r = runfric(p, K, Ts, T, deg2rad(3), 0, 0, b, EPS);
    bt = Inf; if b > 0, bt = p.Iw/b; end
    fprintf('%12.1e %14.1f %14.2f %14.3f %8s\n', b, bt, 1000*b*p.omega_cap, ...
        rad2deg(r.th_ss), string(r.ok));
end

%% ------------------------------------------- 4. envelope vs friction
fprintf('\n--- 4. recoverable tilt vs friction ---\n');
cases = { {'frictionless',      0,      0,      0    }, ...
          {'light',             0.001,  0.002,  1e-5 }, ...
          {'moderate',          0.003,  0.008,  5e-5 }, ...
          {'heavy',             0.008,  0.020,  2e-4 } };
for i = 1:numel(cases)
    c = cases{i};
    lo = deg2rad(1); hi = deg2rad(16);
    for k = 1:10
        mid = 0.5*(lo+hi);
        r = runfric(p, K, Ts, 8, mid, c{2}, c{3}, c{4}, EPS);
        if r.ok, lo = mid; else, hi = mid; end
    end
    fprintf('   %-14s tau_cp=%5.1f  tau_cw=%5.1f mNm  b=%.0e  ->  edge = %.2f deg\n', ...
        c{1}, 1000*c{2}, 1000*c{3}, c{4}, rad2deg(lo));
end

%% ------------------------------------------- 5. limit cycle detail
figure('Name','Block D - stick-slip');
for i = 1:3
    tc = [0 0.005 0.020];
    r = runfric(p, K, Ts, T, deg2rad(3), 0, tc(i), 0, EPS);
    subplot(2,1,1); hold on; grid on
    plot(r.t, rad2deg(r.th), 'DisplayName', sprintf('\\tau_{cw} = %.0f mNm', 1000*tc(i)));
    subplot(2,1,2); hold on; grid on
    plot(r.t, r.phid);
end
subplot(2,1,1); ylabel('\theta [deg]'); legend('Location','best'); xlim([5 20])
subplot(2,1,2); ylabel('$\dot\phi$ [rad/s]','Interpreter','latex'); xlabel('t [s]'); xlim([5 20])

%% ------------------------------------------------------------ simulator
function r = runfric(p, K, Ts, T, th0, tau_cp, tau_cw, b_w, eps)
    n = round(T/Ts);
    x = [th0; 0; 0];
    r.t = (0:n-1)'*Ts;
    r.th = zeros(n,1); r.phid = zeros(n,1); r.u = zeros(n,1);
    u = 0;
    for k = 1:n
        u = -K*x;
        u = max(min(u, p.tau_peak), -p.tau_peak);
        s = min(max((p.omega_cap - abs(x(3)))/(0.1*p.omega_cap), 0), 1);
        if sign(u) == sign(x(3)), u = u*s; end
        r.th(k) = x(1); r.phid(k) = x(3); r.u(k) = u;

        f = @(z) dyn(z, u, p, tau_cp, tau_cw, b_w, eps);
        k1=f(x); k2=f(x+Ts/2*k1); k3=f(x+Ts/2*k2); k4=f(x+Ts*k3);
        x = x + Ts/6*(k1+2*k2+2*k3+k4);
        if ~all(isfinite(x)) || abs(x(1)) > pi/2, break, end
    end
    r.th = r.th(1:k); r.phid = r.phid(1:k); r.u = r.u(1:k); r.t = r.t(1:k);
    r.ok = (k == n) && abs(r.th(end)) < deg2rad(2);
    tail = max(1, round(0.5*numel(r.th))):numel(r.th);      % last half
    r.th_ss   = mean(r.th(tail));
    r.phid_ss = mean(r.phid(tail));
    r.lc_amp  = 0.5*(max(r.th(tail)) - min(r.th(tail)));    % limit-cycle amplitude
end

function dz = dyn(z, u, p, tau_cp, tau_cw, b_w, eps)
    th = z(1); thd = z(2); phid = z(3);
    tau_p =        -tau_cp*tanh(thd /eps);                  % pivot, external
    tau_w = -(b_w*phid + tau_cw*tanh(phid/eps));            % wheel, internal
    thdd  = (p.Sg*sin(th) + tau_p - u - tau_w)/p.Tb;
    phidd = (u + tau_w)/p.Iw - thdd;
    dz = [thd; thdd; phidd];
end

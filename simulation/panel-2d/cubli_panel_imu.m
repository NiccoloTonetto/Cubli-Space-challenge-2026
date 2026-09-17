%CUBLI_PANEL_IMU  Block C - accelerometer lever arm and complementary filter.
%
% The accelerometer measures SPECIFIC FORCE, not gravity. An IMU mounted a
% distance d from the pivot sees the panel's tangential acceleration as a
% false tilt, and that error is worst exactly during recovery - when the
% estimate matters most.
%
%   f_x = thdd*d + g*sin(th)          tangential + gravity
%   f_y = g*cos(th) - thd^2*d         centripetal + gravity
%   naive tilt = atan2(f_x, f_y)
%
% At the Block A envelope edge thdd reaches -45.6 rad/s^2, so an IMU at the
% panel centre (d = 106 mm) reports 26 deg of tilt during a 10 deg recovery.
%
% This script runs three configurations at increasing realism:
%   1. TRUTH        perfect state (the Block A/B baseline)
%   2. NAIVE ACCEL  controller fed atan2(fx,fy) directly - expected to fail
%   3. COMP FILTER  gyro integrated, accelerometer used only at low frequency
%
% It is a PURE MATLAB simulation of the same 3-state plant, not Simscape.
% Faster to sweep, and Gate 3 already proved the two agree.
%
% Run the parameter and LQR cells of cubli_panel_simscape_gates.m first.

assert(exist('p','var')==1 && exist('K_lqr','var')==1, ...
    'Run the parameter/LQR cells of cubli_panel_simscape_gates.m first.');

K   = K_lqr(:).';
g   = p.g;
Ts  = 1/p.f_outer;
T   = 5;

% sensor noise (placeholders - replace with BMI270 measurements)
SIG_ACC  = 0.02;        % m/s^2 rms per axis
SIG_GYRO = 0.002;       % rad/s rms
BIAS_GYRO= 0.005;       % rad/s constant bias

fprintf('\n=== BLOCK C: IMU lever arm ===\n');
fprintf('loop rate %d Hz, Ts = %.2f ms\n', p.f_outer, 1000*Ts);
fprintf('peak thdd at 10 deg recovery = %.1f rad/s^2\n\n', ...
        (p.Sg*sin(deg2rad(10)) - 0.192)/p.Tb);

%% ---------------------------------------------- 1. three configurations
d_test = 0.050;                 % 50 mm from pivot
tau_cf = 1.0;                   % complementary filter time constant, s
th0    = deg2rad(5);

cfg = {'truth','naive','compfilter'};
lbl = {'perfect state','naive accel tilt','complementary filter'};
res = struct();
figure('Name','Block C - estimator comparison');
for i = 1:3
    r = runsim(p, K, Ts, T, th0, d_test, cfg{i}, tau_cf, SIG_ACC, SIG_GYRO, BIAS_GYRO);
    res.(cfg{i}) = r;
    subplot(3,1,1); hold on; grid on
    plot(r.t, rad2deg(r.th), 'DisplayName', lbl{i}); ylabel('\theta true [deg]')
    subplot(3,1,2); hold on; grid on
    plot(r.t, rad2deg(r.th_hat - r.th)); ylabel('estimate error [deg]')
    subplot(3,1,3); hold on; grid on
    plot(r.t, r.u); ylabel('u [N m]'); xlabel('t [s]')
    fprintf('%-24s theta_end = %7.2f deg   max est err = %6.2f deg   %s\n', ...
        lbl{i}, rad2deg(abs(r.th(end))), rad2deg(max(abs(r.th_hat-r.th))), string(r.ok));
end
subplot(3,1,1); legend('Location','best');

%% ---------------------------------------------- 2. how far out can it go
fprintf('\n--- max IMU lever arm, complementary filter, tau = %.1f s ---\n', tau_cf);
fprintf('%8s %14s %14s %8s\n','d [mm]','max est err','theta_end deg','ok');
for d = [0 0.010 0.020 0.030 0.050 0.075 0.106]
    r = runsim(p, K, Ts, T, th0, d, 'compfilter', tau_cf, SIG_ACC, SIG_GYRO, BIAS_GYRO);
    fprintf('%8.0f %14.2f %14.3f %8s\n', 1000*d, ...
        rad2deg(max(abs(r.th_hat-r.th))), rad2deg(abs(r.th(end))), string(r.ok));
end

%% ---------------------------------------------- 3. filter time constant
fprintf('\n--- filter time constant sweep, d = %.0f mm ---\n', 1000*d_test);
fprintf('%10s %14s %14s %8s\n','tau [s]','max est err','theta_end deg','ok');
for tc = [0.05 0.1 0.3 1.0 3.0 10.0]
    r = runsim(p, K, Ts, T, th0, d_test, 'compfilter', tc, SIG_ACC, SIG_GYRO, BIAS_GYRO);
    fprintf('%10.2f %14.2f %14.3f %8s\n', tc, ...
        rad2deg(max(abs(r.th_hat-r.th))), rad2deg(abs(r.th(end))), string(r.ok));
end

%% ---------------------------------------------- 4. recovery envelope
fprintf('\n--- recoverable tilt with the estimator in the loop ---\n');
for cfgi = {'truth','compfilter'}
    lo = deg2rad(1); hi = deg2rad(16);
    for k = 1:10
        mid = 0.5*(lo+hi);
        r = runsim(p, K, Ts, T, mid, d_test, cfgi{1}, tau_cf, SIG_ACC, SIG_GYRO, BIAS_GYRO);
        if r.ok, lo = mid; else, hi = mid; end
    end
    fprintf('   %-14s edge = %.2f deg\n', cfgi{1}, rad2deg(lo));
end

%% ------------------------------------------------------------ simulator
function r = runsim(p, K, Ts, T, th0, d, mode, tau_cf, sa, sg, bg)
    rng(1);                                   % repeatable noise
    n  = round(T/Ts);
    x  = [th0; 0; 0];                         % [theta, thetadot, phidot]
    th_hat = th0;  u = 0;
    r.t = (0:n-1)'*Ts;
    r.th = zeros(n,1); r.th_hat = zeros(n,1); r.u = zeros(n,1);
    alpha = tau_cf/(tau_cf + Ts);             % complementary filter weight

    for k = 1:n
        th = x(1); thd = x(2);
        thdd = (p.Sg*sin(th) - u)/p.Tb;       % true angular acceleration

        % ---- sensors
        fx = thdd*d + p.g*sin(th) + sa*randn;
        fy = p.g*cos(th) - thd^2*d + sa*randn;
        th_acc  = atan2(fx, fy);              % naive accelerometer tilt
        gyro    = thd + bg + sg*randn;

        switch mode
            case 'truth',      th_hat = th;      thd_hat = thd;
            case 'naive',      th_hat = th_acc;  thd_hat = gyro;
            case 'compfilter'
                th_hat  = alpha*(th_hat + gyro*Ts) + (1-alpha)*th_acc;
                thd_hat = gyro;
        end

        u = -K*[th_hat; thd_hat; x(3)];
        u = max(min(u, p.tau_peak), -p.tau_peak);
        s = min(max((p.omega_cap - abs(x(3)))/(0.1*p.omega_cap), 0), 1);
        if sign(u) == sign(x(3)), u = u*s; end

        r.th(k) = th;  r.th_hat(k) = th_hat;  r.u(k) = u;

        % ---- plant, RK4 on the nonlinear equations
        f = @(z) [z(2); (p.Sg*sin(z(1)) - u)/p.Tb; (1/p.Iw + 1/p.Tb)*u];
        k1=f(x); k2=f(x+Ts/2*k1); k3=f(x+Ts/2*k2); k4=f(x+Ts*k3);
        x = x + Ts/6*(k1+2*k2+2*k3+k4);
        if ~all(isfinite(x)) || abs(x(1)) > pi/2, break, end
    end
    r.th = r.th(1:k); r.th_hat = r.th_hat(1:k); r.u = r.u(1:k); r.t = r.t(1:k);
    r.ok = k == n && abs(r.th(end)) < deg2rad(2);
end

%CORNER_CASE_STUDY  Single control panel for the whole corner-balance study.
%
%  Edit CONFIG, set the RUN flags, press play. Nothing else needs touching.
%  Every function in the toolchain is exercised from here:
%     cubli_cube_params  -> mass properties, 8 corner equilibria
%     cubli_corner_plant -> per-corner Theta, Tb, gB, P, A, B, lambda
%     cubli_gains        -> reduced-order LQR + yaw projection + Ms
%     cubli_nlsim        -> nonlinear plant, sensors, actuator limits, estimator
%     cubli_maxrec       -> worst-case recovery angle by bisection
%     cubli_corner_run   -> the Simscape model (optional, slow, LAST)
%
%  Rough cost of each block on a laptop:
%     plant/gains/single   instant      recovery      ~2 s
%     nonlin  ~20 s        estimators   ~40 s         corners  ~60 s
%     actuator_map ~60 s   gain_grid    ~2 min

clear; clc; close all

%% ===================== CONFIG ===========================================

% ---- plant ----
cfg.corner_sign = [1 1 1];      % which corner to balance on
cfg.rho_scale   = [];           % [] default 0.84 | a number | 'cad'

% ---- LQR, Bryson weights (max acceptable excursion of each state) ----
cfg.qa = 0.20;                  % rad     tilt
cfg.qr = 2.0;                   % rad/s   body rate
cfg.qw = 10;                    % rad/s   wheel rate 
cfg.rt = 0.24;                  % N m     torque

% ---- actuator ----
cfg.tau_max   = 0.12;           % N m   per wheel
cfg.omega_cap = Inf;             % rad/s FIRMWARE POLICY. Inf = no cap
cfg.V_bus     = 0;              % V     0 = ignore back-EMF. 22.2 = model it
cfg.R_phase   = 0.10;           % ohm   ESTIMATE, only bites above ~1 N m

% ---- sensing ----
cfg.est       = 'mahony';       % raw | mahony | mahony_u | mahony+ | mahony_or
cfg.kP        = 5;              % CF gain. HARD CLIFF at 10 - do not raise
cfg.kI        = 0.5;
cfg.imu_pos   = 'centre';       % centre | com | [x;y;z] in cube-centre coords
cfg.acc_sd    = 3.2e-3;         % rad    raw BMI270 at 400 Hz
cfg.gyro_sd   = 2.4e-3;         % rad/s
cfg.gyro_bias_dps = [0.3 -0.2 0.15];
cfg.imu_mis_deg   = 0.0;        % IMU-to-body misalignment. >1 deg is expensive
cfg.enc_bits  = 0;              % 0 = no quantisation, else 2*pi/2^bits
cfg.delay     = 2;              % samples. 2 at 400 Hz = 5 ms

% ---- friction ----
cfg.friction  = 1;              % bearing drag from p.tau_cw / p.b_w
cfg.ff        = 1;              % friction feedforward. MANDATORY
cfg.tau_cp    = 0;              % N m  pivot friction

% ---- simulation / sweep ----
cfg.tmax      = 16;              % s   >15 if a run might be marginally unstable
cfg.fs        = 400;            % Hz  control rate
cfg.h         = 6.25e-4;        % s   integration step (rounded to fit fs)
cfg.nazi      = 4;              % tilt azimuths in the recovery sweep
cfg.hi        = 25;             % deg upper bracket for the bisection
cfg.theta0_deg  = 2.0;          % single-run initial tilt
cfg.azimuth_deg = 0;            % single-run tilt direction

% ---- sweep grids ----
cfg.grid_tau = [0.12 0.20 0.30 0.40];
cfg.grid_wc  = [40 80 120 200];
cfg.grid_qa  = [0.10 0.20 0.40];
cfg.grid_qw  = [10 20 80];
cfg.grid_rt  = [0.12 0.24];
cfg.grid_est = {'raw','mahony','mahony_u','mahony+','mahony_or'};
cfg.grid_kP  = [2 5 10 20];

%% ===================== WHAT TO RUN ======================================
run.plant        = 1;   % plant summary
run.gains        = 1;   % LQR + Ms assertion
run.single       = 1;   % one nonlinear trajectory
run.plot         = 1;   %   ... and plot it
run.recovery     = 1;   % worst-case recovery angle
run.nonlin       = 1;   % nonlinearity budget, one effect at a time
run.estimators   = 1;   % raw / mahony / mahony_u / mahony+ / mahony_or x kP
run.actuator_map = 1;   % tau_max x omega_cap
run.gain_grid    = 1;   % Bryson weight sweep
run.corners      = 1;   % all eight corners, own gains vs these gains
run.simscape     = 1;   % LAST - overwrites p, Kp, t, X in the workspace

%% ===================== 1. PLANT =========================================
p = cubli_cube_params(cfg.rho_scale);
n = find(arrayfun(@(s) isequal(s.sign, cfg.corner_sign), p.corners), 1);
assert(~isempty(n), 'corner_sign must be one of the eight [+-1 +-1 +-1]');
c = cubli_corner_plant(p, n);
[e1, e2] = tiltbasis(c.gB);
r_imu = imupos(cfg, p) - c.pos;

if run.plant
    fprintf('\n===== PLANT =====================================================\n');
    fprintf(' corner      [%+d %+d %+d]   mode %s (rho_scale %.2f)\n', c.sign, p.mode, p.rho_scale);
    fprintf(' m_total     %.4f kg     ell %.2f mm     Sg %.4f N m\n', p.m_total, c.ell*1e3, c.Sg);
    fprintf(' Is %.6e   It %.6e   m_wheel %.4f kg\n', p.Is, p.It, p.m_wheel);
    fprintf(' lambda      %s rad/s   (tau %.0f ms)\n', mat2str(round(c.lambda.',4)), 1e3/c.lambda(1));
    fprintf(' gB          %s\n', mat2str(round(c.gB.',5)));
    fprintf(' theta_eq    %.3f deg\n', rad2deg(c.theta_eq));
    fprintf(' rank(ctrb)  %d of 9   (yaw momentum is conserved)\n', rank(ctrb(c.A,c.B)));
    fprintf(' IMU lever   %.1f mm from the contact corner   (cliff at ~150 mm)\n', norm(r_imu)*1e3);
    fprintf(' h_cap       %.5f N m s at omega_cap %g\n', p.Is*cfg.omega_cap, cfg.omega_cap);
    fprintf(' static hold asin(tau/Sg) = %.2f deg at tau_max %.2f\n', rad2deg(asin(min(1,cfg.tau_max/c.Sg))), cfg.tau_max);
end

%% ===================== 2. GAINS =========================================
[Kp, K, gi] = cubli_gains(p, c, cfg.qa, cfg.qr, cfg.qw, cfg.rt);
if run.gains
    fprintf('\n===== LQR =======================================================\n');
    fprintf(' weights   qa %.2f  qr %.1f  qw %g  rt %.2f\n', gi.weights);
    fprintf(' Ms        %.6f   <- MUST be 1.000000 (asserted to 1e-5)\n', gi.Ms);
    fprintf(' marginal  %d modes (conserved momentum + yaw angle)\n', gi.marginal);
    fprintf(' slowest   %+.4f  (%.0f ms)\n', gi.slowest, -1e3/gi.slowest);
    fprintf(' max |Kp|  %.4f\n', max(abs(Kp),[],'all'));
    assert(abs(gi.Ms-1) < 1e-5, 'Ms deviates: implementation bug, not tuning');
end

%% ===================== 3. SINGLE RUN ====================================
o = mkopt(cfg, p, r_imu);
nhat = cosd(cfg.azimuth_deg)*e1 + sind(cfg.azimuth_deg)*e2;
if run.single
    r = cubli_nlsim(p, c, Kp, deg2rad(cfg.theta0_deg)*nhat, o);
    fprintf('\n===== SINGLE RUN  %.2f deg at azimuth %g deg ====================\n', cfg.theta0_deg, cfg.azimuth_deg);
    fprintf(' recovered      %d\n', r.recovered);
    fprintf(' peak tilt      %.4f deg      final %.4f deg\n', rad2deg(r.tilt_max), rad2deg(r.tilt_final));
    fprintf(' peak wheel     %.1f rad/s     peak torque %.4f N m\n', r.wheel_max, r.tau_max_used);
    fprintf(' saturation     torque %.0f%%   momentum cap %.0f%%\n', 100*r.sat_torque, 100*r.sat_wheel);
    fprintf(' estimate error peak %.4f deg   rms(last 2 s) %.4f deg\n', ...
        rad2deg(max(r.est_err)), rad2deg(rms(r.est_err(r.t >= cfg.tmax-2))));
    fprintf(' lever-arm err  peak %.4f deg\n', rad2deg(max(r.lever_err)));
    fprintf(' gyro bias      true %s -> est %s deg/s\n', ...
        mat2str(cfg.gyro_bias_dps), mat2str(round(rad2deg(r.bias_hat).',3)));
    fprintf(' steady wheel   %s rad/s  (nonzero => tilt-measurement bias)\n', ...
        mat2str(round(r.X(end,7:9),2)));
    if run.plot
        figure('Name','corner case study','Color','w');
        tiledlayout(3,1,'TileSpacing','compact');
        nexttile; plot(r.t, rad2deg(r.tilt),'LineWidth',1.2); grid on
        ylabel('tilt [deg]'); title(sprintf('corner [%+d %+d %+d], %s, kP %g', c.sign, o.est, o.kP));
        nexttile; plot(r.t, r.X(:,7:9),'LineWidth',1.1); grid on
        yline([-1 1]*cfg.omega_cap,'r--'); ylabel('wheel [rad/s]'); legend('X','Y','Z','Location','best');
        nexttile; plot(r.t, r.U,'LineWidth',1.1); grid on
        yline([-1 1]*cfg.tau_max,'r--'); ylabel('torque [N m]'); xlabel('t [s]');
    end
end

%% ===================== 4. RECOVERY ANGLE ================================
if run.recovery
    [tm, ta] = cubli_maxrec(p, c, Kp, o, cfg.nazi, cfg.hi);
    fprintf('\n===== RECOVERY ==================================================\n');
    fprintf(' WORST CASE  %.2f deg   (best azimuth %.2f)\n', tm, max(ta));
    fprintf(' per azimuth %s\n', mat2str(round(ta,2)));
end

%% ===================== 5. NONLINEARITY BUDGET ===========================
if run.nonlin
    fprintf('\n===== NONLINEARITY BUDGET =======================================\n');
    fprintf(' %-30s %9s %10s %10s\n','effect','rec[deg]','wheel@end','tau rms');
    eff = { 'ideal',                {'friction',0,'ff',0,'tau_cp',0,'delay',0,'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'}
            'wheel friction, no FF',{'ff',0,'delay',0,'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'}
            '  + feedforward',      {'delay',0,'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'}
            'pivot friction 5 mNm', {'tau_cp',5e-3,'delay',0,'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'}
            'loop delay only',      {'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'}
            'sensor noise only',    {'delay',0,'r_imu',[0;0;0]}
            'IMU lever arm only',   {'delay',0,'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0]}
            'EVERYTHING',           {} };
    for i = 1:size(eff,1)
        oi = ovr(o, eff{i,2});
        ri = cubli_nlsim(p, c, Kp, deg2rad(1)*e1, oi); ss = ri.t >= cfg.tmax-2;
        fprintf(' %-30s %9.2f %10.1f %10.4f\n', eff{i,1}, ...
            cubli_maxrec(p,c,Kp,oi,cfg.nazi,cfg.hi), max(abs(ri.X(end,7:9))), rms(ri.U(ss,1)));
    end
end

%% ===================== 6. ESTIMATORS ====================================
if run.estimators
    fprintf('\n===== ESTIMATORS (rows kP, cols estimator) ======================\n');
    fprintf(' kP  '); fprintf('%11s', cfg.grid_est{:}); fprintf('\n');
    for kP = cfg.grid_kP
        fprintf(' %3d ', kP);
        for j = 1:numel(cfg.grid_est)
            oi = ovr(o, {'kP',kP,'est',cfg.grid_est{j}});
            fprintf('%11.2f', cubli_maxrec(p,c,Kp,oi,cfg.nazi,cfg.hi));
        end
        fprintf('\n');
    end
    fprintf(' mahony+ diverges by construction: loop gain Sg*||Tb^-1||*|r|/g = %.1f\n', ...
        c.Sg*norm(inv(c.Tb))*norm(r_imu)/p.g);
    fprintf(' mahony_or is an ORACLE (true omd) - it is the bound, not an option\n');
end

%% ===================== 7. ACTUATOR MAP ==================================
if run.actuator_map
    fprintf('\n===== RECOVERY [deg] vs tau_max (rows) x omega_cap (cols) ======\n');
    fprintf(' tau \\ wc '); fprintf('%9g', cfg.grid_wc); fprintf('\n');
    for tau = cfg.grid_tau
        fprintf(' %8.2f ', tau);
        for wc = cfg.grid_wc
            oi = ovr(o, {'tau_max',tau,'omega_cap',wc});
            fprintf('%9.2f', cubli_maxrec(p,c,Kp,oi,cfg.nazi,cfg.hi));
        end
        fprintf('\n');
    end
    fprintf(' NOTE non-monotonic cells are real: a bigger cap with fixed gains\n');
    fprintf('      lets the wheel run away. Re-run the gain grid after changing it.\n');
end

%% ===================== 8. GAIN GRID =====================================
if run.gain_grid
    fprintf('\n===== GAIN GRID ================================================\n');
    res = [];
    for qa = cfg.grid_qa, for qw = cfg.grid_qw, for rt = cfg.grid_rt
        [Kg,~,gg] = cubli_gains(p, c, qa, cfg.qr, qw, rt);
        res(end+1,:) = [qa qw rt cubli_maxrec(p,c,Kg,o,cfg.nazi,cfg.hi) gg.slowest gg.Ms]; %#ok<AGROW>
    end, end, end
    [~,ix] = sort(res(:,4),'descend');
    fprintf('   qa     qw     rt  | recovery | slowest |    Ms\n');
    for i = ix(1:min(8,end)).'
        fprintf(' %5.2f %6g %6.2f | %7.2f  | %+7.3f | %.6f\n', res(i,:));
    end
    fprintf(' current cfg set: qa %.2f  qw %g  rt %.2f\n', cfg.qa, cfg.qw, cfg.rt);
end

%% ===================== 9. ALL EIGHT CORNERS =============================
if run.corners
    fprintf('\n===== MULTI-CORNER =============================================\n');
    fprintf(' sign        ell    Sg   lambda  th_eq | own-K | these-K | max Re\n');
    for k = 1:8
        ck = cubli_corner_plant(p, k);
        Kk = cubli_gains(p, ck, cfg.qa, cfg.qr, cfg.qw, cfg.rt);
        ok = mkopt(cfg, p, imupos(cfg,p) - ck.pos);
        mr = max(real(eig(ck.A - ck.B*Kp)));
        fprintf(' [%+d%+d%+d] %6.2f %6.4f %7.4f %6.3f | %5.2f | %6.2f  | %+7.3f%s\n', ...
            ck.sign, ck.ell*1e3, ck.Sg, ck.lambda(1), rad2deg(ck.theta_eq), ...
            cubli_maxrec(p,ck,Kk,ok,cfg.nazi,cfg.hi), ...
            cubli_maxrec(p,ck,Kp,ok,cfg.nazi,cfg.hi), mr, tag(mr));
    end
    fprintf(' a small positive max Re is a SLOW FALL, not a pass. Raise cfg.tmax.\n');
end

%% ===================== 9b. RECOVERY ON THE SIMSCAPE PLANT ===============
if run.ss_recovery
    fprintf('\n===== SIMSCAPE RECOVERY =========================================\n');
    fprintf(' ideal sensors + torque saturation only. No momentum cap, no\n');
    fprintf(' friction, no noise, no delay - that subset lives in cubli_nlsim.\n');
    oss = struct('mdl','simscape3d','tmax',cfg.ss_tmax,'nazi',cfg.nazi,'hi',cfg.hi, ...
                 'tau_max',cfg.tau_max,'b_w',0,'perm',cfg.ss_perm, ...
                 'patch',cfg.ss_patch,'verbose',1);
    [ts, tas, si] = cubli_maxrec_ss(p, c, Kp, oss);
    onl = ovr(o, {'omega_cap',Inf,'friction',0,'ff',0,'delay',0, ...
                  'acc_sd',0,'gyro_sd',0,'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw'});
    tn = cubli_maxrec(p, c, Kp, onl, cfg.nazi, cfg.hi);
    fprintf(' simscape    worst %.2f deg  %s   (%d runs, %.0f s)\n', ts, mat2str(round(tas,2)), si.runs, si.seconds);
    fprintf(' cubli_nlsim worst %.2f deg  (matched settings)\n', tn);
    fprintf(' AGREEMENT   %.2f %%\n', 100*abs(ts-tn)/tn);
    if ~cfg.ss_patch
        fprintf(' NOTE cfg.ss_patch = 0: Euler angles as phi and primitive rates as\n');
        fprintf('      omega. Both are only correct at q = 0. Expect ~20%% pessimism.\n');
    end
end

%% ===================== 10. SIMSCAPE (LAST) ==============================
if run.simscape
    fprintf('\n===== SIMSCAPE (overwrites p, Kp, t, X) ========================\n');
    cubli_corner_run
end

%% ===================== local functions ==================================
function o = mkopt(cfg, p, r_imu)
o = struct( ...
    'tmax',cfg.tmax, 'fs',cfg.fs, 'h',cfg.h, ...
    'tau_max',cfg.tau_max, 'omega_cap',cfg.omega_cap, ...
    'V_bus',cfg.V_bus, 'R_phase',cfg.R_phase, ...
    'friction',cfg.friction, 'ff',cfg.ff, 'tau_cp',cfg.tau_cp, ...
    'delay',cfg.delay, 'est',cfg.est, 'kP',cfg.kP, 'kI',cfg.kI, ...
    'acc_sd',cfg.acc_sd, 'gyro_sd',cfg.gyro_sd, ...
    'gyro_bias',deg2rad(cfg.gyro_bias_dps(:)), 'r_imu',r_imu(:), 'seed',1);
if cfg.enc_bits > 0, o.enc_lsb = 2*pi/2^cfg.enc_bits; else, o.enc_lsb = 0; end
if cfg.imu_mis_deg ~= 0
    [a,~] = tiltbasis([0;0;1]);  o.imu_mis = deg2rad(cfg.imu_mis_deg)*a;
else
    o.imu_mis = [0;0;0];
end
end

function o = ovr(o, kv)
for i = 1:2:numel(kv), o.(kv{i}) = kv{i+1}; end
end

function q = imupos(cfg, p)
if ischar(cfg.imu_pos) || isstring(cfg.imu_pos)
    switch lower(string(cfg.imu_pos))
        case 'centre', q = [0;0;0];
        case 'com',    q = p.com;
        otherwise, error('imu_pos must be centre, com, or a 3-vector');
    end
else
    q = cfg.imu_pos(:);
end
end

function [e1, e2] = tiltbasis(v)
[U,~,~] = svd(v(:));  e1 = U(:,2);  e2 = U(:,3);
end

function s = tag(mr)
if mr > 1e-6, s = '  UNSTABLE'; else, s = ''; end
end

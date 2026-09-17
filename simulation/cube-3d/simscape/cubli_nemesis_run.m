%CUBLI_NEMESIS_RUN  Closed-loop Simscape run for the NEMESIS cube, CORNER.
%
% Mirrors cubli_corner_run: same three fixes, same model block names.
%   1. Mux channels come out INTERLEAVED (Rx.q Rx.w Ry.q Ry.w ...), not
%      grouped. The permutation is DETECTED, not assumed.
%   2. Euler angles are not the reduced attitude. cubli_ss_patch_attitude
%      inserts the gB cross-product block. Without it yaw runs away.
%   3. Gate 3 must be run on the RAW wiring, the closed loop on the PATCHED
%      wiring. Checking stability in the wrong configuration is misleading.
%
% Run cubli_nemesis_sweep first if the weights have not been chosen.

clear; bdclose all;
mdl = 'simscape3d';
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ------------------------------------------------ 1. PLANT AND GAINS
p = cubli_nemesis_params;

QA = 0.20;   QR = 2.0;   QW = 4;   RT = 0.012;    %% <-- from the sweep
[Kp, K, gi] = cubli_nemesis_gains(p, QA, QR, QW, RT);

assert(abs(gi.Ms-1) < 1e-4,        'Ms = %.6f, expected 1.000000', gi.Ms);
assert(gi.n_marginal == 2,         '%d marginal modes, expected 2', gi.n_marginal);
assert(gi.robust_worst < 0,        'robustness grid failed');

%% ------------------------------------------------ 2. CONFIGURE THE MODEL
set_param([mdl '/T_MOUNT'], 'RotationMethod',        'ArbitraryAxis', ...
                            'RotationArbitraryAxis', mat2str(p.mount_axis.',9), ...
                            'RotationAngle',         num2str(p.mount_angle,9), ...
                            'RotationAngleUnits',    'rad');
set_param([mdl '/T_CORNER'], 'TranslationMethod',              'Cartesian', ...
                             'TranslationCartesianOffset',      mat2str(-p.corner.',9), ...
                             'TranslationCartesianOffsetUnits', 'm');
for prim = {'Rx','Ry','Rz'}
    set_param([mdl '/PIVOT'], [prim{1} 'PositionTargetValue'], '0');
end
set_param(mdl, 'RelTol','1e-8', 'MaxStep','1e-3');

%% ------------------------------------------------ 3. GATE 3, RAW WIRING
cubli_ss_patch_attitude(mdl, 'revert');
Kp_model = zeros(3,9);  assignin('base', 'Kp_model', Kp_model);

clear io
io(1) = linio([mdl '/Gain'], 1, 'openinput');
io(2) = linio([mdl '/Mux'],  1, 'openoutput');
sys = linearize(mdl, 0, io);
sn  = erase(sys.StateName, [mdl '.']);

want = {'PIVOT.Rx.q','PIVOT.Ry.q','PIVOT.Rz.q', ...
        'PIVOT.Rx.w','PIVOT.Ry.w','PIVOT.Rz.w', ...
        'Revolute_Joint2.Rz.w','Revolute_Joint1.Rz.w','Revolute_Joint.Rz.w'};
idx = zeros(1,9);
for i = 1:9
    h = find(strcmp(sn, want{i}), 1);
    assert(~isempty(h), 'state %s not found', want{i});
    idx(i) = h;
end
[Af,Bf,Cf] = ssdata(sys);
Am = Af(idx,idx);  Bm = Bf(idx,:);
eA = norm(Am - p.A)/norm(p.A);  eB = norm(Bm - p.B)/norm(p.B);
fprintf('\nGATE 3   A err %.3e   B err %.3e   (want A < 1e-6)\n', eA, eB);
if eA > 1e-6
    warning(['GATE 3 FAILED. Check: masses (m_frame + 3*m_wheel must equal ' ...
             'm_total exactly), T_MOUNT units are rad, T_CORNER sign, ' ...
             'Simscape solids reference p.* not stale numbers.']);
end

%% ------------------------------------------------ 4. DETECT THE MUX ORDER
perm = zeros(1,9);
for ch = 1:9
    [~, st] = max(abs(Cf(ch,:)));
    perm(ch) = find(idx == st, 1);
end
fprintf('mux channel -> design index : %s\n', mat2str(perm));
Sp = eye(9);  Sp = Sp(perm,:);
Kgain = Kp * Sp.';

ev = sort(real(eig(Af - Bf*Kp*Cf)), 'descend');
fprintf('closed loop, Kp unpermuted : max Re %+.4f  <- the failure mode\n', ev(1));
ev = sort(real(eig(Af - Bf*Kgain*Cf)), 'descend');
nun = sum(ev > 1e-6);
fprintf('closed loop, Kgain         : %d unstable, slowest stable %+.4f\n', ...
        nun, max(ev(ev < -1e-6)));
assert(nun == 0, '%d unstable poles with the permuted gain', nun);

%% ------------------------------------------------ 5. PATCH AND RUN
assignin('base', 'gB_ctrl', p.gB);
cubli_ss_patch_attitude(mdl, 'apply');
assignin('base', 'Kp_model', Kgain);

TH0 = 1.5;                                  %% deg, release angle
set_param([mdl '/PIVOT'], 'RxPositionTargetValue', num2str(deg2rad(TH0),9));
out = sim(mdl, 'StopTime', '20');

%% ------------------------------------------------ 6. REPORT
xl = out.xlog;  t = xl.Time;  X = squeeze(xl.Data);
[~, iperm] = sort(perm);   %% perm maps mux->design, so we need its INVERSE
X = X(:, iperm);                             %% into design order
tilt = zeros(numel(t),1);
for k = 1:numel(t), tilt(k) = norm(p.P*X(k,1:3).'); end
u = -(Kp*X.').';
u = max(min(u, p.tau_cont), -p.tau_cont);

fprintf('\n   t        tilt        |om|     wheels [X Y Z]\n');
for tq = [0 0.3 1 2 5 10 15 20]
    [~,k] = min(abs(t-tq));
    fprintf('%6.1f %9.4f deg %8.4f   [%7.2f %7.2f %7.2f]\n', ...
            t(k), rad2deg(tilt(k)), norm(X(k,4:6)), X(k,7:9));
end
fprintf('\ntilt  %.4f -> %.4f deg   peak %.4f\n', ...
        rad2deg(tilt(1)), rad2deg(tilt(end)), rad2deg(max(tilt)));
fprintf('peak |u|   %.4f N m  (%.0f %% of the %.2f clamp)\n', ...
        max(abs(u),[],'all'), 100*max(abs(u),[],'all')/p.tau_cont, p.tau_cont);
fprintf('peak wheel %.2f rad/s (%.0f %% of the %.0f cap)\n', ...
        max(abs(X(:,7:9)),[],'all'), 100*max(abs(X(:,7:9)),[],'all')/p.omega_cap, p.omega_cap);
fprintf('wheels at 20 s: [%+.3f %+.3f %+.3f] rad/s\n', X(end,7:9));
fprintf('phi perpendicular to gB by construction: max |phi.gB| = %.2e\n', ...
        max(abs(X(:,1:3)*p.gB)));

%% ------------------------------------------------ 7. PLOTS
figure('Name','NEMESIS corner balance','Color','w','Position',[80 80 900 760]);
subplot(3,1,1)
plot(t, rad2deg(X(:,1:3)),'LineWidth',1.3); grid on; hold on
plot(t, rad2deg(tilt),'k--','LineWidth',1.8)
ylabel('attitude [deg]','FontSize',12)
legend({'q_x','q_y','q_z','|tilt|'},'Location','best')
title(sprintf('NEMESIS  release %.1f deg   lambda %.2f rad/s   rt %.3f  qw %.0f', ...
      TH0, p.lambda(1), RT, QW),'FontSize',13)
subplot(3,1,2)
plot(t, X(:,7:9),'LineWidth',1.3); grid on; hold on
yline(p.omega_cap,'r--'); yline(-p.omega_cap,'r--');
ylabel('wheel rate [rad/s]','FontSize',12); legend({'X','Y','Z'},'Location','best')
subplot(3,1,3)
plot(t, u,'LineWidth',1.3); grid on; hold on
yline(p.tau_cont,'r--'); yline(-p.tau_cont,'r--');
ylabel('torque [N m]','FontSize',12); xlabel('time [s]','FontSize',12)

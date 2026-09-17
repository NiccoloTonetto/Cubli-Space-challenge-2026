%CUBLI_PANEL_DEMO  Visual 10 deg recovery for showing the team.
%
% Runs the Simscape model from a 10 deg initial tilt, writes an animation from
% Mechanics Explorer, and produces a labelled three-panel plot.
%
% 10 deg is deliberate: it is ~90 % of the recoverable envelope (11.05 deg with
% the conservative torque ceiling), so the wheel spins up to 39.2 rad/s against
% the 40 rad/s cap. It looks dramatic AND it is honest - this is the panel
% working near its limit, not a soft demo.
%
% MODEL STATE EXPECTED
%   ACTUATOR block active (saturation + wheel taper)
%   ZOH + Unit Delay: either is fine. In the loop is more honest.
%   PIVOT position target will be overwritten by this script.

mdl = 'Cubli_sim';
TH0 = deg2rad(10);
TSTOP = 4;

%% ---------------------------------------------------------------- setup
p = cubli_panel_params;

theta_max = 0.20;  rate_max = 2.0;
omega_des = 0.5*p.omega_cap;  rho_lqr = 12;
Q = diag([1/theta_max^2, 1/rate_max^2, 1/omega_des^2]);
R = rho_lqr / p.tau_cont^2;
K_lqr = lqr(p.A, p.B, Q, R);                                        %#ok<NASGU>
Ts = 1/p.f_outer;                                                   %#ok<NASGU>

if ~bdIsLoaded(mdl), load_system(mdl); end
set_param([mdl '/PIVOT'], 'PositionTargetValue', num2str(TH0));

fprintf('Cubli panel - recovery from %.1f deg\n', rad2deg(TH0));
fprintf('  lambda = %.2f rad/s (falls with a %.0f ms time constant)\n', ...
        p.lambda, 1000/p.lambda);
fprintf('  K = [%.4f %.4f %.6f]\n\n', K_lqr);

%% ---------------------------------------------------------------- run
out  = sim(mdl, 'StopTime', num2str(TSTOP), 'MaxStep', '1e-3');
xlog = out.xlog;
t = xlog.Time;  x = squeeze(xlog.Data);
th = x(:,1);  thd = x(:,2);  phid = x(:,3);
u  = max(min(-x*K_lqr(:), p.tau_cont), -p.tau_cont);

[~, i_peak] = max(abs(phid));
i_settle = find(abs(th) > deg2rad(0.5), 1, 'last');

fprintf('  peak wheel speed   %5.1f rad/s  (%.0f %% of the %.0f cap)\n', ...
        max(abs(phid)), 100*max(abs(phid))/p.omega_cap, p.omega_cap);
fprintf('  peak torque demand %5.3f N m   (%.0f %% of continuous)\n', ...
        max(abs(-x*K_lqr(:))), 100*max(abs(-x*K_lqr(:)))/p.tau_cont);
fprintf('  settled to 0.5 deg in %.2f s\n', t(i_settle));
fprintf('  final wheel speed  %5.2f rad/s (should be near zero)\n\n', phid(end));

%% ---------------------------------------------------------------- video
% Mechanics Explorer must be open - it is, after the sim above.
% PlaybackSpeedRatio < 1 slows it down; the whole event is over in ~1.5 s.
try
    smwritevideo(mdl, 'cubli_panel_recovery', ...
                 'PlaybackSpeedRatio', 0.25, 'FrameRate', 30, ...
                 'VideoFormat', 'mpeg-4');
    fprintf('  wrote cubli_panel_recovery.mp4 (4x slow motion)\n');
catch ME
    fprintf('  video skipped: %s\n', ME.message);
    fprintf('  (open Multibody Explorer, set View Convention to Y up, then rerun)\n');
end

%% ---------------------------------------------------------------- plot
figure('Name','Panel recovery from 10 deg','Color','w','Position',[100 100 760 620]);

subplot(3,1,1)
plot(t, rad2deg(th), 'LineWidth', 1.6); grid on; hold on
yline(0, 'k:'); 
plot(t(i_settle), rad2deg(th(i_settle)), 'o', 'MarkerFaceColor','w')
text(t(i_settle)+0.05, rad2deg(th(i_settle))-1.2, ...
     sprintf('settled, %.2f s', t(i_settle)), 'FontSize', 9)
ylabel('tilt  \theta  [deg]')
title(sprintf('Recovery from %.0f deg  -  falls with a %.0f ms time constant', ...
      rad2deg(TH0), 1000/p.lambda))

subplot(3,1,2)
plot(t, phid, 'LineWidth', 1.6); grid on; hold on
yline( p.omega_cap, 'r--'); yline(-p.omega_cap, 'r--');
text(t(end)*0.72, p.omega_cap*0.86, 'firmware wheel cap', 'Color','r','FontSize',9)
plot(t(i_peak), phid(i_peak), 'o', 'MarkerFaceColor','w')
text(t(i_peak)+0.05, phid(i_peak)*0.88, ...
     sprintf('%.1f rad/s', phid(i_peak)), 'FontSize', 9)
ylabel('wheel speed  [rad/s]')

subplot(3,1,3)
plot(t, u, 'LineWidth', 1.6); grid on; hold on
yline( p.tau_cont, 'r--'); yline(-p.tau_cont, 'r--');
text(t(end)*0.72, p.tau_cont*0.84, 'continuous torque limit', 'Color','r','FontSize',9)
ylabel('motor torque  [N m]'); xlabel('time  [s]')

%% -------------------------------------------------------- one-line summary
fprintf(['SUMMARY: tipped %.0f deg, caught in %.2f s by spinning a %.0f g wheel\n' ...
         '         to %.0f rad/s and then unwinding it back to rest.\n'], ...
        rad2deg(TH0), t(i_settle), 1000*p.m_wheel, max(abs(phid)));

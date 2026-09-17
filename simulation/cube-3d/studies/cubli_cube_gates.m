%CUBLI_CUBE_GATES  Six validation gates for the 3D corner-balancing cube.
%
% PREREQUISITES in the Simulink model:
%   Inport  'TorqueIn'  (3-vector) -> Simulink-PS x3 -> WHEEL_X/Y/Z  t ports
%      *** Input Handling: NO filtering, input only. Filtering adds states
%          and Gate 3 will return a 12x12 or larger Aq. ***
%   PS-Simulink x9 -> Mux -> Outport 'StateOut', in the order
%      [ qx qy qz | wx wy wz | phidot_x phidot_y phidot_z ]
%   To Workspace on the Mux output: variable 'xlog', format Timeseries
%   PIVOT: Gimbal Joint, all three primitives sensing Position AND Velocity
%   WHEEL_*: Revolute, Torque = Provided by Input, Motion = Auto,
%            sensing VELOCITY ONLY (Position adds 3 states)
%
% Some gates need a manual dialog change first - each section says which.
% Run sections individually with Ctrl+Enter.

mdl = 'simscape3d';           

%% ===================================================== 0. PARAMETERS
p = cubli_cube_params;

fprintf('\n--- cube plant, mode %s ---\n', p.mode);
fprintf('  m = %.4f kg   ell = %.2f mm   Sg = %.4f N m\n', ...
        p.m_total, p.ell*1e3, p.Sg);
fprintf('  Is = %.4e   It = %.4e\n', p.Is, p.It);
fprintf('  lambda = %s rad/s  (tau = %.0f ms)\n', ...
        sprintf('%.4f ', p.lambda), 1000/max(p.lambda));
fprintf('  mount %.4f deg about [%+.5f %+.5f %+.5f]\n', ...
        rad2deg(p.mount_angle), p.mount_axis);
fprintf('  theta_eq on the balancing corner = %.3f deg\n\n', ...
        rad2deg(p.corners(8).theta_eq));

% swing periods about the two horizontal axes at equilibrium (wheels LOCKED)
[V,~]  = eig(p.P);                       % P has eigenvalues {0,1,1}
[~,ix] = sort(diag(V.'*p.P*V), 'descend');
u1 = V(:,ix(1));  u2 = V(:,ix(2));       % two horizontal directions
Th1 = u1.'*p.Theta*u1;   Th2 = u2.'*p.Theta*u2;
T1  = 2*pi*sqrt(Th1/p.Sg);  T2 = 2*pi*sqrt(Th2/p.Sg);
Tb1 = Th1 - p.Is;           Tb2 = Th2 - p.Is;
T1f = 2*pi*sqrt(Tb1/p.Sg);  T2f = 2*pi*sqrt(Tb2/p.Sg);
fprintf('  swing periods, wheels LOCKED : %.4f s and %.4f s\n', T1, T2);
fprintf('  swing periods, wheels FREE   : %.4f s and %.4f s\n\n', T1f, T2f);

if ~bdIsLoaded(mdl), load_system(mdl); end

%% ===================================================== GATE 1 - hang test
% MANUAL: PIVOT state targets -> set the middle primitive to ~2.5 rad, Low.
%         All three WHEEL joints -> Motion = Provided by Input, fed Constant 0
%         (locks them). Zero damping everywhere.
%
% Watch Mechanics Explorer. The cube must fall AWAY from the balancing corner
% and settle with the COM directly BELOW the contact point. If it stays
% upright, gravity or the mount rotation has the wrong sign - stop and fix it.

out  = sim(mdl, 'StopTime', '10');
xlog = out.xlog;
q    = squeeze(xlog.Data(:,1:3));
fprintf('GATE 1  final attitude [%+.3f %+.3f %+.3f] rad\n', q(end,:));
fprintf('        |q| = %.3f rad = %.1f deg  (expect a large excursion)\n', ...
        norm(q(end,:)), rad2deg(norm(q(end,:))));

%% ================================== GATE 2 - swing periods, TWO of them
% MANUAL: wheels still LOCKED. Zero damping on everything. Displace the cube
%         a few degrees about ONE horizontal axis and release.
%         Repeat for the perpendicular axis, then unlock the wheels and
%         repeat both again.
%
% TWO periods, not one. If they are equal the inertia is isotropic about the
% diagonal; if not, the difference IS the anisotropy and must match the CAD
% tensor (we predict a ~0.2 % split from the Y-asymmetry).

out  = sim(mdl,'StopTime','8');
xlog = out.xlog;
t = xlog.Time;  q = squeeze(xlog.Data(:,1:3));

% tilt away from vertical - immune to yaw drift
tilt = zeros(size(t));
for i = 1:numel(t)
    R = expm([0 -q(i,3) q(i,2); q(i,3) 0 -q(i,1); -q(i,2) q(i,1) 0]);
    tilt(i) = acos(max(-1,min(1, dot(R*p.gB, p.gB))));
end

x = tilt - mean(tilt);
s = sign(x);
k = find(s(1:end-1) < 0 & s(2:end) >= 0);
tc = t(k) - x(k).*(t(k+1)-t(k))./(x(k+1)-x(k));
T  = mean(diff(tc));
fprintf('GATE 2  T = %.4f s  (%d crossings), swing %.2f deg\n', ...
        T, numel(k), rad2deg(max(tilt)-min(tilt)));

%% ============================== GATE 3 - linearisation  ** definitive **
% MANUAL: PIVOT position targets all 0, priority High. Velocity targets 0.
%         WHEEL joints back to Torque = Provided by Input, Motion = Auto.
%         Zero damping everywhere.
% The tilted equilibrium with zero torque is an EXACT equilibrium because
% T_MOUNT already applied the tilt, so no trimming is needed.

sys = linearize(mdl, 0, io);

% Simscape carries wheel ANGLES as states. They do not affect the dynamics
% (nothing depends on absolute wheel position) so the 9-state model omits
% them. Select the states we model, in the project's order.
want = {'PIVOT.Rx.q','PIVOT.Ry.q','PIVOT.Rz.q', ...
        'PIVOT.Rx.w','PIVOT.Ry.w','PIVOT.Rz.w', ...
        'Revolute_Joint2.Rz.w','Revolute_Joint1.Rz.w','Revolute_Joint.Rz.w'};
idx = zeros(1,9);
for i = 1:9
    hit = find(contains(sys.StateName, want{i}), 1);
    assert(~isempty(hit), 'state %s not found', want{i});
    idx(i) = hit;
end

[Aq_full, Bq_full] = ssdata(sys);
Aq = Aq_full(idx, idx);
Bq = Bq_full(idx, :);

if ~isequal(size(Aq), [9 9])
    error(['Aq is %dx%d, expected 9x9.\n' ...
           '  10-11 states -> input filtering on a Simulink-PS Converter\n' ...
           '  12 states    -> a WHEEL joint has Position sensing enabled'], ...
           size(Aq,1), size(Aq,2));
end

eA = norm(Aq - p.A)/norm(p.A);
eB = norm(Bq - p.B)/norm(p.B);
fprintf('GATE 3  A err = %.3e   B err = %.3e\n', eA, eB);
if eA > 1e-6 || eB > 1e-6
    fprintf('\n  Aq(1:3,4:6) should be I3:\n');  disp(Aq(1:3,4:6));
    fprintf('  Aq(4:6,1:3) vs p.A(4:6,1:3):\n'); disp(Aq(4:6,1:3)); disp(p.A(4:6,1:3));
    fprintf('  Bq(4:6,:) vs p.B(4:6,:):\n');     disp(Bq(4:6,:));   disp(p.B(4:6,:));
    warning(['MISMATCH. Gravity block scaled -> Theta or Sg. Gravity block ' ...
             'NOT SYMMETRIC -> products of inertia sign. ONE B column wrong ' ...
             '-> that wheel''s T_W*_ROT. B columns permuted -> Mux order. ' ...
             'Everything scaled -> p.g vs Mechanism Configuration.']);
end

%% ===================================================== GATE 4 - poles
lam = eig(Aq);
fprintf('GATE 4  poles: %s\n', sprintf('%+.4f ', sort(real(lam))));
fprintf('        expect two at %+.4f / %+.4f, two mirrored, five at 0\n', p.lambda);
nun = sum(real(lam) > 1e-6);
fprintf('        unstable poles found: %d (expect 2)\n', nun);
if nun ~= 2
    warning(['%d unstable poles instead of 2. One usually means a gimbal ' ...
             'primitive is locked; three means the mount rotation is wrong.'], nun);
end

%% ============================================ GATE 5 - energy conservation
% MANUAL: zero damping, TorqueIn = [0;0;0], displace the PIVOT ~20 deg.
% E = 1/2 w' Theta w + 1/2 Is sum((w.a_k + phidot_k)^2 - (w.a_k)^2) + Sg*h
% Written in the reduced form used by the plant:
%   E = 1/2 w' Tb w + 1/2 Is sum(w.a_k + phidot_k)^2 + Sg*cos(tilt)

out  = sim(mdl,'StopTime','10');
xlog = out.xlog;
q  = squeeze(xlog.Data(:,1:3));
w  = squeeze(xlog.Data(:,4:6));
pd = squeeze(xlog.Data(:,7:9));

Ebody = zeros(size(q,1),1); Ewheel = Ebody; Epot = Ebody;
for i = 1:size(q,1)
    wi = w(i,:).';
    hk = p.Aw.'*wi + pd(i,:).';
    th = norm(q(i,:));
    if th < 1e-9, R = eye(3);
    else
        k = q(i,:).'/th; K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
        R = eye(3) + sin(th)*K + (1-cos(th))*(K*K);
    end
    Ebody(i)  = 0.5*wi.'*p.Tb*wi;
    Ewheel(i) = 0.5*p.Is*(hk.'*hk);
    Epot(i)   = p.m_total*p.g*(-p.ell*dot(R*p.gB,[0;-1;0]));
end
fprintf('body  %.4e .. %.4e\n', min(Ebody), max(Ebody));
fprintf('wheel %.4e .. %.4e\n', min(Ewheel), max(Ewheel));
fprintf('pot   %.4e .. %.4e\n', min(Epot), max(Epot));

%% ================================== GATE 6 - momentum coupling, THREE runs
% MANUAL: Mechanism Configuration -> Gravity = [0 0 0].
%         PIVOT position targets 0. TorqueIn = a constant on ONE wheel only.
%         RUN THIS THREE TIMES, one wheel each: [u;0;0], [0;u;0], [0;0;u].
%
% Angular momentum about the contact must stay zero for ANY input:
%   L = Theta*w + Aw*Is*phidot
% This is the ONLY gate that catches a wheel bolted to the wrong axis.
% Gates 1-5 all pass happily with that error present.

out  = sim(mdl, 'StopTime', '3');
xlog = out.xlog;
w    = squeeze(xlog.Data(:,4:6));
pd   = squeeze(xlog.Data(:,7:9));

L = (p.Theta*w.' + p.Aw*(p.Is*eye(3))*pd.').';
fprintf('GATE 6  max |L| = %.3e N m s  (want ~0)\n', max(vecnorm(L,2,2)));
fprintf('        final w    = [%+.4f %+.4f %+.4f]\n', w(end,:));
fprintf('        final phid = [%+.4f %+.4f %+.4f]\n', pd(end,:));
fprintf('        >>> REPEAT for the other two wheels before believing this <<<\n');

% REMEMBER to restore gravity to [0 -9.80665 0] before any other gate.

%% ===================================================== GATE 7 - closed loop
% Reduced-order LQR on the controllable subsystem, then the yaw projection.
% See cubli_cube_lqr.m for the full analysis; this is the short version.

theta_max = 0.20;  rate_max = 2.0;
omega_des = 0.5*p.omega_cap;  rho_lqr = 12;
Q = diag([repmat(1/theta_max^2,1,3) repmat(1/rate_max^2,1,3) ...
          repmat(1/omega_des^2,1,3)]);
R = (rho_lqr/p.tau_cont^2)*eye(3);

vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];   % conserved yaw momentum
V2   = null(vcon.');
K    = lqr(V2.'*p.A*V2, V2.'*p.B, V2.'*Q*V2, R) * V2.';
Kp   = K * blkdiag(p.P, eye(3), eye(3));        % USE THIS IN FIRMWARE

fprintf('GATE 7  |K on conserved dir| = %.2e   |Kp on yaw angle| = %.2e\n', ...
        norm(K*(vcon/norm(vcon))), norm(Kp*[p.gB; zeros(6,1)]));
re = sort(real(eig(p.A - p.B*Kp)));
fprintf('        %d marginal mode(s), max Re of the rest = %+.4f\n', ...
        sum(re > -1e-6), max(re(re < -1e-6)));
fprintf('        (2 marginal is correct: conserved momentum + yaw angle)\n');

%% ================================================= closed-loop simulation
% MANUAL: feed TorqueIn from  -Kp * StateOut  (Gain block, Matrix(K*u)).
%         PIVOT position target ~0.05 rad on one primitive.
%         Restore gravity.

out  = sim(mdl, 'StopTime', '5', 'MaxStep', '1e-3');
xlog = out.xlog;
t = xlog.Time;  x = squeeze(xlog.Data);
u = -(Kp*x.').';

figure('Name','Cube recovery','Color','w','Position',[100 100 780 700]);
subplot(3,1,1); plot(t, rad2deg(x(:,1:3)), 'LineWidth', 1.4); grid on
ylabel('attitude [deg]'); legend('q_x','q_y','q_z','Location','best')
title('Corner-balance recovery')
subplot(3,1,2); plot(t, x(:,7:9), 'LineWidth', 1.4); grid on; hold on
yline( p.omega_cap,'r--'); yline(-p.omega_cap,'r--');
ylabel('wheel rate [rad/s]')
subplot(3,1,3); plot(t, u, 'LineWidth', 1.4); grid on; hold on
yline( p.tau_cont,'r--'); yline(-p.tau_cont,'r--');
ylabel('torque [N m]'); xlabel('t [s]')

fprintf('\nRECOVERY: |q| %.2f -> %.3f deg, peak wheel %.1f rad/s (%.0f%% of cap)\n', ...
        rad2deg(norm(x(1,1:3))), rad2deg(norm(x(end,1:3))), ...
        max(abs(x(:,7:9)),[],'all'), 100*max(abs(x(:,7:9)),[],'all')/p.omega_cap);
fprintf('          peak torque %.4f N m (%.0f%% of continuous)\n', ...
        max(abs(u),[],'all'), 100*max(abs(u),[],'all')/p.tau_cont);

%% ============================================================ CHECKLIST
% [ ] G1  falls away from the corner, COM settles below the contact
% [ ] G2  two periods match the analytic values (amplitude-corrected)
% [ ] G3  A err and B err < 1e-6, Aq is 9x9
% [ ] G4  exactly two unstable poles at +7.93 / +7.92
% [ ] G5  energy drift < 0.1 %
% [ ] G6  |L| ~ 0, run SEPARATELY for all three wheels
% [ ] G7  2 marginal modes, rest stable
% [ ] gravity restored to [0 -9.80665 0]

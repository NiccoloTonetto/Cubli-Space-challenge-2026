%CUBLI_CUBE_SIM  Automated closed-loop LQR run. No manual dialog changes.
%
%   cubli_cube_sim               balance on (+1,+1,+1), release 2 deg
%   cubli_cube_sim([-1 -1 -1])   balance on the wheel-side corner
%   cubli_cube_sim([-1 -1 -1], 5)  ... released from 5 deg
%
% Everything is set with set_param, so the only manual model requirement is
% that the blocks exist with the names used below.
%
% REQUIRED MODEL STRUCTURE
%   T_MOUNT       Rigid Transform, rotation = Arbitrary Axis
%   T_CORNER      Rigid Transform, translation = Cartesian
%   PIVOT         Gimbal Joint
%   Mux           9 inputs -> To Workspace 'xlog' AND Outport 'StateOut'
%   TorqueIn      Inport (3) -> Demux -> 3 Simulink-PS -> wheel t ports
%   The feedback Gain block is NOT used - the loop is closed in MATLAB by
%   running short steps and re-commanding. See RUN MODE below.
%
% RUN MODE
%   Uses a Constant block 'TorqueCmd' feeding TorqueIn, stepped every Ts.
%   This mirrors the discrete firmware loop exactly (ZOH + 1 sample delay),
%   which is more honest than a continuous feedback Gain and matches
%   [[Discrete Loop Test - Block B]].

function cubli_cube_sim(corner_sign, th0_deg)

if nargin < 1 || isempty(corner_sign), corner_sign = [1 1 1];  end
if nargin < 2 || isempty(th0_deg),     th0_deg     = 2.0;      end

mdl = 'simscape3d';
p   = cubli_cube_params;

%% ---------------------------------------------- pick the corner
k = find(arrayfun(@(c) isequal(c.sign, corner_sign), p.corners), 1);
assert(~isempty(k), 'corner [%d %d %d] not found', corner_sign);
c = p.corners(k);

fprintf('\n=== CORNER [%+d %+d %+d] ===\n', c.sign);
fprintf('  ell      = %.2f mm\n', c.ell*1e3);
fprintf('  theta_eq = %.3f deg\n', rad2deg(c.theta_eq));

% inertia about THIS corner
bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for i = 1:3, bodies{end+1} = {p.m_wheel, p.com_wheel{i}, p.I_wheel{i}}; end %#ok<AGROW>
Theta = zeros(3);
for i = 1:numel(bodies)
    d = bodies{i}{2} - c.pos;
    Theta = Theta + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
end

ell = c.ell;  gB = c.gB;
Sg  = p.m_total*p.g*ell;
Tb  = Theta - p.Aw*(p.Is*eye(3))*p.Aw.';
P   = eye(3) - gB*gB.';

Z = zeros(3);  I3 = eye(3);
G = Tb \ (Sg*P);
A = [ Z I3 Z ; G Z Z ; -G Z Z ];
B = [ Z ; -inv(Tb) ; inv(p.Is*eye(3)) + inv(Tb) ];

lam = sort(real(eig(A)), 'descend');
fprintf('  lambda   = %.4f, %.4f rad/s\n', lam(1), lam(2));

% mount rotation for this corner
tgt = [0; -1; 0];
ax  = cross(gB, tgt);  ax = ax/norm(ax);
ang = acos(max(-1, min(1, dot(gB, tgt))));
fprintf('  mount    = %.4f deg about [%+.5f %+.5f %+.5f]\n', rad2deg(ang), ax);

%% ---------------------------------------------- gains
theta_max = 0.20;  rate_max = 2.0;
omega_des = 0.5*p.omega_cap;  rho = 12;
Q = diag([repmat(1/theta_max^2,1,3) repmat(1/rate_max^2,1,3) ...
          repmat(1/omega_des^2,1,3)]);
R = (rho/p.tau_cont^2)*eye(3);

vcon = [zeros(3,1); Theta*gB; p.Is*gB];      % conserved yaw momentum
V2   = null(vcon.');
K    = lqr(V2.'*A*V2, V2.'*B, V2.'*Q*V2, R) * V2.';
Kp   = K * blkdiag(P, eye(3), eye(3));       % yaw-angle projection

re = sort(real(eig(A - B*Kp)));
fprintf('  Kp: %d marginal, max Re of the rest = %+.4f\n\n', ...
        sum(re > -1e-6), max(re(re < -1e-6)));

%% ---------------------------------------------- configure the model
if ~bdIsLoaded(mdl), load_system(mdl); end

set_param([mdl '/T_MOUNT'], 'AxisOfRotation', mat2str(ax.'), ...
                            'AngleOfRotation', num2str(ang), ...
                            'AngleUnits', 'rad');
set_param([mdl '/T_CORNER'], 'TranslationCartesianOffset', mat2str(-c.pos.'), ...
                             'OffsetUnits', 'm');

% release: tilt about the gimbal x primitive
set_param([mdl '/PIVOT'], 'RxPositionTargetValue', num2str(deg2rad(th0_deg)));

set_param(mdl, 'RelTol','1e-8', 'MaxStep','1e-3');

%% ---------------------------------------------- run the discrete loop
Ts    = 1/p.f_outer;
TSTOP = 4;
n     = round(TSTOP/Ts);

t  = zeros(n,1);  X = zeros(n,9);  U = zeros(n,3);
u  = zeros(3,1);                        % applied NEXT step: the 1-sample delay

simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setModelParameter('StopTime', num2str(Ts));

for i = 1:n
    set_param([mdl '/TorqueCmd'], 'Value', mat2str(u.'));
    out = sim(simIn);
    x   = squeeze(out.xlog.Data(end,:)).';

    t(i)   = (i-1)*Ts;
    X(i,:) = x.';
    U(i,:) = u.';

    u = -Kp*x;                                        % control law
    s = min(max((p.omega_cap - abs(x(7:9)))/(0.1*p.omega_cap), 0), 1);
    same = sign(u) == sign(x(7:9));
    u(same) = u(same).*s(same);                       % wheel taper
    u = max(min(u, p.tau_cont), -p.tau_cont);         % saturation

    if norm(x(1:3)) > deg2rad(30)
        fprintf('  ABORTED at t = %.2f s, tilt %.1f deg\n', t(i), rad2deg(norm(x(1:3))));
        t = t(1:i); X = X(1:i,:); U = U(1:i,:);
        break
    end
    simIn = simIn.setInitialState(out.xFinal);        % continue from here
end

%% ---------------------------------------------- report
tilt = vecnorm(X(:,1:3),2,2);
fprintf('=== RESULT ===\n');
fprintf('  released from    %.2f deg\n', rad2deg(tilt(1)));
fprintf('  final tilt       %.3f deg\n', rad2deg(tilt(end)));
fprintf('  peak wheel speed %.2f rad/s (%.0f %% of cap)\n', ...
        max(abs(X(:,7:9)),[],'all'), 100*max(abs(X(:,7:9)),[],'all')/p.omega_cap);
fprintf('  peak torque      %.4f N m (%.0f %% of continuous)\n', ...
        max(abs(U),[],'all'), 100*max(abs(U),[],'all')/p.tau_cont);
fprintf('  final wheels     [%+.2f %+.2f %+.2f] rad/s\n', X(end,7:9));

figure('Name',sprintf('Corner [%+d %+d %+d]',c.sign),'Color','w', ...
       'Position',[100 100 800 720]);
subplot(3,1,1)
plot(t, rad2deg(X(:,1:3)),'LineWidth',1.3); grid on; hold on
plot(t, rad2deg(tilt),'k--','LineWidth',1.2)
ylabel('attitude [deg]'); legend('q_x','q_y','q_z','|tilt|','Location','best')
title(sprintf('Corner [%+d %+d %+d], released from %.1f deg, \\lambda = %.2f rad/s', ...
      c.sign, th0_deg, lam(1)))
subplot(3,1,2)
plot(t, X(:,7:9),'LineWidth',1.3); grid on; hold on
yline(p.omega_cap,'r--'); yline(-p.omega_cap,'r--');
ylabel('wheel rate [rad/s]'); legend('X','Y','Z','Location','best')
subplot(3,1,3)
plot(t, U,'LineWidth',1.3); grid on; hold on
yline(p.tau_cont,'r--'); yline(-p.tau_cont,'r--');
ylabel('torque [N m]'); xlabel('time [s]')
end

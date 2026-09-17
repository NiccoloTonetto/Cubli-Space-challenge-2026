function cubli_demo(th0_deg)
%CUBLI_DEMO  Self-contained demonstration. No Simulink, no Simscape.
%   cubli_demo        recovery from 2 deg
%   cubli_demo(3.5)   recovery from 3.5 deg
if nargin<1, th0_deg = 2.0; end
p = cubli_cube_params;  I3 = eye(3); Z = zeros(3);

%% ---------- 1. THE PLANT ----------
fprintf('\n========================================================\n');
fprintf('  CUBLI  -  measured plant\n');
fprintf('========================================================\n');
fprintf('  mass                %8.4f kg\n', p.m_total);
fprintf('  COM from centre     [%+.3f %+.3f %+.3f] mm\n', p.com*1e3);
fprintf('  contact -> COM      %8.2f mm\n', p.ell*1e3);
fprintf('  gravity torque      %8.4f N m\n', p.Sg);
fprintf('  wheel inertia       %8.3e kg m^2\n', p.Is);
fprintf('\n  UNSTABLE POLE       %8.4f rad/s   -> falls with a %.0f ms time constant\n', ...
        p.lambda(1), 1000/p.lambda(1));

%% ---------- 2. THE DESIGN ----------
Q = diag([repmat(1/0.20^2,1,3) repmat(1/2.0^2,1,3) repmat(1/(0.5*p.omega_cap)^2,1,3)]);
R = (12/p.tau_cont^2)*eye(3);
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];
V2 = null(vcon.');
K  = lqr(V2.'*p.A*V2, V2.'*p.B, V2.'*Q*V2, R) * V2.';
Kp = K*blkdiag(p.P,I3,I3);

fprintf('\n--------------------------------------------------------\n');
fprintf('  CONTROL DESIGN\n');
fprintf('--------------------------------------------------------\n');
fprintf('  states              9   [ tilt(3) | body rate(3) | wheel rate(3) ]\n');
fprintf('  rank(ctrb)          %d of 9   yaw MOMENTUM is conserved\n', rank(ctrb(p.A,p.B)));
fprintf('  rank(obsv)          %d of 9   yaw ANGLE is unobservable\n', ...
        rank(obsv(p.A,[p.P Z Z; Z I3 Z; Z Z I3])));
fprintf('  -> designed on the 8-state controllable subsystem\n');
fprintf('     gain on the conserved direction  %.2e\n', norm(K*(vcon/norm(vcon))));
fprintf('     gain on the yaw angle, projected %.2e\n', norm(Kp*[p.gB; zeros(6,1)]));
w = logspace(-2,4,2000); Ms = 0;
for i=1:numel(w), Ms = max(Ms, max(svd(inv(eye(3)+K*((1j*w(i)*eye(9)-p.A)\p.B))))); end
fprintf('\n  Ms = %.6f     theory says EXACTLY 1. Any bug breaks it.\n', Ms);
cl = sort(real(eig(p.A-p.B*Kp)));
fprintf('  closed loop: %d marginal (expected), slowest controlled %.0f ms\n', ...
        sum(cl>-1e-6), -1000/max(cl(cl<-1e-6)));
fprintf('\n  Kp (3x9) - this is what runs on the Teensy:\n');
disp(Kp);

%% ---------- 3. NONLINEAR RECOVERY ----------
Ts = 1/p.f_outer;  T = 4;  n = round(T/Ts);
axn = null(p.gB.');
x = [deg2rad(th0_deg)*axn(:,1); zeros(6,1)];
X = zeros(n,9); U = zeros(n,3); t = (0:n-1)'*Ts;
for k = 1:n
    u = -Kp*x;  X(k,:) = x.';
    s  = min(max((p.omega_cap-abs(x(7:9)))/(0.1*p.omega_cap),0),1);
    sm = sign(u)==sign(x(7:9));  u(sm) = u(sm).*s(sm);
    u  = max(min(u,p.tau_cont),-p.tau_cont);  U(k,:) = u.';
    g  = @(z)[z(4:6); p.Tb\(p.Sg*p.P*z(1:3)-u); (p.Is*eye(3))\u - p.Tb\(p.Sg*p.P*z(1:3)-u)];
    k1=g(x); k2=g(x+Ts/2*k1); k3=g(x+Ts/2*k2); k4=g(x+Ts*k3);
    x = x + Ts/6*(k1+2*k2+2*k3+k4);
end
tilt = zeros(n,1);
for k=1:n, tilt(k) = norm(p.P*X(k,1:3).'); end
is = find(tilt>deg2rad(0.2),1,'last'); if isempty(is), is=1; end

fprintf('\n--------------------------------------------------------\n');
fprintf('  NONLINEAR RECOVERY FROM %.1f DEG\n', th0_deg);
fprintf('--------------------------------------------------------\n');
fprintf('  settled to 0.2 deg in   %5.2f s\n', t(is));
fprintf('  peak wheel speed        %5.1f rad/s  (%2.0f %% of the %.0f cap)\n', ...
        max(abs(X(:,7:9)),[],'all'), 100*max(abs(X(:,7:9)),[],'all')/p.omega_cap, p.omega_cap);
fprintf('  peak torque             %5.3f N m   (%2.0f %% of continuous)\n', ...
        max(abs(U),[],'all'), 100*max(abs(U),[],'all')/p.tau_cont);
fprintf('  final wheel rates       [%+.2f %+.2f %+.2f] rad/s\n\n', X(end,7:9));

%% ---------- 4. PLOTS ----------
f = figure('Name','Cubli recovery','Color','w','Position',[80 80 900 760]);
subplot(3,1,1)
plot(t, rad2deg(X(:,1:3)),'LineWidth',1.4); grid on; hold on
plot(t, rad2deg(tilt),'k--','LineWidth',2)
ylabel('attitude [deg]','FontSize',12)
legend({'q_x','q_y','q_z','|tilt|'},'Location','best','FontSize',11)
title(sprintf('Recovery from %.1f deg   -   plant falls with a %.0f ms time constant', ...
      th0_deg, 1000/p.lambda(1)),'FontSize',13)
set(gca,'FontSize',11)
subplot(3,1,2)
plot(t, X(:,7:9),'LineWidth',1.4); grid on; hold on
yline(p.omega_cap,'r--','LineWidth',1.5); yline(-p.omega_cap,'r--','LineWidth',1.5);
ylabel('wheel rate [rad/s]','FontSize',12)
legend({'X','Y','Z'},'Location','best','FontSize',11); set(gca,'FontSize',11)
subplot(3,1,3)
plot(t, U,'LineWidth',1.4); grid on; hold on
yline(p.tau_cont,'r--','LineWidth',1.5); yline(-p.tau_cont,'r--','LineWidth',1.5);
ylabel('torque [N m]','FontSize',12); xlabel('time [s]','FontSize',12)
set(gca,'FontSize',11)
assignin('base','demo_t',t); assignin('base','demo_X',X); assignin('base','demo_Kp',Kp);
fprintf('  figures drawn. t, X, Kp exported to the workspace as demo_*\n');
fprintf('  run  cubli_demo_anim  for the 3D animation\n\n');
end

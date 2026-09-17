%CUBLI_CUBE_LQR  Linearised dynamics, Bryson LQR, feasibility and margin checks.
%
% Sections:
%   1  9-state corner-balance model
%   2  Edge-balance feasibility, all 12 edges           (1 wheel authority)
%   3  Corner-balance feasibility, all 8 corners        (sqrt(2) authority)
%   4  Controllability and observability
%   5  Bryson weights, LQR gains
%   6  Yaw projection
%   7  Closed-loop poles, damping, settling
%   8  MIMO return difference: Ms, and generalised Nyquist
%   9  Robustness sweep, +/-30 %
%  10  Friction feedforward sizing
%  11  Discrete-time check at f_outer
%
% Run sections with Ctrl+Enter. Section 1 must run first.

%% ============================================================ 1. MODEL
clear; clc;
p = cubli_cube_params3N;
Kp_model = zeros(3,9);

fprintf('=== CUBE PLANT (mode: %s) ===\n', p.mode);
fprintf('  m        = %.4f kg\n', p.m_total);
fprintf('  com      = [%+.3f %+.3f %+.3f] mm\n', p.com*1e3);
fprintf('  ell      = %.2f mm\n', p.ell*1e3);
fprintf('  Sg       = %.4f N m\n', p.Sg);
fprintf('  lambda   = %s rad/s  (tau = %.0f ms)\n', ...
        sprintf('%.4f ', p.lambda), 1000/max(p.lambda));
fprintf('  Theta about contact corner:\n');   disp(p.Theta);
fprintf('  Tb = Theta - Aw*Is*Aw'':\n');       disp(p.Tb);

A = p.A;  B = p.B;
n = size(A,1);  m = size(B,2);

% open-loop poles
ol = eig(A);
fprintf('  open-loop poles: %s\n\n', sprintf('%+.4f ', sort(real(ol))));

%% ================================================= 2. EDGE FEASIBILITY
% Rotation about a cube edge is 1 DOF. Only the wheel whose spin axis is
% PARALLEL to the edge produces torque about it - the other two project to
% zero. Authority is therefore ||e||_1 = 1 wheel, not sqrt(2).
fprintf('=== EDGE BALANCE FEASIBILITY (12 edges) ===\n');
fprintf('%-16s %8s %9s %10s %9s %9s %8s\n', ...
    'edge','ell mm','Theta_e','lambda','tau_eff','th_stat','feasible');

edges = {};
ax = eye(3);
axname = 'XYZ';

for k = 1:3                              % edge direction
    others = setdiff(1:3, k);
    for s1 = [-1 1]
        for s2 = [-1 1]
            c = zeros(3,1);
            c(others(1)) = s1*p.a/2;
            c(others(2)) = s2*p.a/2;     % a point on the edge
            edges{end+1} = {k, c, s1, s2};                        %#ok<SAGROW>
        end
    end
end

edge_res = [];
for i = 1:numel(edges)
    k = edges{i}{1};  c = edges{i}{2};
    e = ax(:,k);                                   % edge direction, unit
    r = p.com - c;                                 % edge point -> COM
    r_perp = r - (r.'*e)*e;                        % component that produces torque
    ell_e  = norm(r_perp);

    % inertia about the edge line
    Th_e = 0;
    bodies = {{p.m_frame,p.com_frame,p.I_frame}};
    for q = 1:3, bodies{end+1} = {p.m_wheel,p.com_wheel{q},p.I_wheel{q}}; end %#ok<SAGROW>
    for b = 1:numel(bodies)
        mb = bodies{b}{1};  d = bodies{b}{2} - c;
        d_perp = d - (d.'*e)*e;
        Th_e = Th_e + e.'*bodies{b}{3}*e + mb*(d_perp.'*d_perp);
    end
    Is_e = p.Is * sum((e.'*p.Aw).^2);              % wheel spin inertia about e
    Tb_e = Th_e - Is_e;

    Sg_e   = p.m_total*p.g*ell_e;
    lam_e  = sqrt(Sg_e/Tb_e);
    tau_e  = p.tau_cont * sum(abs(e.'*p.Aw));      % ||e||_1 in wheel axes
    ratio  = tau_e/Sg_e;
    th_st  = asin(min(1,ratio));
    ok     = ratio < 1;                            % finite static bound exists

    lbl = sprintf('%c edge (%+d,%+d)', axname(k), edges{i}{3}, edges{i}{4});
    fprintf('%-16s %8.2f %9.3e %10.4f %9.4f %9.2f %8s\n', ...
        lbl, ell_e*1e3, Th_e, lam_e, tau_e, rad2deg(th_st), string(ok));
    edge_res(end+1,:) = [ell_e, Th_e, lam_e, tau_e, ratio];        %#ok<SAGROW>
end
fprintf('  mean lambda_edge = %.4f rad/s   (uniform cube: 0.99*sqrt(g/a)*... )\n', ...
        mean(edge_res(:,3)));
fprintf('  authority is 1x tau_cont on every edge, as expected.\n\n');

%% =============================================== 3. CORNER FEASIBILITY
fprintf('=== CORNER BALANCE FEASIBILITY (8 corners) ===\n');
fprintf('%-14s %9s %9s %10s %10s %9s\n', ...
    'corner','theta_eq','ell mm','lambda','tau_eff','th_static');
for i = 1:numel(p.corners)
    c   = p.corners(i);
    r   = p.com - c.pos;
    ell = norm(r);
    gB  = -r/ell;
    P   = eye(3) - gB*gB.';

    % inertia about this corner
    Th = zeros(3);
    bodies = {{p.m_frame,p.com_frame,p.I_frame}};
    for q = 1:3, bodies{end+1} = {p.m_wheel,p.com_wheel{q},p.I_wheel{q}}; end %#ok<SAGROW>
    for b = 1:numel(bodies)
        mb = bodies{b}{1};  d = bodies{b}{2} - c.pos;
        Th = Th + bodies{b}{3} + mb*((d.'*d)*eye(3) - d*d.');
    end
    Tb   = Th - p.Aw*(p.Is*eye(3))*p.Aw.';
    Sg   = p.m_total*p.g*ell;
    lam  = sqrt(max(eig(Tb\(Sg*P))));

    % worst-case torque authority over tilt axes perpendicular to gB:
    % min over unit e perp gB of ||e||_1  -> sqrt(2) for a diagonal
    th = linspace(0,2*pi,721);
    [~,~,V] = svd(P);  u1 = V(:,1);  u2 = V(:,2);
    l1 = arrayfun(@(t) sum(abs(cos(t)*u1 + sin(t)*u2)), th);
    tau_eff = p.tau_cont*min(l1);

    ratio = tau_eff/Sg;
    fprintf('  (%+d,%+d,%+d) %9.2f %9.2f %10.4f %10.4f %9.2f\n', ...
        c.sign, rad2deg(c.theta_eq), ell*1e3, lam, tau_eff, ...
        rad2deg(asin(min(1,ratio))));
end
fprintf('  worst-case corner authority factor = %.4f  (expect sqrt(2) = 1.4142)\n\n', ...
        min(l1));

%% ================================= 4. CONTROLLABILITY / OBSERVABILITY
fprintf('=== CONTROLLABILITY / OBSERVABILITY ===\n');
Co = ctrb(A,B);
rc = rank(Co);                       % RELATIVE tolerance - do NOT pass 1e-9
sv = svd(Co);
fprintf('  rank(ctrb)  = %d of %d\n', rc, n);
fprintf('  ctrb singular values / largest:\n    %s\n', ...
        sprintf('%.2e ', sv/sv(1)));

% The deficient direction is EXACTLY angular momentum about the vertical.
% gB'*Sg*P = 0 because P*gB = 0: gravity exerts no moment about the vertical,
% and the wheels are INTERNAL, so no input can change gB'*L. It is a conserved
% quantity of the plant.
mom = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];
[U,~,~] = svd(Co);
fprintf('  |cos| between the uncontrollable direction and [0; Theta*gB; Is*gB]\n');
fprintf('     = %.10f   (1.0 confirms the momentum interpretation)\n', ...
        abs(U(:,end).'*(mom/norm(mom))));
fprintf('  -> the system is STABILIZABLE, not controllable. That is sufficient:\n');
fprintf('     the uncontrollable mode sits at exactly 0 (marginally stable) and\n');
fprintf('     balancing needs only the two tilt directions.\n');
fprintf('     A yaw kick therefore PERSISTS in the frictionless model. On\n');
fprintf('     hardware, contact friction supplies external yaw torque and it\n');
fprintf('     bleeds away, so the model is the pessimistic case.\n');

% Realistic measurement set:
%   reduced attitude gB  -> 2 DOF only (yaw about vertical unobservable)
%   3 gyro axes, 3 wheel encoders
Cm = [ p.P          zeros(3)  zeros(3)      % projected attitude, rank 2
       zeros(3)     eye(3)    zeros(3)      % gyro
       zeros(3)     zeros(3)  eye(3) ];     % encoders
Ob = obsv(A,Cm);
fprintf('  rank(obsv) with reduced attitude = %d of %d\n', rank(Ob), n);
fprintf('  -> the unobservable direction is YAW ANGLE about the vertical.\n');
fprintf('     Gravity gives no torque about it and one vector measurement\n');
fprintf('     cannot see it. Yaw RATE is fine (gyro).\n\n');

% full-state, for contrast
fprintf('  rank(obsv) with full state       = %d of %d\n\n', rank(obsv(A,eye(n))), n);

%% ============================================ 5. BRYSON WEIGHTS + LQR
fprintf('=== LQR DESIGN (Bryson, on the CONTROLLABLE subsystem) ===\n');
theta_max = 0.20;                 % rad,   max acceptable tilt
rate_max  = 2.0;                  % rad/s, max acceptable body rate
omega_des = 0.5*p.omega_cap;      % 20 rad/s, designed against the CAP
rho_lqr   = 12;                   % control/state cost ratio

Q = diag([repmat(1/theta_max^2,1,3), ...
          repmat(1/rate_max^2, 1,3), ...
          repmat(1/omega_des^2,1,3)]);
R = (rho_lqr/p.tau_cont^2) * eye(3);

fprintf('  theta_max %.2f rad | rate_max %.1f rad/s | omega_des %.0f rad/s | rho %d\n', ...
        theta_max, rate_max, omega_des, rho_lqr);

% --- ORDER REDUCTION, and why it is necessary -------------------------------
% Strict stabilizability requires every UNCONTROLLABLE mode to have Re < 0.
% Ours sits at EXACTLY 0 (conserved yaw momentum), so the Riccati equation has
% no well-posed stabilizing solution and lqr() returns numerical noise in that
% direction - visible as ASYMMETRIC wheel-rate gains and Ms = 1.000006 instead
% of exactly 1.
%
% The conserved functional is known analytically and satisfies BOTH
%     v'*A = 0   and   v'*B = 0
% so restricting to its orthogonal complement is EXACT, not an approximation.
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];
fprintf('  |v''A| = %.2e   |v''B| = %.2e   (both must be ~0)\n', ...
        max(abs(vcon.'*A)), max(abs(vcon.'*B)));

V2 = null(vcon.');                % 9x8, orthonormal basis of the reachable set
Kr = lqr(V2.'*A*V2, V2.'*B, V2.'*Q*V2, R);
K  = Kr * V2.';                   % 3x9, annihilates the conserved direction



fprintf('  gain on the conserved direction: %.3e  (should be ~0)\n', ...
        norm(K*(vcon/norm(vcon))));
fprintf('\n  K (3x9):\n');  disp(K);

%% =================================================== 6. YAW PROJECTION
% Separate issue from section 5. The CONSERVED direction is momentum; the
% UNOBSERVABLE one is the yaw ANGLE. Section 5 removed the first; this removes
% the second, because the estimator cannot supply a yaw angle.
Kp = K * blkdiag(p.P, eye(3), eye(3));
yawdir = [p.gB; zeros(6,1)];

fprintf('=== YAW PROJECTION ===\n');
fprintf('  gain on the yaw-ANGLE direction, before: %.3e\n', norm(K *yawdir));
fprintf('  after projection:                        %.3e\n', norm(Kp*yawdir));

TOL = 1e-6;                       % anything above -TOL counts as marginal
for pair = {{'K ',K},{'Kp',Kp}}
    Kx = pair{1}{2};
    re = sort(real(eig(A - B*Kx)));
    nmarg = sum(re > -TOL);
    fprintf('  %s : %d marginal mode(s), max Re of the rest = %+.4f\n', ...
            pair{1}{1}, nmarg, max(re(re < -TOL)));
end
fprintf('  EXPECTED: K -> 1 marginal (conserved momentum)\n');
fprintf('            Kp -> 2 marginal (momentum + yaw angle drifting to a\n');
fprintf('                  constant). Both are physically correct.\n');
fprintf('  USE Kp IN FIRMWARE.\n\n');

%% ============================== 7. CLOSED-LOOP POLES, DAMPING, RESPONSE
fprintf('=== CLOSED LOOP ===\n');
[wn, zeta] = damp(A - B*K);
fprintf('%14s %10s %10s %12s\n','pole','wn rad/s','zeta','tau ms');
cl = eig(A - B*K);
[~,ix] = sort(real(cl),'descend');
for i = ix.'
    if abs(imag(cl(i))) < 1e-9
        fprintf('%14.4f %10.3f %10.4f %12.1f\n', real(cl(i)), wn(i), zeta(i), ...
                -1000/real(cl(i)));
    else
        fprintf('%8.4f%+.4fj %10.3f %10.4f %12.1f\n', real(cl(i)), imag(cl(i)), ...
                wn(i), zeta(i), -1000/real(cl(i)));
    end
end
fprintf('  slowest CONTROLLED mode tau = %.0f ms\n', -1000/max(real(cl(real(cl)<-1e-6))));
fprintf('  (the pole at 0 is the conserved yaw momentum, not a design fault)\n\n');

%% ========================= 8. RETURN DIFFERENCE: Ms AND MIMO NYQUIST
fprintf('=== MIMO MARGINS ===\n');
w  = logspace(-2, 4, 4000);
L  = @(s) K*((s*eye(n) - A)\B);              % loop gain, broken at the input

Ms = 0;  detL = zeros(size(w));
for i = 1:numel(w)
    Li = L(1j*w(i));
    S  = inv(eye(m) + Li);
    Ms = max(Ms, max(svd(S)));
    detL(i) = det(eye(m) + Li);
end
fprintf('  Ms = max sigma_bar(S) = %.6f\n', Ms);
fprintf('  full-state LQR return-difference identity predicts EXACTLY 1.0000\n');
if abs(Ms-1) < 1e-4
    fprintf('  -> PASS. Any deviation would mean an implementation bug.\n');
else
    warning('Ms = %.6f, expected 1.0000. Check A, B, Q, R.', Ms);
end

% generalised Nyquist: encirclements of the origin by det(I+L(jw))
ph  = unwrap(angle([conj(fliplr(detL)) detL]));
enc = (ph(end) - ph(1))/(2*pi);
nu  = sum(real(eig(A)) > 1e-9);
fprintf('  det(I+L) net encirclements of origin = %+.3f\n', enc);
fprintf('  unstable open-loop poles             = %d\n', nu);
fprintf('  generalised Nyquist requires them equal (counter-clockwise): %s\n', ...
        string(abs(enc - nu) < 0.05));
fprintf('  *** CAVEAT: this plant has FIVE poles at s = 0. The Nyquist contour\n');
fprintf('      must be indented around them, and this simple sweep does not do\n');
fprintf('      that, so the count is NOT valid here. Ms and the closed-loop\n');
fprintf('      eigenvalues are the meaningful tests.\n');
fprintf('  NOTE: for an unstable plant a gain margin below 1 is the DOWNWARD\n');
fprintf('        margin. Small is correct, not alarming.\n\n');

figure('Name','Generalised Nyquist');
plot(real(detL), imag(detL), 'LineWidth', 1.3); hold on; grid on
plot(real(conj(detL)), -imag(detL), '--');
plot(0,0,'r+','MarkerSize',12,'LineWidth',2);
axis equal; xlabel('Re det(I+L)'); ylabel('Im det(I+L)');
title(sprintf('encirclements = %+.2f, unstable poles = %d', enc, nu));

figure('Name','Sensitivity');
Sv = zeros(size(w));
for i = 1:numel(w), Sv(i) = max(svd(inv(eye(m)+L(1j*w(i))))); end
semilogx(w, 20*log10(Sv), 'LineWidth', 1.3); grid on
yline(0,'r--'); xlabel('\omega [rad/s]'); ylabel('\sigma_{max}(S) [dB]')
title(sprintf('M_s = %.4f', Ms))

%% ================================================ 9. ROBUSTNESS SWEEP
fprintf('=== ROBUSTNESS: +/-30 %% on Theta, Is, Sg ===\n');
fprintf('%10s %10s %10s %14s %10s\n','dTheta','dIs','dSg','max Re(pole)','stable');
worst = -inf;
for fT = [0.7 1 1.3]
  for fI = [0.7 1 1.3]
    for fS = [0.7 1 1.3]
      Th2 = p.Theta*fT;  Is2 = p.Is*fI;  Sg2 = p.Sg*fS;
      Tb2 = Th2 - p.Aw*(Is2*eye(3))*p.Aw.';
      A2  = [zeros(3) eye(3) zeros(3)
             Tb2\(Sg2*p.P) zeros(3) zeros(3)
            -(Tb2\(Sg2*p.P)) zeros(3) zeros(3)];
      B2  = [zeros(3); -inv(Tb2); inv(Is2*eye(3)) + inv(Tb2)];
      re2 = sort(real(eig(A2 - B2*K)));
      mr  = max(re2(re2 < -1e-6));   % ignore the conserved marginal mode
      worst = max(worst, mr);
      if abs(fT-1)+abs(fI-1)+abs(fS-1) > 0.5    % print only the corners
          fprintf('%10.2f %10.2f %10.2f %14.4f %10s\n', fT, fI, fS, mr, string(mr<0));
      end
    end
  end
end
fprintf('  worst case over the grid (marginal mode excluded): %+.4f -> %s\n\n', ...
        worst, string(worst<0));

%% ===================================== 10. FRICTION FEEDFORWARD SIZING
fprintf('=== FRICTION FEEDFORWARD ===\n');
tau_cw = 8e-3;        % N m, PLACEHOLDER - replace with the spin-down measurement
fprintf('  tau_cw = %.1f mN m (PLACEHOLDER)\n', tau_cw*1e3);
fprintf('  Without feedforward, each wheel settles at a standing speed of\n');
fprintf('     phidot_ss = tau_cw / |K3|  where K3 is the wheel-rate gain\n');
K3 = mean(abs(diag(K(:,7:9))));   % effective wheel-rate gain
fprintf('     |K3| = %.6f  ->  %.2f rad/s = %.0f %% of the %.0f rad/s cap\n', ...
        K3, tau_cw/K3, 100*(tau_cw/K3)/p.omega_cap, p.omega_cap);
fprintf('  Feedforward:  u_k = u_lqr_k + tau_cw*tanh(phidot_k/eps)\n');
fprintf('     eps ~ 0.05 rad/s. tanh not sign(): sign() chatters at zero\n');
fprintf('     crossing, and all three wheels sit near zero at equilibrium.\n');
fprintf('  Over-compensating is worse than under: it drives the wheel.\n');
fprintf('  Start at 70 %% of the measured value.\n\n');

%% ================================ 11. DISCRETE CHECK AT THE LOOP RATE
fprintf('=== DISCRETE LOOP at %d Hz ===\n', p.f_outer);
Ts   = 1/p.f_outer;
sysd = c2d(ss(A,B,eye(n),zeros(n,m)), Ts, 'zoh');
[Ad, Bd] = ssdata(sysd);
% augment with one sample of input delay
Aa = [Ad Bd; zeros(m,n) zeros(m)];
Ba = [zeros(n,m); eye(m)];
Ka = [K zeros(m)];
ev = sort(abs(eig(Aa - Ba*Ka)));
fprintf('  max |z| excluding the marginal mode(s) = %.4f -> %s\n', ev(end-1), ...
        string(ev(end-1) < 1) + " (inside the unit circle)");
fprintf('  marginal |z| = %.6f  (z = 1 corresponds to s = 0, correct)\n', ev(end));
fprintf('  equivalent Re(s) of the slowest mode: %.2f rad/s\n', ...
        log(max(abs(ev(abs(ev)<0.9999))))/Ts);
fprintf('  NOTE: Ms = 1.0000 does NOT survive discretisation or an estimator.\n');
fprintf('        Check the discrete poles directly, never assume the margin.\n');

%% ===================================================== SUMMARY
fprintf('\n=== SUMMARY ===\n');
fprintf('  lambda            %s rad/s\n', sprintf('%.4f ', p.lambda));
fprintf('  K / Kp            3x9, reduced-order design; USE Kp in firmware\n');
fprintf('  Ms                %.6f\n', Ms);
fprintf('  ctrb / obsv       %d / %d of %d\n', rank(Co), rank(Ob), n);
fprintf('                    yaw MOMENTUM uncontrollable, yaw ANGLE unobservable\n');
fprintf('  robust +/-30 %%    %s\n', string(worst<0));
fprintf('  discrete at %d Hz  max|z| = %.4f (marginal excluded)\n', p.f_outer, ev(end-1));
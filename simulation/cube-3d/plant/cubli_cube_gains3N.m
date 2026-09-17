function [Kp, K, info] = cubli_cube_gains3N(qa, qr, qw, rt, rho)
%CUBLI_CUBE_GAINS3N  Reduced-order Bryson LQR for the 3N cube, with checks.
%
%   [Kp,K,info] = cubli_cube_gains3N()                 recommended set
%   [Kp,K,info] = cubli_cube_gains3N(0.20,2.0,10,0.24,12)
%
%   USE Kp IN FIRMWARE, not K.  Kp = K*blkdiag(P,I,I) removes the gain on
%   the unobservable yaw ANGLE; K still carries it.
%
% WEIGHTS
%   qa  rad     max acceptable tilt                    0.20
%   qr  rad/s   max acceptable body rate               2.0
%   qw  rad/s   wheel-speed weighting scale            10   = 0.25*omega_cap
%   rt  N m     torque scale in R = rho/rt^2           0.24
%   rho -       control/state cost ratio               12
%
%   rho and rt are ONE knob: only R = rho/rt^2 matters.  qw is the knob that
%   actually changes behaviour.  SMALLER qw = tighter momentum regulation.
%   At qw = 20 the wheel-rate gain collapses and the standing wheel speed
%   reaches 92 %% of the cap at rest; at qw = 10 it is 10 %%.
%
% Written 18/08/2026 alongside the corner/rho_scale/m_extra fixes.

if nargin < 5 || isempty(rho), rho = 12;   end
if nargin < 4 || isempty(rt),  rt  = 0.24; end
if nargin < 3 || isempty(qw),  qw  = 10;   end
if nargin < 2 || isempty(qr),  qr  = 2.0;  end
if nargin < 1 || isempty(qa),  qa  = 0.20; end

p = cubli_cube_params3N;

% ---- plant sanity, before designing anything against it --------------------
assert(p.m_extra >= 0,                'negative fastener mass');
assert(abs(p.ell - 0.866*p.a) < 0.020,'ell not near the half-diagonal');
assert(max([p.corners.theta_eq]) < deg2rad(6), 'equilibrium tilt too large');
tol = max(abs(diag(p.Theta)))*0.25;
assert(max(diag(p.Theta))-min(diag(p.Theta)) < tol, 'Theta not near-isotropic');

% ---- Bryson weights ---------------------------------------------------------
Q = diag([repmat(1/qa^2,1,3), repmat(1/qr^2,1,3), repmat(1/qw^2,1,3)]);
R = (rho/rt^2)*eye(3);

% ---- exact 8-state reduction ------------------------------------------------
% Angular momentum about the vertical is conserved: the wheels are internal and
% gB'' * Sg * P = 0 exactly.  That direction is uncontrollable, so it must be
% removed BEFORE the Riccati solve, not stabilised afterwards.
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];
V2   = null(vcon.');
Ar = V2.'*p.A*V2;  Br = V2.'*p.B;  Qr = V2.'*Q*V2;

Kr = lqr(Ar, Br, Qr, R);
K  = Kr*V2.';
Kp = K*blkdiag(p.P, eye(3), eye(3));

% ---- checks -----------------------------------------------------------------
Ms  = hinfnorm(feedback(eye(3), ss(Ar,Br,Kr,zeros(3))));
cl  = eig(p.A - p.B*Kp);
slow = -1/max(real(cl(real(cl) < -1e-6)));
K1 = abs(diag(Kp(:,1:3)));  K3 = abs(diag(Kp(:,7:9)));
wss = p.tau_cw/mean(K3);
zd  = sort(abs(eig(c2d(ss(p.A-p.B*Kp,p.B,eye(9),0), 1/p.f_outer).A)),'descend');

worst = -inf;
for dT = [0.7 1 1.3], for dI = [0.7 1 1.3], for dS = [0.7 1 1.3]
    Tb2 = p.Tb*dT;  Is2 = p.Is*dI;
    G2  = Tb2\(p.Sg*dS*p.P);
    A2  = [zeros(3) eye(3) zeros(3); G2 zeros(3) zeros(3); -G2 zeros(3) zeros(3)];
    B2  = [zeros(3); -inv(Tb2); eye(3)/Is2 + inv(Tb2)];
    e   = eig(A2 - B2*Kp);  e = e(abs(real(e)) > 1e-6);
    worst = max(worst, max(real(e)));
end, end, end

info = struct('Ms',Ms,'slowest_ms',slow*1000,'K1',K1,'K3',K3, ...
              'standing_wheel',wss,'maxz',zd(3),'worst_robust',worst,'p',p);

fprintf('=== CUBE 3N GAINS  (qa %.2f / qr %.1f / qw %g / rt %.2f / rho %g) ===\n', qa,qr,qw,rt,rho);
fprintf('  corner       [%.3f %.3f %.3f]   ell %.2f mm   Sg %.4f N m\n', p.corner, p.ell*1000, p.Sg);
fprintf('  m_total      %.4f kg   lambda %.4f %.4f rad/s\n', p.m_total, p.lambda(1), p.lambda(2));
fprintf('  Ms           %.6f   %s\n', Ms, ternary(abs(Ms-1)<1e-5,'PASS','FAIL - implementation bug'));
fprintf('  slowest mode %.1f ms\n', slow*1000);
fprintf('  |K1| diag    %.3f %.3f %.3f   spread %.3f  %s\n', K1, max(K1)/min(K1), ...
        ternary(max(K1)/min(K1)<1.3,'PASS','FAIL - check p.corner'));
fprintf('  |K3| diag    %.6f %.6f %.6f\n', K3);
fprintf('  standing wheel speed at tau_cw = %.0f mN m: %.2f rad/s = %.0f %% of cap  %s\n', ...
        p.tau_cw*1000, wss, 100*wss/p.omega_cap, ternary(wss<0.2*p.omega_cap,'PASS','FAIL - lower qw'));
fprintf('  discrete %g Hz  max|z| = %.4f  %s\n', p.f_outer, zd(3), ternary(zd(3)<1,'PASS','FAIL'));
fprintf('  robust +/-30%% on Theta,Is,Sg: worst max Re = %.4f  %s\n', worst, ternary(worst<0,'PASS','FAIL'));
fprintf('\n  Kp (3x9), USE THIS IN FIRMWARE:\n');
fprintf('   %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\n', Kp.');
end

function s = ternary(c,a,b), if c, s = a; else, s = b; end, end

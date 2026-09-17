function [Kp, K, info] = cubli_nemesis_gains(p, qa, qr, qw, rt, verbose)
%CUBLI_NEMESIS_GAINS  Reduced-order LQR for the NEMESIS cube, CORNER balance.
%
%   [Kp,K,info] = cubli_nemesis_gains(p)
%   [Kp,K,info] = cubli_nemesis_gains(p, 0.20, 2.0, 10, 0.24)
%
%   Kp   3x9, YAW-PROJECTED.   *** USE THIS IN FIRMWARE ***
%   K    3x9, unprojected. Analysis only.
%
% BRYSON WEIGHTS - envelope commitments, not tuning knobs.
%   qa  rad     max acceptable tilt        Q11 = 1/qa^2
%   qr  rad/s   max acceptable body rate   Q22 = 1/qr^2
%   qw  rad/s   wheel-speed budget         Q33 = 1/qw^2
%   rt  N m     torque budget              R   = 1/rt^2
%
% qw IS WEIGHTED AGAINST THE CAP, NOT THE MOTOR. The motor reaches 883 rad/s;
% the firmware cap is 40. Weighting against the motor makes K3 about 120x
% smaller and momentum management effectively vanishes - the classic failure
% where the cube balances for eight seconds and then falls over.
%
% WHY THE REDUCED-ORDER DESIGN IS NOT OPTIONAL
%   rank(ctrb) = 8 of 9. Angular momentum about the vertical is CONSERVED:
%   gB^T*Sg*P = 0 exactly, because P annihilates gB, so gravity exerts no
%   moment about the vertical - and the wheels are INTERNAL, so no input can
%   change it. Strict stabilizability needs every uncontrollable mode
%   strictly in the left half plane; ours sits at EXACTLY zero, so lqr() on
%   the full 9 states is ill-posed and returns numerical noise: asymmetric
%   wheel gains and Ms = 1.000006 instead of exactly 1.
%   The conserved direction satisfies BOTH v^T A = 0 and v^T B = 0, so
%   restricting to its orthogonal complement is EXACT, not an approximation.
%
% WHY THE YAW PROJECTION IS ALSO NOT OPTIONAL
%   rank(obsv) = 8 of 9 with the real sensor set. One accelerometer measures
%   ONE vector, and one vector determines 2 rotational DOF, not 3, so the yaw
%   ANGLE is unobservable. The unprojected K puts a finite gain on a state
%   the estimator cannot supply.
%
%   Yaw is out of reach of the actuators AND out of sight of the sensors.
%   The two deficiencies are consistent - it is not part of the problem.

p = cubli_nemesis_params;

if nargin < 2 || isempty(qa), qa = 0.20;  end
if nargin < 3 || isempty(qr), qr = 2.0;   end
if nargin < 4 || isempty(qw), qw = 10;    end
if nargin < 5 || isempty(rt), rt = 0.24;  end
if nargin < 6, verbose = true; end

I3 = eye(3);  Z = zeros(3);
Q  = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)]);
R  = (1/rt^2)*eye(3);

% ---- exact 8-state reduction on the controllable subspace --------------
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];      % conserved functional
eA = max(abs(vcon.'*p.A));  eB = max(abs(vcon.'*p.B));
assert(eA < 1e-9 && eB < 1e-9, ...
    'conserved direction not exact: |vA| = %.2e, |vB| = %.2e', eA, eB);

V2 = null(vcon.');                                 % 9x8, orthonormal
K  = lqr(V2.'*p.A*V2, V2.'*p.B, V2.'*Q*V2, R) * V2.';

% ---- project out the unobservable yaw angle ---------------------------
Kp = K * blkdiag(p.P, I3, I3);

% ---- checks -----------------------------------------------------------
info.qa = qa; info.qr = qr; info.qw = qw; info.rt = rt;
info.gain_conserved = norm(K *(vcon/norm(vcon)));
info.gain_yaw       = norm(Kp*[p.gB; zeros(6,1)]);
info.rank_ctrb = rank(ctrb(p.A,p.B));     % relative tol - NEVER pass 1e-9
info.rank_obsv = rank(obsv(p.A,[p.P Z Z; Z I3 Z; Z Z I3]));

w = logspace(-2,4,4000); Ms = 0;
for i = 1:numel(w)
    Ms = max(Ms, max(svd(inv(eye(3) + K*((1j*w(i)*eye(9)-p.A)\p.B)))));
end
info.Ms = Ms;

cl = sort(real(eig(p.A - p.B*Kp)));
info.n_marginal = sum(cl > -1e-6);
info.max_re     = max(cl(cl < -1e-6));
info.tau_slow   = -1000/info.max_re;
info.K1 = mean(abs(diag(Kp(:,1:3))));
info.K2 = mean(abs(diag(Kp(:,4:6))));
info.K3 = mean(abs(diag(Kp(:,7:9))));
info.stand_w = p.tau_cw/info.K3;

% ---- robustness, +/-30 %% simultaneously on Theta, Is, Sg --------------
worst = -inf;
for fT = [0.7 1 1.3]
  for fI = [0.7 1 1.3]
    for fS = [0.7 1 1.3]
      Th2 = p.Theta*fT;  Is2 = p.Is*fI;  Sg2 = p.Sg*fS;
      Tb2 = Th2 - Is2*eye(3);  G2 = Tb2\(Sg2*p.P);
      A2 = [Z I3 Z; G2 Z Z; -G2 Z Z];
      B2 = [Z; -inv(Tb2); inv(Is2*eye(3)) + inv(Tb2)];
      re = sort(real(eig(A2 - B2*Kp)));
      worst = max(worst, max(re(re < -1e-6)));
    end
  end
end
info.robust_worst = worst;

% ---- discrete check at the loop rate ----------------------------------
Ts = 1/p.f_outer;
sysd = c2d(ss(p.A,p.B,eye(9),zeros(9,3)), Ts, 'zoh');
[Ad,Bd] = ssdata(sysd);
Aa = [Ad Bd; zeros(3,9) zeros(3)];
Ba = [zeros(9,3); eye(3)];
zz = sort(abs(eig(Aa - Ba*[Kp zeros(3)])));
info.max_z = zz(end-info.n_marginal);

if verbose
    fprintf('\n=== NEMESIS LQR   corner (%+d,%+d,%+d)   qa %.2f  qr %.1f  qw %.0f  rt %.2f ===\n', ...
            2*(p.corner.'>0)-1, qa, qr, qw, rt);
    fprintf('  plant     ell %.2f mm   Sg %.4f   lambda %.4f, %.4f rad/s\n', ...
            p.ell*1e3, p.Sg, p.lambda(1), p.lambda(2));
    fprintf('  rank ctrb %d of 9   (yaw MOMENTUM conserved)\n', info.rank_ctrb);
    fprintf('  rank obsv %d of 9   (yaw ANGLE unobservable)\n', info.rank_obsv);
    fprintf('  gain on conserved direction  %.2e\n', info.gain_conserved);
    fprintf('  gain on yaw angle, projected %.2e\n', info.gain_yaw);
    fprintf('  Ms = %.6f    theory says EXACTLY 1. Any deviation is a bug.\n', info.Ms);
    fprintf('  closed loop  %d marginal (expect 2), slowest controlled %.0f ms\n', ...
            info.n_marginal, info.tau_slow);
    fprintf('  robust +/-30%%  worst %+.4f -> %s\n', info.robust_worst, string(info.robust_worst<0));
    fprintf('  discrete at %d Hz  max|z| = %.4f\n', p.f_outer, info.max_z);
    fprintf('  |K1| %.4f  |K2| %.4f  |K3| %.6f\n', info.K1, info.K2, info.K3);
    fprintf('  standing wheel speed %.1f rad/s at tau_cw = %.0f mN m  (%.0f%% of cap)\n', ...
            info.stand_w, p.tau_cw*1e3, 100*info.stand_w/p.omega_cap);
    if abs(info.Ms-1) > 1e-4
        warning('Ms = %.6f. Check A, B, Q, R and the reduction.', info.Ms);
    end
    if info.n_marginal ~= 2
        warning('%d marginal modes, expected 2.', info.n_marginal);
    end
    if info.robust_worst >= 0
        warning('robustness grid FAILED.');
    end
    fprintf('\n  Kp (3x9)  ***USE THIS IN FIRMWARE***\n'); disp(Kp);
end
end

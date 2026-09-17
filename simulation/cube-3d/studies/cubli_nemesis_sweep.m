function S = cubli_nemesis_sweep(p, mode)
%CUBLI_NEMESIS_SWEEP  Gain fine-tuning sweeps for the NEMESIS cube.
%
%   S = cubli_nemesis_sweep(p)            all sweeps
%   S = cubli_nemesis_sweep(p, 'rt')      torque budget only
%   S = cubli_nemesis_sweep(p, 'qw')      wheel budget only
%   S = cubli_nemesis_sweep(p, 'grid')    rt x qw grid, picks a winner
%
% THE METRIC THAT MATTERS IS PEAK TORQUE, NOT ENVELOPE.
% A 2 deg recovery at rt = 0.24 demands 719 %% of the clamp. Gains that ask
% for seven times the available torque saturate on noise alone and the
% controller spends its authority chattering instead of balancing. That is
% exactly what put the other cube at 71.8 %% saturation duty.


if nargin < 2, mode = 'all'; end
S = struct();

switch mode
case {'rt','all'}
    fprintf('\n=== TORQUE BUDGET rt   (R = 1/rt^2) ===\n');
    fprintf('%8s %9s %9s %10s %11s %10s %9s %9s\n', ...
        'rt','|K1|','|K2|','K2/K1','peak |u|','%% clamp','env deg','Ms');
    rts = [0.008 0.010 0.012 0.015 0.020 0.030 0.060 0.240];
    S.rt = zeros(numel(rts),8);
    for i = 1:numel(rts)
        S.rt(i,:) = one(p, 0.20, 2.0, 10, rts(i));
        pr(rts(i), S.rt(i,:));
    end
    fprintf('\n  PICK the largest rt whose peak |u| stays under ~120 %% of clamp.\n');
end

switch mode
case {'qw','all'}
    fprintf('\n=== WHEEL BUDGET qw   (Q33 = 1/qw^2) ===\n');
    fprintf('  smaller qw -> stronger momentum management -> more robust,\n');
    fprintf('  but more torque. Envelope barely moves: it is momentum limited.\n');
    fprintf('%8s %9s %9s %10s %11s %10s %9s %9s\n', ...
        'qw','|K1|','|K2|','K2/K1','peak |u|','%% clamp','env deg','stand w');
    qws = [4 6 8 10 15 20 30];
    S.qw = zeros(numel(qws),8);
    for i = 1:numel(qws)
        r = one(p, 0.20, 2.0, qws(i), 0.015);
        S.qw(i,:) = r;
        fprintf('%8.0f %9.3f %9.3f %10.4f %11.4f %9.0f%% %9.2f %9.1f\n', ...
            qws(i), r(1), r(2), r(3), r(4), 100*r(4)/p.tau_cont, r(6), p.tau_cw/r(8));
    end
end

switch mode
case {'grid','all'}
    fprintf('\n=== rt x qw GRID   (peak |u| as %% of clamp; * = feasible) ===\n');
    rts = [0.008 0.012 0.015 0.020 0.030];
    qws = [5 8 10 15 20];
    fprintf('%8s','rt \\ qw'); fprintf('%10.0f', qws); fprintf('\n');
    best = struct('score',-inf);
    S.grid = nan(numel(rts),numel(qws));
    for i = 1:numel(rts)
        fprintf('%8.2f', rts(i));
        for j = 1:numel(qws)
            r = one(p, 0.20, 2.0, qws(j), rts(i));
            pc = 100*r(4)/p.tau_cont;
            S.grid(i,j) = pc;
            ok = pc < 120;
            fprintf('%9.0f%s', pc, char(42*ok + 32*~ok));
            if ok
                score = r(6) - 0.01*abs(pc-100);   % envelope, penalise headroom loss
                if score > best.score
                    best = struct('score',score,'rt',rts(i),'qw',qws(j), ...
                                  'env',r(6),'pc',pc,'Ms',r(7),'rob',r(5));
                end
            end
        end
        fprintf('\n');
    end
    if isfinite(best.score)
        fprintf('\n  RECOMMENDED  rt = %.2f, qw = %.0f\n', best.rt, best.qw);
        fprintf('    peak |u| %.0f%% of clamp,  envelope %.2f deg,  Ms %.6f,  robust %+.3f\n', ...
            best.pc, best.env, best.Ms, best.rob);
        S.best = best;
    else
        warning('no feasible cell - lower rt further');
    end
end
end

% -----------------------------------------------------------------------
function r = one(p, qa, qr, qw, rt)
%ONE  design + evaluate one weight set. Returns
%   [K1 K2 K2/K1 peak_u robust_worst envelope Ms K3]
I3 = eye(3);  Z = zeros(3);
Q = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)]);
R = (1/rt^2)*eye(3);
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB];
V2 = null(vcon.');
K  = lqr(V2.'*p.A*V2, V2.'*p.B, V2.'*Q*V2, R)*V2.';
Kp = K*blkdiag(p.P,I3,I3);

K1 = mean(abs(diag(Kp(:,1:3))));
K2 = mean(abs(diag(Kp(:,4:6))));
K3 = mean(abs(diag(Kp(:,7:9))));

w = logspace(-2,4,1200); Ms = 0;
for i = 1:numel(w)
    Ms = max(Ms, max(svd(inv(eye(3) + K*((1j*w(i)*eye(9)-p.A)\p.B)))));
end

worst = -inf;
for fT = [0.7 1 1.3]
  for fI = [0.7 1 1.3]
    for fS = [0.7 1 1.3]
      Tb2 = p.Theta*fT - p.Is*fI*eye(3);  G2 = Tb2\(p.Sg*fS*p.P);
      A2 = [Z I3 Z; G2 Z Z; -G2 Z Z];
      B2 = [Z; -inv(Tb2); inv(p.Is*fI*eye(3)) + inv(Tb2)];
      re = sort(real(eig(A2 - B2*Kp)));
      worst = max(worst, max(re(re < -1e-6)));
    end
  end
end

pu  = peaku(p, Kp, deg2rad(2));
env = envel(p, Kp);
r = [K1 K2 K2/K1 pu worst env Ms K3];
end

% -----------------------------------------------------------------------
function pu = peaku(p, Kp, th0)
Ts = 1/p.f_outer;  axn = null(p.gB.');
x = [th0*axn(:,1); zeros(6,1)];  pu = 0;
for k = 1:round(4/Ts)
    u = -Kp*x;  pu = max(pu, max(abs(u)));
    s = min(max((p.omega_cap-abs(x(7:9)))/(0.1*p.omega_cap),0),1);
    sm = sign(u)==sign(x(7:9));  u(sm) = u(sm).*s(sm);
    u = max(min(u,p.tau_cont),-p.tau_cont);
    x = rk4(p, x, u, Ts);
    if ~all(isfinite(x)), break; end
end
end

% -----------------------------------------------------------------------
function e = envel(p, Kp)
Ts = 1/p.f_outer;  axn = null(p.gB.');
lo = deg2rad(0.5);  hi = deg2rad(15);
for it = 1:11
    mid = 0.5*(lo+hi);
    x = [mid*axn(:,1); zeros(6,1)];  ok = true;
    for k = 1:round(5/Ts)
        u = -Kp*x;
        s = min(max((p.omega_cap-abs(x(7:9)))/(0.1*p.omega_cap),0),1);
        sm = sign(u)==sign(x(7:9));  u(sm) = u(sm).*s(sm);
        u = max(min(u,p.tau_cont),-p.tau_cont);
        x = rk4(p, x, u, Ts);
        if ~all(isfinite(x)) || norm(p.P*x(1:3)) > deg2rad(40), ok = false; break; end
    end
    ok = ok && norm(p.P*x(1:3)) < deg2rad(1);
    if ok, lo = mid; else, hi = mid; end
end
e = rad2deg(lo);
end

% -----------------------------------------------------------------------
function x = rk4(p, x, u, Ts)
g = @(z)[z(4:6); ...
         p.Tb\(p.Sg*p.P*z(1:3) - u); ...
         (p.Is*eye(3))\u - p.Tb\(p.Sg*p.P*z(1:3) - u)];
k1=g(x); k2=g(x+Ts/2*k1); k3=g(x+Ts/2*k2); k4=g(x+Ts*k3);
x = x + Ts/6*(k1+2*k2+2*k3+k4);
end

% -----------------------------------------------------------------------
function pr(rt, r)
fprintf('%8.2f %9.3f %9.3f %10.4f %11.4f %9.0f%% %9.2f %9.6f\n', ...
        rt, r(1), r(2), r(3), r(4), 100*r(4)/0.12, r(6), r(7));
end

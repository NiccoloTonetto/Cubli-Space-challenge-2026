function C = cubli_nemesis_allcorners(p, qa, qr, qw, rt, verbose)
%CUBLI_NEMESIS_ALLCORNERS  Plant and gains for all eight corners.
%
%   C = cubli_nemesis_allcorners(p)
%   C = cubli_nemesis_allcorners(p, 0.20, 2.0, 4, 0.012)
%
% Returns a 1x8 struct array, one entry per corner:
%   sign, pos, ell, gB, Sg, Theta, Tb, P, A, B, lambda,
%   mount_axis, mount_angle, theta_eq, Kp, K, Ms, tau_slow, env
%
% ONLY CORNER 1 (-1,-1,-1) IS FLOWN. The other seven are computed and stored
% for completeness - multi-corner balancing and locomotion would need them,
% and having them costs nothing because they all follow from the SAME COM.
%
% NO CALIBRATION IS NEEDED PER CORNER. Each target gB is
%     gB_k = -(com - corner_k) / |com - corner_k|
% and corner detection at run time is nearest-match on the measured gravity
% direction: whichever stored gB is closest is the corner you are standing on.
%
% *** CORNER-ORIGIN GEOMETRY: corners are combinations of 0 and a, ***
% *** NOT +/- a/2. Getting this wrong gives 70 deg equilibrium tilts.  ***

if nargin < 2 || isempty(qa), qa = 0.20;   end
if nargin < 3 || isempty(qr), qr = 2.0;    end
if nargin < 4 || isempty(qw), qw = 4;      end
if nargin < 5 || isempty(rt), rt = 0.012;  end
if nargin < 6, verbose = true; end

I3 = eye(3);  Z = zeros(3);
Q  = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)]);
R  = (1/rt^2)*eye(3);

bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for k = 1:3
    bodies{end+1} = {p.m_wheel, p.com_wheel{k}, p.I_wheel{k}};   %#ok<AGROW>
end
ctr = [0.5;0.5;0.5]*p.a;

n = 0;
for sx = [0 1], for sy = [0 1], for sz = [0 1]
    n = n + 1;
    cpos = [sx; sy; sz]*p.a;
    C(n).sign = 2*[sx sy sz]-1;   %#ok<AGROW>
    C(n).pos  = cpos;             %#ok<AGROW>

    % ---- inertia about THIS corner ----
    Th = zeros(3);
    for i = 1:numel(bodies)
        d = bodies{i}{2} - cpos;
        Th = Th + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
    end
    v   = p.com - cpos;
    ell = norm(v);
    gB  = -v/ell;
    Sg  = p.m_total*p.g*ell;
    Tb  = Th - p.Is*eye(3);
    P   = eye(3) - gB*gB.';
    G   = Tb\(Sg*P);
    A   = [Z I3 Z; G Z Z; -G Z Z];
    B   = [Z; -inv(Tb); inv(p.Is*eye(3)) + inv(Tb)];

    ev  = eig(A);
    C(n).ell = ell;  C(n).gB = gB;  C(n).Sg = Sg;   %#ok<AGROW>
    C(n).Theta = Th; C(n).Tb = Tb;  C(n).P = P;     %#ok<AGROW>
    C(n).A = A;      C(n).B = B;                    %#ok<AGROW>
    C(n).lambda = sort(real(ev(real(ev) > 1e-6)),'descend');   %#ok<AGROW>

    % ---- equilibrium tilt: COM offset perpendicular to this diagonal ----
    u  = (cpos-ctr)/norm(cpos-ctr);
    da = dot(p.com-ctr, u);
    C(n).theta_eq = atan(norm((p.com-ctr) - da*u)/(sqrt(3)/2*p.a - da));   %#ok<AGROW>

    % ---- mount rotation, world Y up ----
    tgt = [0;-1;0];
    ax  = cross(gB,tgt);  ax = ax/norm(ax);
    C(n).mount_axis  = ax;                                        %#ok<AGROW>
    C(n).mount_angle = acos(max(-1,min(1,dot(gB,tgt))));          %#ok<AGROW>

    % ---- reduced-order LQR + yaw projection ----
    vcon = [zeros(3,1); Th*gB; p.Is*gB];
    assert(max(abs(vcon.'*A)) < 1e-9 && max(abs(vcon.'*B)) < 1e-9, ...
           'corner %d: conserved direction not exact', n);
    V2 = null(vcon.');
    K  = lqr(V2.'*A*V2, V2.'*B, V2.'*Q*V2, R)*V2.';
    Kp = K*blkdiag(P,I3,I3);
    C(n).K = K;  C(n).Kp = Kp;                                    %#ok<AGROW>

    w = logspace(-2,4,1500);  Ms = 0;
    for i = 1:numel(w)
        Ms = max(Ms, max(svd(inv(eye(3) + K*((1j*w(i)*eye(9)-A)\B)))));
    end
    C(n).Ms = Ms;                                                 %#ok<AGROW>
    cl = sort(real(eig(A - B*Kp)));
    C(n).n_marginal = sum(cl > -1e-6);                            %#ok<AGROW>
    C(n).tau_slow   = -1000/max(cl(cl < -1e-6));                  %#ok<AGROW>
    C(n).K3 = mean(abs(diag(Kp(:,7:9))));                         %#ok<AGROW>

    % ---- nonlinear recoverable tilt ----
    C(n).env = envel(p, A, B, P, Kp);                             %#ok<AGROW>
end, end, end

if verbose
    fprintf('\n=== ALL EIGHT CORNERS   qa %.2f  qr %.1f  qw %.0f  rt %.3f ===\n', qa,qr,qw,rt);
    fprintf('%-12s %8s %8s %9s %8s %9s %9s %8s %8s\n', ...
        'corner','ell mm','th_eq','lambda','Ms','tau ms','env deg','|K3|','flown');
    for n = 1:8
        fl = isequal(C(n).sign, [-1 -1 -1]);
        fprintf('(%+d,%+d,%+d)%4s %8.2f %7.3f %9.4f %8.6f %9.0f %9.2f %8.5f %8s\n', ...
            C(n).sign, '', C(n).ell*1e3, rad2deg(C(n).theta_eq), C(n).lambda(1), ...
            C(n).Ms, C(n).tau_slow, C(n).env, C(n).K3, char(42*fl + 32*~fl));
    end
    fprintf('\n  * = the corner actually flown.\n');
    [~,best] = sort([C.theta_eq]);
    fprintf('  smallest equilibrium tilt: (%+d,%+d,%+d) at %.3f deg, then (%+d,%+d,%+d) at %.3f deg\n', ...
        C(best(1)).sign, rad2deg(C(best(1)).theta_eq), C(best(2)).sign, rad2deg(C(best(2)).theta_eq));
    fprintf('  the other six sit at %.2f-%.2f deg because the COM offset is almost\n', ...
        rad2deg(C(best(3)).theta_eq), rad2deg(C(best(8)).theta_eq));
    fprintf('  entirely ALONG one body diagonal, and two diagonals meet at 70.5 deg.\n');
    fprintf('\n  target gB per corner (firmware lookup, nearest-match detection):\n');
    for n = 1:8
        fprintf('    (%+d,%+d,%+d)  [%+.6f %+.6f %+.6f]\n', C(n).sign, C(n).gB);
    end
end
end

% -----------------------------------------------------------------------
function e = envel(p, A, B, P, Kp)
Ts = 1/p.f_outer;  axn = null(P(1,:)*0 + [1 1 1]);  %#ok<NASGU>
axn = null(( -A(4:6,1:3)*0 + eye(3) )*0 + 1);       %#ok<NASGU>
[V,~] = eig(P);  [~,ix] = sort(diag(V.'*P*V),'descend');
axn = V(:,ix(1:2));
lo = deg2rad(0.5);  hi = deg2rad(12);
for it = 1:10
    mid = 0.5*(lo+hi);
    x = [mid*axn(:,1); zeros(6,1)];  ok = true;
    for k = 1:round(5/Ts)
        u = -Kp*x;
        s = min(max((p.omega_cap-abs(x(7:9)))/(0.1*p.omega_cap),0),1);
        sm = sign(u)==sign(x(7:9));  u(sm) = u(sm).*s(sm);
        u = max(min(u,p.tau_cont),-p.tau_cont);
        f = @(z) A*z + B*u;
        k1=f(x); k2=f(x+Ts/2*k1); k3=f(x+Ts/2*k2); k4=f(x+Ts*k3);
        x = x + Ts/6*(k1+2*k2+2*k3+k4);
        if ~all(isfinite(x)) || norm(P*x(1:3)) > deg2rad(40), ok = false; break; end
    end
    ok = ok && norm(P*x(1:3)) < deg2rad(1);
    if ok, lo = mid; else, hi = mid; end
end
e = rad2deg(lo);
end

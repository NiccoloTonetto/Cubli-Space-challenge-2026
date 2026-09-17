function out = cubli_nlsim(p, c, Kp, phi0, o)
%CUBLI_NLSIM  Full nonlinear corner balance. No small-angle assumption.
%
% PLANT  x = [gam(3); om(3); rho(3)]
%   gam  gravity direction in BODY coordinates (gam == c.gB is upright)
%   om   body angular velocity about the contact point
%   rho  wheel rates RELATIVE to the body (encoder reading)
%   L      = Theta*om + Is*rho
%   Tb*omd = m*g*(r_c x gam) - u_eff - om x L - tau_pivot
%   Is*rhod= u_eff - Is*omd,     gamd = -om x gam
%
% SENSING  the accelerometer measures SPECIFIC FORCE at r_imu, not gravity:
%   f = omd x r + om x (om x r) - g*gam
% The lever-arm terms are in-band and correlated with the control action, so
% no amount of low-pass filtering removes them.
%
% ESTIMATOR  o.est
%   'raw'     feed the normalised accelerometer straight to the gain
%   'mahony'  reduced-attitude complementary filter on the 2-DoF gravity
%             direction with gyro-bias states. Singularity-free, no
%             quaternion, no unobservable-yaw problem.
%   'mahony+' the same, with algebraic lever-arm compensation
%
% o fields: tmax 5 | fs 400 | h 6.25e-4 | tau_max p.tau_cont
%   omega_cap p.omega_cap | V_bus 0 | R_phase 0.10 | friction 0 | ff 0
%   tau_cp 0 | delay 0 | enc_lsb 0 | gyro_sd 0 | acc_sd 0 | gyro_bias [0;0;0]
%   r_imu [0;0;0] (from the CONTACT CORNER, body coords) | est 'raw'
%   om0 [0;0;0] initial body rate - use for IMPULSE (push) tests:
%     a force F for dt at the TOP corner gives J = (r_top x F)*dt, r_top = -2*corner,
%     and with the wheels at rest om0 = Theta\J.
%   kP 2.0 | kI 0.5 | seed []

d = struct('tmax',5,'fs',400,'h',6.25e-4,'tau_max',p.tau_cont, ...
    'omega_cap',p.omega_cap,'friction',0,'ff',0,'tau_cp',0,'delay',0, ...
    'enc_lsb',0,'gyro_sd',0,'acc_sd',0,'seed',[],'V_bus',0,'R_phase',0.10, ...
    'gyro_bias',[0;0;0],'r_imu',[0;0;0],'est','raw','kP',2.0,'kI',0.5,'imu_mis',[0;0;0],'om0',[0;0;0]);
if nargin < 5 || isempty(o), o = struct; end
fn = fieldnames(d);
for i = 1:numel(fn), if ~isfield(o,fn{i}), o.(fn{i}) = d.(fn{i}); end, end
if ~isempty(o.seed), rng(o.seed); end
lev = any(o.r_imu ~= 0);

nsub = max(1, round(1/(o.fs*o.h)));  o.h = 1/(o.fs*nsub);  dt = 1/o.fs;
N    = round(o.tmax*o.fs);
x    = [expm(-skew(phi0))*c.gB; o.om0(:); zeros(3,1)];
ubuf = zeros(3, o.delay+1);  ua = zeros(3,1);
ghat = c.gB;  bhat = zeros(3,1);            % estimator state

t=zeros(N+1,1); X=zeros(N+1,9); U=zeros(N+1,3); E=zeros(N+1,1); Lv=zeros(N+1,1);
satT=false(N+1,1); satW=false(N+1,1);  X(1,:)=x.';
for n = 1:N
    gm = x(1:3)/norm(x(1:3));  om = x(4:6);  rho = x(7:9);

    % ---- accelerometer: specific force, then normalise
    dxn = deriv(x, ua, p, c, o);  omd = dxn(4:6);
    if lev
        f = p.g*gm - cross(omd,o.r_imu) - cross(om,cross(om,o.r_imu));
    else
        f = p.g*gm;
    end
    gacc = f/norm(f);
    if any(o.imu_mis)     % IMU frame rotated w.r.t. the cube body frame
        Rm = expm(skew(o.imu_mis));  gacc = Rm.'*gacc;
    end
    Lv(n) = real(acos(max(-1,min(1, gacc.'*gm))));   % lever-arm tilt error
    if o.acc_sd > 0, gacc = gacc + o.acc_sd*randn(3,1); gacc = gacc/norm(gacc); end
    omm = om + o.gyro_bias;
    if any(o.imu_mis), omm = expm(skew(o.imu_mis)).'*omm; end
    if o.gyro_sd > 0, omm = omm + o.gyro_sd*randn(3,1); end
    rhm = rho;
    if o.enc_lsb > 0, rhm = round(rhm/o.enc_lsb)*o.enc_lsb; end

    % ---- estimator
    switch o.est
      case 'raw'
        gest = gacc;  omest = omm;
      otherwise
        ga = gacc;
        if lev && any(strcmp(o.est,{'mahony+','mahony_u','mahony_or'}))
            omh = omm - bhat;
            switch o.est
              case 'mahony_or'                % ORACLE: true omd/om, NOT implementable
                omh = om;  omd_h = omd;
              case 'mahony_u'                 % SAFE: only the KNOWN torque + measured rate
                Lh    = c.Theta*omh + p.Is*rhm;
                omd_h = c.Tb\(-ua - cross(omh,Lh));
              otherwise                       % mahony+ : also uses tau_g(ghat) -> feedback
                Lh    = c.Theta*omh + p.Is*rhm;
                omd_h = c.Tb\(p.m_total*p.g*cross(c.r_c,ghat) - ua - cross(omh,Lh));
            end
            fc = p.g*gacc + cross(omd_h,o.r_imu) + cross(omh,cross(omh,o.r_imu));
            ga = fc/norm(fc);                 % algebraic lever-arm removal
        end
        e    = cross(ghat, ga);
        wcor = (omm - bhat);
        ghat = ghat - cross(wcor, ghat)*dt + o.kP*cross(e, ghat)*dt;
        ghat = ghat/norm(ghat);
        bhat = bhat + o.kI*e*dt;
        gest = ghat;  omest = omm - bhat;
    end
    E(n) = real(acos(max(-1,min(1, gest.'*gm))));    % total estimate error

    % ---- control
    phi = -cross(c.gB, gest);
    u   = -Kp*[phi; omest; rhm];
    if o.ff, u = u + p.tau_cw*tanh(rhm/p.eps_ff) + p.b_w*rhm; end
    tlim = o.tau_max*ones(3,1);
    if o.V_bus > 0
        tlim = min(tlim, p.Kt*max(0, o.V_bus - p.Kt*abs(rho))/o.R_phase);
    end
    satT(n) = any(abs(u) > tlim);
    u = max(min(u, tlim), -tlim);
    for k = 1:3
        if rho(k) >=  o.omega_cap && u(k) > 0, u(k) = 0; satW(n) = true; end
        if rho(k) <= -o.omega_cap && u(k) < 0, u(k) = 0; satW(n) = true; end
    end
    ubuf = [ubuf(:,2:end) u];  ua = ubuf(:,1);

    for s = 1:nsub
        k1 = deriv(x,          ua,p,c,o);  k2 = deriv(x+o.h/2*k1,ua,p,c,o);
        k3 = deriv(x+o.h/2*k2, ua,p,c,o);  k4 = deriv(x+o.h*k3,  ua,p,c,o);
        x  = x + o.h/6*(k1+2*k2+2*k3+k4);  x(1:3) = x(1:3)/norm(x(1:3));
    end
    t(n+1)=n/o.fs; X(n+1,:)=x.'; U(n,:)=u.';
end
U(end,:)=U(end-1,:); E(end)=E(end-1); Lv(end)=Lv(end-1);

tilt = acos(max(-1,min(1, X(:,1:3)*c.gB)));
out = struct('t',t,'X',X,'U',U,'tilt',tilt,'est_err',E,'lever_err',Lv,'opt',o);
out.tilt_max=max(tilt); out.tilt_final=max(tilt(t>=o.tmax-1));
out.wheel_max=max(abs(X(:,7:9)),[],'all'); out.tau_max_used=max(abs(U),[],'all');
out.sat_torque=mean(satT); out.sat_wheel=mean(satW);
out.bias_hat=bhat;
out.recovered = out.tilt_final < deg2rad(1) && out.tilt_max < deg2rad(60);
end

function dx = deriv(x, u, p, c, o)
gam = x(1:3);  om = x(4:6);  rho = x(7:9);
ue = u;
if o.friction, ue = ue - (p.tau_cw*tanh(rho/p.eps_ff) + p.b_w*rho); end
tg = p.m_total*p.g*cross(c.r_c, gam);
if o.tau_cp > 0, tg = tg - o.tau_cp*tanh(om/0.01); end
L   = c.Theta*om + p.Is*rho;
omd = c.Tb\(tg - ue - cross(om,L));
dx  = [-cross(om,gam); omd; ue/p.Is - omd];
end

function S = skew(v)
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

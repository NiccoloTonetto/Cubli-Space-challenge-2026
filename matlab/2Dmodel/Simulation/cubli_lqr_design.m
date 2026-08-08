%% ========================================================================
%  CUBLI 1D — LQR DESIGN AND VALIDATION
%  ETH-convention state:  x = [theta ; theta_dot ; phi_dot]
%     theta     body angle from vertical            [rad]
%     theta_dot body angular rate                   [rad/s]
%     phi_dot   wheel speed RELATIVE to body        [rad/s]   (what the encoder reads)
%  Input:  u = motor torque applied to the wheel    [N*m]
%
%  WORKFLOW: replace the PARAMETERS block with SolidWorks values, re-run.
%  Every check below is either a model validation or a deployability gate.
% ========================================================================
clear; clc; close all;

%% ===== 1. PARAMETERS   ==================================================
p.g       = 9.81;        % gravity                              [m/s^2]
p.S       = 0.0272;      % first moment m*l about pivot         [kg*m]    <-- CAD
p.Theta   = 3.0162e-3;   % total inertia about pivot, wheel locked [kg m^2] <-- CAD
p.Iw      = 3.0e-4;      % wheel spin inertia                   [kg m^2]  <-- CAD
p.tau_max = 0.40;        % peak motor torque                    [N*m]
p.tau_cont= 0.12;        % continuous motor torque              [N*m]
p.w_max   = 883;         % max wheel speed (6S)                 [rad/s]
p.Ts      = 1e-3;        % control period                       [s]
p.tau_dly = 0.8e-3;      % total loop delay budget (ZOH+compute+CAN) [s]
p.tau_e   = 200e-6;      % motor electrical time constant L/R   [s]  <-- MEASURE R, L

p.Tbar = p.Theta - p.Iw;               % effective body inertia
assert(p.Tbar > 0, 'Theta must exceed Iw');

%% ===== 2. PLANT =========================================================
aa = p.S*p.g/p.Tbar;
A = [ 0    1  0 ;
      aa   0  0 ;
     -aa   0  0 ];
B = [ 0 ;
     -1/p.Tbar ;
      1/p.Iw + 1/p.Tbar ];
n = size(A,1);

lam = sqrt(aa);                        % unstable pole
fprintf('===== PLANT =====\n');
fprintf('Tbar = %.4e kg m^2 | lambda = %.3f rad/s | tau_c = %.0f ms\n', ...
        p.Tbar, lam, 1000/lam);

%% --- CHECK 1: open-loop poles match theory (MODEL VALIDATION) ----------
ol = eig(A);
fprintf('\n[1] OPEN-LOOP POLES\n');
fprintf('    computed : '); fprintf('%+8.4f ', sort(ol)); fprintf('\n');
fprintf('    expected : %+8.4f %+8.4f %+8.4f\n', -lam, 0, lam);
ok1 = norm(sort(ol) - sort([-lam;0;lam])) < 1e-9;
fprintf('    --> %s\n', ternary(ok1,'PASS','FAIL - A matrix is wrong'));

%% --- CHECK 2: controllability + conditioning ---------------------------
Co = ctrb(A,B); sv = svd(Co);
fprintf('\n[2] CONTROLLABILITY\n');
fprintf('    rank %d/%d | singular values %.2e %.2e %.2e | cond %.1e\n', ...
        rank(Co), n, sv, cond(Co));
fprintf('    --> %s (cond < 1e6 is healthy)\n', ...
        ternary(rank(Co)==n && cond(Co)<1e6,'PASS','CHECK'));

%% ===== 3. BRYSON-RULE WEIGHTS ==========================================
%  Q_ii = 1/(max acceptable x_i)^2 ,  R = 1/(max acceptable u)^2
x_max = [ deg2rad(5) ;   % max tilt we tolerate            [rad]
          1.0        ;   % max body rate                   [rad/s]
          0.5*p.w_max];  % max wheel speed we want to use  [rad/s]
u_max = p.tau_max;

% rho scales the control weight ABOVE pure Bryson.
%   rho = 1  -> pure Bryson: aggressive, crossover ~16x lambda, SATURATES
%   rho = 8..16 -> crossover ~5-6x lambda, peak torque well inside limit
% Sweep it in CHECK 9 and pick from the table for your actual parameters.
rho = 12;

Q = diag(1./x_max.^2);
R = rho/u_max^2;

fprintf('\n===== BRYSON WEIGHTS =====\n');
fprintf('x_max = [%.4f rad (%.1f deg), %.2f rad/s, %.0f rad/s]\n', ...
        x_max(1), rad2deg(x_max(1)), x_max(2), x_max(3));
fprintf('u_max = %.2f N*m\n', u_max);

K = lqr(A,B,Q,R);
Acl = A - B*K;
cl  = eig(Acl);
fprintf('K = [%+.4f  %+.4f  %+.6f]\n', K);
fprintf('closed-loop poles: '); fprintf('%+.3f ', cl); fprintf('\n');

%% --- CHECK 3: closed-loop stability + pole placement sanity ------------
fprintf('\n[3] CLOSED-LOOP POLES\n');
[wn,zeta] = damp(Acl);
for i=1:n
    fprintf('    p%d = %+8.3f   wn = %7.2f rad/s   zeta = %.3f\n', ...
            i, real(cl(i)), wn(i), zeta(i));
end
fprintf('    --> %s\n', ternary(all(real(cl)<0),'PASS','FAIL'));
fprintf('    fastest pole %.1f rad/s = %.1f Hz\n', max(abs(real(cl))), max(abs(real(cl)))/2/pi);
fprintf('    NOTE: keep fastest pole well below any structural resonance\n');

%% ===== 4. LOOP TRANSFER, MARGINS, SENSITIVITY ==========================
L  = ss(A,B,K,0);              % loop broken at the plant INPUT
S_ = feedback(1,L);            % sensitivity
T_ = feedback(L,1);            % complementary sensitivity
[Gm,Pm,~,Wcp] = margin(L);
Ms = norm(S_,inf);  Tp = norm(T_,inf);
DM = deg2rad(Pm)/Wcp;          % delay margin [s]

fprintf('\n[4] MARGINS (state feedback, NO estimator)\n');
fprintf('    crossover  wc = %.1f rad/s = %.1f Hz   (~%.1fx lambda)\n', Wcp, Wcp/2/pi, Wcp/lam);
fprintf('    gain margin   = %.3f (%.1f dB)  <-- for UNSTABLE plant this is the\n', Gm, 20*log10(Gm));
fprintf('                                          DOWNWARD margin; small is GOOD\n');
fprintf('    phase margin  = %.1f deg    (LQR guarantees >= 60)\n', Pm);
fprintf('    Ms = %.4f   <-- MUST be 1.0000 for full-state LQR (return-difference\n', Ms);
fprintf('                     identity). Any deviation = implementation error.\n');
fprintf('    Tp = %.3f\n', Tp);
fprintf('    delay margin  = %.2f ms   vs budget %.2f ms  --> %s\n', ...
        1000*DM, 1000*p.tau_dly, ternary(DM > 3*p.tau_dly,'PASS (3x)','TIGHT'));

%% --- CHECK 5: Nyquist encirclement (the REAL test for unstable plants) -
P_unstable = sum(real(ol) > 0);
Z_unstable = sum(real(cl) > 0);
fprintf('\n[5] NYQUIST CRITERION\n');
fprintf('    open-loop unstable poles P = %d\n', P_unstable);
fprintf('    required encirclements of -1: N = -P = %d (counter-clockwise)\n', -P_unstable);
fprintf('    closed-loop unstable poles Z = %d\n', Z_unstable);
fprintf('    --> %s\n', ternary(Z_unstable==0,'PASS','FAIL'));
fprintf('    (for an unstable plant, PM alone is NOT a stability proof)\n');

%% --- CHECK 6: saturation under realistic recovery ----------------------
fprintf('\n[6] SATURATION (nonlinear sim, recovery from x_max tilt)\n');
sat = @(v) max(min(v, p.tau_max), -p.tau_max);
f = @(t,x) [ x(2);
             (p.S*p.g*sin(x(1)) - sat(-K*x))/p.Tbar;
            -(p.S*p.g*sin(x(1)) - sat(-K*x))/p.Tbar + sat(-K*x)/p.Iw ];
[tt,XX] = ode45(f, [0 10], [x_max(1); 0; 0]);
uu = -(K*XX')';
pk_u = max(abs(uu));  pk_w = max(abs(XX(:,3)));
idx  = find(abs(XX(:,1)) < deg2rad(0.5), 1);
fprintf('    peak |u|       = %.3f N*m   (limit %.2f)  --> %s\n', ...
        pk_u, p.tau_max, ternary(pk_u<=p.tau_max,'PASS','SATURATES'));
fprintf('    peak wheel     = %.0f rad/s  (limit %.0f)  --> %s\n', ...
        pk_w, p.w_max, ternary(pk_w<=p.w_max,'PASS','EXCEEDS'));
fprintf('    RMS |u|        = %.3f N*m   (cont limit %.2f)  --> %s\n', ...
        rms(uu), p.tau_cont, ternary(rms(uu)<=p.tau_cont,'PASS','THERMAL RISK'));
if ~isempty(idx)
    fprintf('    settling (<0.5 deg) = %.2f s\n', tt(idx));
end

%% --- CHECK 7: DISCRETE-TIME implementation ----------------------------
sysd = c2d(ss(A,B,eye(n),0), p.Ts, 'zoh');
Kd   = lqrd(A,B,Q,R,p.Ts);
cld  = eig(sysd.A - sysd.B*Kd);
fprintf('\n[7] DISCRETE-TIME (Ts = %.1f ms)\n', 1000*p.Ts);
fprintf('    Kd = [%+.4f  %+.4f  %+.6f]\n', Kd);
fprintf('    |z| = '); fprintf('%.4f ', abs(cld)); fprintf('\n');
fprintf('    --> %s\n', ternary(all(abs(cld)<1),'PASS (inside unit circle)','FAIL'));
fprintf('    continuous K vs discrete Kd differ by %.1f%%\n', 100*norm(Kd-K)/norm(K));

%% --- CHECK 8: PARAMETER ROBUSTNESS (the one that matters most) ---------
fprintf('\n[8] PARAMETER ROBUSTNESS — gains FIXED, plant perturbed\n');
fprintf('    (gains designed on estimates must survive CAD & measured values)\n');
fprintf('    %8s %8s %8s %12s %s\n','dTheta','dS','dIw','max Re(cl)','');
worst = -Inf;
for dT = [-0.3 0 0.3]
  for dS = [-0.3 0 0.3]
    for dI = [-0.2 0 0.2]
      Tb2 = p.Tbar*(1+dT); S2 = p.S*(1+dS); Iw2 = p.Iw*(1+dI);
      a2  = S2*p.g/Tb2;
      A2  = [0 1 0; a2 0 0; -a2 0 0];
      B2  = [0; -1/Tb2; 1/Iw2 + 1/Tb2];
      e   = max(real(eig(A2 - B2*K)));
      worst = max(worst,e);
      if abs(dT)==0.3 && abs(dS)==0.3 && abs(dI)==0.2   % print corners only
        fprintf('    %+7.0f%% %+7.0f%% %+7.0f%% %+12.4f %s\n', ...
                dT*100,dS*100,dI*100,e,ternary(e<0,'stable','UNSTABLE'));
      end
    end
  end
end
fprintf('    WORST CASE over all combinations: %+.4f --> %s\n', ...
        worst, ternary(worst<0,'PASS - robust to +/-30%','FAIL - too fragile'));

%% --- CHECK 9: SYMMETRIC ROOT LOCUS (the meaningful LQR "root locus") ---
fprintf('\n[9] SYMMETRIC ROOT LOCUS — sweep control weight rho\n');
fprintf('    %10s %12s %12s %12s\n','rho','fastest pole','wc [rad/s]','peak u [N*m]');
rhos = logspace(-2,2,5);
for r = rhos
    Kr = lqr(A,B,Q,R*r);
    er = eig(A-B*Kr);
    Lr = ss(A,B,Kr,0); [~,~,~,wr] = margin(Lr);
    [~,Xr] = ode45(@(t,x)[x(2);
        (p.S*p.g*sin(x(1))-sat(-Kr*x))/p.Tbar;
       -(p.S*p.g*sin(x(1))-sat(-Kr*x))/p.Tbar + sat(-Kr*x)/p.Iw], ...
        [0 5],[x_max(1);0;0]);
    fprintf('    %10.3f %12.1f %12.1f %12.3f\n', ...
            r, max(abs(real(er))), wr, max(abs(-(Kr*Xr')')));
end
fprintf('    larger rho -> cheaper control, slower loop, less torque demand\n');

%% --- CHECK 10: ACTUATOR LAG — is hdot = u actually legitimate? ---------
%  The model assumes commanded torque appears instantly. Reality: the
%  current loop has time constant tau_e = L/R. Test by adding a 4th state
%  (actual torque) with a first-order lag, and compare to the ideal.
fprintf('\n[10] ACTUATOR LAG (validates the hdot = u assumption)\n');
f_lag = @(t,x,te) [ x(2);
                    (p.S*p.g*sin(x(1)) - sat(x(4)))/p.Tbar;
                   -(p.S*p.g*sin(x(1)) - sat(x(4)))/p.Tbar + sat(x(4))/p.Iw;
                    (-K*x(1:3) - x(4))/te ];
tspan = linspace(0,3,600);
[~,X0lag] = ode45(@(t,x) f(t,x), tspan, [x_max(1);0;0]);   % ideal reference
fprintf('    %-12s %-12s %s\n','tau_e [us]','max dev [deg]','verdict');
for te = [p.tau_e, 5*p.tau_e, 25*p.tau_e]
    [~,Xl] = ode45(@(t,x) f_lag(t,x,te), tspan, [x_max(1);0;0;0]);
    dev = max(abs(rad2deg(Xl(:,1) - X0lag(:,1))));
    fprintf('    %-12.0f %-12.4f %s\n', te*1e6, dev, ...
            ternary(dev<0.5,'negligible','SIGNIFICANT - model hdot=u invalid'));
end
fprintf('    separation: body tau_c = %.0f ms vs tau_e = %.2f ms  -> %.0fx\n', ...
        1000/lam, 1000*p.tau_e, (1/lam)/p.tau_e);
fprintf('    --> hdot = u is legitimate when separation > ~50x\n');

%% --- CHECK 11: ACTUATION HEADROOM — which limit binds, and by how much -
fprintf('\n[11] ACTUATION HEADROOM\n');
tau_ratio = p.tau_max/(p.S*p.g);          % single wheel, planar
X_mom     = (p.Iw*p.w_max)^2/(2*p.Theta*p.S*p.g);
fprintf('    TORQUE   : tau_max/(S g) = %.3f\n', tau_ratio);
if tau_ratio >= 1
    fprintf('               --> >1: NO torque limit at any angle (over-actuated)\n');
else
    fprintf('               --> static hold limit %.1f deg\n', asind(tau_ratio));
end
fprintf('    MOMENTUM : X = h_max^2/(2 Theta S g) = %.2f\n', X_mom);
if X_mom >= 2
    fprintf('               --> >2: wheel absorbs a fall from ANY angle (over-actuated)\n');
else
    fprintf('               --> momentum limit %.1f deg\n', acosd(1-X_mom));
end
fprintf('    momentum used catching %.0f deg: %.1f%% of h_max\n', ...
        rad2deg(x_max(1)), 100*sqrt(2*p.Theta*p.S*p.g*(1-cos(x_max(1))))/(p.Iw*p.w_max));
if tau_ratio>=1 && X_mom>=2
    fprintf('    WARNING: this body is over-actuated in BOTH senses. It will NOT\n');
    fprintf('             exercise saturation behaviour. To test momentum management,\n');
    fprintf('             artificially cap w_max in firmware (e.g. 100 rad/s).\n');
end


%% ===== SUMMARY GATE ====================================================
fprintf('\n===== DEPLOYMENT GATES =====\n');
gates = { 'open-loop poles match theory', ok1
          'controllable & well-conditioned', rank(Co)==n && cond(Co)<1e6
          'closed-loop stable',              all(real(cl)<0)
          'Ms == 1.00 (LQR identity)',       abs(Ms-1)<1e-3
          'phase margin >= 60 deg',          Pm>=60
          'delay margin > 3x budget',        DM > 3*p.tau_dly
          'peak torque within limit',        pk_u <= p.tau_max
          'wheel speed within limit',        pk_w <= p.w_max
          'discrete poles inside unit circle',all(abs(cld)<1)
          'robust to +/-30% parameters',     worst<0 };
for i=1:size(gates,1)
    fprintf('  [%s] %s\n', ternary(gates{i,2},'x',' '), gates{i,1});
end
fprintf('\nALL GATES: %s\n', ternary(all([gates{:,2}]),'PASS','review failures above'));

%% ===== PLOTS ===========================================================
figure('Name','Cubli LQR validation','Position',[100 100 1100 700]);
subplot(2,3,1); pzmap(ss(Acl,B,eye(n),0)); title('closed-loop poles'); grid on;
subplot(2,3,2); nyquist(L); title('Nyquist (encirclement test)'); grid on;
subplot(2,3,3); bode(L); title('loop transfer L'); grid on;
subplot(2,3,4); bodemag(S_,T_); legend('S','T'); title('sensitivity'); grid on;
subplot(2,3,5); plot(tt,rad2deg(XX(:,1)),'LineWidth',1.2); grid on;
    xlabel('t [s]'); ylabel('\theta [deg]'); title('recovery');
subplot(2,3,6); plot(tt,uu,'LineWidth',1.2); hold on;
    yline([p.tau_max -p.tau_max],'r--'); grid on;
    xlabel('t [s]'); ylabel('u [N*m]'); title('torque vs limit');

%% ===== helper ==========================================================
function s = ternary(c,a,b)
    if c, s = a; else, s = b; end
end

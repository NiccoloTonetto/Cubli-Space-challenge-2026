%CUBLI_EDGE_FIX  Edge balance with the model's actual index ordering.
%
% RUN THIS ONCE.  It:
%   1. builds the edge plant
%   2. designs the reduced-order LQR + yaw projection
%   3. linearises the MODEL and finds which index permutation it uses
%   4. permutes the gain to match and verifies closed-loop stability
%   5. leaves Kp_model in the workspace for the Gain block
%
% Then set the Gain block to  -Kp_model  and press run.

mdl = 'simscape3d';
if ~bdIsLoaded(mdl), load_system(mdl); end
p = cubli_cube_params;


%% ============================================ 1. EDGE PLANT
c_e = [0.075; 0.075; 0];          % Z edge at (+x,+y), gimbal at its midpoint
e   = [0; 0; 1];

r   = p.com - c_e;
r_p = r - (r.'*e)*e;
ell = norm(r_p);
gB  = -r_p/ell;

bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for k = 1:3, bodies{end+1} = {p.m_wheel, p.com_wheel{k}, p.I_wheel{k}}; end %#ok<AGROW>
Theta = zeros(3);
for i = 1:numel(bodies)
    d = bodies{i}{2} - c_e;
    Theta = Theta + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
end

Sg = p.m_total*p.g*ell;
Tb = Theta - p.Is*eye(3);
P  = eye(3) - gB*gB.';
Z  = zeros(3);  I3 = eye(3);
G  = Tb \ (Sg*P);
A  = [ Z I3 Z ; G Z Z ; -G Z Z ];
B  = [ Z ; -inv(Tb) ; inv(p.Is*eye(3)) + inv(Tb) ];

lam = sort(real(eig(A)),'descend');
fprintf('\n=== EDGE PLANT ===\n');
fprintf('  ell %.2f mm | Sg %.4f | lambda %.4f, %.4f rad/s\n', ...
        ell*1e3, Sg, lam(1), lam(2));

tgt = [0;-1;0];  ax = cross(gB,tgt); ax = ax/norm(ax);
ang = acos(max(-1,min(1,dot(gB,tgt))));
fprintf('  T_MOUNT  axis %s  angle %.6f rad\n', mat2str(round(ax.',6)), ang);
fprintf('  T_CORNER translation %s m\n', mat2str(-c_e.'));

%% ============================================ 2. GAINS
Q = diag([repmat(1/0.20^2,1,3) repmat(1/2.0^2,1,3) ...
          repmat(1/(0.5*p.omega_cap)^2,1,3)]);
R = (12/p.tau_cont^2)*eye(3);

vcon = [zeros(3,1); Theta*gB; p.Is*gB];
V2   = null(vcon.');
K    = lqr(V2.'*A*V2, V2.'*B, V2.'*Q*V2, R) * V2.';
Kp   = K * blkdiag(P, eye(3), eye(3));

re = sort(real(eig(A - B*Kp)));
fprintf('\n=== GAINS (design order X,Y,Z) ===\n');
fprintf('  %d marginal, max Re of the rest %+.4f\n', ...
        sum(re > -1e-6), max(re(re < -1e-6)));

%% ============================================ 3. LINEARISE THE MODEL
io(1) = linio([mdl '/Gain'], 1, 'openinput');     % <- rename if your gain differs
io(2) = linio([mdl '/Mux'],  1, 'openoutput');
sys = linearize(mdl, 0, io);

want = {'PIVOT.Rx.q','PIVOT.Ry.q','PIVOT.Rz.q', ...
        'PIVOT.Rx.w','PIVOT.Ry.w','PIVOT.Rz.w', ...
        'Revolute_Joint2.Rz.w','Revolute_Joint1.Rz.w','Revolute_Joint.Rz.w'};
idx = zeros(1,9);
for i = 1:9
    h = find(contains(sys.StateName, want{i}), 1);
    assert(~isempty(h), 'state %s not found', want{i});
    idx(i) = h;
end
[Af, Bf] = ssdata(sys);
Am = Af(idx,idx);  Bm = Bf(idx,:);
fprintf('\n=== MODEL vs DESIGN, as extracted ===\n');
fprintf('  A err %.3e   B err %.3e\n', ...
        norm(Am-A)/norm(A), norm(Bm-B)/norm(B));

%% ============================================ 4. FIND THE PERMUTATION
% Try every permutation of the three axes applied to BOTH the state triples
% and the inputs, and keep whichever reproduces the design plant.
perms3 = perms(1:3);
best = struct('err', inf);
for a = 1:size(perms3,1)                       % state permutation
    E = eye(3);  Ea = E(perms3(a,:),:);
    Ps = blkdiag(Ea, Ea, Ea);
    for b = 1:size(perms3,1)                   % input permutation
        Eb = E(perms3(b,:),:);  Pu = Eb;
        err = norm(Ps*Am*Ps.' - A)/norm(A) + norm(Ps*Bm*Pu.' - B)/norm(B);
        if err < best.err
            best = struct('err',err,'a',perms3(a,:),'b',perms3(b,:),'Ps',Ps,'Pu',Pu);
        end
    end
end
fprintf('\n=== PERMUTATION SEARCH ===\n');
fprintf('  best state order [%d %d %d], input order [%d %d %d], err %.3e\n', ...
        best.a, best.b, best.err);

%% ============================================ 5. PERMUTED GAIN
Kp_model = best.Pu.' * Kp * best.Ps;            %#ok<NASGU>  -> Gain block

cl = sort(real(eig(Am - Bm*Kp_model)));
fprintf('\n=== CLOSED LOOP WITH Kp_model ===\n');
fprintf('  %d marginal, max Re of the rest %+.4f\n', ...
        sum(cl > -1e-6), max(cl(cl < -1e-6)));
if max(cl(cl < -1e-6)) < 0
    fprintf('  STABLE - set the Gain block to  -Kp_model  and run.\n\n');
else
    fprintf('  UNSTABLE - the permutation hypothesis is wrong.\n\n');
end

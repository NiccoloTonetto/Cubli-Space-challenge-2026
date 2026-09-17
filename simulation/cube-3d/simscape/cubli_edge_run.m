%CUBLI_EDGE_RUN  9-state edge balance. Sets up the model and runs it.
%
% Same model, same 3 wheels, same Gimbal, same 9-state controller.
% ONLY T_MOUNT and T_CORNER change - the cube now rests on the Z edge at
% (+75, +75) instead of the (+1,+1,+1) corner.
%
% Paste this whole file into the Command Window, or save it and type
% cubli_edge_run at the prompt.

mdl = 'simscape3d';                       % <-- change if your model is named differently
if ~bdIsLoaded(mdl), load_system(mdl); end

p = cubli_cube_params;

%% ---------------------------------------------------- edge geometry
c_e = [0.075; 0.075; 0];                  % gimbal sits at the edge midpoint
e   = [0; 0; 1];                          % edge direction

r   = p.com - c_e;
r_p = r - (r.'*e)*e;                      % perpendicular component
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

Z = zeros(3); I3 = eye(3);
G = Tb \ (Sg*P);
A = [ Z I3 Z ; G Z Z ; -G Z Z ];
B = [ Z ; -inv(Tb) ; inv(p.Is*eye(3)) + inv(Tb) ];

lam = sort(real(eig(A)),'descend');
fprintf('\n=== Z EDGE (+1,+1) ===\n');
fprintf('  ell    = %.2f mm\n', ell*1e3);
fprintf('  Sg     = %.4f N m\n', Sg);
fprintf('  lambda = %.4f, %.4f rad/s (tau %.0f ms)\n', lam(1), lam(2), 1000/lam(1));

%% ---------------------------------------------------- gains
Q = diag([repmat(1/0.20^2,1,3) repmat(1/2.0^2,1,3) repmat(1/(0.5*p.omega_cap)^2,1,3)]);
R = (12/p.tau_cont^2)*eye(3);

vcon = [zeros(3,1); Theta*gB; p.Is*gB];
V2   = null(vcon.');
K    = lqr(V2.'*A*V2, V2.'*B, V2.'*Q*V2, R) * V2.';
Kp   = K * blkdiag(P, eye(3), eye(3));          %#ok<NASGU>  used by the Gain block

re = sort(real(eig(A - B*Kp)));
fprintf('  Kp: %d marginal, max Re of the rest %+.4f\n', ...
        sum(re > -1e-6), max(re(re < -1e-6)));

%% ---------------------------------------------------- configure the model
tgt = [0;-1;0];
ax  = cross(gB,tgt);  ax = ax/norm(ax);
ang = acos(max(-1,min(1,dot(gB,tgt))));
fprintf('  mount  %.4f deg about [%+.4f %+.4f %+.4f]\n', rad2deg(ang), ax);
fprintf('\nSET THESE TWO BLOCKS BY HAND, then run the model:\n');
fprintf('  T_MOUNT   axis  %s   angle %.6f rad\n', mat2str(round(ax.',6)), ang);
fprintf('  T_CORNER  translation %s m\n', mat2str(round(-c_e.',6)));
fprintf('  Gain block: -Kp,  Multiplication = Matrix(K*u)\n');
fprintf('  PIVOT: Rx position 0.02 rad Low, everything else 0 High\n');
fprintf('  All damping 0. Gravity [0 -9.80665 0].\n\n');

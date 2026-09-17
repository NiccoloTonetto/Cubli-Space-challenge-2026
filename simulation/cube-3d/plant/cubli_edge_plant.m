function ed = cubli_edge_plant(p, e, sgn)
%CUBLI_EDGE_PLANT  Edge balance is a ONE-DoF problem, not three.
%
%  Contact is a LINE, not a point. The cube has one rotational freedom about
%  the edge axis e. Rotation about the perpendicular horizontal axis would
%  pivot on an end corner and is unilaterally constrained - stable while the
%  COM projects inside the segment, which it does with ~70 mm to spare. Yaw
%  about the vertical needs sliding and is held by friction.
%
%  CRUCIAL: only the wheel whose spin axis is PARALLEL to e produces torque
%  about the edge. For a Z edge, torque about e from wheel k is -u_k*(a_k.e)
%  = -u_3. Wheels X and Y push against the contact line and do nothing but
%  add inertia. Edge balance is therefore SINGLE-WHEEL: exactly the panel
%  stage with different numbers.
%
%  Modelling the edge as a 3-DoF pivot at the edge MIDPOINT is wrong and was
%  the source of the 19.8 % A-matrix error: the COM sits 4.77 mm off the
%  perpendicular through the midpoint, so that mount is not an equilibrium.
%
%  e   edge direction, a body axis, e.g. [0;0;1]
%  sgn signs of the two transverse coordinates at the contact, e.g. [1 1]
%      -> for e = z, the edge at (+x,+y)

e = e(:)/norm(e);
k = find(abs(e) > 0.5, 1);                 % which body axis
t = setdiff(1:3, k);                       % the two transverse axes
cl = zeros(3,1);  cl(t) = sgn(:)*p.a/2;    % a point ON the contact line
ed.e = e;  ed.k = k;  ed.line = cl;

bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for j = 1:3, bodies{end+1} = {p.m_wheel, p.com_wheel{j}, p.I_wheel{j}}; end %#ok<AGROW>

r  = p.com - cl;  rp = r - (r.'*e)*e;       % perpendicular COM offset
ed.ell = norm(rp);
ed.gB  = -rp/ed.ell;                        % gravity direction at balance
ed.Sg  = p.m_total*p.g*ed.ell;

Th = 0;                                     % scalar inertia about the LINE
for i = 1:numel(bodies)
    d  = bodies{i}{2} - cl;  dp = d - (d.'*e)*e;
    Th = Th + e.'*bodies{i}{3}*e + bodies{i}{1}*(dp.'*dp);
end
ed.Theta = Th;
ed.Tb    = Th - p.Is;                       % the active wheel is freed
ed.lambda = sqrt(ed.Sg/ed.Tb);

ed.A = [0 1 0; ed.Sg/ed.Tb 0 0; -ed.Sg/ed.Tb 0 0];
ed.B = [0; -1/ed.Tb; 1/p.Is + 1/ed.Tb];
ed.rank_ctrb = rank(ctrb(ed.A, ed.B));      % 3 of 3 - NO conserved mode
ed.theta_static = asin(min(1, p.tau_cont/ed.Sg));
end

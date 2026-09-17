function c = cubli_corner_plant(p, n)
%CUBLI_CORNER_PLANT  Linear + nonlinear constants for balancing on corner n.
%   n indexes p.corners (1..8). Omit for the (+1,+1,+1) primary corner.
if nargin < 2 || isempty(n)
    % FIXED 21/08/2026. This defaulted to a HARDCODED (+1,+1,+1) while
    % p.corner had been moved to (-1,-1,-1). The model was mounted on one
    % corner and the gains designed for the other - ell 138.75 vs 121.08 mm,
    % Sg 2.222 vs 1.939 - which left one closed-loop pole at +0.186 and made
    % the Simscape run drift away over ~20 s.
    % Default now FOLLOWS p.corner.
    n = find(arrayfun(@(s) norm(s.pos - p.corner) < 1e-9, p.corners), 1);
    assert(~isempty(n), 'p.corner does not match any entry in p.corners');
end
c.n = n;  c.sign = p.corners(n).sign;  c.pos = p.corners(n).pos;
c.ell = p.corners(n).ell;  c.gB = p.corners(n).gB;
c.r_c = -c.ell*c.gB;                      % contact -> COM, body coords
c.Sg  = p.m_total*p.g*c.ell;
c.theta_eq = p.corners(n).theta_eq;

bodies = {{p.m_frame, p.com_frame, p.I_frame}};
for k = 1:3, bodies{end+1} = {p.m_wheel, p.com_wheel{k}, p.I_wheel{k}}; end %#ok<AGROW>
c.Theta = zeros(3);
for i = 1:numel(bodies)
    d = bodies{i}{2} - c.pos;
    c.Theta = c.Theta + bodies{i}{3} + bodies{i}{1}*((d.'*d)*eye(3) - d*d.');
end
c.Tb = c.Theta - p.Is*eye(3);
c.P  = eye(3) - c.gB*c.gB.';
Z = zeros(3); I3 = eye(3);  G = c.Tb\(c.Sg*c.P);
c.A = [Z I3 Z; G Z Z; -G Z Z];
c.B = [Z; -inv(c.Tb); inv(p.Is*I3) + inv(c.Tb)];
ev = eig(c.A);  c.lambda = sort(real(ev(real(ev) > 1e-6)),'descend');
end

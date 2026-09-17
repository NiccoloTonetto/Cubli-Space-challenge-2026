function y = cubli_ss_att(u, gB)
%CUBLI_SS_ATT  Gimbal [q; qdot] -> [phi; omega_body], the two design signals.
%   phi = -gB x gam,  gam = R'*gB,  R = Rx(q1)Ry(q2)Rz(q3)
%   om  = Rz'Ry'[qd1;0;0] + Rz'[0;qd2;0] + [0;0;qd3]
% The Gimbal reports EULER ANGLES and PRIMITIVE RATES. Neither is what the
% design plant uses. Both agree only at q = 0.
if nargin < 2, gB = evalin('base','gB_ctrl'); end
q = u(1:3);  qd = u(4:6);  gB = gB(:);
ca=cos(q(1)); sa=sin(q(1)); cb=cos(q(2)); sb=sin(q(2)); cg=cos(q(3)); sg=sin(q(3));
Rx=[1 0 0;0 ca -sa;0 sa ca];  Ry=[cb 0 sb;0 1 0;-sb 0 cb];  Rz=[cg -sg 0;sg cg 0;0 0 1];
R  = Rx*Ry*Rz;
phi = -cross(gB, R.'*gB);
om  = Rz.'*Ry.'*[qd(1);0;0] + Rz.'*[0;qd(2);0] + [0;0;qd(3)];
y = [phi; om];
end

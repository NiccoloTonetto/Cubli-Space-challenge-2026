function phi = q2phi(q, gB)
%Q2PHI  Gimbal Euler triple -> REDUCED ATTITUDE, the signal the design uses.
%   q  = [Rx.q; Ry.q; Rz.q] from the Simscape Gimbal, intrinsic XYZ
%   phi = -gB x gam,  gam = R'*gB,  R = Rx(q1)*Ry(q2)*Rz(q3)
%
%  phi is perpendicular to gB BY CONSTRUCTION, so yaw cannot leak into the
%  tilt channel. Feeding the raw Euler triple instead only works while the
%  TOTAL rotation is small, and yaw reaches tens of degrees within 0.3 s
%  because gB'*Theta*gB is only 0.0054 kg m^2.
if nargin < 2, gB = evalin('base','gB_ctrl'); end
ca=cos(q(1)); sa=sin(q(1)); cb=cos(q(2)); sb=sin(q(2)); cg=cos(q(3)); sg=sin(q(3));
R = [ cb*cg,           -cb*sg,            sb    ;
      sa*sb*cg+ca*sg,  -sa*sb*sg+ca*cg,  -sa*cb ;
     -ca*sb*cg+sa*sg,   ca*sb*sg+sa*cg,   ca*cb ];
gam = R.'*gB(:);
phi = -cross(gB(:), gam);
end

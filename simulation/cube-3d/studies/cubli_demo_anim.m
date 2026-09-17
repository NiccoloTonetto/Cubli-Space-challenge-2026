function cubli_demo_anim(th0_deg, speed)
%CUBLI_DEMO_ANIM  3D animation of a corner-balance recovery. No Simscape.
%   cubli_demo_anim          2 deg, real time
%   cubli_demo_anim(3, 0.25) 3 deg, quarter speed
if nargin<1, th0_deg = 2.0; end
if nargin<2, speed = 0.5; end
p = cubli_cube_params; I3=eye(3);
Q = diag([repmat(1/0.20^2,1,3) repmat(1/2.0^2,1,3) repmat(1/(0.5*p.omega_cap)^2,1,3)]);
R = (12/p.tau_cont^2)*eye(3);
vcon = [zeros(3,1); p.Theta*p.gB; p.Is*p.gB]; V2 = null(vcon.');
Kp = (lqr(V2.'*p.A*V2, V2.'*p.B, V2.'*Q*V2, R)*V2.')*blkdiag(p.P,I3,I3);

% ---- simulate ----
Ts = 1/p.f_outer; T = 4; n = round(T/Ts);
axn = null(p.gB.'); x = [deg2rad(th0_deg)*axn(:,1); zeros(6,1)];
X = zeros(n,9);
for k=1:n
  u = -Kp*x; X(k,:) = x.';
  s = min(max((p.omega_cap-abs(x(7:9)))/(0.1*p.omega_cap),0),1);
  sm = sign(u)==sign(x(7:9)); u(sm) = u(sm).*s(sm);
  u = max(min(u,p.tau_cont),-p.tau_cont);
  g = @(z)[z(4:6); p.Tb\(p.Sg*p.P*z(1:3)-u); (p.Is*eye(3))\u - p.Tb\(p.Sg*p.P*z(1:3)-u)];
  k1=g(x);k2=g(x+Ts/2*k1);k3=g(x+Ts/2*k2);k4=g(x+Ts*k3);
  x = x + Ts/6*(k1+2*k2+2*k3+k4);
end

% ---- cube geometry, corner at the origin ----
a = p.a; c = p.corner;
V0 = a*[0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
V0 = V0 - repmat((c.'/a+0.5)*a,8,1);
F  = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
% rotation putting gB along -Z(world)
tgt=[0;0;-1]; ax=cross(p.gB,tgt); ax=ax/norm(ax);
an=acos(max(-1,min(1,dot(p.gB,tgt)))); Kx=[0 -ax(3) ax(2);ax(3) 0 -ax(1);-ax(2) ax(1) 0];
Rm = eye(3)+sin(an)*Kx+(1-cos(an))*(Kx*Kx);

f = figure('Name','Cubli - corner balance','Color','w','Position',[100 60 760 720]);
ax3 = axes('Parent',f); hold(ax3,'on'); grid(ax3,'on')
L = 0.16;
patch('Parent',ax3,'XData',[-L L L -L],'YData',[-L -L L L],'ZData',[0 0 0 0], ...
      'FaceColor',[0.93 0.93 0.93],'EdgeColor',[0.7 0.7 0.7]);
hC = patch('Parent',ax3,'Vertices',zeros(8,3),'Faces',F, ...
      'FaceColor',[0.30 0.55 0.85],'FaceAlpha',0.28,'EdgeColor',[0.1 0.2 0.4],'LineWidth',1.6);
hW = gobjects(3,1); th = linspace(0,2*pi,40);
for k=1:3, hW(k)=plot3(ax3,nan,nan,nan,'LineWidth',2.2,'Color',[0.85 0.33 0.10]); end
hT = title(ax3,'','FontSize',14);
axis(ax3,'equal'); xlim(ax3,[-L L]); ylim(ax3,[-L L]); zlim(ax3,[-0.01 2.1*L]);
view(ax3,[132 18]); xlabel(ax3,'x'); ylabel(ax3,'y'); zlabel(ax3,'z');
set(ax3,'FontSize',11);

step = max(1,round(1/(speed*30*Ts)));
for k = 1:step:n
  ph = X(k,1:3).'; nn = norm(ph);
  if nn<1e-9, Rb=eye(3); else
    kk=ph/nn; Kk=[0 -kk(3) kk(2);kk(3) 0 -kk(1);-kk(2) kk(1) 0];
    Rb=eye(3)+sin(nn)*Kk+(1-cos(nn))*(Kk*Kk);
  end
  Rt = Rm*Rb;
  V = (Rt*V0.').';
  set(hC,'Vertices',V);
  for w=1:3
    ctr = Rt*p.com_wheel{w}; nrm = Rt(:,w);
    e1 = null(nrm.'); ring = ctr + 0.055*(e1(:,1)*cos(th)+e1(:,2)*sin(th));
    set(hW(w),'XData',ring(1,:),'YData',ring(2,:),'ZData',ring(3,:));
  end
  set(hT,'String',sprintf('t = %.2f s      tilt = %.2f deg      wheels = [%+.0f %+.0f %+.0f] rad/s', ...
      (k-1)*Ts, rad2deg(norm(p.P*X(k,1:3).')), X(k,7:9)));
  drawnow limitrate
end
set(hT,'String',sprintf('BALANCED   -   final tilt %.3f deg', rad2deg(norm(p.P*X(end,1:3).'))));
end

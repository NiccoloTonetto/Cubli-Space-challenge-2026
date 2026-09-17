function r = cubli_edge_sim(p, ed, K, th0, om0, o)
%CUBLI_EDGE_SIM  One-DoF edge balance, single wheel, 400 Hz ZOH.
%   x = [theta; thetadot; rho]   theta about the edge axis e
%   Tb*thddot = Sg*sin(theta) - u ;  Is*(thddot + rhoddot) = u
%   Accelerometer at r_arm from the contact LINE sees theta - omdot*r_arm/g.
%   o: tmax fs h tau_max omega_cap ff delay acc_sd gyro_sd gyro_bias r_arm kP kI
d=struct('tmax',6,'fs',400,'h',6.25e-4,'tau_max',p.tau_cont,'omega_cap',p.omega_cap, ...
  'ff',1,'delay',2,'acc_sd',0,'gyro_sd',0,'gyro_bias',0,'r_arm',0,'kP',4,'kI',0.5,'seed',1);
if nargin<6||isempty(o), o=struct; end
fn=fieldnames(d); for i=1:numel(fn), if ~isfield(o,fn{i}), o.(fn{i})=d.(fn{i}); end, end
rng(o.seed);
nsub=max(1,round(1/(o.fs*o.h))); h=1/(o.fs*nsub); N=round(o.tmax*o.fs); dt=1/o.fs;
x=[th0;om0;0]; ub=zeros(1,o.delay+1); ua=0; ghat=0; bhat=0;
th=zeros(N+1,1); rho=zeros(N+1,1); U=zeros(N+1,1); th(1)=th0;
for n=1:N
    omd=(ed.Sg*sin(x(1))-ua)/ed.Tb;
    ta = x(1) - omd*o.r_arm/p.g;                 % accelerometer tilt, lever arm
    if o.acc_sd>0, ta = ta + o.acc_sd*randn; end
    omm = x(2) + o.gyro_bias; if o.gyro_sd>0, omm=omm+o.gyro_sd*randn; end
    e = ta - ghat;
    ghat = ghat + ((omm-bhat) + o.kP*e)*dt;
    bhat = bhat - o.kI*e*dt;
    u = -K*[ghat; omm-bhat; x(3)];
    if o.ff, u = u + p.tau_cw*tanh(x(3)/p.eps_ff); end
    u = max(min(u,o.tau_max),-o.tau_max);
    if (x(3)>= o.omega_cap && u>0) || (x(3)<=-o.omega_cap && u<0), u=0; end
    ub=[ub(2:end) u]; ua=ub(1);
    for s=1:nsub
        k1=fx(x,ua,p,ed,o); k2=fx(x+h/2*k1,ua,p,ed,o);
        k3=fx(x+h/2*k2,ua,p,ed,o); k4=fx(x+h*k3,ua,p,ed,o);
        x=x+h/6*(k1+2*k2+2*k3+k4);
    end
    th(n+1)=x(1); rho(n+1)=x(3); U(n)=u;
end
r.t=(0:N).'/o.fs; r.th=th; r.rho=rho; r.U=U;
r.wmax=max(abs(rho)); r.thmax=max(abs(th));
r.ok = max(abs(th(end-o.fs:end))) < deg2rad(1) && max(abs(th)) < deg2rad(60);
end
function dx = fx(x,u,p,ed,o)
uf = u - p.tau_cw*tanh(x(3)/p.eps_ff) - p.b_w*x(3);
thdd = (ed.Sg*sin(x(1)) - uf)/ed.Tb;
dx = [x(2); thdd; uf/p.Is - thdd];
end

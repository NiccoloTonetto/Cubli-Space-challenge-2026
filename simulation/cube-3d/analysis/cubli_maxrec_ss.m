function [th_min, th_azi, info] = cubli_maxrec_ss(p, c, Kp, opt)
%CUBLI_MAXREC_SS  Recovery-angle bisection run on the SIMSCAPE model.
%
%  Same definition and same bisection as CUBLI_MAXREC, but the plant is the
%  multibody model built from the imported STEP geometry.
%
%  WHAT THE SIMSCAPE MODEL CONTAINS
%    rigid-body dynamics from the STEP files        YES
%    torque saturation (the Saturation block)       YES
%    viscous wheel damping (opt.b_w)                YES
%    Coulomb friction / feedforward                 NO
%    momentum cap (omega_cap)                       NO
%    accelerometer, gyro, noise, bias, lever arm    NO
%    estimator, loop delay, quantisation            NO
%    back-EMF envelope                              NO
%
%  So this sweeps the IDEAL-SENSOR, IDEAL-ACTUATOR subset only. That is
%  exactly the subset where CUBLI_NLSIM was validated against it, which is
%  what makes the comparison worth running: agreement here licenses the
%  nonlinear sweeps for everything in the NO column.
%
%  opt: mdl 'simscape3d' | tmax 6 | nazi 4 | hi 25 | iters 12
%       tau_max 0.12 | b_w 0 | perm [1 4 2 5 3 6 7 8 9] | patch 1 | verbose 1

d = struct('mdl','simscape3d','tmax',20,'nazi',4,'hi',25,'iters',12,'patch',1, ...
           'tau_max',0.12,'b_w',0,'perm',[1 4 2 5 3 6 7 8 9],'verbose',1);
if nargin < 4 || isempty(opt), opt = struct; end
fn = fieldnames(d);
for i = 1:numel(fn), if ~isfield(opt,fn{i}), opt.(fn{i}) = d.(fn{i}); end, end
mdl = opt.mdl;
if ~bdIsLoaded(mdl), load_system(mdl); end

% ---- sensing hygiene: filtering off, reduced attitude + body rate on
assignin('base','gB_ctrl', c.gB);
for b = {'Simulink-PS Converter','Simulink-PS Converter1','Simulink-PS Converter2'}
    set_param([mdl '/' b{1}],'FilteringAndDerivatives','zero');
end
if opt.patch, cubli_ss_patch_attitude(mdl,'apply'); else, cubli_ss_patch_attitude(mdl,'revert'); end

% ---- mount the model on THIS corner
tgt = [0; -1; 0];  ax = cross(c.gB, tgt);
mount_axis  = ax/norm(ax);
mount_angle = acos(max(-1, min(1, dot(c.gB, tgt))));
set_param([mdl '/T_MOUNT'],'RotationMethod','ArbitraryAxis', ...
    'RotationArbitraryAxis', mat2str(mount_axis.',9), ...
    'RotationAngle', sprintf('%.9f',mount_angle), 'RotationAngleUnits','rad');
set_param([mdl '/T_CORNER'],'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset', mat2str(-c.pos.',9));
set_param([mdl '/Saturation'],'UpperLimit',sprintf('%.9g',opt.tau_max), ...
                              'LowerLimit',sprintf('%.9g',-opt.tau_max));
for jn = {'Revolute Joint','Revolute Joint1','Revolute Joint2'}
    set_param([mdl '/' jn{1}],'DampingCoefficient',sprintf('%.9g',opt.b_w));
end
set_param(mdl,'StopTime',sprintf('%g',opt.tmax),'SimulationMode','normal');

% ---- the gain the GAIN BLOCK needs: design order permuted into mux order
Sp = eye(9);  Sp = Sp(opt.perm,:);
assignin('base','Kp_model', Kp*Sp.');
qi = [find(opt.perm==1) find(opt.perm==2) find(opt.perm==3)];   % mux cols of qx,qy,qz
vn = get_param([mdl '/To Workspace'],'VariableName');

[e1, e2] = deal_basis(c.gB);
azi = linspace(0,2*pi,opt.nazi+1);  azi(end) = [];
th_azi = zeros(1,opt.nazi);  nrun = 0;  t0 = tic;
for j = 1:opt.nazi
    nhat = cos(azi(j))*e1 + sin(azi(j))*e2;
    lo = 0;  up = opt.hi;
    [ok, ~] = trial(up);  nrun = nrun + 1;
    if ok, th_azi(j) = up; continue; end
    for it = 1:opt.iters
        mid = 0.5*(lo+up);
        [ok, ~] = trial(mid);  nrun = nrun + 1;
        if ok, lo = mid; else, up = mid; end
    end
    th_azi(j) = lo;
    if opt.verbose
        fprintf('   azimuth %3.0f deg -> %.2f deg  (%d runs, %.0f s elapsed)\n', ...
            rad2deg(azi(j)), lo, nrun, toc(t0));
    end
end
th_min = min(th_azi);
info = struct('runs',nrun,'seconds',toc(t0),'azi',rad2deg(azi),'opt',opt, ...
              'mount_angle_deg',rad2deg(mount_angle));

% ------------------------------------------------------------------ nested
    function [ok, tilt] = trial(theta_deg)
        % exact intrinsic-XYZ angles for a rotation vector theta*nhat
        Rt = expm(sk(deg2rad(theta_deg)*nhat));
        bb = asin(max(-1,min(1,Rt(1,3))));
        gg = atan2(-Rt(1,2), Rt(1,1));
        aa = atan2(-Rt(2,3), Rt(3,3));
        set_param([mdl '/PIVOT'], ...
            'RxPositionTargetValue',sprintf('%.12g',aa),'RxPositionTargetPriority','Low', ...
            'RyPositionTargetValue',sprintf('%.12g',bb),'RyPositionTargetPriority','Low', ...
            'RzPositionTargetValue',sprintf('%.12g',gg),'RzPositionTargetPriority','Low');
        so = sim(mdl);
        X  = so.(vn);  tt = X.Time;  q = X.Data(:,qi);
        tilt = zeros(numel(tt),1);
        for k = 1:numel(tt)
            R = rx(q(k,1))*ry(q(k,2))*rz(q(k,3));
            tilt(k) = acos(max(-1,min(1, (R.'*c.gB).'*c.gB)));
        end
        ok = max(tilt(tt >= opt.tmax-1)) < deg2rad(1) && max(tilt) < deg2rad(60);
    end
end

function [e1,e2] = deal_basis(v), [U,~,~] = svd(v(:)); e1 = U(:,2); e2 = U(:,3); end
function S = sk(v), S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0]; end
function R = rx(a), R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
function R = ry(b), R = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)]; end
function R = rz(g), R = [cos(g) -sin(g) 0; sin(g) cos(g) 0; 0 0 1]; end

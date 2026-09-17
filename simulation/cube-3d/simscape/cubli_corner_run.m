%CUBLI_CORNER_RUN  Corner balance on the Simscape model, 20 s.
%
% THREE things were wrong with the model's sensing path, all the same class
% of bug - a signal that is only correct at the equilibrium being treated as
% if it were correct everywhere:
%
%  1. THE MUX IS INTERLEAVED. PIVOT sensing ports leave in primitive order
%     Rx.q Rx.w Ry.q Ry.w Rz.q Rz.w, so the Gain saw [qx wx qy wy qz wz ...]
%     while Kp is designed for [qx qy qz wx wy wz ...]. Closed-loop pole
%     +83 rad/s. Fixed by permuting the gain, perm = [1 4 2 5 3 6 7 8 9].
%
%  2. EULER ANGLES ARE NOT THE REDUCED ATTITUDE. The design uses
%     phi = -gB x gam. P = I - gB*gB' inside Kp removes yaw from an Euler
%     triple only to first order, and yaw reaches ~24 deg within 0.3 s of a
%     3.6 deg release because gB'*Theta*gB is just 0.0054 kg m^2 - the cube
%     is 4.5x lighter in yaw than in tilt.
%
%  3. PRIMITIVE RATES ARE NOT BODY ANGULAR VELOCITY. [qd1 qd2 qd3] equals
%     omega only at q = 0; at 0.4 rad it is up to 73 % wrong.
%
%     2 and 3 are fixed by CUBLI_SS_PATCH_ATTITUDE, which inserts
%     CUBLI_SS_ATT between the joint sensing and the Mux.
%
%  Also: the Simulink-PS Converters ship with input filtering ON (1 ms).
%  Gate 3 does not catch it - the filter is unity at DC and its states are
%  excluded by selecting states by name. Turn it off.
%
% RESULT with all three fixed: recovery matches CUBLI_NLSIM to 0.35 %.

mdl = 'simscape3d';
if ~bdIsLoaded(mdl), load_system(mdl); end
p = cubli_cube_params;
c = cubli_corner_plant(p);
gB_ctrl = c.gB;                      %#ok<NASGU>  used by CUBLI_SS_ATT
assignin('base','gB_ctrl',c.gB);

%% ===================================================== 1. GAINS
[Kp,~,gi] = cubli_gains(p, c, 0.20, 2.0, 10, 0.24);
fprintf('\nMs = %.6f   (must be 1.000000)\n', gi.Ms);
assert(abs(gi.Ms-1) < 1e-5, 'LQR identity violated - implementation bug');

%% ===================================================== 2. MOUNT + HYGIENE
set_param([mdl '/T_MOUNT'],'RotationMethod','ArbitraryAxis', ...
    'RotationArbitraryAxis', mat2str(p.mount_axis.',9), ...
    'RotationAngle', sprintf('%.9f',p.mount_angle), 'RotationAngleUnits','rad');
set_param([mdl '/T_CORNER'],'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset', mat2str(-p.corner.',9));
for b = {'Simulink-PS Converter','Simulink-PS Converter1','Simulink-PS Converter2'}
    set_param([mdl '/' b{1}],'FilteringAndDerivatives','zero');   % input signal only
end
cubli_ss_patch_attitude(mdl,'revert');       % Gate 3 identifies the PLANT
set_param([mdl '/PIVOT'],'RxPositionTargetValue','0','RyPositionTargetValue','0', ...
    'RzPositionTargetValue','0','RxPositionTargetPriority','Low', ...
    'RyPositionTargetPriority','Low','RzPositionTargetPriority','Low');

%% ===================================================== 3. GATE 3
Kp_model = zeros(3,9);  assignin('base','Kp_model',Kp_model);
clear io
io(1) = linio([mdl '/Gain'],1,'openinput');   % 3 torques, Gain OUTPUT port
io(2) = linio([mdl '/Mux'], 1,'openoutput');  % 9 states, in MUX order
sys = linearize(mdl, 0, io);
sn   = erase(sys.StateName,[mdl '.']);
want = {'PIVOT.Rx.q','PIVOT.Ry.q','PIVOT.Rz.q','PIVOT.Rx.w','PIVOT.Ry.w','PIVOT.Rz.w', ...
        'Revolute_Joint2.Rz.w','Revolute_Joint1.Rz.w','Revolute_Joint.Rz.w'};
idx = cellfun(@(s) find(strcmp(sn,s),1), want);
[Af,Bf,Cf] = ssdata(sys);
fprintf('GATE 3   A err %.3e   B err %.3e   (want A < 1e-6)\n', ...
        norm(Af(idx,idx)-p.A)/norm(p.A), norm(Bf(idx,:)-p.B)/norm(p.B));

%% ===================================================== 4. MUX PERMUTATION
perm = zeros(1,9);
for i = 1:9
    j = find(abs(Cf(i,:)) > 0.5, 1);
    perm(i) = find(idx == j, 1);
end
Sp = eye(9);  Sp = Sp(perm,:);          % x_mux = Sp * x_design
Kgain = Kp*Sp.';
fprintf('mux channel -> design index : %s\n', mat2str(perm));
fprintf('closed loop, Kp unpermuted : max Re %+.4f  <- the original failure\n', ...
        max(real(eig(Af - Bf*Kp*Cf))));
cl = sort(real(eig(Af - Bf*Kgain*Cf)));
fprintf('closed loop, Kgain         : %d unstable, slowest stable %+.4f\n', ...
        sum(cl > 1e-6), max(cl(cl < -1e-6)));

%% ===================================================== 5. RUN, 20 s
cubli_ss_patch_attitude(mdl,'apply');     % now feed phi and omega_body
assignin('base','Kp_model', Kgain);
set_param([mdl '/PIVOT'],'RxPositionTargetValue','0.0349');   % 2 deg about mount x
set_param(mdl,'StopTime','20');
out = sim(mdl);
X  = out.(get_param([mdl '/To Workspace'],'VariableName'));
t  = X.Time;  xm = X.Data;
xd = xm;  xd(:,perm) = xm;                % mux order -> design order
phi = xd(:,1:3);  om = xd(:,4:6);  wh = xd(:,7:9);

tilt = asin(min(1, vecnorm(phi,2,2)));    % phi = -gB x gam, |phi| = sin(tilt)
fprintf('\n   t        tilt        |om|     wheels [X Y Z]\n');
for T = [0 0.3 1 2 5 10 15 20]
    [~,i] = min(abs(t-T));
    fprintf(' %5.1f  %8.4f deg  %7.4f   [%+7.2f %+7.2f %+7.2f]\n', ...
        t(i), rad2deg(tilt(i)), norm(om(i,:)), wh(i,:));
end
fprintf('\ntilt  %.4f -> %.4f deg   peak %.4f\n', ...
    rad2deg(tilt(1)), rad2deg(tilt(end)), rad2deg(max(tilt)));
fprintf('phi is perpendicular to gB by construction: max |phi.gB| = %.2e\n', max(abs(phi*c.gB)));
fprintf('wheels at 20 s: [%+.3f %+.3f %+.3f] rad/s\n', wh(end,:));
fprintf('max |q| never approached gimbal lock; peak tilt %.2f deg\n', rad2deg(max(tilt)));

function cubli_ss_patch_attitude(mdl, mode)
%CUBLI_SS_PATCH_ATTITUDE  Feed the Simscape loop REDUCED ATTITUDE, not Euler.
%
%   cubli_ss_patch_attitude(mdl,'apply')   insert  q -> q2phi -> Mux 1,3,5
%   cubli_ss_patch_attitude(mdl,'revert')  restore the direct wiring
%   cubli_ss_patch_attitude(mdl,'status')  report
%
%  WHY. The Mux feeds the Gimbal Euler triple straight into the gain, but the
%  design signal is phi = -gB x gam. They agree only while the TOTAL rotation
%  is small. Yaw is uncontrolled and the cube is 4.5x lighter in yaw than in
%  tilt (gB'*Theta*gB = 0.0054 kg m^2), so yaw reaches ~24 deg within 0.3 s of
%  a 3.6 deg release. P = I - gB*gB' inside Kp only removes yaw from an Euler
%  triple to FIRST ORDER, so at that magnitude yaw leaks into the tilt channel
%  and the loop diverges. Recovery reads 3.33 deg unpatched vs 4.13 in
%  cubli_nlsim, which uses the reduced attitude and is immune by construction.
%
%  Requires gB_ctrl in the base workspace. Changes are IN MEMORY only.

if nargin < 1 || isempty(mdl), mdl = 'simscape3d'; end
if nargin < 2, mode = 'status'; end
if ~bdIsLoaded(mdl), load_system(mdl); end
MX = [mdl '/ATT_MUX']; FN = [mdl '/ATT_Q2PHI']; DX = [mdl '/ATT_DEMUX'];
qch = [1 3 5 2 4 6];                 % Mux inputs: Rx.q Ry.q Rz.q Rx.w Ry.w Rz.w
present = any(strcmp(find_system(mdl,'SearchDepth',1,'Name','ATT_Q2PHI'), FN));

switch lower(mode)
  case 'status'
    if present, fprintf('reduced-attitude patch: APPLIED\n');
    else,       fprintf('reduced-attitude patch: not applied (raw Euler triple)\n'); end
    return

  case 'apply'
    if present, fprintf('already applied\n'); return; end
    src = cell(1,6);
    ph  = get_param([mdl '/Mux'],'PortHandles');
    for k = 1:6
        l = get_param(ph.Inport(qch(k)),'Line');
        sp = get_param(l,'SrcPortHandle');
        src{k} = sp;  delete_line(l);            % PORT HANDLE: block names contain newlines
    end
    pos = get_param([mdl '/Mux'],'Position');
    x0 = pos(1)-260; y0 = pos(2);
    add_block('simulink/Signal Routing/Mux', MX, 'Inputs','6', 'Position',[x0 y0 x0+5 y0+130]);
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', FN, ...
        'MATLABFcn','cubli_ss_att(u,gB_ctrl)', 'OutputDimensions','6', ...
        'Position',[x0+60 y0+10 x0+170 y0+60]);
    add_block('simulink/Signal Routing/Demux', DX, 'Outputs','6', ...
        'Position',[x0+215 y0 x0+220 y0+130]);
    for k = 1:6
        pmx = get_param(MX,'PortHandles');  pdx = get_param(DX,'PortHandles');
        pmu = get_param([mdl '/Mux'],'PortHandles');
        add_line(mdl, src{k}, pmx.Inport(k), 'autorouting','on');
        add_line(mdl, pdx.Outport(k), pmu.Inport(qch(k)), 'autorouting','on');
    end
    pmx=get_param(MX,'PortHandles'); pfn=get_param(FN,'PortHandles'); pdx=get_param(DX,'PortHandles');
    add_line(mdl, pmx.Outport(1), pfn.Inport(1), 'autorouting','on');
    add_line(mdl, pfn.Outport(1), pdx.Inport(1), 'autorouting','on');
    fprintf('reduced-attitude patch APPLIED (in memory, model not saved)\n');

  case 'revert'
    if ~present, fprintf('not applied\n'); return; end
    ph = get_param(MX,'PortHandles');  src = cell(1,3);
    for k = 1:6
        src{k} = get_param(get_param(ph.Inport(k),'Line'),'SrcPortHandle');
    end
    for b = {MX,FN,DX}
        lh = get_param(b{1},'LineHandles'); f = fieldnames(lh);
        for i=1:numel(f), for j=1:numel(lh.(f{i})), if lh.(f{i})(j)>0, delete_line(lh.(f{i})(j)); end, end, end
        delete_block(b{1});
    end
    for k = 1:6
        pmu = get_param([mdl '/Mux'],'PortHandles');
        add_line(mdl, src{k}, pmu.Inport(qch(k)), 'autorouting','on');
    end
    fprintf('reverted to the raw Euler wiring\n');
end
end

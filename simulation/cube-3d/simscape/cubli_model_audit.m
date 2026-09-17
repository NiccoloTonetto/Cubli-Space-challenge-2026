%CUBLI_MODEL_AUDIT  Dump every block parameter that affects the dynamics and
% compare against cubli_cube_params. Run this before rebuilding.

mdl = 'simscape3d';
p = cubli_cube_params;

blocks = {'T_MOUNT','T_CORNER','T_WX_POS','T_WX_ROT','T_WY_POS','T_WY_ROT', ...
          'T_WZ_POS','FRAME','WHEEL_X','WHEEL_Y','WHEEL_Z'};

for i = 1:numel(blocks)
    b = [mdl '/' blocks{i}];
    fprintf('\n===== %s =====\n', blocks{i});
    try
        d = get_param(b,'DialogParameters');
        f = fieldnames(d);
        for j = 1:numel(f)
            v = get_param(b, f{j});
            if ischar(v) && ~isempty(v) && ~strcmp(v,'off')
                fprintf('  %-32s %s\n', f{j}, v);
            end
        end
    catch ME
        fprintf('  NOT FOUND: %s\n', ME.message);
    end
end
fprintf('\n================ MODEL AUDIT ================\n');

for b = {'ROT_INV','POS_INV','ROT_INV1','POS_INV1','ROT_INV2','POS_INV2'}
    try
        h = [mdl '/' b{1}];
        fprintf('\n== %s ==\n', b{1});
        fprintf('  rot method : %s\n', get_param(h,'RotationMethod'));
        fprintf('  std axis   : %s\n', get_param(h,'RotationStandardAxis'));
        fprintf('  angle      : %s %s\n', get_param(h,'RotationAngle'), ...
            get_param(h,'RotationAngleUnits'));
        fprintf('  trans      : %s\n', get_param(h,'TranslationMethod'));
        fprintf('  offset     : %s %s\n', get_param(h,'TranslationCartesianOffset'), ...
            get_param(h,'TranslationCartesianOffsetUnits'));
    catch
        fprintf('\n== %s : not found ==\n', b{1});
    end
end

%% gravity
mc = find_system(mdl,'BlockType','SimscapeMechanismConfiguration');
if ~isempty(mc)
    fprintf('GRAVITY : %s   (want [0 -9.80665 0])\n', ...
            get_param(mc{1},'UniformGravity'));
end

%% every rigid transform
fprintf('\n--- RIGID TRANSFORMS ---\n');
rt = find_system(mdl,'BlockType','SimscapeRigidTransform');
for i = 1:numel(rt)
    nm = get_param(rt{i},'Name');
    fprintf('%-14s rot=%-12s ', nm, get_param(rt{i},'RotationMethod'));
    try
        fprintf('axis=%-22s ang=%-12s %s ', ...
            get_param(rt{i},'AxisOfRotation'), ...
            get_param(rt{i},'AngleOfRotation'), ...
            get_param(rt{i},'AngleUnits'));
    catch
        try
            fprintf('stdaxis=%-6s ang=%-10s %s ', ...
                get_param(rt{i},'StandardAxis'), ...
                get_param(rt{i},'AngleOfRotation'), ...
                get_param(rt{i},'AngleUnits'));
        catch, fprintf('%-45s ',''); end
    end
    fprintf('| trans=%-10s ', get_param(rt{i},'TranslationMethod'));
    try
        fprintf('off=%-28s %s', get_param(rt{i},'TranslationCartesianOffset'), ...
                get_param(rt{i},'OffsetUnits'));
    catch, end
    fprintf('\n');
end

%% every solid
fprintf('\n--- SOLIDS ---\n');
sb = [find_system(mdl,'BlockType','SimscapeSolid'); ...
      find_system(mdl,'BlockType','SimscapeFileSolid')];
for i = 1:numel(sb)
    nm = get_param(sb{i},'Name');
    fprintf('%-12s inertia=%-10s ', nm, get_param(sb{i},'InertiaType'));
    try
        fprintf('m=%-12s com=%-30s\n', get_param(sb{i},'Mass'), ...
                get_param(sb{i},'CenterOfMass'));
        fprintf('%14s moi=%-34s poi=%s\n', '', ...
                get_param(sb{i},'MomentsOfInertia'), ...
                get_param(sb{i},'ProductsOfInertia'));
    catch ME
        fprintf('  (%s)\n', ME.message);
    end
end

%% joints
fprintf('\n--- JOINTS ---\n');
jb = [find_system(mdl,'BlockType','SimscapeRevoluteJoint'); ...
      find_system(mdl,'BlockType','SimscapeGimbalJoint')];
for i = 1:numel(jb)
    nm = get_param(jb{i},'Name');
    fprintf('%-20s ', nm);
    for prim = {'','Rx','Ry','Rz'}
        try
            d = get_param(jb{i},[prim{1} 'DampingCoefficient']);
            fprintf('%sdamp=%-8s ', prim{1}, d);
        catch, end
    end
    fprintf('\n');
end

%% what the params say
fprintf('\n--- EXPECTED FROM cubli_cube_params ---\n');
fprintf('T_MOUNT   axis %s  angle %.6f rad\n', mat2str(round(p.mount_axis.',6)), p.mount_angle);
fprintf('T_CORNER  translation %s m\n', mat2str(-p.corner.'));
for k = 1:3
    axname = 'XYZ';
    fprintf('WHEEL_%c   m=%.6f  com=%s\n', axname(k), p.m_wheel, ...
            mat2str(round(p.com_wheel{k}.',6)));
    I = p.I_wheel{k};
    fprintf('          moi=[%.6e %.6e %.6e]\n', I(1,1), I(2,2), I(3,3));
    fprintf('          poi=[%.6e %.6e %.6e]\n', I(2,3), I(3,1), I(1,2));
end
fprintf('FRAME     m=%.6f  com=%s\n', p.m_frame, mat2str(round(p.com_frame.',6)));
fprintf('          moi=[%.6e %.6e %.6e]\n', p.I_frame(1,1), p.I_frame(2,2), p.I_frame(3,3));
fprintf('          poi=[%.6e %.6e %.6e]\n', p.I_frame(2,3), p.I_frame(3,1), p.I_frame(1,2));
fprintf('=============================================\n\n');

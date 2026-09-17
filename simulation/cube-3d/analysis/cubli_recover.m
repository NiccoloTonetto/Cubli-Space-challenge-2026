function m = cubli_recover(mdl,p,qx,perm)
%CUBLI_RECOVER  One recovery run from an Rx-only initial tilt.
set_param([mdl '/PIVOT'],'RxPositionTargetValue',sprintf('%.6f',qx));
out = sim(mdl);
X  = out.(get_param([mdl '/To Workspace'],'VariableName'));
xm = X.Data;  xd = xm;  xd(:,perm) = xm;
q  = xd(:,1:3);  wh = xd(:,7:9);
m.t        = X.Time;
m.tilt     = vecnorm((p.P*q.').',2,2);
m.tilt_end = m.tilt(end);
m.tilt_max = max(m.tilt);
m.wmax     = max(abs(wh(:)));
m.wh       = wh;
m.ok       = rad2deg(m.tilt_end) < 0.5 && m.tilt_max < deg2rad(90);
end

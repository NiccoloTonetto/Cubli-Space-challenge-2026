function [th_min, th_azi, azi] = cubli_maxrec(p, c, Kp, o, nazi, hi)
%CUBLI_MAXREC  Largest initial tilt from rest that still recovers, by bisection.
%   Swept over nazi tilt azimuths in the plane perpendicular to gB.
%   th_min = worst-case (the number that matters), th_azi = per-azimuth.
if nargin<5||isempty(nazi), nazi=8; end
if nargin<6||isempty(hi),   hi=25;  end          % deg, upper bracket
[U,~,~] = svd(c.gB);  e1=U(:,2); e2=U(:,3);
azi = linspace(0,2*pi,nazi+1); azi(end)=[];
th_azi = zeros(1,nazi);
for j = 1:nazi
    n = cos(azi(j))*e1 + sin(azi(j))*e2;
    lo = 0; up = hi;
    r = cubli_nlsim(p,c,Kp,deg2rad(up)*n,o);
    if r.recovered, th_azi(j)=up; continue; end
    for it = 1:12
        mid = 0.5*(lo+up);
        r = cubli_nlsim(p,c,Kp,deg2rad(mid)*n,o);
        if r.recovered, lo=mid; else, up=mid; end
    end
    th_azi(j) = lo;
end
th_min = min(th_azi);
end

function [Kp, K, info] = cubli_gains(p, c, qa, qr, qw, rt)
%CUBLI_GAINS  Reduced-order LQR for one corner. Bryson weights.
%   qa rad, qr rad/s, qw rad/s, rt N m.  Defaults 0.20 / 2.0 / 20 / tau_cont.
if nargin<3||isempty(qa), qa=0.20; end
if nargin<4||isempty(qr), qr=2.0;  end
if nargin<5||isempty(qw), qw=20;   end
if nargin<6||isempty(rt), rt=p.tau_cont; end
Q = diag([repmat(1/qa^2,1,3) repmat(1/qr^2,1,3) repmat(1/qw^2,1,3)]);
R = (12/rt^2)*eye(3);
vcon = [zeros(3,1); c.Theta*c.gB; p.Is*c.gB];   % conserved yaw momentum
V2   = null(vcon.');
K    = lqr(V2.'*c.A*V2, V2.'*c.B, V2.'*Q*V2, R) * V2.';
Kp   = K * blkdiag(c.P, eye(3), eye(3));        % yaw angle unobservable
if nargout > 2
    Ar=V2.'*c.A*V2; Br=V2.'*c.B; Kr=K*V2;
    w=logspace(-2,4,4000); Ms=0;
    for k=1:numel(w)
        Ms=max(Ms,1/min(svd(eye(3)+Kr*((1i*w(k)*eye(8)-Ar)\Br))));
    end
    re = sort(real(eig(c.A - c.B*Kp)));
    info.Ms = Ms;
    info.marginal = sum(re > -1e-6);
    info.slowest  = max(re(re < -1e-6));
    info.weights  = [qa qr qw rt];
end
end

function cubli_figures(what)
%CUBLI_FIGURES  Generate the report figures into figures/.
%   cubli_figures         all
%   cubli_figures('rec')  just one, by tag
% Tags: rec, est, corners, loop, ctrb, poles, swing, actuator, qw, kp
if nargin<1, what='all'; end
outdir = fullfile(fileparts(mfilename('fullpath')),'figures');
if ~isfolder(outdir), mkdir(outdir); end
set(0,'DefaultAxesFontName','Helvetica','DefaultAxesFontSize',9, ...
      'DefaultLineLineWidth',1.2,'DefaultAxesBox','on');
CB = [0.00 0.45 0.70; 0.84 0.37 0.00; 0.00 0.62 0.45; 0.80 0.47 0.65; 0.35 0.35 0.35];

p = cubli_cube_params();
n_pri = find(arrayfun(@(s) isequal(s.sign,[-1 -1 -1]), p.corners),1);
c = cubli_corner_plant(p,n_pri);
Kp = cubli_gains(p,c,0.20,2.0,10,0.24);
o = struct('tmax',5,'fs',400,'tau_max',0.12,'omega_cap',40,'friction',1,'ff',1, ...
    'tau_cp',5e-3,'delay',2,'acc_sd',3.2e-3,'gyro_sd',2.4e-3, ...
    'gyro_bias',deg2rad([0.3;-0.2;0.15]),'r_imu',-c.pos,'est','mahony','kP',4,'kI',0.5,'seed',1);
doall = strcmp(what,'all');

%% ---- 1. cube recovery, 3-panel ----
if doall||strcmp(what,'rec')
  [U,~,~]=svd(c.gB); nrm = U(:,2);
  r = cubli_nlsim(p,c,Kp,deg2rad(2.8)*nrm,o);
  f=figure('Position',[100 100 700 620],'Color','w');
  ax1=subplot(3,1,1); plot(r.t,rad2deg(r.tilt),'Color',CB(1,:)); hold on
  plot(r.t,rad2deg(r.est_err),'--','Color',CB(2,:));
  ylabel('tilt [deg]'); legend('true tilt','estimate error','Location','northeast');
  title(sprintf('Corner (%+d,%+d,%+d), release from %.1f deg, shipping configuration',c.sign,2.8));
  grid on; xlim([0 3]);
  ax2=subplot(3,1,2); plot(r.t,r.X(:,7:9)); hold on
  yline(+o.omega_cap,'k--'); yline(-o.omega_cap,'k--');
  ylabel('wheel rate [rad/s]'); legend('\rho_x','\rho_y','\rho_z','cap','Location','southeast');
  grid on; xlim([0 3]);
  ax3=subplot(3,1,3); plot(r.t,r.U); hold on
  yline(+o.tau_max,'k--'); yline(-o.tau_max,'k--');
  ylabel('torque [N m]'); xlabel('time [s]');
  legend('u_x','u_y','u_z','clamp','Location','southeast'); grid on; xlim([0 3]);
  linkaxes([ax1 ax2 ax3],'x');
  exportgraphics(f,fullfile(outdir,'cube_recovery.pdf'),'ContentType','vector');
  close(f); fprintf('  cube_recovery.pdf   (tilt_max %.2f deg, wheel %.1f rad/s)\n',rad2deg(r.tilt_max),r.wheel_max);
end

%% ---- 2. estimator comparison ----
if doall||strcmp(what,'est')
  [U,~,~]=svd(c.gB); nrm=U(:,2);
  o_raw=o; o_raw.est='raw';
  o_ideal=o; o_ideal.r_imu=[0;0;0]; o_ideal.acc_sd=0; o_ideal.gyro_sd=0; o_ideal.gyro_bias=[0;0;0];
  rm = cubli_nlsim(p,c,Kp,deg2rad(2.5)*nrm,o);
  rr = cubli_nlsim(p,c,Kp,deg2rad(2.5)*nrm,o_raw);
  ri = cubli_nlsim(p,c,Kp,deg2rad(2.5)*nrm,o_ideal);
  f=figure('Position',[100 100 700 420],'Color','w');
  subplot(2,1,1);
  plot(ri.t,rad2deg(ri.tilt),'Color',CB(5,:)); hold on
  plot(rm.t,rad2deg(rm.tilt),'Color',CB(1,:));
  plot(rr.t,rad2deg(rr.tilt),'Color',CB(2,:));
  ylabel('tilt [deg]'); grid on; xlim([0 3]);
  legend('ideal sensor','complementary filter','raw accelerometer','Location','northwest');
  title('Estimator comparison, release from 2.5 deg, IMU at the geometric centre');
  subplot(2,1,2);
  plot(rm.t,rad2deg(rm.lever_err),'Color',CB(3,:)); hold on
  plot(rm.t,rad2deg(rm.est_err),'Color',CB(1,:));
  ylabel('error [deg]'); xlabel('time [s]'); grid on; xlim([0 3]);
  xline(0.2,':','Color',[0.5 0.5 0.5],'Label','filter converged');
  legend('raw lever-arm error','filtered estimate error','Location','northeast');
  i0 = rm.t>0.2;
  exportgraphics(f,fullfile(outdir,'estimator_comparison.pdf'),'ContentType','vector');
  close(f);
  fprintf('  estimator_comparison.pdf  (raw lever err %.2f deg, filtered after convergence %.3f deg)\n', ...
     rad2deg(max(rm.lever_err)), rad2deg(max(rm.est_err(i0))));
end

%% ---- 3. multi-corner ----
if doall||strcmp(what,'corners')
  rec=[2.89 2.83 2.75 2.73 2.48 2.41 2.40 2.52];
  teq=[0.894 3.521 3.935 3.940 3.811 3.274 3.820 0.780];
  lbl={'(-1,-1,-1)','(-1,+1,-1)','(-1,-1,+1)','(+1,-1,-1)','(+1,+1,-1)','(+1,-1,+1)','(-1,+1,+1)','(+1,+1,+1)'};
  f=figure('Position',[100 100 700 330],'Color','w');
  b=bar([teq; max(rec-teq,0)].','stacked'); b(1).FaceColor=CB(2,:); b(2).FaceColor=CB(1,:);
  b(1).FaceAlpha=0.85; b(2).FaceAlpha=0.55;
  set(gca,'XTickLabel',lbl,'XTickLabelRotation',35);
  ylabel('angle [deg]'); grid on; ylim([0 4.4]);
  legend('equilibrium tilt','usable margin','Location','northeast');
  title('Per-corner recovery envelope, shipping configuration');
  exportgraphics(f,fullfile(outdir,'multi_corner.pdf'),'ContentType','vector'); close(f);
  fprintf('  multi_corner.pdf\n');
end

%% ---- 4. loop rate ----
if doall||strcmp(what,'loop')
  fs=logspace(log10(30),log10(1000),60); mz=zeros(size(fs)); eq=zeros(size(fs));
  for k=1:numel(fs)
    M=zeros(12); M(1:9,1:9)=c.A; M(1:9,10:12)=c.B; eM=expm(M/fs(k));
    Ad=eM(1:9,1:9); Bd=eM(1:9,10:12);
    z=abs(eig([Ad -Bd*Kp; eye(9) zeros(9)]));
    zz=sort(z(abs(z-1)>1e-6&z>1e-9),'descend'); mz(k)=zz(1); eq(k)=fs(k)*log(zz(1));
  end
  f=figure('Position',[100 100 640 340],'Color','w');
  plot(fs,100*eq/(-7.9306),'Color',CB(1,:)); hold on
  xline(38.6,'--','Color',CB(2,:),'Label','stability floor 38.6 Hz','LabelOrientation','horizontal');
  xline(281,'--','Color',CB(3,:),'Label','anti-alias floor 281 Hz','LabelOrientation','horizontal');
  xline(400,'-','Color','k','Label','design 400 Hz','LabelOrientation','horizontal');
  set(gca,'XScale','log'); xlabel('loop rate [Hz]'); ylabel('damping retained [% of continuous]');
  ylim([0 105]); grid on; title('Discrete closed-loop damping vs loop rate, one-cycle delay');
  exportgraphics(f,fullfile(outdir,'loop_rate.pdf'),'ContentType','vector'); close(f);
  fprintf('  loop_rate.pdf\n');
end

%% ---- 5. controllability spectrum ----
if doall||strcmp(what,'ctrb')
  sv=svd(ctrb(c.A,c.B)); sv=sv/sv(1);
  f=figure('Position',[100 100 560 320],'Color','w');
  b=bar(sv,'FaceColor',CB(1,:),'FaceAlpha',0.8); hold on
  b.CData=repmat(CB(1,:),9,1); b.FaceColor='flat'; b.CData(9,:)=CB(2,:);
  set(gca,'YScale','log'); ylim([1e-18 5]);
  xlabel('singular value index'); ylabel('\sigma_i / \sigma_1'); grid on;
  yline(eps,'k--','Label','double precision');
  title('Controllability matrix spectrum: rank 8 of 9');
  exportgraphics(f,fullfile(outdir,'ctrb_spectrum.pdf'),'ContentType','vector'); close(f);
  fprintf('  ctrb_spectrum.pdf   (sigma_9/sigma_1 = %.2e)\n',sv(9));
end

%% ---- 6. pole map ----
if doall||strcmp(what,'poles')
  ol=eig(c.A); cl=eig(c.A-c.B*Kp);
  f=figure('Position',[100 100 560 340],'Color','w');
  plot(real(ol),imag(ol),'x','MarkerSize',11,'Color',CB(2,:),'LineWidth',1.6); hold on
  plot(real(cl),imag(cl),'o','MarkerSize',8,'Color',CB(1,:),'LineWidth',1.4);
  xline(0,'k-'); yline(0,'k-'); grid on;
  xlabel('Re(s) [1/s]'); ylabel('Im(s) [1/s]');
  legend('open loop','closed loop','Location','northwest');
  title('Open- and closed-loop eigenvalues');
  exportgraphics(f,fullfile(outdir,'pole_map.pdf'),'ContentType','vector'); close(f);
  fprintf('  pole_map.pdf\n');
end

%% ---- 7. swing amplitude ----
if doall||strcmp(what,'swing')
  th=linspace(1,60,300); rt=zeros(size(th));
  for k=1:numel(th), rt(k)=ellipke(sin(deg2rad(th(k))/2)^2)*2/pi; end
  f=figure('Position',[100 100 560 320],'Color','w');
  plot(th,(rt.^2-1)*100,'Color',CB(1,:)); hold on
  plot(th,(th.^2*pi^2/180^2/8)*100*2,'--','Color',CB(5,:));
  xlabel('swing half-amplitude [deg]'); ylabel('apparent \Theta overstatement [%]');
  grid on; legend('exact (elliptic)','small-angle \theta^2/8','Location','northwest');
  yline(0.4,':','Color',CB(3,:),'Label','0.4% at 10 deg');
  title('Amplitude error in swing-test inertia measurement');
  exportgraphics(f,fullfile(outdir,'swing_amplitude.pdf'),'ContentType','vector'); close(f);
  fprintf('  swing_amplitude.pdf\n');
end

%% ---- 8. actuator design chart ----
if doall||strcmp(what,'actuator')
  tau=[0.10 0.15 0.20 0.30 0.40]; wc=[40 60 80 120 200];
  Rm=[3.09 3.53 3.45 3.45 3.45; 3.80 4.61 5.12 5.16 5.16;
      4.25 5.38 6.15 7.04 6.86; 4.79 6.36 7.53 9.21 10.83;
      5.03 6.92 8.41 10.76 13.39];
  f=figure('Position',[100 100 600 400],'Color','w');
  [WC,TAU]=meshgrid(wc,tau);
  contourf(WC,TAU,Rm,12,'LineColor',[1 1 1]*0.6); hold on
  [Cn,h]=contour(WC,TAU,Rm,[3 4 5 7 10],'k-','LineWidth',1.1); clabel(Cn,h,'FontSize',8);
  plot(40,0.12,'p','MarkerSize',15,'MarkerFaceColor',CB(2,:),'MarkerEdgeColor','k');
  text(46,0.12,'current','FontSize',9);
  colormap(parula); cb=colorbar; cb.Label.String='worst-case recovery [deg]';
  xlabel('\omega_{cap} [rad/s]'); ylabel('\tau_{max} [N m]');
  title('Actuator design space');
  exportgraphics(f,fullfile(outdir,'actuator_chart.pdf'),'ContentType','vector'); close(f);
  fprintf('  actuator_chart.pdf\n');
end

%% ---- 9. qw robustness ----
if doall||strcmp(what,'qw')
  sc=[0.8 0.9 1.0 1.1 1.2];
  D=[3.11 3.05 2.99 NaN NaN; 3.07 3.00 2.94 2.57 NaN;
     2.98 2.92 2.86 2.80 NaN; 2.91 2.85 2.79 2.73 2.68; 2.84 2.78 2.72 2.67 2.62];
  qwv=[5 6 8 10 12];
  f=figure('Position',[100 100 600 340],'Color','w');
  for k=1:5
    st='-o'; lw=1.2; col=CB(min(k,5),:);
    if qwv(k)==10, lw=2.2; end
    plot(sc,D(k,:),st,'Color',col,'LineWidth',lw,'MarkerFaceColor',col,'MarkerSize',4); hold on
  end
  xlabel('\Theta scaling'); ylabel('recovery [deg]'); grid on; ylim([2.4 3.2]);
  legend(arrayfun(@(v)sprintf('q_w = %d',v),qwv,'uni',0),'Location','southwest');
  title('Wheel-rate weight: robustness to inertia error (gaps = failure)');
  exportgraphics(f,fullfile(outdir,'qw_robustness.pdf'),'ContentType','vector'); close(f);
  fprintf('  qw_robustness.pdf\n');
end

%% ---- 10. kP cliff ----
if doall||strcmp(what,'kp')
  kp=[3 4 5 6 7 8]; rec=[2.67 2.74 2.79 2.82 0 0];
  f=figure('Position',[100 100 560 320],'Color','w');
  b=bar(kp,rec,'FaceColor','flat'); b.CData=repmat(CB(1,:),6,1);
  b.CData(2,:)=CB(3,:); b.CData(5:6,:)=repmat(CB(2,:),2,1);
  hold on; text(4,2.95,'shipped','HorizontalAlignment','center','FontSize',9);
  xlabel('k_P'); ylabel('recovery [deg]'); grid on; ylim([0 3.2]);
  title('Complementary-filter gain: hard cliff between 6 and 7');
  exportgraphics(f,fullfile(outdir,'kp_cliff.pdf'),'ContentType','vector'); close(f);
  fprintf('  kp_cliff.pdf\n');
end
fprintf('done -> %s\n', outdir);
end

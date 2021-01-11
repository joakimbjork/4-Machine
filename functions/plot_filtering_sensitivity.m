function [hfig] = plot_filtering_sensitivity(Gyw,Gzw)


% plot
omega = logspace(0,1.5,500);
M1 = (Gyw(1,1)/Gzw(1,1))*1e-5;
M2 = (Gyw(1,2)/Gzw(1,2))*1e-5;
[MAG,PHASE] = bode([M1,M2,Gzw(1,1)],omega);
MAG = squeeze(MAG);
PHASE = squeeze(PHASE);

MAG = MAG/max(MAG(3,:));
MAGz = MAG(3,:);
m = max(max(MAG(1,:),MAG(2,:)));

% 
h=1/sqrt(2);
figureLatex
co = ([0,0,0;1,0,0;0.7,0.7,0.7;1,0.7,0.7]);
set(groot,'defaultAxesColorOrder',co)
subplot(2,1,1);
loglog(omega,MAG(1,:)/m); hold on
loglog(omega,MAG(2,:)/m);
loglog(omega,MAGz);
% legend('M_1','M_2','G_z_d_1')
yticks(logspace(-4,6,6))
xticks(logspace(-2,2,5));
xlim([min(omega),max(omega)])
ylabel('Amplitude')
box off

% Phase angles, for a beter figure, adjust n*360 
subplot(2,1,2);
n1 = 1; n2 = 1;
semilogx(omega,PHASE(1,:)-n1*360); hold on 
semilogx(omega,PHASE(2,:)-n2*360); 
% semilogx(omega,PHASE([3],:));
yticks(-360*10:90:360*10)
% ylim([-300,190])
ylabel('Phase [$^\circ$]')
xlabel('Angular frequency [rad/s]')
box off
xlim([min(omega),max(omega)])
end
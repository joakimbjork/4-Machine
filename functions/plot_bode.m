function [hfig] = plot_bode(G1,G2)


% plot
omega = logspace(-2,2,500);
[MAG,PHASE] = bode([G1,G2],omega);
MAG = squeeze(MAG);
PHASE = squeeze(PHASE);



% 
h=1/sqrt(2);
figureLatex
co = ([0,0,0;1,0,0;0.7,0.7,0.7;1,0.7,0.7]);
set(groot,'defaultAxesColorOrder',co)
subplot(2,1,1);
loglog(omega,MAG(1,:)); hold on
loglog(omega,MAG(2,:));
% legend('M_1','M_2','G_z_d_1')
% yticks(logspace(-4,6,6))
% xticks(logspace(-2,2,5));
xlim([min(omega),max(omega)])
ylabel('Amplitude')
box off
subplot(2,1,2);
semilogx(omega,PHASE(1,:)); hold on
semilogx(omega,PHASE(2,:)-360); 
% semilogx(omega,PHASE([3],:));
yticks(-360*10:90:360*10)
% ylim([-300,190])
ylabel('Phase [$^\circ$]')
xlabel('Angular frequency [rad/s]')
box off
xlim([min(omega),max(omega)])
end


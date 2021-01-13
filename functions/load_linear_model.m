function [sys,sys_r] = load_linear_model(varargin)
% Loads linearized model
% Reduce model for use in controller design
% Simply using minreal to remove irrelevant states does not work since
% numerical errors are introduced. Manually reducing the system gives more
% control over the process.

if nargin == 1
    plot_fig = varargin{1};
else
    plot_fig = false;
end

load('linearized_four_machine.mat')
sys = sminreal(sys);
% sys = d2c(sys,'tustin'); % No need
% Linearized 4 machine 2 area test system loaded.
% Initial poweflow from area 1 to area 2 = 540 MW.
% Inputs:   1 - d7 area 1
%           2 - d9 area 2
%           3 - Pdc 
% Ouputs: 1   y1: w_1 - machine speed 1, area 1 (pu)
%         2       w_2 - machine speed 2, area 1 (pu)
%         3       w_3 - machine speed 3, area 1 (pu)
%         4  	  w_4 - machine speed 4, area 1 (pu)
%         5  	  th_7 - phase at load bus, area 1 
%         6   y2: th_9 - phase at load bus, area 2 
%         7  	  V_7 - Voltage at load bus, area 1 
%         8   y3: V_9 -   area 2
%         9       Pac_top - ac power flow top line, area 1
%         10  y4: Pac_bot - ac power flow bottom line, area 2

[A,B,C,D] = ssdata(sys);

% State Index of generators speed
idx_gen = [];
for i = 1:4
idx_gen = [idx_gen, find(C(i,:)==1)]; % index of first generator
end

%% Eienvalues and eigenvectors
[E,W,Cmodes,data] = modal_vectors(A);
disp(['mode=', num2str(E(1)),...
      ' freq=', num2str(data.freq(1)),...
      ' damping=' num2str(data.damping(1))]); 
  
%% Rotate eigenvectors
mode_number = 1;
i = mode_number*2-1;
% V = Cmodes'; % real projected eigenvector;
V = W;
v = V(idx_gen,i);

vpos  = sum(v(real(v)>0));
vneg  = sum(v(real(v)<=0));
v2 = [vneg;vpos];
[~,idx] = max(abs(v2));
ang = angle(v2(idx));
%
v_ = v*exp(-1j*ang);
if real(v_(1))<0
    ang = ang+pi; % Make sure that generator 1 is in positive real direction
end

V = V*exp(-1j*ang);

v = V(idx_gen,i);

if plot_fig
    h=1;
    figureLatex
    compass(v(1),'r'); hold all; 
    compass(v(2),'b'); 
    compass(v(3),'k'); 
    compass(v(4),'m'); 
    % v = 50*V(idx_gen+1,i+1); % Rotor angles
    % compass(v,'m');
end

%% Performance variable
Cz = Cmodes(1:2,:);
G0 = ss(A,B,Cz,0*Cz*B);
Gz = modelred_hsv(G0,2); % <------------- model reduction -----------

if plot_fig
    figure()
    bode(G0,'k'); hold all
    bode(Gz,'r--')
    title('Gz')
end

sys_r.Gz = Gz;

%%
outputs = [6,1,8,10];
G = sys(outputs,:);
sys_r.G = G;

%% th9 measurement
G0 = G(1,:);
Gth9 = modelred_hsv(G0,20); % <------------- model reduction -----------

if plot_fig
    figure()
    bode(G0,'k'); hold all
    bode(Gth9,'r--')
    title('Gth9')
end 

sys_r.Gth9 = Gth9;

%% w1 measurement
G0 = G(2,:);
Gw1 = modelred_hsv(G0,8); % <------------- model reduction -----------

if plot_fig
    figure()
    bode(G0,'k'); hold all
    bode(Gw1,'r--')
    title('Gw1')
end

sys_r.Gw1 = Gw1;



%% V9 measurement
G0 = G(3,:);
GV9 = modelred_hsv(G0,20); % <------------- model reduction -----------

if plot_fig
    figure()
    bode(G0,'k'); hold all
    bode(GV9,'r--')
title('GV9')
end  

sys_r.GV9 = GV9;

%% Pac_bottom measurement 
G0 = G(4,:);
Gac = modelred_hsv(G0,20); % <------------- model reduction -----------

if plot_fig
    figure()
    bode(G0,'k'); hold all
    bode(Gac,'r--')
    title('Gac')
end 

sys_r.Gac = Gac;

%% Reduce all together
Gr = modelred_hsv(G,21);

if plot_fig
    figure()
    bode(G,'k'); hold all
    bode(Gr,'r--')
    title('G0')
end
sys_r.Gr = Gr;

%% Load model
[G,Gr] = load_linear_model(false); % Load linearized model


%% Filtering Sensitivity
% Ouputs: (1-4: w_1-w_1) (5-6: th_7/9) (7-8: V_7/9) (9-10: Pac_top/bot)
Gyw = G(6,1:2); % th9 output
Gzw = Gr.Gz(1,1:2); % mode output oriented to gen speeds

plot_filtering_sensitivity(Gyw,Gzw);

%% Modal analysis
[v,e,w] = modal_analysis(G.A);
w0 = abs(e);

%Observability and controllability
v_obsv = G.C([1,6,8,10],:)*v;
w_ctrb = w*G.B(:,3);

residues=  v_obsv.*w_ctrb; 
% residue gives a metric on controllabilit/observability of input/output
% selections. Note that this is not dimension-less so we can only compare
% similar measurement. That is, we cannot compare voltage measurements to
% phase angle measurements directly.


% Simulation of 100 MW  result in the follwing signal amplitudes 
% (just to get a hint on how to start choosing gains)

% d_load = 100; % MW; - Disturbance size
% d_th9 = 2; % deg; - Resulting system response
% d_w1 = 0.004; % pu
% d_V9 = 0.01; % pu
% d_Pac = 25; % pu

%% Extended system
%  Size of extended system - (nz+nu+ny)x(nd+ny+ny)
%
% (perf. var.)  % | z |   |Gzd  Gzn  Gzu |   |d| % (disturbances)
% (input usage) % |z_u| = | 0    0    I  | * |n| % (measurement noise)
% (measurement) % | y |   |Gyd   I    Gyu|   |u| % (controlled input)

Gy = Gr.Gth9;
% Gy = Gr.Gr(1,:);
Gz = Gr.Gz(1,:);

Gyd = Gy(:,1:2); Gyu = Gy(:,3);
Gzd = Gz(:,1:2); Gzu = Gz(:,3);

[nz,nu] = size(Gzu);
[ny,nd] = size(Gyd);

P0 = [Gzd,          zeros(nz,ny), -Gzu;
      zeros(nu,nd), zeros(nu,ny), -eye(nu);
      Gyd,          eye(ny),      -Gyu];
%
P0 = minreal(P0);

% OBS h2syn cannot deal with pure integrators. 

%% H2 Controller
s = tf('s'); 
% Input weights/scalings
Wi_d = 100; % disturbance amplitude
Wi_n = 1; % noise amplitude
      
% Input weights/scalings
Wi =blkdiag(eye(nd)*Wi_d,...
            eye(ny)*Wi_n,...
            eye(nu));

% Output weights
Wo_z = 40*360; % performance weight  
Wo_u = 1*(s+0.1*w0)/(s+1e-9); % input weight  
Wo =blkdiag(eye(nz)*Wo_z,...
            eye(nu)*Wo_u,....
            eye(ny)); 
        
P = Wo*P0*Wi;  
% P = pade(P,6); % Rational approx of time eventual time delays.
P = minreal(P);
[K2_0,CL,GAM,INFO] = h2syn(P,ny,nu);

K2 = modelred_hsv(K2_0,4); % <-------------------------- Model Reduction

% figure()
bode(K2_0,'k'); hold all
bode(K2,'r--')

L = minreal(Gyu*K2);
S = minreal(inv(eye(ny)+L));

[E,~,~,data] = modal_vectors(S.A,6);
disp(['Iteration ' num2str(i), ' gamma = ', num2str(GAM) ])
disp(['mode=', num2str(E(1)), ', ', num2str(E(3))])
disp(['freq=', num2str(data.freq(1)), ', ', num2str(data.freq(3))])
disp(['damping=', num2str(data.damping(1)), ', ', num2str(data.damping(3))]); 

N2 = lft(P0,K2);

K{2} = K2;

%% PSS style controller
s = tf('s');
Fw = (s/(s+w0/5)); % Wahsout
F0 = s*Fw*(w0*5/(s+w0*5))^2; % Derivative * Washout * Low-pass
GF0 = -Gyu*F0;
F = tunePSS(GF0); % Targets the most poorly damped oscillatory mode.
k = 1.05; % <------- Gain for 10% damping
Kpss = F*F0*k;
% Kpss = F0*k; % Ommit phase comp

L = minreal(Gyu*Kpss);
S = minreal(inv(eye(ny)+L));

[E,~,~,data] = modal_vectors(S.A,6);
disp(['Iteration ' num2str(i), ' gamma = ', num2str(GAM) ])
disp(['mode=', num2str(E(1)), ', ', num2str(E(3))])
disp(['freq=', num2str(data.freq(1)), ', ', num2str(data.freq(3))])
disp(['damping=', num2str(data.damping(1)), ', ', num2str(data.damping(3))]); 

Npss = lft(P0,Kpss);


figure()
step(N2(1:2,1:2),20), hold all
step(Npss(1:2,1:2),20),
step(Gzd,20)

K{1} = ss(Kpss);

%% Root locus
h = 1;
figureLatex
box off
plot([0,0],[-100,100],'k','linewidth',0.5); hold on
plot([-100,100],[0,0],'k','linewidth',0.5);


resolution = 500;
gains = logspace(-1,3,resolution); % <------------ Plot range

Lpss = minreal(Gyu*Kpss);
[R,~] = rlocus(Lpss,gains);
plot(real(R)', imag(R)','color',[1,1,1]*0.7)

resolution = 10;
gains = linspace(0,k,resolution); % <------------ Plot range

% Lpss = minreal(Gyu*Kpss);
[R,~] = rlocus(Lpss,gains);

plot(real(R(:,1)),imag(R(:,1)),'ro'), 
plot(real(R)', imag(R)','r')
plot(real(R(:,end))', imag(R(:,end))','rx')

[~,z] = pzmap(Gyu);
plot(real(z),imag(z),'x','color',[1,1,1]*0.7), 

L2 = minreal(Gyu*K2);
KS2 = minreal(inv(1+L2)*s/(s*0.0001+1));
[p,~] = pzmap(KS2);
plot(real(p),imag(p),'kx'), 

ylim([-2,8.5])
xlim([-2,0.75])
xlabel('Real [rad/s]')
ylabel('Imag [rad/s]')

%% Disturbance response ratio
R1 = minreal([Npss(1,1)/Gzd(1,1), Npss(1,2)/Gzd(1,2)]);
R2 = minreal([N2(1,1)/Gzd(1,1), N2(1,2)/Gzd(1,2)]);

omega = logspace(-1,1.5,500);
[MAG1,~] = bode(R1,omega);
MAG1 = squeeze(MAG1);
[MAG2,~] = bode(R2,omega);
MAG2 = squeeze(MAG2);

h=1/sqrt(2);
doubleColumn=true;
figureLatex
doubleColumn=false;
co = ([0,0,0;0,0,0;1,0,0;1,0,0;]);
set(groot,'defaultAxesColorOrder',co)

semilogx(omega,MAG1(1,:)); hold on
semilogx(omega,MAG1(2,:),'--');

semilogx(omega,MAG2(1,:)); hold on
semilogx(omega,MAG2(2,:),'--');

yline(1,'k')

% legend('M_1','M_2','G_z_d_1')
% yticks(logspace(-4,6,6))
xticks(logspace(-2,2,5));
xlim([min(omega),max(omega)])
ylabel('Amplitude')
box off
xlabel('Angular frequency [rad/s]')
box off
xlim([min(omega),max(omega)])

l = legend('$K_\mathrm{PSS}: \ d_1$', '$K_\mathrm{PSS}: \  d_2$',...
           '$K_\mathrm{H2}: \  d_1$', '$K_\mathrm{H2}: \  d_2$');
        set(l,'Interpreter','LaTeX');
        set(l,'Box','off')
        set(l,'location','best')
        set(l,'FontSize',9)
        set(l,'Orientation','vertical')

%% Extended system, with external w1 measurement
%  Size of extended system - (nz+nu+ny)x(nd+ny+ny)
%
% (perf. var.)  % | z |   |Gzd  Gzn  Gzu |   |d| % (disturbances)
% (input usage) % |z_u| = | 0    0    I  | * |n| % (measurement noise)
% (measurement) % | y |   |Gyd   I    Gyu|   |u| % (controlled input)

Gy = [Gr.Gth9; Gr.Gw1];
Gy = Gr.Gr(1:2,:);
Gz = Gr.Gz(1,:);

s = tf('s'); 
td = 0.2; % ms delay        
Wd = blkdiag(1,exp(-td*s));

Gyd = Gy(:,1:2); Gyu = Gy(:,3);
Gzd = Gz(:,1:2); Gzu = Gz(:,3);

[nz,nu] = size(Gzu);
[ny,nd] = size(Gyd);

P0 = [Gzd,          zeros(nz,ny), -Gzu;
      zeros(nu,nd), zeros(nu,ny), -eye(nu);
      Wd*Gyd,       Wd*eye(ny),   -Wd*Gyu];
%
P0 = minreal(P0);

% OBS h2syn cannot deal with pure integrators. 

% H2 Controller
s = tf('s'); 
% Input weights/scalings
Wi_d = 100; % disturbance amplitude
w1_mult = 60*360;
Wi_n = diag([1,1/w1_mult]); % noise amplitude
      
% Input weights/scalings
Wi =blkdiag(eye(nd)*Wi_d,...
            eye(ny)*Wi_n,...
            eye(nu));

% Output weights
Wo_z = 40*360; % performance weight  
Wo_u = 1*(s+0.1*w0)/(s+1e-9); % input weight  
Wo =blkdiag(eye(nz)*Wo_z,...
            eye(nu)*Wo_u,....
            eye(ny)); 
        
P = Wo*P0*Wi;  
P = pade(P,2); % Rational approx of time eventual time delays.
P = minreal(P);
[K0_w1,CL,GAM,INFO] = h2syn(P,ny,nu);

% K2_w1 = modelred_hsv(K0_w1,17); % <-------------------------- Model Reduction
K2_w1(1) = modelred_hsv(K0_w1(1),7); % <-------------------------- Model Reduction
K2_w1(2) = modelred_hsv(K0_w1(2),5); % <-------------------------- Model Reduction
% 
figure()
bode(K0_w1(1),'k'); hold all
bode(K0_w1(2)/w1_mult,'r'); 
bode(K2_w1(1),'k--'); hold all
bode(K2_w1(2)/w1_mult,'r--'); 
%
L = minreal(Wd*Gyu*K2_w1);
S = minreal(inv(eye(ny)+L));

[E,~,~,data] = modal_vectors(S.A,6);
disp(['Iteration ' num2str(i), ' gamma = ', num2str(GAM) ])
disp(['mode=', num2str(E(1)), ', ', num2str(E(3))])
disp(['freq=', num2str(data.freq(1)), ', ', num2str(data.freq(3))])
disp(['damping=', num2str(data.damping(1)), ', ', num2str(data.damping(3))]); 

N2_w1 = lft(P0,K2_w1);

figure()
step(N2_w1(1:2,1:2),20), hold all
step(N2(1:2,1:2),20), hold all
step(Gzd,20)

K{3} = K2_w1;

%% Extended system, with Pac measurement
%  Size of extended system - (nz+nu+ny)x(nd+ny+ny)
%
% (perf. var.)  % | z |   |Gzd  Gzn  Gzu |   |d| % (disturbances)
% (input usage) % |z_u| = | 0    0    I  | * |n| % (measurement noise)
% (measurement) % | y |   |Gyd   I    Gyu|   |u| % (controlled input)

Gy = [Gr.Gac];
Gz = Gr.Gz(1,:);

Gyd = Gy(:,1:2); Gyu = Gy(:,3);
Gzd = Gz(:,1:2); Gzu = Gz(:,3);

[nz,nu] = size(Gzu);
[ny,nd] = size(Gyd);

P0 = [Gzd,          zeros(nz,ny), -Gzu;
      zeros(nu,nd), zeros(nu,ny), -eye(nu);
      Gyd,          eye(ny),      -Gyu];
%
P0 = minreal(P0);

% OBS h2syn cannot deal with pure integrators. 

% H2 Controller
s = tf('s'); 
% Input weights/scalings
Wi_d = 100; % disturbance amplitude
Wi_n = 1; % noise amplitude
      
% Input weights/scalings
Wi =blkdiag(eye(nd)*Wi_d,...
            eye(ny)*Wi_n,...
            eye(nu));

% Output weights
Wo_z = 40*360; % performance weight  
Wo_u = 1*(s+0.1*w0)/(s+1e-9); % input weight  
Wo =blkdiag(eye(nz)*Wo_z,...
            eye(nu)*Wo_u,....
            eye(ny)); 
        
P = Wo*P0*Wi;  
P = pade(P,2); % Rational approx of time eventual time delays.
P = minreal(P);
[K0_ac,CL,GAM,INFO] = h2syn(P,ny,nu);

K2_ac = modelred_hsv(K0_ac,3); % <-------------------------- Model Reduction
% 
figure()
bode(K0_ac,'k'); hold all
bode(K2_ac,'k--'); 

%
L = minreal(Gyu*K2_ac);
S = minreal(inv(eye(ny)+L));

[E,~,~,data] = modal_vectors(S.A,6);
disp(['Iteration ' num2str(i), ' gamma = ', num2str(GAM) ])
disp(['mode=', num2str(E(1)), ', ', num2str(E(3))])
disp(['freq=', num2str(data.freq(1)), ', ', num2str(data.freq(3))])
disp(['damping=', num2str(data.damping(1)), ', ', num2str(data.damping(3))]); 

N2_ac = lft(P0,K2_ac);

figure()
step(N2_ac(1:2,1:2),20), hold all
step(N2(1:2,1:2),20,'b'), hold all
step(Gzd,20,'r')

K{4} = K2_ac;

%% Extended system, with V9 measurement
%  Size of extended system - (nz+nu+ny)x(nd+ny+ny)
%
% (perf. var.)  % | z |   |Gzd  Gzn  Gzu |   |d| % (disturbances)
% (input usage) % |z_u| = | 0    0    I  | * |n| % (measurement noise)
% (measurement) % | y |   |Gyd   I    Gyu|   |u| % (controlled input)

Gy = [Gr.GV9];
Gz = Gr.Gz(1,:);

Gyd = Gy(:,1:2); Gyu = Gy(:,3);
Gzd = Gz(:,1:2); Gzu = Gz(:,3);

[nz,nu] = size(Gzu);
[ny,nd] = size(Gyd);

P0 = [Gzd,          zeros(nz,ny), -Gzu;
      zeros(nu,nd), zeros(nu,ny), -eye(nu);
      Gyd,          eye(ny),      -Gyu];
%
P0 = minreal(P0);

% OBS h2syn cannot deal with pure integrators. 

% H2 Controller
s = tf('s'); 
% Input weights/scalings
Wi_d = 100; % disturbance amplitude
Wi_n = 1/150; % noise amplitude
      
% Input weights/scalings
Wi =blkdiag(eye(nd)*Wi_d,...
            eye(ny)*Wi_n,...
            eye(nu));

% Output weights
Wo_z = 40*360; % performance weight  
Wo_u = 1*(s+0.1*w0)/(s+1e-9); % input weight  
Wo =blkdiag(eye(nz)*Wo_z,...
            eye(nu)*Wo_u,....
            eye(ny)); 
        
P = Wo*P0*Wi;  
P = pade(P,2); % Rational approx of time eventual time delays.
P = minreal(P);
[K0_V9,CL,GAM,INFO] = h2syn(P,ny,nu);

K2_V9 = modelred_hsv(K0_V9,3); % <-------------------------- Model Reduction
% 
figure()
bode(K0_V9,'k'); hold all
bode(K2_V9,'k--'); 

%
L = minreal(Gyu*K0_V9);
S = minreal(inv(eye(ny)+L));

[E,~,~,data] = modal_vectors(S.A,6);
disp(['Iteration ' num2str(i), ' gamma = ', num2str(GAM) ])
disp(['mode=', num2str(E(1)), ', ', num2str(E(3))])
disp(['freq=', num2str(data.freq(1)), ', ', num2str(data.freq(3))])
disp(['damping=', num2str(data.damping(1)), ', ', num2str(data.damping(3))]); 

N2_V9 = lft(P0,K2_V9);

figure()
step(N2_V9(1:2,1:2),20), hold all
step(N2_ac(1:2,1:2),20), hold all
step(Gzd,20)

K{5} = K2_V9;
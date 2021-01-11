clear all
model = 'four_machine';
kpss= 0.2;

load('POD_controllers_example.mat')

% d_rad=0; %rad/s
% d1_sine = 0; %ampilitude of dsinusoidal disturbance (begins at time 1 sec)
% d2_sine =0;

H = [6.5 6.5 6.175 6.175]*.75;
Tdelay = 0.2;

case_n = 2; % Choose controller
d5_step = -350; %ampilitude of disturbance step (at time 1 sec)
d11_step = 0;

%%
numer = 1;
Ki = zpk(K{numer});

for j = 1:length(Ki.K)
    Ki_ =zpk(Ki.Z{j}, Ki.P{j},Ki.K(j),'DisplayFormat','time constant')
end

%%
simulate_model = false;
if simulate_model
    runs = [];
    load_step_size = 350;
    for dist = 1:2
        if dist == 1
            d5_step = -load_step_size; %ampilitude of disturbance step (at time 1 sec)
            d11_step = 0;
        else
            d5_step = 0; %ampilitude of disturbance step (at time 1 sec)
            d11_step = load_step_size;
        end
        for i = 1:6
            case_n = i-1;
            try
                sim(model)
            catch
                warning('Simulation error');
            end
            runs{i}{dist}.t = t;
            runs{i}{dist}.u = u; % Control input
            runs{i}{dist}.d14 = d14; % Phase angle diff d1-d4
            runs{i}{dist}.d = d; % Load d5,d7,d9,d11
        end
    end
end

%% Plot
% load('runs_example_350.mat')
xlims = [0,20];
ylims = [-10,170];
ylims2 = [-100,100];

load_sgn = -1;

path = 'C:\Users\joakbj\Dropbox\KTH\Forskning\Latex Version_Transactions_review_response\IEEEtran2020\Figures';

%% Plot, case 0 and 1 (PSS-style controller, th9 feedback)
doubleColumn = true;
figureLatex
doubleColumn = false;
co1 = ([0.6,0.6,0.6;0,0,0]);
co2 = ([1,0,0;0.6,0.6,0.6;0,0,0]);
plot_bot_label = true;
plot_top_label = true;
for dist = 1:2
    plot_dist = true;
    for i = 1:2            
        time = runs{i}{dist}.t;                    
        subplot(2,2,dist)
        set(groot,'defaultAxesColorOrder',co1)
        
        if i == 1
            plot(time, runs{i}{dist}.d14, '--'); hold all
        else
            plot(time, runs{i}{dist}.d14); hold all
        end
        
        if plot_top_label
                ylabel({'Rotor angle';' $\delta_1-\delta_4$ [$^\circ$]'})
                plot_top_label = false;
        end
        ylim(ylims)
        xlim(xlims)
        
        subplot(2,2,dist+2)
        set(groot,'defaultAxesColorOrder',co2)
        if plot_dist
            plot(time, load_sgn*(runs{i}{dist}.d(:,1)+runs{i}{dist}.d(:,4))); hold all
            xlabel('Time [s]')
            plot_dist=false; 
        end
        if plot_bot_label
            ylabel('Power [MW]')              
            plot_bot_label = false;
        end
        plot(time, runs{i}{dist}.u); 
        ylim(ylims2)
        xlim(xlims)
        
        if dist==1           
            l = legend('Load loss bus 5');
        elseif dist ==2     
            l = legend('Generation loss bus 11');
        end
        set(l,'Interpreter','LaTeX');
        set(l,'Box','off')
        set(l,'location','southeast')
        set(l,'FontSize',7)
        set(l,'Orientation','vertical')
    end
end

saveFigure = true;
if saveFigure
    saveas(hfig,fullfile(path, 'simPSS'),'epsc');
end

%% Plot, case 0 and 2 (H2 controller, th9 feedback)
doubleColumn = true;
figureLatex
doubleColumn = false;
co1 = ([0.6,0.6,0.6;0,0,0]);
co2 = ([1,0,0;0.6,0.6,0.6;0,0,0]);
plot_bot_label = true;
plot_top_label = true;
for dist = 1:2
    plot_dist = true;
    for i = [1,3]            
        time = runs{i}{dist}.t;                    
        subplot(2,2,dist)
        set(groot,'defaultAxesColorOrder',co1)
        
        if i == 1
            plot(time, runs{i}{dist}.d14, '--'); hold all
        else
            plot(time, runs{i}{dist}.d14); hold all
        end
        
        if plot_top_label
                ylabel({'Rotor angle';' $\delta_1-\delta_4$ [$^\circ$]'})
                plot_top_label = false;
        end
        ylim(ylims)
        xlim(xlims)
        
        subplot(2,2,dist+2)
        set(groot,'defaultAxesColorOrder',co2)
        if plot_dist
            plot(time, load_sgn*(runs{i}{dist}.d(:,1)+runs{i}{dist}.d(:,4))); hold all
            xlabel('Time [s]')
            plot_dist=false; 
        end
        if plot_bot_label
            ylabel('Power [MW]')              
            plot_bot_label = false;
        end
        plot(time, runs{i}{dist}.u); 
        ylim(ylims2)
        xlim(xlims)
        
        if dist==1           
            l = legend('Load loss bus 5');
        elseif dist ==2     
            l = legend('Generation loss bus 11');
        end
        set(l,'Interpreter','LaTeX');
        set(l,'Box','off')
        set(l,'location','southeast')
        set(l,'FontSize',7)
        set(l,'Orientation','vertical')
    end
end

saveFigure = true;
if saveFigure
    saveas(hfig,fullfile(path, 'simH2'),'epsc');
end

%% Plot, case 0 and 3 (H2 controller, th9+w1 feedback)
doubleColumn = true;
figureLatex
doubleColumn = false;
co1 = ([0.6,0.6,0.6;0,0,0;0,0.5,0.7]);
co2 = ([1,0,0;0.6,0.6,0.6;0,0,0;0,0.5,0.7]);
plot_bot_label = true;
plot_top_label = true;
for dist = 1:2
    plot_dist = true;
    for i = [1,3,4]            
        time = runs{i}{dist}.t;                    
        subplot(2,2,dist)
        set(groot,'defaultAxesColorOrder',co1)
        
        if i == 1
            plot(time, runs{i}{dist}.d14, '--'); hold all
        else
            plot(time, runs{i}{dist}.d14); hold all
        end
        
        if plot_top_label
                ylabel({'Rotor angle';' $\delta_1-\delta_4$ [$^\circ$]'})
                plot_top_label = false;
        end
        ylim(ylims)
        xlim(xlims)
        
        subplot(2,2,dist+2)
        set(groot,'defaultAxesColorOrder',co2)
        if plot_dist
            plot(time, load_sgn*(runs{i}{dist}.d(:,1)+runs{i}{dist}.d(:,4))); hold all
            xlabel('Time [s]')
            plot_dist=false; 
        end
        if plot_bot_label
            ylabel('Power [MW]')              
            plot_bot_label = false;
        end
        plot(time, runs{i}{dist}.u); 
        ylim(ylims2)
        xlim(xlims)
        
        if dist==1           
            l = legend('Load loss bus 5');
        elseif dist ==2     
            l = legend('Generation loss bus 11');
        end
        set(l,'Interpreter','LaTeX');
        set(l,'Box','off')
        set(l,'location','southeast')
        set(l,'FontSize',7)
        set(l,'Orientation','vertical')
    end
end

saveFigure = true;
if saveFigure
    saveas(hfig,fullfile(path, 'simH2_th9_plus_w1'),'epsc');
end

%% Plot, case (0 and 4,5) + 1 (H2 controller, Pac and V9 feedback)
doubleColumn = true;
figureLatex
doubleColumn = false;
co1 = ([0.6,0.6,0.6;0,0,0;0,0.5,0.7]);
co2 = ([1,0,0;0.6,0.6,0.6;0,0,0;0,0.5,0.7]);
plot_bot_label = true;
plot_top_label = true;
for dist = 1:2
    plot_dist = true;
    for i = [1,5,6]            
        time = runs{i}{dist}.t;                    
        subplot(2,2,dist)
        set(groot,'defaultAxesColorOrder',co1)
        if i == 1
            plot(time, runs{i}{dist}.d14, '--'); hold all
        else
            plot(time, runs{i}{dist}.d14); hold all
        end
        if plot_top_label
                ylabel({'Rotor angle';' $\delta_1-\delta_4$ [$^\circ$]'})
                plot_top_label = false;
        end
        ylim(ylims)
        xlim(xlims)
        
        subplot(2,2,dist+2)
        set(groot,'defaultAxesColorOrder',co2)
        if plot_dist
            plot(time, load_sgn*(runs{i}{dist}.d(:,1)+runs{i}{dist}.d(:,4))); hold all
            xlabel('Time [s]')
            plot_dist=false; 
        end
        if plot_bot_label
            ylabel('Power [MW]')              
            plot_bot_label = false;
        end
        if i == 1
            plot(time, runs{i}{dist}.u, '-'); 
        else
            plot(time, runs{i}{dist}.u); 
        end
        ylim(ylims2)
        xlim(xlims)
        
        if dist==1           
            l = legend('Load loss bus 5');
        elseif dist ==2     
            l = legend('Generation loss bus 11');
        end
        set(l,'Interpreter','LaTeX');
        set(l,'Box','off')
        set(l,'location','southeast')
        set(l,'FontSize',7)
        set(l,'Orientation','vertical')
    end
end

saveFigure = true;
if saveFigure
    saveas(hfig,fullfile(path, 'simH2_P_and_V'),'epsc');
end

% Identification demo for the polydome model
% Content:
% 1. Data preparation
% 2. Order selection
% 3. Model fitting and validation

clear; close all;

addpath(genpath('./data'))
addpath(genpath('./utilities'))

%% ==== 1. Prepare data ====
% --- 1.1 load data ---
load('Exp1')
load('Exp2')
load('Exp3')
load('Exp4')
load('Exp5')
load('Exp6')
load('Exp7')


%% --- 1.2 plot data ---
for i = 1:7
    Exp = eval(['Exp',mat2str(i)]);
    tExp = Exp.Power.time/86400 + datenum(1970,1,1);
    h=figure;
    hold on
    set(h,'Units','normalized','Position',[0 0 1 .5]); 
    yyaxis left
    plot(tExp ,Exp.Power.values,'b','LineWidth',1)
    yyaxis right
    plot(tExp,Exp.Setpoint.values,'r','LineWidth',1)
    plot(tExp,Exp.InsideTemp.values,'g','LineWidth',1)
    plot(tExp,Exp.SupplyTemp.values,'y','LineWidth',1.5)
    title('Meteorological data','FontSize',18)
    legend({'power','setpoint','room temperature','supply temperature'},'FontSize',18)
    ylabel('[¡ãC]','FontSize',18)
    datetick('x','mm/dd/yy HH:MM');
end

%% --- 1.2 Create iidata object ---
% Sampled data: Ts = 5 min; Wanted data: Ts = 15 min
T_old = 60*5;
T_new = 60*15;

Exp1 = create_iddata( Exp1,  T_old, T_new);	
Exp2 = create_iddata( Exp2,  T_old, T_new);
Exp3 = create_iddata( Exp3,  T_old, T_new);
Exp4 = create_iddata( Exp4,  T_old, T_new);
Exp5 = create_iddata( Exp5,  T_old, T_new);
Exp6 = create_iddata( Exp6,  T_old, T_new);
Exp7 = create_iddata( Exp7,  T_old, T_new);
% 	Exp8 = create_iddata( Exp8,  T_old, T_new);
%
Data_iden = merge(Exp7, Exp6, Exp4, Exp2);
Data_val = merge(Exp3, Exp5, Exp1);


% Detrend each dataset: y(i)-mean(y), u(i)-mean(u)
% In real exmperiments, need to estimate trend by latest data
[Data_iden_de, T_iden] = detrend(Data_iden);
[Data_val_de, T_val] = detrend(Data_val);

% % Compute trend of the identification datasets: mean(y), mean(u)
% % Detrend all the datasets by this trend
% % However, it shows worse performance than above detrending
Exp_iden = [Exp1;Exp2;Exp3;Exp4];
[~, T_real] = detrend(Exp_iden);
Data_iden_real = detrend(Data_iden, T_real);
Data_val_real = detrend(Data_val, T_real);
%% ===== 2. order selection =====
% --- 2.1 By arx model and loss function ---
nk = 1;
n = 10;
arx_loss = zeros(n,1);

for i = 1:n
    na = i;nb = i;
    arx_model = arx(Data_iden_de,[na [nb,nb,nb] [nk,nk,nk]]);
    arx_loss(i) = arx_model.EstimationInfo.LossFcn;
end
stem(1:n,arx_loss);
title('Loss function evolution')
xlabel('system order');ylabel('Loss');
% The losses are not so different
%% --- 2.2 By armax model and zero/pole cancellation ---
for i = 2:4
na = i;nb = i;nc = i;
ArmaxSys = armax(Data_iden_de,[na [nb,nb,nb] nc [nk,nk,nk]]);
figure;
h=iopzplot(ArmaxSys);axis([-1 1 -1 1]);
str_i = strcat('Assumed Model order : ',num2str(i));title(str_i);
showConfidence(h,2);
end
% zero/pole cancellation when n >= 3;
% so the order of the system could be 2
%% --- 2.3 delay --- 
% function 'delayest' evaluate the loss function of arx model, for various choices of delays
for i = 2:5
    na = i; nb = i;
    nd_min = 1; nd_max = 5;
    delay = delayest(Data_iden_de, na, nb, nd_min,nd_max) 
end

% Nonparametric impulse estimation
FIRModel = impulseest(Data_iden_de);
h = impulseplot(FIRModel);
showConfidence(h,3)
% By impulse repsonse, no delay

%% --- 2.4 Comparation with matlab's function "selstruc" ---
nn = struc(1:7, 1:7,1:7,1:7, 1,1,1);
V = arxstruc(Data_iden_de,Data_val_de,nn);
selstruc(V); %invoke selstruc in an interactive mode
% Close the iterative mode, matlab may still run, use "ctrl + c" in command window.
% by best fit in selstruc
% na = 5£¬ nb = [6,2,4], nk = [1 1 1]

%% --- 2.5 By state space model ---
nx = 1:10;
opt = ssestOptions('SearchMethod','auto');
opt.Focus = 'prediction';

% An automatically generated plot shows the Hankel singular values for
% models of the orders specified by nx 
% States with relatively small Hankel singular values can be safely discarded.
sys = ssest(Data_iden_de,nx, 'Ts', T_new, opt);
% By this function
% n = 3

% By impulse repsonse, no delay
m = ssest(Data_iden_de, 3,'Ts', T_new, opt);
showConfidence(impulseplot(m),3)


%% ==== 3. Model fitting and validation ====
% 3.1 --- Comparation of different detrend method by arx and ss model ---
nx = 3;
opt = ssestOptions('SearchMethod','auto');
% 'simulation': use intial y and measured u to estimate y; 
% "prediction": use past measured y and measured u to estimate y;
% if we need long horizon length in MPC, choose 'simulation'
opt.Focus = 'prediction'; 
opt.N4Horizon = 6*3600/T_new;

% Derend 1: prediction with 10 steps
pred_step = 10;
m1 = arx(Data_iden_de,[5 [6 2 4] [1 1 1]]);
[m2, X0] = ssest(Data_iden_de, nx, 'Ts', T_new, opt);
[~,fit,~] = compare(Data_iden_de,m1,m2,pred_step)
[~,fit,~] = compare(Data_val_de,m1,m2,pred_step)

% Derend 2: prediction with 10 steps
pred_step = 10;
m3 = arx(Data_iden_real,[5 [6 2 4] [1 1 1]]);
[m4, X0] = ssest(Data_iden_real, nx, 'Ts', T_new, opt);
[~,fit,~] = compare(Data_iden_real,m3,m4,pred_step)
[~,fit,~] = compare(Data_val_real,m3,m4,pred_step)

% Plot: prediction vs measured
% 'Data_val_de' contains several datasets
pred_step = 10;
figure
compare(Data_val_de(:,:,:,1),m1,m2,pred_step)
figure
compare(Data_val_de(:,:,:,2),m1,m2,pred_step)
figure
compare(Data_val_de(:,:,:,3),m1,m2,pred_step)
%% 3.2 --- Comparation of different order of ss model ---

opt = ssestOptions('SearchMethod','auto');
opt.Focus = 'prediction'; 
opt.N4Horizon = 6*3600/T_new;

nx = 3;
[m1, X0] = ssest(Data_iden_de, nx, 'Ts', T_new, opt);
nx = 4;
[m2, X0] = ssest(Data_iden_de, nx, 'Ts', T_new, opt);
nx = 5;
[m3, X0] = ssest(Data_iden_de, nx, 'Ts', T_new, opt);

pred_step = 10;
[~,fit,~] = compare(Data_iden_de,m1,m2,m3,pred_step)
[~,fit,~] = compare(Data_val_de,m1,m2,m3,pred_step)
% save('./model_ss','m1')
%% 
% Plot: prediction vs measured
% 'Data_val_de' contains several datasets
pred_step = 10;
figure
compare(Data_val_de(:,:,:,1),m1,m2,m3,pred_step)
figure
compare(Data_val_de(:,:,:,2),m1,m2,m3,pred_step)
figure
compare(Data_val_de(:,:,:,3),m1,m2,m3,pred_step)

%% Correlation Test
% whiteness only for structures with a noise model
% ARX,ARMAX,BJ,SS,but not for OE
figure
subplot(3,1,1)
resid(Data_val_de,m1);legend('ss_3');
subplot(3,1,2)
resid(Data_val_de,m2);legend('ss_4');
subplot(3,1,3)
resid(Data_val_de,m3);legend('ss_5');



% Identification demo for the polydome model
% Content:
% 1. Data preparation
% 2. Order selection
% 3. Model fitting and validation
% (Note: no detrending also shows good results)

clear; close all;

addpath(genpath('./data_07'))
addpath(genpath('./utilities'))

%% ==== 1. Prepare data ====
% --- 1.1 load data ---
load('exp1_15min')
load('exp2_15min')
load('exp3_15min')
load('exp4_15min')



%% --- 1.2 plot data ---
for i = 1:4
    exp = eval(['exp',mat2str(i)]);
    tExp = exp.power.time + 2/24;
    h=figure;
    hold on
    set(h,'Units','normalized','Position',[0 0 1 .5]); 
    yyaxis left
    plot(tExp ,exp.power.value,'b','LineWidth',1)
    yyaxis right
%     plot(tExp,exp.setpoint_winter.value/10.0,'r','LineWidth',1)
    plot(tExp,exp.air_temp.value,'r','LineWidth',1)
    plot(tExp,exp.sensor_temp.value,'g','LineWidth',1)
    plot(tExp,exp.supply_temp.value/10.0,'y','LineWidth',1.5)
    title('Meteorological data','FontSize',18)
    legend({'power','air temperature','room temperature','supply temperature'},'FontSize',18)
    ylabel('[¡ãC]','FontSize',18)
    datetick('x','mm/dd/yy HH:MM');
end

%% --- 1.2 Create iidata object ---
% Sampled data: Ts = 5 min; Wanted data: Ts = 15 min
T_old = 60*15;
T_new = 60*15;

exp1 = create_iddata( exp1,  T_old, T_new);	
exp2 = create_iddata( exp2,  T_old, T_new);
exp3 = create_iddata( exp3,  T_old, T_new);
exp4 = create_iddata( exp4,  T_old, T_new);
% exp5 = create_iddata( exp5,  T_old, T_new);


%%
data_iden = merge(exp1,exp3,exp4);
data_val = merge(exp2);

% Detrend each dataset: y(i)-mean(y), u(i)-mean(u)
% In real exmperiments, need to estimate trend by latest data
[data_iden_de, trend_iden] = detrend(data_iden);
[data_val_de, trend_val] = detrend(data_val);

% % Compute trend of the identification datasets: mean(y), mean(u)
% % Detrend all the datasets by this trend
% % However, it shows worse performance than above detrending
exp_iden = [exp1;exp3;exp4];
[~, trend_real] = detrend(exp_iden);
data_iden_real = detrend(data_iden, trend_real);
data_val_real = detrend(data_val, trend_real);
%% ===== 2. order selection =====
% --- 2.1 By arx model and loss function ---
nk = 1;
n = 10;
arx_loss = zeros(n,1);

for i = 1:n
    na = i;nb = i;
    arx_model = arx(data_iden_de,[na [nb,nb,nb] [nk,nk,nk]]);
    arx_loss(i) = arx_model.EstimationInfo.LossFcn;
end
figure
stem(1:n,arx_loss);
title('Loss function evolution')
xlabel('system order');ylabel('Loss');
% Choose n = 3 or 4
%% --- 2.2 By armax model and zero/pole cancellation ---
for i = 2:4
na = i;nb = i;nc = i;
ArmaxSys = armax(data_iden_de,[na [nb,nb,nb] nc [nk,nk,nk]]);
figure;
h=iopzplot(ArmaxSys);axis([-1 1 -1 1]);
str_i = strcat('Assumed Model order : ',num2str(i));title(str_i);
showConfidence(h,2);
end
% zero/pole cancellation when n >= 3;
% so the order of the system could be 2
%% --- 2.3 delay --- 
% function 'delayest' evaluate the loss function of arx model, for various choices of orders
for i = 2:5
    na = i; nb = i;
    nd_min = 1; nd_max = 5;
    delay = delayest(data_iden_de, na, nb, nd_min, nd_max) 
end

% Nonparametric impulse estimation
FIRModel = impulseest(data_iden_de);
h = impulseplot(FIRModel);
showConfidence(h,2)
ylim([-1e-4,1e-4])
% By impulse repsonse, no delay

%% --- 2.4 Comparation with matlab's function "selstruc" ---
nn = struc(1:7, 1:7,1:7,1:7, 1,1,1);
V = arxstruc(data_iden_de,data_val_de,nn);
selstruc(V); %invoke selstruc in an interactive mode
% Close the iterative mode, matlab may still run, use "ctrl + c" in command window.
% by AIC in selstruc
% na = 1£¬ nb = [6,1,5], nk = [1 1 1]

%% --- 2.5 By state space model ---
nx = 1:10;
opt = ssestOptions('SearchMethod','auto');
opt.Focus = 'prediction';

% An automatically generated plot shows the Hankel singular values for
% models of the orders specified by nx 
    % States with relatively small Hankel singular values can be safely discarded.
sys = ssest(data_iden_de,nx, 'Ts', T_new, opt);
% By this function
% n = 3

% By impulse repsonse, no delay
m = ssest(data_iden_de, 3,'Ts', T_new, opt);
figure
showConfidence(impulseplot(m),3)
ylim([-1e-4,1e-4])


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
ss3 = arx(data_iden_de,[6 [6 1 5] [1 1 1]]);
[ss4, X0] = ssest(data_iden_de, nx, 'Ts', T_new, opt);
[~,fit,~] = compare(data_iden_de,ss3,ss4,pred_step)
[~,fit,~] = compare(data_val_de,ss3,ss4,pred_step)

% Derend 2: prediction with 10 steps
pred_step = 10;
ss5 = arx(data_iden_real,[1 [6 1 5] [1 1 1]]);
[m4, X0] = ssest(data_iden_real, nx, 'Ts', T_new, opt);
[~,fit,~] = compare(data_iden_real,ss5,m4,pred_step)
[~,fit,~] = compare(data_val_real,ss5,m4,pred_step)

% Plot: prediction vs measured
% 'Data_val_de' contains several datasets
pred_step = 10;
figure
compare(data_val_de(:,:,:,1),ss3,ss4,pred_step)
figure
compare(data_val_de(:,:,:,2),ss3,ss4,pred_step)

%% 3.2 --- Comparation of different order of ss model ---

opt = ssestOptions('SearchMethod','auto');
opt.Focus = 'simulation'; 
opt.N4Horizon = 6*3600/T_new;

nx = 3;
[ss3, X0] = ssest(data_iden_de, nx, 'Ts', T_new, opt);
nx = 4;
[ss4, X0] = ssest(data_iden_de, nx, 'Ts', T_new, opt);
nx = 5;
[ss5, X0] = ssest(data_iden_de, nx, 'Ts', T_new, opt);

pred_step = 36;
[~,fit,~] = compare(data_iden_de,ss3,ss4,ss5,pred_step)
[~,fit,~] = compare(data_val_de,ss3,ss4,ss5,pred_step)
% order >3 does not differ obviously from order 3  
% save('./model/model_ss3_2','m2')
%% 
% Plot: prediction vs measured
% 'Data_val_de' contains several datasets
pred_step = 96;
figure
compare(data_val_de(:,:,:,1),ss3,ss4,ss5,pred_step)
h = legend('Validation data', 'SS3','SS4','SS5');
set(h,'fontsize',30)
% figure
% compare(data_val_de(:,:,:,1),m1,m2,m3,pred_step)

%% Correlation Test
% whiteness only for structures with a noise model
% ARX,ARMAX,BJ,SS,but not for OE
figure
subplot(3,1,1)
resid(data_val_de,ss3);legend('ss_3');
subplot(3,1,2)
resid(data_val_de,ss4);legend('ss_4');
subplot(3,1,3)
resid(data_val_de,ss5);legend('ss_5');



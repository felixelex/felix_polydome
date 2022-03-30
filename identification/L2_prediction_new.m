clear 
clc
%% ==== 2. Use old Polydome data ===
% clear; close all;
addpath('./data')
addpath('./utilities')
addpath(genpath('../DeePC_bilevel'))

% --- 2.1 load data ---
load('exp11_15min_cool')
load('exp13_15min_cool')
exp_iden = exp11;
exp_val = exp13;
t_exp = exp_iden.power.time;
h=figure;
hold on
set(h,'Units','normalized','Position',[0 0 1 .5]); 
yyaxis left
plot(t_exp ,exp_iden.power.value,'b','LineWidth',1)
yyaxis right
plot(t_exp,exp_iden.air_temp.value,'r','LineWidth',1)
plot(t_exp,exp_iden.sensor_temp.value,'g','LineWidth',1)
plot(t_exp,exp_iden.supply_temp.value/10.0,'y','LineWidth',1.5)
plot(t_exp,exp_iden.solar_GHI.value/50.0,'y','LineWidth',1.5)
title('Meteorological data','FontSize',18)
legend({'power','air temperature','room temperature','supply temperature'},'FontSize',18)
ylabel('[��C]','FontSize',18)
datetick('x','mm/dd/yy HH:MM');

T_old = 60*15;
T_new = 60*15;
exp_iden = create_iddata( exp_iden,  T_old, T_new);
exp_val = create_iddata( exp_val,  T_old, T_new);

% [exp_iden, Trend] = detrend(exp_iden);
% [exp_val] = detrend(exp_val); %[Exp_val] = detrend(Exp_val,Trend);
nu = 3;
ny = 1;

% --- 2.2 Build Hankel matrix ---
% Behavioural system: /B/: if there is a representation by ss system (A,B,C,D)
% Its order by n(/B/). 
% Its lag by l(/B/).
% N_init >= l(/B/)
load('./model/model_ss3_2.mat')
% Rstimate l(/B/) by observability matrix
C = m1.C; A = m1.A;
CA = C;
Ob = [CA];
nx = size(A,1);
for i = 2:10
    CA= CA*A;
    Ob = [Ob;CA];
    if rank(Ob)>=nx
        i
        break;
    end
end

N_ini = 10;
N_pred = 10;
T = 200; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1


% Define the weights for g vector: L1 norm or adaptive term
diag_vector = zeros(T-N_ini-N_pred+1,1);
for i = 1:T-N_ini-N_pred+1
    diag_vector(i) = 10 - 9/(T-N_ini-N_pred+1)*i;
end
Qg = diag(diag_vector);

% Define the weights for slack variables
Q_slack = 50;

% load Hankel matrix
load('hankel_cool_prediction_200.mat');
Hankel_U = Hankel_uw_cool;%(:,101:end);
Hankel_Y = Hankel_y_cool;%(:,101:end);
Hankel_y_past = Hankel_Y(1:ny*(N_ini+1), :);
Hankel_y_future = Hankel_Y(ny*(N_ini+1)+1:end, :);


Hankel_U_old = Hankel_U;
Hankel_Y_old = Hankel_Y;
%% --- 2.2 Prediction ---
% Build y, u vector for prediction
y_cl(:,T+1) = 0;
u_cl(:,T+1) = [0;0;0];
y_cl = [y_cl, exp_val.y'];
u_cl = [u_cl, exp_val.u'];
T_tot = size(y_cl,2); % total steps for simulation
%%
% Matrix for recording
y_pred = zeros(T_tot, N_pred); 

for t = T+N_ini+2:T_tot-N_pred
    u_ini_pred = reshape(u_cl(:,t-N_ini:t+N_pred-1), [(N_ini+N_pred)*nu,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]); 

    var_slack = sdpvar(ny*(N_ini+1), 1);
    var_g = sdpvar(T-N_ini-N_pred+1, 1);

    cost = 0;
    constraints = [];

    constraints = [constraints;
                   [Hankel_U;Hankel_y_past]*var_g == [u_ini_pred; y_ini]];           %  +var_slack  
    cost = cost+ var_slack'*Q_slack*var_slack + var_g'*Qg*var_g; 
%     cost = cost+ var_slack'*Q_slack*var_slack + norm(Qg*var_g,1); 

    options = sdpsettings('verbose',0,'solver', 'quadprog','gurobi.TimeLimit', 10);
    optimize(constraints, cost, options);   
    t
    y_pred(t,:) = reshape(Hankel_y_future*double(var_g),[N_pred,ny]);
    var_slack
    [Hankel_U;Hankel_y_past]*var_g - [u_ini_pred; y_ini+var_slack]    
    
     % Update Hankel matrix by latest data
    if mod(t,10)==0 && t-T >= N_ini + N_pred + 1 
        T_tem = t-T-1;
        if T_tem >T
            T_tem = T;
        end
          
        N_total = N_ini + N_pred;
        Hankel_col = T_tem - N_total + 1;
        Hankel_U_new = compute_Hankel_matrix(u_cl(:,t-T_tem:t-1),nu,N_total, Hankel_col);
        Hankel_U = [Hankel_U_old(:,Hankel_col+1:end), Hankel_U_new];
        
        N_total = N_ini + N_pred + 1;
        Hankel_col = T_tem - N_total + 2;
        Hankel_Y_new = compute_Hankel_matrix(y_cl(:,t-T_tem:t),ny,N_total, Hankel_col);
        Hankel_Y = [Hankel_Y_old(:,Hankel_col+1:end), Hankel_Y_new];
        
        Hankel_y_past = Hankel_Y(1:ny*(N_ini+1), :);
        Hankel_y_future = Hankel_Y(ny*(N_ini+1)+1:end, :);   

%         N_total = N_ini + N_pred + nx;
%         Hankel_col = T - N_total +1 ;
%         % Need: u is persistently exciting of order is N_ini + N_pred + nx
%         Hankel_UW_check = compute_Hankel_matrix(u_cl(:,t-T+1:t), nu, N_total, Hankel_col);
%         if rank(Hankel_UW_check)== nu*(N_ini + N_pred + nx)  
%             1;
%         else
%             warning('Exciting of order of Hu is samller than N_ini + N_pred + nx')
%         end
    end    
end
%% --- 2.4 Plot result ---
% Choose some time
t = 220;
figure
hold on
yyaxis left
a = plot(y_pred(t,:),'b');
aa = plot(y_cl(:,t+1:t+N_pred),'--r');
yyaxis right
aaa = plot(y_pred(t,:)-y_cl(:,t+1:t+N_pred),'-*k');

h = legend([a,aa,aaa], 'Prediction', 'Real', 'Difference');
set(h,'fontsize',10, 'interpreter', 'latex')
%%
% Collect all the N_pred_want step prediction
N_pred_want = 10; % N_pred_want<=N_pred
y_pred_N = y_pred(T+N_ini+2:end-N_pred,N_pred_want);
y_cl_N = y_cl(:,T+N_ini+N_pred_want+2:end-(N_pred-N_pred_want))';
u_cl_N = u_cl(:,T+N_ini+N_pred_want+2:end-(N_pred-N_pred_want))';
save('test.mat','y_pred_N','y_cl_N')
% figure
% hold on
% yyaxis left
% a = plot(y_pred_N,'b');
% aa = plot(y_cl_N,'--r');
% yyaxis right
% aaa = plot(y_pred_N-y_cl_N,'-*k');
% 
% h = legend([a,aa,aaa], 'N_{pred}-th step Prediction', 'Real', 'Difference');
% set(h,'fontsize',10)

%
% Use "compare"

iddata_real = iddata(y_cl_N,u_cl_N,T_new);
iddata_pred = iddata(y_pred_N,[],T_new);
figure
deepc = iddata_pred;
ss_model= m1;
% compare(iddata_real, deepc,ss_model,N_pred_want);
compare(iddata_real, deepc,N_pred_want);
fit = goodnessOfFit(y_pred_N, y_cl_N, 'NRMSE')
iddata_pred = iddata(y_pred_N,[],T_new);
% figure
% compare(iddata_real, iddata_pred);
% datetick('x','mm/dd/yy HH:MM');
% Basic deepc prediction
% Content:
% 1. Use a linear model
% 2. Use old Polydome data: bad performance

clc
clear all
addpath(genpath('../DeePC_bilevel'))
%% ==== 1. Use a linear model ===
% --- 1.1 Create necessary elements ---
% system definition
% A = [1 0.1; 0 1];
A = [1 0.1; 0 0.5];
B = eye(2);%[0.5; 1]; 
C = [1, 0];
Q = 1;R = 1;
nu = size(B, 2);
nx = size(A, 1);
ny = size(C, 1);

mysys = LinearSystem(A, B, C);

N_pred = 10;
N_ini = 3;
T = 50; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1
T_tot = 70; % total steps for simulation

% Simulate a trajectory by random input
x_cl = zeros(nx,T_tot); y_cl = zeros(ny,T_tot);
u_cl = 3*rand(nu,T_tot)-1.5;
x_cl(:, 1) = [0; 0];
y_cl(:, 1) = C*x_cl(:, 1);
for t = 1:T_tot-1
    [x_cl(:, t+1), y_cl(:, t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t));
end

% Build the Henkal matrix 
disp('Computing Hankel...')

N_total = N_ini + N_pred;
Hankel_col = T - N_total + 1;
Hankel_U = compute_Hankel_matrix(u_cl(:,1:T),nu,N_total, Hankel_col);

N_total = N_ini + N_pred + 1;
Hankel_col = T - N_total + 2;
Hankel_Y = compute_Hankel_matrix(y_cl(:,1:T+1),ny,N_total, Hankel_col);

% Need: u is persistently exciting of order is N_ini + N_pred + nx
N_total = N_ini + N_pred + nx;
Hankel_col = T - N_total +1 ;
Hankel_U_check = compute_Hankel_matrix(u_cl(:,1:T),nu,N_total, Hankel_col); 

% Check if the past u trajectory is persistently exciting of order N_ini + N_pred + nx
if rank(Hankel_U_check)== nu*(N_ini + N_pred + nx) 
    disp('Hankel rank is ok')
else
    warning('Exciting of order of Hu is samller than N_ini + N_pred + nx')
end

Hankel_y_past = Hankel_Y(1:ny*(N_ini+1), :);
Hankel_y_future = Hankel_Y(ny*(N_ini+1)+1:end, :);

% Matrix for recording
y_pred = zeros(T_tot, N_pred); 

%% --- 1.2 Prediction ---
% Hankel_p*g == [uini;yini]
% If A is a rectangular m-by-n matrix with m ~= n, and B is a matrix with m rows, 
% then A\B returns a least-squares solution to the system of equations A*x= B.

for t = T+1:T_tot-N_pred
    u_ini_pred = reshape(u_cl(:,t-N_ini:t+N_pred-1), [(N_ini+N_pred)*nu,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]); 
    g = [Hankel_U;Hankel_y_past]\[u_ini_pred;y_ini];

%     [Hankel_U;Hankel_y_past]*g-[u_ini_pred;y_ini] % Almost 0

    y_pred(t,:) = reshape(Hankel_y_future*g,[N_pred,ny]);
end

%% --- 1.3 Plot result ---
% Choose some time
t = 58;
figure
hold on
yyaxis left
a = plot(y_pred(t,:),'b');
aa = plot(y_cl(:,t+1:t+N_pred),'--r');
yyaxis right
aaa = plot(y_pred(t,:)-y_cl(:,t+1:t+N_pred),'-*k');

h = legend([a,aa,aaa], 'Prediction', 'Real', 'Difference');
set(h,'fontsize',10)
%%
% Collect all the N_pred-th step prediction
y_pred_N = y_pred(T+1:end-N_pred,end);
y_cl_N = y_cl(:,T+N_pred+1:end)';
figure
hold on
yyaxis left
a = plot(y_pred_N,'b');
aa = plot(y_cl_N,'--r');
yyaxis right
aaa = plot(y_pred_N-y_cl_N,'-*k');

h = legend([a,aa,aaa], 'N_{pred}-th step Prediction', 'Real', 'Difference');
set(h,'fontsize',10)

% Use "compare"
iddata_real = iddata(y_cl_N,[]);
iddata_pred = iddata(y_pred_N,[]);
figure
compare(iddata_real, iddata_pred);
%% ==== 2. Use old Polydome data ===
clear; close all;
addpath('./data')
addpath('./utilities')

% --- 2.1 load data ---
load('exp1')
exp = exp1;
t_exp = exp.power.time;
h=figure;
hold on
set(h,'Units','normalized','Position',[0 0 1 .5]); 
yyaxis left
plot(t_exp ,exp.power.value,'b','LineWidth',1)
yyaxis right
plot(t_exp,exp.air_temp.value,'r','LineWidth',1)
plot(t_exp,exp.sensor_temp.value,'g','LineWidth',1)
plot(t_exp,exp.supply_temp.value/10.0,'y','LineWidth',1.5)
title('Meteorological data','FontSize',18)
legend({'power','air temperature','room temperature','supply temperature'},'FontSize',18)
ylabel('[¡ãC]','FontSize',18)
datetick('x','mm/dd/yy HH:MM');

T_old = 60*5;
T_new = 60*5;
exp = create_iddata( exp,  T_old, T_new);
[exp, trend] = detrend(exp);
nu = 3;
ny = 1;
y_cl = exp.y';
u_cl = exp.u';
% --- 2.2 Build Hankel matrix ---
% Behavioural system: /B/: if there is a representation by ss system (A,B,C,D)
% Its order by n(/B/). 
% Its lag by l(/B/).
% N_init >= l(/B/)
load('./model/model_ss3.mat')
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
T = 300; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1
T_tot = size(y_cl,2); % total steps for simulation
%%

% Build the Henkal matrix 
disp('Computing Hankel...')

N_total = N_ini + N_pred;
Hankel_col = T - N_total + 1;
Hankel_U = compute_Hankel_matrix(u_cl(:,1:T),nu,N_total, Hankel_col);

N_total = N_ini + N_pred + 1;
Hankel_col = T - N_total + 2;
Hankel_Y = compute_Hankel_matrix(y_cl(:,1:T+1),ny,N_total, Hankel_col);

% Need: u is persistently exciting of order is N_ini + N_pred + nx
N_total = N_ini + N_pred + nx;
Hankel_col = T - N_total +1 ;
Hankel_U_check = compute_Hankel_matrix(u_cl(:,1:T),nu,N_total, Hankel_col); 

% Check if the past u trajectory is persistently exciting of order N_ini + N_pred + nx
if rank(Hankel_U_check)== nu*(N_ini + N_pred + nx) 
    disp('Hankel rank is ok')
else
    warning('Exciting of order of Hu is samller than N_ini + N_pred + nx')
end

Hankel_y_past = Hankel_Y(1:ny*(N_ini+1), :);
Hankel_y_future = Hankel_Y(ny*(N_ini+1)+1:end, :);

% Matrix for recording
y_pred = zeros(T_tot, N_pred); 

%% --- 2.3 Prediction ---
% Hankel_p*g == [uini;yini]
% If A is a rectangular m-by-n matrix with m ~= n, and B is a matrix with m rows, 
% then A\B returns a least-squares solution to the system of equations A*x= B.

for t = T+1:T_tot-N_pred 
              
    u_ini_pred = reshape(u_cl(:,t-N_ini:t+N_pred-1), [(N_ini+N_pred)*nu,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]); 
    g = [Hankel_U;Hankel_y_past]\[u_ini_pred;y_ini];

%     [Hankel_U;Hankel_y_past]*g-[u_ini_pred;y_ini] % Almost 0

    y_pred(t,:) = reshape(Hankel_y_future*g,[N_pred,ny]);
    
     % Update Hankel matrix by latest data
    if 0 % mod(t,100)==0
        N_total = N_ini + N_pred;
        Hankel_col = T - N_total + 1;
        Hankel_U = compute_Hankel_matrix(u_cl(:,t-T:t-1),nu,N_total, Hankel_col);
        
        N_total = N_ini + N_pred + 1;
        Hankel_col = T - N_total + 2;
        Hankel_Y = compute_Hankel_matrix(y_cl(:,t-T:t),ny,N_total, Hankel_col);
        Hankel_y_past = Hankel_Y(1:ny*(N_ini+1), :);
        Hankel_y_future = Hankel_Y(ny*(N_ini+1)+1:end, :);            

        N_total = N_ini + N_pred + nx;
        Hankel_col = T - N_total +1 ;
        % Need: u is persistently exciting of order is N_ini + N_pred + nx
        Hankel_UW_check = compute_Hankel_matrix(u_cl(:,t-T+1:t), nu, N_total, Hankel_col);
        if rank(Hankel_UW_check)== nu*(N_ini + N_pred + nx)  
            1;
        else
            warning('Exciting of order of Hu is samller than N_ini + N_pred + nx')
        end
    end
        
end
%% --- 2.4 Plot result ---
% Choose some time
t = 301;
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
% Collect all the N_pred-th step prediction
y_pred_N = y_pred(T+1:end-N_pred,end);
y_cl_N = y_cl(:,T+N_pred+1:end)';
u_cl_N = u_cl(:,T+N_pred+1:end)';
figure
hold on
yyaxis left
a = plot(y_pred_N,'b');
aa = plot(y_cl_N,'--r');
yyaxis right
aaa = plot(y_pred_N-y_cl_N,'-*k');

h = legend([a,aa,aaa], 'Npred step Prediction', 'Real', 'Difference');
set(h,'fontsize',10, 'interpreter', 'latex')
%%
% Use "compare"
iddata_real = iddata(y_cl_N,u_cl_N,T_new);
iddata_pred = iddata(y_pred_N,[],T_new);
figure
compare(iddata_real, iddata_pred);
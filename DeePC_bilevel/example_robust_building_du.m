clc
clear all
addpath('utils','-end')
addpath('src','-end')
rng(124)
%% system definition
% The system includes some time-varying terms
T_tot = 72*2+60; % total steps for simulation
T_day = 72; % one-day steps
num_day = floor(T_tot/T_day)+1 ; % one more day for parameters in prediction
num_T = 2; % date to start deepc
T_start = num_T*T_day +1 ; % time to start deepc

% Time length choice: 
N_ini = 6;
N_pred = 6;
T = 144; % T >= (nu + nw + 1)*(N_ini + N_pred + nx) - 1

% Real system dynamics
A_tem = [0.8011  0.0541  0.0707 ; 
        0.1293  0.8135  0.0055 ;
        0.0989  0.0032  0.7541 ];       
A = @(t) A_tem + 0.025*sin(2*pi*t/T_day)*eye(length(A_tem));
B = [0.07;0.006;0.004];
E = 1e-3*[22.2170 1.7912 42.2123;
          1.5376 0.6944 2.9214;
          103.1813 0.1032 196.0444;];
C = [1 0 0]; 
nu = size(B, 2);
nx = size(A_tem , 1);
ny = size(C, 1);
nw = size(E, 2);

% Matrix in stage cost
Q = 0%10 %0;%10*eye(1);
R = 0.01%0.01 %1;%0.01;

% Reference
r = repmat([20*ones(1,24), 22*ones(1,30), 20*ones(1,18)],1,num_day);%24*ones(1,T);%

% Constraints
Hy = [eye(1);-eye(1)];
for i = 1:T_tot+T_day
     if mod(i,T_day) <=24
         by = [30; -18];
     elseif  mod(i,T_day)<=54
         by = [26; -20];
     else
         by = [30; -18];
     end
     Y{i} = Polyhedron('A', Hy, 'b', by);
end
val_u = 30.0; 
U = Polyhedron([val_u; 0]);

% %%
% Get disturbance for initial trajectory
var_w_y = 0.0;
[w_pred, w_real,w_y,w_rand,W] = get_ini_dist(T_day,num_day,num_T, var_w_y);
% [w_pred, w_real,w_y,W] = get_disturbance(3,T_day,-2,2, var_w_y);
[w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,num_day,num_T, T_day, var_w_y);
%%
val_u = 2; 
dU = Polyhedron([val_u; 0]);
% Define the object of the time-varying linear system
mysys = LinearSystemNoiseTV(A, B, C, Q, R, U, Y,W,dU);
mysys.set_reference(r);
mysys.set_noise(w_pred, w_real, E,w_y);


% Define the weights for g vector: L2 norm or adaptive term
diag_vector = 0.2:- 0.18/(T-N_ini-N_pred):0.02;
Qg = diag(diag_vector);
w_s = 50;

%% get past trajectories from random inputs
u_cl = zeros(nu,T_tot); x_cl = zeros(nx,T_tot+1); y_cl = zeros(ny,T_tot+1);
x_current = [19; 19; 15];
y_current = C*x_current + w_y(1);
x_cl(:, 1) = x_current;
y_cl(:, 1) = y_current;

for t = 1:144
    if x_cl(1,t) > 20.5
        u_cl(:, t) = 10*rand(1);
    else
        u_cl(:, t) = 30 + 10*rand(1);
    end
    [x_cl(:, t+1), y_cl(:, t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
end

figure()
hold on
grid on
yyaxis left
ylim([ 18,30])
a  = plot(x_cl(1,1:T+1), '-db');
% ab  = plot(x_cl(2,1:end-4), '-db');
aa  = plot( r(1,1:end-3), '-r');
xlabel('Time t', 'interpreter', 'latex','fontsize',20);
ylabel('Output y1', 'interpreter', 'latex','fontsize',20);
yyaxis right
ylim([ 0,45])
aaa  = plot(u_cl(1,1:end-3), '--k*');
ylabel('Input u1', 'fontsize',20);

h = legend([a, aa, aaa],  'y1','y1 reference', 'u1','Location','northeast');
set(h,'fontsize',24, 'interpreter', 'latex')

%%
% build the Henkal matrix 
disp('Computing Hankel...')

t = T_start;
N_total = N_ini + N_pred;
Hankel_col = T - N_total + 1;
Hankel_U = compute_Hankel_matrix(u_cl(:,t-T:t-1),nu,N_total, Hankel_col);
Hankel_W = compute_Hankel_matrix(w_real(:,t-T:t-1),nw,N_total, Hankel_col);

N_total = N_ini + N_pred + 1;
Hankel_col = T - N_total + 2;
Hankel_Y = compute_Hankel_matrix(y_cl(:,t-T:t),ny,N_total, Hankel_col);

N_total = N_ini + N_pred + nx;
Hankel_col = T - N_total +1 ;

% Need: u is persistently exciting of order is N_ini + N_pred + nx
Hankel_UW_check = compute_Hankel_matrix([u_cl(:,t-T:t-1); w_real(:,t-T:t-1)], nu+nw, N_total, Hankel_col);


% Check if the past u trajectory is persistently exciting of order N_ini + N_pred + nx
if rank(Hankel_UW_check)== (nu+nw)*(N_ini + N_pred + nx) 
    disp('Hankel rank is ok')
else
    disp('Exciting of order of Hu is samller than N_ini + N_pred + nx')
end

Hankel_p = [Hankel_U(1:nu*N_ini, :);
            Hankel_W(1:nw*N_ini, :);
              Hankel_Y(1:ny*(N_ini+1), :)];
Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
            Hankel_W(nw*N_ini+1:end, :);
              Hankel_Y(ny*(N_ini+1)+1:end, :)];
Hankel_U_backup = Hankel_U; Hankel_Y_backup = Hankel_Y;  Hankel_W_backup = Hankel_W;        
%%
% Define object of predictive controller
controller = PredictiveControl(mysys, N_ini, N_pred, T);

% Current time
t = T_start;
controller.initialize_deepc_robust();
[x_current, y_current] = mysys.move_one_step(x_cl(:,t-1), u_cl(:,t-1), t-1);

%% update disturbance
for num_monte = 1:1
%     [w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,num_day,num_T, T_day, var_w_y);
%     mysys.set_noise(w_pred, w_real, E,w_y);
    controller = PredictiveControl(mysys, N_ini, N_pred, T);
    for num_which =1:1
        % Restart from t = T+1
        t = T_start;
        controller.initialize_deepc_robust();
        [x_current, y_current] = mysys.move_one_step(x_cl(:,t-1), u_cl(:,t-1), t-1);
        Hankel_U = Hankel_U_backup; Hankel_Y = Hankel_Y_backup;  Hankel_W = Hankel_W_backup; 
        Hankel_p = [Hankel_U(1:nu*N_ini, :);
                    Hankel_W(1:nw*N_ini, :);
                      Hankel_Y(1:ny*(N_ini+1), :)];
        Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
                    Hankel_W(nw*N_ini+1:end, :);
                      Hankel_Y(ny*(N_ini+1)+1:end, :)];

% Run the controller

        while(t<=T_tot)
            u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
            w_ini = reshape(w_real(:,t-N_ini:t-1), [N_ini*nw,1]);
            y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]);
            display('=============');
            t
        % (1) Basic deepc
        %     controller.set_deepc_noise(Hankel_p, Hankel_f, u_ini, w_ini, y_ini)

        % (2) Use L2-norm for g 
        if num_which == 1
            controller.set_deepc_robust(Hankel_p, Hankel_f, Qg, w_s)
        % (3) Use bi-level
        elseif num_which == 2
            controller.set_deepc_adaptive_noise(Hankel_p, Hankel_f, Qg, w_s)
        end

            try
                [u_opt, u_seq, y_seq] = controller.solve_deepc(t, u_ini, w_ini, y_ini,Hankel_p,Hankel_f);
            catch
%                 val_u = 12.0; 
%                 U = Polyhedron([val_u; 0]);
%                 controller.sys.U = U; 
                controller.set_deepc_robust(Hankel_p, Hankel_f, Qg, w_s,1) % use another slack
                [u_opt, u_seq, y_seq] = controller.solve_deepc(t, u_ini, w_ini, y_ini,Hankel_p,Hankel_f);
%                 val_u = 6.0; 
%                 U = Polyhedron([val_u; 0]);
%                 controller.sys.U = U;                 
            end
             % if too many predicted u are very small, add du
            if sum(abs(u_seq)<20) >=5
                controller.set_deepc_robust_du(Hankel_p, Hankel_f, Qg, w_s) % use another slack
                [u_opt, u_seq, y_seq] = controller.solve_deepc(t, u_ini, w_ini, y_ini,Hankel_p,Hankel_f);
                u_opt = u_opt + 2*rand(1);
            end           
            
            if u_opt >30
                u_cl_sp(:,t) = u_opt;
                u_opt = 30;                
            end
            [x_next, y_next] = mysys.move_one_step(x_current, u_opt, t);
if t == 151
    1
end
            u_cl(:, t) = u_opt;
            x_cl(:, t+1) = x_next;
            y_cl(:, t+1) = y_next;
            x_current = x_next;
            y_current = y_next;

            % Update Hankel matrix by latest data
            if 0%mod(t,10)==0
                N_total = N_ini + N_pred;
                Hankel_col = T - N_total + 1;

                Hankel_U = compute_Hankel_matrix(u_cl(:,t-T:t-1),nu,N_total, Hankel_col);
                Hankel_W = compute_Hankel_matrix(w_real(:,t-T:t-1),nw,N_total, Hankel_col);

                N_total = N_ini + N_pred + 1;
                Hankel_col = T - N_total + 2;
                Hankel_Y = compute_Hankel_matrix(y_cl(:,t-T:t),ny,N_total, Hankel_col);

                Hankel_p = [Hankel_U(1:nu*N_ini, :);
                            Hankel_W(1:nw*N_ini, :);
                              Hankel_Y(1:ny*(N_ini+1), :)];
                Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
                            Hankel_W(nw*N_ini+1:end, :);
                              Hankel_Y(ny*(N_ini+1)+1:end, :)];

                N_total = N_ini + N_pred + nx;
                Hankel_col = T - N_total +1 ;

                % Need: u is persistently exciting of order is N_ini + N_pred + nx
                Hankel_UW_check = compute_Hankel_matrix([u_cl(:,t-T+1:t); w_real(:,t-T+1:t)], nu+nw, N_total, Hankel_col);
                if rank(Hankel_UW_check)== (nu+nw)*(N_ini + N_pred + nx)  
                    1;
                else
                    warning('Exciting of order of Hu is samller than N_ini + N_pred + nx')
                end
            end
            t = t+1;
        end
        if num_which == 1
            record_L2.y_cl{num_monte} = y_cl;
            record_L2.u_cl{num_monte} = u_cl;
            record_L2.w_pred{num_monte} = w_pred;
            record_L2.w_real{num_monte} = w_real;
            record_L2.w_y{num_monte} = w_y;
        else
            record_ada.y_cl{num_monte} = y_cl;
            record_ada.u_cl{num_monte} = u_cl;
            record_ada.w_pred{num_monte} = w_pred;
            record_ada.w_real{num_monte} = w_real;
            record_ada.w_y{num_monte} = w_y;            
        end

    end
end

%% plot
num_tem =1;
% y_cl1 = record_ada.y_cl{num_tem};
% u_cl1 = record_ada.y_cl{num_tem};
% y_cl1 = record_L2.y_cl{num_tem};
% u_cl1 = record_L2.y_cl{num_tem};
y_cl1 = y_cl;
u_cl1 = u_cl;
figure()
hold on
grid on
yyaxis left
a  = plot(y_cl1(1,1:end-1), '-db');
b  = plot(C*x_cl(:,1:end-1), '-dg');
% ab  = plot(x_cl(2,1:end-4), '-db');
aa  = plot( r(1:72*3), '-r');
tem = [30*ones(1,24), 26*ones(1,30), 30*ones(1,18)];
plot([tem,tem,tem],'g')
tem = [18*ones(1,24), 20*ones(1,30), 18*ones(1,18)];
plot([tem,tem,tem],'g')
xlabel('Time t', 'interpreter', 'latex','fontsize',20);
ylabel('Output y1', 'interpreter', 'latex','fontsize',20);
yyaxis right
ylim([ -5,50])
aaa  = plot(u_cl1, '--k*');
ylabel('Input u1', 'fontsize',20);

h = legend([a, aa, aaa],  'y1','y1 reference', 'u1','Location','northeast');
set(h,'fontsize',24, 'interpreter', 'latex')
Costa = compute_cost( y_cl(:, 73:144),u_cl(:, 73:144),Q ,R, r(:,73:144), 72);
Costa(1)
%%
% i = 144;
% A{i}*x_cl(:,i)+B*u_cl(:,i)+ E*w_pred(:,i)
 % save('plant_144_Q0R0.01_N6.mat','x_cl','y_cl','u_cl','w_real','w_pred',)
%%
% % [a b] = mysys.move_N_step(x_current, u_seq, t, N_pred);
% Costa = compute_costQ( y_cl(:, 73:144),u_cl(:, 73:144),Q ,R, r(:,73:144), 72)
% %%
% save('result/L2_144_Q0R0.01_N8.mat','record_L2','Q' ,'R', 'r','T',...
% 'w1_pred', 'w2_pred', 'w3_pred','w1_rand', 'w2_rand', 'w3_rand');
%%
%% plot noise
% [w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,num_day,num_T, T_day, 0.1);
figure()
hold on
grid on
yyaxis left
a  = plot(w_pred(1,:), '-dm');
b = plot(w_real(1,:), '--m');
aa  = plot(w_pred(2,:), '-db');
bb  = plot( w_real(2,:), '--b');
xlabel('Time t', 'interpreter', 'latex','fontsize',20);
ylabel('Disturbance ', 'interpreter', 'latex','fontsize',20);
aaa  = plot(w_pred(3,:), '-dg');
bbb  = plot( w_real(3,:), '--g');

h = legend([a,b,aa,bb, aaa, bbb],  'w1_{pred}', 'w1_{real}','w2_{pred}', 'w2_{real}','w3_{pred}', 'w3_{real}','Location','northeast');
set(h,'fontsize',24, 'interpreter', 'tex')
figure()
plot(w_y)
%%
% ax=findall(gcf,'type','axes');
% yyaxis right
% ylim([ -15,15])
% %%
% Hankel_U = update_Hankel(Hankel_U, u_cl(:, t-T+1:t), nu);
% Hankel_Y = update_Hankel(Hankel_Y, y_cl(:, t-T+2:t+1), ny);
% Hankel_W = update_Hankel(Hankel_W, w_real(:, t-T+1:t), nw);
% %%
% [~,~,w_y1] = get_disturbance(w1_pred, w2_pred, w3_pred,w1_rand, w2_rand, w3_rand,w_y);
% % [w_pred, w_real] = get_disturbance(w1_pred, w2_pred, w3_pred,w1_rand, w2_rand, w3_rand);
% %%
% x_tem(:,1) = x_cl(:,1);
% y_tem(:,1) = C*x_tem(:,1)  + w_y(1);
% for i = 1:72
%         x_tem(:,i+1) = A{i}*x_tem(:,i) + B*u_cl(:,i) + E*w_real(:,i);
%         y_tem(:,i+1) = C*x_tem(:,i+1) + w_y(:,i);
% end
% x_tem-x_cl(:,1:73)
% y_tem-y_cl(:,1:73)
% %%
% figure()
% hold on
% plot(w_y,'b')
% plot(w_y1, 'r')
%%
function [Hankel_U] = update_Hankel(Hankel_U,u, nu)
        Hankel_U = Hankel_U(nu+1:end,:);
        num_col = size(Hankel_U,2);
        Hankel_U = [Hankel_U; u(:, end-num_col+1:end)];
end
%%
function [w_pred, w_real,w_y,w_rand, W] = get_ini_dist(T_day,num_day,num_T, var_y)
    Hw = [eye(3);-eye(3)];
    bw = [2;2;2;2;2;2];
    W = Polyhedron('A', Hw, 'b', bw);

    w1_rand = -3 + 6*rand(1,T_day*num_day);
    w2_rand = -5 + 10*rand(1,T_day*num_day);
    w3_rand = -2 + 4*rand(1,T_day*num_day);

    w1_pred = zeros(1,T_day*num_day);
    w2_pred = zeros(1,T_day*num_day);
    w3_pred = zeros(1,T_day*num_day);

        for j = 0:num_T-1
            % Disturbance
            a1 = 10 + 4*rand; 
            a2 = 2 + 4*rand; 
            a3 = 0 + 16*rand; 
            a4 = 0 + 2*rand; 
            a5 = 6 + 1*rand; 
%         a1 = 15 ; 
%         a2 = 4; 
%         a3 = 8; 
%         a4 = 2; 
%         a5 = 8; 

%             w1_rand(j*T_day+1:T_day*(j+1)) = -3 + 6*rand(1,T_day);
%             w2_rand(j*T_day+1:T_day*(j+1)) = -5 + 10*rand(1,T_day);
%             w3_rand(j*T_day+1:T_day*(j+1)) = -2 + 4*rand(1,T_day);
        w_max = 2; w_min = -2;
        w1_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w2_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w3_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);


            for i = 1:T_day
                w1_pred(i+j*T_day) = a1 + a2*sin(-0.5*pi+(i)/T_day*2*pi);
            end

            for i = 1:T_day
                if i<=18
                elseif i<=36
                    w2_pred(i+j*T_day) = a3*(i-18)/18;
                elseif i<=54
                    w2_pred(i+j*T_day) = a3*(54-i)/18;
                end
            end

            for i = 1:T_day
                if i<=24
                    w3_pred(i+j*T_day) = a4;
                elseif i<=64
                    w3_pred(i+j*T_day) =  a4+ a5;
                else
                    w3_pred(i+j*T_day) = a4;
                end
            end
        end
    w_y = var_y*randn(1,T_day*num_day);
    w_pred = [w1_pred; w2_pred; w3_pred];
    w_real = [w1_pred+w1_rand; w2_pred+w2_rand; w3_pred+w3_rand];
    w_rand = [w1_rand; w2_rand; w3_rand]; 
end
%%
function [w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,n,n_T,T_day,var_y)
w1_pred=w_pred(1,:); w2_pred=w_pred(2,:); w3_pred=w_pred(3,:);
w1_rand=w_rand(1,:); w2_rand=w_rand(2,:); w3_rand=w_rand(3,:);
w_y(n_T*T_day+1:n*T_day) = var_y*randn(1,(n-n_T)*T_day);
    for j = n_T:n-1
        % Disturbance
        a1 = 10 + 4*rand; 
        a2 = 2 + 4*rand; 
        a3 = 0 + 16*rand; 
        a4 = 0 + 2*rand; 
        a5 = 6 + 1*rand; 
%         a1 = 15 ; 
%         a2 = 4; 
%         a3 = 8; 
%         a4 = 2; 
%         a5 = 8; 

%         w1_rand(j*T_day+1:T_day*(j+1)) = -3 + 6*rand(1,T_day);
%         w2_rand(j*T_day+1:T_day*(j+1)) = -5 + 10*rand(1,T_day);
%         w3_rand(j*T_day+1:T_day*(j+1)) = -2 + 4*rand(1,T_day);
        w_max = 2; w_min = -2;
        w1_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w2_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w3_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);


        for i = 1:T_day
            w1_pred(i+j*T_day) = a1 + a2*sin(-0.5*pi+(i)/T_day*2*pi);
        end

        for i = 1:T_day
            if i<=18
            elseif i<=36
                w2_pred(i+j*T_day) = a3*(i-18)/18;
            elseif i<=54
                w2_pred(i+j*T_day) = a3*(54-i)/18;
            end
        end

        for i = 1:T_day
            if i<=24
                w3_pred(i+j*T_day) = a4;
            elseif i<=64
                w3_pred(i+j*T_day) =  a4+ a5;
            else
                w3_pred(i+j*T_day) = a4;
            end
        end
    end
    w_pred = [w1_pred; w2_pred; w3_pred];
    w_real = [w1_pred+w1_rand; w2_pred+w2_rand; w3_pred+w3_rand];
    
end
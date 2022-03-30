% Basic deepc demo for the a linear model
clc
clear all
addpath('utils','-end')
addpath('src','-end')
%% system definition
% Define your linear model
A = [1 0.1; 0 1];
B = eye(2);%[0.5; 1]; 
C = [1, 0];


Q = 1;
R = 1;

nu = size(B, 2);
nx = size(A, 1);
ny = size(C, 1);

% reference
r = 1;

y_max = 30.0;
Y = Polyhedron([y_max; -y_max]);

x_max = 2.0;
v_max = 2.0;
U = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);

mysys = LinearSystem(A, B, C, Q, R, U, Y);
mysys.set_reference(r);

N_pred = 3;
N_ini = 3;
T = 30; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1
T_tot = 50; % total steps for simulation
%% get past trajectories from random inputs
u_cl = zeros(nu,T_tot); x_cl = zeros(nx,T_tot+1); y_cl = zeros(ny,T_tot+1);
u_cl(:, 1:T) = 3*rand(nu,T)-1.5;
x_current = [0; 0];
y_current = C*x_current;
x_cl(:, 1) = x_current;
y_cl(:, 1) = y_current;


for t = 1:T
    [x_cl(:, t+1), y_cl(:, t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t));
end
y_mpc = y_cl; x_mpc = x_cl; u_mpc = u_cl; 

% build the Henkal matrix 
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

Hankel_p = [Hankel_U(1:nu*N_ini, :);
              Hankel_Y(1:ny*(N_ini+1), :)];
Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
              Hankel_Y(ny*(N_ini+1)+1:end, :)];
Hankel_U_backup = Hankel_U; Hankel_Y_backup = Hankel_Y;   

%% Initialize the controller
t = T + 1;
controller = PredictiveControl(mysys, N_ini, N_pred, T);
controller.initialize_deepc();
[x_current, y_current] = mysys.move_one_step(x_cl(:,T), u_cl(:,T));


%% Restart from t = T+1
t = T+1;
controller.initialize_deepc_TV(t);
[x_current, y_current] = mysys.move_one_step(x_cl(:,T), u_cl(:,T));

Hankel_U = Hankel_U_backup; Hankel_Y = Hankel_Y_backup;
Hankel_p = [Hankel_U(1:nu*N_ini, :);
              Hankel_Y(1:ny*(N_ini+1), :)];
Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
              Hankel_Y(ny*(N_ini+1)+1:end, :)];

%%
while(t<=T_tot)

    u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]); 
    
    controller.initialize_deepc();
    controller.set_deepc(Hankel_p, Hankel_f, u_ini, y_ini)
    [u_opt, u_seq, y_seq] = controller.solve(Hankel_p,Hankel_f, u_ini, y_ini);
    
    [x_next, y_next] = mysys.move_one_step(x_current, u_opt);
    
    u_cl(:, t) = u_opt;
    x_cl(:, t+1) = x_next;
    y_cl(:, t+1) = y_next;
    x_current = x_next;
    y_current = y_next;
    
    t = t+1;
end

%% MPC
t = T+1;
[x_current, y_current] = mysys.move_one_step(x_mpc(:,T), u_mpc(:,T));
controller = PredictiveControl(mysys, N_ini, N_pred, T);
while(t<=T_tot)
    
    [u_opt, u_seq, y_seq] = controller.solve_mpc(x_current);
    
    [x_next, y_next] = mysys.move_one_step(x_current, u_opt);
    
    u_mpc(:, t) = u_opt;
    x_mpc(:, t+1) = x_next;
    y_mpc(:, t+1) = y_next;
    x_current = x_next;
    y_current = y_next;
    
    t = t+1;
end
%% plot
figure()
hold on
grid on
yyaxis left
a  = plot(y_cl(1,:), '-dm');
aa  = plot(T+1:(size(y_cl,2)), r*ones(1,size(y_cl,2)-T), '-r');
b  = plot(y_mpc(1,:), '-sb');
xlabel('iteration', 'interpreter', 'latex','fontsize',20);
ylabel('iteration cumulated cost', 'interpreter', 'latex','fontsize',20);
yyaxis right
ylim([ -2,2])
aaa  = plot(u_cl(1,:), '--k*');
bb = plot(u_mpc(1,:), '--g');
ylabel('Difference', 'fontsize',20);

h = legend([a,b, aa, aaa,bb],  'y1_{deepc}','y1_{mpc}', 'y1_{reference}', 'u1_{deepc}', 'u1_{mpc}','Location','northeast');
set(h,'fontsize',24)

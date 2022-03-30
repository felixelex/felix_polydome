% Adaptive deepc demo for the a linear time-varying model

clc
clear all
addpath('utils','-end')
addpath('src','-end')
%% system definition
% The system includes some time-varying terms
% Time length choice: 
T_tot = 50; % total steps for simulation
N_ini = 3;
N_pred = 3;
T = 30; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1

% Real system dynamics
for i = 1:T_tot
    A{i} = [0.9 + 0.1/T_tot*i 0.1;
        0 1];
end
B = eye(2);%[0.5; 1]; 
C = [1, 0];

nu = size(B, 2);
nx = size(A{1}, 1);
ny = size(C, 1);

% Matrix in stage cost
Q = 1;
R = 1;

% Reference
r = 1;

% Constraints
y_max = 15.0; y_min = -15.0;
H_tem =  kron(eye(N_pred), [eye(ny); -eye(ny)]);
h_tem = kron(ones(N_pred,1), [y_max; -y_min]);
Y.H = H_tem; Y.h = h_tem;
% y_max = 15.0; 
Y.P = Polyhedron([y_max; -y_max]);

% val_u = 2.0; 
% U = Polyhedron([val_u; -val_u]);

u_max = [2.0; 2.0];
u_min = [-2.0; -2.0];
H_tem =  kron(eye(N_pred), [eye(nu); -eye(nu)]);
h_tem = kron(ones(N_pred,1), [u_max; -u_min]);
U.H = H_tem; U.h = h_tem;

x_max = 2.0;
v_max = 2.0;
U.P = Polyhedron([x_max v_max; x_max -v_max; -x_max -v_max; -x_max v_max]);

% Define the object of the time-varying linear system
mysys = LinearSystemTV(A, B, C, Q, R, U, Y);
mysys.set_reference(r);



% Define the weights for g vector: L1 norm or adaptive term
diag_vector = zeros(T-N_ini-N_pred+1,1);
for i = 1:T-N_ini-N_pred+1
    diag_vector(i) = 10 - 9/(T-N_ini-N_pred+1)*i;
end
Qg = diag(diag_vector);

%% get past trajectories from random inputs
u_cl = zeros(nu,T_tot); x_cl = zeros(nx,T_tot+1); y_cl = zeros(ny,T_tot+1);
u_cl(:, 1:T) = 3*rand(nu,T)-1.5;
x_current = [0; 0];
y_current = C*x_current;
x_cl(:, 1) = x_current;
y_cl(:, 1) = y_current;

for t = 1:T
    [x_cl(:, t+1), y_cl(:, t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
end


% build the Henkal matrix 
disp('Computing Hankel...')

N_total = N_ini + N_pred;
Hankel_col = T - N_total + 1;
Hankel_U = compute_Hankel_matrix(u_cl(:,1:T),nu,N_total, Hankel_col);

N_total = N_ini + N_pred + 1;
Hankel_col = T+1 - N_total + 1;
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
%%
% Define object of predictive controller
controller = PredictiveControl(mysys, N_ini, N_pred, T);

% Current time
t = T+1;
controller.initialize_deepc_TV(t);
[x_current, y_current] = mysys.move_one_step(x_cl(:,T), u_cl(:,T), T);


%%
% Restart from t = T+1
t = T+1;
controller.initialize_deepc_TV(t);
[x_current, y_current] = mysys.move_one_step(x_cl(:,T), u_cl(:,T), T);
Hankel_U = Hankel_U_backup; Hankel_Y = Hankel_Y_backup;
Hankel_p = [Hankel_U(1:nu*N_ini, :);
              Hankel_Y(1:ny*(N_ini+1), :)];
Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
              Hankel_Y(ny*(N_ini+1)+1:end, :)];
          
%% Run the controller

while(t<=T_tot)
    u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]);

    controller.initialize_deepc_TV(t);
    
% (1) Basic deepc
%     controller.set_deepc(Hankel_p, Hankel_f, u_ini, y_ini)

% (2) Use L1-norm for g 
%     controller.set_deepc_L1(Hankel_p, Hankel_f, u_ini, y_ini, Qg)
    
% (3) Use bi-level
    controller.set_deepc_adaptive(Hankel_p, Hankel_f, u_ini, y_ini, Qg)
    
    u_opt = controller.solve();
    [x_next, y_next] = mysys.move_one_step(x_current, u_opt, t);
    
    u_cl(:, t) = u_opt;
    x_cl(:, t+1) = x_next;
    y_cl(:, t+1) = y_next;
    x_current = x_next;
    y_current = y_next;
    
% Update Hankel matrix by latest data
    [Hankel_U,Hankel_Y] = update_Hankel(Hankel_U, Hankel_Y, u_cl(:, t-T+1:t), y_cl(:, t-T+2:t+1), nu, ny);
    Hankel_p = [Hankel_U(1:nu*N_ini, :);
                  Hankel_Y(1:ny*(N_ini+1), :)];
    Hankel_f = [Hankel_U(nu*N_ini+1:end, :);
                  Hankel_Y(ny*(N_ini+1)+1:end, :)];   
    
    t = t+1;
end

%%
Costl = compute_cost( y_cl(:, 31:50),u_cl(:, 31:50),Q ,R, r, 20);
%% plot
figure()
hold on
grid on
yyaxis left
a  = plot(y_cl(1,:), '-dm');
aa  = plot(T+1:(size(y_cl,2)), r*ones(1,size(y_cl,2)-T), '-r');
xlabel('Time t', 'interpreter', 'latex','fontsize',20);
ylabel('Output y1', 'interpreter', 'latex','fontsize',20);
yyaxis right
ylim([ -2,2])
aaa  = plot(u_cl(1,:), '--k*');
ylabel('Input u1', 'fontsize',20);

h = legend([a, aa, aaa],  'y1', 'y1 reference', 'u1','Location','northeast');
set(h,'fontsize',24, 'interpreter', 'latex')


%%
function [Hankel_U,Hankel_Y] = update_Hankel(Hankel_U,Hankel_Y,u,y, nu, ny)
        Hankel_U = Hankel_U(nu+1:end,:);
        Hankel_Y = Hankel_Y(ny+1:end,:);
        num_col = size(Hankel_U,2);
        Hankel_U = [Hankel_U; u(:, end-num_col+1:end)];
        Hankel_Y = [Hankel_Y; y(:, end-num_col+1:end)];

end

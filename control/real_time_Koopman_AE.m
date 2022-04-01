%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%=================           Koopman-based MPC           =================%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clean up
clearvars
close all
clc
clear all

% Paths
addpath('./parameters')
addpath('./KoopmanAE')
addpath(genpath('../utilities'))

% Start time of experiment
h = 18; 
min = 15;
TimeZone = 'Europe/Zurich';
current_time = datetime('now', 'TimeZone', TimeZone);
what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
what_time_start = datenum(what_year, what_month, what_day, h, min, 0);

%% Getting global parameters
parameter = KoopmanAE_par(0, what_time_start);
% current_time = datetime('now');
% what_year = year(current_time);
% what_month = month(current_time);
% what_day = day(current_time);
parameter.date_exp_start = now(); %in fraction of days 

fprintf('Matlab started at %s \n', datestr(now))

% Define object of predictive controller
koopman_ae_controller = KoopmanMPCController(parameter);

N_pred = parameter.N_pred;
N_ini = parameter.N_ini;
T = parameter.T; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1 %UNDERSTANDING: What does this parameter represent? Total experiment time?
Ts = parameter.Ts; % Sampling period
time_delay = parameter.time_delay;

nz = parameter.sys.nz;
nu = parameter.sys.nu;
ny = parameter.sys.ny;
nw = parameter.sys.nw;
nx = parameter.sys.nx;

T_experiment = parameter.T_experiment; % Number of closed-loop time steps
w_cl = zeros(nw,T_experiment);
u_cl = zeros(nu,T_experiment); 
x_cl = zeros(nx,T_experiment+1);
y_cl = zeros(ny,T_experiment+1);

%% Get initial state and input
disp('*============================================*')
disp('*============================================*')
fprintf('Experiment started on: %s at time zone %s \n', datestr(current_time,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      

% Start of initialization (1min before experiment start)
what_time_init = what_time_start - 60/86400;

% First computation and first sending timestemp
what_time_computing	= what_time_start - 50/86400;
what_time_sending	= what_time_start - 10/86400;

% what_time_computing	= datenum(what_year, what_month, what_day, h, min, 10)-N_ini*Ts/86400;
% what_time_sending	= datenum(what_year, what_month,what_day, h, min, 50)-N_ini*Ts/86400;
% what_time_ini	    = datenum(what_year, what_month, what_day, h, min+1, 00);

%%
disp("Waiting before getting initialization data...")
what_time_init = sleep(0, what_time_init);

for t = 1:N_ini
	fprintf('\n## Initialization step %s at %s \n', int2str(t), datestr(what_time_computing))
	tf = what_time_computing - 10/86400 - (N_ini-t)*900/86400;
	t0 = tf - Ts/86400;
	
	[y,x,u,w,m] = retreive_initial_data(t0, tf); %Get measurements from db
	y_cl(:,t) = y.value;
	x_cl(:,t) = x.value;
	cache.cl.t(:,t) = t; 
	cache.cl.y_cl(:,t) = y_cl(:,t); 
	cache.cl.x_cl(:,t) = x_cl(:,t);
	cache.cl.time(:,t) = now();
	
	if t ~= 1
		u_cl(:,t-1) = u.value;
		w_cl(:,t-1) = w.value;
		cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
		cache.cl.w_cl(:,t-1) = w_cl(:,t-1); 
	end
end

% % First initialization step
% t = 1;
% fprintf('## First initialization step at %s \n', datestr(what_time_computing))
% tf = what_time_computing;
% t0 = tf - 900/86400;
% [y,u,w] = retreive_initial_data(t0, tf); %Get measurements from db
% y_cl(:,t) = y.value;
% cache.cl.t(:,t) = t; 
% cache.cl.y_cl(:,t) = y_cl(:,t); 
% cache.cl.time(:,t) = now();
% 
% what_time_computing = sleep(900, what_time_computing);
% what_time_sending = sleep(900, what_time_sending);
% 
% % All other initialization steps
% for t = 2:N_ini
% 	fprintf('## Initialization step %s at %s \n', int2str(t), datestr(what_time_computing))
% 	tf = what_time_computing;
% 	t0 = tf - 900/86400;
% 	[y,u,w] = retreive_initial_data(t0, tf); %Get measurements from db
% 	y_cl(:,t) = y.value;
% 	u_cl(:,t-1) = u.value;
% 	w_cl(:,t-1) = w.value;
% 	cache.cl.t(:,t) = t; 
%     cache.cl.y_cl(:,t) = y_cl(:,t); 
%     cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
% 	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); 
%     cache.cl.time(:,t) = now();
% 	
% 	what_time_computing = sleep(900, what_time_computing);
%     if t<N_ini
%         what_time_sending = sleep(900, what_time_sending);	
%     end
% end

fprintf('\n*============End of initialization===========*\n')
disp('Initialization outputs:')
disp(y_cl(:,1:N_ini))
disp('Initialization inputs:')
disp(u_cl(:,1:N_ini))
disp('Initialization weather:')
disp(w_cl(:,1:N_ini))

%%
% --- Start/Restart from t = N_ini + 1 --- 
fprintf('*============Start of control loop===========*\n\n')

t = N_ini+1;
koopman_ae_controller.initialize_mpc_controller();

%%
what_time_computing = what_time_computing - Ts/86400;
what_time_sending = what_time_sending - Ts/86400;
while t <= T_experiment%-N_pred
    %
    disp('*============================================*')
    fprintf('*======== Control iteration number: %d =======* \n', t);
    disp('*============================================*') 
        
    % Pause before measuring state and computing input
	disp("Waiting before measureing state and computing input...")
    what_time_computing = sleep(Ts, what_time_computing);	
    
    % Update room temperature measurement
	fprintf('\n*============================================*\n')
    fprintf('===Iteration %d, Step 1: get measurment\n', t);
	fprintf('Time now is: %s \n', datestr(now))

	tf = what_time_computing;
	t0 = tf - Ts/86400;
	[y,x,u,w,m] = retreive_initial_data(t0, tf); %Get measurements from db
	y_cl(:,t) = y.value;
	x_cl(:,t) = x.value;
	u_cl(:,t-1) = u.value;
	w_cl(:,t-1) = w.value;
    fprintf('The current temperature: %d, last input:: %d, last air temp: %d, last radiaction: %d \n',...
		y_cl(:, t),u_cl(:, t), w_cl(1, t), w_cl(2, t));
	
	cache.cl.t(:,t) = t; 
    cache.cl.y_cl(:,t) = y_cl(:,t);
	cache.cl.x_cl(:,t) = x_cl(:,t);
    cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); 
    cache.cl.time(:,t) = now();

	disp('*============================================*')
    fprintf('===Iteration %d, Step 2: get prediction\n', t);
	fprintf('Time now is: %s \n' ,datestr(now))
    % Update forecast of weather conditions
    koopman_ae_controller.get_disturbance();
    w_pred_tem = [koopman_ae_controller.last_forecast.temp; koopman_ae_controller.last_forecast.rad]';
	
	cache.pred.air_temp(:,t) = koopman_ae_controller.last_forecast.temp';
	cache.pred.rad(:,t) = (koopman_ae_controller.last_forecast.rad)';
	cache.pred.time(:,t) = now();

    % update initial vectors 
    u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini,nu]);
    w_ini = reshape(w_cl(:,t-N_ini:t-1), [N_ini,nw])';
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1),ny]);
	x_ini = reshape(x_cl(:,t-N_ini:t)', [(N_ini+1),nx]);
       
    % Call the actual computation function for the controller 
    % (real-time controller should be implemented inside). The
    % result should be stored in .lastInput
 
    disp('*============================================*')
    fprintf('===Iteration %d, Step 3: compute input \n', t);
	fprintf('Time now is: %s \n', datestr(now))
%     try
%         % Robust MPC
%         koopman_ae_controller.initialize_koopman_ae_controller(); % Create OptKoopmanAEMPC obj
%         koopman_ae_controller.set_koopman_ae() % Adding cost and constraints
%         koopman_ae_controller.get_koopman_representation(y_ini, w_ini, parameter.model) % Get lifting and Koopman operators
%         [u_opt_tem, u_seq, y_seq] = koopman_ae_controller.solve(t, u_ini, w_ini, y_ini, w_pred_tem);        
%     catch e1
%         try
%             disp("Primary controller failed with message: " + e1);
% 			assert False
% 			% TODO: Implement Fallback controller.
%         catch e2
%             % TODO: Implement Fallback controller.
%         end
%     end   
	
	% Soft-constraint MPC
	koopman_ae_controller.get_koopman_representation(x_ini(end-time_delay,:), w_ini(end-time_delay,:), parameter.model) % Get lifting and Koopman operators
	koopman_ae_controller.initialize_mpc_controller(); % Create OptKoopmanAEMPC obj
	koopman_ae_controller.set_mpc_controller(w_pred_tem); % Adding cost and constraints
	[u_opt_tem, u_seq, y_seq, s_seq] = koopman_ae_controller.solve();  

    % Check the input
    if u_opt_tem > 6
        cache.koopman.u_sp(:,t) = u_opt_tem;
        u_opt_tem = 6; 
    elseif u_opt_tem <0
        cache.koopman.u_sp(:,t) = u_opt_tem;
        u_opt_tem = 0;
    end  
 
    fprintf('Optimal input %d \n', u_opt_tem);
	fprintf('Optimal input sequence \n');
	disp(u_seq')
	fprintf('Optimal output sequence \n');
	disp(y_seq')
	fprintf('Optimal slack sequence \n');
	disp(s_seq')
	fprintf('Time now is: %s \n' ,datestr(now))
	
	solution = koopman_ae_controller.soltion_cache;

	cache.koopman.u_opt_seq(:,t) = solution.u_opt_seq';
	cache.koopman.y_opt_seq(:,t) = solution.y_opt_seq';
	cache.koopman.u_opt(:,t) = u_opt_tem;
	cache.koopman.time(:,t) = now();
	
    % Pause before sending data to the low-level controller
	disp("Waiting before sending input to low-lvel controller...")
    what_time_sending = sleep(Ts, what_time_sending);
		
	disp('*============================================*')
    fprintf('===Iteration %d, Step 4: send input \n', t);
	fprintf('Time now is: %s \n', datestr(now))
	
	fprintf('Current computing time : %s, and sending time: %s \n', datestr(what_time_computing), datestr(what_time_sending))
    % Sending to the low-level controller
    koopman_ae_controller.send_input_to_low_level(u_opt_tem, 1); % 1 = heating mode

    % Update the current iteration
    t = t + 1; 
end

%% Saving
deepc_controller.date_exp_end = now();
save('./results/polydome_01-03_04_2022.mat', 'cache', 'y_cl','u_cl','w_cl','h','min','parameter')
save('./results/koopman_ae_01-03_04_2022.mat', 'koopman_ae_controller')

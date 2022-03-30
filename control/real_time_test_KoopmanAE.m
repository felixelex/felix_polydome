%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%=================           Koopman-based MPC           =================%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO:
% - replace initialization code
% - why do we go backwards in time?
% - understand error handling
% - change date in filename for saving

% Clean up
clearvars
close all
clc
clear all

% Paths
addpath('./parameters')
addpath('./KoopmanAE')
addpath(genpath('../utilities'))

%% Getting global parameters
parameter = KoopmanAE_par(0);
% current_time = datetime('now');
% what_year = year(current_time);
% what_month = month(current_time);
% what_day = day(current_time);
parameter.date_exp_start = now(); %in fraction of days 

fprintf('Matlab started at %s \n', datestr(now))

% Define object of predictive controller
koopman_ae_controller = KoopmanMPCController(parameter);
koopman_ae_controller_backup = KoopmanMPCController(parameter);

N_pred = parameter.N_pred;
N_ini = parameter.N_ini;
T = parameter.T; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1 %UNDERSTANDING: What does this parameter represent? Total experiment time?
Ts = parameter.Ts; % Sampling period

nx = parameter.sys.nx;
nu = parameter.sys.nu;
ny = parameter.sys.ny;
nw = parameter.sys.nw;

T_experiment = parameter.T_experiment; % Number of closed-loop time steps
w_cl = zeros(nw,T_experiment);
u_cl = zeros(nu,T_experiment); x_cl = zeros(nx,T_experiment+1); y_cl = zeros(ny,T_experiment+1);

%% Get initial state and input

TimeZone = 'Europe/Zurich';
current_time = datetime('now', 'TimeZone', TimeZone );
disp('*============================================*')
disp('*============================================*')
fprintf('Experiment started on: %s at time zone %s \n', datestr(current_time,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      

what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
% Takes care of time drifting
% The experiment starts at 18h00 (computing of first inpu at 17h59). The
% initialization starts running N_ini time steps earlier.
h = 17;
min = 59;
what_time_computing	= datenum(what_year, what_month, what_day, h, min, 10)-N_ini*Ts/86400;
what_time_sending	= datenum(what_year, what_month,what_day, h, min, 50)-N_ini*Ts/86400;
what_time_ini	    = datenum(what_year, what_month, what_day, h, min+1, 00);
%%
%UNDERSTANDING: During intialization we fill the cache with measurements
%from the system. But what is the actual control input used during these
%steps?

% First initialization step
t = 1
fprintf('Time is: %s \n', datestr(what_time_computing))
tf = what_time_computing;
t0 = tf - 900/86400;
[y,u,w] = retreive_initial_data(t0, tf); %Get measurements from db
y_cl(:,t) = y.value;
cache.cl.t(:,t) = t; 
cache.cl.y_cl(:,t) = y_cl(:,t); 
cache.cl.time(:,t) = now();

what_time_computing = sleep(900, what_time_computing);
what_time_sending = sleep(900, what_time_sending);

% All other initialization steps
for t = 2:N_ini
	t
	fprintf('Time is: %s \n' ,datestr(what_time_computing))
	tf = what_time_computing;
	t0 = tf - 900/86400;
	[y,u,w] = retreive_initial_data(t0, tf); %Get measurements from db
	y_cl(:,t) = y.value;
	u_cl(:,t-1) = u.value;
	w_cl(:,t-1) = w.value;
	cache.cl.t(:,t) = t; 
    cache.cl.y_cl(:,t) = y_cl(:,t); 
    cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); 
    cache.cl.time(:,t) = now();
	
	what_time_computing = sleep(900, what_time_computing);
    if t<N_ini
        what_time_sending = sleep(900, what_time_sending);	
    end
end

%%
% --- Start/Restart from t = N_ini + 1 --- 

t = N_ini+1;
koopman_ae_controller.initialize_koopman_ae_controller();

% what_time_sending	= what_time_sending - Ts/86400;
what_time_computing	= what_time_computing - Ts/86400;

%%
while t <= T_experiment-N_pred
    %
    disp('*============================================*')
    fprintf('*======== Control iteration number: %d =======* \n', t);
    disp('*============================================*') 
        
    % Pause before measuring state and computing input
    what_time_computing = sleep(Ts, what_time_computing);	
    
    % Update room temperature measurement
	disp('*============================================*')
    fprintf('===Iteration %d, Step 1: get measurment\n', t);
	fprintf('Time now is: %s \n', datestr(now))
%     y_cl(:, t) = read_value_from_influxdb("simulation_temperature", Ts);

	tf = what_time_computing;
	t0 = tf - 900/86400;
	[y,u,w] = retreive_initial_data(t0, tf); %Get measurements from db
	y_cl(:,t) = y.value;
	u_cl(:,t-1) = u.value;
	w_cl(:,t-1) = w.value;
    fprintf('The current temperature: %d, last input:: %d, last air temp: %d, last radiaction: %d \n',...
		y_cl(:, t),u_cl(:, t), w_cl(1, t), w_cl(2, t));
	
	cache.cl.t(:,t) = t; 
    cache.cl.y_cl(:,t) = y_cl(:,t); 
    cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); 
    cache.cl.time(:,t) = now();

	disp('*============================================*')
    fprintf('===Iteration %d, Step 2: get prediction\n', t);
	fprintf('Time now is: %s \n' ,datestr(now))
    % Update forecast of weather conditions
    koopman_ae_controller.get_disturbance();
    w_pred_tem = [koopman_ae_controller.last_forecast.temp; koopman_ae_controller.last_forecast.rad];
    w_pred_tem = reshape(w_pred_tem, [nw*N_pred,1]);
	
	cache.pred.air_temp(:,t) = koopman_ae_controller.last_forecast.temp';
	cache.pred.rad(:,t) = (koopman_ae_controller.last_forecast.rad/1000.0)';
	cache.pred.time(:,t) = now();

    % update initial vectors %UNDERSTANDING: These data matrices are used
    % for building the Henkel matrix and are not necessary in our case
    % right?
    u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
    w_ini = reshape(w_cl(:,t-N_ini:t-1), [N_ini*nw,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]);
       
    % Call the actual computation function for the controller 
    % (real-time controller should be implemented inside). The
    % result should be stored in .lastInput
 
    disp('*============================================*')
    fprintf('===Iteration %d, Step 3: compute input \n', t);
	fprintf('Time now is: %s \n', datestr(now))
    try
        % Robust MPC
        koopman_ae_controller.initialize_koopman_ae_controller(); % Create OptKoopmanAEMPC obj
        koopman_ae_controller.set_koopman_ae() % Adding cost and constraints
        koopman_ae_controller.get_koopman_representation(y_ini, w_ini, parameter.model) % Get lifting and Koopman operators
        [u_opt_tem, u_seq, y_seq] = koopman_ae_controller.solve(t, u_ini, w_ini, y_ini, w_pred_tem);        
    catch e1
        try
            % Using MPC as a fallback when robust MPC is not working.
            fprintf(2,'Error 1. The message was:\n%s',e1.message);
            koopman_ae_controller.set(1) % TODO: Understand error handling!
            [u_opt_tem, u_seq, y_seq] = koopman_ae_controller.solve(t,u_ini, w_ini, y_ini, w_pred_tem);     
            cache.koopman.backup(:,t) = 1;
        catch e2
            % Using default controller, if robust MPC and standard MPC are
            % not working.
            fprintf(2,'Error 2. The message was:\n%s',e2.message);
            koopman_ae_controller.initialize_deepc_noise_TV(t); %TODO: change this!
            koopman_ae_controller.set_deepc_noise("adaptive")
            [u_opt_tem, u_seq, y_seq] = koopman_ae_controller.solve(t, u_ini, w_ini, y_ini, w_pred_tem);
            cache.koopman.backup(:,t) = 2;
        end
    end   
    
    % Check the input
    if u_opt_tem > 6
        cache.koopman.u_sp(:,t) = u_opt_tem;
        u_opt_tem = 6; 
    elseif u_opt_tem <0
        cache.koopman.u_sp(:,t) = u_opt_tem;
        u_opt_tem = 0;
    end   
 
    fprintf('Get input %d \n', u_opt_tem);
	fprintf('Time now is: %s \n' ,datestr(now))
	
	solution = koopman_ae_controller.soltion_cache;

	parameter.sys.r(t:t+N_pred-1)

	cache.koopman.u_opt_seq(:,t) = solution.u_opt_seq';
	cache.koopman.y_opt_seq(:,t) = solution.y_opt_seq';
	cache.koopman.u_opt(:,t) = u_opt_tem;
	cache.koopman.time(:,t) = now();
	
    % Pause before sending data to the low-level controller
    what_time_sending = sleep(Ts, what_time_sending);
		
	disp('*============================================*')
    fprintf('===Iteration %d, Step 4: send input \n', t);
	fprintf('Time now is: %s \n', datestr(now))
	
	fprintf('Current computing time : %s, and sending time: %s \n', datestr(what_time_computing), datestr(what_time_sending))
    % Sending to the low-level controller
    koopman_ae_controller.send_input_to_low_level(u_opt_tem);

    % Update the current iteration
    t = t + 1; 
end

%% Saving
deepc_controller.date_exp_end = now();
save('./results/polydome_24-25_05_2021.mat', 'cache', 'y_cl','u_cl','w_cl','h','min','parameter')
save('./results/koopman_ae_24-25_05_2021.mat', 'koopman_ae_controller')

%% Plotting
figure
hold on
yyaxis left
plot(cache.cl.u_cl)
plot(cache.cl.w_cl(1,:))
plot(cache.cl.y_cl)
yyaxis right
plot(cache.cl.w_cl(2,:))
h = legend('u', 'air temp','y', 'GHI');
set(h,'fontsize',24, 'interpreter', 'latex')
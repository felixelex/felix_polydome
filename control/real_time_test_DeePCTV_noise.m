clearvars
close all
clc
clear all
addpath('./parameters')
addpath('./DeePC_solver')
addpath(genpath('../utilities'))
%%
parameter = DeePC_par(0);
current_time = datetime('now');
what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
parameter.date_exp_start = now();

fprintf('Matlab started at %s \n', datestr(now))


% Define object of predictive controller
deepc_controller = PredictiveController(parameter);

N_pred = parameter.N_pred;
N_ini = parameter.N_ini;
T = parameter.T; % T >= (nu + 1)*(N_ini + N_pred + nx) - 1
Ts = parameter.Ts; % Sampling period

nx = parameter.sys.nx;
nu = parameter.sys.nu;
ny = parameter.sys.ny;
nw = parameter.sys.nw;


T_experiment = parameter.T_experiment;
w_cl = zeros(nw,T_experiment);
u_cl = zeros(nu,T_experiment); x_cl = zeros(nx,T_experiment+1); y_cl = zeros(ny,T_experiment+1);
%%
load('exp6_15min')
% Build y, u vector for Hankel matrix
y_hankel = exp6.sensor_temp.value(1:T+1,:)';
u_hankel = exp6.power.value(1:T,:)'-2.35;
w_hankel = [exp6.air_temp.value(1:T,:)';exp6.solar_GHI.value(1:T,:)'/1000.0];

% Build the Henkal matrix 
deepc_controller.compute_Hankel_matrix(u_hankel, w_hankel,y_hankel);

%% get initial state and input

TimeZone = 'Europe/Zurich';
current_time = datetime('now', 'TimeZone', TimeZone );
disp('*============================================*')
disp('*============================================*')
fprintf('Experiment started on: %s at time zone %s \n', datestr(current_time,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      

what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
% Takes care of time drifting
h = 16;
min = 59;
what_time_computing	= datenum(what_year, what_month, what_day, h, min, 10)-N_ini*Ts/86400;
what_time_sending	= datenum(what_year, what_month,what_day, h, min, 50)-N_ini*Ts/86400;

what_time_ini	= datenum(what_year, what_month, what_day, h, min+1, 00);
%%
t = 1
fprintf('Time is: %s \n' ,datestr(what_time_computing))
tf = what_time_computing;
t0 = tf - 900/86400;
[y,u,w] = retreive_initial_data( t0, tf);
y_cl(:,t) = y.value;
cache.cl.t(:,t) = t; cache.cl.y_cl(:,t) = y_cl(:,t); cache.cl.time(:,t) = now();

what_time_computing = sleep(900, what_time_computing);
what_time_sending = sleep(900, what_time_sending);


for t = 2:N_ini
	t
	fprintf('Time is: %s \n' ,datestr(what_time_computing))
	tf = what_time_computing;
	t0 = tf - 900/86400;
	[y,u,w] = retreive_initial_data( t0, tf);
	y_cl(:,t) = y.value;
	u_cl(:,t-1) = u.value;
	w_cl(:,t-1) = w.value;
	cache.cl.t(:,t) = t; cache.cl.y_cl(:,t) = y_cl(:,t); cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); cache.cl.time(:,t) = now();
	
	what_time_computing = sleep(900, what_time_computing);
    if t<N_ini
        what_time_sending = sleep(900, what_time_sending);	
    end
end

%%
% --- Start/Restart from t = N_ini + 2 ---

t = N_ini+1;
deepc_controller.initialize_deepc_noise_TV(t);

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
	fprintf('Time now is: %s \n' ,datestr(now))
%     y_cl(:, t) = read_value_from_influxdb("simulation_temperature", Ts);

	tf = what_time_computing;
	t0 = tf - 900/86400;
	[y,u,w] = retreive_initial_data( t0, tf);
	y_cl(:,t) = y.value;
	u_cl(:,t-1) = u.value;
	w_cl(:,t-1) = w.value;
    fprintf('The current temperature: %d, last input:: %d, last air temp: %d, last radiaction: %d \n',...
		y_cl(:, t),u_cl(:, t), w_cl(1, t), w_cl(2, t));
	
	cache.cl.t(:,t) = t; cache.cl.y_cl(:,t) = y_cl(:,t); cache.cl.u_cl(:,t-1) = u_cl(:,t-1); 
	cache.cl.w_cl(:,t-1) = w_cl(:,t-1); cache.cl.time(:,t) = now();

	disp('*============================================*')
    fprintf('===Iteration %d, Step 2: get prediction\n', t);
	fprintf('Time now is: %s \n' ,datestr(now))
    % Update forecast of weather conditions
    deepc_controller.get_disturbance();
%     w_cl(:,t) = [deepc_controller.last_dist.temp; deepc_controller.last_dist.rad;0];
    w_pred_tem = [deepc_controller.last_forecast.temp; deepc_controller.last_forecast.rad/1000.0];
    w_pred_tem = reshape(w_pred_tem, [nw*N_pred,1]);
	
	cache.pred.air_temp(:,t) = deepc_controller.last_forecast.temp';
	cache.pred.rad(:,t) = (deepc_controller.last_forecast.rad/1000.0)';
	cache.pred.time(:,t) = now();


    % update initial vectors
    u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
    w_ini = reshape(w_cl(:,t-N_ini:t-1), [N_ini*nw,1]);
    y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]);
    
    % Update Hankel matrix by latest data
    fprintf('===Iteration %d, Step 2.2: update hankel matrix \n', t);
    if mod(t,10)==0 && t >= N_ini + N_pred + 1 
        T_new = t-1;
        if T_new < T
            deepc_controller.update_Hankel_matrix(u_cl(:,t-T_new:t-1), w_cl(:,t-T_new:t-1), y_cl(:,t-T_new:t), T_new);
        else
            deepc_controller.compute_Hankel_matrix(u_cl(:,t-T:t-1), w_cl(:,t-T:t-1), y_cl(:,t-T:t));
        end
    end        
    

    % Call the actual computation function for the controller 
    % (real-time controller should be implemented inside). The
    % result should be stored in .lastInput
    %
	disp('*============================================*')
    fprintf('===Iteration %d, Step 3: compute input \n', t);
	fprintf('Time now is: %s \n' ,datestr(now))
    deepc_controller.initialize_deepc_noise_TV(t);
    deepc_controller.set_deepc_noise("adaptive")
    [u_opt_tem, u_seq, y_seq] = deepc_controller.solve_deepc(t, u_ini, w_ini, y_ini, w_pred_tem);    
    fprintf('Get input %d \n', u_opt_tem);
	fprintf('Time now is: %s \n' ,datestr(now))
	
	solution = deepc_controller.soltion_cache;

	parameter.sys.r(t:t+N_pred-1)

	cache.deepc.u_opt_seq(:,t) = solution.u_opt_seq';
	cache.deepc.y_opt_seq(:,t) = solution.y_opt_seq';
	cache.deepc.g_opt(:,t) = solution.g_opt;
	cache.deepc.slack_opt(:,t) = solution.slack_opt;
	cache.deepc.u_opt(:,t) = u_opt_tem;
	cache.deepc.time(:,t) = now();
	
	
    %                
    % Pause before sending data to the low-level controller
    what_time_sending = sleep(Ts, what_time_sending);
		
    %
	disp('*============================================*')
    fprintf('===Iteration %d, Step 4: send input \n', t);
	fprintf('Time now is: %s \n' ,datestr(now))
	
	fprintf('Current computing time : %s, and sending time: %s \n' ,datestr(what_time_computing ),datestr(what_time_sending))
    % Sending to the low-level controller
    deepc_controller.send_input_to_low_level(u_opt_tem);

    %
%     fprintf('===Iteration %d, Step 5: simulation step \n', t);	
%     what_time_simulation = sleep(Ts, what_time_simulation);
%     [x_cl(:,t+1), y_current] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
%     write_value_to_influxdb(y_current, "simulation_temperature");
    % Update the current iteration
% 	break;
% 	solution
% 	solution.slack_opt'
    t = t + 1; 

%     fprintf('Time now is: %s \n' ,datestr(now))
end
%%
deepc_controller.date_exp_end = now();

save('./results/polydome_011-01205_2021.mat', 'cache', 'y_cl','u_cl','w_cl','h','min','parameter')
save('./results/deepc_011-01205_2021.mat', 'deepc_controller')
%%
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
% %% get past trajectories from random inputs
% %Start control iteration
% % Loop Starts
% TimeZone = 'Europe/Zurich';
% current_time = datetime('now', 'TimeZone', TimeZone );
% disp('*============================================*')
% disp('*============================================*')
% fprintf('Experiment started on: %s at time zone %s \n', datestr(current_time,...
%             'dd-mmm-yyyy HH:MM:SS'), TimeZone)      
% 
% what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
% % Takes care of time drifting
% what_hour = 21;
% what_min = 13;
% what_time_computing	= datenum(what_year, what_month, what_day, what_hour, what_min, 10);
% what_time_sending	= datenum(what_year, what_month,what_day, what_hour, what_min, 30);
% 
% what_time_simulation	= datenum(what_year, what_month, what_day, 13, 0, 00);
% %
% t = 1;
% %%%
% sleep(0, what_time_simulation);
% write_value_to_influxdb(19, "simulation_temperature");
% %%%
% %%
% 
% %% Use some random input to build the Hankel matrix
% while t <= T
%     %
%     disp('*============================================*')
%     fprintf('*======== Control iteration number: %d =======* \n', t);
%     disp('*============================================*')   
%     what_time_computing = sleep(Ts, what_time_computing);
%     datestr(what_time_computing)
%     
%     % Update room temperature measurement
%     y_cl(:, t) = read_value_from_influxdb("simulation_temperature", Ts);
%     fprintf('The last temperature measurement is: %d\n', y_cl(:, t));    
%     
%     % Update measurements and forecast of weather conditions
%     deepc_controller.get_disturbance();
%     w_cl(:,t) = [deepc_controller.last_dist.temp; deepc_controller.last_dist.rad;0];
%  
%     % Call the actual computation function for the controller 
%     % (real-time controller should be implemented inside). The
%     % result should be stored in .lastInput
%     %
%     u_cl(:,t) = deepc_controller.sample_random_input(6,0);
%     fprintf('Get input %d \n', u_cl(:,t));
%     %                
%     % Pause before sending data to the low-level controller
% % 				%
%     what_time_sending = sleep(Ts, what_time_sending);
% % 				
%     % Sending to the low-level controller
%     deepc_controller.send_input_to_low_level(u_cl(:,t));
% 
%     %
%     what_time_simulation = sleep(Ts, what_time_simulation);
%     [x_cl(:,t+1), y_current] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
%     write_value_to_influxdb(y_current, "simulation_temperature");
%     % Update the current iteration
%     t = t + 1;                
%                 
% end
%     
% %%           
% 
% 
% u_cl = zeros(nu,T_experiment); x_cl = zeros(nx,T_experiment+1); y_cl = zeros(ny,T_experiment+1);
% u_cl(:, 1:T) = 6*rand(nu,T);
% % u_cl(:, 1:30) = 0.1*rand(nu,30);
% % u_cl(:, 31:T) = -3*rand(nu,T-31+1); %0.85+0.2:
% x_current = [19; 19; 15];
% y_current = 19;
% x_cl(:, 1) = x_current;
% y_cl(:, 1) = y_current;
% 
% for t = 1:T
%     [x_cl(:, t+1), y_cl(:, t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
% end
% %%
% TimeZone = 'Europe/Zurich';
% current_time = datetime('now', 'TimeZone', TimeZone );
% disp('*============================================*')
% disp('*============================================*')
% fprintf('Experiment started on: %s at time zone %s \n', datestr(current_time,...
%             'dd-mmm-yyyy HH:MM:SS'), TimeZone)      
% 
% what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
% % Takes care of time drifting
% min = 1;
% what_time_computing	= datenum(what_year, what_month, what_day, 18, min, 10);
% what_time_sending	= datenum(what_year, what_month,what_day, 18, min, 30);
% 
% what_time_simulation	= datenum(what_year, what_month, what_day, 18, min+1, 00);
% 
% sleep(0, what_time_simulation);
% write_value_to_influxdb(y_cl(:,T+1), "simulation_temperature");
% 
% % --- Start/Restart from t = T+1 ---
% 
% t = T+1;
% deepc_controller.initialize_deepc_noise_TV(t);
% 
% while t <= T_experiment-N_pred
%     %
%     disp('*============================================*')
%     fprintf('*======== Control iteration number: %d =======* \n', t);
%     disp('*============================================*') 
%      
%     
%     % Pause before measuring state and computing input
%     what_time_computing = sleep(Ts, what_time_computing);	
%     
%     % Update room temperature measurement
%     fprintf('===Iteration %d, Step 1: get temperature measurment\n', t);
%     y_cl(:, t) = read_value_from_influxdb("simulation_temperature", Ts);
% %     fprintf('The last temperature measurement is: %d \n', y_cl(:, t)); 
% 
%     % Update measurements and forecast of weather conditions
%     deepc_controller.get_disturbance();
%     w_cl(:,t) = [deepc_controller.last_dist.temp; deepc_controller.last_dist.rad;0];
%     w_pred_tem = [temp_pred; rad_pred;zeros(1,deepc_controller.N_pred)];  
% 
% 
%     % update initial vectors
%     u_ini = reshape(u_cl(:,t-N_ini:t-1), [N_ini*nu,1]);
%     w_ini = reshape(w_cl(:,t-N_ini:t-1), [N_ini*nw,1]);
%     y_ini = reshape(y_cl(:,t-N_ini:t), [(N_ini+1)*ny,1]);
%     
%     % Update Hankel matrix by latest data
%     fprintf('===Iteration %d, Step 2: compute hankel matrix \n', t);
%     if 1 %mod(t,10)==0
%         deepc_controller.compute_Hankel_matrix(u_cl(:,t-T:t-1), w_cl(:,t-T:t-1), y_cl(:,t-T:t));
%     end      
%     
% 
%     % Call the actual computation function for the controller 
%     % (real-time controller should be implemented inside). The
%     % result should be stored in .lastInput
%     %
%     fprintf('===Iteration %d, Step 3: compute input \n', t);
%     deepc_controller.initialize_deepc_noise_TV(t);
%     deepc_controller.set_deepc_noise(u_ini, w_ini, y_ini, w_pred_tem, "adaptive")
%     [u_cl(:,t), u_seq, y_seq] = deepc_controller.solve();    
%     fprintf('Get input %d \n', u_cl(:,t));
%     %                
%     % Pause before sending data to the low-level controller
%     what_time_sending = sleep(Ts, what_time_sending);
%     % 
%     fprintf('===Iteration %d, Step 4: send input \n', t);				
%     % Sending to the low-level controller
%     deepc_controller.send_input_to_low_level(u_cl(:,t));
% 
%     %
%     fprintf('===Iteration %d, Step 5: simulation step \n', t);	
%     what_time_simulation = sleep(Ts, what_time_simulation);
%     [x_cl(:,t+1), y_current] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
%     write_value_to_influxdb(y_current, "simulation_temperature");
%     % Update the current iteration
%     t = t + 1;                
%                 
% end
% 
% %%
% % [a b] = mysys.move_N_step(x_current, u_seq, t, N_pred);
% Costa = compute_cost( y_cl(:, 61:141),u_cl(:, 61:141),Q ,R, r(:, 61:141), 81);
% 
% %% plot
% figure()
% hold on
% grid on
% yyaxis left
% a  = plot(y_cl(1,1:end-4), '-db');
% % ab  = plot(x_cl(2,1:end-4), '-db');
% aa  = plot( parameter.sys.r(1,1:end-3), '-r');
% xlabel('Time t', 'interpreter', 'latex','fontsize',20);
% ylabel('Output y1', 'interpreter', 'latex','fontsize',20);
% yyaxis right
% ylim([ -15,15])
% aaa  = plot(u_cl(1,1:end-3), '--k*');
% ylabel('Input u1', 'fontsize',20);
% 
% h = legend([a, aa, aaa],  'y1','y1 reference', 'u1','Location','northeast');
% set(h,'fontsize',24, 'interpreter', 'latex')
% %% plot noise
% figure()
% hold on
% grid on
% yyaxis left
% a  = plot(w_pred(1,:), '-dm');
% b = plot(w_real(1,:), '--m');
% aa  = plot(w_pred(2,:), '-db');
% bb  = plot( w_real(2,:), '--b');
% xlabel('Time t', 'interpreter', 'latex','fontsize',20);
% ylabel('Disturbance ', 'interpreter', 'latex','fontsize',20);
% aaa  = plot(w_pred(3,:), '-dg');
% bbb  = plot( w_real(3,:), '--g');
% 
% h = legend([a,b,aa,bb, aaa, bbb],  'w1_pred', 'w1_real','w2_pred', 'w2_real','w3_pred', 'w3_real','Location','northeast');
% set(h,'fontsize',24, 'interpreter', 'latex')
% %%
% ax=findall(gcf,'type','axes');
% yyaxis right
% ylim([ -15,15])
% 
% %%
% a = x_cl(:,1);
% b = y_cl(1);
% for t = 1:108
%     [a(:,t+1), b(t+1)] = mysys.move_one_step(x_cl(:,t), u_cl(:,t), t);
% end
% a - x_cl(:,1:109)
% b - y_cl(:,1:109)
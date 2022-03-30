clc
clear all
addpath(genpath('../utilities'))
%% Basic parameters
T = 63*3; % Total steps
t = 1; % Current step
Ts = 900; % Smpling time: time between each step

u_cl = zeros(1,T);
band = [0,1];
range = [0,1];
u_cl = idinput([63,1,3],'prbs',band,range);
%% get past trajectories from random inputs
%Start control iteration
% Loop Starts
TimeZone = 'Europe/Zurich';
currentTime = datetime('now', 'TimeZone', TimeZone );
disp('*============================================*')
disp('*============================================*')
fprintf('Experiment started on: %s at time zone %s \n', datestr(currentTime,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      

WhatYear = year(currentTime);WhatMonth = month(currentTime);WhatDay = day(currentTime);
% Takes care of time drifting
what_time_computing	= datenum(WhatYear, WhatMonth, WhatDay, 10, 42, 50);
% what_time_sending	= datenum(WhatYear, WhatMonth,WhatDay, 13, 00, 30);
% 
% what_time_simulation	= datenum(WhatYear, WhatMonth, WhatDay, 16, 02, 00);

%%
while t <= 1%T_experiment
    %
    what_time_computing = sleep(Ts, what_time_computing);	
	disp('*============================================*')
    fprintf('*======== Control step number: %d =======* \n', t);
	currentTime = datetime('now', 'TimeZone', TimeZone );
	fprintf('Experiment started on: %s at time zone %s \n', datestr(currentTime,...
				'dd-mmm-yyyy HH:MM:SS'), TimeZone)   	
    disp('*============================================*')     
%     % Update room temperature measurement
%     y_cl(:, t) = read_value_from_influx("simulation_temperature", Ts);
%     fprintf('The last temperature measurement is: %d', y_cl(:, t));    
    
	% prbs signal
% 	u_cl(:,t) = round(rand);
	fprintf('Get input %d \n', u_cl(t));
	            
% 	% Pause before sending data to the low-level controller
% 	what_time_sending = sleep(Ts, what_time_sending);
	% Sending to the low-level controller
	write_value_to_influxdb(u_cl(t), "sysid_input");
	
	% Update the current iteration
	t = t + 1;                            
end
what_time_computing = sleep(Ts, what_time_computing);
disp('*============================================*')
fprintf('*======== Control step number: %d =======* \n', t);
disp('*============================================*')   
write_value_to_influx(2, "sysid_input");
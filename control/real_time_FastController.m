clearvars
clear first
close all

addpath(genpath('./parameters'))
addpath(genpath('../utilities'))

parameter = fast_controller_par();

current_time = datetime('now');
what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
parameter.date_exp_start = datenum(what_year, what_month, what_day, 17,59, 55);
LLController = FastController(parameter);

fprintf('Matlab started at %s \n', datestr(now))
%% Test run controller
try
	LLController.run_controller()
catch
	warning('Problem with LLController')
end
pause(60)
if LLController.mode == 0 % cooling
	LLController.write_setpoint_to_HP(22);
else
	LLController.write_setpoint_to_HP(20);
end


%%
save('./results/fast_01-1006_2021.mat','LLController','parameter')
%%
figure
hold on
plot(u_cl)
plot(w_cl(1,:))
plot(y_cl)

% %%
% % write_value_to_influxdb(20.0, 'HVAC',{'registers'},{'TEMP_MAND'});
% a = LLController.get_HP_room_temp()
% %%
% % write_value_to_influxdb(2.0, 'HP_Active_Power',{},{});
% a = LLController.get_HP_status()
% 
% %%
% LLController.get_control_input()
% % write_value_to_influxdb(20.0, 'HVAC',{'registers'},{'TEMP_MAND'});
% % write_value_to_influxdb(5.0, 'HP_Active_Power',{},{});
% LLController.create_input_profile()
% fprintf('Computed setpoint profile for chiller: \n'), disp(LLController.chiller_setpoint) 
% % Send data to Chiller
% current_time = datetime('now');
% what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
% what_time_to_send = datenum(what_year, what_month, what_day, 12, 14, 55);
% what_time_to_send = what_time_to_send - seconds_to_matlab_time(60);
% 
% for i = 1:LLController.Ts/LLController.send_time-1
% 	what_time_to_send = sleep(60, what_time_to_send);
%     LLController.write_setpoint_to_HP(LLController.chiller_setpoint(i));
%     disp('*============================================*')
%     fprintf('Setpoint to chiller: %d \n' ,LLController.chiller_setpoint(i)) 
%     fprintf('Time now is: %s \n' ,datestr(now))
%     
% end
% what_time_to_send = sleep(60, what_time_to_send);
% LLController.write_setpoint_to_HP(LLController.chiller_setpoint(end));
% disp('*============================================*')
% fprintf('Setpoint to chiller: %d \n' ,LLController.chiller_setpoint(end)) 
% fprintf('Time now is: %s \n' ,datestr(now))
% 
% LLController.get_control_input()
% % write_value_to_influxdb(20.0, 'HVAC',{'registers'},{'TEMP_MAND'});
% % write_value_to_influxdb(5.0, 'HP_Active_Power',{},{});
% LLController.create_input_profile()
% fprintf('Computed setpoint profile for chiller: \n'), disp(LLController.chiller_setpoint) 
% 
% 
% for i = 1:LLController.Ts/LLController.send_time-1
% 	what_time_to_send = sleep(60, what_time_to_send);
%     LLController.write_setpoint_to_HP(LLController.chiller_setpoint(i));
%     disp('*============================================*')
%     fprintf('Setpoint to chiller: %d \n' ,LLController.chiller_setpoint(i)) 
%     fprintf('Time now is: %s \n' ,datestr(now))
%     
% end
% what_time_to_send = sleep(60, what_time_to_send);
% LLController.write_setpoint_to_HP(LLController.chiller_setpoint(end));
% disp('*============================================*')
% fprintf('Setpoint to chiller: %d \n' ,LLController.chiller_setpoint(end)) 
% fprintf('Time now is: %s \n' ,datestr(now))
% 
% 
% what_time_to_send = sleep(60, what_time_to_send);
% LLController.write_setpoint_to_HP(22)
% disp('*============================================*')
% fprintf('Setpoint to chiller: %d \n' ,LLController.chiller_setpoint(end)) 
% fprintf('Time now is: %s \n' ,datestr(now))
% 
% 
% 
% %% Test run controller
% try
% 	LLController.run_controller()
% catch
% 	warning('Problem with LLController')
% end
% pause(60)
% LLController.write_setpoint_to_HP(22.5)
% 
% 
% 
% 

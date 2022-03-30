% This function retreives measurements, forecast and last weather conditions
% and uses them to compute the next control action while taking care of the
% controller sampling time.
% 
function run_controller(obj)
            disp('*============================================*')
			disp('*============================================*')
			fprintf('Experiment started on: %s \n' ,datestr(now))
	
			%% Start control iteration
            % Loop Starts
			% The LL controller starts at the desired exp starting time. This
			% explains the -900 due to the sleep function at the beginning of the loop.
			wall_clock_comput	= obj.date_exp_start - seconds_to_matlab_time(900);
			% TODO: pass this as a parameter
			while obj.i_iter <=  96*11 % 96*30
				%
				wall_clock_comput = sleep(obj.Ts, wall_clock_comput);	
    			%
                %% Read Text File
                input_time = obj.get_control_input();
                fprintf('Control input received from high level controller: '), disp(obj.control_input)

                %% Create Chiller sepoint profile
%                 if obj.i_iter == 1
%                     write_value_to_influxdb(20.0, 'HVAC',{'registers'},{'TEMP_MAND'});
%                     write_value_to_influxdb(2.0, 'HP_Active_Power',{},{});                   
%                 else
%                     write_value_to_influxdb(22.0, 'HVAC',{'registers'},{'TEMP_MAND'});
%                     write_value_to_influxdb(15.0, 'HP_Active_Power',{},{});                       
%                 end
                obj.create_input_profile();
                fprintf('Computed setpoint profile for chiller: \n'), disp(obj.chiller_setpoint)
				
				
				%% If MPC fails/crashes: Shift to a Default Profile
				if obj.i_iter > 1
					if isequal(input_time, obj.control_input_time)
						fprintf('Did not receive a new setpoint from MPC controller. Shifting to default profile'), disp(obj.default_profile)
						obj.chiller_setpoint = obj.default_profile;
					end
				end
				obj.control_input_time = input_time;
				
				obj.cache.control_input.value(:, obj.i_iter) = obj.control_input;
				obj.cache.control_input.time{obj.i_iter} = input_time;
				obj.cache.profile.value(:, obj.i_iter) = obj.chiller_setpoint;
				obj.cache.profile.time(:, obj.i_iter) = now();
                
                %% Send data to Chiller
				wall_clock_sender = now();
                for i = 1:obj.Ts/obj.send_time-1
                    obj.write_setpoint_to_HP(obj.chiller_setpoint(i));
                    disp('*============================================*')
                    fprintf('Setpoint to chiller: %d \n' ,obj.chiller_setpoint(i)) 
                    fprintf('Time now is: %s \n' ,datestr(now))
                    wall_clock_sender = sleep(60, wall_clock_sender);
                end
                obj.write_setpoint_to_HP(obj.chiller_setpoint(end));
                disp('*============================================*')
                fprintf('Setpoint to chiller: %d \n' ,obj.chiller_setpoint(end)) 
                fprintf('Time now is: %s \n' ,datestr(now))
   				%% Update the current iteration
                obj.i_iter	...
					= obj.i_iter + 1;
                
			end%%
			disp('*============================================*')
			disp('*============================================*')
			
			currentTime = datestr(datetime('now'),'dd-mmm-yyyy HH:MM:SS');
			fprintf('Experiment ended on: %s \n' ,currentTime)
			obj.date_exp_end = datenum(currentTime);		
end
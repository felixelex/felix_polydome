% This function retreives measurements, forecast and last weather conditions
% and uses them to compute the next control action while taking care of the
% controller sampling time.
%
function runController(hController)
			%
			disp('*============================================*')
			disp('*============================================*')
			fprintf('Experiment started on: %s \n' ,datestr(now))
	
			%% Start control iteration
            % Loop Starts
			currentTime = now(); hController.dateExpStart = datestr(currentTime);
			WhatYear = year(currentTime);WhatMonth = month(currentTime);WhatDay = day(currentTime);
			% Takes care of time drifting
			wallClock_Comput	= datenum(WhatYear, WhatMonth, WhatDay, 00, 59, 00);
			wallClock_Sending	= datenum(WhatYear, WhatMonth,WhatDay, 00, 59, 59);
			%
			while hController.iIter <= hController.iExpLength
				%
				wallClock_Comput = sleep(hController.Ts, wallClock_Comput);	
				%
                %% Update room temperature measurement
                hController.getMeasurement();
				%% Update prediction horizon for MPC controller
				% This corresponds to the predefined pred horizon in case
				% there are enough forecast for the dispatch plans.
				% Otherwise, it is set to the difference btw end of the day
				% and current time
				hController.updatePredHorizon()
                %% Update measurements and forecast of weather conditions
				hController.getForecast();
				%% Update the last measured disturbance for extern perturbation
				hController.getLastDisturbance();
				%
				disp('*============================================*')
				fprintf('*======== Control iteration number: %d =======* \n', hController.iIter);
				disp('*============================================*')
				disp('The last temperature measurement is: '); disp(hController.lastMeasurement);
				%
				%% Call the actual computation function for the controller 
				% (real-time controller should be implemented inside). The
				% result should be stored in .lastInput
				%
                hController.ControlFunction();
				%
				%% Update the current iteration
                hController.iIter	...
					= hController.iIter + 1;
				
				%% Pause before sending data to the low-level controller
				%
				wallClock_Sending = sleep(hController.iSamplingTime, wallClock_Sending);
				
				%% Sending to the low-level controller
				hController.sendToLowLevel()
				%
				fprintf('Data sent to the Sender at %s \n',datestr(datetime('now'),'dd-mmm-yyyy HH:MM:SS:FFF'))
				%
			end%%
			disp('*============================================*')
			disp('*============================================*')
			currentTime = datestr(datetime('now'),'dd-mmm-yyyy HH:MM:SS:FFF');
			currentDate = datestr(datetime('now'),'dd-mmm-yyyy');
			fprintf('Experiment ended on: %s \n' ,currentTime)
	
			hController.dateExpEnd = datestr(currentTime);
			
			%% Saving the results
			expName = strcat('Polydome', hACSTracking.strTypeBid, currentDate);
			save('./Results/', expName)
             % Make sure that the standard setpoint is implemented at the
             % end of the experiment
            hController.setStandardSetPoint()
		end


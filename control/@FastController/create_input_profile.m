function create_input_profile(obj)
%This function creates the input profile for sending it to the chiller at
%the fast controller's sampling rate.
    
    %% Control Input
    input = obj.control_input;
    mode = obj.mode;
	if mode == 0 % cooling
        input_max = 4.6;
        delta = -1.5;
%         obj.another_setpoint = 21;
		obj.another_setpoint = 21;
    else % heating
        input_max = 6;
        delta = 1.5;   
%         obj.another_setpoint = 25;
		obj.another_setpoint = 25;
    end
        
    
    %% On-Time
    % Fast controller sampling time
    send_time = obj.send_time;
    % Fast controller horizon
    N = obj.Ts/send_time;
    on_time = (input/input_max)*N;
    
	%% Chiller return air temperature
	return_temp = obj.get_HP_room_temp;	

	
    %% On / Off Profile
    
    profile = zeros(N,1);
    
	HP_status = obj.get_HP_status;
	
	if HP_status == 1
		disp('Chiller is on: \n') 
        % The chiller is already ON
        profile(1:round(on_time)) = 1;
    elseif HP_status == 0
		disp('Chiller is off: \n') 
        % The chiller is OFF
        profile(N-round(on_time)+1:end) = 1;
	end
	
	obj.cache.HP.status(:, obj.i_iter) = HP_status;
	obj.cache.HP.return_temp(:, obj.i_iter) = return_temp;
	obj.cache.HP.time(:, obj.i_iter) = now();
    
    %% Chiller/heater setpoint
    % OFF
    if mode == 0
        if return_temp<=24 % 24
            chiller_setpoint = 24*ones(N,1); 
        elseif return_temp<=26 %26
            chiller_setpoint = 26*ones(N,1); 
        else
            chiller_setpoint = 27*ones(N,1); % 27
        end        
    else
        if return_temp>=22 %22
            chiller_setpoint = 22*ones(N,1); 
        elseif return_temp>=20 %20
            chiller_setpoint = 20*ones(N,1); 
        else
            chiller_setpoint = 19*ones(N,1); %19
        end
    end
    
    chiller_setpoint(logical(profile)) = return_temp+delta;  % Chiller ON when profile = 1
    
    obj.chiller_setpoint = chiller_setpoint;
    
        
	
	%% Default setpoint profile
	% If the predictive controller crashes or fails to update the control
	% setpoint, the Fast controller shifts to a default setpint
	if mode == 0 %cooling
		obj.default_profile = 22*ones(N,1);  %24
	else
		obj.default_profile = 20*ones(N,1); %22
	end
	
end


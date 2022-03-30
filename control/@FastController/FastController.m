classdef FastController < Controller
	%% Fast Controller for controlling the chiller
	
	properties
		%
        control_input
        control_input_time
        chiller_setpoint
		default_profile
        fan_power % The power consumed by the fan
        send_time % The samping time of the fast controller
		cache % save historical data
        mode
        another_setpoint;
        %
	end%%
	%
	methods
		% Methods signature
		initialize_fast_controller(obj, parameter)
        str_date = get_control_input(obj)
		bOn = get_HP_status(obj)
		value = get_HP_room_temp(obj)
        create_input_profile(obj)
        write_setpoint_to_HP(obj, value)
        run_controller(obj)
        
		% Constructor
        function object = FastController(parameter)
			%
			object = object@Controller(parameter);
			%
			object.initialize_fast_controller(parameter)
            obj.another_setpoint = 20;
            obj.mode = 1;
		end%%
		%
	end%%
	%
end%%
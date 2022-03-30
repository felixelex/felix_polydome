classdef Controller < handle
	%CONTROLLER Basic controller class for Polydome exp
	%
	properties
		%
		create_at % Controller created at:
		date_exp_start % Start of the experiment
		date_exp_end % End of the experiment
		%
		str_type % Type of Object
		%
		Ts % Sampling time
		i_iter % Current iteration of the controller
		i_exp_length % Number of iteration for the controller
		last_measurement % Last measurment
		last_input % Last computed input
		%
		input_info
		output_info
	end%%
	
	methods
		get_measurement(obj, series_name)
		initialize_controller(obj, parameter)
        send_input_to_low_level(obj,input,mode) % Send new input to the low-level (fast controller)
		% Constructor
		function obj = Controller(parameter)
			obj.initialize_controller(parameter)
		end%%
	end
	
end


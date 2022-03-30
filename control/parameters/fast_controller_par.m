function parameter = fast_controller_par()
%Specify the parameters of the fast controller

	parameter = controller_par();
	parameter.str_ctr_type = 'fast_controller';
	
	% Remove the offset from the DB measurement
	parameter.fan_power = 1.67; %[kW]
    
    % Time step to send input to chiller (in seconds)
    parameter.send_time = 60;  % 1 minute
end


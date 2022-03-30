function initialize_controller(obj, parameter )
%INITIALIZATION Initialize all properties of the controller class
	%
	obj.Ts = parameter.Ts;
	obj.i_exp_length = parameter.i_exp_length;
	obj.i_iter = 1;
	obj.input_info = parameter.input_info;
	obj.output_info = parameter.output_info;
	obj.str_type = parameter.str_ctr_type;
	obj.create_at  = datestr(now()); 
    obj.date_exp_start  = parameter.date_exp_start; 
	%
end%%


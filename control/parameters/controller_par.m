function parameter = controller_par()
	%
	parameter.Ts = 60*15;
	parameter.i_exp_length = 24*3600/parameter.Ts;
	
	parameter.input_info.label  = 'Power';
	parameter.input_info.unit   = '[kW]';
	parameter.output_info.label = 'Indoor Temp';
	parameter.output_info.unit  = '[C^0]';
	parameter.str_ctr_type      = 'Controller';
end%%

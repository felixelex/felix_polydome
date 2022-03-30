function write_setpoint_to_HP(obj, value)
%GETHPSTATUS Get the current status of the HP
	%
	try%%
        if obj.mode == 0 % cooling
            value_cool = value*10;
            value_heat = obj.another_setpoint*10;
        else % heating
            value_heat = value*10;
            value_cool = obj.another_setpoint*10;
        end
        
		seriesname	= 'setpoint_cmd_heat';
		tag_keys	= {};
		tag_values	= {};
		write_value_to_influxdb( value_heat, seriesname, tag_keys, tag_values);
        
		seriesname	= 'setpoint_cmd_cool';
		tag_keys	= {};
		tag_values	= {};
		write_value_to_influxdb( value_cool, seriesname, tag_keys, tag_values);        
		%
	catch%
		error('Problem sending data to the HP');
		%
	end%%

end


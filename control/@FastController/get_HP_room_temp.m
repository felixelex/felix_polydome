function value = get_HP_room_temp(obj)
%GETHPSTATUS Get the current status of the HP
	%
	try%%
		% HP uses return temperature as room temperature 
		series_name	= 'HVAC';
		tag_key	= {'register'};
		tag_value	= {'TEMP_RIPR'};
		value = read_value_from_influxdb(series_name,	70, tag_key, tag_value);
		value = value/10.0;
		%
		%
	catch%
		error('Problem retrieving the internal temperature measured by the HP');
		%
	end
end%%
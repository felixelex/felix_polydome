function on_HP = get_HP_status( obj )
%GETHPSTATUS Get the current status of the HP
	%
	try%%
		% Use 
		series_name	= 'HP_Active_Power';
        value = read_value_from_influxdb(series_name, 70, {}, {},'Power');
		
        % On: HP_Active_Power > 3 kW
		if value > 3
			on_HP = 1;
		else
			on_HP = 0;
		end
		%
	catch%
		error('Problem checking the status of the HP');
		%
	end
end%%


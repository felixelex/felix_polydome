function write_value_time_to_influxdb(value, series_name, time)
% Write value with specific time
% !!! Important: use UTC time because influxdb only uses UTC time 
	try%%
		% Send temerature
% 		seriesname	= 'simulation_temperature';

		tag_keys	= {};
		tag_values	= {};
        
        % Timestamp generation
        TimeZone = 'UTC'; % Use UTC time zone 
        fprintf('Writing variable "%s" in InfluxDB, at time: %s at time zone %s \n', series_name, datestr(time,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      
               
        ts = round((time-datenum(1970,1,1))*86400); % Transfer t0 to time stamp structure in InfluxDB

        if numel(ts) ~= numel(value)
            error('Timestamp and values sequences should be with same length!')    
        end

        exit_state = write_series_to_influxdb(value, ts, series_name, tag_keys, tag_values );        
		%
	catch%
		error('Problem sending variable "%s" to the HP',series_name);
		%
	end%%
end


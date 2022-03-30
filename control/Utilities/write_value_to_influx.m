function write_value_to_influx(value, seriesname)
% Write value with current time
% !!! Important: use UTC time because influxdb only uses UTC time 
	try%%
		% Send temerature
% 		seriesname	= 'simulation_temperature';

		tag_keys	= {};
		tag_values	= {};
        
        % Timestamp generation
        TimeZone = 'UTC'; % Use UTC time zone 
        currentTime = datetime('now', 'TimeZone', TimeZone ); 
        t0 = datenum(currentTime); % Use UTC time for datenum, because InfluxDB only use UTC time
        fprintf('Writing variable "%s" in InfluxDB, current time: %s at time zone %s \n', seriesname, datestr(currentTime,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      
               
        ts = round((t0-datenum(1970,1,1))*86400); % Transfer t0 to time stamp structure in InfluxDB

        if numel(ts) ~= numel(value)
            error('Timestamp and values sequences should be with same length!')    
        end

        exit_state = writeSeriesToDatabase(value, ts, seriesname, tag_keys, tag_values );        
		%
	catch%
		error('Problem sending variable "%s" to the HP',seriesname);
		%
	end%%
end


function write_value_to_influxdb(value, series_name, tag_key,tag_value)
% Write value with current time
% !!! Important: use UTC time because influxdb only uses UTC time 
	try%%
        
        % If the tag pair is not given 
        if nargin < 4
            tag_key	= {};
            tag_value	= {};
        end
        
        % Timestamp generation
        TimeZone = 'UTC'; % Use UTC time zone 
        currentTime = datetime('now', 'TimeZone', TimeZone ); 
        t0 = datenum(currentTime); % Use UTC time for datenum, because InfluxDB only use UTC time
        fprintf('Writing variable "%s" in InfluxDB, current time: %s at time zone %s \n', series_name, datestr(currentTime,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)      
               
        ts = round((t0-datenum(1970,1,1))*86400); % Transfer t0 to time stamp structure in InfluxDB

        if numel(ts) ~= numel(value)
            error('Timestamp and values sequences should be with same length!')    
        end

        exit_state = write_series_to_influxdb(value, ts, series_name, tag_key, tag_value );        
		%
	catch%
		error('Problem sending variable "%s" to the HP',series_name);
		%
	end%%
end


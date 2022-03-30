function [value] = read_value_from_influxdb(series_name, Ts, tag_key,tag_value, field_value)
% Read mean value during [now()-Ts, now()]
	%
	try%%

        % If the field value is not given
		if nargin < 5
			field_value = 'value';
		end
		% If the tag pair is not given
        if nargin < 4
            tag_key	= {};
            tag_value	= {};
		end
		
		% First sensor
		[value, ~, time_stamp] = read_lastest_values_from_influxdb(	series_name,		...
												tag_key,		...
												tag_value,		...
												1,				...
												Ts, field_value);                                   
        if isnan(value)
            disp('!!!!! Extracted nan. Kept previous value. !!!!')
        else
            value = mean(value);
            fprintf('Reading a average variable "%s" in InfluxDB, from %s to %s at time zone UTC \n', series_name, ...
                datestr((time_stamp(end)/86400+datenum(1970,1,1)),'dd-mmm-yyyy HH:MM:SS'),...
                datestr((time_stamp(1)/86400+datenum(1970,1,1)),'dd-mmm-yyyy HH:MM:SS')) 
        end
	catch
		value = nan;
		disp('!!!!! Problem with extracting last Tindoor. !!!!')
	end%%


end

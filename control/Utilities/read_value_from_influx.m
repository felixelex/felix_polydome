function [value] = read_value_from_influx(seriesname, Ts)
% Read value with current time
	%
	try%%
		%
% 		seriesname	= {'simulation_temperature'}; %{'Temp_TSOL'};
		tag_keys	= {{}};
		tag_values	= {{}};
		% First sensor
		[value, ~, time_stamp] = readValuesFromDatabase(	seriesname,		...
												tag_keys,		...
												tag_values,		...
												1,				...
												Ts);                                   
        fprintf('Reading variable "%s" in InfluxDB, at time: %s at time zone UTC \n', seriesname, ...
            datestr((time_stamp/86400+datenum(1970,1,1)),...
            'dd-mmm-yyyy HH:MM:SS')) 
		if isnan(value)
			disp('!!!!! Extracted nan. Kept previous value. !!!!')
		end
	catch
		%
		disp('!!!!! Problem with extracting last Tindoor. !!!!')
	end%%


end

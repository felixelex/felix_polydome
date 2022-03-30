function get_measurement( obj,series_name)
%GETMEASUREMENT Retreives the last measurement for the internal temp
	Ts = obj.Ts;
	%
	try%%
		%
		value = read_value_from_influxdb(series_name, Ts);
		obj.last_measurement = value;
		if isnan(value)
			disp('!!!!! Extracted nan. Kept previous value. !!!!')
		end
	catch
		%
		disp('!!!!! Problem with extracting last Tindoor. !!!!')
	end%%


end


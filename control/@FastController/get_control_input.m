function input_time = get_control_input(obj)
% This function retreives the last control input
%
try%%
	% Read last control input from file written by the real-time MPC
	%
	strFilePath = './control_input';

	fid = fopen([strFilePath, filesep, 'control_input.txt'],'r');
	input_time = fgetl(fid);
    obj.control_input = str2double(fgetl(fid));
    obj.mode = str2double(fgetl(fid));
	fclose(fid);
    
    seriesname	= 'power_want';
    tag_keys	= {};
    tag_values	= {};
    write_value_to_influxdb( obj.control_input, seriesname, tag_keys, tag_values);    
	%
catch
	warning('Problem with the extraction of last control input')
end

end


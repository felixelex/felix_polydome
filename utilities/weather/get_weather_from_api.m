function [temp_cur,rad_cur,temp_pred,rad_pred] = get_weather_from_api(num_pred, start_time,end_time)
%GET_WEATHER_FROM_API: query current and future weather from api: tomorrow.io
% Input:
%       num_pred: number of prediction steps
%       start_time: ISO 8601 format like "2021-04-19T09:00:00Z"
%       end_time: ISO 8601 format like "2021-04-19T14:30:00Z"
% Output:
%       current and future weather
% More details: https://docs.tomorrow.io/reference/get-timelines


    options = weboptions();
%     start_time = '2021-04-19T09:00:00Z';
%     end_time = '2021-04-19T14:30:00Z';
    timestep = '15m'; % resolution: "1m", "5m", "15m", "30m", "1h", "1d" or "current"
    location = '46.52147, 6.56899';
    parameter = 'temperature,solarGHI';
    
    % If the start and end time is not given 
    if nargin < 2
        url = ['https://api.tomorrow.io/v4/timelines?location=' location ... 
            '&fields=' parameter '&units=metric&timesteps=' timestep ... 
            '&apikey=u7KrCzV5HfkeHcn8zU8wNUfAxBFDeh6g'];
    else
        url = ['https://api.tomorrow.io/v4/timelines?location=' location ...
            '&fields=' parameter '&units=metric&timesteps=' timestep ...
            '&startTime=' start_time '&endTime=' end_time... 
            '&apikey=u7KrCzV5HfkeHcn8zU8wNUfAxBFDeh6g'];
    end

    d = webread(url, options);
    %
    weather = d.data.timelines.intervals;
    for i = 1:num_pred+1
        temp(i) = weather(i).values.temperature;
        rad(i) = weather(i).values.solarGHI;
        timeline{i} = weather(i).startTime;
    end
    temp_cur = temp(1);
    rad_cur = rad(1);
    temp_pred = temp(2:end);
    rad_pred = rad(2:end);


%     %% write current weather into influxdb database
% 	try%%
%         write_value_time_to_influxdb(rad_cur, "solar_GHI", datenum(timeline{1}, 'yyyy-mm-ddTHH:MM:SS'));
%         write_value_time_to_influxdb(temp_cur, "temp_air", datenum(timeline{1}, 'yyyy-mm-ddTHH:MM:SS'));
% 
% 	catch
% 		%	
% 		disp('Problem with writing last disturbance in the influxdb')
% 	end%%
	
	
%     write_value_to_influxdb( temp_cur, 'temp_outside', tag_keys, tag_values);
%     write_value_to_influxdb( rad_cur, 'solar_rad', tag_keys, tag_values);
    
end


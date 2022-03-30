function get_last_disturbance( hController, time)
%GETFORECAST Retreive the forecast over pred horizon for the specified 
% quantity from meteomatics
    try%%
        TimeZone = 'UTC';
        currentTime = datetime('now', 'TimeZone', TimeZone ); 
        fprintf('Getting weather prediction current time: %s at time zone %s \n', datestr(currentTime,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)  
        % An exampe code for Meteomatics Weather API
        % https://www.mathworks.com/matlabcentral/fileexchange/63992-meteomatics-weather-api-connector
        user     = 'epfl_jicheng';   %insert-your-username-here';
        password = 'rzN7g1nUZ3KEt';  %insert-your-password-here';

        lat   = 46.520;
        lon   = 6.632;
        
        Ts = hController.Ts; % Building sampling time
        sample_time = 60; % Weather sampling time
        start_date  = now; % Could be anything like a datenum datenum(2016,12,24,12,3,2) <-> '2016-12-24T12:03:02Z';
        resolution  = 1/24/60; % 1 minute resolution % minimal: 1s
        end_date    = start_date + resolution*(Ts/sample_time); % period of 5 days

        parameters = 't_2m:C,direct_rad:W';  % Temperature and dew point at 2m ,diffuse_rad:W,global_rad:W

        [dn,data]=query_time_series_from_weather_api(user,password,start_date,resolution,end_date,parameters,lat,lon);
        Tair = mean(data(:,1));
        SolRad = mean(data(:,2));
            
        obj.last_dist.Tair = removeOutliers(Tair);
        obj.last_dist.SolRad = removeOutliers(SolRad);
        fprintf('Last air temperature: %d \n', Tair); 
        fprintf('Last radiation: %d \n', SolRad);

			
	catch
		%
		values = zeros(obj.N_pred);
		hController.last_dist.SolRad = values;

		values = 15*ones(obj.N_pred,1);
		hController.last_dist.Tair = values;
		disp('Problem with extracting forecast. Created a dummy forecast')
	end%%

	try%%
        write_value_time_to_influx(SolRad, "SolRad", time);
        write_value_time_to_influx(Tair, "Tair", time);

	catch
		%	
		disp('Problem with writing last disturbance in the influxdb')
	end%%

end


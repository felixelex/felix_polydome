function [y,u,w,m] = retreive_initial_data( t0, tf )
%Retreives all relevant data for Polydome
%   INPUT:	- t0: initial time instant in epoch time
%			- tf: final time instant in epoch time
%			- Ts: time sampling in seconds
%	OUTPUT:	- Data: structure containing all the information
	
	%% Time used for InfluxDB and meteomatics
	% UTC time zone in InfluxDB
	TimeZone = 'Europe/Zurich';
	currentTime = datetime('now', 'TimeZone', TimeZone );
	if isdst(currentTime)
		t0 = (t0-datenum(1970,1,1))*86400 - 2*3600;
		tf = (tf-datenum(1970,1,1))*86400 - 2*3600;
	else
		t0 = (t0-datenum(1970,1,1))*86400 - 1*3600;
		tf = (tf-datenum(1970,1,1))*86400 - 1*3600;		
	end


	%% Indoor temperature from zwave sensor
	[sensor_temp_1, ~, t_sensor_temp] = read_valuelist_from_influxdb('sensor_tem_3_temperature', {}, {}, t0, tf);
	sensor_temp_1 = removeOutliers(sensor_temp_1);
	[sensor_temp_2, ~, ~] = read_valuelist_from_influxdb('sensor_tem_4_temperature', {}, {}, t0,tf);
	sensor_temp_2 = removeOutliers(sensor_temp_2);
	[sensor_temp_3, ~, ~] = read_valuelist_from_influxdb('sensor_tem_6_temperature', {}, {}, t0, tf);
	sensor_temp_3 = removeOutliers(sensor_temp_3);
	[sensor_temp_4, ~, ~] = read_valuelist_from_influxdb('sensor_tem_7_temperature', {}, {}, t0,tf);
	sensor_temp_4 = removeOutliers(sensor_temp_4);    
	
    
	y.time = t_sensor_temp/86400+datenum(1970,1,1);
	y.value = mean([sensor_temp_1(1), sensor_temp_2(1), sensor_temp_3(1), sensor_temp_4(1)]);
% 	y.value = mean([sensor_temp_1(1), sensor_temp_2(1)]);
    
    
	y.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'sensor temperature', ...
            datestr(y.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(y.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	%% Power
	[power, ~, t_power] = read_valuelist_from_influxdb('HP_Active_Power', {}, {}, t0,tf, 'Power');
% 	power = removeOutliers(power);
	%
	u.time = t_power/86400+datenum(1970,1,1);
	u.value = mean(power)-2.35;
	u.unit = '[kW]';
	
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n','power', ...
            datestr(u.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(u.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		


    %% Weather data by tomorrow.io
    % A python script record weather every 5 miniutes
	[outside_temp, ~, t_outside_temp] = read_valuelist_from_influxdb('temp_air', {}, {}, t0,tf);
	outside_temp = removeOutliers(outside_temp);
	[solar_rad, ~, t_solar_rad] = read_valuelist_from_influxdb('solar_GHI', {}, {}, t0,tf);
	solar_rad = removeOutliers(solar_rad);	

 	
	% Outside temperature	
	data.air_temp.time = t_outside_temp/86400+datenum(1970,1,1);
	data.air_temp.value = outside_temp;
	data.air_temp.unit = '[C^0]';	
    

    
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'outside temperature', ...
            datestr(data.air_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.air_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	% Solar Radiation
	data.solar_GHI.time = t_solar_rad/86400+datenum(1970,1,1);
	data.solar_GHI.value = solar_rad/1000.0;
	data.solar_GHI.unit = '[kW/m^2]';

%     ratio = Ts/300;
%     data.solar_GHI.value = data.solar_GHI.value(1:ratio:end);
% 	data.solar_GHI.time = data.solar_GHI.time(1:ratio:end); 
    
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'solar radiation', ...
            datestr(data.solar_GHI.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.solar_GHI.time(end),'dd-mmm-yyyy HH:MM:SS')) 	
	w.time = data.solar_GHI.time;
	w.value = [data.air_temp.value(end);data.solar_GHI.value(end)];
 %% read mode   
		series_name	= 'HVAC';
		tag_key	= {'register'};
		tag_value	= {'INVERNO'};
        [mode, ~, t_mode] = read_valuelist_from_influxdb('HVAC', {'register'}, {'INVERNO'}, t0,tf);
        mode = removeOutliers(mode);
	
    
	m.time = t_mode/86400+datenum(1970,1,1);
	m.value = mode(1);
    
    
	m.unit = '[0:cooling;1:heating]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'mode', ...
            datestr(m.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(m.time(end),'dd-mmm-yyyy HH:MM:SS'))     
end%%


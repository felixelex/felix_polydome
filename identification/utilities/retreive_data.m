function data = retreive_data( t0, tf, Ts )
%Retreives all relevant data for Polydome
%   INPUT:	- t0: initial time instant in epoch time
%			- tf: final time instant in epoch time
%			- Ts: time sampling in seconds
%	OUTPUT:	- Data: structure containing all the information
	
	%% Time used for InfluxDB and meteomatics
	% UTC time zone in InfluxDB
	t0_influx = (t0-datenum(1970,1,1))*86400 - 1*3600;
	tf_influx = (tf-datenum(1970,1,1))*86400 - 1*3600;


	%% Indoor temperature from zwave sensor
	[sensor_temp_1, t_sensor_temp] = readSeriesFromDatabase('sensor_tem_3_temperature', {}, {}, t0_influx, tf_influx, 60);
	sensor_temp_1 = removeOutliers(sensor_temp_1);
	[sensor_temp_2, ~] = readSeriesFromDatabase('sensor_tem_4_temperature', {}, {}, t0_influx, tf_influx, 60);
	sensor_temp_2 = removeOutliers(sensor_temp_2);
	[sensor_temp_3, ~] = readSeriesFromDatabase('sensor_tem_6_temperature', {}, {}, t0_influx, tf_influx, 60);
	sensor_temp_3 = removeOutliers(sensor_temp_3);
	[sensor_temp_4, ~] = readSeriesFromDatabase('sensor_tem_7_temperature', {}, {}, t0_influx, tf_influx, 60);
	sensor_temp_4 = removeOutliers(sensor_temp_4);
	
    
	data.sensor_temp.time = t_sensor_temp/86400+datenum(1970,1,1);
	data.sensor_temp.value = mean([sensor_temp_1, sensor_temp_2, sensor_temp_3, sensor_temp_4]')';
% 	data.sensor_temp.value = mean([sensor_temp_3, sensor_temp_4]')';
% 	data.sensor_temp.value = mean([sensor_temp_1, sensor_temp_2]')';
    
    ratio = Ts/60;
    data.sensor_temp.value = data.sensor_temp.value(1:ratio:end);
	data.sensor_temp.time = data.sensor_temp.time(1:ratio:end);   
    
	data.sensor_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'sensor temperature', ...
            datestr(data.sensor_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.sensor_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	%% Power
	[power, t_power] = readSeriesFromDatabase('HP_Active_Power', {}, {}, t0_influx, tf_influx, 300,'Power');
	power = removeOutliers(power);
	%
	data.power.time = t_power/86400+datenum(1970,1,1);
	data.power.value = power;
	data.power.unit = '[kW]';
	
	ratio = Ts/300;
	if ratio~=1
		data.power.value	= mean(reshape(data.power.value(1:end), ratio, []))';
		data.power.time = data.power.time(1:ratio:end);
	end
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n','power', ...
            datestr(data.power.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.power.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	%% Mode
	[mode, t_mode] = readSeriesFromDatabase('HVAC', {'register'}, {'INVERNO'}, t0_influx, tf_influx, Ts);
	mode = removeOutliers(mode);
	
	data.mode.time = t_mode/86400+datenum(1970,1,1);
	data.mode.value = mode;
	data.mode.unit = '1:winter;0:summer';	
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'season mode', ...
            datestr(data.mode.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.mode.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
		
	%% Setpoint winter
	[setpoint, t_setpoint] = readSeriesFromDatabase('HVAC', {'register'}, {'SET_TEMP_I_1'}, t0_influx, tf_influx, Ts);
	setpoint = removeOutliers(setpoint);
	
	data.setpoint_winter.time = t_setpoint/86400+datenum(1970,1,1);
	data.setpoint_winter.value = setpoint;
	data.setpoint_winter.unit = '[C^0]';
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'setpoint winter', ...
            datestr(data.setpoint_winter.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.setpoint_winter.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
	%% Setpoint summer
	[setpoint, t_setpoint] = readSeriesFromDatabase('HVAC', {'register'}, {'SET_TEMP_E_1'}, t0_influx, tf_influx, Ts);
	setpoint = removeOutliers(setpoint);
	
	data.setpoint_summer.time = t_setpoint/86400+datenum(1970,1,1);
	data.setpoint_summer.value = setpoint;
	data.setpoint_summer.unit = '[C^0]';	
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'setpoint summer', ...
            datestr(data.setpoint_summer.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.setpoint_summer.time(end),'dd-mmm-yyyy HH:MM:SS')) 

	
	%% Supply temperature
	[supply_tem, t_supply_tem] = readSeriesFromDatabase('HVAC', {'register'}, {'TEMP_MAND'}, t0_influx, tf_influx, Ts);
	supply_tem = removeOutliers(supply_tem);
	
	data.supply_temp.time = t_supply_tem/86400+datenum(1970,1,1);
	data.supply_temp.value = supply_tem;
	data.supply_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'supply temperature', ...
            datestr(data.supply_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.supply_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
	%% Return temperature
	[return_temp, t_return_temp] = readSeriesFromDatabase('HVAC', {'register'}, {'TEMP_RIPR'}, t0_influx, tf_influx, Ts);
	return_temp = removeOutliers(return_temp);
	
	data.return_temp.time = t_return_temp/86400+datenum(1970,1,1);
	data.return_temp.value = return_temp;
	data.return_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'return temperature', ...
            datestr(data.return_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.return_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	

    %% Weather data by tomorrow.io
    % A python script record weather every 5 miniutes
	[outside_temp, t_outside_temp] = readSeriesFromDatabase('temp_air', {}, {}, t0_influx, tf_influx, 300);
	outside_temp = removeOutliers(outside_temp);
	[solar_rad, t_solar_rad] = readSeriesFromDatabase('solar_GHI', {}, {}, t0_influx, tf_influx, 300);
	solar_rad = removeOutliers(solar_rad);	

 	
	% Outside temperature	
	data.air_temp.time = t_outside_temp/86400+datenum(1970,1,1);
	data.air_temp.value = outside_temp;
	data.air_temp.unit = '[C^0]';	
    
    ratio = Ts/300;
    data.air_temp.value = data.air_temp.value(1:ratio:end);
	data.air_temp.time = data.air_temp.time(1:ratio:end);   
    
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'outside temperature', ...
            datestr(data.air_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.air_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	% Solar Radiation
	data.solar_GHI.time = t_solar_rad/86400+datenum(1970,1,1);
	data.solar_GHI.value = solar_rad;
	data.solar_GHI.unit = '[kW/m^2]';

    ratio = Ts/300;
    data.solar_GHI.value = data.solar_GHI.value(1:ratio:end);
	data.solar_GHI.time = data.solar_GHI.time(1:ratio:end); 
    
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'solar radiation', ...
            datestr(data.solar_GHI.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.solar_GHI.time(end),'dd-mmm-yyyy HH:MM:SS')) 	
	

end%%


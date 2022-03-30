function data = retreive_data( t0, tf, Ts )
%Retreives all relevant data for Polydome
%   INPUT:	- t0: initial time instant in epoch time
%			- tf: final time instant in epoch time
%			- Ts: time sampling in seconds
%	OUTPUT:	- Data: structure containing all the information
	
	%% Time used for InfluxDB and meteomatics
	% UTC time zone in InfluxDB
	t0_influx = (t0-datenum(1970,1,1))*86400 - 2*3600;
	tf_influx = (tf-datenum(1970,1,1))*86400 - 2*3600;
		
	
	%% Power [kW]
	[power, t_power] = readSeriesFromDatabase('HP_Active_Power', {}, {}, t0_influx, tf_influx, Ts, 'Power');
	power = removeOutliers(power);
	%
	data.power.time = t_power/86400+datenum(1970,1,1);
	data.power.value = power;
	data.power.unit = '[kW]';
	
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
	
		
	%% Setpoint heating [C^0*10] - This value should be divided by 10
	[setpoint, t_setpoint] = readSeriesFromDatabase('HVAC', {'register'}, {'SET_TEMP_I_1'}, t0_influx, tf_influx, Ts);
	setpoint = removeOutliers(setpoint);
	
	data.setpoint_heat.time = t_setpoint/86400+datenum(1970,1,1);
	data.setpoint_heat.value = setpoint/10.0;
	data.setpoint_heat.unit = '[C^0]';
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'setpoint winter', ...
            datestr(data.setpoint_heat.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.setpoint_heat.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
	%% Setpoint cooling  [C^0*10] - This value should be divided by 10
	[setpoint, t_setpoint] = readSeriesFromDatabase('HVAC', {'register'}, {'SET_TEMP_E_1'}, t0_influx, tf_influx, Ts);
	setpoint = removeOutliers(setpoint);
	
	data.setpoint_cool.time = t_setpoint/86400+datenum(1970,1,1);
	data.setpoint_cool.value = setpoint/10.0;
	data.setpoint_cool.unit = '[C^0]';	
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'setpoint summer', ...
            datestr(data.setpoint_cool.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.setpoint_cool.time(end),'dd-mmm-yyyy HH:MM:SS')) 

	
	%% Supply temperature  [C^0*10] - This value should be divided by 10
	[supply_tem, t_supply_tem] = readSeriesFromDatabase('HVAC', {'register'}, {'TEMP_MAND'}, t0_influx, tf_influx, Ts);
	supply_tem = removeOutliers(supply_tem);
	
	data.supply_temp.time = t_supply_tem/86400+datenum(1970,1,1);
	data.supply_temp.value = supply_tem/10.0;
	data.supply_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'supply temperature', ...
            datestr(data.supply_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.supply_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
	%% Return temperature C^0*10 - [C^0*10] - This value should be divided by 10
	[return_temp, t_return_temp] = readSeriesFromDatabase('HVAC', {'register'}, {'TEMP_RIPR'}, t0_influx, tf_influx, Ts);
	return_temp = removeOutliers(return_temp);
	
	data.return_temp.time = t_return_temp/86400+datenum(1970,1,1);
	data.return_temp.value = return_temp/10.0;
	data.return_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'return temperature', ...
            datestr(data.return_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.return_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 

	%% Supply air flow rate [(m3/h)/10] - This value should be multiplied by 10
	[supply_flow_rate, t_supply_flow_rate] = readSeriesFromDatabase('HVAC', {'register'}, {'PORTATA_ARIA_Return'}, t0_influx, tf_influx, Ts);
	supply_flow_rate = removeOutliers(supply_flow_rate);
	
	data.supply_flow_rate.time = t_supply_flow_rate/86400+datenum(1970,1,1);
	data.supply_flow_rate.value = supply_flow_rate*10.0;
	data.supply_flow_rate.unit = '[m3/h]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'return temperature', ...
            datestr(data.supply_flow_rate.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.supply_flow_rate.time(end),'dd-mmm-yyyy HH:MM:SS'))
		
	%% Indoor temperature from zwave sensor
	[sensor_temp_1, t_sensor_temp] = readSeriesFromDatabase('sensor_tem_3_temperature', {}, {}, t0_influx, tf_influx, Ts);
	sensor_temp_1 = removeOutliers(sensor_temp_1);
	[sensor_temp_2, ~] = readSeriesFromDatabase('sensor_tem_4_temperature', {}, {}, t0_influx, tf_influx, Ts);
	sensor_temp_2 = removeOutliers(sensor_temp_2);
	[sensor_temp_3, ~] = readSeriesFromDatabase('sensor_tem_6_temperature', {}, {}, t0_influx, tf_influx, Ts);
	sensor_temp_3 = removeOutliers(sensor_temp_3);
	[sensor_temp_4, ~] = readSeriesFromDatabase('sensor_tem_7_temperature', {}, {}, t0_influx, tf_influx, Ts);
	sensor_temp_4 = removeOutliers(sensor_temp_4);
		
	data.sensor_temp.time = t_sensor_temp/86400+datenum(1970,1,1);
	data.sensor_temp.value = {sensor_temp_1, sensor_temp_2, sensor_temp_3, sensor_temp_4};
	data.sensor_temp.position = {'control room top','entrance room top', 'control room wall', 'entrance room wall' };
	data.sensor_temp.unit = '[C^0]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'sensor temperature', ...
            datestr(data.sensor_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.sensor_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	
	
    %% Weather data by tomorrow.io
	[outside_temp, t_outside_temp] = readSeriesFromDatabase('temp_air', {}, {}, t0_influx, tf_influx, Ts);
% 	outside_temp = removeOutliers(outside_temp);
	[solar_rad, t_solar_rad] = readSeriesFromDatabase('solar_GHI', {}, {}, t0_influx, tf_influx, Ts);
% 	solar_rad = removeOutliers(solar_rad);	

 	
	% Outside temperature	
	data.outside_temp.time = t_outside_temp/86400+datenum(1970,1,1);
	data.outside_temp.value = outside_temp;
	data.outside_temp.unit = '[C^0]';	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'outside temperature', ...
            datestr(data.outside_temp.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.outside_temp.time(end),'dd-mmm-yyyy HH:MM:SS')) 
		
	% Solar Radiation
	data.solar_rad.time = t_solar_rad/86400+datenum(1970,1,1);
	data.solar_rad.value = solar_rad;
	data.solar_rad.unit = '[kW/m^2]';
	
	fprintf('Reading variable "%s" in InfluxDB, from time %s to time %s at time zone UTC \n', 'solar radiation', ...
            datestr(data.solar_rad.time(1),'dd-mmm-yyyy HH:MM:SS'),...
			datestr(data.solar_rad.time(end),'dd-mmm-yyyy HH:MM:SS')) 
	%
end%%


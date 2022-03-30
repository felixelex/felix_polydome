function [exp] = resample(exp,ratio)
%RESAMPLE
%   
	exp.power.value	= mean(reshape(exp.power.value(1:end), ratio, []))';
	exp.power.time = exp.power.time(1:ratio:end);
	%
	exp.mode.value = exp.mode.value(1:ratio:end);
	exp.mode.time = exp.mode.time(1:ratio:end);
	
	exp.setpoint_winter.value = exp.setpoint_winter.value(1:ratio:end);
	exp.setpoint_winter.time = exp.setpoint_winter.time(1:ratio:end);
    
    exp.setpoint_summer.value = exp.setpoint_summer.value(1:ratio:end);
	exp.setpoint_summer.time = exp.setpoint_summer.time(1:ratio:end);
    
    exp.supply_temp.value = exp.supply_temp.value(1:ratio:end);
	exp.supply_temp.time = exp.supply_temp.time(1:ratio:end);
    
    exp.return_temp.value = exp.return_temp.value(1:ratio:end);
	exp.return_temp.time = exp.return_temp.time(1:ratio:end);
    
    exp.sensor_temp.value = exp.sensor_temp.value(1:ratio:end);
	exp.sensor_temp.time = exp.sensor_temp.time(1:ratio:end);
    
    exp.air_temp.value = exp.air_temp.value(1:ratio:end);
	exp.air_temp.time = exp.air_temp.time(1:ratio:end);   
    
    exp.solar_GHI.value = exp.solar_GHI.value(1:ratio:end);
	exp.solar_GHI.time = exp.solar_GHI.time(1:ratio:end);      
end


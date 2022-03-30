function exp  = create_iddata( exp,  T_old, T_new)
% creates iddata struct for system identification purposes
%   INPUT:	- Exp: A structure containing all the data of the exp
%			- T_old: Original time sampling of the data
%			- T_new: New time sampling
%	OUTPUT:	- Exp: iddata containing the data
	%% Resample data
    if  T_old ~=T_new
        exp = resample_data( exp, T_old, T_new );
    end
	%% Create Iddata
	fan_baseline = 2.35; % Fan is always on. 
	%% Pre-process power:
	power = exp.power.value;
	power = power - fan_baseline;
	%% Pre-process temp
	exp = iddata(	exp.sensor_temp.value,		...
				[power,...
				exp.air_temp.value/1.0,	...
				exp.solar_GHI.value/1000.0], T_new);
	exp.InputName	= {'power', 'air_temp', 'solar_rad'};
	exp.OutputName = {'sensor_temp'};
	%
end%%


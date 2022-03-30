function exp  = create_iddata( exp,  T_old, T_new)
%CREATEIDDATA creates iddata struct for system identification purposes
%   INPUT:	- Exp: A structure containing all the data of the exp
%			- T_old: Original time sampling of the data
%			- T_new: New time sampling
%	OUTPUT:	- Exp: iddata containing the data
	%% Resample data
	exp = resample_data( exp, T_old, T_new );
	%% Create Iddata
	FanBaseline = 1.67; % Fan is always on. 
	%% Pre-process power:
	% 1) scale to kW and remove fan consumption
	% 2) cap to 1 compressor to remove non-linear effects
	Power = exp.Power.values/10^3;
	Power = min(Power, 6.3);
	Power = Power -FanBaseline;
	%% Pre-process temp
	exp = iddata(	exp.LakeTemp.values,		...
				[Power,...
				exp.OutsideTemp.values,	...
				exp.SolRad.values/10^3], T_new);
	exp.InputName	= {'Power', 'Tout', 'SolRad'};
	exp.OutputName = {'Tind'};
	%
end%%


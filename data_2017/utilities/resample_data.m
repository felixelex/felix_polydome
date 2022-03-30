function Data  = resample_data( Data, T_old, T_new )
%RESAMPLE_DATA downsamples the identification data to another Ts
%   Detailed explanation goes here
	%
	ratio = T_new/T_old;
	n = length(Data.Power.values);
	while ~rem(n , (T_new/T_old)) == 0
		n = n-1;
	end
	
	if ratio >1
	Data.Power.values	= mean(reshape(Data.Power.values(1:n), ratio, []))';
	Data.Power.time		= mean(reshape(Data.Power.time(1:n), ratio, []))';
	%
	Data.Setpoint.values	= mean(reshape(Data.Setpoint.values(1:n), ratio, []))';
	Data.Setpoint.time		= mean(reshape(Data.Setpoint.time(1:n), ratio, []))';
	
	Data.OutsideTemp.values = mean(reshape(Data.OutsideTemp.values(1:n), ratio, []))';
	Data.OutsideTemp.time = mean(reshape(Data.OutsideTemp.time(1:n), ratio, []))';
	
	Data.SupplyTemp.values	= mean(reshape(Data.SupplyTemp.values(1:n), ratio, []))';
	Data.SupplyTemp.time	= mean(reshape(Data.SupplyTemp.time(1:n), ratio, []))';
	
	Data.InsideTemp.values	= Data.InsideTemp.values(1:ratio:n);
	Data.InsideTemp.time	= Data.InsideTemp.time(1:ratio:n);
	
	Data.LakeTemp.values	= Data.LakeTemp.values(1:ratio:n);
	Data.LakeTemp.time		= Data.LakeTemp.time(1:ratio:n);
	
	Data.SolRad.values		= Data.SolRad.values(1:ratio:n);
	Data.SolRad.time		= Data.SolRad.time(1:ratio:n);
	
	Data.SolRad_DA.values	= mean(reshape(Data.SolRad_DA.values(1:n), ratio, []))';
	Data.SolRad_DA.time		= mean(reshape(Data.SolRad_DA.time(1:n), ratio, []))';
	end
end%%


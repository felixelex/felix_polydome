function iTimeShift = seconds_to_matlab_time(iSeconds)
%SECONDTOMATTIME Transform seconds to mat time
% datenum: 1 unit = 1 day
	oneSecInMatTime = datenum(2001,12,19,18,0,1)- datenum(2001,12,19,18,0,0);
	iTimeShift = iSeconds*oneSecInMatTime;
	%
end%%


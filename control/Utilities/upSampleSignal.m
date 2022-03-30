%% UPSAMPLESIGNAL(Signal, Ts_new, Ts_old)
% It upsamples a signal from a sampling time of Ts_old to Ts_new by applying
% a zero-holder

function Signal_up = upSampleSignal(Signal, Ts_old, Ts_new)
	%
	if size(Signal, 2) < size(Signal,1)
		Signal = Signal';
	end
	
	Signal_up = repmat(Signal, Ts_old/Ts_new, 1); 
	Signal_up = Signal_up(:);
	%
end%%
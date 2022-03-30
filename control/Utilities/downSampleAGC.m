function [outputAGC] = downSampleAGC(AGC, oldTs, newTs)
% This function converts the given AGC signal to a slower frequency taking
% averages.

% Inputs:
% AGC: The given AGC signal (size: Nx1)
% oldTs: Sampling time of the given AGC in minutes
% newTs: Sampling time of the output AGC in minutes

% Output:
% outputAGC: downsample AGC signal

%% Error Catching
if floor(newTs/oldTs) ~= newTs/oldTs
    error('The ratio of newTs and oldTs is a rational number !')
end

%%
if newTs/oldTs > 1
    % down sample ratio
    downSample = newTs/oldTs;
    % converting to average AGC
    outputAGC = mean(reshape(AGC,downSample,[]));
    outputAGC = outputAGC';

elseif newTs/oldTs == 1
    outputAGC = AGC;
end

end


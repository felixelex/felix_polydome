function [rr, mm] = intraDayPolicyGen(a, aReal, parameters)
% This function processes the ACS signal using the intraDay policy to
% generate intraday transaction and the resulting residual tracking signal.
% Inputs:
% - a:          Scenarios of the ACS signal (dimention: L x Ns)
% - aReal:      The real ACS signal needed to be processed. It can be a
%               single signal or many (dimention: L x 1, or L x Ns)
% - parametrs:  A structure containing all the parameters of the intraDay
%               policy.
%
% Outputs:
% - rr:          Residual tracking signal (dimention - same as aReal)
% - mm:          Intraday control actions (dimention - same as aReal)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parameters

% Total length of the signals
L = parameters.L;
% Intraday transaction period (after this time the intraday transaction
% algorithm updates)
intraDayPeriod = parameters.intraDayPeriod;
% Offset for intraday transaction (after this time the first intraday
% transaction takes place)
offset = parameters.offset;
% Horizon for intraday transactions
N = parameters.N;
% Number of time steps for which intraday transaction are fixed.
M = parameters.M;

% Numbero of ACS scenarios
Ns = size(a,2);
% Number of real ACS signals
NsReal = size(aReal,2);

%% Intraday Algorithm

mm = [];
for j=1:NsReal
    
    % real ACS signal
    aR = aReal(:,j);

   % Initializing variables
    % intraday transaction
    m = zeros(L,1);
    
    % initial value of the index
    i = 0 + offset;
%     for counter = 1:(L/intraDayPeriod)-2    % hard coded == need to change
    for counter = 1:(L/intraDayPeriod)-4-offset    % hard coded == need to change
		
        % update cumulative sum of the residual tracking signal
        rHat = sum(aR(1:i) + m(1:i));

        % Expectation of rHat over the horizon
%         mStar = -( rHat + mean(sum(a(i:i+N,:))) + sum(m(i+1:i+N-M)) );
        mStar = -( rHat + mean(sum(a(i+1:i+N,:))) + sum(m(i+1:i+N-M)) );
    %     mStar = -( rHat + sum(m(i+1:i+N-M)) );

        % update the intraday transction
        m(i+N-M+1:i+N) = mStar/M;

        % update index
        i = i + intraDayPeriod;
    end
    
    mm = [mm,m];
end
    
% Residual Tracking signal
rr = mm + aReal;


end


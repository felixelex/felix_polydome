function [AGCSen, scenSelect] = generateAGCScenarios(AGC, Ns, n, Nv)
% This function randomly generates AGC scenarios from the given data.

% Inputs:
% AGC: The AGC signal data (size: Nx1)
% Ns: Required number of scenarios
% Nv: Scenario for the validation
% n: number of data points in one scenario

% Output:
% AGCSen: Randomly generated AGC scenarios

%% Total number of possible scenarios
totalScenarios = length(AGC)/n;

%% Error Catching
if floor(totalScenarios) ~= totalScenarios
    error('Total data points are not divisible by n - Incorrect value of n')
end

if Ns+Nv > totalScenarios
    error('Number of scenarios cannot be grater than maximum possible - Incorrect value of Ns')
end

%% Random scenario generationtic
% randomly selecting Ns days (scenarios)
% Scenario 96 is corrupted!

rng(now)
scenSelect = randperm(totalScenarios, Ns+Nv);



%% Total scenarios generation
AGCSen = reshape(AGC, n, totalScenarios);

%% Selecting randomly Ns scenarios
AGCSen = AGCSen(:,scenSelect);

end


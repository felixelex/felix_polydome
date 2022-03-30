clear 
clc
Hankel_u_cool = [];
Hankel_y_cool = [];
Hankel_w_cool = [];
%%
clear exp; clear exp6
load('./data/exp11_15min_cool')
exp = exp11;
T = 431;
% Build y, u vector for Hankel matrix
y = exp.sensor_temp.value(1:T+1,:)';
u = exp.power.value(1:T,:)'-2.35;
w = [exp.air_temp.value(1:T,:)';exp.solar_GHI.value(1:T,:)'/1000.0];

%
% Load parameters
ny = 1; nu = 1; nw = 2;
nx = 3; % nx estimated by system ID, used for persistent exciting order check
N_ini = 10; N_pred = 10; 
% T = T_new;

% --- Compute Hankel matrix for u and w ---
Hankel_row = N_ini + N_pred;
Hankel_col = T - Hankel_row + 1;
if Hankel_row+Hankel_col-1 ~= size(u,2)
    error("The length of u is wrong!")
end
if Hankel_row+Hankel_col-1 ~= size(w,2) 
    error("The length of w is wrong!")
end    

Hankel_u = zeros(nu*Hankel_row, Hankel_col);
Hankel_w = zeros(nw*Hankel_row, Hankel_col);
for i = 1:Hankel_row
    Hankel_u((i-1)*nu+1:i*nu,:) = u(:,i:i+Hankel_col-1);
    Hankel_w((i-1)*nw+1:i*nw,:) = w(:,i:i+Hankel_col-1);
end

% --- Compute Hankel matrix for y ---
Hankel_row = N_ini + N_pred + 1; 
Hankel_col = T - Hankel_row + 2;
if Hankel_row+Hankel_col-1 ~= size(y,2)
    error("The length of y is wrong!")
end

Hankel_y = zeros(ny*Hankel_row, Hankel_col);
for i = 1:Hankel_row
    Hankel_y((i-1)*ny+1:i*ny,:) = y(:,i:i+Hankel_col-1);
end

%
Hankel_u_cool = [Hankel_u_cool, Hankel_u];
Hankel_y_cool = [Hankel_y_cool, Hankel_y];
Hankel_w_cool = [Hankel_w_cool, Hankel_w];
%%
Hankel_u_cool = [Hankel_u_cool(:,1:21),Hankel_u_cool(:,81:120),Hankel_u_cool(:,177:216),...
    Hankel_u_cool(:,273:312), Hankel_u_cool(:,369:408)];
Hankel_w_cool = [Hankel_w_cool(:,1:21),Hankel_w_cool(:,81:120),Hankel_w_cool(:,177:216),...
    Hankel_w_cool(:,273:312), Hankel_w_cool(:,369:408)];
Hankel_y_cool = [Hankel_y_cool(:,1:21),Hankel_y_cool(:,81:120),Hankel_y_cool(:,177:216),...
    Hankel_y_cool(:,273:312), Hankel_y_cool(:,369:408)];
%%
save('hankel_cool_0608.mat','Hankel_u_cool','Hankel_y_cool','Hankel_w_cool');
%%
clear
Hankel_uw_cool = [];
Hankel_y_cool = [];
%%
clear exp; clear exp6
load('./data/exp11_15min_cool')
T = 431;
exp = exp11;
% Build y, u vector for Hankel matrix
y = exp.sensor_temp.value(1:T+1,:)'; %!!!
u = exp.power.value(1:T,:)'-2.35 ;   %!!!
w = [exp.air_temp.value(1:T,:)'/1.0;exp.solar_GHI.value(1:T,:)'/1000.0]; %!!!
u = [u;w];
%
% Load parameters
ny = 1; nu = 3; 
nx = 3; % nx estimated by system ID, used for persistent exciting order check
N_ini = 10; N_pred = 10; 
% T = T_new;

% --- Compute Hankel matrix for u and w ---
Hankel_row = N_ini + N_pred;
Hankel_col = T - Hankel_row + 1;
if Hankel_row+Hankel_col-1 ~= size(u,2)
    error("The length of u is wrong!")
end
if Hankel_row+Hankel_col-1 ~= size(w,2) 
    error("The length of w is wrong!")
end    

Hankel_uw = zeros(nu*Hankel_row, Hankel_col);

for i = 1:Hankel_row
    Hankel_uw((i-1)*nu+1:i*nu,:) = u(:,i:i+Hankel_col-1);

end

% --- Compute Hankel matrix for y ---
Hankel_row = N_ini + N_pred + 1; 
Hankel_col = T - Hankel_row + 2;
if Hankel_row+Hankel_col-1 ~= size(y,2)
    error("The length of y is wrong!")
end

Hankel_y = zeros(ny*Hankel_row, Hankel_col);
for i = 1:Hankel_row
    Hankel_y((i-1)*ny+1:i*ny,:) = y(:,i:i+Hankel_col-1);
end

%
Hankel_uw_cool = [Hankel_uw_cool, Hankel_uw];
Hankel_y_cool = [Hankel_y_cool, Hankel_y];
%%
Hankel_uw_cool = [Hankel_uw_cool(:,1:21),Hankel_uw_cool(:,81:120),Hankel_uw_cool(:,177:216),...
    Hankel_uw_cool(:,273:312), Hankel_uw_cool(:,369:408)];
Hankel_y_cool = [Hankel_y_cool(:,1:21),Hankel_y_cool(:,81:120),Hankel_y_cool(:,177:216),...
    Hankel_y_cool(:,273:312), Hankel_y_cool(:,369:408)];
%%
Hankel_uw_cool = [Hankel_uw_cool(:,1:21),Hankel_uw_cool(:,81:120),Hankel_uw_cool(:,177:216),...
    Hankel_uw_cool(:,273:302)];
Hankel_y_cool = [Hankel_y_cool(:,1:21),Hankel_y_cool(:,81:120),Hankel_y_cool(:,177:216),...
    Hankel_y_cool(:,273:302)];
%%
save('hankel_cool_prediction_200.mat','Hankel_uw_cool','Hankel_y_cool');
clear 
close 
%%

load('./data/exp_3103_0904', 'exp')

%% Sensors measure room temperature
% mode: 1: winterm 0: summer
figure
hold on
yyaxis left
plot(exp.sensor_temp.time+2/24,exp.power.value);
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value);
datetick('x','mm/dd/yy HH:MM');
%% HP Power 
figure
hold on
yyaxis left
plot(exp.power.time+2/24,exp.power.value);
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value);
datetick('x','mm/dd/yy HH:MM');
%% HP setpoint
figure
hold on
yyaxis left
plot(exp.setpoint_summer.time+2/24,exp.setpoint_summer.value/10.0,'b');
plot(exp.setpoint_winter.time+2/24,exp.setpoint_winter.value/10.0,'r');
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value,'g');
legend({'summer','winter'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');

%%
figure
hold on

plot(exp.supply_temp.time+2/24,exp.supply_temp.value/10.0,'r');
plot(exp.return_temp.time+2/24,exp.return_temp.value/10.0,'g');
plot(exp.sensor_temp.time+2/24,exp.sensor_temp.value,'b');
legend({'supply','return', 'sensor'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');

%%
exp = exp1;
figure
hold on
yyaxis left
plot(exp.outside_temp.time+2/24,exp.outside_temp.value,'r');
yyaxis right
plot(exp.solar_rad.time+2/24,exp.solar_rad.value,'b');
datetick('x','mm/dd/yy HH:MM');
legend({'Temperature','Direct radiation'},'FontSize',18)
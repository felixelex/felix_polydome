clear 
close 
addpath(genpath('../utilities'))
%%
Ts = 300;
t0 = datenum(2021,04,16,12,00,00);
tf = datenum(2021,04,28,00,00,00);
exp = retreive_data( t0, tf, Ts );
save('./measurement_16-27_04_2021', 'exp')


%%
figure
hold on
yyaxis left
plot(exp.power.time+2/24,exp.power.value);
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value);
legend({'power','mode'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');
%%
figure
hold on
yyaxis left
plot(exp.setpoint_cool.time+2/24,exp.setpoint_cool.value,'b');
plot(exp.setpoint_heat.time+2/24,exp.setpoint_heat.value,'r');
ylim([20 25])
% yyaxis right
% plot(exp.mode.time+2/24,exp.mode.value,'g');
legend({'cooling','heating'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');

%%
figure
hold on

plot(exp.supply_temp.time+2/24,exp.supply_temp.value,'r');
plot(exp.return_temp.time+2/24,exp.return_temp.value,'g');
plot(exp.sensor_temp.time+2/24,exp.sensor_temp.value{1},'b');
legend({'supply','return', 'sensor'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');

%%
figure
hold on
yyaxis left
plot(exp.outside_temp.time+2/24,exp.outside_temp.value,'r');
yyaxis right
plot(exp.solar_rad.time+2/24,exp.solar_rad.value,'b');
datetick('x','mm/dd/yy HH:MM');
legend({'Temperature','Direct radiation'},'FontSize',18)

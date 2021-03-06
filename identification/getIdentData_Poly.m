clear 
close 
addpath(genpath('./utilities'))
%%
Ts = 900;
t0 = datenum(2021,12,06,00,00,00);
tf = datenum(2021,12,12,00,00,00);
exp1 = retreive_data_co2( t0, tf, Ts, 0 );
save('./data_12_21/exp1206-1212_raw', 'exp1')
exp2 = retreive_data_co2( t0, tf, Ts, 1 );
save('./data_12_21/exp1206-1212', 'exp2')
%%
Ts = 900;
t0 = datenum(2021,09,02,00,00,00);
tf = datenum(2021,09,08,00,00,00);
exp1 = retreive_data_seperate_temp( t0, tf, Ts, 0 );
save('./data_07/exp0902-0908_raw', 'exp1')
exp2 = retreive_data_seperate_temp( t0, tf, Ts, 1 );
save('./data_07/exp0902-0908', 'exp2')
%%
Ts = 900;
t0 = datenum(2021,11,18,16,00,00);
tf = datenum(2021,11,23,15,00,00);
exp1 = retreive_data_co2( t0, tf, Ts, 0 );
save('./data_21-22/exp1118-1123_raw', 'exp1')
exp2 = retreive_data_co2( t0, tf, Ts, 1 );
save('./data_21-22/exp1118-1123', 'exp2')

%%
Ts = 900;
t0 = datenum(2021,08,09,00,00,00);
tf = datenum(2021,08,15,00,00,00);
exp1 = retreive_data_seperate_temp( t0, tf, Ts, 0 );
save('./data_07/exp0809-0815_raw', 'exp1')
exp2 = retreive_data_seperate_temp( t0, tf, Ts, 1 );
save('./data_07/exp0809-0815', 'exp2')


%%
Ts = 900;
t0 = datenum(2021,11,01,12,00,00);
tf = datenum(2021,11,03,12,00,00);
exp8 = retreive_data( t0, tf, Ts );
save('./data_21-22/exp8_15min_heat', 'exp8')


%%
exp = exp1;
figures
hold on
yyaxis left
plot(exp.power.time+2/24,exp.power.value);
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value);
datetick('x','mm/dd/yy HH:MM');
%%
exp = exp10;
figure
hold on
yyaxis left
plot(exp.setpoint_summer.time+2/24,exp.setpoint_summer.value,'b');
plot(exp.setpoint_winter.time+2/24,exp.setpoint_winter.value,'r');
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value,'g');
legend({'summer','winter','mode'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');

%%
exp = exp1;
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
plot(exp.air_temp.time+2/24,exp.air_temp.value,'r');
yyaxis right
plot(exp.solar_GHI.time+2/24,exp.solar_GHI.value,'b');
datetick('x','mm/dd/yy HH:MM');
legend({'Temperature','Direct radiation'},'FontSize',18)
%%
Ts = 900;
t0 = datenum(2021,05,05,15,00,00);
tf = datenum(2021,05,06,09,00,00);
exp9 = retreive_data( t0, tf, Ts );
save('./data/exp9_15min', 'exp9')
Ts = 900;
t0 = datenum(2021,05,06,03,00,00);
tf = datenum(2021,05,07,14,00,00);
exp10 = retreive_data( t0, tf, Ts );
save('./data/exp10_15min', 'exp10')

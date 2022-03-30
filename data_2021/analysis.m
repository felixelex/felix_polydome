clear 
close 

load('./measurement_16-27_04_2021', 'exp')

%% Sensors measure room temperature
% Choose the period as you wish
start_time = 1;
end_time = 3312;
datestr(exp.sensor_temp.time(start_time)+2/24)
datestr(exp.sensor_temp.time(end_time)+2/24)
figure
hold on
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{1}(start_time:end_time), 'r');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{2}(start_time:end_time), 'g');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{3}(start_time:end_time), 'b');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{4}(start_time:end_time), 'k');
ylim([18 26])
% xlim([exp.sensor_temp.time(1) exp.sensor_temp.time(end)+4/24])
legend({exp.sensor_temp.position{1},exp.sensor_temp.position{2},...
	exp.sensor_temp.position{3},exp.sensor_temp.position{4}},'FontSize',18)
datetick('x','mm/dd/yy HH:MM','keeplimits');

%% mean and standard deviation
mean1 = mean(exp.sensor_temp.value{1});
mean2 = mean(exp.sensor_temp.value{2});
mean3 = mean(exp.sensor_temp.value{3});
mean4 = mean(exp.sensor_temp.value{4});
std1 = std(exp.sensor_temp.value{1});
std2 = std(exp.sensor_temp.value{2});
std3 = std(exp.sensor_temp.value{3});
std4 = std(exp.sensor_temp.value{4});
figure
hold on
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{1}(start_time:end_time)-mean1, 'r');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{2}(start_time:end_time)-mean2, 'g');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{3}(start_time:end_time)-mean3, 'b');
plot(exp.sensor_temp.time(start_time:end_time)+2/24,exp.sensor_temp.value{4}(start_time:end_time)-mean4, 'k');
% ylim([18 26])
% xlim([exp.sensor_temp.time(1) exp.sensor_temp.time(end)+4/24])
legend({sprintf([exp.sensor_temp.position{1},'mean: %s, std: %s'], string(mean1), string(std1))...
	,sprintf([exp.sensor_temp.position{2},'mean: %s, std: %s'], string(mean2), string(std2)),...
	sprintf([exp.sensor_temp.position{3},'mean: %s, std: %s'], string(mean3), string(std3)),...
	sprintf([exp.sensor_temp.position{4},'mean: %s, std: %s'], string(mean4), string(std4))},'FontSize',25)
datetick('x','mm/dd/yy HH:MM','keeplimits');

%% Discrete Fourier transform:
which_one = 1;
Ts = 300; % sampling time of the data: 5 min
Fs = 1/Ts; % sampling frequency
N = size(exp.sensor_temp.value{which_one},1);

F_vec = [0:N-1]*Fs/N; % frequency vector
temp_fft = fft(exp.sensor_temp.value{which_one});

m = abs(temp_fft);  % Magnitude
temp_fft(m<1e-6) = 0;
p = unwrap(angle(temp_fft)); % Phase

figure
subplot(2,1,1)
plot(F_vec,m)
title('Magnitude')
ylim([0 1000])

subplot(2,1,2)
plot(F_vec,p*180/pi)
title('Phase')



%% thermal energy estimation every 5 minutes, suppose air's volumetric heat capacity: 1210J/m3/K
energy_5min = (exp.supply_temp.value-exp.return_temp.value).*exp.supply_flow_rate.value/12 * 1.21;

figure
hold on
plot(exp.supply_temp.time+2/24,energy_5min);
% ylim([10 50])
ylabel('thermal energy estimation: [kJ/5min]');
legend({'thermal energy estimation: [kJ/5min]'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM','keeplimits');

%% Cooling/heating curve
start_time = 1585;
end_time = 3312;
datestr(exp.sensor_temp.time(start_time)+2/24)
datestr(exp.sensor_temp.time(end_time)+2/24)
% cooling
energy = energy_5min(start_time:end_time);
energy(energy>0) = 0;
ouside_temp = exp.outside_temp.value(start_time:end_time);

energy_1d = abs(sum(reshape(energy,12*24,[]))');
outside_temp_1d = mean(reshape(ouside_temp,12*24,[]))';
figure
hold on
scatter(outside_temp_1d,energy_1d);
xlabel('daily mean of ambient temperature [C^0]');
ylabel('daily sum of thermal energy estimation: [kJ/day]');
legend({'Cooling curve'},'FontSize',18)

%% heating
energy = energy_5min(start_time:end_time);
energy(energy<0) = 0;
ouside_temp = exp.outside_temp.value(start_time:end_time);

energy_1d = sum(reshape(energy,12*24,[]))';
outside_temp_1d = mean(reshape(ouside_temp,12*24,[]))';
figure
hold on
scatter(outside_temp_1d,energy_1d);
xlabel('daily mean of ambient temperature [C^0]');
ylabel('daily sum of thermal energy estimation: [kJ/day]');
legend({'Heating curve'},'FontSize',18)








%% weather
figure
hold on
yyaxis left
plot(exp.outside_temp.time+2/24,exp.outside_temp.value,'b');
yyaxis right
plot(exp.solar_rad.time+2/24,exp.solar_rad.value,'r');
datetick('x','mm/dd/yy HH:MM','keeplimits');
legend({'Temperature[C^0]','Direct radiation [W/m^2]'},'FontSize',18)
%% HP Power and mode
% mode: 1: heating 0: cooling
figure
hold on
yyaxis left
plot(exp.power.time+2/24,exp.power.value);
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value);
ylim([-1 2])
legend({'electrical power:[kW]','mode'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM','keeplimits');
%% HP setpoint
figure
hold on
yyaxis left
plot(exp.setpoint_cool.time+2/24,exp.setpoint_cool.value,'g');
plot(exp.setpoint_heat.time+2/24,exp.setpoint_heat.value,'b-');
ylim([21.5 23.5])
yyaxis right
plot(exp.mode.time+2/24,exp.mode.value,'r');
ylim([-1 2])
legend({'cooling [C^0]','heating [C^0]','mode'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM','keeplimits');

%% room-side supply and return temperaure
figure
hold on
yyaxis left
plot(exp.supply_temp.time+2/24,exp.supply_temp.value,'g');
plot(exp.return_temp.time+2/24,exp.return_temp.value,'b-');
ylim([10 50])
yyaxis right
plot(exp.supply_flow_rate.time+2/24,exp.supply_flow_rate.value,'r');
ylim([4600 5000])
legend({'supply [C^0]','return [C^0]', 'supply air flow [m^3/h]'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM','keeplimits');

%% weather
figure
hold on
yyaxis left
plot(exp.outside_temp.time+2/24,exp.outside_temp.value,'b');
yyaxis right
plot(exp.solar_rad.time+2/24,exp.solar_rad.value,'r');
datetick('x','mm/dd/yy HH:MM','keeplimits');
legend({'Temperature[C^0]','Direct radiation [W/m^2]'},'FontSize',18)
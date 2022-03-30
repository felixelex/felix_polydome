clear all
clc

load('./exp1_1min.mat')
exp_1min = exp1;
load('./exp1.mat')
exp_5min = exp1;


%%
exp_5min.sensor_temp.value  = exp_1min.sensor_temp.value(1:5:end);
exp_5min.sensor_temp.time  = exp_1min.sensor_temp.time(1:5:end);


%%
exp1_15min = exp_5min;

exp1_15min = resample(exp1_15min,3);
%%
exp1 = exp1_15min;
save('exp1_15min.mat', 'exp1')
%%
figure
hold on

plot(exp1.sensor_temp.time+2/24,exp1.sensor_temp.value,'r');
plot(exp_5min.sensor_temp.time+2/24,exp_5min.sensor_temp.value,'b');
plot(exp_2_15min.sensor_temp.time+2/24,exp_2_15min.sensor_temp.value,'y');
legend({'average','sample','15 min', 'sensor'},'FontSize',18)
datetick('x','mm/dd/yy HH:MM');
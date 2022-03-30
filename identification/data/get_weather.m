clear all 
clc
%%
filename = 'weather.csv';
fileID = fopen(filename);
% Date;UT time;Temperature;Relative Humidity;Pressure;Wind speed;
% Wind direction;Rainfall;Snowfall;Snow depth;Short-wave irradiation
C = textscan(fileID,'%s %s %f %f %f %f %f %f %f %f %f',...
'Delimiter',';','EmptyValue',-Inf);
fclose(fileID);

temp = C{3}(1:2:end)-273.15;
GHI = C{end}(1:2:end)*12; % It measures Wh/m2 of past time period, here: 5 minutes. http://www.soda-pro.com/web-services/meteo-data/merra
dn =  string(C{1}(1:2:end))+ ' '+ string(C{2}(1:2:end));
%%
% 4.02,21:00  -- 4.04, 13:30
load('./exp1.mat')
dn_start = 228;
dn_end = 713;

exp1 = save_weather(exp1, temp, GHI, dn,  dn_start, dn_end);

save('./exp1.mat', 'exp1')
%%
% 4.04,20:00  -- 4.06, 13:30
load('./data/exp2.mat')
dn_start = 792;
dn_end = 1289;
exp2 = save_weather(exp2, temp, GHI, dn,  dn_start, dn_end);
save('./exp2.mat', 'exp2')
%%
% 4.06,19:30  -- 4.07, 15:00
load('./exp3.mat')
dn_start = 1362;
dn_end = 1595;
exp3 = save_weather(exp3, temp, GHI, dn,  dn_start, dn_end);
save('./exp3.mat', 'exp3')
%%
% 4.07,20:00  -- 4.08, 14:00
load('./data/exp4.mat')
dn_start = 1656;
dn_end = 1871;
exp4 = save_weather(exp4, temp, GHI, dn,  dn_start, dn_end);
save('./exp4.mat', 'exp4')
%%
% 4.08,21:00  -- 4.09, 13:00
load('./data/exp5.mat')
dn_start = 1956;
dn_end = 2147;
exp5 = save_weather(exp5, temp, GHI, dn,  dn_start, dn_end);
save('./exp5.mat', 'exp5')
%%
exp = exp5;
h=figure;
hold on
set(h,'Units','normalized','Position',[0 0 1 .5]); 

yyaxis right
ylabel('[C^0]','FontSize',18)
a = plot(exp.air_temp.time, exp.air_temp.value, 'r', 'LineWidth', 4);

yyaxis left
ylabel('[W/m2]','FontSize',18)
aa = plot(exp.solar_GHI.time, exp.solar_GHI.value, 'g', 'LineWidth', 4);

h = legend([a, aa], 'temperature','GHI', 'Location','northeast');
set(h,'fontsize',18, 'interpreter', 'latex')
datetick('x','mm/dd/yy HH:MM');

%%
function exp1 = save_weather(exp1, temp, GHI, dn,  dn_start, dn_end)
    fprintf('Start time: %s, end time: %s \n',datestr(exp1.power.time(1)),datestr(exp1.power.time(end)))
    exp1.air_temp.time = datenum(dn(dn_start:dn_end), 'yyyy-mm-dd HH:MM');
    exp1.air_temp.value = temp(dn_start:dn_end);
    exp1.air_temp.unit = '[C^0]';

    exp1.solar_GHI.time = datenum(dn(dn_start:dn_end), 'yyyy-mm-dd HH:MM');
    exp1.solar_GHI.value = GHI(dn_start:dn_end);
    exp1.solar_GHI.unit = '[W/m^2]';
    fprintf('Chosen from .csv, start time: %s, end time: %s \n',datestr(exp1.air_temp.time(1)), datestr(exp1.air_temp.time(end))) 
end
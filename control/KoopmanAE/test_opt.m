clc
clear all
close all

% Paths
addpath('./parameters')
addpath('./KoopmanAE')
addpath(genpath('../utilities'))
addpath(genpath('../'))

% Start time of experiment
h = 11; 
min = 20;
TimeZone = 'Europe/Zurich';
current_time = datetime('now', 'TimeZone', TimeZone);
what_year = year(current_time);what_month = month(current_time);what_day = day(current_time);
what_time_start = datenum(what_year, what_month, what_day, h, min, 0);

parameter = KoopmanAE_par(0, datetime(what_time_start,'ConvertFrom','datenum'));
parameter.date_exp_start = now()

controller = KoopmanMPCController(parameter);

opt = OptKoopmanAEMPC(controller.sys, controller.N_ini, controller.N_pred);




function signal = get_disturbance(obj)
%GETFORECAST Retreive the forecast over pred horizon 
% for the specified quantity

    try%%
		signal = 0;
        TimeZone = 'UTC';
        currentTime = datetime('now', 'TimeZone', TimeZone ); 
        fprintf('Getting weather prediction current time: %s at time zone %s \n', datestr(currentTime,...
            'dd-mmm-yyyy HH:MM:SS'), TimeZone)
        
        % get weather
        [temp_cur,rad_cur,temp_pred,rad_pred] = get_weather_from_api(obj.N_pred);
        
        obj.last_dist.temp = temp_cur;
        obj.last_dist.rad = rad_cur/10000.0;
        obj.last_forecast.temp = removeOutliers(temp_pred);
        obj.last_forecast.rad = removeOutliers(rad_pred)/10000.0;
        fprintf('Air temperature, current: %d, prediction: max %d, min %d \n', obj.last_dist.temp,...
            max(obj.last_forecast.temp),...
            min(obj.last_forecast.temp)); 
        fprintf('Radiation, current: %d, prediction: max %d, min %d \n', obj.last_dist.rad,...
            max(obj.last_forecast.rad),...
            min(obj.last_forecast.rad));
        % h=figure;
        % hold on
        % set(h,'Units','normalized','Position',[0 0 1 .5]); 
        % yyaxis left
        % ylim([ 0,15])
        % ylabel('[°C]','FontSize',18)
        % plot(dn,data(:,1),'b','LineWidth',4)
        % 
        % yyaxis right
        % ylabel('[W/m2]','FontSize',18)
        % plot(dn,data(:,2),'r','LineWidth',4)
        % title('Meteorological data','FontSize',18)
        % legend({'Temperature','Direct radiation'},'FontSize',18)
        % 
        % datetick('x','mm/dd/yy HH:MM');

	%

			
	catch
		%
% 		values = zeros(1,obj.N_pred);
% 		obj.last_forecast.rad = values;
% 
% 		values = 15*ones(1,obj.N_pred);
% 		obj.last_forecast.temp = values;
		disp('Problem with extracting forecast. Created a dummy forecast')
        
%         obj.last_dist.rad = 0;
%         obj.last_dist.temp = 15;
        disp('Problem with extracting current weather. Created a dummy forecast')
		signal = 1;
	end%%

end


function [values, ts] = readSeriesFromDatabase( ...
    seriesname, tag_keys, tag_values, t0, tf, resolution, field)
%READSERIESFROMDATABASE read <quantity> with the couple <tag_keys, tag_values>  from influxdb for the
% time period between <t0> and <tf> (epochtime, in second) at resolution
% <resolution> second.

% If the time series is empty or does not exist, it returns nan.
% to use this function:
% addpath('~/desl-development/util/matlab')
% addpath('desl-development/lib/matlab/jsonlab')
% addpath('desl-development/util/matlab/time')



% if (nargin < 1)
%     % for debug only
%     t0 = matlabToEpochTime(datenum(2016, 10, 11));
%     tf = matlabToEpochTime(datenum(2016, 10, 12));
%     seriesname = 'Power';
%     tag_keys = {};
%     tag_values = {};
%     resolution = 15*60;
% end
if (nargin < 7)
	field = 'value';
end

% Compose query
groupbytime_clause = sprintf('time(%.0fs)', resolution);
time_clause = sprintf('(time >= %.0fs AND time < %.0fs)', t0, tf);

if numel(tag_keys) ~= numel(tag_values)
    error('numel(tag_keys) ~= numel(tag_values)!');
end

where_clause = sprintf('%s', time_clause);
for i=1:numel(tag_keys)
    where_clause = sprintf('%s AND %s=''%s''', where_clause, tag_keys{i}, tag_values{i});
end


query = sprintf('SELECT mean(%s) as value FROM %s WHERE %s GROUP BY %s', ...
    field,seriesname, where_clause, groupbytime_clause);

% Perform query
IP = '127.0.0.1'; %'10.23.1.85';
DB = 'test_radu_pls_delete'; %'openhab_db';

url = sprintf('http://%s:8086/query?pretty=true&db=%s&epoch=s&q=%s', IP, DB, strrep(query, ' ', '%20'));

html = urlread(url);
result = loadjson(html);


try
    n = max(size(result.results{1,1}.series{1, 1}.values));
    results = result.results{1,1}.series{1, 1}.values;
    values = results(:,2);
    ts = results(:,1);
catch
    values = nan;
    ts = nan;
end


if (nargin < 1)
    plot(ts, values)
end


if iscell(values)
    % Depending on the fact the json contains or not empty values,
    % the json has a different structure (shit!). This is a workaround to
    % still get the good values and setting to NaN the empty values.
    
    results = result.results{1,1}.series{1, 1}.values;
    values = nan(n,1);
    ts = nan(n,1);
    for i=1:n
        if isnumeric(results{i}(1))
            ts(i) = results{i}(1);
            values(i) = results{i}(2);
        else
            ts(i) = cell2mat(results{i}(1));
        end
    end
end



end


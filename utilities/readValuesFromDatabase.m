function [vals, valid, ts] = readValuesFromDatabase (seriesname_seq, tag_keys_seq, tag_values_seq, k, Ts)
% Returns the mean value of several time series at <seriesname_seq>,
% <tag_keys_seq>,<tag_values_seq> for the last k period at Ts resolution with respect to server time.


% if nargin == 0
%     seriesname_seq = {'EL_L', 'RealPower'};
%     tag_keys_seq = {{}, {'device'}};
%     tag_values_seq = {{}, {'container0'}};
%     k = 1;
%     Ts = 10;
% end

n = numel(seriesname_seq);

if numel(tag_keys_seq) ~= n
    error('numel(tag_keys) should match n');
end

if numel(tag_values_seq) ~= n
    error('numel(tag_keys) should match n');
end

QUERY = '';
for i=1:n
    tag_keys = tag_keys_seq{i};
    tag_values = tag_values_seq{i};
    seriesname = seriesname_seq{i};
    time_clause = sprintf('(time > now() - %.0fs AND time <= now())', Ts*k);
    where_clause = sprintf('%s', time_clause);

    if numel(tag_keys) ~= numel(tag_values)
        error('numel(tag_keys) ~= numel(tag_values)!');
    end
    
    for j=1:numel(tag_keys)
        where_clause = sprintf('%s AND %s=''%s''', where_clause, tag_keys{j}, tag_values{j});
    end

    query = sprintf('SELECT value FROM %s WHERE %s ORDER BY time desc', ...
        seriesname, where_clause);

    QUERY = sprintf('%s%s;', QUERY, query);
end

[~, whichMachine] = system('hostname');
if strcmp(whichMachine,'influxdbserver0')
    IP = '127.0.0.1';
else
    IP = '10.23.1.85'; % 128.178.10.131
end
    DB = 'openhab_db'; %'Polydome';


QUERY = strrep(QUERY, ' ', '%20');
QUERY = strrep(QUERY, ';', '%3B');

url = sprintf('http://%s:8086/query?pretty=true&db=%s&epoch=s&q=%s', IP, DB, QUERY);
html = urlread(url,'Timeout',0.5);
result = loadjson(html);


vals = nan(n, 1);
ts = nan(n, 1);
try
    for i = 1:n
        vals(i) = result.results{1,i}.series{1,1}.values(1, 2);
        ts(i) = result.results{1,i}.series{1,1}.values(1, 1);
        %if i > 1 && (ts(i)-ts(i-1))~=0
        %    error('Timestamp should equal!')
        %end
    end
catch
    vals = nan(n, 1);
end


valid = sum(isnan(vals)) == 0;

end

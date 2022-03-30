function [vals, valid, ts] = read_valuelist_from_influxdb(series_name, tag_key, tag_value, t0, tf, field_value)
% Returns the all the values at <series_name>,
% <tag_keys_seq>,<tag_values_seq> during t0 and tf with respect to server time.


if nargin < 6
	field_value = 'value';
end
QUERY = '';

time_clause = sprintf('(time >= %.0fs AND time < %.0fs)', t0, tf);
where_clause = sprintf('%s', time_clause);

if numel(tag_key) ~= numel(tag_value)
    error('numel(tag_keys) ~= numel(tag_values)!');
end

for j=1:numel(tag_key)
    where_clause = sprintf('%s AND %s=''%s''', where_clause, tag_key{j}, tag_value{j});
end

query = sprintf('SELECT %s FROM %s WHERE %s ORDER BY time desc', ...
    field_value, series_name, where_clause);

QUERY = sprintf('%s%s;', QUERY, query);


IP = '127.0.0.1'; %'127.0.0.1'; %'10.23.1.85'; % 128.178.10.131
DB =  'test_radu_pls_delete'; %'test_radu_pls_delete'; %'openhab_db'; %'Polydome';

QUERY = strrep(QUERY, ' ', '%20');
QUERY = strrep(QUERY, ';', '%3B');

url = sprintf('http://%s:8086/query?pretty=true&db=%s&epoch=s&q=%s', IP, DB, QUERY);
html = urlread(url,'Timeout',0.5);
result = loadjson(html);


try
    vals = result.results{1,1}.series{1,1}.values(:, 2);
    ts = result.results{1,1}.series{1,1}.values(:, 1);
    %if i > 1 && (ts(i)-ts(i-1))~=0
    %    error('Timestamp should equal!')
    %end
catch
    vals = nan;
    ts = nan;
end


valid = sum(isnan(vals)) == 0;

end

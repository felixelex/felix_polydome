function [exit_state] = write_series_to_influxdb( values, ts, seriesname, tag_keys, tag_values )
%WRITESERIESTODATABASE Save a time series to the database through the curl
% unix utility. Supported by Linux and Mac. Not currently tested on Win.

% <values> is the sequence of values to save
% <ts> is the sequence of timestamp (epochtime in seconds)
% <seriesname> is the name of the time series to which values refers.
% <tag_keys> is a list of tags keys
% <tag_values> is a list of values associated to the previous keys.

% <exit_state> denotes the state of the request. Zero means everything
% cool.

if nargin == 0
    % For debug purpose only
    values = randn(10,1);
    ts = 0:9;
    seriesname = 'test';
    tag_keys = {'device'};
    tag_values = {'test'};
end

if numel(ts) ~= numel(values)
    error('Timestamp and values sequences should be with same length!')    
end


IP = '127.0.0.1'; %'127.0.0.1'; %'10.23.1.85';
DB = 'test_radu_pls_delete'; %'test_radu_pls_delete'; %'openhab_db';

tags_string = '';
for i=1:numel(tag_keys)
    if numel(tags_string) > 0
        tags_string = sprintf('%s,%s=%s', tags_string, tag_keys{i}, tag_values{i});
    else
        tags_string = sprintf('%s=%s', tag_keys{i}, tag_values{i});
    end
end

if numel(tags_string) > 0
    tags_string = sprintf(',%s', tags_string);
end



fileID = fopen('toInfluxdb.temp','w');
for i=1:numel(ts)
    line = sprintf('%s%s value=%f %d', seriesname, tags_string, values(i), ts(i)*1e9);
    fprintf(fileID, '%s\n', line);
end
fclose(fileID);
% OS: DYLD_LIBRARY_PATH, linux:LD_LIBRARY_PATH
% exit_state = system('LD_LIBRARY_PATH="";/usr/bin/curl -i -XPOST "http://10.23.1.85:8086/write?db=openhab_db" --data-binary @toInfluxdb.temp');
exit_state = system(['LD_LIBRARY_PATH="";/usr/bin/curl -XPOST "http://',IP,':8086/write?db=',DB,'" --data-binary @toInfluxdb.temp']);  

if exit_state ~= 0
    disp('** Writing to InlfuxDB has failed! **')
end


delete(sprintf('%s/toInfluxdb.temp', pwd()));

end


function send_input_to_low_level(obj, input, mode)
% Write current input in "./controlInput/controlInput.txt" file, which is
% used by fast controller

strFilePath = './control_input';

fid = fopen([strFilePath, filesep, 'control_input.txt'],'w');
fprintf(fid,'%s\n',datestr(now));
fprintf(fid,'%d\n',input); 
fprintf(fid,'%d\n',mode);

fclose(fid);


end


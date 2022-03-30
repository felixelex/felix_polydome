function [z0, A, B_u, B_d, C] = get_koopman_representation(y0, d0, model)
    % Function that calls the Koopman model in Python and returns the
    % lifted state and Koopman operators.

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%                   Write measurements to file                    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    s = struct("y0",y0,"d0",d0);
    json = jsonencode(s);
	

    fid = fopen('tmp\measurements.json', 'w');
    fprintf(fid, '%s', json);
    fclose(fid);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%                        Run Python script                        %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    commandStr = '/Users/lucafabietti/Documents/Felix_polydome/proj_env/bin/python get_'+model+'_representation.py';
    [status, commandOut] = system(commandStr);
    if status ~= 0
        error("=======Python error======" + commandOut);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%                   Get Output of python script                   %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    jsonText = fileread('tmp\lifting.json');
    data = jsondecode(jsonText);

    z0 = data.z0;
    A = data.A;
    B_u = data.B_u;
    B_d = data.B_d;
    C = data.C;
end



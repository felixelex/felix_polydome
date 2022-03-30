function change_mode(obj, mode)
    % update parameters if mode is changed
    if mode ~= obj.last_mode
        obj.mode_count = 1; % Restart the count
        if mode == 0 % cooling
            if obj.last_mode~=-1
                obj.Hankel_u_heat = obj.Hankel_u; obj.Hankel_y_heat = obj.Hankel_y; % !!! Adaptive change
                obj.Hankel_w_heat = obj.Hankel_w; 
            end
            obj.Hankel_u = obj.Hankel_u_cool; obj.Hankel_y = obj.Hankel_y_cool; 
            obj.Hankel_w = obj.Hankel_w_cool;  
            obj.sys.U= obj.sys.U_cool;  
            obj.sys.dU= obj.sys.dU_cool;  
        else % heating
            if obj.last_mode~=-1
                obj.Hankel_u_cool = obj.Hankel_u; obj.Hankel_y_cool = obj.Hankel_y; % !!! Adaptive change
                obj.Hankel_w_cool = obj.Hankel_w; 
            end
            obj.Hankel_u = obj.Hankel_u_heat; obj.Hankel_y = obj.Hankel_y_heat; 
            obj.Hankel_w = obj.Hankel_w_heat;    
            obj.sys.U= obj.sys.U_heat;  
            obj.sys.dU= obj.sys.dU_heat;          
        end
        % --- Split the Hankel matrix for initial and prediction sequence 
        Hankel_ini = [obj.Hankel_u(1:obj.sys.nu*obj.N_ini, :);
                    obj.Hankel_w(1:obj.sys.nw*obj.N_ini, :);
                      obj.Hankel_y(1:obj.sys.ny*(obj.N_ini+1), :)];
        Hankel_pred = [obj.Hankel_u(obj.sys.nu*obj.N_ini+1:end, :);
                    obj.Hankel_w(obj.sys.nw*obj.N_ini+1:end, :);
                      obj.Hankel_y(obj.sys.ny*(obj.N_ini+1)+1:end, :)];
        obj.Hankel_ini = Hankel_ini;
        obj.Hankel_pred = Hankel_pred;        
    else
        obj.mode_count = obj.mode_count + 1;
        % If radiation is always 0, we will lose excitation
        % For cooling
        if obj.last_dist.rad <= 5/1000 && mode == 0 
            obj.mode_count = 1;
        end
    end
    obj.last_mode = mode;
end


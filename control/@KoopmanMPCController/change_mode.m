function change_mode(obj, mode)
    % update parameters if mode is changed
    if mode ~= obj.last_mode
        obj.mode_count = 1; % Restart the count
        if mode == 0 % cooling
            pass
        else % heating
            pass       
        end
      
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



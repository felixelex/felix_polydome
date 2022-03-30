    function [u_ini_next, y_ini_next] = update_initial_sequence(obj,u_ini, y_ini, u_current, y_current)
        u_ini_next = [u_ini(obj.nu+1:end,:); u_current];
        y_ini_next = [y_ini(obj.ny+1:end,:); y_current];
    end  
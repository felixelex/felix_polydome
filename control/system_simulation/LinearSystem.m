classdef LinearSystem < handle
    
    properties
        A; B; C;
        nu;ny;
        Hankel_mat;
        Q; R;
        Y; U;
        r; % reference
    end
    
    methods
        function obj = LinearSystem(tParameter)
            obj.A = tParameter.sys.A;
            obj.B = tParameter.sys.B;
            obj.C = tParameter.sys.C;
            obj.Q = tParameter.sys.Q;
            obj.R = tParameter.sys.R;
            obj.ny = tParameter.sys.ny;
            obj.nu = tParameter.sys.nu;
            obj.U = tParameter.sys.U;
            obj.Y = tParameter.sys.Y;
            obj.r = tParameter.sys.r; 
        end
        
        function [x_next, y_next] = move_one_step(obj, x, u)
            x_next = obj.A*x + obj.B*u;
            y_next = obj.C*x_next;
        end       
                   
    end

end


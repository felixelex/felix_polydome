classdef LinearSystem < handle
    
    properties
        A; B; C;
        nu;ny;nx;
        Hankel_mat;
        Q; R;
        Y; U;
        r; % reference
    end
    
    methods
        function obj = LinearSystem(A, B, C, Q, R, U, Y)
            if nargin <7
                Q=[]; R=[]; U=[]; Y=[];
            end
            obj.A = A;
            obj.B = B;
            obj.C = C;
            obj.Q = Q;
            obj.R = R;
            obj.nx = size(A, 1);
            obj.ny = size(C, 1);
            obj.nu = size(B, 2);
            obj.U = U;
            obj.Y = Y;            
        end
        
        function [x_next, y_next] = move_one_step(obj, x, u)
            x_next = obj.A*x + obj.B*u;%*(1+0.1*x(2));
            y_next = obj.C*x_next;
        end     
        
        function [u_ini_next, y_ini_next] = update_ini(obj,u_ini_next, y_ini_next, u_current, y_current)
            u_ini_next = [u_ini_next(obj.nu+1:end,:); u_current];
            y_ini_next = [y_ini_next(obj.ny+1:end,:); y_current];
        end    
        
        function set_reference(obj, r)
            obj.r = r;
        end
%         function [u_ini_next, y_ini_next] = update_Hankel(obj, Up, Uf, Ypu_current, y_current)
%             x_next = obj.A*x + obj.B*u;
%             y_next = obj.C*x_next;
%         end             
    end
    
        

        
      
        

end


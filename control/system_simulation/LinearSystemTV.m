classdef LinearSystemTV < LinearSystem
    % Linear system including some time-varying terms
    properties
    end
    
    methods
        function obj = LinearSystemTV(tParameter)
            obj =  obj@LinearSystem(tParameter);
        end
        
        function [x_next, y_next] = move_one_step(obj, x, u, t)
            x_next = obj.A{t}*x + obj.B*u;
            y_next = obj.C*x_next;
        end     
                  
    end
    
        

        
      
        

end


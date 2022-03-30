classdef LinearSystemNoiseTV < LinearSystem
    % Linear system including some time-varying terms
    properties
        w_real;
        w_pred;
        E;
        nw;
    end
    
    methods
        function obj = LinearSystemNoiseTV(tParameter)
            obj =  obj@LinearSystem(tParameter);
            obj.w_pred = tParameter.w_pred;
            obj.w_real = tParameter.w_real;
            obj.E = tParameter.sys.E;
            obj.nw = tParameter.sys.nw;            
        end
        
         function [x_next, y_next] = move_one_step(obj, x, u, t)
            x_next = obj.A(t)*x + obj.B*u +obj.E*obj.w_real(:,t);
            y_next = obj.C*x_next;
         end  
         
         function [x_next, y_next] = move_N_step(obj, x, u_seq, t, N)
            x_next = obj.A{t}*x + obj.B*u_seq(:,1) +obj.E*obj.w_pred(:,t);
            y_next = obj.C*x_next;
            for i = 2:N
                x_next(:,i) = obj.A{t+i-1}*x + obj.B*u_seq(:,i) +obj.E*obj.w_pred(:,t+i-1);
                y_next(:,i) = obj.C*x_next(:,i);
            end
          end         
                        
    end

end


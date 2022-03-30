classdef LinearSystemNoiseTV < LinearSystem
    % Linear system including some time-varying terms
    properties
        w_real;
        w_pred;
        w_y;
        E;
        nw;
        W;
        dU;
    end
    
    methods
        function obj = LinearSystemNoiseTV(A, B, C, Q, R, U, Y,W,dU)
            if nargin<9
                dU = 0;
            end            
            obj =  obj@LinearSystem(A, B, C, Q, R, U, Y);
            obj.W = W;
            obj.dU = dU;
        end
        
         function [x_next, y_next] = move_one_step(obj, x, u, t)
            x_next = obj.A(t)*x + obj.B*u +obj.E*obj.w_real(:,t);
            y_next = obj.C*x_next + obj.w_y(:,t+1);
         end  
         
         function [x_next, y_next] = move_N_step(obj, x, u_seq, t, N)
            x_next = obj.A(t)*x + obj.B*u_seq(:,1) +obj.E*obj.w_pred(:,t);
            y_next = obj.C*x_next;
            for i = 2:N
                x_next(:,i) = obj.A{t+i-1}*x + obj.B*u_seq(:,i) +obj.E*obj.w_pred(:,t+i-1);
                y_next(:,i) = obj.C*x_next(:,i) + obj.w_y(:,i);
            end
          end         
        
        function set_noise(obj, w_pred, w_real, E, w_y)
            obj.w_pred = w_pred;
            obj.w_real = w_real;
            obj.E = E;
            obj.nw = size(E, 2);
            obj.w_y = w_y;
        end                  
    end
    
        

        
      
        

end


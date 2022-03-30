classdef OptDeePCNoiseTV < OptDeePC
    % Solve the DeePC optimal problem with some time-varying terms:
    %   min_g sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. Hankel_mat * g = [u_ini; y_ini; u; y]
    %            u_k \in U_k, y_k \in Y_k
    %   where g is decision variables
 
    properties
        t; % current time
        kappa; % dual variables in KKT condition
        Qg;
    end
    
    methods
        function obj = OptDeePCNoiseTV(sys, N_ini, N_pred, T, t)
            obj = obj@OptDeePC(sys, N_ini, N_pred, T);
            obj.t = t;
            obj.kappa = sdpvar((sys.nu+sys.nw)*(N_ini+N_pred) + sys.ny*(N_ini+1), 1);
            obj.slack = sdpvar(sys.ny*(N_ini+1), 1);   
        end
        
        function add_g_penalty(obj, Qg)
            obj.Qg = Qg;
            obj.Cost = obj.Cost + norm(Qg*obj.g,1);
        end
         
        
        function add_Hankel_noise(obj, Hankel_p,Hankel_f, u_ini, w_ini, y_ini, w_pred)
            obj.Constraints = [obj.Constraints;
                                Hankel_p*obj.g == [u_ini; w_ini; y_ini+obj.slack]];
            uyw = Hankel_f*obj.g;
            w_pred = reshape(w_pred, [obj.sys.nw*obj.N_pred,1]);
            obj.Constraints = [obj.Constraints;
                                uyw(obj.sys.nu*obj.N_pred+1:(obj.sys.nu+obj.sys.nw)*obj.N_pred,:) ...
                                    == w_pred];            
            obj.u = reshape(uyw(1:obj.sys.nu*obj.N_pred,:), [obj.sys.nu,obj.N_pred]);
            obj.y = reshape(uyw((obj.sys.nu+obj.sys.nw)*obj.N_pred+1:end,:), [obj.sys.ny,obj.N_pred]);
        end
        
        function add_adaptive_KKT(obj, Hankel_p, Hankel_f, Qg)
            H = [Hankel_p; Hankel_f(1:(obj.sys.nu+obj.sys.nw)*obj.N_pred, :)];
            obj.Constraints = [obj.Constraints;
                                Qg*obj.g + H'*obj.kappa == 0;
                                100*obj.slack - obj.kappa(1:obj.sys.ny*(obj.N_ini+1),:)];
        end
        
        function add_ineq_constraint(obj)
            U = obj.sys.U; Y = obj.sys.Y;
            Hu  = U.A;  bu  = U.b;
            for i = 1:obj.N_pred          
                Hy  = Y{obj.t+i}.A;  by  = Y{obj.t+i}.b;       %!!! y(:,1) at t+1           
                obj.Constraints = [obj.Constraints;
                                    Hu*obj.u(:,i) <= bu;
                                    Hy*obj.y(:,i) <= by];
            end
        end
 
        function compute_cost(obj)
            r = obj.sys.r;
            Q = obj.sys.Q;
            R = obj.sys.R;
            for i=1:obj.N_pred
%                 r_i = r(:,obj.t+i); %!!! y(:,1) at t+1  
%                 obj.Cost = obj.Cost + (obj.y(:,i)-r_i)'*Q*(obj.y(:,i)-r_i) + obj.u(:,i)'*R*obj.u(:,i);
% (obj.y(:,i)-r_i)'*Q*(obj.y(:,i)-r_i) +
                obj.Cost = obj.Cost +  obj.u(:,i)*R;
            end 
            obj.Cost = obj.Cost+ 50* obj.slack'*obj.slack;%100*norm(obj.slack,1);% 100* obj.slack'*obj.slack; 
        end
        
        
        function [u_opt_seq, y_opt_seq,g_opt, slack_opt] = solve(obj)
            u_opt_seq = [];
            y_opt_seq = [];
            options = sdpsettings('verbose',1,'solver','gurobi','gurobi.TimeLimit', 5);
            optimize(obj.Constraints, obj.Cost, options);
            if double(obj.u(:,1)) == 0 |isnan(double(obj.u(:,1))) 
                options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
                optimize(obj.Constraints, obj.Cost, options);
            end
            for i = 1:obj.N_pred
                u_opt_seq = [u_opt_seq, double(obj.u(:,i) )];
                y_opt_seq = [y_opt_seq, double(obj.y(:,i))];
            end
%             slack_opt = double(obj.slack);
            cost_opt = double(obj.Cost);
            fprintf('optimal cost - cost of slack variables: %d \n',cost_opt - double(50* obj.slack'*obj.slack));
            %double(100*norm(obj.slack,1))%double(100* obj.slack'*obj.slack) 
            g_opt = double(obj.g);
			slack_opt = double(obj.slack);
        end 
        
    end
end


classdef OptDeePC < handle
    % Solve the DeePC optimal problem:
    %   min_g sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. Hankel_mat * g = [u_ini; y_ini; u; y]
    %            u_k \in U, y_k \in Y
    %   where g is decision variables
 
    properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        u; y;
        g; % Decision variables
        slack;
        Constraints;
        Cost;
    end
    
    methods
        function obj = OptDeePC(sys, N_ini, N_pred, T)

            obj.N_ini = N_ini;
            obj.N_pred = N_pred;
            obj.sys = sys;
            
%             obj.y = sdpvar(sys.ny*ones(1,N_pred),ones(1,N_pred));
%             obj.u = sdpvar(sys.nu*ones(1,N_pred),ones(1,N_pred));
            obj.u = [];
            obj.y = [];
            obj.g = sdpvar(T-N_ini-N_pred+1, 1);
            obj.slack = sdpvar(sys.ny*(N_ini+1), 1);
            obj.Constraints = [];
            obj.Cost = 0;
        end
        
        function add_Hankel(obj, Hankel_p,Hankel_f, u_ini, y_ini)           
            obj.Constraints = [obj.Constraints;
                                Hankel_p*obj.g == [u_ini; y_ini]]; % y_ini+obj.slack
            uy = Hankel_f*obj.g;
            obj.u = reshape(uy(1:obj.sys.nu*obj.N_pred,:), [obj.sys.nu,obj.N_pred]);
            obj.y = reshape(uy(obj.sys.nu*obj.N_pred+1:end,:), [obj.sys.ny,obj.N_pred]);
        end
        
        function add_ineq_constraint(obj)
            Hu  = obj.sys.U.A;  bu  = obj.sys.U.b;
            Hy  = obj.sys.Y.A;  by  = obj.sys.Y.b;
                for i = 1:obj.N_pred
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
                obj.Cost = obj.Cost + (obj.y(:,i)-r)'*Q*(obj.y(:,i)-r) + obj.u(:,i)'*R*obj.u(:,i);
            end 
%             obj.Cost = obj.Cost+ 10*norm(obj.slack,1); 
        end
        function initialization(obj)
%             obj.g = sdpvar(T-N_ini-N_pred+1, 1);
%             obj.slack = sdpvar(sys.ny*N_ini, 1);
            obj.Constraints = [];
            obj.Cost = 0;            
        end
 
        function [u_opt_seq, y_opt_seq,g_opt] = solve(obj)
            u_opt_seq = [];
            y_opt_seq = [];
            options = sdpsettings('verbose',1,'solver', 'quadprog','gurobi.TimeLimit', 10); % gurobi
            optimize(obj.Constraints, obj.Cost, options);
            if double(obj.u(:,1)) == 0 | isnan(double(obj.u(:,1))) 
                options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
                optimize(obj.Constraints, obj.Cost, options);
            end               
            for i = 1:obj.N_pred
                u_opt_seq = [u_opt_seq, double(obj.u(:,i) )];
                y_opt_seq = [y_opt_seq, double(obj.y(:,i))];
            end
            g_opt = double(obj.g);
        end        
    end
end


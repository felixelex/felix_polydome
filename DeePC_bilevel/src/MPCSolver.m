classdef MPCSolver< handle
    % Solve the MPC optimal problem:
    %   min_u sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. x_{k+1} = A*x + B*u
    %            y_k = C*x_k
    %            u_k \in U, y_k \in Y
 
    properties
        sys;
        N_pred; % Prediction steps
        y; u;
        Constraints;
        Cost;
    end
    
    methods
        function obj = MPCSolver(sys, N_pred)
            obj.N_pred = N_pred+1;
            obj.sys = sys;
            obj.y = sdpvar(sys.ny*ones(1,obj.N_pred),ones(1,obj.N_pred));
            obj.u = sdpvar(sys.nu*ones(1,obj.N_pred),ones(1,obj.N_pred));
            obj.Constraints = [];
            obj.Cost = 0;
        end
           
        function add_ineq_constraint(obj)
            Hu  = obj.sys.U.A;  bu  = obj.sys.U.b;
            Hy  = obj.sys.Y.A;  by  = obj.sys.Y.b;
                for i = 1:obj.N_pred
                    obj.Constraints = [obj.Constraints;
                                        Hu*obj.u{i} <= bu;
                                        Hy*obj.y{i} <= by];
                end
        end
 
        function compute_cost(obj)
            r = obj.sys.r;
            Q = obj.sys.Q;
            R = obj.sys.R;
            for i=1:obj.N_pred-1
                obj.Cost = obj.Cost + (obj.y{i}-r)'*Q*(obj.y{i}-r) + obj.u{i}'*R*obj.u{i};
            end 
            obj.Cost = obj.Cost + (obj.y{obj.N_pred}-r)'*Q*(obj.y{obj.N_pred}-r);
%             obj.Cost = obj.Cost+ 10*norm(obj.slack,1); 
        end
        function initialization(obj, x0)
            obj.Constraints = [obj.y{1} == obj.sys.C*x0;];
            obj.Cost = 0;
            
            x = x0;
            for i=1:obj.N_pred-1
                x = obj.sys.A*x + obj.sys.B*obj.u{i};
                obj.y{i+1} = obj.sys.C*x;
            end             
        end
 
        function [u_opt_seq, y_opt_seq] = solve(obj)
            u_opt_seq = [];
            y_opt_seq = [];
            options = sdpsettings('verbose',1,'solver', 'gurobi','gurobi.TimeLimit', 10);
            optimize(obj.Constraints, obj.Cost, options);
            if double(obj.u{1}) == 0 | isnan(double(obj.u{1})) 
                options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
                optimize(obj.Constraints, obj.Cost, options);
            end               
            for i = 1:obj.N_pred
                u_opt_seq = [u_opt_seq, double(obj.u{i})];
                y_opt_seq = [y_opt_seq, double(obj.y{i})];
            end

        end
    end
end


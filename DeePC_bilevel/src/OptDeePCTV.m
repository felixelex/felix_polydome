classdef OptDeePCTV < OptDeePC
    % Solve the DeePC optimal problem with some time-varying terms:
    %   min_g sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. Hankel_mat * g = [u_ini; y_ini; u; y]
    %            u_k \in U_k, y_k \in Y_k
    %   where g is decision variables
 
    properties
        t; % current time
        kappa; % dual variables in KKT condition
        Qg; % Quadratic cost matrix for variable g
    end
    
    methods
        function obj = OptDeePCTV(sys, N_ini, N_pred, T, t)
            obj = obj@OptDeePC(sys, N_ini, N_pred, T);
            obj.t = t;
            obj.kappa = sdpvar(sys.nu*(N_ini+N_pred) + sys.ny*(N_ini+1), 1);
            obj.slack = sdpvar(sys.ny*(N_ini+1), 1);
        end
        
        function add_g_penalty(obj, Qg)
            obj.Qg = Qg;
            obj.Cost = obj.Cost + norm(Qg*obj.g,1);
        end
        
        function add_Hankel(obj, Hankel_p,Hankel_f, u_ini, y_ini)
            obj.Constraints = [obj.Constraints;
                                Hankel_p*obj.g == [u_ini; y_ini+obj.slack]];
            uy = Hankel_f*obj.g;
            obj.u = reshape(uy(1:obj.sys.nu*obj.N_pred,:), [obj.sys.nu,obj.N_pred]);
            obj.y = reshape(uy(obj.sys.nu*obj.N_pred+1:end,:), [obj.sys.ny,obj.N_pred]);
        end
        
        function add_adaptive_KKT(obj, Hankel_p, Hankel_f, Qg)
            obj.Qg = Qg;
            H = [Hankel_p; Hankel_f(1:obj.sys.nu*obj.N_pred, :)];
            obj.Constraints = [obj.Constraints;
                                Qg*obj.g + H'*obj.kappa == 0;
                                10*obj.slack - obj.kappa(1:obj.sys.ny*(obj.N_ini+1),:)];
        end
        
        function add_ineq_constraint(obj, Hankel_f)
%             Hankel_u = Hankel_f(1:obj.sys.nu*obj.N_pred,:);
%             Hankel_y = Hankel_f(obj.sys.nu*obj.N_pred+1:end,:);            
%             Hy = obj.sys.Y.H; hy = obj.sys.Y.h;
%             Hu = obj.sys.U.H; hu = obj.sys.U.h;
%             obj.Constraints = [obj.Constraints;
%                                 Hy*Hankel_y*obj.g <= hy;
%                                 Hu*Hankel_u*obj.g <= hu];
                            
            Hu  = obj.sys.U.P.A;  bu  = obj.sys.U.P.b;
            Hy  = obj.sys.Y.P.A;  by  = obj.sys.Y.P.b;
                for i = 1:obj.N_pred
                    obj.Constraints = [obj.Constraints;
                                        Hu*obj.u(:,i) <= bu;
                                        Hy*obj.y(:,i) <= by];
                end
        end
 
        function compute_cost(obj, Hankel_f)
            r = obj.sys.r;
            Q = obj.sys.Q;
            R = obj.sys.R;
            
%             u_tilde = Hankel_f(1:obj.sys.nu*obj.N_pred,:)*obj.g;
%             y_tilde = Hankel_f(obj.sys.nu*obj.N_pred+1:end,:)*obj.g;
%             obj.Cost = obj.Cost + (y_tilde-r)'*Q*(y_tilde-r) + u_tilde'*R*u_tilde;
%                 
            for i=1:obj.N_pred
                obj.Cost = obj.Cost + (obj.y(:,i)-r)'*Q*(obj.y(:,i)-r) + obj.u(:,i)'*R*obj.u(:,i);
            end 
            obj.Cost = obj.Cost+ 10*norm(obj.slack,1); 
        end
        
        function [u_opt_seq, y_opt_seq,g_opt] = solve(obj)
            u_opt_seq = [];
            y_opt_seq = [];
            options = sdpsettings('verbose',1,'solver', 'quadprog','gurobi.TimeLimit', 10); %gurobi
            optimize(obj.Constraints, obj.Cost, options);   
            if double(obj.u(:,1)) == 0 |isnan(double(obj.u(:,1))) 
                options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
                optimize(obj.Constraints, obj.Cost, options);
            end
            cost_opt = double(obj.Cost);
            cost_opt - double(10*norm(obj.slack,1))
            for i = 1:obj.N_pred
                u_opt_seq = [u_opt_seq, double(obj.u(:,i) )];
                y_opt_seq = [y_opt_seq, double(obj.y(:,i))];
            end
            cost_bilevel = double(obj.g'*obj.Qg*obj.g + 10*obj.slack'*obj.slack);
            g_opt = double(obj.g);
        end 
        
    end
end


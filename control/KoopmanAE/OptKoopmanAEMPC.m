classdef OptKoopmanAEMPC < handle
    % Solve the Koopman predictive control problem
 
    properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        x; u; y;
        Constraints;
        Cost;
    end
    
    methods
        % Methods signature
        [z0, A, B_u, B_d, C] = get_koopman_representation(y0, d0, model)

        % Constructor
        function obj = OptKoopmanAEMPC(sys, N_ini, N_pred)
            obj.N_ini = N_ini;
            obj.N_pred = N_pred;
            obj.sys = sys;
            obj.u = spdvar(N_pred, sys.n_u);
            obj.x = spdvar(N_pred, sys.n_x);
            obj.y = spdvar(N_pred, sys.n_y);
            obj.s = spdvar(N_pred, sys.n_y);
            obj.Constraints = [];
            obj.Cost = 0;
        end
               
        function add_constraints(obj, w)
        %ADD_CONSTRAINT Defines constraints for optimization problem.
            
            con = [];
            % First iteration
            con = con + (obj.x(2,:) == obj.sys.A*(obj.x(1,:)) + obj.sys.B*obj.u(1,:) + obj.sys.E*w(1,:)); %Dynamics
            con = con + (obj.sys.U.A*obj.u(1,:) <= obj.sys.U.b); %Input constraints
            con = con + (obj.sys.Y.A*obj.u(1,:) <= obj.sys.Y.b); %Output constraints
            
            % Following iterations
            for i = 2:obj.N_pred-1
              con = con + (obj.x(i+1,:) == obj.sys.A*(obj.x(i,:)) + obj.sys.B*obj.u(i,:) + obj.sys.E*w(i,:));
              con = con + (obj.sys.U.A*obj.u(1,:) <= obj.sys.U.b);
              con = con + (obj.sys.Y.A*obj.u(1,:) <= obj.sys.Y.b);
            end
            
            % Final iteration
            con = con + (obj.sys.Y.A*obj.u(1,:) <= obj.sys.Y.b);

            obj.Constraints = con;
        end 
 
        function initialization(obj)
        %INITIALIZATION Earases constraints and costs. Should be called at
        %each MPC step.
            obj.Constraints = [];
            obj.Cost = 0;            
        end

        function compute_cost(obj)
        %COMPUTE_COST Calculates the cost of a given solution.
            r = obj.sys.r;
            Q = obj.sys.Q;
            R = obj.sys.R;
            S = obj.sys.S;
            for i=1:obj.N_pred
                obj.Cost = obj.Cost + (obj.y(:,i)-r)'*Q*(obj.y(:,i)-r) + obj.u(:,i)'*R*obj.u(:,i) + obj.s(:,i)'*S*obj.s(:,i);
            end 
        end
 
        function [u_opt_seq, y_opt_seq] = solve(obj)
        %SOLVE Finds solution to constraint optimization problem.
            u_opt_seq = [];
            y_opt_seq = [];
            options = sdpsettings('verbose',1,'solver', 'gurobi','gurobi.TimeLimit', 10);
            optimize(obj.Constraints, obj.Cost, options);
            if double(obj.u(:,1)) == 0 || isnan(double(obj.u(:,1))) 
                options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
                optimize(obj.Constraints, obj.Cost, options);
            end               
            for i = 1:obj.N_pred
                u_opt_seq = [u_opt_seq, double(obj.u(:,i))];
                y_opt_seq = [y_opt_seq, double(obj.y(:,i))];
            end
        end        
    end
end
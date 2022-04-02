classdef OptKoopmanAEMPC < handle
    % Solve the Koopman predictive control problem
 
    properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        z; u; y; s;
        Constraints;
        Cost;
    end
    
    methods
        % Constructor
        function obj = OptKoopmanAEMPC(sys, N_ini, N_pred)
            obj.N_ini = N_ini;
            obj.N_pred = N_pred;
            obj.sys = sys;
            obj.u = sdpvar(N_pred-1, sys.nu);
            obj.z = sdpvar(N_pred, sys.nz);
            obj.y = sdpvar(N_pred, sys.ny);
            obj.s = sdpvar(N_pred, 2*sys.nu);
            obj.Constraints = [];
            obj.Cost = 0;
        end
		
		function [z0, A, B_u, B_d, C, T_min, T_scale] = get_koopman_representation(obj, x0, d0, model)
			% Function that calls the Koopman model in Python and returns the
			% lifted state and Koopman operators.

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%%                   Write measurements to file                    %%%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

			data = struct('x0',x0,'d0',d0);
			json = string(jsonencode(data));


			fid = fopen('KoopmanAE/tmp/measurements.json', 'w');
			fprintf(fid, '%s', json);
			fclose(fid);

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%%                        Run Python script                        %%%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

			commandStr = char('/Users/lucafabietti/Documents/Felix_polydome/proj_env/bin/python KoopmanAE/get_'+model+'_representation.py');
			[status, commandOut] = system(commandStr);
			if status ~= 0
				error("=======Python error======" + commandOut);
			end

			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			%%%                   Get Output of python script                   %%%
			%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

			jsonText = fileread('KoopmanAE/tmp/lifting.json');
			data = jsondecode(jsonText);

			z0 = data.z0;
			A = data.A;
			B_u = data.B_u;
			B_d = data.B_d;
			C = data.C;
			T_min = data.T_min;
			T_scale = data.T_scale;
		end
               
        function add_constraints(obj, w)
        %ADD_CONSTRAINT Defines constraints for optimization problem.
		
			w_min = obj.sys.w_min;
			w_scale = obj.sys.w_scale;
			
			w = w .* w_scale;
			w = w + w_min;
			
            % Slack variable:
            % s(1:N_pred-1,:) = Slack for input constraints
            % s(N_pred,:) = Slack for FIRST output constraint

            con = [];
            % First iteration
			con = con + (obj.z(1,:) == obj.sys.z0); %Initial condition
            con = con + (obj.z(2,:) == (obj.sys.A*(obj.z(1,:)') + obj.sys.B*obj.u(1,:) + obj.sys.E*w(1,:)')'); %Dynamics
			con = con + (obj.y(1,:) == obj.sys.C*(obj.z(1,:)')); %Output dynamics
            con = con + (obj.sys.U_heat.A*obj.u(1,:) <= obj.sys.U_heat.b + diag([1 1])*obj.s(1,:)'); %Input constraints
            con = con + (obj.sys.Y{1}.A*((obj.y(1,:)-obj.sys.T_min)/obj.sys.T_scale) <= obj.sys.Y{1}.b + diag([1 1])*obj.s(obj.N_pred,:)'); %Output constraints
            
            % Following iterations
            for i = 2:obj.N_pred-1
              con = con + (obj.z(i+1,:) == (obj.sys.A*(obj.z(i,:)') + obj.sys.B*obj.u(i,:) + obj.sys.E*w(i,:)')');
			  con = con + (obj.y(i,:) == obj.sys.C*(obj.z(i,:)')); %Output dynamics
              con = con + (obj.sys.U_heat.A*obj.u(i,:) <= obj.sys.U_heat.b + diag([1 1])*obj.s(i,:)'); %Input constraints
              con = con + (obj.sys.Y{i}.A*((obj.y(i,:)-obj.sys.T_min)/obj.sys.T_scale) <= obj.sys.Y{i}.b); %Output constraints
            end
            
            % Final iteration
			con = con + (obj.y(obj.N_pred,:) == obj.sys.C*(obj.z(obj.N_pred,:)')); %Output dynamics
            con = con + (obj.sys.Y{obj.N_pred}.A*((obj.y(obj.N_pred,:)-obj.sys.T_min)/obj.sys.T_scale) <= obj.sys.Y{obj.N_pred}.b);

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
            R = obj.sys.R;
            S = obj.sys.S;
            for i=1:obj.N_pred-1
                obj.Cost = obj.Cost + obj.u(i,:)'*R*obj.u(i,:) + obj.s(i,:)*S*obj.s(i,:)';
			end 
        end
 
        function [u_opt_seq, y_opt_seq, s_opt_seq] = solve(obj)
        %SOLVE Finds solution to constraint optimization problem.
            options = sdpsettings('verbose',1,'solver', 'gurobi','gurobi.TimeLimit', 10);
            optimize(obj.Constraints, obj.Cost, options);
%             if double(obj.u(:,1)) == 0 || isnan(double(obj.u(:,1))) 
%                 options = sdpsettings('verbose',1,'solver','quadprog','gurobi.TimeLimit', 20);
%                 optimize(obj.Constraints, obj.Cost, options);
%             end      
			u_opt_seq = [double(obj.u(:,:))];
			y_opt_seq = (double(obj.y(:,:)) - obj.sys.T_min)/obj.sys.T_scale;
			s_opt_seq = double(obj.s(:,:));
        end        
    end
end
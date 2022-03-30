classdef PredictiveController < Controller
	%MPCCONTROLLER Implements a standard MPC Controller
	
	properties
		%
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        T; % Number of data in Hankel matrix
        Hankel_ini; % Hankel matrix for initial steps
        Hankel_pred; % Hankel matrix for prediction steps
        Hankel_u; Hankel_y; Hankel_w;
        Hankel_u_heat; Hankel_y_heat; Hankel_w_heat;
        Hankel_u_cool; Hankel_y_cool; Hankel_w_cool;
        Qg; % the weights for g vector: L2 norm or adaptive term
        w_s; % the weights for slack
        opt_deepc; % optimal deepc problem solver
        soltion_cache;
        soltion_cache_vector; 
        last_forecast % last available forecast
        last_dist % last measured disturbance
        last_mode
        mode_count % Within 1 mode, count the number of steps, for hankel matrix updating 
        
		%
	end%%
	%
	methods
		% Methods signature
		signal = get_disturbance(obj) % Forecast from current time to future after iN steps
        change_mode(obj,mode) % update some parameters when mode is changed
% 		getLastDisturbance(obj) % Last measured disturbance
% 		getLastStateEstimate(hController) % Kalman filtering
% 		updatePredHorizon(hController) % Update the pre horizon
% 		setStandardSetPoint(hController) % Reset the temp setpoint to the building to nominal
        
        compute_Hankel_matrix(obj, u, w, y) % Compute the hankel matrix  

		
		% Constructor
        function obj = PredictiveController(parameter)
            obj = obj@Controller(parameter);
            obj.sys = parameter.sys;
            obj.N_ini = parameter.N_ini;
            obj.N_pred = parameter.N_pred;
            obj.T = parameter.T;
            obj.Qg = parameter.Qg;
            obj.w_s = parameter.w_s;
            obj.soltion_cache = [];
            obj.Hankel_u_heat=parameter.Hankel_u_heat; obj.Hankel_y_heat=parameter.Hankel_y_heat; 
            obj.Hankel_w_heat=parameter.Hankel_w_heat;
            obj.Hankel_u_cool=parameter.Hankel_u_cool; obj.Hankel_y_cool=parameter.Hankel_y_cool; 
            obj.Hankel_w_cool=parameter.Hankel_w_cool;        
            obj.last_mode = -1; 
            obj.mode_count = 0;
        end        
        
        function [input] = sample_random_input(obj, upper_bound, lower_bound)
        % COMUPTE_RANDOM_INPUT Sample a random input (Prepare data for DeePC)
            input = lower_bound + (upper_bound-lower_bound)*rand;
        end

        function initialize_deepc(obj)
            obj.opt_deepc = OptDeePC(obj.sys, obj.N_ini, obj.N_pred, obj.T);
        end
        
        function initialize_deepc_TV(obj, t)
            obj.opt_deepc = OptDeePCTV(obj.sys, obj.N_ini, obj.N_pred, obj.T, t);
        end
        
        function initialize_deepc_noise_TV(obj, t)
            obj.opt_deepc = OptDeePCNoiseTV(obj.sys, obj.N_ini, obj.N_pred, obj.T, t);
        end        
        
        function initialize_deepc_robust(obj)
            obj.opt_deepc = OptDeePCRobust(obj.sys, obj.N_ini, obj.N_pred);
        end            
           
        function set_deepc(obj, u_ini, y_ini, option)
            % Input:
            %   u_ini, y_ini: intial input and output sequence
            
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel(obj.Hankel_ini, obj.Hankel_pred, u_ini, y_ini);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost();
            
            if option == "L1"
                obj.opt_deepc.add_g_penalty(obj.Qg);
                fprintf('Initialize L1 DeePC solver \n' )
            elseif option == "adaptive"
                obj.opt_deepc.add_adaptive_KKT(obj.Hankel_ini, obj.Hankel_pred, obj.Qg)
                fprintf('Initialize adaptive DeePC solver \n' )
            else
                fprintf('Initialize basic DeePC solver \n' )
            end
        end

       
        
        function set_deepc_noise(obj, option)
            % Input:
            %   u_ini, w_ini, y_ini: intial input and output sequence
           
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel_noise(obj.Hankel_ini, obj.Hankel_pred);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost(obj.w_s);
            
            if option == 'L2'
                obj.opt_deepc.add_g_penalty(obj.Qg);
                fprintf('Initialize L2 DeePC solver \n' )
            elseif option == 'adaptive'
                obj.opt_deepc.add_adaptive_KKT(obj.Hankel_ini, obj.Hankel_pred, obj.Qg)
                fprintf('Initialize adaptive DeePC solver \n' )
            else
                fprintf('Initialize basic DeePC solver \n' )
            end
            obj.opt_deepc.get_ctrl();  
        end   
        
        function set_deepc_robust(obj,soft)
            if nargin<2
                soft = 0;
            end            
            obj.opt_deepc.initialization();          
            obj.opt_deepc.add_constraint(obj.Hankel_ini, obj.Hankel_pred,obj.Qg,soft);
            obj.opt_deepc.compute_cost(obj.w_s,soft);
            
            obj.opt_deepc.get_ctrl();       
        end        
        function set_deepc_robust_du(obj,soft)
            if nargin<2
                soft = 0;
            end            
            obj.opt_deepc.initialization();          
            obj.opt_deepc.add_constraint_du(obj.Hankel_ini, obj.Hankel_pred,obj.Qg,soft);
            obj.opt_deepc.compute_cost(obj.w_s,soft);
            
            obj.opt_deepc.get_ctrl();       
        end        
        
        function [u_opt, u_opt_seq, y_opt_seq] = solve(obj)
            % Output:
            %   u_opt: input used in the closed loop system
            [u_opt_seq, y_opt_seq, g_opt, slack_opt] = obj.opt_deepc.solve();
            obj.soltion_cache = struct(...
                'u_opt_seq', u_opt_seq, 'y_opt_seq', y_opt_seq, 'g_opt', g_opt, 'slack_opt', slack_opt );
            u_opt = u_opt_seq(:, 1);
        end   
        
        function [u_opt, u_opt_seq, y_opt_seq] = solve_deepc(obj, t, u_ini, w_ini, y_ini,w_pred)
            % Prepare the time-varying reference, Q, yb, w_pred 
            ref = obj.sys.r(t+1:t+obj.N_pred)';
%             Q = [];
            by = [];
            for i = 1:obj.N_pred
%                 Q = [Q;obj.sys.Q{t+i}];
                by = [by; obj.sys.Y{t+i}.b];
            end
%             Q = diag(Q);
%             w_pred = obj.sys.w_pred(:,t:t+obj.N_pred-1);
%             w_pred = reshape(w_pred, [obj.sys.nw*obj.N_pred,1]);
            
            [sol,flag] = obj.opt_deepc.ctrl({y_ini, u_ini, w_ini, w_pred, ref, by});
            u_opt_seq = sol{1}; y_opt_seq = sol{2}; g_opt = sol{3}; slack_opt = sol{4};
            if flag ~= 0
                msg = yalmiperror(flag);
                error(msg);
            end
            obj.soltion_cache = struct(...
                'u_opt_seq', u_opt_seq, 'y_opt_seq', y_opt_seq, 'g_opt', g_opt,'slack_opt',slack_opt );
            y_opt_seq' %obj.soltion_cache;
            u_opt_seq'
            slack_opt'
            u_opt = u_opt_seq(1:obj.sys.nu);   
        end        
		%
	end%%
	%
end%%


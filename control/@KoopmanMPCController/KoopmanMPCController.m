classdef KoopmanMPCController < Controller
	%MPCCONTROLLER Implements a standard MPC Controller
	
	properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        T; % Number of data in Hankel matrix
        soltion_cache;
        soltion_cache_vector; 
        last_forecast % last available forecast
        last_dist % last measured disturbance
        last_mode
        mode_count
		opt_koopman_ae;
    end

	methods
		% Methods signature
		signal = get_disturbance(obj) % Forecast from current time to future after iN steps
        change_mode(obj,mode) % update some parameters when mode is changed
% 		getLastDisturbance(obj) % Last measured disturbance
% 		getLastStateEstimate(hController) % Kalman filtering
% 		updatePredHorizon(hController) % Update the pre horizon
% 		setStandardSetPoint(hController) % Reset the temp setpoint to the building to nominal
		
		% Constructor
        function obj = KoopmanMPCController(parameter)
            obj = obj@Controller(parameter);
            obj.sys = parameter.sys;
            obj.N_ini = parameter.N_ini;
            obj.N_pred = parameter.N_pred;
            obj.T = parameter.T;
            obj.soltion_cache = [];      
            obj.last_mode = -1; 
            obj.mode_count = 0;
        end        
        
        function [input] = sample_random_input(obj, upper_bound, lower_bound)
        %SAMPLE_RANDOM_INPUT Sample a random input (Prepare data for DeePC)
            input = lower_bound + (upper_bound-lower_bound)*rand;
        end

        function initialize_mpc_controller(obj)
            obj.opt_koopman_ae = OptKoopmanAEMPC(obj.sys, obj.N_ini, obj.N_pred);
        end         
           
        function set_mpc_controller(obj, w)
            % Input:
            %   u_ini, y_ini: intial input and output sequence
            
            obj.opt_koopman_ae.initialization();
            obj.opt_koopman_ae.add_constraints(w);
            obj.opt_koopman_ae.compute_cost();
        end     
        
        function get_koopman_representation(obj, y0, d0, model)
            [z0, A, B_u, B_d, C] = obj.opt_koopman_ae.get_koopman_representation(y0, d0, model);
            obj.sys.z0 = z0;
            obj.sys.A = A;
            obj.sys.B = B_u;
            obj.sys.E = B_d;
            obj.sys.C = C;
        end

        function [u_opt, u_opt_seq, y_opt_seq] = solve(obj)
            % Output:
            %   u_opt: input used in the closed loop system
            [u_opt_seq, y_opt_seq] = obj.opt_koopman_ae.solve();
            obj.soltion_cache = struct(...
                'u_opt_seq', u_opt_seq, 'y_opt_seq', y_opt_seq);
            u_opt = u_opt_seq(:, 1);
        end        
		%
	end%%
	%
end%%
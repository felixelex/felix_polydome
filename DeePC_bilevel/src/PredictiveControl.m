classdef PredictiveControl < handle
    
    properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        T; % Number of data in Hankel matrix
        opt_deepc; % optimal deepc problem solver
        opt_mpc; % optimal mpc problem solver
        soltion_cache;
        soltion_cache_vector;
        mycontroller; % defined yalmip controller
    end
    
    methods
        function obj = PredictiveControl(sys, N_ini, N_pred, T)
            obj.sys = sys;
            obj.N_ini = N_ini;
            obj.N_pred = N_pred;
            obj.T = T;
            obj.soltion_cache = [];
%             obj.soltion_cache_vector{3} = [];
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
        
        function set_deepc(obj,Hankel_p, Hankel_f, u_ini, y_ini)
            % Input:
            %   u_ini, y_ini: intial input and output sequence
            %   Hankel_mat: Hankel matrix
            %   U, Y: constraints
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel(Hankel_p, Hankel_f, u_ini, y_ini);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost();
        end

        function set_deepc_L1(obj,Hankel_p, Hankel_f, u_ini, y_ini, Qg)
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel(Hankel_p, Hankel_f, u_ini, y_ini);            
            obj.opt_deepc.add_ineq_constraint(Hankel_f);
            obj.opt_deepc.compute_cost(Hankel_f);
            
            obj.opt_deepc.add_g_penalty(Qg);
        end        

        function set_deepc_adaptive(obj,Hankel_p, Hankel_f, u_ini, y_ini, Qg)
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel(Hankel_p, Hankel_f, u_ini, y_ini);            
            obj.opt_deepc.add_ineq_constraint(Hankel_f);
            obj.opt_deepc.compute_cost(Hankel_f);
            
            obj.opt_deepc.add_adaptive_KKT(Hankel_p, Hankel_f, Qg)
            
        end 

        function set_deepc_noise(obj,Hankel_p, Hankel_f, u_ini, w_ini, y_ini)
            % Input:
            %   u_ini, y_ini: intial input and output sequence
            %   Hankel_mat: Hankel matrix
            %   U, Y: constraints
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel_noise(Hankel_p, Hankel_f, u_ini, w_ini, y_ini);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost();
        end
        
        function set_deepc_L2_noise(obj,Hankel_p, Hankel_f, Qg, w_s)
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel_noise(Hankel_p, Hankel_f);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost(w_s);
            
            obj.opt_deepc.add_g_penalty(Qg);
            obj.opt_deepc.get_ctrl();
        end       
        
        function set_deepc_adaptive_noise(obj, Hankel_p, Hankel_f, Qg, w_s)
            obj.opt_deepc.initialization();
            obj.opt_deepc.add_Hankel_noise(Hankel_p, Hankel_f);            
            obj.opt_deepc.add_ineq_constraint();
            obj.opt_deepc.compute_cost(w_s);
            
            obj.opt_deepc.add_adaptive_KKT(Hankel_p, Hankel_f, Qg, w_s);
            obj.opt_deepc.get_ctrl();       
        end     
        
        function set_deepc_robust(obj, Hankel_p, Hankel_f, Qg, w_s, soft)
            if nargin<6
                soft = 0;
            end
            obj.opt_deepc.initialization();          
            obj.opt_deepc.add_constraint(Hankel_p, Hankel_f,Qg,soft);
            obj.opt_deepc.compute_cost(w_s,soft);
            
            obj.opt_deepc.get_ctrl(soft);       
        end            
        
        
        function set_deepc_robust_du(obj, Hankel_p, Hankel_f, Qg, w_s)

            obj.opt_deepc.initialization();          
            obj.opt_deepc.add_constraint_du(Hankel_p, Hankel_f,Qg);
            obj.opt_deepc.compute_cost(w_s,1);
            
            obj.opt_deepc.get_ctrl(1);       
        end    
        
        function [u_opt, u_opt_seq, y_opt_seq] = solve(obj,Hankel_p,Hankel_f, u_ini, y_ini)
            % Output:
            %   u_opt: input used in the closed loop system
            [u_opt_seq, y_opt_seq, g_opt] = obj.opt_deepc.solve();
            obj.soltion_cache = struct(...
                'u_opt_seq', u_opt_seq, 'y_opt_seq', y_opt_seq, 'g_opt', g_opt );
            obj.soltion_cache
            u_opt = u_opt_seq(:, 1);
        end
        
        function [u_opt, u_opt_seq, y_opt_seq] = solve_mpc(obj, x0,t,w_real)
            obj.opt_mpc = MPCSolver(obj.sys, obj.N_pred,t);
            obj.opt_mpc.initialization(x0,w_real);
            obj.opt_mpc.add_ineq_constraint();
            obj.opt_mpc.compute_cost();  
            
            [u_opt_seq, y_opt_seq] = obj.opt_mpc.solve();
            u_opt = u_opt_seq(:, 1);            
        end      
        
        function [u_opt, u_opt_seq, y_opt_seq] = solve_deepc(obj, t, u_ini, w_ini, y_ini,Hankel_p,Hankel_f)
            % Prepare the time-varying reference, Q, yb, w_pred 
            ref = obj.sys.r(t+1:t+obj.N_pred)';
%             Q = [];
            by = [];
            for i = 1:obj.N_pred
%                 Q = [Q;obj.sys.Q{t+i}];
                by = [by; obj.sys.Y{t+i}.b];
            end
%             Q = diag(Q);
            w_pred = obj.sys.w_pred(:,t:t+obj.N_pred-1);
            w_pred = reshape(w_pred, [obj.sys.nw*obj.N_pred,1]);
            
            [sol,flag] = obj.opt_deepc.ctrl({y_ini, u_ini, w_ini, w_pred, ref, by});
            u_opt_seq = sol{1}; y_opt_seq = sol{2}; g_opt = sol{3}; slack = sol{4};
            if flag ~= 0
                msg = yalmiperror(flag);
                error(msg);
            end
            obj.soltion_cache = struct(...
                'u_opt_seq', u_opt_seq, 'y_opt_seq', y_opt_seq, 'g_opt', g_opt,'slack',slack );
            y_opt_seq' %obj.soltion_cache;
            u_opt_seq'
            slack'
            u_opt = u_opt_seq(1:obj.sys.nu);   
        end
     
        
    end
end


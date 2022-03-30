classdef OptDeePCNoiseTV < OptDeePC
    % Solve the DeePC optimal problem with some time-varying terms:
    %   min_g sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. Hankel_mat * g = [u_ini; y_ini; u; y]
    %            u_k \in U_k, y_k \in Y_k
    %   where g is decision variables
 
    properties
        y_ini; u_ini; w_ini; w_pred;
        t; % current time
        kappa; % dual variables in KKT condition
%         Q; % time-varying penalty for y
        ref; % time-varying penalty for y
        by; % time-varying constriants for y
        ctrl; % the defined yalmip controller
    end
    
    methods
        function obj = OptDeePCNoiseTV(sys, N_ini, N_pred, T, t)
            obj = obj@OptDeePC(sys, N_ini, N_pred, T);
            obj.t = t;
            obj.y_ini = sdpvar(sys.ny*(N_ini+1),1,'full');
            obj.u_ini = sdpvar(sys.nu*N_ini,1,'full');
            obj.w_ini = sdpvar(sys.nw*N_ini,1,'full');
            obj.w_pred = sdpvar(sys.nw*N_pred,1,'full');
            obj.kappa = sdpvar((sys.nu+sys.nw)*(N_ini+N_pred) + sys.ny*(N_ini+1), 1);
            obj.slack = sdpvar(sys.ny*(N_ini+1), 1);
%             obj.Q = sdpvar(sys.ny*N_pred,sys.ny*N_pred,'full');
            obj.ref = sdpvar(sys.ny*N_pred,1,'full');
            obj.by = sdpvar(sys.ny*2*N_pred,1,'full'); % use box constraints 
        end
        
        function initialization(obj)
            obj.Constraints = [];
            obj.Cost = 0;            
        end        
        
        function add_g_penalty(obj, Qg)
            obj.Cost = obj.Cost + obj.g'*Qg*obj.g;
        end
         
        
        function add_Hankel_noise(obj, Hankel_p,Hankel_f)
            obj.Constraints = [obj.Constraints;
                                Hankel_p*obj.g == [obj.u_ini; obj.w_ini; obj.y_ini+obj.slack]];
            uwy = Hankel_f*obj.g;
            obj.Constraints = [obj.Constraints;
                                uwy(obj.sys.nu*obj.N_pred+1:(obj.sys.nu+obj.sys.nw)*obj.N_pred,:) ...
                                    == obj.w_pred];               
         
            obj.u = uwy(1:obj.sys.nu*obj.N_pred,:);
            obj.y = uwy((obj.sys.nu+obj.sys.nw)*obj.N_pred+1:end,:);
        end
        
        function add_adaptive_KKT(obj, Hankel_p, Hankel_f, Qg)
            H = [Hankel_p; Hankel_f(1:(obj.sys.nu+obj.sys.nw)*obj.N_pred, :)];
            obj.Constraints = [obj.Constraints;
                                Qg*obj.g + H'*obj.kappa == 0;
                                obj.slack - obj.kappa(1:obj.sys.ny*(obj.N_ini+1),:)]; %2*w_s*obj.slack - obj.kappa(1:obj.sys.ny*(obj.N_ini+1),:)];
        end
        
        function add_ineq_constraint(obj)
            U = obj.sys.U;
            Hu  = U.A;  bu  = U.b;
            Hu = kron(eye(obj.N_pred),Hu); bu = kron(ones(obj.N_pred,1),bu); 
            Hy = kron(eye(obj.N_pred),[eye(obj.sys.ny);-eye(obj.sys.ny)]);             
            obj.Constraints = [obj.Constraints;
                                    Hu*obj.u <= bu;
                                    Hy*obj.y <= obj.by];
        end
 
        function compute_cost(obj, w_s)
            Q = obj.sys.Q;
            R = kron(ones(obj.N_pred,1),obj.sys.R);
            obj.Cost = obj.Cost + Q*(obj.y-obj.ref)'*(obj.y-obj.ref) + obj.u'*R;
            obj.Cost = obj.Cost+ w_s*obj.slack'*obj.slack; 
        end
        
        function get_ctrl(obj)
            opts = sdpsettings('solver','gurobi','verbose',1);
            obj.ctrl = optimizer(obj.Constraints,obj.Cost,opts,{obj.y_ini, obj.u_ini, obj.w_ini, obj.w_pred, obj.ref, obj.by},...
                {obj.u, obj.y, obj.g, obj.slack});
            obj.ctrl.options.verbose = 1;
        end
        
        
    end
end


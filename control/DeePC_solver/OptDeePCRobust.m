classdef OptDeePCRobust < handle
    % Solve the DeePC optimal problem:
    %   min_g sum_{k=0}^{N_pred-1}{||y_k - r_{t+k}||_Q^2 + ||u_k||_R^2}
    %       s.t. Hankel_mat * g = [u_ini; y_ini; u; y]
    %            u_k \in U, y_k \in Y
    %   where g is decision variables
 
    properties
        sys;
        N_ini; % Initial steps
        N_pred; % Prediction steps
        y_ini; u_ini; w_ini;      
        u_norm; u_bound; % L1 norm
        y_norm; 
        w_norm; % Predicted noises, like temperature prediction by weather api
        w_p; % Random perturbation noises happenning in the fuction 
        K; % feedback control law
        Ku;
        g_norm; % Decision variables
        slack;
        slack_pred;
        Constraints;
        Cost;        
        t; % Current time
%         Q; % Time-varying penalty for y
        ref; % Time-varying penalty for y
        f_y; % Time-varying constriants for y
        ctrl; % Defined yalmip controller       
    end
    
    methods
        function obj = OptDeePCRobust(sys, N_ini, N_pred)
            obj.N_ini = N_ini;
            obj.N_pred = N_pred;
            obj.sys = sys;
            
            obj.y_ini = sdpvar(sys.ny*(N_ini+1),1,'full');
            obj.u_ini = sdpvar(sys.nu*N_ini,1,'full');
            obj.w_ini = sdpvar(sys.nw*N_ini,1,'full');
            obj.u_norm = sdpvar(sys.nu*N_pred,1,'full'); ... nominal control input
            obj.u_bound = sdpvar(length(obj.u_norm),1,'full');
            obj.y_norm = [];
            obj.w_norm = sdpvar(sys.nw*N_pred,1,'full');
            obj.w_p = sdpvar(sys.nw*N_pred,1,'full');
            
            %feedback control law(block lower triangle)
            temp = kron((tril(ones(N_pred))-eye(N_pred)),ones(obj.sys.nu,obj.sys.nw));
            obj.K = sdpvar(obj.sys.nu*N_pred,obj.sys.nw*N_pred,'full').*temp;  
            temp = kron((tril(ones(N_pred))-eye(N_pred)),ones(obj.sys.nu,obj.sys.nu));
            obj.Ku = sdpvar(obj.sys.nu*N_pred,obj.sys.nu*N_pred,'full').*temp;  
            
            obj.g_norm = [];
            obj.slack = [];
            obj.Constraints = [];
            obj.Cost = 0;           
            
%             obj.Q = sdpvar(sys.ny*N_pred,sys.ny*N_pred,'full');
            obj.ref = sdpvar(sys.ny*N_pred,1,'full');
            obj.f_y = sdpvar(sys.ny*2*N_pred,1,'full'); % use box constraints             
        end
        
        function initialization(obj)
            obj.Constraints = [];
            obj.Cost = 0;            
        end        
 
        function add_constraint(obj,Hankel_p,Hankel_f,w_g_tilde, soft)
            nu = obj.sys.nu; nw = obj.sys.nw; ny = obj.sys.ny;

            Hu_init = Hankel_p(1:nu* obj.N_ini,:);
            Hw_init = Hankel_p(nu* obj.N_ini+1:(nu+nw)*obj.N_ini,:);
            Hy_init = Hankel_p((nu+nw)*obj.N_ini+1:end,:);
            Hu_pred = Hankel_f(1:nu* obj.N_pred,:);
            Hw_pred = Hankel_f(nu* obj.N_pred+1:(nu+nw)*obj.N_pred,:);
            Hy_pred = Hankel_f((nu+nw)*obj.N_pred+1:end,:);            
            H = [Hu_init;Hw_init;Hw_pred;Hu_pred];
            
            % ====== Compute g_norm from KKT condition
            % the KKT matrix is
            %[Hy_init'*Hy_init+w_g_tilde , H';H,O], use block matrix inversion to
            %directly calculate g, where we need the inversion of
            %Hy_init'*Hy_init+w_g_tilde, which we use matrix inversion lemma to lower
            %the computational cost to O((ny*obj.N_ini)^2), following second temp is this
            %inversion
            temp = inv(w_g_tilde);
            % desired inversion
            temp = temp - temp*Hy_init'*((eye((obj.N_ini+1)*ny)... %/(2*w_s)...
                +Hy_init*temp*Hy_init')\Hy_init)*temp;
            temp_inv = temp*H'/(H*temp*H');
            % nominal g
            obj.g_norm = (temp-temp*H'*((H*temp*H')\H)*temp)*Hy_init'*obj.y_ini...% *(2*w_s) ...
                 + temp_inv*[obj.u_ini;obj.w_ini;obj.w_norm;obj.u_norm];

            obj.y_norm = Hy_pred*obj.g_norm;
            obj.slack = Hy_init*obj.g_norm-obj.y_ini;

            % ====== Constraints
            U = obj.sys.U;
            F_u  = U.A;  f_u  = U.b;
            F_u = kron(eye(obj.N_pred),F_u); f_u = kron(ones(obj.N_pred,1),f_u); 
            F_y = kron(eye(obj.N_pred),[eye(obj.sys.ny);-eye(obj.sys.ny)]); 
            W = obj.sys.W;
            F_w  = W.A;  f_w  = W.b;
            F_w = kron(eye(obj.N_pred),F_w); f_w = kron(ones(obj.N_pred,1),f_w);             
            
            % robust constraints, each column corresponds to one robust constraint
            % robust input cons
            lambda_u = sdpvar(size(F_w,1),size(F_u,1),'full');        
            if soft == 1
                obj.slack_pred = sdpvar(size(F_u,1),1,'full');
                obj.Constraints = [obj.Constraints;lambda_u'*f_w<=f_u + obj.slack_pred  - F_u*obj.u_norm;
					obj.slack_pred>=0];
            else              
                obj.Constraints = [obj.Constraints;lambda_u'*f_w<=f_u-F_u*obj.u_norm];
            end
            for i = 1:size(lambda_u,2)
                obj.Constraints = [obj.Constraints;F_w'*lambda_u(:,i)==(F_u(i,:)*obj.K)'];
                obj.Constraints = [obj.Constraints;lambda_u(:,i)>=zeros(size(F_w,1),1)];
            end
            % robust output cons
            % perturbation on y_pred is temp_inv(:,(nw+nu)*obj.N_ini+1:end)[I;K]*w_pred
            temp_F_y = F_y*Hy_pred*temp_inv(:,(nw+nu)*obj.N_ini+1:end);
            lambda_y = sdpvar(size(F_w,1),size(temp_F_y,1),'full');
            temp = [eye(nw*obj.N_pred);obj.K];
            obj.Constraints = [obj.Constraints;lambda_y'*f_w<=obj.f_y-F_y*obj.y_norm];
            for i = 1:size(lambda_y,2)
                obj.Constraints = [obj.Constraints;F_w'*lambda_y(:,i)==(temp_F_y(i,:)*temp)'];
                obj.Constraints = [obj.Constraints;lambda_y(:,i)>=zeros(size(F_w,1),1)];
            end            
        end
        

        
 
        function compute_cost(obj, w_s, soft)
            obj.Constraints = [obj.Constraints; -obj.u_bound <= obj.u_norm  <= obj.u_bound];
            Q = obj.sys.Q;
            R = kron(ones(obj.N_pred,1),obj.sys.R);
            obj.Cost = obj.Cost + Q*(obj.y_norm-obj.ref)'*(obj.y_norm-obj.ref) + obj.u_bound'*R;
            obj.Cost = obj.Cost+ w_s*obj.slack'*obj.slack; % 100* obj.slack'*obj.slack; 
            if soft==1
                obj.Cost = obj.Cost+ w_s*obj.slack_pred'*obj.slack_pred;
            end             
        end
        
        function get_ctrl(obj)
            opts = sdpsettings('solver','gurobi','verbose', 0, 'gurobi.TimeLimit', 10);
            obj.ctrl = optimizer(obj.Constraints,obj.Cost,opts,{obj.y_ini, obj.u_ini, obj.w_ini, obj.w_norm, obj.ref, obj.f_y},...
                {obj.u_norm, obj.y_norm, obj.g_norm, obj.slack});
            obj.ctrl.options.verbose = 1;
        end  
        
        
        function add_constraint_du(obj,Hankel_p,Hankel_f,w_g_tilde, soft)
            nu = obj.sys.nu; nw = obj.sys.nw; ny = obj.sys.ny;

            Hu_init = Hankel_p(1:nu* obj.N_ini,:);
            Hw_init = Hankel_p(nu* obj.N_ini+1:(nu+nw)*obj.N_ini,:);
            Hy_init = Hankel_p((nu+nw)*obj.N_ini+1:end,:);
            Hu_pred = Hankel_f(1:nu* obj.N_pred,:);
            Hw_pred = Hankel_f(nu* obj.N_pred+1:(nu+nw)*obj.N_pred,:);
            Hy_pred = Hankel_f((nu+nw)*obj.N_pred+1:end,:);            
            H = [Hu_init;Hw_init;Hw_pred;Hu_pred];
            
            % ====== Compute g_norm from KKT condition
            % the KKT matrix is
            %[Hy_init'*Hy_init+w_g_tilde , H';H,O], use block matrix inversion to
            %directly calculate g, where we need the inversion of
            %Hy_init'*Hy_init+w_g_tilde, which we use matrix inversion lemma to lower
            %the computational cost to O((ny*obj.N_ini)^2), following second temp is this
            %inversion
            temp = inv(w_g_tilde);
            % desired inversion
            temp = temp - temp*Hy_init'*((eye((obj.N_ini+1)*ny)... %/(2*w_s)...
                +Hy_init*temp*Hy_init')\Hy_init)*temp;
            temp_inv = temp*H'/(H*temp*H');
            % nominal g
            obj.g_norm = (temp-temp*H'*((H*temp*H')\H)*temp)*Hy_init'*obj.y_ini...% *(2*w_s) ...
                 + temp_inv*[obj.u_ini;obj.w_ini;obj.w_norm;obj.u_norm];

            obj.y_norm = Hy_pred*obj.g_norm;
            obj.slack = Hy_init*obj.g_norm-obj.y_ini;

            % ====== Constraints
            U = obj.sys.U;
            F_u  = U.A;  f_u  = U.b;
            F_u = kron(eye(obj.N_pred),F_u); f_u = kron(ones(obj.N_pred,1),f_u); 
            F_y = kron(eye(obj.N_pred),[eye(obj.sys.ny);-eye(obj.sys.ny)]); 
            % !!! New noise bound: F_w*[du;dw]<=f_w,
            % [du;dw] = dw_new
            dU = obj.sys.dU;
            W = obj.sys.W;
            F_w1  = dU.A;  f_w1  = dU.b;
            F_w2  = W.A;  f_w2  = W.b;
            F_w = blkdiag(kron(eye(obj.N_pred),F_w1), kron(eye(obj.N_pred),F_w2)); 
            f_w = [kron(ones(obj.N_pred,1),f_w1); kron(ones(obj.N_pred,1),f_w2)];
            % !!! New K: 
            K_tem1 = [eye(obj.sys.nu*obj.N_pred)+obj.Ku,obj.K]; % u = u_bar + Kdw + du + Kudu = u_bar+ K_new*dw_new
            K_tem2 = [zeros(nw*obj.N_pred,obj.sys.nu*obj.N_pred),eye(nw*obj.N_pred);
                eye(obj.sys.nu*obj.N_pred)+obj.Ku,obj.K]; % u = u_bar + Kdw + du + Kudu = u_bar+ K_new*dw_new                       
            
            % robust constraints, each column corresponds to one robust constraint
            % robust input cons
            lambda_u = sdpvar(size(F_w,1),size(F_u,1),'full');        
            if soft == 1
                obj.slack_pred = sdpvar(size(F_u,1),1,'full');
                obj.Constraints = [obj.Constraints;lambda_u'*f_w<=f_u + obj.slack_pred  - F_u*obj.u_norm;
					obj.slack_pred>=0];
            else              
                obj.Constraints = [obj.Constraints;lambda_u'*f_w<=f_u-F_u*obj.u_norm];
            end
            for i = 1:size(lambda_u,2)
                obj.Constraints = [obj.Constraints;F_w'*lambda_u(:,i)==(F_u(i,:)*K_tem1)']; %!!!
                obj.Constraints = [obj.Constraints;lambda_u(:,i)>=zeros(size(F_w,1),1)];
            end
            % robust output cons
            % perturbation on y_pred is temp_inv(:,(nw+nu)*obj.N_ini+1:end)[I;K]*w_pred
            temp_F_y = F_y*Hy_pred*temp_inv(:,(nw+nu)*obj.N_ini+1:end);
            lambda_y = sdpvar(size(F_w,1),size(temp_F_y,1),'full');
 
            obj.Constraints = [obj.Constraints;lambda_y'*f_w<=obj.f_y-F_y*obj.y_norm];
            for i = 1:size(lambda_y,2)
                obj.Constraints = [obj.Constraints;F_w'*lambda_y(:,i)==(temp_F_y(i,:)*K_tem2)']; %!!!
                obj.Constraints = [obj.Constraints;lambda_y(:,i)>=zeros(size(F_w,1),1)];
            end            
        end        
        
    end
end


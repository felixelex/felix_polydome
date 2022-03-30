function parameter = KoopmanAE_par(example)
%MPCCONTROLLER_PAR Set the parameters for the MPC controller
	
	parameter = controller_par(); %sets Ts to 900
	parameter.str_ctr_type = 'Koopman_AE_MPC_Controller';
    parameter.model = "ae";
    
    parameter.N_pred    = 10*900/parameter.Ts; % # of prediction steps
    parameter.N_ini     = 10*900/parameter.Ts; % # of prediction steps
    parameter.T         = -1; % Obsolete parameter for us
    %UNDERSTAND: What does this parameteer stand for? total number of steps
    %has alread been defined in controller_par()
    parameter.T_experiment = 24*11*60*60/parameter.Ts; % # of experiemt steps
            
    % real system
    if example == 0
        nu = 1; % HP electrical power
        nx = 3;
        ny = 1;
        nw = 2;

        % Matrix in stage cost
        Q = 0*eye(1);
        R = 0.1;
        r = 0; % no reference
        S = 10; % soft constraints penalty
        
        T_tot = parameter.T_experiment;
        % Constraints
        Hy = [eye(1);-eye(1)];
        for i = 1:T_tot
             if i <= parameter.N_ini
                 % Constraints during initialization
                 by = [23; -21];
                 % r(i) = 21;
             else
                 if  mod(i - parameter.N_ini,96)<=56 && mod(i - parameter.N_ini,96)>=1
                     % Constraints during night
                     by = [24; -20];
                     % r(i) = 21;
                 else
                     % Constraints during day
                     by = [23; -21];
                     % r(i) = 21;
                 end
             end
             Y{i} = Polyhedron('A', Hy, 'b', by);
        end
        val_u = 6.0; 
        U_heat = Polyhedron([val_u; 0]);
%         val_u = 4.6; 
%         U_cool = Polyhedron([val_u; 0]);       
        val_u = 0.8;  % 6kW/15 
        dU_heat = Polyhedron([val_u; 0]);
%         val_u = 0.6;  % 4.6kW/15 
%         dU_cool = Polyhedron([val_u; 0]);       
        Hw = [eye(2);-eye(2)];
        bw = [1.0;0.05;1.0;0.05];     
        W = Polyhedron('A', Hw, 'b', bw);
        
        parameter.sys.U_heat = U_heat;
        parameter.sys.dU_heat = dU_heat;
%         parameter.sys.U_cool = U_cool;
%         parameter.sys.dU_cool = dU_cool; 
        parameter.sys.W = W;
        parameter.sys.nw = nw;  
        
    end
    
    parameter.sys.Q = Q;
    parameter.sys.R = R;
    parameter.sys.r = r;
    parameter.sys.nx = nx;
    parameter.sys.ny = ny;
    parameter.sys.nu = nu;
    parameter.sys.Y = Y;  
    
end%%

%%
function [w_pred, w_real,w_y,w_rand, W] = get_ini_dist(T_day,num_day,num_T, var_y)
    Hw = [eye(3);-eye(3)];
    bw = [2;2;2;2;2;2];
    W = Polyhedron('A', Hw, 'b', bw);

    w1_rand = -3 + 6*rand(1,T_day*num_day);
    w2_rand = -5 + 10*rand(1,T_day*num_day);
    w3_rand = -2 + 4*rand(1,T_day*num_day);

    w1_pred = zeros(1,T_day*num_day);
    w2_pred = zeros(1,T_day*num_day);
    w3_pred = zeros(1,T_day*num_day);

        for j = 0:num_T-1
            % Disturbance
            a1 = 10 + 4*rand; 
            a2 = 2 + 4*rand; 
            a3 = 0 + 16*rand; 
            a4 = 0 + 2*rand; 
            a5 = 6 + 1*rand; 
%         a1 = 15 ; 
%         a2 = 4; 
%         a3 = 8; 
%         a4 = 2; 
%         a5 = 8; 

%             w1_rand(j*T_day+1:T_day*(j+1)) = -3 + 6*rand(1,T_day);
%             w2_rand(j*T_day+1:T_day*(j+1)) = -5 + 10*rand(1,T_day);
%             w3_rand(j*T_day+1:T_day*(j+1)) = -2 + 4*rand(1,T_day);
        w_max = 2; w_min = -2;
        w1_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w2_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w3_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);


            for i = 1:T_day
                w1_pred(i+j*T_day) = a1 + a2*sin(-0.5*pi+(i)/T_day*2*pi);
            end

            for i = 1:T_day
                if i<=18
                elseif i<=36
                    w2_pred(i+j*T_day) = a3*(i-18)/18;
                elseif i<=54
                    w2_pred(i+j*T_day) = a3*(54-i)/18;
                end
            end

            for i = 1:T_day
                if i<=24
                    w3_pred(i+j*T_day) = a4;
                elseif i<=64
                    w3_pred(i+j*T_day) =  a4+ a5;
                else
                    w3_pred(i+j*T_day) = a4;
                end
            end
        end
    w_y = var_y*randn(1,T_day*num_day);
    w_pred = [w1_pred; w2_pred; w3_pred];
    w_real = [w1_pred+w1_rand; w2_pred+w2_rand; w3_pred+w3_rand];
    w_rand = [w1_rand; w2_rand; w3_rand]; 
end
%%
function [w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,n,n_T,T_day,var_y)
w1_pred=w_pred(1,:); w2_pred=w_pred(2,:); w3_pred=w_pred(3,:);
w1_rand=w_rand(1,:); w2_rand=w_rand(2,:); w3_rand=w_rand(3,:);
w_y(n_T*T_day+1:n*T_day) = var_y*randn(1,(n-n_T)*T_day);
    for j = n_T:n-1
        % Disturbance
        a1 = 10 + 4*rand; 
        a2 = 2 + 4*rand; 
        a3 = 0 + 16*rand; 
        a4 = 0 + 2*rand; 
        a5 = 6 + 1*rand; 
%         a1 = 15; 
%         a2 = 4; 
%         a3 = 8; 
%         a4 = 2; 
%         a5 = 8; 

%         w1_rand(j*T_day+1:T_day*(j+1)) = -3 + 6*rand(1,T_day);
%         w2_rand(j*T_day+1:T_day*(j+1)) = -5 + 10*rand(1,T_day);
%         w3_rand(j*T_day+1:T_day*(j+1)) = -2 + 4*rand(1,T_day);
        w_max = 2; w_min = -2;
        w1_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w2_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w3_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);


        for i = 1:T_day
            w1_pred(i+j*T_day) = a1 + a2*sin(-0.5*pi+(i)/T_day*2*pi);
        end

        for i = 1:T_day
            if i<=18
            elseif i<=36
                w2_pred(i+j*T_day) = a3*(i-18)/18;
            elseif i<=54
                w2_pred(i+j*T_day) = a3*(54-i)/18;
            end
        end

        for i = 1:T_day
            if i<=24
                w3_pred(i+j*T_day) = a4;
            elseif i<=64
                w3_pred(i+j*T_day) =  a4+ a5;
            else
                w3_pred(i+j*T_day) = a4;
            end
        end
    end
    w_pred = [w1_pred; w2_pred; w3_pred];
    w_real = [w1_pred+w1_rand; w2_pred+w2_rand; w3_pred+w3_rand];
    
end
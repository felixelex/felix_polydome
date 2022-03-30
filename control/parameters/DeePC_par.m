function parameter = DeePC_par(example)
%MPCCONTROLLER_PAR Set the parameters for the MPC controller
	%
	parameter = controller_par();
	parameter.str_ctr_type = 'DeePCController';
% 	tParameter.logData.pathOutput = './Output';
% 	tParameter.model = load('../Identification/ResultIdent/Model_15min.mat');

    
    parameter.N_pred = 900*10/parameter.Ts; % # of prediction steps
    parameter.N_ini = 900*10/parameter.Ts; % # of prediction steps
    parameter.T = 200; % # of steps for Hankel matrix T >= (nu + 1)*(N_ini + N_pred + nx) - 1
    parameter.T_experiment = 24*11*60*60/parameter.Ts; % # of experiemt steps
    
    parameter.Ts = 900; % for quick test
    
    % Define the weights for g vector: L1 norm or adaptive term
    diag_vector = 0.2:- 0.18/(parameter.T-parameter.N_ini-parameter.N_pred):0.02;
    parameter.Qg = diag(diag_vector);
    parameter.w_s = 50;
    
    % System setup for simulation
    if example == 3
        % Real system dynamics
        A_tem = [0.8011  0.0541  0.0707 ; 
                0.1293  0.8135  0.0055 ;
                0.0989  0.0032  0.7541 ];       
        A = @(t) A_tem + 0.025*sin(2*pi*t/72)*eye(length(A_tem));

        B = [0.07;0.006;0.004];
        E = 1e-3*[22.2170 1.7912 42.2123;
                  1.5376 0.6944 2.9214;
                  103.1813 0.1032 196.0444;];
        % C = [1 0 0;
        %      0 1 0];
        C = [1 0 0];

        nu = size(B, 2);
        nx = size(A(1), 1);
        ny = size(C, 1);
        nw = size(E, 2);

        % Matrix in stage cost
        Q = 0*eye(1);
        R = 0.01;

        % Reference
        r = repmat([20*ones(1,24), 22*ones(1,30), 20*ones(1,18)],1,3);
        
        T_tot = 72*2+60; % total steps for simulation
        T_day = 72; % one-day steps
        num_day = floor(T_tot/T_day)+1 ; % one more day for parameters in prediction
        num_T = 2; % date to start deepc
        var_w_y = 0.0;
        [w_pred, w_real,w_y,w_rand,W] = get_ini_dist(T_day,num_day,num_T, var_w_y);
        [w_pred, w_real,w_y] = get_dist(w_pred, w_rand, w_y,num_day,num_T, T_day, var_w_y);   
        % Constraints
        Hy = [eye(1);-eye(1)];
        for i = 1:T_tot+T_day
             if mod(i,T_day) <=24
                 by = [30; -18];
             elseif  mod(i,T_day)<=54
                 by = [26; -20];
             else
                 by = [30; -18];
             end
             Y{i} = Polyhedron('A', Hy, 'b', by);
        end
        val_u = 30.0; 
        U = Polyhedron([val_u; 0]);
        val_u = 2; 
        dU = Polyhedron([val_u; 0]);
        
        parameter.sys.dU = dU;
        parameter.w_pred = w_pred;
        parameter.w_real = w_real;
        parameter.sys.E = 1e-3*[22.2170 1.7912 42.2123;
          1.5376 0.6944 2.9214;
          103.1813 0.1032 196.0444;];
        parameter.T_experiment = T_tot;
        parameter.N_pred = 6; % # of prediction steps
        parameter.N_ini = 6; % # of prediction steps
        parameter.T = 144; % # of steps for Hankel matrix T >= (nu + 1)*(N_ini + N_pred + nx) - 1        
        parameter.sys.nw = nw;
        parameter.sys.W = W;
        parameter.sys.U = U;
        
        diag_vector = 0.2:- 0.18/(parameter.T-parameter.N_ini-parameter.N_pred):0.02;
        parameter.Qg = diag(diag_vector);
        parameter.w_s = 50;        
        
        parameter.sys.A = A; parameter.sys.B = B; parameter.sys.C = C;
    end
    
    % real system
    if example == 0
        nu = 1; % HP electrical power
        nx = 3;
        ny = 1;
        nw = 2;

        % Matrix in stage cost
        Q = 0*eye(1);
        R = 0.1;

        % Reference: first part for the initial data, could be 0
%         r = [21*ones(1,parameter.N_ini),19*ones(1,46), 21*ones(1,40), 19*ones(1,56), 21*ones(1,40), 19*ones(1,56),21*ones(1,40)];%24*ones(1,T);%
        
        T_tot = parameter.T_experiment;
        % Constraints
        Hy = [eye(1);-eye(1)];
        for i = 1:T_tot
             if i <= parameter.N_ini
                 by = [25; -21];
                 r(i) = 21;
             else
                 if  mod(i - parameter.N_ini,96)<=56 && mod(i - parameter.N_ini,96)>=1
                     by = [25; -21];
                     r(i) = 21;
                 else
                     by = [25; -21];
                     r(i) = 21;
                 end
             end
             Y{i} = Polyhedron('A', Hy, 'b', by);
        end
        val_u = 6.0; 
        U_heat = Polyhedron([val_u; 0]);
        val_u = 4.6; 
        U_cool = Polyhedron([val_u; 0]);       
        val_u = 0.8;  % 6kW/15 
        dU_heat = Polyhedron([val_u; 0]);
        val_u = 0.6;  % 4.6kW/15 
        dU_cool = Polyhedron([val_u; 0]);       
        Hw = [eye(2);-eye(2)];
        bw = [1.0;0.05;1.0;0.05];     
        W = Polyhedron('A', Hw, 'b', bw);
        
        parameter.sys.U_heat = U_heat;
        parameter.sys.U_cool = U_cool;
        parameter.sys.dU_heat = dU_heat;
        parameter.sys.dU_cool = dU_cool; 
        parameter.sys.W = W;
        parameter.sys.nw = nw;  
        
        load('hankel_heat.mat');  load('hankel_cool_0610.mat');
        parameter.Hankel_u_heat=Hankel_u_heat; parameter.Hankel_w_heat=Hankel_w_heat;
        parameter.Hankel_y_heat=Hankel_y_heat;
        parameter.Hankel_u_cool=Hankel_u_cool; parameter.Hankel_w_cool=Hankel_w_cool;
        parameter.Hankel_y_cool=Hankel_y_cool;       
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
%         a1 = 15 ; 
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
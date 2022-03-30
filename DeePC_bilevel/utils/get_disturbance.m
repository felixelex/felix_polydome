function [w_pred, w_real,w_y,W] = get_disturbance(n,T_day,w_min,w_max,var)
    Hw = [eye(3);-eye(3)];
    bw = [2;2;2;2;2;2];
    W = Polyhedron('A', Hw, 'b', bw);
w_y(1:n*T_day) = var*randn(1,n*T_day);
    for j = 0:n-1
        % Disturbance
        a1 = 15 ; 
        a2 = 4; 
        a3 = 8; 
        a4 = 2; 
        a5 = 8; 
%         a1 = 10 + 4*rand; 
%         a2 = 2 + 4*rand; 
%         a3 = 0 + 16*rand; 
%         a4 = 0 + 2*rand; 
%         a5 = 6 + 1*rand; 
        
        w1_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w2_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);
        w3_rand(j*T_day+1:T_day*(j+1)) = w_min + (w_max-w_min)*rand(1,T_day);


        for i = 1:T_day
            w1_pred(i+j*T_day) = a1 + a2*sin(-0.5*pi+(i)/T_day*2*pi);
        end

        for i = 1:T_day
            if i<=18
                w2_pred(i+j*T_day) = -w_min;
            elseif i<=36
                w2_pred(i+j*T_day) = -w_min+a3*(i-18)/18;
            elseif i<=54
                w2_pred(i+j*T_day) = -w_min+a3*(54-i)/18;
            else
                w2_pred(i+j*T_day) = -w_min;
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
%     w_pred = [zeros(1,n*T_day); zeros(1,n*T_day); zeros(1,n*T_day)];
%     w_real = [w1_rand; w2_rand; w3_rand];   
    
end
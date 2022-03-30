function [Hankel_ini, Hankel_pred] = update_Hankel_matrix(obj, u, w, y, T_new)
    % Build Hankel matrices
    % Input: y: ny*(T+1), u: nu*T w: nw*T
    % Output: Hankel_past
    
    % Load parameters
    ny = obj.sys.ny; nu = obj.sys.nu; nw = obj.sys.nw;
    nx = obj.sys.nx; % nx estimated by system ID, used for persistent exciting order check
    N_ini = obj.N_ini; N_pred = obj.N_pred; 
    T = T_new;
    
    % --- Compute Hankel matrix for u and w ---
    Hankel_row = N_ini + N_pred;
    Hankel_col = T - Hankel_row + 1;
    if Hankel_row+Hankel_col-1 ~= size(u,2)
        error("The length of u is wrong!")
    end
    if Hankel_row+Hankel_col-1 ~= size(w,2) 
        error("The length of w is wrong!")
    end    
    
    Hankel_u = zeros(nu*Hankel_row, Hankel_col);
    Hankel_w = zeros(nw*Hankel_row, Hankel_col);
    for i = 1:Hankel_row
        Hankel_u((i-1)*nu+1:i*nu,:) = u(:,i:i+Hankel_col-1);
        Hankel_w((i-1)*nw+1:i*nw,:) = w(:,i:i+Hankel_col-1);
    end
    
    % If the new sequence is not long enough, combine it with the initial
    % Hankel matrix
    if T_new<obj.T 
        Hankel_u = [obj.Hankel_u(:,Hankel_col+1:end), Hankel_u];
        Hankel_w = [obj.Hankel_w(:,Hankel_col+1:end), Hankel_w];
    end
    
    % --- Compute Hankel matrix for y ---
    Hankel_row = N_ini + N_pred + 1; 
    Hankel_col = T - Hankel_row + 2;
    if Hankel_row+Hankel_col-1 ~= size(y,2)
        error("The length of y is wrong!")
    end
    
    Hankel_y = zeros(ny*Hankel_row, Hankel_col);
    for i = 1:Hankel_row
        Hankel_y((i-1)*ny+1:i*ny,:) = y(:,i:i+Hankel_col-1);
    end
    % If the new sequence is not long enough, combine it with the initial
    % Hankel matrix
    if T_new<obj.T     
        Hankel_y = [obj.Hankel_y(:,Hankel_col+1:end), Hankel_y];
    end
    
    
    % --- Split the Hankel matrix for initial and prediction sequence 
    Hankel_ini = [Hankel_u(1:nu*N_ini, :);
                Hankel_w(1:nw*N_ini, :);
                  Hankel_y(1:ny*(N_ini+1), :)];
    Hankel_pred = [Hankel_u(nu*N_ini+1:end, :);
                Hankel_w(nw*N_ini+1:end, :);
                  Hankel_y(ny*(N_ini+1)+1:end, :)];
    obj.Hankel_ini = Hankel_ini;
    obj.Hankel_pred = Hankel_pred;
    
end
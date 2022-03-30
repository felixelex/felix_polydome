function [Hankel_U,Hankel_Y] = update_Hankel_matrix(Hankel_U,Hankel_Y,u,y, nu, ny)
        Hankel_U = Hankel_U(nu+1:end,:);
        Hankel_Y = Hankel_Y(ny+1:end,:);
        num_col = size(Hankel_U,2);
        Hankel_U = [Hankel_U; u(:, end-num_col+1:end)];
        Hankel_Y = [Hankel_Y; y(:, end-num_col+1:end)];

end
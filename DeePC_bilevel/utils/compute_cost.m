function [ Qfun] = compute_cost( y,u,Q ,R, r, T)
% ComputeCost 
% This function takes as inputs:
% - x = [x_0^j, ..., x_{T_j}^j]   : vector of stored 
%                                   states at the jth 
%                                   iteration 
% - u = [u_0^j, ..., u_{T_j-1}^j}]: vector of stored
%                                   inputs at the jth 
%                                   iteration
% - (Q,R): matrices defining the running cost 
%          h(x,u) = x^T Q x + u^T R u


if (size(y,2)~= T)
    warning('The length of state trajectory is not %d', [T]);
end

for i = 1:(size(y,2))
    Index = size(y,2) -i +1; % Need to start from the end
    if size(r,2)>1
        ref = r(:,Index);
    else
        ref = r;
    end
    if i == 1
%                 Cost(Index) = (C*x(:,Index)-ys(:,Index))'*Q*(C*x(:,Index)-ys(:,Index)) + u(:,Index)'*R*u(:,Index);  
        Cost(Index) = (y(:,Index)-ref)'*Q*(y(:,Index)-ref) + u(:,Index)'*R; 
    else
        Cost(Index) = Cost(Index+1) + (y(:,Index)-ref)'*Q*(y(:,Index)-ref) + u(:,Index)'*R; 
%                 Cost(Index)
%                 (C*x(:,Index)-ys(:,Index))'*Q(Index)*(C*x(:,Index)-ys(:,Index))
%                  norm(cp(Index)*u(:,Index),1)
    end
        
Qfun = Cost;
end


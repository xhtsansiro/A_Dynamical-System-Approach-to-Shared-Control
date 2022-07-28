function [x_dot, var] = gp_ds(x, points, att)

% -------------------------------------------------------------------------
% This function outputs the predicted velocity field and its variance from
% Gaussian Process Regression (GPR), it represents the Dynamics of Locally
% Modulate Dynamical Systems (LMDS).

% Inputs: 
%   x: given position, points: data points in GP dataset.
%   att: position of global attractor.

% Outputs:
%   x_dot: the prediction of velocity at given position.
%   var: the predictive variance outputted by GPR.
% -------------------------------------------------------------------------

    A = -0.4 * eye(2); % 2 means the dimension
    Br = 0.08; 
    num = size(points, 1);  % number of rows is number of the samples
    
    sigma_f = 1; l = 0.001;  % hyperparameters for GP
    k_XX = sigma_f * exp( -l^-1 * pdist2(points(:,1:2), points(:,1:2)).^ 2 /2);
    k_XX = k_XX + 0.01*eye(num);   
    k_xx = sigma_f;    % used when calculating variance
    R = chol(k_XX);  % do chelosky decomposition
    x_dot = zeros(size(x,1), size(x,2));  % dimension is the same as x, 
    var = zeros(1, size(x,2));  % variance matrix.
    % use the loop to calculate every points in x.
    for i = 1: size(x,2)
        pos =  x(:,i);  % a column vector of current position

        k_Xx = sigma_f * exp( -l^-1 * pdist2 (pos', points(:,1:2)).^2 /2); 
        alpha = (R\(R'\k_Xx'))';  % (k_XX ^-1 * k_xX)' = k_Xx * k_XX^-1
        alpha_t = truncate(alpha);   % truncate weights
        theta_pred = alpha_t * points(:,3);
        k_pred = alpha_t * points(:,4);
        variance = k_xx - k_Xx * alpha';  % variance of this point 

        % Limit scaling factor
        if abs(k_pred + 1) < 1e-2   % change from 1e-2 to 1e-1 
            k_pred = 0;
        elseif k_pred < -1
            k_pred = 0;
        elseif k_pred > 2
            k_pred = 2;
        else
            k_pred = 1* k_pred;
        end

        % calculate the velocity prediction based on theta and k
        Mr = [cos(theta_pred) -sin(theta_pred); sin(theta_pred) cos(theta_pred)];
        M = (1+k_pred) * Mr;
        
        v_o = linear_ds(pos, A, att);  % the originial linear dynamics
        v_o = velocity_limit(v_o);  % limit the velocity. 
        x_tmp = M * v_o;   % modulated velocity
        x_dot(:,i) = x_tmp; % no limit on velocity.
        x_dot(:,i) = velocity_limit(x_tmp); % velocity should be limited.
        var(:,i) = variance;
    end

end


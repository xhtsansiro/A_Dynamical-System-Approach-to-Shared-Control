function omega_d = pos_check(x, x_cen, x_len, sigmascale)

% ------------------------------------------------------------------------- 
% This function computes the weights of each local linear DS of VSDS before 
% normalization, given a positon x, and outputs the maximum weight.

% Inputs:
%   x: the current position. x_cen: positions of local attractors.
%   x_len: the length between two consecutive local attractors.
%   sigmascale: hyperparameter.
% -------------------------------------------------------------------------

    % omega_d = [];
    K = size(x_cen,2);
%    omega = zeros(1,K);  % a row vector
    sigma = x_len * sigmascale;  % sigma in paper 
   
   % for j = 1:size(x,2)
    omega = zeros(1,K);  % a row vector
    for i = 1:K
        omega(i) = exp(-(1/(2*sigma(i)*sigma(i)))*(x-x_cen(:,i))'*(x-x_cen(:,i)));

    end
    omega_d = max(omega);  % dominant omega
  %  end
    
end

function K_des = get_stiffness(x, var, stiff_type)

% -------------------------------------------------------------------------
% This function provides the setting of the desired stiffness.

% Inputs:
%   x: the position. var: the variance at this position. 
%   stiff_type: the stiffness type, variable or constant.

% Outputs:
%   K_des: the desired stiffness.
% -------------------------------------------------------------------------
    
    k =  1/var;  % scaling factor of stiffness,
    % the smaller the variance is,  the bigger stiffness should be.
    if strcmp(stiff_type, 'variable')
        %K_des = [50*(sin(2*x(2,1))+1)/2, 0; 0, 100]; 
        K_des = [50*k, 0; 0, 100*k]; 
        
    elseif strcmp(stiff_type, 'constant')
        K_des = [25,0;0,100];
    end
end


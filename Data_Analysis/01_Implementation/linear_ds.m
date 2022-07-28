function x_dot = linear_ds(x, A, att)
% ------------------------------------------------------------------------- 
% This function implements a first order Dynamical Systems (DS).
% -------------------------------------------------------------------------

    x_dot =  A * (x-att);
    
end


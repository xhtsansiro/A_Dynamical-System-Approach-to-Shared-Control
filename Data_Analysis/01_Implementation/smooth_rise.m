function omega_threshold = smooth_rise(var, var_low, var_up, value1, value2)

% ------------------------------------------------------------------------- 
% This function implements the smooth-rising between a lower limit and an
% upper limit

% Upper limit: value1 + value2, lower limit: value1 - value2.
% -------------------------------------------------------------------------

    if var >= var_up
        omega_threshold = value1 + value2;
    elseif var <= var_low
        omega_threshold = value1 - value2;
    else
        T = 2 *(var_up - var_low);
        omega_threshold = value1 + value2 * sin(2*pi*(var - var_low)/T - pi*0.5);
    end
end
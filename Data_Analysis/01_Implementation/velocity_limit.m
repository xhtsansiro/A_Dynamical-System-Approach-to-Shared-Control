function [velocity_new] = velocity_limit(velocity)

% ------------------------------------------------------------------------
% This function limits the velocity magnitude. The velocity can't be higher 
% than 0.20 m/s.
% ------------------------------------------------------------------------

    if norm(velocity)  > 0.20  % the max allow speed is 0.20 m/s
        velocity_new = 0.20 * velocity / norm(velocity);
    else
        velocity_new = velocity;
    end

end


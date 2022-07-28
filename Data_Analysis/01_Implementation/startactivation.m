function alpha = startactivation(x,x0,th)

% -------------------------------------------------------------------------
%  This function designs the position dependent scale function of VSDS,
%  which aims to avoid the large acceleration in the beginning of the motion.
% -------------------------------------------------------------------------

    b = 0.1; % 0.01 used before
    if norm(x-x0) > th   % th is shown as d in the paper 
        alpha = 1;
    else
        temp = asin(1-b);
        tempp = (temp)*(norm(x-x0)/th);
        alpha = sin(tempp) + b;
    end
end


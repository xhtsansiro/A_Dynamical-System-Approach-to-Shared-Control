function [B] = findDampingBasis(xd)

% -------------------------------------------------------------------------
% This function finds the damping base given the current velocity.

% Inputs: xd: the velocity
% Outputs: B: damping base, which is a matrix containing two unit vectors, 
%          one is along the velocity direction, the other is perpendicular 
%          to the velocity direction. 
% -------------------------------------------------------------------------
    y1 = 1;
    y2 = -xd(1)/(xd(2)+eps);
    y = [y1;y2];  % y is perpendicular to xd
    B = [xd./norm(xd), y./norm(y)];
end


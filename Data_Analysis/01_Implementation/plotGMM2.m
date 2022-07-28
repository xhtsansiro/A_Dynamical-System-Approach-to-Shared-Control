function [h, X] = plotGMM2(Mu, Sigma, color, valAlpha)

% ------------------------------------------------------------------------- 
% This function displays the parameters of a Gaussian Mixture Model (GMM).

% Inputs 
%   Mu:           D x K array representing the centers of K Gaussians.
%   Sigma:        D x D x K array representing the covariance matrices of K Gaussians.
%   color:        3 x 1 array representing the RGB color to use for the display.
%   valAlpha:     transparency factor (optional).
% -------------------------------------------------------------------------

    nbStates = size(Mu,2);
    nbDrawingSeg = 60;
    darkcolor = color*0.5; %max(color-0.5,0);
    t = linspace(-pi, pi, nbDrawingSeg);
% if nargin<4
%     valAlpha = 1;
% end

    h = [];
    X = zeros(2,nbDrawingSeg,nbStates);
    for i=1:nbStates
        [V,D] = eig(Sigma(:,:,i));  % D is eigenvalue, V is eigenvector,
        R = real(V*D.^.5);
        X(:,:,i) = R * [cos(t); sin(t)] + repmat(Mu(:,i), 1, nbDrawingSeg);
        if nargin>3 %Plot with alpha transparency
            h = [h patch(X(1,:,i), X(2,:,i), color, 'lineWidth', 1, 'EdgeColor', darkcolor, 'facealpha', valAlpha,'edgealpha', valAlpha)];
            h = [h plot(Mu(1,:), Mu(2,:), '.', 'markersize', 6, 'color', darkcolor)];
        else %Plot without transparency
        %Standard plot
            h = [h patch(X(1,:,i), X(2,:,i), color, 'lineWidth', 1, 'EdgeColor', darkcolor)];
            h = [h plot(Mu(1,:), Mu(2,:), '.', 'markersize', 6, 'color', darkcolor)];
%         %Plot only contours
%         h = [h plot(X(1,:,i), X(2,:,i), '-', 'color', color, 'lineWidth', 1)];
        end
    end

end
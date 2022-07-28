function plot_2_row_data(data, leg1, leg2, til, lab1, lab2, dt)

% -------------------------------------------------------------------------
% This fucntion plot the row vector
% Inputs:
%   data: the data to be plotted
%   leg1, leg2 : legends.
%   til: title of the plot.
%   lab1, lab2: labels of axis.
%   dt: time interval.
% -------------------------------------------------------------------------
    tt = dt *(0:size(data,2)-1);
    figure();
    hold on;
    plot(tt, data(1,:), 'r', 'LineWidth',2);
    plot(tt, data(2,:), 'b', 'LineWidth',2);
    legend({leg1, leg2}, 'Interpreter','latex', 'FontSize',20);
    title([til], 'Interpreter','latex','FontSize',20);
    xlabel(lab1, 'Interpreter','latex', 'FontSize',25);
    ylabel(lab2, 'Interpreter','latex', 'FontSize',25);
    set(gca, 'Fontsize', 25);
    box on;
    grid on;
end
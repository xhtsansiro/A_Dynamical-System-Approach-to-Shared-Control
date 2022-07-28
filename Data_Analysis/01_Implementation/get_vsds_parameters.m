function [A_hat, B_, x_rec, f_dot, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, ds, stiff_type)

% -------------------------------------------------------------------------
% This function generates necessary parameters to construct Variable
% Stiffness Dynamical Systems (VSDS).

% Inputs:
%  x0: the starting position. attractor: the position of global attractor.
%  N_points: Number of sampling points along a reference path. 
%  ifplot: plot the sampling process or not.
%  limits: axis limits.
%  ds: original Dynamical Systems (DS), which outputs a reference path.
%  stiff_type: variable stiffness or constant stiffness.

% Outputs:
% A_hat: the stiffness of local attractors of VSDS.
% B_: damping basis.
% x_rec: sampling points along the reference path.
% f_dot: predicted velocity at the sampled points.
% th_begin: threshold of the distance. 

% -------------------------------------------------------------------------

    dt = 0.002; threshold = 0.00001;
    x_rec = x0;  % sampled points for VSDS
    x_tmp = x0;  % record the trajectory
    f_dot = []; f_pre = [0;0]; f_dot_tmp = [];
    
    A_hat = []; b_hat = []; B_ = [];
    
    xnow = x0;  % col vector
    while (norm(xnow-attractor) > threshold)
        xd = ds(xnow);  % x_d from gp_ds
        xd = xd(:,1);
        x_tmp(:,end+1) = xnow + dt * xd; 
        
        xnow = xnow + dt * xd;
        f_dot_tmp(:,end+1) = (xd - f_pre)/dt;
        f_pre = xd;
    end
    x_tmp(:,end+1) = attractor;
        
    x_len_tmp = vecnorm(x_tmp(:,1:end-1) - x_tmp(:,2:end)); % the total length of the trajectory
    th_begin = 0.1 * sum(x_len_tmp);
    
    last_ratio = 1;
    len_sub = sum(x_len_tmp)/(N_points+last_ratio-1);
    len =  zeros(1, N_points);  
    for i = 1:N_points-1
        len(i) = i * len_sub;
    end
    len(end) = sum(x_len_tmp); % len has the length of the total trajectory, incrementally increasing
    
    N = size(x_tmp, 2); 
    distance = 0; i = 1; j = 1;
    while i <= N-1
        if distance < len(j)
            distance = distance + x_len_tmp(i);
            i = i + 1;
        else
            x_rec(:, end+1) = x_tmp(:,i);
            f_dot(:, end+1) = f_dot_tmp(:,i);
            distance = distance + x_len_tmp(i);
            i = i + 1;
            j = j + 1;
        end
    end
    
    x_rec(:, end+1) = attractor;
    length_seq = size(x_rec, 2);
    
    f_dot(:,end+1) = f_dot_tmp(:,end);
    
    % plot the sampled points in the streamline figure
    if ifplot == 1
        h_act = figure(); hold on;
        length_seq = size(x_rec, 2); 
        plot(x_rec(1,1),x_rec(2,1),'y.','markersize',15);
        plot(x_rec(1,2:length_seq),x_rec(2,2:length_seq),'r.','markersize',15);
        set(gca,'fontsize',20,'LineWidth',1);
        box on; % $x_y [m]$
        xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',25);
        ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',25);
        scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',2); hold on;
        axis(limits)  % the limits [x_low, x_high, y_low, y_high]
        % for plotting streamline
        plot_ds( ds, limits,'medium'); 
        legend({'Starting Point', 'Sequence Points','Goal Point'},'Interpreter','LaTex','FontSize',20)
%         title(['Streamlines of GP-based LMDS'], 'Interpreter','LaTex','FontSize',20)
        hold on;

    end
    
    for i = 1:size(x_rec,2)-1
        % x_g: velocity; v_g: variance
        [x_g, v_g] = ds(x_rec(:,i));  % because at final point, velocity is zero.
        B = findDampingBasis(x_g);  % along v and perpendicular v
        B_(:,end+1:end+2) = B;
        A_hat(:,end+1:end+2) = -B*get_stiffness(x_rec(:,i), v_g, stiff_type)*B';
    end
    disp(['Sequence of ',num2str(length_seq-1),' points is generated'])
   
end


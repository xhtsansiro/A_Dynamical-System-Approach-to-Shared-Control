function realtimePlot(real_data, desire_data, limits, plot_title, Vid_title, cas)

% -------------------------------------------------------------------------
% This function generates video for showing the evolvement of the robot
% motion.

% Inputs: 
%   real_data: the real robot motion.
%   desire_data: the reference motion.
%   limits: the axis limits.
%   plot_title: the title of the plot.
%   Vid_title: the name of the video.
%   cas: which case to generate video.
% -------------------------------------------------------------------------

    att = [0.4, 0.1];
    figure();
    box on;
    grid on;
    hold on;

    xl = xlabel('$x_y [m]$');
    yl = ylabel('$x_z [m]$');
    set([xl, yl],'interpreter','latex','fontsize',20);
    set(gca,'fontsize',16);
    tl = title(plot_title);
    set(tl,'fontsize', 16, 'interpreter','latex');
    xlim([limits(1) limits(2)])  ;
    ylim([limits(3) limits(4)]) ;

    video = VideoWriter(strcat(Vid_title,'.avi')); %'MPEG-4'
    video.FrameRate=50 ;
    open(video);

    incr=15 ;  % for move_back video, set it to 1, otherwise to 15
    index_end=length(real_data) ;

    % index_end=5000 ; % Only For force disturbance
    %% begin to plot
    % plot streamlines
    if cas == 1 
        % border points
        border_1 = load("org_traj/border_1.txt");
        border_2 = load("org_traj/border_2.txt");
        plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        points = load('sampled_points.mat').points_new; % GP dataset
        limits = [-0.2, 0.7, -0.2, 0.7];
        attractor = [0.4; 0.1];
        N_points = 20;    
        ifplot = 0; % plot out the figure
        stiff_type = 'constant'; 
        x0 = [-0.0468; 0.1508];  % starting from original location
        gp_lmds =  @(x) gp_ds (x, points, attractor);
        % intialize VSDS based on GP-LMDS, starting from point x0.
        [A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
        sigma_scale = 1;
        my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

        x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
        x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
        w_th = 0.3;
        par = {x_cen, x_len, w_th, sigma_scale};
        plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;

    elseif cas == 2  % failcase 
        border_1 = load("fail_traj_1/border_1.txt");
        border_2 = load("fail_traj_1/border_2.txt");
        plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on

        points = load('sampled_points.mat').points_new; % GP dataset
        limits = [-0.2, 0.7, -0.2, 0.7];
        attractor = [0.4; 0.1];

        N_points = 8;
        ifplot = 0; % plot out the figure
        stiff_type = 'constant'; 
        x0 = [0.670397; 0.207491];  % starting from original location

        gp_lmds =  @(x) gp_ds (x, points, attractor);
        % intialize VSDS based on GP-LMDS, starting from point x0.
        [A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
        sigma_scale = 1;
        my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

        x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
        x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
        w_th = 0.7;
        par = {x_cen, x_len, w_th, sigma_scale};
        plot_ds_combined( gp_lmds, my_vsds, limits, par, 'medium'); hold on;

    elseif cas == 3
        % border points
        border_1 = load("suc_traj_1/border_1.txt");
        border_2 = load("suc_traj_1/border_2.txt");
        plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        new_pos = load('new_points_case1/pos_train.txt');
        new_vel = load('new_points_case1/vel_train.txt');
        input = []; 
        for i = 1:length(new_pos)   % generate 200 points along the trajectory.
            % calculate original velocity, in the form of column vector
            v_origin = [-0.4 * (new_pos(i,1)-0.4); -0.4 * (new_pos(i,2)-0.1)];
             % linear dynamics is : x' = -0.4x;
            v = [new_vel(i,1); new_vel(i,2)];
            theta = acos(dot (v_origin, v) / (norm(v_origin) * norm(v))); % in radian
            % check it is clockwimse or anti-clockwise rotation (from v_org to v_real) 
            if (v_origin(1) * v(2) - v_origin(2) * v(1) > 0)
                sgn = 1;
            else
                sgn = -1;
            end
            k = norm(v)/norm(v_origin) -1;

            p = [new_pos(i,1), new_pos(i,2), sgn*theta, k];
            input(end+1,:) =  p;
        end

        att = [0.4; 0.1];
      
        %scatter(new_pos(:,1), new_pos(:,2), 'm'); hold on
        ds.limits = [-0.2, 0.7, -0.2, 0.7];
        myds =  @(x) gp_ds (x, input, att);

        % plot streamlines
        limits = [-0.2, 0.7, -0.2, 0.7];
        N_points = 14;
        ifplot = 0; % plot out the figure
        stiff_type = 'constant'; 
        x0 = [0.6657 ; 0.223655];  % starting from original location

        gp_lmds = myds; % myds is generated in the previous section
        % intialize VSDS based on GP-LMDS, starting from point x0.
        [A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, att, N_points, ifplot, limits, gp_lmds, stiff_type);
        sigma_scale = 1;
        my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

        x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
        x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
        w_th = 0.3;
        par = {x_cen, x_len, w_th, sigma_scale};
        plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;

    elseif cas ==4
        border_1 = load("fail_traj_2/border_1.txt");
        border_2 = load("fail_traj_2/border_2.txt");
        plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on 
        new_pos = load('new_points_case1/pos_train.txt');
        new_vel = load('new_points_case1/vel_train.txt');
        input = []; 
        for i = 1:length(new_pos)   % generate 200 points along the trajectory.
            % calculate original velocity, in the form of column vector
            v_origin = [-0.4 * (new_pos(i,1)-0.4); -0.4 * (new_pos(i,2)-0.1)];
             % linear dynamics is : x' = -0.4x;
            v = [new_vel(i,1); new_vel(i,2)];
            theta = acos(dot (v_origin, v) / (norm(v_origin) * norm(v))); % in radian
            % check it is clockwise or anti-clockwise rotation (from v_org to v_real) 
            if (v_origin(1) * v(2) - v_origin(2) * v(1) > 0)
                sgn = 1;
            else
                sgn = -1;
            end
            k = norm(v)/norm(v_origin) -1;

            p = [new_pos(i,1), new_pos(i,2), sgn*theta, k];
            input(end+1,:) =  p;
        end

        att = [0.4; 0.1];
        ds.limits = [-0.2, 0.7, -0.2, 0.7];
        myds =  @(x) gp_ds (x, input, att);

        % plot streamlines
        limits = [-0.2, 0.7, -0.2, 0.7];
        N_points = 20;
        ifplot = 0; % plot out the figure
        stiff_type = 'constant'; 
        x0 = [-0.0468; 0.1508]; % starting from original location

        gp_lmds = myds; % myds is generated in the previous section
        % intialize VSDS based on GP-LMDS, starting from point x0.
        [A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, att, N_points, ifplot, limits, gp_lmds, stiff_type);
        sigma_scale = 1;
        my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

        x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
        x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
        w_th = 0.3;
        par = {x_cen, x_len, w_th, sigma_scale};
        plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;
    
    elseif cas ==5
        new_pos = load('new_points_case2/pos_train.txt');
        new_vel = load('new_points_case2/vel_train.txt');
        input = []; 
        for i = 1:length(new_pos)   % generate 200 points along the trajectory.
            % calculate original velocity, in the form of column vector
            v_origin = [-0.4 * (new_pos(i,1)-0.4); -0.4 * (new_pos(i,2)-0.1)];
            % linear dynamics is : x' = -0.4x;
             v = [new_vel(i,1); new_vel(i,2)];
            theta = acos(dot (v_origin, v) / (norm(v_origin) * norm(v))); % in radian
            % check it is clockwise or anti-clockwise rotation (from v_org to v_real) 
            if (v_origin(1) * v(2) - v_origin(2) * v(1) > 0)
                sgn = 1;
            else
                sgn = -1;
            end
            k = norm(v)/norm(v_origin) -1;

            p = [new_pos(i,1), new_pos(i,2), sgn*theta, k];
            input(end+1,:) =  p;
        end

        att = [0.4; 0.1];

        ds.limits = [-0.2, 0.7, -0.2, 0.7];
        myds =  @(x) gp_ds (x, input, att);

        border_1 = load("suc_traj_2/border_1.txt");
                   
        border_2 = load("suc_traj_2/border_2.txt");

        plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on
        plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',3.5, 'LineStyle', ':');
        hold on 

        x0 = [-0.0468; 0.1508];
        N_points = 25;
        ifplot = 0; % plot out the figure
        stiff_type = 'constant'; 
        gp_lmds = myds; % myds is generated in the previous section
        % intialize VSDS based on GP-LMDS, starting from point x0.
        [A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, att, N_points, ifplot, limits, gp_lmds, stiff_type);
        sigma_scale = 1;
        my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

        x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
        x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
        w_th = 0.3;
        par = {x_cen, x_len, w_th, sigma_scale};
        plot_ds_combined( gp_lmds, my_vsds, limits, par, 'medium'); hold on;
    elseif cas ==6
        plot(desire_data(:,1), desire_data(:,2), 'm','LineWidth',2) ;
    else
        incr = 1;
    end
    
    if (cas ~= 0 && cas ~= 6)
        % incr = 1;
        plot(desire_data(:,1), desire_data(:,2), '--r','LineWidth',2) ;

    end

    hold on;
    if (strcmp(plot_title,'Trajectory of Robot')| strcmp(plot_title,'New Demonstrations'))
        scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
        % plot the wall
        hold on
        rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
        x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
        rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
        x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
        rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
        x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
        hold on
        if (strcmp(Vid_title,'video/robot_traj_4') || strcmp(Vid_title,'video/robot_traj_5') )
            rectangle('Position', [0.19,0,0.02,0.4345],'LineWidth', 2, 'LineStyle','-'); 
            x = [0.19  0.21 0.21 0.19]; y=[0 0 0.4345 0.4345]; fill(x,y,[0.6350 0.0780 0.1840]);
        end
    end
  
    for i=1:incr:index_end
        plot(real_data(1:i,1),real_data(1:i,2),'b','LineWidth',2) ;
        hold on  
        drawnow
       
        frame = getframe(gcf);
        writeVideo(video,frame);
    
    end
    plot(real_data(:,1),real_data(:,2),'b','LineWidth',2) ;
    frame = getframe(gcf);
    writeVideo(video,frame);


    close(video);
    disp('Done') ;
end

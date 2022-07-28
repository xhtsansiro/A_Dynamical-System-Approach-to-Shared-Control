% -------------------------------------------------------------------------
% This script visualizes the process of VSDS generation. Plot streamlines
% of the original dynamics LMDS, the streamlines of VSDS. 

% In addition, it visulizes the system behavior (trajectory, velocity,
% accerleration of a simple second order DS, when controlled by VSDS in
% closed loop.
% -------------------------------------------------------------------------

%% initailize variables, and LMDS 
points = load('sampled_points.mat').points_new;  % points used for prediction. maybe input argument?
% x0 = [0; 0.5];
x0 = [-0.0468; 0.1508];  % starting from original location
attractor = [0.4; 0.1];
N_points = 20;
ifplot = 1; % plot out the figure
limits = [-0.2, 0.7, -0.2, 0.7]; % (-0.2,0.6) both in x and y direction.
sigma_scale = 1; 
stiff_type = 'constant';  % constant or variable
plotmyds = 1; 
global_ds =  @(x) gp_ds (x, points, attractor); % dynamics of LMDS

%% Implement VSDS based on LMDS and plot VSDS
[A_hat, B_, x_rec, f_dot, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, global_ds, stiff_type);
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

if plotmyds == 1
    disp('Plotting new DS...')
    figure();
    [hatt_rob] = scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',4); hold on;
    plot(x_rec(1,:), x_rec(2,:), 'b', 'LineWidth', 4);
    plot(x_rec(1,2:end-1),x_rec(2,2:end-1),'r.','markersize',20); % 40 for paper plot
    [hds_rob1] = plot_ds(my_vsds, limits,'medium'); hold on;
    set(gca,'fontsize',25,'LineWidth',1);

    xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',35);
    ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',35);

    legend({'$global \, attractor$', '$reference$', '$local \,attractor$'}, 'Interpreter','LaTex', 'FontSize',20);

    box on;
    axis(limits);

    disp('Plot done')

end

%% plot original streamlines of LMDS
figure();
scatter(points(:,1), points(:,2), 'b');
hold on; 
scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',2); 
hold on;  limits = [-0.2, 0.7, -0.2, 0.7];
plot_ds(global_ds, limits,'medium');
set(gca,'fontsize',25,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',35);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',35);
box on;
legend({'$selected\, points$', '$global \, attractor$'}, 'Interpreter','latex', 'FontSize',20);

axis(limits);
% legend({'$Target$'}, 'Interpreter','latex', 'FontSize',15);

%% valid the system behavior of second order DS by using the closed-loop VSDS control  
%  Controller input u_c = f_vsds -D(x)*x'
%  System dynamics: [z_dot; z_ddot] = [0 1;0 0] *[z, z_dot] + [0; u_c]

D = [5,0;0,10];
z = x0; z_dot = [0;0]; z_ddot = [];dt = 0.002;
t0 = 0; t_tot = 15; 
while t0 < t_tot
     vsds_part = my_vsds(z(:,end));
     basis = findDampingBasis(vsds_part); 
     z_ddot(:,end+1) = vsds_part - basis*D*basis' *z_dot(:,end); 
%     z_ddot(:,end+1) = my_vsds(z(:,end)) - D*[0.0029;0.0041]; 
    
    % Taylor series for calculating velocity and position
    z_dot(:,end+1) = z_dot(:,end) + z_ddot(:,end) * dt;  % velocity field
    z(:,end+1) = z(:,end) + z_dot(:,end)*dt + 1/2 * z_ddot(:,end)*dt*dt;

    t0 = t0 + dt;
end

% plot position field  
plot_2_row_data(z, '$z_1$', '$z_2$', 'Position Profile', '$t$', '$z$', dt);
% plot velocity filed
plot_2_row_data(z_dot, '$\dot{z}_1$', '$\dot{z}_2$', 'Velocity Profile', '$t$', '$v$', dt);
% plot accerleration field 
plot_2_row_data(z_ddot, '$\ddot{z}_1$', '$\ddot{z}_2$', 'Acceleration Profile', '$t$', '$a$', dt);
% plot the trajectory
figure();
plot(z(1,:), z(2,:), 'r', 'Linewidth', 2);
xlabel('$z_1$', 'Interpreter','latex','FontSize',25);
ylabel('$z_2$', 'Interpreter','latex','FontSize',25);
title(['Trajectory'], 'Interpreter','latex', 'FontSize',25)
grid on
hold on
plot(x_rec(1,1:end), x_rec(2,1:end), 'b' , 'LineWidth',2);
set(gca,'fontsize',25,'LineWidth',1);

legend({'$z_{simulation}$', '$z_{desired}$'}, 'Interpreter','latex', 'FontSize',20, 'Location', 'Northwest');

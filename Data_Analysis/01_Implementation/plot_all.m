% -------------------------------------------------------------------------
% This script plots all the figures for visualization.

% -------------------------------------------------------------------------

%% Plot the reference traj. from three different position, 
% background is the streamline of GP-LMDS

% plot the trajectory of the robot, 
r1 = load("pos_1/robot_real.txt");
r2 = load("pos_2/robot_real.txt");
r3 = load("pos_3/robot_real.txt");
figure(1)
plot(r1(:,2), r1(:,3),'g', 'LineWidth', 2); 
hold on
plot(r2(1149:end,2), r2(1149:end,3),'b', 'LineWidth', 2);
hold on
plot(r3(200:end,2), r3(200:end,3),'r', 'LineWidth', 2);

points_new = load('sampled_points.mat').points_new; % velocity 0.4
att = [0.4; 0.1]; 
% points_new = []; idx = 0;
% for i = 1: size(points,1)
%     idx = idx + 1;
%     if norm(points(i,1:2)'-att) > 0.15    % 
%         points_new(idx,:) = points(i,:);
%     end
% end

att = [0.4; 0.1];
h_act = figure(1);
ds.limits = [-0.2, 0.7, -0.2, 0.7];
% ds.limits = [-0.3, 1.2, -0.4, 1.1]; % from -1 to 1 in both two dimensions
% att = [0; 0];  % the global attractor is at (0,0).
A  = - 0.4 * eye(2);
myds =  @(x) gp_ds (x, points_new, att);
[hatt_rob] = scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% [0 0 0] is the color RGB setting. 'd' represents the type of the marker.
[hds_rob1] = plot_ds(myds, ds.limits,'medium'); hold on;
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',20);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',20);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);
set(gca,'fontsize',16,'LineWidth',1);
box on;
axis(ds.limits);
legend({'$Trajectory_1$', '$Trajectory_2$',  '$Trajectory_3$',  '$Target$' }, 'Interpreter','Latex', 'FontSize',12);

%% Plot the stiffness along the three paths

% plot the ellipses reflecting stiffness
% execute this section repetitively, change the name of the path pos_3 to 
% pos_2, pos_1.

pos = load('pos_3/pos_field.txt'); data.pos = pos(1:end-1,:)';
vel = load('pos_3/vel_field.txt'); v_vel = [];
% start from demonstration points
for i = 1:2:size(vel,1)
    v_vel(end+1,:) = vel(i,:);
end
data.vel = v_vel' ;
stiff = load('pos_3/stiffness.txt'); stiffness = [];
for i = 1:2:size(stiff,1)
    stiffness(:,:,end+1) = stiff(i:i+1,1:2);
end
data.stiffness = stiffness(:,:,2:end);

figure(2);
scatter(0.4, 0.1, 150, [0 0 0],'d','Linewidth',2);  % target is (0.4, 0.1)
hold on
% hold on
% for i= 1:8   % this is for pos_1
for i= 1:length(data.pos) -1  % this is for pos_2 and pos_3
    plotGMM2([data.pos(1,i); data.pos(2,i)], -data.stiffness(1:2,1:2,i) * 0.000001,[0.4940 0.1840 0.5560], .8);
%     plotGMM2([s_aug.x_att(1,i);s_aug.x_att(2,i)], s_aug.K_regress(1:2,1:2,i) * 0.002,[ 0.8500    0.3250    0.0980], .8);

    hold on
%    quiver(s_aug(k).x_att(1,i),s_aug(k).x_att(2,i),-s_aug(k).DataF(1,i)*0.0035,-s_aug(k).DataF(2,i)*0.0035,0,'r','LineWidth',3)
    quiver(data.pos(1,i), data.pos(2,i), data.vel(1,i)*0.0035, data.vel(2,i)*0.0035, 0, 'r', 'LineWidth', 3);
    hold on
%     plot(s_aug(k).x_att(1,i),s_aug(k).x_att(2,i),'*k','LineWidth',3)
    plot(data.pos(1,i), data.pos(2,i),'*k', 'LineWidth',4);
    hold on
end

hold on
plotGMM2([data.pos(1,1); data.pos(2,1)], -data.stiffness(1:2,1:2,1) * 0.000001,[0.4940 0.1840 0.5560], .8);
hold on
grid on
plot(pos(:,1), pos(:,2), 'r', 'Linewidth', 2);  % g: pos1, b: pos2, r: pos3;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',35);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',35);
% xlim([0.3, 0.7]);
box on;
ylim ([0.05, 0.5]);
legend({'$target$', '$stiffness$' }, 'Interpreter','LaTex', 'FontSize',20);
% legend({'$Demonstration$', '$Target Position$', '$Stiffness$' }, 'Interpreter','latex', 'FontSize',10);

%% Plot the escape force which tries to get rid of the vsds tunnel

force_1 = load('escape_force_1.txt');   % starting from intial position the same as demonstration
force_2 = load('escape_force_2.txt');    % starting from another intial position

% plot force separately
figure();
tt_1 = (0:length(force_1)-1) * 0.002;
plot(tt_1, force_1(:,1), 'b',  tt_1, force_1(:,2), 'r');
xlabel('$time:s$', 'Interpreter','latex','FontSize',20);
ylabel('$Force:N$', 'Interpreter','latex','FontSize',20);
grid on;
box on
title(['Force acting on haptic device'], 'Interpreter','latex', 'FontSize',20);
legend({'$F_{y}$', '$F_{z}$'}, 'Interpreter','latex', 'FontSize',20);

figure();
tt_2 = (0:length(force_2)-1) * 0.002;
plot(tt_2, force_2(:,1), 'b',  tt_2, force_2(:,2), 'r');
xlabel('$time:s$', 'Interpreter','latex','FontSize',20);
ylabel('$Force:N$', 'Interpreter','latex','FontSize',20);
grid on;
box on
title(['Force acting on haptic device'], 'Interpreter','latex', 'FontSize',20);
legend({'$F_{y}$', '$F_{z}$'}, 'Interpreter','latex', 'FontSize',20);

%
% filter the trajectory
fs=500 ; dt=1/fs ;
[B,A]=butter(1,3/(fs/2)); 
force_filtered_1 = filtfilt(B,A,force_1);
force_filtered_2 = filtfilt(B,A,force_2);

comb_force_f_1 = vecnorm(force_filtered_1,2,2);
comb_force_f_2 = vecnorm(force_filtered_2,2,2);

% calculate the force combination from both axes

figure();
scatter(tt_1(1,3833),comb_force_f_1(3833,1), 170, [0 0 0],'x','Linewidth',4); hold on;
plot(tt_2, comb_force_f_2, 'r', 'LineWidth', 4); 
hold on;
plot(tt_1, comb_force_f_1, 'b', 'LineWidth', 4);   %,  tt, force_filtered(:,2), 'r');
hold on;
scatter(tt_2(1,2638),comb_force_f_2(2638,1), 170, [0 0 0],'x','Linewidth',4); hold on;


set(gca,'fontsize',25,'LineWidth',1);
x1 = xlabel('$Time [s]$');
% xlabel('$time [s]$', 'Interpreter','latex','FontSize',20);
y1 = ylabel('$||Force|| [N]$');
set([x1 y1],'interpreter','Latex','fontsize',30);
% ylabel('$Force:N$', 'Interpreter','latex','FontSize',20);
grid on;
box on
% title(['Escaping Force'], 'Interpreter','latex', 'FontSize',20);
scatter(tt_1(1,3833),comb_force_f_1(3833,1), 170, [0 0 0],'x','Linewidth',4); hold on;
scatter(tt_2(1,2638),comb_force_f_2(2638,1), 170, [0 0 0],'x','Linewidth',4); hold on;
legend({'$escaping\,point$', '$case\,1$', '$case\,2$'}, 'Interpreter','LaTex', 'FontSize',25, 'Location', 'Northwest');
% legend({'$F_{y}$', '$F_{z}$'}, 'Interpreter','latex', 'FontSize',20);


%% plot the effect region of the VSDS background streamlines, 
% Case 1: far away from demonstrations.

points = load('sampled_points.mat').points_new;
limits = [-0.2, 0.7, -0.2, 0.7];
attractor = [0.4; 0.1];
N_points = 9;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 
x0 = [0.6; 0.4];
gp_lmds =  @(x) gp_ds (x, points, attractor);
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

h_act = figure();
hold on;
quality='medium';
if strcmpi(quality,'high')
    nx=400;
    ny=400;
elseif strcmpi(quality,'medium')
    nx=200;
    ny=200;
else
    nx=50;
    ny=50;
end

axlim = limits;
ax_x=linspace(axlim(1),axlim(2),nx); % computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); % computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y);  % meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
x_ = x;
% x_ = x-repmat(attractor,1,size(x,2));
for i = 1:size(x_,2)
    xd(:,i) = feval(my_vsds, x_(:,i));
end

% calculate the weights
omega_max = zeros(1,nx*ny);
x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length

for i = 1:size(x,2)
    omega_max(i) = pos_check(x_(:,i), x_cen, x_len, sigma_scale);
end

% for omega_threshold smaller than 0.4, have to use gp-lmds to recalculate
omega_threshold = 0.8;  % big value is 0.7
% omega_threshold = 0.1;
for i = 1:size(x_,2)
    if omega_max(i) < omega_threshold
        xd(:,i) = feval(gp_lmds, x_(:,i));
    end
end
z_tmp = reshape(omega_max,nx,ny);
hcolor = pcolor(x_tmp,y_tmp,reshape(omega_max,nx,ny));
set(hcolor,'linestyle','none');
% load whiteCopperColorMap;

colormap(cubehelix([],1.53,-1.32,2.74,0.81,[0.37,1],[0.35,0.78]));  % w_th = 0.1
colorbar;
caxis([min(min(z_tmp)), max(max(z_tmp))]);
h = streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),3,'method','cubic');
set(h,'LineWidth', 1)
set(h,'color',[0.0667  0.0667 0.0667]);

xl = xlabel('$x_y [m]$');
yl = ylabel('$x_z [m]$ ');
set(gca,'fontsize',20,'LineWidth',1);
set([xl yl],'interpreter','Latex','fontsize',35);
xlim([axlim(1),axlim(2)])
ylim([axlim(3),axlim(4)])
box on;
% set(gcf, 'Renderer', 'opengl')joint_external_force_callback
axis(limits);
scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',2); 
plot(x_rec(1,:), x_rec(2,:), 'm', 'Linewidth', 4);

%% plot the effect region of the VSDS background streamlines, 
% Case 2: close to demonstrations.

points = load('sampled_points.mat').points_new;
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

%target = [0.4; 0.1];
h_act = figure();
hold on;
quality='medium';
if strcmpi(quality,'high')
    nx=400;
    ny=400;
elseif strcmpi(quality,'medium')
    nx=200;
    ny=200;
else
    nx=50;
    ny=50;
end

axlim = limits;
ax_x=linspace(axlim(1),axlim(2),nx); % computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); % computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y);  % meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
x_ = x;
% x_ = x-repmat(attractor,1,size(x,2));
for i = 1:size(x_,2)
    xd(:,i) = feval(my_vsds, x_(:,i));
end

% calculate the weights
omega_max = zeros(1,nx*ny);
x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length

for i = 1:size(x,2)
    omega_max(i) = pos_check(x_(:,i), x_cen, x_len, sigma_scale);
%     if omega_max(i) > 2
%         omega_max(i) = 0;
%     end
end

% for omega_threshold smaller than 0.1, have to use gp-lmds to recalculate
omega_threshold = 0.1;
for i = 1:size(x_,2)
    if omega_max(i) < omega_threshold
        xd(:,i) = feval(gp_lmds, x_(:,i));
    end
end
z_tmp = reshape(omega_max,nx,ny);
hcolor = pcolor(x_tmp,y_tmp,reshape(omega_max,nx,ny));
set(hcolor,'linestyle','none');
% load whiteCopperColorMap;

colormap(cubehelix([],1.53,-1.32,2.74,0.81,[0.37,1],[0.35,0.78])); 

% cubehelix_view(h_act)

colorbar;
caxis([min(min(z_tmp)), max(max(z_tmp))]);
h = streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),3,'method','cubic');
set(h,'LineWidth', 1)
set(h,'color',[0.0667  0.0667 0.0667]);

xl = xlabel('$x_y [m]$');
yl = ylabel('$x_z [m]$ ');
set(gca,'fontsize',20,'LineWidth',1);
set([xl yl],'interpreter','Latex','fontsize',35);
xlim([axlim(1),axlim(2)])
ylim([axlim(3),axlim(4)])
box on;
axis(limits);
scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',2); 
plot(x_rec(1,:), x_rec(2,:), 'm', 'Linewidth', 4);


%% Plot the success path in normal execution, Case 1

figure();
real_traj = load('org_success_traj_1/robot_real.txt');
hold on; plot(real_traj(:,2), real_traj(:,3), 'b', 'LineWidth', 4); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('org_success_traj_1/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
att = [0.4; 0.1]; 
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% border points
border_1 = load('org_success_traj_1/border_1.txt');
border_2 = load('org_success_traj_1/border_2.txt');
% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on 
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
% plot the border line of VSDS, generated based on the VSDS sampled
% attractors, calculate the velocity direction, then finds out its
% perpendicular direction vector k, then x +/- delta_l *k should be the
hold on
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
% plot ds streamlines in background 
% intialize GP-LMDS
points = load('sampled_points.mat').points_new; % GP dataset
limits = [-0.2, 0.7, -0.2, 0.7];
attractor = [0.4; 0.1];
N_points = 20; 
% N_points = 9;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 
x0 = [-0.0468; 0.1508];  % starting from original location
% x0 = [0.6; 0.4];
gp_lmds =  @(x) gp_ds (x, points, attractor);
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.3;
par = {x_cen, x_len, w_th, sigma_scale};
[~,~] = plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);
% set(gca,'fontsize',16,'LineWidth',1);
box on;
axis(limits);

legend({'$robot\,motion$', '$reference$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',18);

%% Plot the success path in normal execution, Case 2

figure();
real_traj = load('org_success_traj_2/robot_real.txt');
hold on; plot(real_traj(:,2), real_traj(:,3), 'b', 'LineWidth', 4); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('org_success_traj_2/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
att = [0.4; 0.1]; 
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% border points
border_1 = load('org_success_traj_2/border_1.txt');
border_2 = load('org_success_traj_2/border_2.txt');
% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth', 4, 'LineStyle', ':');

% plot ds streamlines in background 
% intialize GP-LMDS
points = load('sampled_points.mat').points_new; % GP dataset
limits = [-0.2, 0.7, -0.2, 0.7];
attractor = [0.4; 0.1];
%N_points = 20; 
N_points = 9;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 
x0 = [0.481526; 0.435139];  % starting from original location
% x0 = [0.6; 0.4];
gp_lmds =  @(x) gp_ds (x, points, attractor);
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.7;
par = {x_cen, x_len, w_th, sigma_scale};
plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
axis(limits);

legend({'$robot\,motion$', '$reference$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',18, 'Location','northwest'); % org font size is 20


%% Incremental learning
% Plot the failed path in Case 1: far away from demonstrations

figure();
attractor = [0.4; 0.1];
fail_traj = load('failed_traj_case1/robot_real.txt');
hold on; plot(fail_traj(1:4680,2), fail_traj(1:4680,3), 'b', 'LineWidth', 4); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('failed_traj_case1/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');

hold on; scatter(fail_traj(4680,2),fail_traj(4680,3), 150, [0 0 1],'x','Linewidth',2); 
scatter(attractor(1),attractor(2), 150, [0 0 0],'d','Linewidth',2); hold on;
border_1 = load('failed_traj_case1/border_1.txt');
border_2 = load('failed_traj_case1/border_2.txt');

% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');

points = load('sampled_points.mat').points_new; % GP dataset
limits = [-0.2, 0.7, -0.2, 0.7];
attractor = [0.4; 0.1];
%N_points = 20; 
N_points = 8;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 
x0 = [0.670397; 0.207491];  % starting from original location
% x0 = [0.6; 0.4];
gp_lmds =  @(x) gp_ds (x, points, attractor);
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.7;
par = {x_cen, x_len, w_th, sigma_scale};
plot_ds_combined(gp_lmds, my_vsds, limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
axis(limits);

hold on; scatter(fail_traj(4680,2),fail_traj(4680,3), 150, [0 0 1],'x','Linewidth',4); 

legend({'$robot\, motion$', '$reference$', '$stop\, point$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',18); % org font size 20

%% Plot streamlines after incremental learning
% case 1, far away from demonstration.
% escape from VSDS, demonstrate new path, update DS.

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
h_act = figure(); 
selected = [];
for i = 121:201
    if mod(i,2) == 0
        selected(end+1,:) = new_pos(i,1:2);
    end
end
selected = [selected; new_pos(1:120,1:2)];
scatter(selected(:,1), selected(:,2), 'b'); hold on;

ds.limits = [-0.2, 0.7, -0.2, 0.7];
% ds.limits = [-0.3, 1.2, -0.4, 1.1]; % from -1 to 1 in both two dimensions
% att = [0; 0];  % the global attractor is at (0,0).
A  = -0.4 * eye(2);
% myds = @(x) linear_dynamics(x, A_hat, att);  % linear dynamics
myds =  @(x) gp_ds (x, input, att);
[hatt_rob] = scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% [0 0 0] is the color RGB setting. 'd' represents the type of the marker.
[hds_rob1] = plot_ds( myds, ds.limits,'medium'); hold on;
set(gca,'fontsize',25,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',35);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',35);
hold on
box on
axis(ds.limits);
legend({'$selected\, points$', '$target$'}, 'Interpreter','Latex', 'FontSize',25);


%% plot the successful execution after incremental learning
% case 1: far away from demonstration.
% notice: for successful execution of code, previous section must be first 
%         executed.

figure();
s_traj = load('success_traj_case1/robot_real.txt');
hold on; plot(s_traj(:,2), s_traj(:,3), 'b', 'LineWidth', 4); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('success_traj_case1/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
att = [0.4; 0.1];
hold on; 
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); 
% border points
border_1 = load('success_traj_case1/border_1.txt');
border_2 = load('success_traj_case1/border_2.txt');

% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
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
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);
box on;
axis(limits);

legend({'$robot\,motion$', '$reference$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',18, 'Location', 'northwest');

%% Incremental learning
% Plot the failed path in Case 2: close to demonstrations
% the path fails due to a new obstacle

att = [0.4;0.1];
figure();
fail_traj = load('failed_traj_case2/robot_real.txt');
hold on; plot(fail_traj(1:6440,2), fail_traj(1:6440,3), 'b', 'LineWidth', 4); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('failed_traj_case2/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');

hold on; scatter(fail_traj(6440,2),fail_traj(6440,3), 150, [0 0 1],'x','Linewidth',4); 
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% border points

border_1 = load('failed_traj_case2/border_1.txt');
border_2 = load('failed_traj_case2/border_2.txt');

% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.19,0,0.02,0.4345],'LineWidth', 2, 'LineStyle','-'); 
x = [0.19  0.21 0.21 0.19]; y=[0 0 0.4345 0.4345]; fill(x,y,[0.6350 0.0780 0.1840]);
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');

% plot ds streamlines in background 
limits = [-0.2, 0.7, -0.2, 0.7];
N_points = 20;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 

% starting from original location
x0 = [-0.0468; 0.1508];
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
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
axis(ds.limits);

hold on; scatter(fail_traj(6440,2),fail_traj(6440,3), 150, [0 0 1],'x','Linewidth',4); 
legend({'$robot\, motion$', '$reference$', '$stop\, point$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',18);

%% Plot streamlines after incremental learning
% case 2, close to demonstration.
% escape from VSDS, demonstrate new path, update DS.

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
h_act = figure(); 
selected = [];
for i = 1:160
    if mod(i,2) == 0
        selected(end+1,:) = new_pos(i,1:2);
    end
end
selected = [selected; new_pos(160:end,1:2)];
scatter(selected(:,1), selected(:,2), 'b'); hold on;

ds.limits = [-0.2, 0.7, -0.2, 0.7];
myds =  @(x) gp_ds (x, input, att);
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% [0 0 0] is the color RGB setting. 'd' represents the type of the marker.
plot_ds( myds, ds.limits,'medium'); hold on;
set(gca,'fontsize',25,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',35);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',35);
hold on
box on
axis(ds.limits);
legend({'$selected\, points$', '$target$'}, 'Interpreter','Latex', 'FontSize',25);

%% plot the successful execution after incremental learning
% case 2: Close to demonstration.
% notice: for successful execution of code, previous section must be first 
%         executed.

figure();
s_traj = load('success_traj_case2/robot_real.txt');
hold on; plot(s_traj(:,2), s_traj(:,3), 'b', 'LineWidth', 4 ); % take the first 4847 entries, ignore the rest chaotic 

ref_traj = load('success_traj_case2/vsds.txt');
hold on; plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');

hold on; 
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); 

border_1 = load('success_traj_case2/border_1.txt');
border_2 = load('success_traj_case2/border_2.txt');

% plot the wall
hold on
rectangle('Position', [0.295,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.305 0.305 0.295]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on

rectangle('Position', [0.505,0,0.01,0.235],'LineWidth', 2, 'LineStyle','-'); 
x = [0.505  0.515 0.515 0.505]; y=[0 0 0.235 0.235]; fill(x,y,[0.6350 0.0780 0.1840]);
rectangle('Position', [0.295,0,0.21,0.01],'LineWidth', 2, 'LineStyle','-'); 
x = [0.295  0.505 0.505 0.295]; y=[0 0 0.01 0.01]; fill(x,y,[0.6350 0.0780 0.1840]);
% a new wall
% rectangle('Position', [0.1875,0,0.025,0.4345],'LineWidth', 2, 'LineStyle','-'); 
% x = [0.1875  0.2125 0.2125 0.1875]; y=[0 0 0.4345 0.4345]; fill(x,y,'g');
rectangle('Position', [0.19,0,0.02,0.4345],'LineWidth', 2, 'LineStyle','-'); 
x = [0.19  0.21 0.21 0.19]; y=[0 0 0.4345 0.4345]; fill(x,y,[0.6350 0.0780 0.1840]);
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');

% plot streamlines
x0 = [-0.0468; 0.1508];
N_points = 25; stiff_type = 'constant';
gp_lmds = myds; % myds is generated in the previous section
ifplot = 0;
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, att, N_points, ifplot, ds.limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.3;
par = {x_cen, x_len, w_th, sigma_scale};
plot_ds_combined( gp_lmds, my_vsds, ds.limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);
box on;
axis(ds.limits);
legend({'$robot\, motion$', '$reference$', '$target$', '$wall$','$border$'}, 'Interpreter','LaTex', 'FontSize',18);

%% plot an example of stiffness profile evolves based on variance.

var = 0:0.01:1; y = [];
k =1;
for i = 0:0.01:1
   y(k) = smooth_fall(i, 0, 0.85, 1100, 700);
   k = k+1;
end
figure();
plot(var, y, 'LineWidth', 4);
% set(gca,'fontsize',20,'LineWidth',10);
grid on
set(gca,'fontsize',25,'LineWidth',1);
xlabel('$\sigma^2$','Interpreter','LaTex','FontSize',35);
ylabel('$k [N/m]$','Interpreter','LaTex','FontSize',35);
ylim([300 1900])


%% plot an example of how w_th evolves based on variance

var = 0:0.01:1; y = [];
k =1;
for i = 0:0.01:1
   y(k) = smooth_rise(i, 0, 1, 0.5, 0.2);
   k = k+1;
end
figure();
plot(var, y, 'LineWidth', 4);
% set(gca,'fontsize',20,'LineWidth',10);
grid on
set(gca,'fontsize',25,'LineWidth',1);
xlabel('$\sigma^2$','Interpreter','LaTex','FontSize',35);
ylabel('$w_{th}$','Interpreter','LaTex','FontSize',35);
% set(gca,'fontsize',30,'LineWidth',1);
ylim([0.25 0.75])

%% Visualize the sinusoid to truncate the weights

alpha_bar = 0.2; rho = 0.4;
t = -pi:0.01:pi;
y = [];
y_t = [];
for i = -pi:0.01:pi
    val = cos(i)+1;
    y(end+1) = val;
    if val < alpha_bar
        y_t(end+1) = 0;
    elseif val > alpha_bar + rho
        y_t(end+1) = val;
    else
        y_t(end+1) = 0.5*(1+sin(pi*(val-alpha_bar)/rho - pi/2)) * val;
    end
end
figure();

plot(t,y,'color', 'c',  'LineWidth', 2);
hold on;
plot(t,y_t, 'color', 'b', 'LineWidth', 2, 'LineStyle', '--');
set(gca,'fontsize',25,'LineWidth',1);
legend({'$original$', '$truncated$'}, 'Interpreter','latex', 'FontSize',25)
box on; 
grid on;

%% plot the streamlines of linear DS

A  = - 0.4 * eye(2);
att = [0.4; 0.1]; 
figure();
lin_ds = @(x) l_ds(x, A, att);
ds.limits = [-0.2, 0.7, -0.2, 0.7];

scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
% [0 0 0] is the color RGB setting. 'd' represents the type of the marker.
plot_ds(lin_ds, ds.limits,'medium'); hold on;

xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',25);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',25);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);
set(gca,'fontsize',25,'LineWidth',1);
legend({'$target$'}, 'Interpreter','latex', 'FontSize',20);

box on;
axis(ds.limits);


%% streamlines of LMDS + local VSDS before incremental learning.

border_1 = load('org_success_traj_1/border_1.txt');
border_2 = load('org_success_traj_1/border_2.txt');
att = [0.4; 0.1];
ref_traj = load('org_success_traj_1/vsds.txt');
plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
hold on
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
 
points = load('sampled_points.mat').points_new; % GP dataset
limits = [-0.2, 0.7, -0.2, 0.7];
attractor = [0.4; 0.1];
N_points = 20; 
% N_points = 9;
ifplot = 0; % plot out the figure
stiff_type = 'constant'; 
x0 = [-0.0468; 0.1508];  % starting from original location
% x0 = [0.6; 0.4];
gp_lmds =  @(x) gp_ds (x, points, attractor);
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, attractor, N_points, ifplot, limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.3;
par = {x_cen, x_len, w_th, sigma_scale};
plot_ds_combined( gp_lmds, my_vsds, limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',20);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',20);
legend({'$reference$', '$border$', '$target$'}, 'Interpreter','LaTex', 'FontSize',20);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
% legend({'$robot\,motion$', '$reference$', '$target$', '$wall$', '$border$'}, 'Interpreter','LaTex', 'FontSize',12);
axis(limits);

%% streamlines of LMDS + local VSDS after incremental learning, case 1

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
figure();
ds.limits = [-0.2, 0.7, -0.2, 0.7];

myds =  @(x) gp_ds (x, input, att);

% border points
border_1 = load('success_traj_case1/border_1.txt');
border_2 = load('success_traj_case1/border_2.txt');

ref_traj = load('success_traj_case1/vsds.txt');
plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
hold on;

plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on

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
plot_ds_combined( gp_lmds, my_vsds, limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',20);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',20);
legend({'$reference$', '$border$', '$target$'}, 'Interpreter','LaTex', 'FontSize',20);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
axis(limits);


%% streamlines of LMDS + local VSDS after incremental learning, case 2

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
figure(); 

ds.limits = [-0.2, 0.7, -0.2, 0.7];
myds =  @(x) gp_ds (x, input, att);

border_1 = load('success_traj_case2/border_1.txt');
border_2 = load('success_traj_case2/border_2.txt');

ref_traj = load('success_traj_case2/vsds.txt');
plot(ref_traj(:,1), ref_traj(:,2), 'r', 'LineWidth', 4, 'LineStyle','--');
hold on;
plot(border_1(:,1),border_1(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on
scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;
plot(border_2(:,1),border_2(:,2),'m', 'LineWidth',4, 'LineStyle', ':');
hold on

x0 = [-0.0468; 0.1508];
N_points = 25; stiff_type = 'constant';
gp_lmds = myds; % myds is generated in the previous section
ifplot = 0;
% intialize VSDS based on GP-LMDS, starting from point x0.
[A_hat, ~, x_rec, ~, th_begin] = get_vsds_parameters(x0, att, N_points, ifplot, ds.limits, gp_lmds, stiff_type);
sigma_scale = 1;
my_vsds = @(x) vsds(x, A_hat, x_rec, sigma_scale, x0, th_begin);

x_cen = (x_rec(:,1:end-1)+x_rec(:,2:end))/2;   % the center of the springs
x_len = vecnorm(x_rec(:,1:end-1)-x_rec(:,2:end)); % the length
w_th = 0.3;
par = {x_cen, x_len, w_th, sigma_scale};
plot_ds_combined( gp_lmds, my_vsds, ds.limits, par, 'medium'); hold on;
set(gca,'fontsize',20,'LineWidth',1);
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',20);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',20);
% title(['Streamlines of GP-based LMDS'], 'Interpreter','latex', 'FontSize',20);

box on;
axis(ds.limits);
legend({'$reference$', '$border$', '$target$'}, 'Interpreter','LaTex', 'FontSize',20);


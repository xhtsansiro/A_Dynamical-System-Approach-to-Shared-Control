% -------------------------------------------------------------------------
% This script processes demonstrated trajectory，including following steps 
% 1. Filtering 
% 2. Segmentation (remove zero velocity part)
% 3. Downsampling 
% 4. Generate rotation angle and scaling factor from samples.
% -------------------------------------------------------------------------

%% plot the filtered trajectory and the velocity.

traj = load('LfD/01_Data/trial3/data_mes.txt');
pos =[traj(:,2), traj(:,3)];

% create butterworth filter to filter raw data.
fs=500 ; dt=1/fs ;
[B,A]=butter(1,3/(fs/2)); 
pos_filtered=filtfilt(B,A,pos); 

% plot the scatter plot of trajectory after filtering
figure()
plot(pos_filtered(:,1), pos_filtered(:,2), 'b', 'Linewidth',2);
set(gca,'fontsize',25,'LineWidth',1);
hold on
scatter(0.4, 0.1, 150, [0 0 0],'d','Linewidth',2);
grid on 

xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
xlim([-0.05,0.45]); ylim([0.05,0.45]);
legend({'$trajectory$', '$target$'}, 'Interpreter','latex', 'FontSize',25);
% title('Trajectory after filtering');

% create the velocity and filter it 
velocity = diff(pos_filtered)/dt;
velocity(end+1,:)=velocity(end,:) ;
t = (0: length(velocity)-1) * dt;
v_filtered = filtfilt(B,A,velocity); 

figure()

plot(t, v_filtered(:,1),'b', 'LineWidth', 2)
hold on
plot(t, v_filtered(:,2),'r', 'LineWidth', 2);
set(gca,'fontsize',25,'LineWidth',1);
grid on
legend({'$v_y$', '$v_z$'}, 'Interpreter','latex', 'FontSize',30);
xlabel('$t [s]$','Interpreter','LaTex','FontSize',30);
ylabel('$v [m/s]$','Interpreter','LaTex','FontSize',30);
% title(['Velocity Profile before Segmentation'], 'Interpreter','LaTex','fontsize',20,'fontweight','normal');

%% Segmentataion
% start_pos= pos_filtered(1,:); %idx_end = pos_filtered(end,:);
flag_start = false;
for i = 1: length(v_filtered)
    if (norm(v_filtered(i,:)) > 1e-3 && flag_start == false)
        idx_start = i;
        flag_start = true;
    end
    
    % stop 
    if (flag_start == true  && norm(v_filtered(i,:)) < 1e-3)
        idx_end = i;
        break;
    end
    
end
tt = dt * (0:idx_end - idx_start);
pos_seg = pos_filtered(idx_start:idx_end,:);
v_seg = v_filtered(idx_start:idx_end, :);

figure()
plot(pos_seg(:,1), pos_seg(:,2),'b', 'Linewidth',2);
hold on
scatter(0.4, 0.1, 150, [0 0 0],'d','Linewidth',2);
set(gca,'fontsize',25,'LineWidth',1);

xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
xlim([-0.05,0.45]); ylim([0.05,0.45]);
legend({'$Trajectory$', '$Target$'}, 'Interpreter','latex', 'FontSize',30);

% title('Segemented Trajectory')
grid on
figure()
plot(tt, v_seg(:,1),'b', 'LineWidth', 2)
hold on
grid on
plot(tt, v_seg(:,2), 'r', 'LineWidth', 2);
set(gca,'fontsize',25,'LineWidth',1);
legend({'$v_y$', '$v_z$'}, 'Interpreter','latex', 'FontSize',30);
xlabel('$t [s]$','Interpreter','LaTex','FontSize',30);
ylabel('$v [m/s]$','Interpreter','LaTex','FontSize',30);
% title(['Velocity Profile after Segmentation'], 'Interpreter','LaTex','fontsize',20,'fontweight','normal');

%% Downsample the data based on time scale

t_new = linspace(tt(1), tt(end), 200);  % downsampling to 200
pos_train(:,1) = spline(tt, pos_seg(:,1), t_new);
pos_train(:,2) = spline(tt, pos_seg(:,2), t_new);
v_train(:,1) = spline(tt, v_seg(:,1), t_new);
v_train(:,2) = spline(tt, v_seg(:,2), t_new);

%% Use sampled point to generate rotation angle and scaling factor.
% the generated data are saved in 0.4velocity.mat

% Calculate rotation angle theta and scaling factor k 
input = []; v_org = [];
for i = 1:200   % generate 200 points along the trajectory.
    
    % calculate original velocity, linear dynamics is : x' = -0.4x;
    v_origin = [-0.4 * (pos_train(i,1)-0.4); -0.4 * (pos_train(i,2)-0.1)];
   
    v = [v_train(i,1); v_train(i,2)];
    theta = acos(dot (v_origin, v) / (norm(v_origin) * norm(v))); % in radian
    
    % check it is clockwise or anti-clockwise rotation (from v_org to v_real) 
    if (v_origin(1) * v(2) - v_origin(2) * v(1) > 0)
        sgn = 1;
    else
        sgn = -1;
    end
    k = norm(v)/norm(v_origin) -1;
    v_org(:,end+1) = v_origin' ;
    p = [pos_train(i,1), pos_train(i,2), sgn*theta, k];
    input(i,:) =  p;
end


% -------------------------------------------------------------------------
% This script implemente the Gaussian Process Regression (GPR), and checks 
% the prediction from GPR, when the starting pos. is the initial pos. of
% the original demonstration.
% -------------------------------------------------------------------------

%% Load the demonstration data
points = load('0.4velocity.mat').input;
start_pos = [-0.0468, 0.1508];  % origin of the trajectory.
target = [0.4, 0.1];
Br = 0.15; 

% preprocessing, delete points close to the global stable point
points_new = []; cc = 0;
for i = 1: size(points,1)

    if norm(points(i,1:2)-target) > Br   % Inside Br, no points needed 
        points_new(end+1,:) = points(i,:);
    end
end
num = size(points_new,1);   % how many rows

%% GaussianProcessRegression
current = start_pos; 
traj = current;
dt = 0.002;
miu = zeros(length(points_new),1);  % zero-mean
t = 0;
Br = 0.03; 
l_distance = 0.02;
l_distance_h = 0.04; 

v_all =[]; vv = [];
sigma_f = 1; l = 0.001;  % GP hyperparameters
variance = [];
trace = current;
k_XX = sigma_f * exp( -l^-1 * pdist2(points_new(:,1:2), points_new(:,1:2)).^ 2 /2);
k_XX = k_XX + 0.01*eye(num);   % no inverse of the matrix
R = chol(k_XX);  % do chelosky decomposition

k_Xx = zeros(1,num); k_xx = sigma_f; idx = 0;
% T = [];
% tic

% keep evolving till coming close to the target.
while norm (current-target) > 1e-2
    
    idx = idx + 1;
    tic   % start of the cycle
    k_Xx = sigma_f * exp( -l^-1 * pdist2 (current, points_new(:,1:2)).^2 /2); 
    
    alpha = (R\(R'\k_Xx'))';  % (k_XX ^-1 * k_xX)' = k_Xx * k_XX^-1
    alpha_t = truncate(alpha);   % truncate weights

    theta_pred = alpha_t * points_new(:,3);
    k_pred = alpha_t * points_new(:,4);
        
    if abs(k_pred + 1) < 1e-2  % before was 1e-2 
        k_pred = 0;
    
    % the scaling factor is not allowed to be smaller than -1.
    elseif k_pred < -1 
        k_pred = 0;
    elseif k_pred > 2
        k_pred = 2;
    else
        k_pred = 1* k_pred;
    end

    % calculate the prediction
    variance = k_xx - k_Xx * alpha';
    vv(end+1) = variance;  % note down the predictive variance.
    
    % calculate the velocity prediction based on theta and k
    Mr = [cos(theta_pred) -sin(theta_pred); sin(theta_pred) cos(theta_pred)];
    M = (1+k_pred) * Mr;
    v_o = -0.4 * (current- target);  % the originial linear dynamics
    v_o = velocity_limit(v_o);  % limit the velocity. 
    v_n = (M * v_o')';   % modulated velocity
    v_n = velocity_limit(v_n);  % limit the velocity. 
    
    % v_all = [v_all; v_n];
    v_all(:, end+1) = v_n;
    current = current + v_n * dt;
%     T(end+1) = toc;
    t = t + dt;
    trace(end+1,:) = current;
    % trace = [trace; current];
    
end
% toc

%% Plot the trajectory.
%hold on
figure()
%scatter(trace(:,1), trace(:,2),8, 'b');
% hold on
plot(trace(:,1), trace(:,2),'LineWidth',2, 'Color', 'm');

hold on 
% scatter(points(:,1), points(:,2), 'g');
plot(points(:,1), points(:,2), 'LineWidth',2, 'Color', 'b')
scatter(target(1),target(2), 150, [0 0 0],'d','Linewidth',2); 
ylim([0.05, 0.45]); % the limits of y axis

hold on 
scatter(points_new(:,1), points_new(:,2), 'm');
grid on
box on
set(gca,'fontsize',25,'LineWidth',1);

legend('$predicted \,trajectory$', '$original\, demonstration$', '$target$', '$selected\, points$','Interpreter','LaTex','FontSize',20, 'Location', 'southwest');
xlabel('$x_y [m]$','Interpreter','LaTex','FontSize',30);
ylabel('$x_z [m]$','Interpreter','LaTex','FontSize',30);
title(['GP Predictive Trajectory'], 'Interpreter','latex', 'FontSize',20);


%% Compare the predictive velocity profile with the demonstr. velocity profile
load ('velocity.mat');
% create butterworth filter to filter raw data.
fs=500 ;
[B,A]=butter(1,3/(fs/2)); 
v_all=filtfilt(B,A,v_all'); 
tt = dt * (1:length(vv)); 

t_new = linspace(tt(1), tt(end), 200);  % downsampling to 200
v_t(:,1) = spline(tt, v_all(:,1), t_new);
v_t(:,2) = spline(tt, v_all(:,2), t_new);

figure()
plot (v_train(:,1), 'LineWidth', 2);
hold on
plot (v_t(:,1), 'LineWidth', 2);
set(gca,'fontsize',25,'LineWidth',1);
grid on
box on
ylabel('$v_y [m/s]$','Interpreter','LaTex','FontSize',30);
legend({'$demonstration$', '$prediction$'}, 'Interpreter','latex', 'FontSize',25);

figure()
plot (v_train(:,2), 'LineWidth', 2);
hold on 
plot (v_t(:,2), 'LineWidth',2);
set(gca,'fontsize',25,'LineWidth',1);
grid on
box on
ylabel('$v_z [m/s]$','Interpreter','LaTex','FontSize',30);
legend({'$demonstration$', '$prediction$'}, 'Interpreter','latex', 'FontSize',25);


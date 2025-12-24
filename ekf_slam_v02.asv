clc;
clear;
close all;

%% ============================================================
% FULL EKF-SLAM (Robot Pose + Landmark Map Estimation)
% State: [x y theta xL1 yL1 xL2 yL2 ...]'
% ============================================================

%% Simulation parameters
dt = 0.1;
N  = 350;

%% Control inputs
v = 1.0;
w = 0.08;

%% True landmark positions (UNKNOWN to filter)
landmarks_true = [ 5  10  15  10;
                  10   0  10  15];
numL = size(landmarks_true,2);

%% Noise covariances
R = diag([0.01 0.01 0.005]);   % Robot motion noise
Q = diag([0.1 0.02]);         % Measurement noise


%% Initial true robot state
x_true = [0; 0; 0];

%% Initial EKF-SLAM state (robot only)
u_0 = zeros(3 + 2*numL, 1); 

robot_pose_block = zeros(3, 3);
landmark_block = diag(ones(2*numL, 1) * 1e6); % Very high uncertainty 

Sigma_0 = blkdiag(robot_pose_block, landmark_block); % (3+2N)x(3+2N) matrix

%% Landmark initialization flag
landmark_initialized = false(1, numL);

%% Data storage
true_path = zeros(3,N);
est_path  = zeros(3,N);

Sigma_history = zeros(3+2*numL, 3+2*numL, N);
state_history = zeros(3+2*numL, N);


%% ============================================================
% Main EKF-SLAM Loop
% ============================================================

u_t_1 = u_0;
Sigma_t_1 = Sigma_0;

I_1 = eye(3);
Z = zeros(3, 2*numL);
F_x = [I_1, Z];
R_new = F_x' * R * F_x;
I_2 = eye(3 + 2*numL);

for k = 1:N
    
        %% -------- TRUE ROBOT MOTION --------
    x_true = x_true + ...
        [v*dt*cos(x_true(3));
         v*dt*sin(x_true(3));
         w*dt];
    
        %% -------- EKF PREDICTION --------
              
    u_t_predicted = u_t_1 +  F_x' * [v*dt*cos(u_t_1(3));
                           v*dt*sin(u_t_1(3));
                           w*dt];
     
    p = [0 0 -v*dt*sin(u_t_1(3));
         0 0  v*dt*cos(u_t_1(3));
         0 0  0];
    G_t = I_2 + F_x' * p * F_x;
    Sigma_t_predicted = G_t * Sigma_t_1 * G_t' + R_new;
    
    %% -------- EKF UPDATE (FOR EACH LANDMARK) --------
    for i = 1:numL
        
        % True measurement
        dx = landmarks_true(1,i) - x_true(1);
        dy = landmarks_true(2,i) - x_true(2);
        r  = sqrt(dx^2 + dy^2);
        b  = atan2(dy, dx) - x_true(3);

        z = [r; b] + sqrt(Q)*randn(2,1);
        
        idx = 3 + 2*i - 1;   % Landmark index in state
        
        %% ---- Initialize landmark if unseen ----
        if ~landmark_initialized(i)
            u_jx = u_t_predicted(1) + z(1) * cos(z(2) + u_t_predicted(3));
            u_jy = u_t_predicted(2) + z(1) * sin(z(2) + u_t_predicted(3));
            
            u_t_predicted(idx) = u_jx;
            u_t_predicted(idx+1) = u_jy;
            
            landmark_initialized(i) = true;
            continue;
        end
        
        delta_x = u_t_predicted(idx) - u_t_predicted(1);
        delta_y = u_t_predicted(idx+1) - u_t_predicted(2);
        
        delta = [delta_x;
                 delta_y];
        q = delta' * delta;
        
        z_pred = [sqrt(q);
                  atan2(delta_y, delta_x) - u_t_predicted(3)];

        P1 = [eye(3); zeros(2, 3)];
        P2 = zeros(5, 2*i - 2);
        P3 = [zeros(3, 2); eye(2)];
        P4 = zeros(5, 2*numL - 2*i);

        % Concatenate all parts 
        F_xj = [P1, P2, P3, P4];
        
        h_it = (1/q) * [ -sqrt(q)*delta_x, -sqrt(q)*delta_y,  0,  sqrt(q)*delta_x,  sqrt(q)*delta_y;
                  delta_y,         -delta_x,        -q, -delta_y,          delta_x ];

        H_it = h_it * F_xj;
        
        % Kalman gain
        S = H_it * Sigma_t_predicted *  H_it' + Q;
        K = (Sigma_t_predicted * H_it') / S;
        
        % Update
        
        Z_ = z - z_pred;
        Z_(2) = atan2(sin(Z_(2)), cos(Z_(2)));
        
        u_t_predicted = u_t_predicted + K * (Z_);
        Sigma_t_predicted = (eye(3+2*numL) - K * H_it) * Sigma_t_predicted;
    end
    
    u_t_corrected = u_t_predicted;
    u_t_1 = u_t_corrected;
    
    Sigma_t_corrected = Sigma_t_predicted;
    Sigma_t_1 = Sigma_t_corrected;
    
    true_path(:,k) = x_true;
    est_path(:,k)  = u_t_corrected(1:3);
    
    Sigma_history(:,:,k) = Sigma_t_corrected;
    state_history(:,k)   = u_t_corrected;

    

end 

% Robot state errors
x_error = est_path(1,:) - true_path(1,:);
y_error = est_path(2,:) - true_path(2,:);
theta_error = est_path(3,:) - true_path(3,:);

% Wrap angle error to [-pi, pi]
theta_error = atan2(sin(theta_error), cos(theta_error));


%% ============================================================
% PLOTTING
% ============================================================

figure;
plot(true_path(1,:), true_path(2,:), 'g-', 'LineWidth', 2); hold on;
plot(est_path(1,:), est_path(2,:), 'b--', 'LineWidth', 2);
plot(landmarks_true(1,:), landmarks_true(2,:), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Plot estimated landmarks
for i = 1:numL
    idx = 3 + 2*i - 1;
    plot(u_t_corrected(idx), u_t_corrected(idx+1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
end

grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Trajectory','EKF-SLAM Trajectory','True Landmarks','Estimated Landmarks');
title('Full EKF-SLAM: Robot and Map Estimation');

%% ============================================================
% ERROR PLOTS 
% ============================================================

figure;

subplot(3,1,1);
plot(x_error, 'LineWidth', 1.8);
grid on;
ylabel('x error (m)');
title('Robot State Estimation Errors');

subplot(3,1,2);
plot(y_error, 'LineWidth', 1.8);
grid on;
ylabel('y error (m)');

subplot(3,1,3);
plot(theta_error, 'LineWidth', 1.8);
grid on;
xlabel('Time step');
ylabel('\theta error (rad)');

%% ============================================================
% LANDMARK UNCERTAINTY EVOLUTION
% ============================================================

figure; hold on;

plot(landmarks_true(1,:), landmarks_true(2,:), 'ro', ...
     'MarkerSize', 10, 'LineWidth', 2);

steps = [50 150 350];     % Early, mid, late iterations
colors = {'r','g','b'};
labels = {'Early (k=50)', 'Mid (k=150)', 'Final (k=350)'};
h_legend = [];

for s = 1:length(steps)
    k = steps(s);
    Sigma_k = Sigma_history(:,:,k);
    mu_k    = state_history(:,k);

    for i = 1:numL
        idx = 3 + 2*i - 1;

        mu_L = mu_k(idx:idx+1);
        Sigma_L = Sigma_k(idx:idx+1, idx:idx+1);

        h = draw_covariance_ellipse(mu_L, Sigma_L, 2, colors{s});
        if i == 1
            h_legend(s) = h; % Store the handle for the legend
        end
    end
end

grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend([h_legend], labels, 'Location', 'best');
title('Evolution of Landmark Uncertainty Ellipses');


%% ============================================================
% LANDMARK UNCERTAINTY ELLIPSES (FINAL ESTIMATE)
% ============================================================

figure;
plot(landmarks_true(1,:), landmarks_true(2,:), 'ro', ...
     'MarkerSize', 10, 'LineWidth', 2); hold on;

for i = 1:numL
    idx = 3 + 2*i - 1;

    % Estimated landmark mean
    mu_L = u_t_corrected(idx:idx+1);

    % Landmark covariance
    Sigma_L = Sigma_t_corrected(idx:idx+1, idx:idx+1);

    % Draw 95% uncertainty ellipse
    draw_covariance_ellipse(mu_L, Sigma_L, 2, 'b');
end

grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Landmarks','95% Landmark Uncertainty');
title('Landmark Position Uncertainty Ellipses');


%% ============================================================
% Landmark Uncertainty Ellipses
% ============================================================
function h = draw_covariance_ellipse(mu, Sigma, scale, color)
% mu     : 2x1 mean [x; y]
% Sigma  : 2x2 covariance matrix
% scale  : confidence scaling (2 ? 95%)
% color  : plot color

    [V, D] = eig(Sigma);
    t = linspace(0, 2*pi, 100);

    a = scale * sqrt(D(1,1));
    b = scale * sqrt(D(2,2));

    ellipse = V * [a*cos(t); b*sin(t)];

    h = plot(mu(1) + ellipse(1,:), mu(2) + ellipse(2,:), color, 'LineWidth', 1.5);
end


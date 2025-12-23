clc;
clear;
close all;

%% ============================================================
% FULL EKF-SLAM (Robot Pose + Landmark Map Estimation)
% State: [x y theta xL1 yL1 xL2 yL2 ...]'
% ============================================================

%% Simulation parameters
dt = 0.1;
N  = 250;

%% Control inputs
v = 1.0;
w = 0.08;

%% True landmark positions (UNKNOWN to filter)
landmarks_true = [ 5  10  15;
                  10   0  10 ];
numL = size(landmarks_true,2);

%% Noise covariances
Q = diag([0.01 0.01 0.005]);   % Robot motion noise
R = diag([0.3 0.05]);          % Measurement noise

%% Initial true robot state
x_true = [0; 0; 0];

%% Initial EKF-SLAM state (robot only)
x_est = [0; 0; 0];
P = eye(3) * 0.1;

%% Landmark initialization flag
landmark_initialized = false(1, numL);

%% Data storage
true_path = zeros(3,N);
est_path  = zeros(3,N);

%% ============================================================
% Main EKF-SLAM Loop
% ============================================================
for k = 1:N

    %% -------- TRUE ROBOT MOTION --------
    x_true = x_true + ...
        [v*dt*cos(x_true(3));
         v*dt*sin(x_true(3));
         w*dt];

    %% -------- EKF PREDICTION --------
    x_est(1:3) = x_est(1:3) + ...
        [v*dt*cos(x_est(3));
         v*dt*sin(x_est(3));
         w*dt];

    Fx = eye(length(x_est));
    Fx(1:3,1:3) = ...
        [1 0 -v*dt*sin(x_est(3));
         0 1  v*dt*cos(x_est(3));
         0 0  1];

    Q_full = zeros(length(x_est));
    Q_full(1:3,1:3) = Q;

    P = Fx * P * Fx' + Q_full;

    %% -------- EKF UPDATE (FOR EACH LANDMARK) --------
    for i = 1:numL

        % True measurement
        dx = landmarks_true(1,i) - x_true(1);
        dy = landmarks_true(2,i) - x_true(2);
        r  = sqrt(dx^2 + dy^2);
        b  = atan2(dy, dx) - x_true(3);

        z = [r; b] + sqrt(R)*randn(2,1);

        %% ---- Initialize landmark if unseen ----
        if ~landmark_initialized(i)
            lx = x_est(1) + z(1)*cos(z(2) + x_est(3));
            ly = x_est(2) + z(1)*sin(z(2) + x_est(3));

            x_est = [x_est; lx; ly];

            P = blkdiag(P, eye(2)*1e3);
            landmark_initialized(i) = true;
            continue;
        end

        %% ---- EKF update for known landmark ----
        idx = 3 + 2*i - 1;   % Landmark index in state

        lx = x_est(idx);
        ly = x_est(idx+1);

        dx = lx - x_est(1);
        dy = ly - x_est(2);
        q  = dx^2 + dy^2;

        z_pred = [sqrt(q);
                  atan2(dy, dx) - x_est(3)];

        % Measurement Jacobian
        H = zeros(2, length(x_est));

        H(:,1:3) = ...
            [-dx/sqrt(q), -dy/sqrt(q), 0;
              dy/q,       -dx/q,      -1];

        H(:,idx:idx+1) = ...
            [ dx/sqrt(q), dy/sqrt(q);
             -dy/q,       dx/q];

        % Kalman gain
        S = H * P * H' + R;
        K = P * H' / S;

        % Update
        x_est = x_est + K * (z - z_pred);
        P = (eye(length(x_est)) - K * H) * P;
    end

    %% -------- SAVE DATA --------
    true_path(:,k) = x_true;
    est_path(:,k)  = x_est(1:3);
end

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
    plot(x_est(idx), x_est(idx+1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
end

grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Trajectory','EKF-SLAM Trajectory','True Landmarks','Estimated Landmarks');
title('Full EKF-SLAM: Robot and Map Estimation');

% --- Add this inside your plotting section ---
% Plot Robot Uncertainty Ellipse
plot_ellipse(x_est(1:2), P(1:2,1:2), 'b');

% Plot Landmark Uncertainty Ellipses
for i = 1:numL
    idx = 3 + 2*i - 1;
    plot_ellipse(x_est(idx:idx+1), P(idx:idx+1, idx:idx+1), 'r');
end

% --- Add this function at the very bottom of the file ---
function plot_ellipse(mu, cov, color)
    % Calculate the error ellipse for a 95% confidence interval
    [eigenvec, eigenval] = eig(cov);
    phi = atan2(eigenvec(2,1), eigenvec(1,1));
    theta = linspace(0, 2*pi, 100);
    
    % 2.447 is the scaling factor for 95% confidence in 2D
    chisquare_val = 2.447; 
    
    a = chisquare_val * sqrt(eigenval(1,1));
    b = chisquare_val * sqrt(eigenval(2,2));
    
    ellipse_x = a * cos(theta);
    ellipse_y = b * sin(theta);
    
    % Rotate and translate the ellipse
    R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    rotated_coords = R * [ellipse_x; ellipse_y];
    
    plot(rotated_coords(1,:) + mu(1), rotated_coords(2,:) + mu(2), color);
end

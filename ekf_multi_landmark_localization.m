clc;
clear;
close all;

%% ============================================================
% EKF Localization with Multiple Known Landmarks (Near-SLAM)
% State: x = [x; y; theta]
% ============================================================

%% Simulation parameters
dt = 0.1;          % Time step (s)
N  = 250;          % Number of time steps

%% Noise covariance matrices
Q = diag([0.01 0.01 0.005]);   % Process noise covariance
R = diag([0.3 0.05]);          % Measurement noise [range; bearing]

%% Initial states
x_true = [0; 0; 0];            % True robot state
x_est  = [0; 0; 0];            % Estimated state
P = eye(3);                    % Initial covariance

%% Control inputs
v = 1.0;                       % Linear velocity (m/s)
w = 0.08;                      % Angular velocity (rad/s)

%% Known landmark positions (map is known)
landmarks = [ 5  10  15;
              10  0  10 ];     % Each column is [x_L; y_L]
numL = size(landmarks,2);

%% Data storage
true_path = zeros(3,N);
est_path  = zeros(3,N);

%% ============================================================
% Main EKF Loop
% ============================================================
for k = 1:N

    %% -------- TRUE ROBOT MOTION --------
    x_true = x_true + ...
        [v*dt*cos(x_true(3));
         v*dt*sin(x_true(3));
         w*dt];

    %% -------- EKF PREDICTION --------
    x_pred = x_est + ...
        [v*dt*cos(x_est(3));
         v*dt*sin(x_est(3));
         w*dt];

    F = [1 0 -v*dt*sin(x_est(3));
         0 1  v*dt*cos(x_est(3));
         0 0  1];

    P_pred = F * P * F' + Q;

    %% -------- EKF UPDATE (MULTIPLE LANDMARKS) --------
    x_upd = x_pred;
    P_upd = P_pred;

    for i = 1:numL
        % Landmark position
        lx = landmarks(1,i);
        ly = landmarks(2,i);

        % True measurement
        dx_t = lx - x_true(1);
        dy_t = ly - x_true(2);

        r_true = sqrt(dx_t^2 + dy_t^2);
        b_true = atan2(dy_t, dx_t) - x_true(3);

        z = [r_true; b_true] + sqrt(R)*randn(2,1);

        % Predicted measurement
        dx = lx - x_upd(1);
        dy = ly - x_upd(2);
        q  = dx^2 + dy^2;

        z_pred = [sqrt(q);
                  atan2(dy, dx) - x_upd(3)];

        % Measurement Jacobian
        H = [-dx/sqrt(q),  -dy/sqrt(q),  0;
              dy/q,        -dx/q,       -1];

        % Kalman gain
        K = P_upd * H' / (H * P_upd * H' + R);

        % Update
        x_upd = x_upd + K * (z - z_pred);
        P_upd = (eye(3) - K * H) * P_upd;
    end

    x_est = x_upd;
    P     = P_upd;

    %% -------- SAVE DATA --------
    true_path(:,k) = x_true;
    est_path(:,k)  = x_est;
end

%% ============================================================
% PLOTTING
% ============================================================

figure;
plot(true_path(1,:), true_path(2,:), 'g-', 'LineWidth', 2); hold on;
plot(est_path(1,:), est_path(2,:), 'b--', 'LineWidth', 2);
plot(landmarks(1,:), landmarks(2,:), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Trajectory', 'EKF Estimate', 'Landmarks');
title('EKF Localization with Multiple Landmarks');

% Position error
pos_error = sqrt((true_path(1,:) - est_path(1,:)).^2 + ...
                 (true_path(2,:) - est_path(2,:)).^2);

figure;
plot(pos_error, 'k', 'LineWidth', 2);
grid on;
xlabel('Time step');
ylabel('Position Error (m)');
title('Position Estimation Error');

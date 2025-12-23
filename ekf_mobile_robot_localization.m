clc;
clear;
close all;

%% ============================================================
%  Extended Kalman Filter for Mobile Robot Localization
%  State: [x; y; theta]
% ============================================================

%% Simulation parameters
dt = 0.1;              % Time step (s)
N  = 200;              % Number of steps

%% Noise covariance matrices
Q = diag([0.01 0.01 0.005]);   % Process noise covariance
R = diag([0.5 0.5]);           % Measurement noise covariance

%% Initial states
x_true = [0; 0; 0];    % True state
x_est  = [0; 0; 0];    % EKF estimated state
P = eye(3);            % Initial estimation covariance

%% Storage for plotting
true_path = zeros(3, N);
est_path  = zeros(3, N);
meas_path = zeros(2, N);

%% Control inputs (constant velocity motion)
v = 1.0;               % Linear velocity (m/s)
w = 0.1;               % Angular velocity (rad/s)

%% ============================================================
%  Main EKF Loop
% ============================================================
for k = 1:N

    %% -------- TRUE ROBOT MOTION (GROUND TRUTH) --------
    x_true = x_true + ...
        [v*dt*cos(x_true(3));
         v*dt*sin(x_true(3));
         w*dt];

    %% -------- MEASUREMENT MODEL --------
    % Direct noisy measurement of x and y position
    z = x_true(1:2) + sqrt(R)*randn(2,1);

    %% -------- EKF PREDICTION STEP --------
    % State prediction using nonlinear motion model
    x_pred = x_est + ...
        [v*dt*cos(x_est(3));
         v*dt*sin(x_est(3));
         w*dt];

    % Jacobian of motion model
    F = [1 0 -v*dt*sin(x_est(3));
         0 1  v*dt*cos(x_est(3));
         0 0  1];

    % Covariance prediction
    P_pred = F * P * F' + Q;

    %% -------- EKF UPDATE STEP --------
    % Measurement matrix
    H = [1 0 0;
         0 1 0];

    % Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R);

    % State update
    x_est = x_pred + K * (z - H * x_pred);

    % Covariance update
    P = (eye(3) - K * H) * P_pred;

    %% -------- SAVE DATA --------
    true_path(:,k) = x_true;
    est_path(:,k)  = x_est;
    meas_path(:,k) = z;
end

%% ============================================================
%  PLOTTING RESULTS
% ============================================================

% Trajectory plot
figure;
plot(true_path(1,:), true_path(2,:), 'g-', 'LineWidth', 2); hold on;
plot(meas_path(1,:), meas_path(2,:), 'r.', 'MarkerSize', 8);
plot(est_path(1,:), est_path(2,:), 'b--', 'LineWidth', 2);
grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Trajectory', 'Noisy Measurements', 'EKF Estimate');
title('EKF-based Mobile Robot Localization');

% Error plot
position_error = sqrt((true_path(1,:) - est_path(1,:)).^2 + ...
                      (true_path(2,:) - est_path(2,:)).^2);

figure;
plot(position_error, 'k', 'LineWidth', 2);
grid on;
xlabel('Time step');
ylabel('Position Error (m)');
title('EKF Position Estimation Error');

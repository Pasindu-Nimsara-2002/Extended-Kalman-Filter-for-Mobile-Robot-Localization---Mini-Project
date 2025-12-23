clc;
clear;
close all;

%% ============================================================
%  Extended Kalman Filter (EKF) for Mobile Robot Localization
%  with Range-Bearing Landmark Measurements
%
%  State: x = [x; y; theta]
% ============================================================

%% Simulation parameters
dt = 0.1;            % Time step (s)
N  = 200;            % Number of simulation steps

%% Noise covariance matrices
Q = diag([0.01 0.01 0.005]);     % Process noise covariance
R = diag([0.3 0.05]);            % Measurement noise covariance
                               % [range; bearing]

%% Initial states
x_true = [0; 0; 0];              % True robot state
x_est  = [0; 0; 0];              % EKF estimated state
P = eye(3);                      % Initial covariance estimate

%% Landmark position (known)
landmark = [10; 10];             % [x_L; y_L]

%% Control inputs
v = 1.0;                         % Linear velocity (m/s)
w = 0.1;                         % Angular velocity (rad/s)

%% Storage for plotting
true_path = zeros(3, N);
est_path  = zeros(3, N);
meas_rng  = zeros(1, N);
meas_brg  = zeros(1, N);

%% ============================================================
%  Main EKF Loop
% ============================================================
for k = 1:N

    %% -------- TRUE ROBOT MOTION --------
    x_true = x_true + ...
        [v*dt*cos(x_true(3));
         v*dt*sin(x_true(3));
         w*dt];

    %% -------- MEASUREMENT GENERATION --------
    dx = landmark(1) - x_true(1);
    dy = landmark(2) - x_true(2);

    range_true   = sqrt(dx^2 + dy^2);
    bearing_true = atan2(dy, dx) - x_true(3);

    % Add measurement noise
    z = [range_true; bearing_true] + sqrt(R)*randn(2,1);

    %% -------- EKF PREDICTION STEP --------
    x_pred = x_est + ...
        [v*dt*cos(x_est(3));
         v*dt*sin(x_est(3));
         w*dt];

    F = [1 0 -v*dt*sin(x_est(3));
         0 1  v*dt*cos(x_est(3));
         0 0  1];

    P_pred = F * P * F' + Q;

    %% -------- EKF UPDATE STEP --------
    dx = landmark(1) - x_pred(1);
    dy = landmark(2) - x_pred(2);
    q  = dx^2 + dy^2;

    % Predicted measurement
    z_pred = [sqrt(q);
              atan2(dy, dx) - x_pred(3)];

    % Measurement Jacobian
    H = [-dx/sqrt(q),  -dy/sqrt(q),  0;
          dy/q,        -dx/q,       -1];

    % Kalman Gain
    K = P_pred * H' / (H * P_pred * H' + R);

    % State update
    x_est = x_pred + K * (z - z_pred);

    % Covariance update
    P = (eye(3) - K * H) * P_pred;

    %% -------- SAVE DATA --------
    true_path(:,k) = x_true;
    est_path(:,k)  = x_est;
    meas_rng(k)    = z(1);
    meas_brg(k)    = z(2);
end

%% ============================================================
%  PLOTTING RESULTS
% ============================================================

% Trajectory plot
figure;
plot(true_path(1,:), true_path(2,:), 'g-', 'LineWidth', 2); hold on;
plot(est_path(1,:), est_path(2,:), 'b--', 'LineWidth', 2);
plot(landmark(1), landmark(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
legend('True Trajectory', 'EKF Estimate', 'Landmark');
title('EKF Localization using Range-Bearing Measurements');

% Position error plot
pos_error = sqrt((true_path(1,:) - est_path(1,:)).^2 + ...
                 (true_path(2,:) - est_path(2,:)).^2);

figure;
plot(pos_error, 'k', 'LineWidth', 2);
grid on;
xlabel('Time step');
ylabel('Position Error (m)');
title('Position Estimation Error (EKF)');

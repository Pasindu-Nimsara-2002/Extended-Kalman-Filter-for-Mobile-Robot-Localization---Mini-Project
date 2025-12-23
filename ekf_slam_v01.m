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

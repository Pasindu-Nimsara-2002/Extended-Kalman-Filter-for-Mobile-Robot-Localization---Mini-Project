# EKF-SLAM Mini Project Report

Author: (Your Name)
Date: December 2025

**Abstract**

This report documents an Extended Kalman Filter SLAM implementation for a differential mobile robot. The implementation is in `ekf_slam_v01.m`. I describe the algorithm, show how the code follows the lecture algorithm, list differences, and provide instructions to reproduce results.

**Introduction**

- Problem: Estimate robot pose and map (landmark positions) concurrently using EKF-SLAM.
- Files of interest: `ekf_slam_v01.m`, supporting scripts `ekf_mobile_robot_localization.m` and `ekf_multi_landmark_localization.m`.

**Algorithm (summary)**

State vector:

$$x = [x_r\; y_r\; \theta_r\; x_{L1}\; y_{L1} \; x_{L2} \; y_{L2} \; \dots]^T$$

Prediction:

- Motion model (discrete): robot pose updated by control-driven increment.
- Linearize motion by Jacobian $G_t$ and predict covariance by

$$\Sigma_{t|t-1} = G_t \Sigma_{t-1|t-1} G_t^T + F_x^T R F_x$$

Measurement (range-bearing) per landmark:

- Measurement function: $z = h(x)$ where $h$ returns range and bearing to a landmark.
- Linearize with Jacobian $H$, compute innovation covariance $S = H \Sigma H^T + Q$, Kalman gain $K = \Sigma H^T S^{-1}$, then update.

Landmark initialization: invert the first observation to get landmark coordinates in world frame and insert into state vector.

**Implementation details (`ekf_slam_v01.m`)**

- Motion prediction: implemented with robot increment added to state and Jacobian `G_t` built using small-angle linearization — matches lecture approach.
- Process noise: robot 3×3 `R` mapped into full-state via `F_x' * R * F_x` (same as $F_x^T R F_x$).
- Measurements: range & bearing computed from true landmarks; observations are corrupted with noise drawn by `sqrt(Q)*randn(2,1)`.
- Sequential update: for each landmark the code computes the 5×(3+2N) Jacobian slice `H_it`, S, K, and applies state and covariance updates.
- Landmark initialization: first observation sets the landmark mean via inverse measurement; a boolean `landmark_initialized` tracks this.

**Comparison to lecture algorithm (matches)**

- **Prediction step:** motion update and linearized Jacobian `G_t` present.
- **Process noise mapping:** uses `F_x' * R * F_x` to embed robot process noise into full covariance.
- **Measurement model & Jacobian:** range-bearing model and derived Jacobian are consistent with standard EKF-SLAM formulas.
- **Kalman update:** computes `S`, `K` and updates `u` and `Sigma` as lecture prescribes.
- **Landmark initialization:** inverse measurement for new landmarks.
- **Angle handling:** bearing innovation normalized via `atan2(sin, cos)`.

**Differences and suggested improvements**

- **Landmark covariance upon initialization:** code sets landmark mean but leaves the landmark covariance large (prior). Lecture typically sets the new landmark covariance and cross-covariances using the Jacobian of the inverse observation — recommended: compute and set the 2×2 block and cross-covariances right after initialization.

- **Initial robot covariance:** current code uses a zero robot prior (`robot_pose_block = zeros(3,3)`), which assumes exact initial pose. If the lecture assumes uncertain initial pose, set a nonzero prior.

- **Measurement noise sampling:** `sqrt(Q)*randn(2,1)` is acceptable for diagonal `Q`, but `chol(Q)` or `mvnrnd` is clearer for general `Q`.

- **State angle wrapping:** the code normalizes angle innovation but does not explicitly wrap the robot pose angle after updates — recommended to wrap `theta` to [-pi,pi] after each update.

- **Batch vs sequential update:** code updates sequentially per landmark. That's valid but you may prefer stacking measurements and performing a single multi-measurement update for numerical stability and efficiency.

- **Data association:** code assumes known correspondences and all landmarks are observed. If lecture discusses association, add matching logic (e.g., nearest-neighbor or gating).

**How to run and reproduce figures**

1. Open MATLAB and change folder to the project directory.
2. Run the script:

```matlab
ekf_slam_v01
```

3. To save the main figure automatically, add the following line at the end of `ekf_slam_v01.m` before `title(...)` or after plotting:

```matlab
saveas(gcf, 'ekf_slam_results.png')
```

4. To force a fixed random seed for repeatability, add at the top of the script:

```matlab
rng(0)
```

**Results**

- (Place results here: paste observed plots and quantitative error metrics such as final pose RMSE and landmark position RMSE.)

**Conclusion**

- The code implements the core EKF-SLAM pipeline and follows the lecture algorithm closely. Addressing landmark-covariance initialization and angle wrapping will make results more faithful to canonical EKF-SLAM and numerically robust.

**References**

- Lecture notes provided in `EN4594 SLAM-II-notes.pdf`.
- Mini-project description `MiniProject_FilterImplementation.pdf`.

**Appendix: Suggested code snippets**

- Landmark covariance initialization (concept):

```matlab
% After computing Jacobian J_inv of inverse measurement mapping:
Sigma_x = Sigma_t_predicted; % before update
% Compute new covariance blocks and cross-covariances here using J_inv and measurement noise
```

- Wrap angle helper:

```matlab
wrap = @(a) atan2(sin(a), cos(a));
state(3) = wrap(state(3));
```

---

(End of draft)

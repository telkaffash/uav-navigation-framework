function [x_pred, P_pred] = ekf_predict(x_est, P, imu_accel, imu_gyro_yaw, cfg)
% ekf_predict  EKF prediction step driven by IMU measurements.
%
% State vector: [x; y; z; vx; vy; vz; yaw]  (7x1)
%
% Inputs:
%   x_est         (7x1) current state estimate
%   P             (7x7) current covariance
%   imu_accel     (3x1) noisy accelerometer reading [ax; ay; az]
%   imu_gyro_yaw  scalar noisy yaw rate (rad/s)
%   cfg           config struct from default_config()
%
% Outputs:
%   x_pred   (7x1) predicted state
%   P_pred   (7x7) predicted covariance

dt       = cfg.dt;
n_states = 7;

%% State transition Jacobian (linearised constant-velocity + IMU accel)
F_jac = eye(n_states);
F_jac(1,4) = dt;
F_jac(2,5) = dt;
F_jac(3,6) = dt;

%% Predicted state
x_pred    = x_est;
x_pred(1) = x_est(1) + x_est(4)*dt + 0.5*imu_accel(1)*dt^2;
x_pred(2) = x_est(2) + x_est(5)*dt + 0.5*imu_accel(2)*dt^2;
x_pred(3) = x_est(3) + x_est(6)*dt;
x_pred(4) = x_est(4) + imu_accel(1)*dt;
x_pred(5) = x_est(5) + imu_accel(2)*dt;
x_pred(6) = x_est(6) + imu_accel(3)*dt;
x_pred(7) = x_est(7) + imu_gyro_yaw*dt;

%% Clamp to map
x_pred(1) = min(max(x_pred(1), cfg.map_xlim(1)), cfg.map_xlim(2));
x_pred(2) = min(max(x_pred(2), cfg.map_ylim(1)), cfg.map_ylim(2));

%% Process noise covariance Q
q_pos = (cfg.sigma_accel * dt^2)^2;
q_vel = (cfg.sigma_accel * dt)^2;
q_yaw = (cfg.sigma_gyro  * dt)^2;
Q = diag([q_pos, q_pos, (cfg.sigma_baro*0.01)^2, q_vel, q_vel, 1e-6, q_yaw]);

%% Predicted covariance
P_pred = F_jac * P * F_jac' + Q;

end
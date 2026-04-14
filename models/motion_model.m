function drone_true_full = motion_model(drone_true_full, u_xy, cfg)
% motion_model  Propagates the true 7-state drone through one time step.
%
% State vector: [x; y; z; vx; vy; vz; yaw]
%
% Inputs:
%   drone_true_full  (7x1) current true state
%   u_xy             (1x2) velocity command [vx, vy]
%   cfg              config struct from default_config()
%
% Output:
%   drone_true_full  (7x1) updated true state (with process noise)

dt = cfg.dt;

drone_true_full(1) = drone_true_full(1) + u_xy(1)*dt + cfg.sigma_accel*randn()*dt^2;
drone_true_full(2) = drone_true_full(2) + u_xy(2)*dt + cfg.sigma_accel*randn()*dt^2;
drone_true_full(3) = cfg.z_ground + cfg.sigma_baro*0.3*randn();   % altitude hovers
drone_true_full(4) = u_xy(1) + cfg.sigma_accel*randn();
drone_true_full(5) = u_xy(2) + cfg.sigma_accel*randn();
drone_true_full(6) = 0;
drone_true_full(7) = atan2(u_xy(2), max(norm(u_xy), 1e-9)) + cfg.sigma_gyro*randn();

drone_true_full(1) = min(max(drone_true_full(1), cfg.map_xlim(1)), cfg.map_xlim(2));
drone_true_full(2) = min(max(drone_true_full(2), cfg.map_ylim(1)), cfg.map_ylim(2));

end
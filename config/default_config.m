function cfg = default_config()
% default_config  Returns a struct with all shared parameters.
%
% Usage:
%   cfg = default_config();

%% Map
cfg.map_xlim = [0 6];
cfg.map_ylim = [0 6];
cfg.z_ground = 1.5;          % constant flight altitude (m) for barometer

%% Start / Goal
cfg.start = [0.5 0.5];
cfg.goal  = [5.0 5.0];

%% APF parameters
cfg.rho0   = 1.2;
cfg.k_att  = 1.0;
cfg.k_rep  = 0.8;
cfg.k_tan  = 1.2;
cfg.d_goal = 1.0;

%% Motion
cfg.v_max = 0.08;
cfg.dt    = 1.0;
cfg.N     = 1000;
cfg.tol   = 0.10;
cfg.beta  = 0.20;

%% Sensor noise
cfg.sigma_accel = 0.02;    % m/s^2 accelerometer noise per axis
cfg.sigma_gyro  = 0.005;   % rad/s gyro yaw noise
cfg.sigma_gps   = 0.12;    % m per axis (2D GPS)
cfg.sigma_mag   = 0.03;    % rad (~1.7 deg) compass
cfg.sigma_baro  = 0.12;    % m barometer (BMP280)
cfg.sigma_lidar = 0.05;    % m LiDAR (A2 RPLidar)
cfg.sigma_range = 0.10;    % m generic range sensor (MCL)

%% Moving obstacles
cfg.palms0   = [4 3; 0 1; 1 5];
cfg.palmVel0 = [-0.03 -0.02; +0.06 +0.1; +0.015 -0.03];

%% Particle filter
cfg.n_particles  = 200;
cfg.sigma_motion = 0.04;

end
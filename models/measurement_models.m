function z = measurement_models(sensor, drone_true_full, palms, cfg)
% measurement_models  Simulates noisy sensor measurements from ground truth.
%
% Inputs:
%   sensor           string selecting which sensor to simulate:
%                    'gps'    -> (2x1) noisy [x; y]
%                    'mag'    -> scalar noisy yaw (rad)
%                    'baro'   -> scalar noisy altitude (m)
%                    'lidar'  -> (n_obs x 2) [range, bearing] per obstacle
%                    'range'  -> (n_obs x 1) range to each obstacle (MCL)
%                    'gps2d'  -> (1x2) noisy [x, y] row vector (MCL GPS)
%   drone_true_full  (7x1) true state [x;y;z;vx;vy;vz;yaw], or (1x2) for MCL
%   palms            (n_obs x 2) obstacle positions
%   cfg              config struct from default_config()
%
% Output:
%   z  measurement (size depends on sensor string)

n_obs = size(palms, 1);

switch sensor

    case 'gps'
        z = drone_true_full(1:2) + cfg.sigma_gps * randn(2,1);

    case 'gps2d'
        % Row-vector version used by MCL
        z = drone_true_full(1:2) + cfg.sigma_gps * randn(1,2);

    case 'mag'
        z = drone_true_full(7) + cfg.sigma_mag * randn();

    case 'baro'
        z = drone_true_full(3) + cfg.sigma_baro * randn();

    case 'lidar'
        z = zeros(n_obs, 2);
        for i = 1:n_obs
            dx = palms(i,1) - drone_true_full(1);
            dy = palms(i,2) - drone_true_full(2);
            z(i,1) = sqrt(dx^2 + dy^2) + cfg.sigma_lidar * randn();
            z(i,2) = atan2(dy, dx)     + (cfg.sigma_lidar/1.5) * randn();
        end

    case 'range'
        z = zeros(n_obs, 1);
        for i = 1:n_obs
            z(i) = norm(drone_true_full(1:2) - palms(i,:)) + cfg.sigma_range * randn();
        end

    otherwise
        error('measurement_models: unknown sensor "%s"', sensor);
end

end
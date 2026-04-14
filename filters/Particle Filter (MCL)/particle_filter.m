function [particles, weights, drone_est, N_eff] = particle_filter(...
    particles, weights, u, z_gps, z_range, palms, cfg)
% particle_filter  One full predict-update-estimate cycle of the MCL filter.
%
% Inputs:
%   particles  (n_particles x 2) current particle positions
%   weights    (n_particles x 1) current particle weights
%   u          (1x2) velocity command applied this step
%   z_gps      (1x2) noisy GPS measurement [x, y]
%   z_range    (n_obs x 1) noisy range measurements to each obstacle
%   palms      (n_obs x 2) current obstacle positions
%   cfg        config struct from default_config()
%
% Outputs:
%   particles  updated particle positions
%   weights    updated (and possibly resampled) weights
%   drone_est  (1x2) weighted mean estimate [x, y]
%   N_eff      effective particle count

n_particles = size(particles, 1);
dt          = cfg.dt;

%% Predict
particles = particles + repmat(u * dt, n_particles, 1) + ...
            cfg.sigma_motion * randn(n_particles, 2);
particles(:,1) = min(max(particles(:,1), cfg.map_xlim(1)), cfg.map_xlim(2));
particles(:,2) = min(max(particles(:,2), cfg.map_ylim(1)), cfg.map_ylim(2));

%% Update weights (log-domain for numerical stability)
log_w = zeros(n_particles, 1);
for p = 1:n_particles
    px = particles(p,:);

    err_gps = z_gps - px;
    log_w(p) = log_w(p) - 0.5 * sum(err_gps.^2) / cfg.sigma_gps^2;

    for i = 1:size(palms, 1)
        err_r = z_range(i) - norm(px - palms(i,:));
        log_w(p) = log_w(p) - 0.5 * (err_r^2) / cfg.sigma_range^2;
    end
end
log_w   = log_w - max(log_w);
weights = exp(log_w);
weights = weights / sum(weights);

%% Estimate (weighted mean)
drone_est = sum(particles .* repmat(weights, 1, 2), 1);

%% Effective particle count + systematic resample
N_eff = 1 / sum(weights.^2);
if N_eff < n_particles / 2
    particles = systematic_resample(particles, weights, n_particles);
    weights   = ones(n_particles, 1) / n_particles;
end

end


%% Systematic resampling
function new_p = systematic_resample(particles, weights, N)
% systematic_resample  Low-variance resampling.
    positions = ((0:N-1)' + rand()) / N;
    cumw  = cumsum(weights);
    new_p = zeros(N, 2);
    i = 1;
    for j = 1:N
        while positions(j) > cumw(i) && i < N
            i = i + 1;
        end
        new_p(j,:) = particles(i,:);
    end
end
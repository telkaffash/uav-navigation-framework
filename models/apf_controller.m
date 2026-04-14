function [u, v_prev] = apf_controller(drone_est, goal, palms, v_prev, cfg)
% apf_controller  Computes a velocity command using Artificial Potential Fields.
%
% Combines attractive, repulsive, and tangential forces, then smooths the
% direction with a first-order filter and scales speed by distance to goal.
%
% Inputs:
%   drone_est  (1x2) current estimated position [x, y]
%   goal       (1x2) goal position [x, y]
%   palms      (n_obs x 2) obstacle positions
%   v_prev     (1x2) previous smoothed velocity direction
%   cfg        config struct from default_config()
%
% Outputs:
%   u          (1x2) velocity command [vx, vy]
%   v_prev     updated smoothed direction

g_vec  = goal - drone_est;
g_dist = norm(g_vec);

%% Attractive force
if g_dist < 1e-9
    F_att = [0 0];
elseif g_dist <= cfg.d_goal
    F_att = cfg.k_att * g_vec;
else
    F_att = cfg.k_att * cfg.d_goal * (g_vec / g_dist);
end

%% Repulsive + tangential forces
F_rep_total = [0 0];
F_tan_total = [0 0];

for i = 1:size(palms, 1)
    diff = drone_est - palms(i,:);
    rho  = norm(diff);
    if rho < 1e-6 || rho > cfg.rho0
        continue;
    end
    r_hat = diff / rho;

    F_rep = cfg.k_rep * (1/rho - 1/cfg.rho0) * (1/rho^2) * r_hat;

    t_ccw = [-r_hat(2),  r_hat(1)];
    t_cw  = [ r_hat(2), -r_hat(1)];
    g_hat = g_vec / max(g_dist, 1e-9);
    t_hat = t_ccw;
    if dot(t_cw, g_hat) > dot(t_ccw, g_hat)
        t_hat = t_cw;
    end
    F_tan = cfg.k_tan * (1/rho - 1/cfg.rho0) * t_hat;

    F_rep_total = F_rep_total + F_rep;
    F_tan_total = F_tan_total + F_tan;
end

%% Total force -> velocity command
F_total = F_att + F_rep_total + F_tan_total;
if norm(F_total) < 1e-9
    F_total = v_prev;
end
v_cmd = F_total / norm(F_total);

%% Smooth direction
v_sm = (1 - cfg.beta) * v_prev + cfg.beta * v_cmd;
if norm(v_sm) < 1e-9
    v_sm = v_cmd;
end
v_sm   = v_sm / norm(v_sm);
v_prev = v_sm;

%% Scale speed by proximity to goal
speed = cfg.v_max * min(1, g_dist / 0.8);
u     = speed * v_sm;

end
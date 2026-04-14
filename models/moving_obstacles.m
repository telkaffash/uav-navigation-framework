function [palms, palmVel] = moving_obstacles(palms, palmVel, dt, map_xlim, map_ylim)
% moving_obstacles  Advances obstacle positions and bounces off map walls.
%
% Inputs:
%   palms      (n_obs x 2) current obstacle positions
%   palmVel    (n_obs x 2) current obstacle velocities
%   dt         time step (s)
%   map_xlim   [xmin xmax]
%   map_ylim   [ymin ymax]
%
% Outputs:
%   palms      updated positions
%   palmVel    updated velocities (reversed on wall hit)

palms = palms + palmVel * dt;

hit = palms(:,1) < map_xlim(1) | palms(:,1) > map_xlim(2);
palmVel(hit, 1) = -palmVel(hit, 1);
palms(:,1) = min(max(palms(:,1), map_xlim(1)), map_xlim(2));

hit = palms(:,2) < map_ylim(1) | palms(:,2) > map_ylim(2);
palmVel(hit, 2) = -palmVel(hit, 2);
palms(:,2) = min(max(palms(:,2), map_ylim(1)), map_ylim(2));

end
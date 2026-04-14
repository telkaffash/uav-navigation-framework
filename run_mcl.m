function run_mcl()
% run_mcl  APF navigation with Particle Filter (Monte Carlo Localisation).
%
% Sensors modelled:
%   GPS (2D x,y)     - absolute position measurement
%   Range sensors    - distance to each moving obstacle
%
% Add all subfolders to path before running.

clc; close all;
addpath('config', 'models', genpath('filters'));

cfg = default_config();

%% Unpack frequently used config fields
map_xlim    = cfg.map_xlim;
map_ylim    = cfg.map_ylim;
start       = cfg.start;
goal        = cfg.goal;
rho0        = cfg.rho0;
n_particles = cfg.n_particles;
dt          = cfg.dt;
N           = cfg.N;
tol         = cfg.tol;

%% Initialise obstacles
palms   = cfg.palms0;
palmVel = cfg.palmVel0;
n_obs   = size(palms, 1);

%% Initialise particles
particles = repmat(start, n_particles, 1) + cfg.sigma_motion*5*randn(n_particles,2);
particles(:,1) = min(max(particles(:,1), map_xlim(1)), map_xlim(2));
particles(:,2) = min(max(particles(:,2), map_ylim(1)), map_ylim(2));
weights = ones(n_particles, 1) / n_particles;

drone_true = start;
drone_est  = start;
v_prev     = [1 0];

%% Figure layout
fig = figure('Color','w','Position',[60 60 1280 700]);

ax_map = subplot('Position',[0.03 0.08 0.52 0.88]);
hold(ax_map,'on'); axis(ax_map,'equal');
xlim(ax_map, map_xlim); ylim(ax_map, map_ylim); grid(ax_map,'on');
xlabel(ax_map,'X (m)'); ylabel(ax_map,'Y (m)');
title(ax_map,'APF + Particle Filter - Navigation');

plot(ax_map, start(1),start(2),'go','MarkerSize',8,'MarkerFaceColor','g');
plot(ax_map, goal(1), goal(2), 'md','MarkerSize',9,'MarkerFaceColor','m');

infCircles = gobjects(n_obs,1);
for i = 1:n_obs
    infCircles(i) = rectangle('Parent',ax_map,'Position', ...
        [palms(i,1)-rho0, palms(i,2)-rho0, 2*rho0, 2*rho0], ...
        'Curvature',[1 1],'EdgeColor',[0.6 0.6 1.0],'LineStyle','--');
end

palmPlot = plot(ax_map, palms(:,1),palms(:,2),'kp', ...
    'MarkerSize',14,'MarkerFaceColor',[0.2 0.8 0.2]);
partPlot = scatter(ax_map, particles(:,1),particles(:,2),6, ...
    [1.0 0.5 0.0],'filled','MarkerFaceAlpha',0.35);
truePlot = plot(ax_map, drone_true(1),drone_true(2),'bs', ...
    'MarkerSize',10,'MarkerFaceColor','b');
estPlot  = plot(ax_map, drone_est(1),drone_est(2),'r^', ...
    'MarkerSize',10,'MarkerFaceColor','r');

pathTrueX = drone_true(1); pathTrueY = drone_true(2);
pathEstX  = drone_est(1);  pathEstY  = drone_est(2);
pathTruePlot = plot(ax_map, pathTrueX,pathTrueY,'b-','LineWidth',1.2);
pathEstPlot  = plot(ax_map, pathEstX, pathEstY, 'r--','LineWidth',1.0);

legend(ax_map,{'Start','Goal','Influence radius','Obstacles','Particles', ...
    'True pose','PF estimate','True path','Estimated path'}, ...
    'Location','northwest','FontSize',7);

infoText = text(ax_map, 0.62,0.98,'','Units','normalized', ...
    'VerticalAlignment','top','FontName','Consolas','FontSize',8);

% Error subplot 1: position error
ax_err = subplot('Position',[0.59 0.71 0.39 0.23]);
hold(ax_err,'on'); grid(ax_err,'on');
xlabel(ax_err,'Step'); ylabel(ax_err,'Error (m)');
title(ax_err,'Localization error  |true - est|');
errLine    = plot(ax_err, NaN,NaN,'r-','LineWidth',1.4);
gpsErrLine = plot(ax_err, NaN,NaN,'color',[0.9 0.6 0.1],'LineWidth',0.9,'LineStyle',':');
legend(ax_err,{'PF error','Raw GPS error'},'FontSize',7,'Location','northeast');

% Error subplot 2: X / Y components
ax_xy = subplot('Position',[0.59 0.40 0.39 0.23]);
hold(ax_xy,'on'); grid(ax_xy,'on');
xlabel(ax_xy,'Step'); ylabel(ax_xy,'Error (m)');
title(ax_xy,'X / Y estimation error');
errXLine = plot(ax_xy, NaN,NaN,'b-','LineWidth',1.2);
errYLine = plot(ax_xy, NaN,NaN,'m-','LineWidth',1.2);
yline(ax_xy, 0,'k--','LineWidth',0.6);
legend(ax_xy,{'X error','Y error'},'FontSize',7,'Location','northeast');

% Error subplot 3: effective particle count
ax_neff = subplot('Position',[0.59 0.08 0.39 0.23]);
hold(ax_neff,'on'); grid(ax_neff,'on');
xlabel(ax_neff,'Step'); ylabel(ax_neff,'N_{eff}');
title(ax_neff,'Effective particle count  (resample when < N/2)');
neffLine = plot(ax_neff, NaN,NaN,'k-','LineWidth',1.2);
yline(ax_neff, n_particles/2,'r--','LineWidth',1.0);
legend(ax_neff,{'N_{eff}','Resample threshold'},'FontSize',7,'Location','northeast');
ylim(ax_neff,[0 n_particles+10]);

%% Data logs
steps_log = []; err_log = []; gps_err_log = [];
errX_log  = []; errY_log = []; neff_log   = [];

%% Main loop
for k = 1:N

    % 0) Move obstacles
    [palms, palmVel] = moving_obstacles(palms, palmVel, dt, map_xlim, map_ylim);

    % 1) APF on estimated position
    [u, v_prev] = apf_controller(drone_est, goal, palms, v_prev, cfg);

    % 2) True drone moves (simple 2D for MCL)
    drone_true = drone_true + u*dt + cfg.sigma_motion*randn(1,2);
    drone_true(1) = min(max(drone_true(1), map_xlim(1)), map_xlim(2));
    drone_true(2) = min(max(drone_true(2), map_ylim(1)), map_ylim(2));

    % 3) Measurements
    z_gps   = measurement_models('gps2d', drone_true, palms, cfg);
    z_range = measurement_models('range', drone_true, palms, cfg);

    % 4) Particle filter
    [particles, weights, drone_est, N_eff] = particle_filter(...
        particles, weights, u, z_gps, z_range, palms, cfg);

    % 5) Log errors
    pos_err = norm(drone_true - drone_est);
    gps_err = norm(drone_true - z_gps);
    errX    = drone_true(1) - drone_est(1);
    errY    = drone_true(2) - drone_est(2);

    steps_log(end+1)   = k;       %#ok
    err_log(end+1)     = pos_err; %#ok
    gps_err_log(end+1) = gps_err; %#ok
    errX_log(end+1)    = errX;    %#ok
    errY_log(end+1)    = errY;    %#ok
    neff_log(end+1)    = N_eff;   %#ok

    % 6) Update map
    pathTrueX(end+1) = drone_true(1); pathTrueY(end+1) = drone_true(2); %#ok
    pathEstX(end+1)  = drone_est(1);  pathEstY(end+1)  = drone_est(2);  %#ok

    set(pathTruePlot,'XData',pathTrueX,'YData',pathTrueY);
    set(pathEstPlot, 'XData',pathEstX, 'YData',pathEstY);
    set(truePlot,    'XData',drone_true(1),'YData',drone_true(2));
    set(estPlot,     'XData',drone_est(1), 'YData',drone_est(2));
    set(partPlot,    'XData',particles(:,1),'YData',particles(:,2));
    set(palmPlot,    'XData',palms(:,1),'YData',palms(:,2));
    for i = 1:n_obs
        set(infCircles(i),'Position', ...
            [palms(i,1)-rho0, palms(i,2)-rho0, 2*rho0, 2*rho0]);
    end

    % 7) Update error plots
    set(errLine,    'XData',steps_log,'YData',err_log);
    set(gpsErrLine, 'XData',steps_log,'YData',gps_err_log);
    set(errXLine,   'XData',steps_log,'YData',errX_log);
    set(errYLine,   'XData',steps_log,'YData',errY_log);
    set(neffLine,   'XData',steps_log,'YData',neff_log);

    if k > 1
        xlim(ax_err,  [1 k]);
        xlim(ax_xy,   [1 k]);
        xlim(ax_neff, [1 k]);
    end

    set(infoText,'String',sprintf([ ...
        'Step: %d\n' ...
        'True:  (%.2f, %.2f)\n' ...
        'Est:   (%.2f, %.2f)\n' ...
        'PF err: %.3f m\n' ...
        'GPS err: %.3f m\n' ...
        'N_eff: %.0f/%d'], ...
        k, drone_true(1),drone_true(2), ...
        drone_est(1),drone_est(2), ...
        pos_err, gps_err, N_eff, n_particles));

    drawnow; pause(0.01);

    if norm(goal - drone_true) < tol
        disp('Goal reached!');
        break;
    end
end

%% Final summary plot
figure('Color','w','Position',[100 100 1100 380]);

subplot(1,3,1);
plot(steps_log, err_log,    'r-',  'LineWidth',1.4); hold on;
plot(steps_log, gps_err_log,'color',[0.9 0.6 0.1],'LineWidth',0.9,'LineStyle',':');
xlabel('Step'); ylabel('Error (m)'); title('Localization error'); grid on;
legend({'PF error','Raw GPS error'},'FontSize',8);

subplot(1,3,2);
plot(steps_log, errX_log,'b-','LineWidth',1.2); hold on;
plot(steps_log, errY_log,'m-','LineWidth',1.2);
yline(0,'k--','LineWidth',0.7);
xlabel('Step'); ylabel('Error (m)'); title('X / Y error'); grid on;
legend({'X error','Y error'},'FontSize',8);

subplot(1,3,3);
plot(steps_log, neff_log,'k-','LineWidth',1.2); hold on;
yline(n_particles/2,'r--','LineWidth',1.0);
xlabel('Step'); ylabel('N_{eff}'); title('Effective particle count'); grid on;
legend({'N_{eff}','Resample threshold'},'FontSize',8);

sgtitle('Particle Filter Performance Summary','FontSize',13,'FontWeight','bold');
end
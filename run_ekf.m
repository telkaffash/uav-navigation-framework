function run_ekf()
% run_ekf  APF navigation with Extended Kalman Filter sensor fusion.
%
% Sensors modelled:
%   IMU  (BMI270)       - prediction step: integrates accel + gyro
%   GPS  (M80-5883)     - absolute (x,y) position update
%   Compass (M80-5883)  - yaw heading update
%   Barometer (BMP280)  - altitude z update
%   LiDAR (A2 RPLidar)  - range-to-obstacle bearing-range update
%
% EKF state: [x; y; z; vx; vy; vz; yaw]  (7x1)
%
% Add all subfolders to path before running.

clc; close all;
addpath('config', 'models', genpath('filters'));

cfg = default_config();

%% Unpack frequently used config fields
map_xlim = cfg.map_xlim;
map_ylim = cfg.map_ylim;
start    = cfg.start;
goal     = cfg.goal;
rho0     = cfg.rho0;
dt       = cfg.dt;
N        = cfg.N;
tol      = cfg.tol;

%% Initialise obstacles
palms   = cfg.palms0;
palmVel = cfg.palmVel0;
n_obs   = size(palms, 1);

%% EKF initialisation
n_states = 7;
x_est = [start(1); start(2); cfg.z_ground; 0; 0; 0; 0];
P     = diag([0.1 0.1 0.05 0.01 0.01 0.01 0.02].^2);

%% True drone state (full 7-state)
drone_true_full = [start(1); start(2); cfg.z_ground; 0; 0; 0; 0];
drone_true      = start;
v_prev          = [1 0];

%% Figure layout
fig = figure('Color','w','Position',[60 60 1280 700]);

ax_map = subplot('Position',[0.03 0.08 0.52 0.88]);
hold(ax_map,'on'); axis(ax_map,'equal');
xlim(ax_map, map_xlim); ylim(ax_map, map_ylim); grid(ax_map,'on');
xlabel(ax_map,'X (m)'); ylabel(ax_map,'Y (m)');
title(ax_map,'APF + EKF Sensor Fusion - Navigation');

plot(ax_map, start(1),start(2),'go','MarkerSize',8,'MarkerFaceColor','g');
plot(ax_map, goal(1), goal(2), 'md','MarkerSize',9,'MarkerFaceColor','m');

infCircles = gobjects(n_obs,1);
for i = 1:n_obs
    infCircles(i) = rectangle('Parent',ax_map, ...
        'Position',[palms(i,1)-rho0, palms(i,2)-rho0, 2*rho0, 2*rho0], ...
        'Curvature',[1 1],'EdgeColor',[0.6 0.6 1.0],'LineStyle','--');
end

palmPlot = plot(ax_map, palms(:,1),palms(:,2),'kp', ...
    'MarkerSize',14,'MarkerFaceColor',[0.2 0.8 0.2]);
truePlot = plot(ax_map, drone_true(1),drone_true(2),'bs', ...
    'MarkerSize',10,'MarkerFaceColor','b');
estPlot  = plot(ax_map, x_est(1),x_est(2),'r^', ...
    'MarkerSize',10,'MarkerFaceColor','r');
ellH     = plot(ax_map, NaN,NaN,'r-','LineWidth',1.0);

pathTrueX = drone_true(1); pathTrueY = drone_true(2);
pathEstX  = x_est(1);      pathEstY  = x_est(2);
pathTruePlot = plot(ax_map, pathTrueX,pathTrueY,'b-','LineWidth',1.2);
pathEstPlot  = plot(ax_map, pathEstX, pathEstY, 'r--','LineWidth',1.0);

legend(ax_map,{'Start','Goal','Influence radius','Obstacles', ...
    'True pose','EKF estimate','3 sigma ellipse','True path','EKF path'}, ...
    'Location','northwest','FontSize',7);

infoText = text(ax_map, 0.62,0.98,'','Units','normalized', ...
    'VerticalAlignment','top','FontName','Consolas','FontSize',8);

% Error subplot 1: position error
ax_err = subplot('Position',[0.59 0.71 0.39 0.23]);
hold(ax_err,'on'); grid(ax_err,'on');
xlabel(ax_err,'Step'); ylabel(ax_err,'Error (m)');
title(ax_err,'Localization error |true - est|');
errLine    = plot(ax_err, NaN,NaN,'r-','LineWidth',1.4);
gpsErrLine = plot(ax_err, NaN,NaN,'color',[0.9 0.6 0.1],'LineWidth',0.9,'LineStyle',':');
legend(ax_err,{'EKF error','Raw GPS error'},'FontSize',7,'Location','northeast');

% Error subplot 2: X / Y components
ax_xy = subplot('Position',[0.59 0.40 0.39 0.23]);
hold(ax_xy,'on'); grid(ax_xy,'on');
xlabel(ax_xy,'Step'); ylabel(ax_xy,'Error (m)');
title(ax_xy,'X / Y estimation error');
errXLine = plot(ax_xy, NaN,NaN,'b-','LineWidth',1.2);
errYLine = plot(ax_xy, NaN,NaN,'m-','LineWidth',1.2);
yline(ax_xy, 0,'k--','LineWidth',0.6);
legend(ax_xy,{'X error','Y error'},'FontSize',7,'Location','northeast');

% Error subplot 3: yaw error
ax_yaw = subplot('Position',[0.59 0.08 0.39 0.23]);
hold(ax_yaw,'on'); grid(ax_yaw,'on');
xlabel(ax_yaw,'Step'); ylabel(ax_yaw,'Yaw error (rad)');
title(ax_yaw,'Yaw estimation error (EKF vs compass)');
yawErrLine = plot(ax_yaw, NaN,NaN,'k-','LineWidth',1.2);
yline(ax_yaw, 0,'r--','LineWidth',0.8);

%% Data logs
steps_log = []; err_log = []; gps_err_log = [];
errX_log  = []; errY_log = []; yawErr_log = [];

%% Main loop
for k = 1:N

    % 0) Move obstacles
    [palms, palmVel] = moving_obstacles(palms, palmVel, dt, map_xlim, map_ylim);

    % 1) APF on EKF estimated (x,y)
    drone_est2d = x_est(1:2)';
    [u_xy, v_prev] = apf_controller(drone_est2d, goal, palms, v_prev, cfg);

    % 2) True drone moves (full 7-state)
    drone_true_full = motion_model(drone_true_full, u_xy, cfg);
    drone_true      = drone_true_full(1:2)';

    % 3) Sensor measurements
    imu_accel    = [u_xy(1)/dt + cfg.sigma_accel*randn(); ...
                    u_xy(2)/dt + cfg.sigma_accel*randn(); ...
                    0];
    imu_gyro_yaw = cfg.sigma_gyro * randn();

    z_gps   = measurement_models('gps',   drone_true_full, palms, cfg);
    z_mag   = measurement_models('mag',   drone_true_full, palms, cfg);
    z_baro  = measurement_models('baro',  drone_true_full, palms, cfg);
    z_lidar = measurement_models('lidar', drone_true_full, palms, cfg);

    % 4) EKF predict
    [x_pred, P_pred] = ekf_predict(x_est, P, imu_accel, imu_gyro_yaw, cfg);

    % 5) EKF update: GPS (x,y)
    H_gps = zeros(2, n_states);
    H_gps(1,1) = 1; H_gps(2,2) = 1;
    R_gps = cfg.sigma_gps^2 * eye(2);
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, z_gps, H_gps, R_gps, @(x) x(1:2));

    % 6) EKF update: compass (yaw)
    H_mag = zeros(1, n_states);
    H_mag(1,7) = 1;
    R_mag = cfg.sigma_mag^2;
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, z_mag, H_mag, R_mag, @(x) x(7));

    % 7) EKF update: barometer (z)
    H_baro = zeros(1, n_states);
    H_baro(1,3) = 1;
    R_baro = cfg.sigma_baro^2;
    [x_pred, P_pred] = ekf_update(x_pred, P_pred, z_baro, H_baro, R_baro, @(x) x(3));

    % 8) EKF update: LiDAR (nonlinear range-bearing)
    for i = 1:n_obs
        px = x_pred(1); py = x_pred(2);
        ox = palms(i,1); oy = palms(i,2);
        dx = ox - px;   dy = oy - py;
        rng_pred  = sqrt(dx^2 + dy^2 + 1e-9);
        bear_pred = atan2(dy, dx);

        H_lidar = zeros(2, n_states);
        H_lidar(1,1) = -dx/rng_pred;    H_lidar(1,2) = -dy/rng_pred;
        H_lidar(2,1) =  dy/rng_pred^2;  H_lidar(2,2) = -dx/rng_pred^2;

        z_l    = z_lidar(i,:)';
        h_pred = [rng_pred; bear_pred];
        innov  = z_l - h_pred;
        innov(2) = wrapToPi(innov(2));

        R_lidar = diag([cfg.sigma_lidar^2, (cfg.sigma_lidar/1.5)^2]);
        S = H_lidar * P_pred * H_lidar' + R_lidar;
        K = P_pred * H_lidar' / S;
        x_pred = x_pred + K * innov;
        P_pred = (eye(n_states) - K * H_lidar) * P_pred;
    end

    % Symmetrise and commit
    P_pred = 0.5 * (P_pred + P_pred');
    x_est  = x_pred;
    P      = P_pred;

    % 9) Log errors
    pos_err = norm(drone_true_full(1:2) - x_est(1:2));
    gps_err = norm(drone_true_full(1:2) - z_gps);
    yaw_err = wrapToPi(drone_true_full(7) - x_est(7));

    steps_log(end+1)   = k;        %#ok
    err_log(end+1)     = pos_err;  %#ok
    gps_err_log(end+1) = gps_err;  %#ok
    errX_log(end+1)    = drone_true_full(1) - x_est(1); %#ok
    errY_log(end+1)    = drone_true_full(2) - x_est(2); %#ok
    yawErr_log(end+1)  = yaw_err;  %#ok

    pathTrueX(end+1) = drone_true_full(1); %#ok
    pathTrueY(end+1) = drone_true_full(2); %#ok
    pathEstX(end+1)  = x_est(1);           %#ok
    pathEstY(end+1)  = x_est(2);           %#ok

    % 10) Update map
    [ev, ed] = eig(P(1:2,1:2));
    sig3  = 3*sqrt(diag(ed))';
    theta = linspace(0, 2*pi, 60);
    ell   = ev * diag(sig3) * [cos(theta); sin(theta)];
    set(ellH,'XData',x_est(1)+ell(1,:),'YData',x_est(2)+ell(2,:));

    set(pathTruePlot,'XData',pathTrueX,'YData',pathTrueY);
    set(pathEstPlot, 'XData',pathEstX, 'YData',pathEstY);
    set(truePlot,    'XData',drone_true_full(1),'YData',drone_true_full(2));
    set(estPlot,     'XData',x_est(1),'YData',x_est(2));
    set(palmPlot,    'XData',palms(:,1),'YData',palms(:,2));
    for i = 1:n_obs
        set(infCircles(i),'Position', ...
            [palms(i,1)-rho0, palms(i,2)-rho0, 2*rho0, 2*rho0]);
    end

    set(errLine,    'XData',steps_log,'YData',err_log);
    set(gpsErrLine, 'XData',steps_log,'YData',gps_err_log);
    set(errXLine,   'XData',steps_log,'YData',errX_log);
    set(errYLine,   'XData',steps_log,'YData',errY_log);
    set(yawErrLine, 'XData',steps_log,'YData',yawErr_log);

    if k > 1
        xlim(ax_err, [1 k]);
        xlim(ax_xy,  [1 k]);
        xlim(ax_yaw, [1 k]);
    end

    set(infoText,'String',sprintf([ ...
        'Step: %d\n' ...
        'True:   (%.2f, %.2f, z=%.2f)\n' ...
        'EKF:    (%.2f, %.2f, z=%.2f)\n' ...
        'Yaw est: %.2f deg\n' ...
        'Pos err: %.3f m\n' ...
        'GPS err: %.3f m\n' ...
        '3sig x: %.3f m  y: %.3f m'], ...
        k, ...
        drone_true_full(1),drone_true_full(2),drone_true_full(3), ...
        x_est(1),x_est(2),x_est(3), ...
        rad2deg(x_est(7)), ...
        pos_err, gps_err, ...
        3*sqrt(P(1,1)), 3*sqrt(P(2,2))));

    drawnow; pause(0.01);

    if norm(goal - drone_true) < tol
        disp('Goal reached!');
        break;
    end
end

%% Final summary plot
figure('Color','w','Position',[100 100 1200 380]);

subplot(1,3,1);
plot(steps_log, err_log,    'r-', 'LineWidth',1.4); hold on;
plot(steps_log, gps_err_log,'color',[0.9 0.6 0.1],'LineStyle',':','LineWidth',0.9);
xlabel('Step'); ylabel('Error (m)'); title('Position error'); grid on;
legend({'EKF','Raw GPS'},'FontSize',8);

subplot(1,3,2);
plot(steps_log, errX_log,'b-','LineWidth',1.2); hold on;
plot(steps_log, errY_log,'m-','LineWidth',1.2);
yline(0,'k--','LineWidth',0.7);
xlabel('Step'); ylabel('Error (m)'); title('X / Y error'); grid on;
legend({'X','Y'},'FontSize',8);

subplot(1,3,3);
plot(steps_log, rad2deg(yawErr_log),'k-','LineWidth',1.2); hold on;
yline(0,'r--','LineWidth',1.0);
xlabel('Step'); ylabel('Yaw error (deg)'); title('Yaw error'); grid on;

sgtitle('EKF Sensor Fusion Performance Summary','FontSize',13,'FontWeight','bold');
end
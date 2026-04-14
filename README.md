# APF Navigation with EKF and Particle Filter

MATLAB simulation of a drone navigating from a start position to a goal using **Artificial Potential Fields (APF)** for path planning, combined with two localisation approaches: an **Extended Kalman Filter (EKF)** and a **Particle Filter (MCL)**. Three moving obstacles bounce around the map and must be avoided in real time.

---

## Project Structure

```
.
├── config/
│   └── default_config.m          # All tunable parameters in one place
├── filters/
│   ├── Extended Kalman Filter/
│   │   ├── ekf_predict.m         # IMU-driven prediction step
│   │   └── ekf_update.m          # Generic linear/nonlinear update step
│   └── Particle Filter (MCL)/
│       └── particle_filter.m     # Predict, weight, estimate, resample
├── models/
│   ├── apf_controller.m          # APF force calculation and velocity command
│   ├── measurement_models.m      # Sensor simulation (GPS, IMU, LiDAR, etc.)
│   ├── motion_model.m            # True drone state propagation (7-state)
│   └── moving_obstacles.m        # Obstacle kinematics with wall bouncing
├── results/
│   ├── comparison/               # Store side-by-side result plots here
│   ├── ekf/                      # Store EKF run outputs here
│   └── mcl/                      # Store MCL run outputs here
├── run_ekf.m                     # Entry point: EKF fusion run
├── run_mcl.m                     # Entry point: Particle filter run
└── README.md
```

---

## Quick Start

1. Open MATLAB and set the project root as your working directory.
2. Run either entry point:

```matlab
run_ekf   % APF + Extended Kalman Filter
run_mcl   % APF + Particle Filter (MCL)
```

Both scripts call `addpath` automatically, so no manual path setup is needed.

---

## How It Works

### Path Planning: Artificial Potential Fields

The drone is attracted to the goal and repelled by obstacles. A tangential force component is added to help escape local minima near obstacles. The resulting force vector is normalised and smoothed with a first-order filter before being used as a velocity command. Speed is scaled down as the drone approaches the goal.

### Localisation: Extended Kalman Filter (`run_ekf`)

The EKF maintains a 7-state estimate: `[x, y, z, vx, vy, vz, yaw]`.

**Prediction** is driven by IMU (accelerometer + gyro) readings.

**Updates** fuse four sensor types sequentially:
- GPS (M80-5883): absolute 2D position
- Compass (M80-5883): yaw heading
- Barometer (BMP280): altitude
- LiDAR (A2 RPLidar): range and bearing to each obstacle (nonlinear update with full Jacobian)

A 3-sigma uncertainty ellipse is drawn live on the map.

### Localisation: Particle Filter (`run_mcl`)

200 particles represent the belief over the drone's 2D position.

Each step:
1. Particles are propagated forward using the velocity command plus motion noise.
2. Weights are updated in log-domain using GPS and range-to-obstacle likelihoods.
3. The estimate is the weighted mean of all particles.
4. Systematic resampling is triggered when the effective particle count `N_eff` drops below `N/2`.

---

## Sensors Modelled

| Sensor | Used in | Noise model |
|---|---|---|
| GPS (M80-5883) | EKF + MCL | sigma = 0.12 m per axis |
| IMU / Accel (BMI270) | EKF | sigma = 0.02 m/s^2 |
| Gyro (BMI270) | EKF | sigma = 0.005 rad/s |
| Compass (M80-5883) | EKF | sigma = 0.03 rad |
| Barometer (BMP280) | EKF | sigma = 0.12 m |
| LiDAR (A2 RPLidar) | EKF | sigma = 0.05 m |
| Range sensors | MCL | sigma = 0.10 m |

---

## Configuration

All parameters live in `config/default_config.m`. Key settings:

| Parameter | Default | Description |
|---|---|---|
| `map_xlim`, `map_ylim` | [0 6] | Map bounds (m) |
| `start`, `goal` | [0.5 0.5], [5 5] | Start and goal positions |
| `rho0` | 1.2 | Obstacle influence radius (m) |
| `k_att`, `k_rep`, `k_tan` | 1.0, 0.8, 1.2 | APF gain constants |
| `v_max` | 0.08 | Maximum drone speed (m/s) |
| `beta` | 0.20 | Velocity smoothing factor |
| `n_particles` | 200 | Number of MCL particles |
| `dt` | 1.0 | Time step (s) |
| `N` | 1000 | Maximum simulation steps |
| `tol` | 0.10 | Goal reached tolerance (m) |

---

## Outputs

Each run produces:
- A live animation window showing the map, paths, particles or ellipse, and obstacle positions.
- Three real-time error subplots (position error, X/Y components, and either `N_eff` or yaw error).
- A static summary figure when the simulation ends.

---

## Dependencies

- MATLAB R2020b or later (uses `yline`, `wrapToPi`, `sgtitle`)
- No additional toolboxes required
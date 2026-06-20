# Robotics Maze G1 Production

Slim production branch for Unitree G1 maze navigation in MuJoCo + ROS 2 Humble.

This branch keeps the runtime surface intentionally small:

1. Unitree RL Gym native G1 policy and Unitree G1 model.
2. Added simulated D435i RGB-D/IMU sensor and Livox-style 360 degree laser scan.
3. Square maze grid cells from `1.0` to `4.0` meters.
4. Oracle path-following mode.
5. SLAM with `slam_toolbox`.
6. `m-explore-ros2` + Nav2 cold-start exploration.
7. Docker runtime with ROS 2 Humble, Nav2, SLAM Toolbox, RViz, MuJoCo, and Python deps.
8. Rosbag recording for ROS runs.

## Docker Quick Start

Build the image:

```bash
make docker-build
```

Start a headless shell:

```bash
make docker-run
```

Start a GUI shell for RViz and MuJoCo viewer:

```bash
make docker-run-gui
```

Check that ROS, Nav2, SLAM Toolbox, RViz, and bridge dependencies resolve inside Docker:

```bash
make docker-check-ros
```

Inside Docker, prebuild the ROS workspace and external runtime deps:

```bash
make prebuild
```

## Configuration

The production config is [configs/default.yaml](configs/default.yaml).

The default robot is:

```yaml
robot:
  model_xml_path: third_party/unitree_rl_gym/resources/robots/g1_description/scene.xml
  base_model_xml_path: third_party/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml
```

The default locomotion policy is:

```yaml
nav2_navigation:
  locomotion_policy: unitree_rl_gym_native
```

Maze cells are square. Use `CELL_SIZE_M` between `1.0` and `4.0`:

```bash
make maze CELL_SIZE_M=1.0
make maze CELL_SIZE_M=2.5
make maze CELL_SIZE_M=4.0
```

For compatibility with older command snippets, `CORRIDOR_WIDTH` is accepted as an alias:

```bash
make navigate CELL_SIZE_M=4.0
make navigate CORRIDOR_WIDTH=4.0
```

## Commands

Generate and validate the grid:

```bash
make maze SEED=123 CELL_SIZE_M=4.0
```

Generate the MuJoCo maze world:

```bash
make world SEED=123 CELL_SIZE_M=4.0
```

Run oracle path following with the Unitree RL Gym native policy:

```bash
make oracle SEED=123 CELL_SIZE_M=4.0 ORACLE_DURATION=300
```

Open the MuJoCo viewer for oracle mode:

```bash
make oracle-view SEED=123 CELL_SIZE_M=4.0
```

Run oracle-driven SLAM:

```bash
make slam SEED=123 CELL_SIZE_M=4.0 SLAM_DURATION=300
```

Open RViz during SLAM:

```bash
make slam-view SEED=123 CELL_SIZE_M=4.0
```

Run cold-start navigation with SLAM, `m-explore`, Nav2, and rosbag:

```bash
make navigate SEED=123 CELL_SIZE_M=4.0 NAVIGATE_DURATION=600
```

Open RViz during navigation:

```bash
make navigate-view SEED=123 CELL_SIZE_M=4.0
```

Open RViz and the MuJoCo passive viewer:

```bash
make navigate-full-view SEED=123 CELL_SIZE_M=4.0
```

## Rosbag

`make navigate`, `make navigate-view`, and `make navigate-full-view` record every discovered ROS topic, including hidden topics, under the run directory:

```text
runs/navigate/seed-<seed>/<timestamp>__cell_size-<N>m__duration-<N>s/rosbag/
```

`make slam` and `make slam-view` record the SLAM topics under:

```text
runs/visual/slam_seed-<seed>_bag/
```

Inspect a bag:

```bash
make bag-info BAG=runs/.../rosbag
```

## Docker Environment

The Docker image is based on ROS 2 Humble and installs:

- Nav2
- SLAM Toolbox
- depthimage-to-laserscan
- robot-localization
- RViz2
- MuJoCo Python
- PyTorch CPU dependency path for Unitree RL Gym native policy use
- project Python requirements

Headless Docker uses `MUJOCO_GL=osmesa`.
GUI Docker uses `MUJOCO_GL=glfw` and mounts the X11 socket.

## Runtime Data Flow

Oracle mode:

```text
maze grid -> oracle path planner -> turn-aware follower -> Unitree RL Gym native policy -> MuJoCo G1
```

SLAM mode:

```text
MuJoCo G1 + D435i/Livox scan -> /scan -> slam_toolbox -> /map -> artifacts + rosbag
```

Navigation mode:

```text
MuJoCo G1 + sensors
  -> /scan + D435i/Livox scan odometry
  -> slam_toolbox
  -> /map
  -> m-explore frontiers
  -> Nav2 NavigateToPose
  -> /cmd_vel
  -> Unitree RL Gym native policy
  -> MuJoCo G1
```

`navigate`, `navigate-view`, and `navigate-full-view` do not use MuJoCo ground-truth pose for navigation. The active `/odom` source is `d435i_scan_odometry`, and MuJoCo `/ground_truth/odom` is recorded only for offline comparison and dashboard metrics.

## Outputs

Run artifacts go under `runs/`.

Typical navigation run:

```text
runs/navigate/seed-123/<timestamp>/
  run_manifest.json
  resolved_config.yaml
  resolved_nav2_params.yaml
  world_seed-123.xml
  world_seed-123_topdown.svg
  rosbag/
  rosbag-record.log
  summary.json
  dashboard.html
```

## Verification

Check the production Docker ROS environment:

```bash
make docker-check-ros
```

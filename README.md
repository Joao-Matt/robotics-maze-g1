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

## Interview Demo

Run the live interview demo with the seed provided in the interview:

```bash
make demo SEED=<n> CELL_SIZE_M=4.0
```

`make demo` is a thin wrapper around `make navigate-full-view`: it generates the fresh maze for the seed, runs the same MuJoCo + RViz + SLAM + `m-explore` + Nav2 stack, records the run, and forces the live KPI dashboard on. You can also pass `NAVIGATE_DURATION`, `DASHBOARD_PORT`, and `NAVIGATE_SKIP_BUILD`.

Expected windows during the demo:

- MuJoCo passive viewer: live G1 motion in the generated maze.
- RViz: map, scan, plan, frontiers, and trajectory.
- Browser KPI dashboard: solve status, time-to-goal, collisions/stuck count, and capture health.

Open the dashboard only after the `live_kpi_monitor` log prints the bound URL:

```text
Live KPI dashboard: http://127.0.0.1:<port>/index.html
```

If the run has already ended, the live server is gone. Open the final report instead:

```text
runs/navigate/seed-<n>/latest/dashboard.html
```

Navigation runs also start a lightweight live KPI sidecar by default. The default URL is reachable only while the run is active and only after the monitor prints that it has bound the HTTP server:

```text
Live KPI dashboard: http://127.0.0.1:8765/index.html
```

Disable it or move it to another port with:

```bash
make navigate-full-view NAVIGATE_DASHBOARD=false
make navigate-full-view DASHBOARD_PORT=8770
```

## Deliverables Map

- One-command setup: `make docker-build`, `make docker-run-gui`, then `make prebuild`.
- Generate a maze by seed: `make maze SEED=<n> CELL_SIZE_M=4.0`.
- Run one navigation episode/live demo: `make demo SEED=<n>`.
- Collect data: navigation runs automatically write rosbag, manifest, summary, map images, trajectories, command CSV, localization CSV, and live KPI snapshots under `runs/navigate/seed-<n>/...`.
- Reproduce the KPI report: `make heldout-navigate`, then `make heldout-report`.
- Answer "would you trust this robot to run unsupervised?": use `runs/heldout-20/heldout_summary.html`, because that verdict needs many unseen seeds. The live demo answers "is this run solving cleanly right now?"

## Held-Out Seed Evaluation

Run the fixed 20-seed held-out navigation batch headlessly:

```bash
make heldout-navigate CELL_SIZE_M=4.0 NAVIGATE_DURATION=600
```

This writes per-seed runs under:

```text
runs/heldout-20/navigate/seed-<seed>/<timestamp>/
```

The batch runner prebuilds once, disables the live dashboard, and uses isolated `ROS_DOMAIN_ID` values so short-span parallel runs do not collide:

```bash
make heldout-navigate HELDOUT_JOBS=3 HELDOUT_BASE_ROS_DOMAIN_ID=40
```

Aggregate the latest summary for each held-out seed:

```bash
make heldout-report
```

The report counts success with `maze_goal_reached == true`, prints the held-out solve rate with a 95% Wilson confidence interval, and writes machine-readable data plus an HTML KPI report:

```text
runs/heldout-20/heldout_summary.json
runs/heldout-20/heldout_summary.csv
runs/heldout-20/heldout_summary.html
```

The HTML report includes KPI cards, plots, dominant failure modes, design/tradeoff notes, and the verdict: whether the held-out evidence supports trusting the robot to run unsupervised.

To compare against tuning/development seeds, pass the root containing those `navigate/seed-*` summaries:

```bash
make heldout-report SEEN_NAV_ROOT=runs/development-set/navigate
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
  map.png
  map.pgm
  map.yaml
  ground_truth_map.png
  slam_vs_maze.png
  trajectory.svg
  command_timeline.svg
  cmd_vel.csv
  localization_comparison.csv
  live_dashboard/
    index.html
    kpis.latest.json
    kpi_stream.ndjson
    events.ndjson
    timeseries_downsampled.csv
    map_thumb.png
  summary.json
  dashboard.html
```

## Verification

Check the production Docker ROS environment:

```bash
make docker-check-ros
```

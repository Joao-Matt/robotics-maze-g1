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
make navigate CELL_SIZE_M=2.0
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

Calibrate the frozen G1 walking policy on open-floor MuJoCo command sweeps:

```bash
make locomotion-calibrate SEED=123
make locomotion-calibrate-smoke SEED=123
make locomotion-calibrate-batch SEED=123 CALIBRATION_BATCH_COUNT=100
make locomotion-calibrate-batch-smoke SEED=123
```

The full calibration writes `command_results.csv`, `summary.json`,
`locomotion_calibration.json`, `report.md`, and `dashboard.html` under
`runs/calibration/g1_locomotion/seed-<seed>/<timestamp>/`. Navigation
does not run this sweep automatically. To reuse a measured calibration
for a navigation run, pass it explicitly:

```bash
make navigate SEED=123 LOCOMOTION_CALIBRATION=runs/calibration/g1_locomotion/seed-123/latest/locomotion_calibration.json
```

When `LOCOMOTION_CALIBRATION` is set, `navigate`, `navigate-view`,
`navigate-full-view`, and `demo` default to `NAV2_LIMIT_MODE=use-calibration`.
That makes Nav2's rendered `max_vel_x`, `max_speed_xy`, and `max_vel_theta`
use the measured command envelope through the maze-stability caps in
`nav2_navigation.calibrated_nav2_*`. Normal DWB path following remains
forward-only by default; controlled reverse is still available through the Nav2
BackUp recovery behavior and the bridge-side `max_reverse_mps` limit. To keep
the conservative behavior where calibration only caps the configured Nav2
limits, pass `NAV2_LIMIT_MODE=cap`.

To try those calibrated limits inside a maze without ROS/Nav2, run the
direct MuJoCo oracle maze follower with the same JSON:

```bash
make oracle-calibrated \
  SEED=123 \
  CELL_SIZE_M=4.0 \
  ORACLE_DURATION=300 \
  LOCOMOTION_CALIBRATION=runs/calibration/g1_locomotion/seed-123/latest/locomotion_calibration.json
```

By default `oracle-calibrated` uses `CALIBRATED_ORACLE_SPEED_MODE=cap`,
which keeps the current maze follower speeds and only caps them with
calibration-derived turn/recovery limits. Use
`CALIBRATED_ORACLE_SPEED_MODE=use-safe` to actively try the measured safe
straight speed and preferred tight-turn commands as a more aggressive stress
test.

The batch calibration does not use `CHECKPOINT` or `VEC_NORMALIZE`; it is
testing the frozen walking policy, not the PPO maze controller. It writes
per-seed folders plus `seed_metrics.csv`, `seed_metrics.json`,
`summary.json`, `report.md`, and `dashboard.html` under
`runs/calibration/g1_locomotion_seed_batch/seed-<base-seed>/<timestamp>/`.
Seeds are generated deterministically from `SEED` unless you provide an
explicit list:

```bash
make locomotion-calibrate-batch \
  SEED=123 \
  CALIBRATION_BATCH_SEEDS="11 22 33 44"
```

The batch success rate means each seed completed the calibration sweep
and passed the configured safety gates. It is not a maze goal. The
defaults require at least 70% stable commands, no falls, no stuck
commands, no more than 5 non-floor contacts, and discovered safe limits
of at least 0.40 m/s forward and 0.40 rad/s yaw.

Train the direct MuJoCo PPO velocity controller:

```bash
make rl-train SEED=123 RL_TIMESTEPS=200000 RL_NUM_ENVS=1
```

Evaluate and replay a trained checkpoint:

```bash
make rl-eval CHECKPOINT=runs/rl_velocity/train/<run>/final_model.zip
make rl-replay CHECKPOINT=runs/rl_velocity/train/<run>/final_model.zip SEED=123
```

Run the fixed 100-episode random-maze corridor sweep on a trained checkpoint:

```bash
make rl-eval-corridor-sweep \
  CHECKPOINT=runs/rl_velocity/train/<run>/final_model.zip \
  VEC_NORMALIZE=runs/rl_velocity/train/<run>/vec_normalize.pkl \
  LOCOMOTION_CALIBRATION=runs/calibration/g1_locomotion/seed-123/latest/locomotion_calibration.json
```

That suite is defined in
[configs/rl_velocity_eval_corridor_sweep_100.yaml](configs/rl_velocity_eval_corridor_sweep_100.yaml).
It evaluates 100 deterministic random mazes across 2.0, 2.5, 3.0,
3.5, and 4.0 m corridors. Results are written under
`runs/rl_velocity/eval/<timestamp>/`: `episode_metrics.csv` has the
per-episode fields, and `grouped_summary.json` aggregates by stage,
corridor width, failure status, and turn/straight failure phase.

The RL controller runs without ROS/Nav2/RViz. It learns high-level
`vx`, `vy`, and `yaw_rate` commands around the frozen Unitree RL Gym G1
locomotion policy using oracle path features and MuJoCo wall raycasts.
The default curriculum includes right-turn, left-turn, mirrored S-turn,
and reverse-recovery starts. During training, route-relative observations
can be perturbed with bounded odom drift/noise from
`rl_velocity_controller.odometry_training`; rewards and success checks still
use MuJoCo ground truth so `/odom`-like errors are training difficulty, not
live navigation input.
Configuration lives in [configs/rl_velocity_controller.yaml](configs/rl_velocity_controller.yaml);
outputs are written under `runs/rl_velocity/`.

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
make navigate SEED=123 CELL_SIZE_M=4.0 NAVIGATE_DURATION=1200
```

By default, `nav2_navigation.initial_spawn_yaw_rad: auto` faces the robot from
the generated start cell into the first validated corridor, not toward the wall.
Set that config value to a numeric yaw in radians only when intentionally testing
a fixed spawn orientation.

Open RViz during navigation:

```bash
make navigate-view SEED=123 CELL_SIZE_M=4.0
```

Open RViz and the MuJoCo passive viewer:

```bash
make navigate-full-view SEED=123 CELL_SIZE_M=2.0
```

## Interview Demo

Run the live interview demo with the seed provided in the interview:

```bash
make demo SEED=123 DEMO_CELL_SIZE_M=2.0
```

`make demo` is a thin wrapper around `make navigate-full-view`: it generates the fresh maze for the seed, runs the same MuJoCo + RViz + SLAM + `m-explore` + Nav2 stack, records the run, and forces the live KPI dashboard on. You can also pass `CELL_SIZE_M` or `DEMO_CELL_SIZE_M`, plus `NAVIGATE_DURATION`, `DASHBOARD_PORT`, `DASHBOARD_AUTO_OPEN`, `NAVIGATE_SKIP_BUILD`, and `LOCOMOTION_CALIBRATION`. When calibration is supplied, the demo uses the calibrated Nav2 maze envelope unless `NAV2_LIMIT_MODE=cap` is set. `NAVIGATE_SKIP_BUILD=auto` skips the ROS package rebuild whenever `ros_ws/install` is already present; run `make prebuild` when you intentionally want to refresh the installed ROS packages.

By default, a navigation run counts the generated maze goal as reached when the robot is within one corridor width of the goal. Override that with launch arguments when you want a stricter or looser definition:

```bash
make demo SEED=123 DEMO_CELL_SIZE_M=2.0 NAVIGATE_LAUNCH_ARGS="goal_reached_tolerance_m:=0.5"
make demo SEED=123 DEMO_CELL_SIZE_M=2.0 NAVIGATE_LAUNCH_ARGS="goal_reached_tolerance_cell_width_fraction:=0.5"
```

Expected windows during the demo:

- MuJoCo passive viewer: live G1 motion in the generated maze.
- RViz: map, scan, plan, frontiers, and trajectory.
- Browser KPI dashboard: solve status, time-to-goal, collisions/stuck count, and capture health.

The live KPI dashboard opens automatically in your browser after `live_kpi_monitor` binds the HTTP server. The browser tab/window is yours to close manually. If the browser does not open, use the bound URL printed in the log:

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
make navigate-full-view DASHBOARD_AUTO_OPEN=false
make navigate-full-view DASHBOARD_PORT=8770
```

## Deliverables Map

- One-command setup: `make docker-build`, `make docker-run-gui`, then `make prebuild`.
- Generate a maze by seed: `make maze SEED=123 CELL_SIZE_M=2.0`.
- Run one navigation episode/live demo: `make demo SEED=123`.
- Collect debug data: normal navigation runs write manifest, summary, map images, trajectories, command CSV, localization CSV, live KPI snapshots, and, unless disabled, an all-topic debug rosbag under `runs/navigate/seed-<n>/...`.
- Collect assignment dataset evidence: use `make navigate-record`, which is the schema-validated dataset path.
- Reproduce the KPI report: `make heldout-navigate`, then `make heldout-report`.
- Current submission KPI writeup: `docs/final_kpi_report.md`.
- Answer "would you trust this robot to run unsupervised?": use `runs/heldout-20/heldout_summary.html`, because that verdict needs many unseen seeds. The live demo answers "is this run solving cleanly right now?"

## Held-Out Seed Evaluation

Run the fixed 20-seed held-out navigation batch headlessly:

```bash
make heldout-navigate CELL_SIZE_M=2.0 NAVIGATE_DURATION=1200
```

This writes per-seed runs under:

```text
runs/heldout-20/navigate/seed-<seed>/<timestamp>/
```

The batch runner prebuilds once, disables the live dashboard, disables per-seed rosbag recording by default, and uses isolated `ROS_DOMAIN_ID` values so short-span parallel runs do not collide. This keeps the held-out batch focused on KPI summaries; use `make navigate-record` for the validated dataset artifact. If you intentionally want per-seed debug bags, pass `--record-bag` to `scripts/run_navigation_seed_batch.py`.

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

`make navigate`, `make navigate-view`, and `make navigate-full-view` record every discovered ROS topic, including hidden topics, under the run directory by default:

```text
runs/navigate/seed-<seed>/<timestamp>__cell_size-<N>m__duration-<N>s/rosbag/
```

Set `NAVIGATE_RECORD_BAG=false` only for summary-only KPI/debug runs where a rosbag is not part of the deliverable. Full-view/demo bags are broad debug evidence, not the official assignment dataset unless they are separately validated against the capture schema.

For dataset capture, use `make navigate-record`. It enables raw RGB-D at 3 Hz,
records the documented topic allowlist from
`configs/navigation_capture_topics.yaml`, writes 512 MB split SQLite bag files,
and creates `capture_manifest.json`, `capture_validation.json`, and
`capture_samples.csv`:

```bash
make navigate-record SEED=123
```

Repair or validate a dataset run after `kill -9` or power loss:

```bash
make repair-run RUN_DIR=runs/navigate-record/seed-123/<timestamp>
```

The capture schema and timestamp policy are documented in
`docs/data_capture_schema.md`.

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
  -> /map + map-frame /pose
  -> m-explore frontiers
  -> map/SLAM-pose fallback goals when exploration is idle
  -> Nav2 NavigateToPose
  -> /cmd_vel
  -> Unitree RL Gym native policy
  -> MuJoCo G1
```

`navigate`, `navigate-view`, and `navigate-full-view` do not use MuJoCo ground-truth pose for navigation. The active `/odom` source is `d435i_scan_odometry`, and MuJoCo `/ground_truth/odom` is recorded only for offline comparison and dashboard metrics.
Generated maze/oracle paths are evaluation references only; live navigation goal producers must not read them or publish goals from them.

Tune the odometry stack over repeated maze runs:

```bash
make odom-tune ODOM_TUNE_SEEDS="123 81" ODOM_TUNE_DURATION=240
```

This runs several scan-odometry launch-argument candidates, then scores each run against `localization_evaluation_ground_truth_only` in the generated summaries. Ground truth is used only after each run for scoring, not as an online navigation input. Pass custom launch arguments to one run with `NAVIGATE_LAUNCH_ARGS`, for example:

```bash
make navigate NAVIGATE_LAUNCH_ARGS="scan_maximum_points:=260 icp_min_inlier_ratio:=0.18"
```

Odom quality is evaluated before SLAM tuning. The navigation summary reports
`final_position_error_per_meter`, `yaw_p95_deg`,
`estimated_time_offset_s`, `distance_scale`, and odom jump counts under
`localization_evaluation_ground_truth_only`. Use those fields to target
roughly 0.05-0.10 m final drift per meter, 3-5 deg yaw error over normal
turns, and zero sudden odom jumps.

The active navigation TF tree is intentionally single-owner: `slam_toolbox`
owns `map -> odom`, `d435i_scan_odometry` owns `/odom` plus
`odom -> base_link`, and the MuJoCo bridge disables its active odom TF when
`external_navigation_odom` is true. Direct odom follows the ROS planar
convention: `+X` forward, `+Y` left, `+yaw` counter-clockwise, and integrates
body-frame deltas with `x += cos(yaw) * dx - sin(yaw) * dy`,
`y += sin(yaw) * dx + cos(yaw) * dy`. Current defaults use IMU orientation yaw
with 50% scan-matching yaw correction.

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

Typical dataset capture run also includes:

```text
  capture_schema.yaml
  capture_manifest.json
  capture_validation.json
  capture_samples.csv
```

## Verification

Check the production Docker ROS environment:

```bash
make docker-check-ros
```

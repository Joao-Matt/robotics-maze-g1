# Robotics Maze G1

Milestone-based MuJoCo + Unitree G1 maze-navigation assignment repo.

This repository currently implements **Milestone 3**: project scaffold, reproducible Python environment, configuration loading, MuJoCo installation, Unitree G1 model bring-up, deterministic maze generation/validation, maze-to-MuJoCo world generation, visible command artifacts, tests, and engineering worklog. Navigation, data logging, metrics, and demo behavior are intentionally left for later milestones.

## Repository Layout

```text
configs/        YAML configuration files
assets/         third-party MuJoCo model assets via Git submodules
sim/            simulation integration and shared setup helpers
maze/           deterministic maze generation, validation, and visualization
nav/            future navigation, planning, localization, and control code
data/           future run logging and data schema code
eval/           future KPI and report code
demo/           future live demo entrypoints
scripts/        runnable command-line scripts
tests/          automated tests
docs/           worklog and architecture decision records
runs/           generated run artifacts, ignored by Git
```

## Environment

The project is intended to use Python 3.11 from pyenv and a repo-local virtual environment:

```bash
cd "$HOME/dev_workspaces/robotics-maze-g1"
git submodule update --init --recursive
make setup
```

If Python 3.11.15 is missing, install it first:

```bash
env \
  CPPFLAGS="-I$HOME/.local/openssl/include" \
  LDFLAGS="-L$HOME/.local/openssl/lib" \
  PKG_CONFIG_PATH="$HOME/.local/openssl/lib/pkgconfig" \
  LD_LIBRARY_PATH="$HOME/.local/openssl/lib" \
  CONFIGURE_OPTS="--with-openssl=$HOME/.local/openssl" \
  "$HOME/.pyenv/bin/pyenv" install 3.11.15
```

On the original Ubuntu 20.04 development environment, Python 3.11.15 was built against a user-local OpenSSL under `$HOME/.local/openssl` because interactive `sudo apt` was not available for installing `libssl-dev`. The Makefile sets `LD_LIBRARY_PATH` for repo commands so the venv can use that OpenSSL runtime.

## Docker Quick Start

The Docker environment is the portable development path for ROS work. The host does not need ROS installed. The image contains Ubuntu 22.04, ROS 2 Humble, Nav2, SLAM/navigation packages, RViz/rqt tools, MuJoCo rendering libraries, ffmpeg, Python, and this repo's Python requirements. The Dockerfile defaults to the multi-architecture official `ros:humble-ros-base-jammy` image and installs desktop/navigation packages explicitly; `osrf/ros:humble-desktop` was not used as the default because it is not multi-architecture in this environment.

```bash
make docker-build
make docker-run
make docker-run-gui
make docker-test
make docker-smoke
make docker-check-ros
```

`make docker-run` starts a terminal/headless development shell. `make docker-run-gui` starts a GUI-capable shell for RViz, MuJoCo viewer runs, and dashboard opening when the Linux host has X11 display support; it exits with guidance to use the headless runner when `DISPLAY` or the X11 socket is unavailable. Inside either container, the repo is bind-mounted at `/workspace`, so normal host edits appear immediately inside Docker. Rebuild the image only when dependencies change: `docker/Dockerfile`, `requirements.txt`, apt packages, ROS packages, or system libraries.

The Docker run scripts export `VENV=/usr`, so existing Make targets use the Python packages installed in the image instead of requiring a repo-local `.venv` inside the container. The normal host workflow still defaults to `.venv`.

Examples inside the container:

```bash
echo $ROS_DISTRO
ros2 --help
scripts/check_ros_docker_env.sh
make test
make smoke
make report-milestone_5 SEED=123 MILESTONE_5_DURATION=5
```

GUI examples from a GUI-capable shell:

```bash
rviz2
make milestone_5 SEED=123
```

The run scripts mount the current repo into `/workspace`, use host networking, set `ROS_DOMAIN_ID`, and run as the host UID/GID where practical so generated files under `runs/`, `reports/`, `ros_ws/`, dashboards, videos, logs, and KPI outputs persist on the host without becoming root-owned. If a file does end up owned by root after manual Docker use, fix it from the host with:

```bash
sudo chown -R "$(id -u):$(id -g)" runs reports ros_ws
```

Multi-architecture build support is available for x86 laptops and ARM/Jetson targets:

```bash
make docker-build-multiarch
PLATFORMS=linux/amd64 docker/build_multiarch.sh
PUSH=1 IMAGE_NAME=ghcr.io/<user>/robotics-maze-g1:humble docker/build_multiarch.sh
```

True multi-platform manifests normally require `PUSH=1` and a registry image name. Local `--load` works only for a single platform. GPU acceleration is not part of this base image; this is a portable CPU/GUI-capable development environment first, and GPU support can be added later if needed.

`requirements.txt` pins pytest below 9 because ROS 2 Humble's `launch_testing` pytest plugin is not compatible with pytest 9.

### Storage Safety

Large project outputs stay relative to the repository, including `runs/`, `.venv/`, `third_party/`, generated worlds, dashboards, logs, and videos. Put the checkout on the desired storage device and create an ignored machine-local policy file:

```bash
cp .env.storage.example .env.storage
# Edit REQUIRED_STORAGE_MOUNT and EXPECTED_STORAGE_UUID for this machine.
make storage-check
```

The check runs before environment setup, dependency fetches, Docker builds, tests, and milestone execution. On the host it verifies the required mount, repository, Docker root, and containerd root. In Docker it verifies that `/workspace` is a bind mount rather than the container writable layer.

Docker Engine 29 keeps image layers in containerd separately from Docker's `data-root`. The guarded migration helper backs up configuration, performs conservative cleanup, quarantines old stores on the external filesystem, configures both roots, and restarts the services:

```bash
sudo scripts/migrate_system_storage.sh /path/to/external/mount FILESYSTEM_UUID
```

Run that command from a host terminal after leaving development containers. The script retains its timestamped quarantine until the rebuilt image and tests are verified.

## Milestone Commands

```bash
make docker-build
make docker-run
make docker-run-gui
make docker-test
make docker-check-ros
make smoke
make view-smoke
make test
make view-test
make maze SEED=123
make view-maze SEED=123 MAZE_CELL_PX=48
make world SEED=123
make view-world SEED=123
make run SEED=123 DURATION=3
make view-run SEED=123 DURATION=3
make view SEED=123 VIEW_DURATION=30
make d435i-visual-check SEED=123
make ros-bridge-check SEED=123
make ros-bridge-view SEED=123
make d435i-scan-check SEED=123
make d435i-scan-view SEED=123
make slam-map SEED=123
make slam-map-view SEED=123
make nav2-slam-demo SEED=123
make nav2-slam-view SEED=123
make milestone_4 SEED=123
make view-milestone_4 SEED=123
make milestone_4-wide SEED=123
make view-milestone_4-wide SEED=123
make milestone_5 SEED=123
make view-milestone_5 SEED=123
make report-milestone_5 SEED=123
make g1-oracle-follow SEED=123             # compatibility alias for milestone_5
make view-g1-oracle-follow SEED=123        # live-view compatibility alias
make report-g1-oracle-follow SEED=123      # headless-report compatibility alias
make g1-loco-sandbox POLICY=placeholder
make fetch-lucky-g1-policy
make g1-loco-sandbox POLICY=lucky_walker
make fetch-unitree-rl-gym-policy
make install-torch-cpu
make g1-loco-sandbox POLICY=unitree_rl_gym_g1
make g1-loco-view POLICY=placeholder
```

`make smoke` loads `configs/default.yaml`, prints a short environment/config summary, and writes `runs/visual/smoke_latest.html`. `make test` validates config loading, package imports, runner error handling, maze determinism/solvability, planner/controller behavior, and generated-world XML while writing `runs/visual/test_latest.txt` and `runs/visual/test_latest.html`. `make maze` prints a seeded ASCII maze with the BFS validation path overlaid and writes SVG/ASCII/PGM artifacts. `make world` builds a Lucky G1 MuJoCo maze world XML. `make run` opens a side-by-side visual dashboard first, then launches the live MuJoCo passive viewer in the generated maze world. `make view-run` runs headlessly and opens the same side-by-side dashboard with the final render.

## Milestone 4 — Oracle Path Planner

Milestone 4 is planner-only. It uses the known generated maze grid in explicit oracle/debug mode, calculates a path from start to goal, converts path cells into world-frame waypoints, and saves visual artifacts. It does not command or simulate robot motion.

```bash
make milestone_4 SEED=123
make view-milestone_4 SEED=123
make milestone_4-wide SEED=123
```

Artifacts are written under `runs/visual/`:

```text
runs/visual/milestone_4_seed-123_path.svg
runs/visual/milestone_4_seed-123_path.txt
runs/visual/milestone_4_seed-123_planner_summary.json
```

The summary explicitly records `robot_execution: false` and points to `make milestone_5` as the next step.

## Milestone 5 — Oracle Path Execution

Milestone 5 is the accepted oracle/debug physical-execution milestone. It takes the oracle plan and drives the Lucky G1 walking policy through the maze using the turn-aware waypoint follower and MuJoCo ground-truth pose. This is a development baseline, not sensor-based autonomy.

The runner uses heading-aware A*, pre-turn and post-turn points, forward arc turns, bounded stuck recovery, and honest failure reporting. It never silently replaces the humanoid with a proxy body.

```bash
make milestone_5 SEED=123
make view-milestone_5 SEED=123
make report-milestone_5 SEED=123 MILESTONE_5_DURATION=18 CORRIDOR_WIDTH_M=2.0 MILESTONE_5_LABEL=rightturn
```

Both `make milestone_5` and `make view-milestone_5` open the live MuJoCo simulator. Use `make report-milestone_5` for a fast headless run that opens the HTML dashboard afterward. The older `g1-oracle-follow`, `view-g1-oracle-follow`, and `report-g1-oracle-follow` names remain as compatibility aliases.

Milestone 5 artifacts are written under `runs/visual/`:

```text
runs/visual/g1_oracle_follow_seed-123_dashboard.html
runs/visual/g1_oracle_follow_seed-123_summary.json
runs/visual/g1_oracle_follow_seed-123_topdown_overlay.svg
runs/visual/g1_oracle_follow_seed-123_trajectory.csv
runs/visual/g1_oracle_follow_seed-123_commands.csv
runs/visual/g1_oracle_follow_seed-123_events.jsonl
runs/visual/g1_oracle_follow_seed-123_final.png
runs/visual/g1_oracle_follow_seed-123_policy_compatibility.json
```

The dashboard shows the planned path, actual trajectory, pre-turn points, left/right arc sections, final render, controller state, and event counts. The JSONL log records state changes, turns, recovery attempts, and failures. Reaching and following the oracle route is sufficient for this project's Milestone 5 acceptance.

## Phase 1 — Simulated D435i Camera

Phase 1 mounts a visible D435i-style RGB-D/IMU assembly on the G1 upper torso. It is a MuJoCo-only sensor check: ROS, SLAM, Nav2, and `/scan` are intentionally not included.

```bash
make d435i-visual-check SEED=123
```

The command records the configured camera pose, frame names, resolution, FOV/intrinsics, depth statistics, seed, model, and generated world. It writes:

```text
runs/visual/d435i_mount_seed-123_final.png
runs/visual/d435i_camera_view_seed-123_rgb.png
runs/visual/d435i_camera_view_seed-123_depth.png
runs/visual/d435i_mount_dashboard.html
runs/visual/d435i_mount_summary.json
```

## Phase 2 — ROS 2 MuJoCo Sensor Bridge

Phase 2 publishes the simulated G1 and D435i state through ROS 2 Humble. From the host, the check automatically runs in the existing Docker image; inside a ROS-enabled development container it builds and runs directly. Source changes use the bind mount, so this does not require rebuilding the Docker image.

```bash
make ros-bridge-check SEED=123
```

For a live browser view showing the robot in the maze beside continuously updating RGB and depth feeds:

```bash
make ros-bridge-view SEED=123
```

Then open [http://127.0.0.1:8765/ros_bridge_live.html](http://127.0.0.1:8765/ros_bridge_live.html). Keep the terminal running and press `Ctrl-C` there when finished. Use `ROS_BRIDGE_PORT=8766` if port 8765 is already occupied.

The bridge publishes `/clock`, `/joint_states`, `/imu/data`, RGB and metric `32FC1` depth images, both `CameraInfo` topics, and a connected `map → odom → base_link → torso_link → d435i` TF tree. The headless CPU configuration targets 3 Hz RGB-D, 50 Hz IMU/joints/TF, and 100 Hz clock publication. `/scan`, SLAM, and Nav2 remain outside Phase 2.

Artifacts are written under `runs/visual/`:

```text
ros_bridge_seed-123_topics.txt
ros_bridge_seed-123_topic_rates.txt
ros_bridge_seed-123_tf_frames.svg
ros_bridge_seed-123_rgb.png
ros_bridge_seed-123_depth.png
ros_bridge_seed-123_dashboard.html
ros_bridge_seed-123_summary.json
```

While the check is running, another ROS-enabled terminal can inspect it with:

```bash
ros2 topic list
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /imu/data
ros2 topic echo /camera/depth/camera_info --once
```

## Phase 3 — D435i Depth to LaserScan

Phase 3 converts the metric D435i depth stream into `/scan` with ROS Humble's `depthimage_to_laserscan`. It validates scan rate, range and angle fields, TF connectivity, and alignment against the known maze walls. SLAM, mapping, Nav2, and walking-policy integration are intentionally excluded.

```bash
make d435i-scan-check SEED=123
```

The bounded check produces RGB, depth, scan overlay, topic-rate, dashboard, summary, and headless RViz-equivalent artifacts under `runs/visual/`.

For live inspection with RViz plus a browser showing the robot, RGB, depth, and continuously updating scan overlay:

```bash
make d435i-scan-view SEED=123
```

Then open [http://127.0.0.1:8765/ros_bridge_live.html](http://127.0.0.1:8765/ros_bridge_live.html). Keep the terminal running and press `Ctrl-C` when finished. The live command requires the GUI Docker path for RViz; the bounded check remains fully headless.

Artifacts:

```text
runs/visual/d435i_scan_seed-123_rviz.png
runs/visual/d435i_scan_seed-123_rgb.png
runs/visual/d435i_scan_seed-123_depth.png
runs/visual/d435i_scan_seed-123_scan_overlay.png
runs/visual/d435i_scan_seed-123_topic_rates.txt
runs/visual/d435i_scan_seed-123_dashboard.html
runs/visual/d435i_scan_seed-123_summary.json
```

## Phase 4 — SLAM Toolbox Mapping

Phase 4 runs the Lucky Walker start-to-goal oracle follower while feeding Phase 3 `/scan` into asynchronous `slam_toolbox`. The bridge publishes MuJoCo ground-truth `odom → base_link` as an explicit mapping baseline; `slam_toolbox` exclusively owns `map → odom`. Nav2 and autonomous navigation are not started.

```bash
make slam-map SEED=123
```

The default run allows up to 300 seconds and records a compact rosbag. While it runs, open [http://127.0.0.1:8765/ros_bridge_live.html](http://127.0.0.1:8765/ros_bridge_live.html) to see the moving robot, RGB, depth, scan overlay, and evolving occupancy map.

For the same mapping run with RViz:

```bash
make slam-map-view SEED=123
```

Generated artifacts include `slam_seed-123_map.pgm`, its YAML metadata, dashboard, TF tree, RViz-equivalent preview, summary, and `slam_seed-123_bag/`. Use `SLAM_DURATION=30` for a shorter development run; a short run maps only the beginning of the route.

## Phase 5 — Two-Stage Nav2 Evaluation

Phase 5 first maps the complete oracle start-to-exit route with SLAM, then resets G1 at the entrance, reloads that saved map, and gives both Nav2 and the oracle the final maze goal. Lucky Walker remains the only controller applied to G1; Nav2's `/cmd_vel` is a monitor-only shadow command.

Run the headless two-stage evaluation:

```bash
make nav2-slam-demo SEED=123
```

While it runs, open [http://127.0.0.1:8765/ros_bridge_live.html](http://127.0.0.1:8765/ros_bridge_live.html). The browser contains exactly four live panels: robot in maze, RGB camera, overhead maze view, and Nav2-versus-oracle commands. Stage 1 builds the map; stage 2 reloads it and performs the comparison. Use another `ROS_BRIDGE_PORT` if needed.

For the same evaluation with RViz showing the occupancy map, LaserScan overlay, and depth camera:

```bash
make nav2-slam-view SEED=123
```

Artifacts are written under `runs/visual/`:

```text
nav2_slam_seed-123_rviz.png
nav2_slam_seed-123_costmaps.png
nav2_slam_seed-123_path.svg
nav2_slam_seed-123_cmd_vel.csv
nav2_slam_seed-123_dashboard.html
nav2_slam_seed-123_summary.json
nav2_slam_seed-123_command_comparison.csv
nav2_slam_seed-123_command_comparison.svg
nav2_slam_seed-123_slam_vs_maze.png
nav2_slam_seed-123_trajectory_overlay.svg
```

The report places the final SLAM map beside a same-scale ground-truth maze, plots aligned linear and yaw commands, records MAE/RMSE/correlation/sign agreement, evaluates the Nav2 path, and overlays the actual G1 trajectory with the oracle route and latest Nav2 plan. Each stage ends at the oracle goal, a 600-second cap, or 20 continuous seconds of zero oracle commands. Override these with `NAV2_MAP_MAX_DURATION`, `NAV2_EVAL_MAX_DURATION`, and `NAV2_ZERO_COMMAND_TIMEOUT`; `NAV2_SLAM_DURATION` remains a compatibility override for both duration caps.

## G1 Locomotion Policy Sandbox

This sandbox is for testing locomotion policies. It is separate from maze navigation.

```bash
make g1-loco-sandbox POLICY=placeholder
make fetch-lucky-g1-policy
make g1-loco-sandbox POLICY=lucky_walker
make fetch-unitree-rl-gym-policy
make install-torch-cpu
make g1-loco-sandbox POLICY=unitree_rl_gym_g1
make g1-loco-sandbox POLICY=/path/to/walker.onnx
make g1-loco-view POLICY=/path/to/walker.onnx
```

If `POLICY` is omitted, the Makefile uses `POLICY=placeholder`. Placeholder mode resets the Unitree G1 to the `stand` keyframe and holds the standing control target when available. It does not claim to walk:

```text
No real walking policy loaded. This mode validates viewer, teleop input, recording, and logging only.
```

`POLICY=/path/to/walker.onnx` selects the generic ONNX adapter. `onnxruntime` is installed by `make setup`. The adapter writes a compatibility report and fails clearly if the ONNX file, metadata, observation/action dimensions, actuator count, actuator order, control rate, or action scaling are unknown or mismatched. It does not silently guess G1 joint order.

`POLICY=lucky_walker` uses the pretrained walker from `luckyrobots/g1-manipulation-challenge`. Fetch it locally first:

```bash
make fetch-lucky-g1-policy
make g1-loco-sandbox POLICY=lucky_walker G1_LOCO_DURATION=30
```

The fetched repo is stored under `third_party/g1-manipulation-challenge/` and is ignored by Git. The sandbox writes a generated flat-ground wrapper at `third_party/g1-manipulation-challenge/flat_scene_locomotion_sandbox.xml`, validates the first 29 body actuators by name, loads `walker.onnx`, and applies its 99D observation to 29D joint-target pipeline. For this backend, tap `W` or the up arrow several times to ramp `vx` in visible walking increments; use `Space` or `X` to stop immediately. The upstream repo does not advertise a license in the fetched files, so these policy assets are kept local instead of vendored into this repository.

`POLICY=unitree_rl_gym_g1` uses the official `unitreerobotics/unitree_rl_gym` pretrained G1 policy on the regular Menagerie G1 model. Fetch the repo and install CPU Torch first:

```bash
make fetch-unitree-rl-gym-policy
make install-torch-cpu
make g1-loco-sandbox POLICY=unitree_rl_gym_g1 G1_LOCO_DURATION=60
```

This is an experimental bridge: Unitree's pretrained `motion.pt` is a 47D observation to 12D leg-action TorchScript policy trained/deployed with Unitree's own 12-DoF torque-actuated G1 XML. The sandbox maps those 12 leg targets onto the first 12 regular Menagerie G1 position actuators and holds the upper body at the stand pose. For a closer official deployment comparison, use `POLICY=unitree_rl_gym_native`; that mode loads Unitree's own 12-DoF XML from `third_party/unitree_rl_gym/resources/robots/g1_description/scene.xml`.

`POLICY=module:some.python.module` selects the future external Python policy path. This adapter is documented as a stub until a module contract is added.

Live teleop controls:

```text
Up arrow / W      = desired forward velocity +
Down arrow / Z    = desired forward velocity -
Left arrow / A    = desired yaw left
Right arrow / D   = desired yaw right
Space / X         = zero command / stop motion
Q                 = quit safely
R                 = start recording video frames
S or T            = stop recording
```

`S` is reserved for stop-recording, so `Z` is the fallback backward key. The terminal status shows `vx`, `vy`, `yaw_rate`, recording state, policy, and standing/walking/fallen/error status.

Every sandbox run writes latest artifacts under `runs/visual/`:

```text
runs/visual/g1_loco_latest_dashboard.html
runs/visual/g1_loco_latest_summary.json
runs/visual/g1_loco_latest_commands.csv
runs/visual/g1_loco_latest_state.csv
runs/visual/g1_loco_latest_final_render.png
runs/visual/g1_loco_latest_policy_compatibility.json
```

When recording is started with `R`, the sandbox writes PNG frames under `runs/visual/g1_loco_<timestamp>_frames/` plus `runs/visual/g1_loco_<timestamp>_recording_summary.json`. MP4 export is intentionally optional; if ffmpeg/imageio support is unavailable, the frames are kept and the summary says so.

## How To See Things

All user-facing commands leave inspectable artifacts under `runs/visual/`.

```bash
make view-smoke
make view-test
make view-maze SEED=123 MAZE_CELL_PX=48
make view-world SEED=123
make run SEED=123 DURATION=30
make view-run SEED=123 DURATION=3
```

The `view-*` commands run the underlying command and open the latest artifact with `xdg-open` when `DISPLAY` is available. If no desktop opener is available, the command prints the exact artifact path.

Expected artifacts include:

```text
runs/visual/smoke_latest.html
runs/visual/test_latest.html
runs/visual/test_latest.txt
runs/visual/maze_seed-123.svg
runs/visual/maze_seed-123.txt
runs/visual/maze_seed-123.pgm
runs/visual/world_seed-123.xml
runs/visual/world_seed-123_summary.json
runs/visual/world_seed-123_topdown.svg
runs/visual/run_seed-123_dashboard.html
runs/visual/run_seed-123_preview.png
runs/visual/run_seed-123_summary.json
runs/visual/run_seed-123_final.png
```

To generate and validate an ASCII maze:

```bash
make maze SEED=123
```

To save and view simple visual artifacts:

```bash
make view-maze SEED=123
```

This writes `runs/visual/maze_seed-123.svg`, `runs/visual/maze_seed-123.txt`, and `runs/visual/maze_seed-123.pgm`. If your terminal has `DISPLAY` and `xdg-open`, it also opens the SVG image. Increase `MAZE_CELL_PX` for a larger rendering:

```bash
make view-maze SEED=123 MAZE_CELL_PX=48
```

To try the MuJoCo passive viewer:

```bash
make run SEED=123 DURATION=30
```

`make run` opens `runs/visual/run_seed-123_dashboard.html` first. The dashboard shows the 2D maze representation next to a MuJoCo render of the generated maze world. Then the live MuJoCo viewer opens so you can inspect/travel through the scene with the viewer camera controls. It does not yet drive the robot through the maze.

For a non-realtime inspection run:

```bash
make view-run SEED=123 DURATION=3
```

To build only the generated MuJoCo maze world:

```bash
make world SEED=123
make view-world SEED=123
```

The generated world uses the maze center as the MuJoCo origin:

```text
x = (col - (width_cells - 1) / 2) * cell_size_m
y = ((height_cells - 1) / 2 - row) * cell_size_m
z = 0 is floor height
```

The default `make run` now uses `WORLD=maze`. To run the original empty Unitree scene from Milestone 1:

```bash
make run WORLD=empty SEED=1 DURATION=3
make view WORLD=empty SEED=1 VIEW_DURATION=30
```

## Model Assets

This branch defaults to the Lucky Robot G1 model and walker policy fetched from `luckyrobots/g1-manipulation-challenge`:

```text
third_party/g1-manipulation-challenge/scene.xml
third_party/g1-manipulation-challenge/g1.xml
third_party/g1-manipulation-challenge/walker.onnx
```

The fetched repo is ignored by Git and is not vendored here. Legacy Menagerie paths are still present in `configs/default.yaml` as explicit fallback references:

```text
assets/mujoco_menagerie/unitree_g1/scene.xml
assets/mujoco_menagerie/unitree_g1/g1.xml
```

The generated maze world is written under `runs/visual/` and uses the configured `robot.base_model_xml_path` as its base model.

## Next Milestone

The next step after the accepted Milestone 5 oracle-follow baseline is sensor simulation and timebase work. Keep the boundary clear: simulator ground truth may be used for debugging and evaluation, but final navigation should not secretly depend on privileged simulator state.

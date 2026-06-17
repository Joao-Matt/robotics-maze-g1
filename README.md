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
cd /home/gary/dev_workspaces/robotics-maze-g1
git submodule update --init --recursive
make setup
```

If Python 3.11.15 is missing, install it first:

```bash
env \
  CPPFLAGS=-I/home/gary/.local/openssl/include \
  LDFLAGS=-L/home/gary/.local/openssl/lib \
  PKG_CONFIG_PATH=/home/gary/.local/openssl/lib/pkgconfig \
  LD_LIBRARY_PATH=/home/gary/.local/openssl/lib \
  CONFIGURE_OPTS=--with-openssl=/home/gary/.local/openssl \
  /home/gary/.pyenv/bin/pyenv install 3.11.15
```

On this Ubuntu 20.04 environment, Python 3.11.15 was built against a user-local OpenSSL at `/home/gary/.local/openssl` because interactive `sudo apt` was not available for installing `libssl-dev`. The Makefile sets `LD_LIBRARY_PATH` for repo commands so the venv can use that OpenSSL runtime.

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

`make docker-run` starts a terminal/headless development shell. `make docker-run-gui` starts a GUI-capable shell for RViz, MuJoCo viewer runs, and dashboard opening when the Linux host has X11 display support. Inside either container, the repo is bind-mounted at `/workspace`, so normal host edits appear immediately inside Docker. Rebuild the image only when dependencies change: `docker/Dockerfile`, `requirements.txt`, apt packages, ROS packages, or system libraries.

The Docker run scripts export `VENV=/usr`, so existing Make targets use the Python packages installed in the image instead of requiring a repo-local `.venv` inside the container. The normal host workflow still defaults to `.venv`.

Examples inside the container:

```bash
echo $ROS_DISTRO
ros2 --help
scripts/check_ros_docker_env.sh
make test
make smoke
make view-g1-oracle-follow SEED=123 ORACLE_FOLLOW_DURATION=5
```

GUI examples from a GUI-capable shell:

```bash
rviz2
make milestone_4 SEED=123
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
make g1-oracle-follow SEED=123
make view-g1-oracle-follow SEED=123
make milestone_4 SEED=123
make view-milestone_4 SEED=123
make milestone_4-wide SEED=123
make view-milestone_4-wide SEED=123
make g1-loco-sandbox POLICY=placeholder
make fetch-lucky-g1-policy
make g1-loco-sandbox POLICY=lucky_walker
make fetch-unitree-rl-gym-policy
make install-torch-cpu
make g1-loco-sandbox POLICY=unitree_rl_gym_g1
make g1-loco-view POLICY=placeholder
```

`make smoke` loads `configs/default.yaml`, prints a short environment/config summary, and writes `runs/visual/smoke_latest.html`. `make test` validates config loading, package imports, runner error handling, maze determinism/solvability, planner/controller behavior, and generated-world XML while writing `runs/visual/test_latest.txt` and `runs/visual/test_latest.html`. `make maze` prints a seeded ASCII maze with the BFS validation path overlaid and writes SVG/ASCII/PGM artifacts. `make world` builds a Lucky G1 MuJoCo maze world XML. `make run` opens a side-by-side visual dashboard first, then launches the live MuJoCo passive viewer in the generated maze world. `make view-run` runs headlessly and opens the same side-by-side dashboard with the final render. `make g1-oracle-follow` runs the turn-aware Lucky walker oracle follower. `make milestone_4` runs the earlier Lucky walking policy in explicit oracle mode through maze waypoints.

## Turn-Aware G1 Oracle Follow

This is the visual walking-policy oracle runner for debugging right turns before ROS 2/Nav2/SLAM are introduced. It uses heading-aware A*, inserts pre-turn and post-turn points, starts turns before corners, commands arc turns with forward velocity, and logs recovery attempts instead of silently switching to a proxy.

```bash
make g1-oracle-follow SEED=123
make view-g1-oracle-follow SEED=123
make view-g1-oracle-follow SEED=123 CORRIDOR_WIDTH_M=2.0 ORACLE_FOLLOW_LABEL=wide
```

`make g1-oracle-follow` opens the live MuJoCo viewer. `make view-g1-oracle-follow` runs headlessly and opens the dashboard. The default corridor width is `1.6 m`; use `CORRIDOR_WIDTH_M=1.0` through `2.0` to tune clearance.

Artifacts are written under `runs/visual/`:

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

The dashboard shows planned path, actual trajectory, pre-turn points, left/right arc sections, final render, final status, current/final controller state, and event counts. The JSONL event log records state changes, turn starts, recovery starts/ends, and failure reasons.

## Milestone 4 Oracle Walking

Milestone 4 uses the known generated maze grid as an oracle planner, converts the path to world waypoints, and drives the Lucky G1 walker with conservative velocity commands. It does not use a mocap/proxy body.

```bash
make milestone_4 SEED=123
make view-milestone_4 SEED=123
make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=2.0 MILESTONE_4_LABEL=wide_test
make view-milestone_4-wide SEED=123
```

`make milestone_4` opens the live MuJoCo viewer. `make view-milestone_4` runs headlessly and opens a dashboard. Both targets fetch/refresh the ignored Lucky checkout first. The waypoint controller uses arc turns: when the robot is misaligned, it crawls forward while yawing instead of trying to spin in place.

The default corridor width is `1.6 m`. Override it per command from `1.0 m` to `2.0 m`:

```bash
make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=1.0 MILESTONE_4_LABEL=narrow
make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=1.6
make view-milestone_4 SEED=123 CORRIDOR_WIDTH_M=2.0 MILESTONE_4_LABEL=wide
```

Use the wide shortcut when you want the same maze topology at the maximum supported corridor width:

```bash
make milestone_4-wide SEED=123
make view-milestone_4-wide SEED=123
```

The wide targets use `configs/lucky_wide_maze.yaml` and `WIDE_CORRIDOR_WIDTH_M=2.0`. Wide artifacts use a separate prefix such as `milestone_4_wide_seed-123_*`, so they can be compared against the default run.

Artifacts are written under `runs/visual/`:

```text
runs/visual/milestone_4_seed-123_dashboard.html
runs/visual/milestone_4_seed-123_summary.json
runs/visual/milestone_4_seed-123_world.xml
runs/visual/milestone_4_seed-123_path.svg
runs/visual/milestone_4_seed-123_trajectory.csv
runs/visual/milestone_4_seed-123_final.png
runs/visual/milestone_4_seed-123_policy_compatibility.json
```

Milestone 4 summaries include `contact_summary`, and the trajectory CSV includes `contact_count`, `wall_contact_count`, and `wall_contact_pairs`. `wall_contact_count=0` means the robot did not touch any `maze_wall_*` geoms during sampled control ticks; nonzero entries name the wall and robot body/geom pair.

This mode is honestly labeled oracle/debug: the planner uses the generated maze grid and the controller uses MuJoCo ground-truth base pose. If the walker falls, gets stuck, or times out, the summary reports that status instead of switching to a proxy fallback.

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

The next step after Milestone 4 is sensor simulation and timebase work. Keep the boundary clear: simulator ground truth may be used for debugging and evaluation, but final navigation should not secretly depend on privileged simulator state.

# Robot Improvement Report

Generated on 2026-06-23 from local run artifacts in `runs/`.

## Executive Summary

The robot improved in three measurable ways:

1. The visual/sensor stack became valid and usable: D435i TF, scan shape, topic rates, and wall alignment all passed, with the scan overlay reporting 603/603 valid ranges aligned to maze walls.
2. The robot moved from very short visual smoke runs to long no-ground-truth navigation attempts. Early seed-123 D435i smoke runs had a median distance of 0.043 m. Later held-out no-ground-truth runs had a median distance of 112.38 m and a median best path completion of 68.75%.
3. Locomotion calibration turned command selection from guesswork into a measured safe envelope: 67/87 commands were stable in the balanced sweep, with 0 falls, 0 stuck commands, and recommended limits of 1.4 m/s forward and 2.0 rad/s yaw. The 100-request seed batch evaluated 81 seeds and passed 78 of them.

The main remaining gap is also clear: the no-ground-truth held-out navigation runs still had 0/13 completed goals among the available manifest runs. They made large progress, but ended in collision aborts or timeouts. The PPO controller also showed a strong initial eval result, then failed to generalize cleanly on the fixed corridor sweep.

## Visual Evidence

### D435i Scan Validation

![D435i scan overlay](../runs/visual/d435i_scan_seed-123_scan_overlay.png)

Evidence:

- Summary: [runs/visual/d435i_scan_seed-123_summary.json](../runs/visual/d435i_scan_seed-123_summary.json)
- RGB image: [runs/visual/d435i_scan_seed-123_rgb.png](../runs/visual/d435i_scan_seed-123_rgb.png)
- Depth image: [runs/visual/d435i_scan_seed-123_depth.png](../runs/visual/d435i_scan_seed-123_depth.png)
- RViz capture: [runs/visual/d435i_scan_seed-123_rviz.png](../runs/visual/d435i_scan_seed-123_rviz.png)

Measured result: `tf_valid=true`, `topic_rates_valid=true`, `scan_shape_valid=true`, and `wall_alignment_fraction=1.0`.

### Early Visual Robot Run

![Early robot-maze visual run](../runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/live/robot_maze.png)

Evidence:

- Summary: [runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/summary.json](../runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/summary.json)
- Dashboard: [runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/dashboard.html](../runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/dashboard.html)
- SLAM comparison: [runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/slam_vs_maze.png](../runs/navigate-d435i-smoke/seed-123/20260620T163006.575+0300__corridor-2.0m__duration-20s/slam_vs_maze.png)

This run was still mostly a smoke check: `TIMEOUT`, 0.04 m traveled, and 905 known cells.

### SLAM and Nav2 Visual Validation

![Nav2 SLAM vs maze](../runs/visual/nav2_slam_seed-123_slam_vs_maze.png)

Evidence:

- SLAM summary: [runs/visual/slam_seed-123_summary.json](../runs/visual/slam_seed-123_summary.json)
- Nav2 shadow-control summary: [runs/visual/nav2_slam_seed-123_summary.json](../runs/visual/nav2_slam_seed-123_summary.json)
- RViz capture: [runs/visual/nav2_slam_seed-123_rviz.png](../runs/visual/nav2_slam_seed-123_rviz.png)
- Costmaps: [runs/visual/nav2_slam_seed-123_costmaps.png](../runs/visual/nav2_slam_seed-123_costmaps.png)
- Command comparison: [runs/visual/nav2_slam_seed-123_command_comparison.svg](../runs/visual/nav2_slam_seed-123_command_comparison.svg)

Measured result: the phase-4 SLAM run reached the oracle goal over 111.84 m, produced 24,676 known cells, and had 30.22% map coverage. The phase-5 Nav2 shadow-control run sent and accepted a goal, reached the same oracle-driven motion goal, and aligned 1,219 command samples.

### Held-Out No-Ground-Truth Progress

![Held-out SLAM vs maze near-complete run](../runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/slam_vs_maze.png)

Evidence:

- Summary: [runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/summary.json](../runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/summary.json)
- Dashboard: [runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/dashboard.html](../runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/dashboard.html)
- Batch manifest: [runs/heldout-20/batches/20260621T000619.052__navigate/batch_manifest.json](../runs/heldout-20/batches/20260621T000619.052__navigate/batch_manifest.json)

This run did not complete, but it reached 99.26% best path completion without using ground truth for navigation.

### Calibrated Oracle Locomotion

![Calibrated oracle trajectory](../runs/visual/g1_oracle_follow_calibrated-cap_seed-123_topdown_overlay.svg)

Evidence:

- Summary: [runs/visual/g1_oracle_follow_calibrated-cap_seed-123_summary.json](../runs/visual/g1_oracle_follow_calibrated-cap_seed-123_summary.json)
- Dashboard: [runs/visual/g1_oracle_follow_calibrated-cap_seed-123_dashboard.html](../runs/visual/g1_oracle_follow_calibrated-cap_seed-123_dashboard.html)
- Commands: [runs/visual/g1_oracle_follow_calibrated-cap_seed-123_commands.csv](../runs/visual/g1_oracle_follow_calibrated-cap_seed-123_commands.csv)

Measured result: `GOAL_REACHED`, 44/44 navigation segments completed, 0 recovery attempts, 0 wall-contact steps, and calibrated limits applied from [runs/calibration/g1_locomotion/seed-123/latest/locomotion_calibration.json](../runs/calibration/g1_locomotion/seed-123/latest/locomotion_calibration.json).

## Measurable Improvements

| Area | Earlier evidence | Later evidence | Measured improvement |
|---|---:|---:|---|
| D435i visual/sensor validity | Sensor pipeline still under visual bring-up | D435i scan overlay completed with valid TF, valid rates, valid scan shape, 603/603 wall-aligned ranges | Sensor inputs became measurable and trustworthy for SLAM/Nav2 |
| Seed-123 D435i smoke movement | Median distance 0.043 m across 7 unique early smoke runs | Median distance 4.99 m across 32 unique seed-123 visual iterations | About 116x median distance increase |
| Seed-123 known map cells | Median 917 known cells in early smoke | Median 2,243 known cells in seed-123 visual iterations | About 2.4x median known-cell increase |
| Ground-truth-assisted visual runs | Early max distance 1.03 m | Broader visual/debug runs reached 81.29 m max and 50,894 known cells max | From tiny motion checks to long visual runs |
| No-ground-truth navigation | 2-run clearance smoke median distance 6.98 m and 7.18% median best path completion | 13 available held-out manifest summaries median distance 112.38 m and 68.75% median best path completion | About 16.1x distance and 9.6x best-completion increase, with different durations/cell sizes |
| Oracle path following | `waypointfix_seed-123`: 2/44 segments, failed after max recovery attempts, 902 wall-contact steps | `calibrated-cap_seed-123`: 44/44 segments, goal reached, 0 wall-contact steps | Full route completion and wall-contact elimination in this artifact |
| Locomotion command safety | Manual/static command assumptions | Balanced calibration: 67/87 stable commands, 0 falls, 0 stuck, 2 non-floor contacts | Measured safe command envelope |
| Locomotion seed robustness | Single calibration can overfit one seed | Batch: 78 successes from 81 evaluated seeds, 96.30% pass rate among evaluated seeds | Cross-seed confidence in command envelope |
| PPO velocity controller | No learned high-level controller | First 100-episode PPO eval: 67% success, 0 falls, 0 collisions | Learned controller can solve part of the task distribution |
| PPO generalization | Initial eval looked promising | Fixed corridor sweep dropped to 7% success, later sweeps fell frequently | Measured remaining risk, especially generalization and stability |

## How The Improvement Happened

The project improved by separating the robot problem into measurable layers:

| Layer | What changed | Evidence and files |
|---|---|---|
| Sensor bring-up | D435i mount, RGB/depth, TF, scan conversion, and scan geometry were validated before relying on Nav2 | [scripts/run_d435i_visual_check.py](../scripts/run_d435i_visual_check.py), [sim/d435i_sensor.py](../sim/d435i_sensor.py), [runs/visual/d435i_scan_seed-123_summary.json](../runs/visual/d435i_scan_seed-123_summary.json) |
| SLAM validation | SLAM artifacts compared generated maps against the maze and recorded map coverage, known cells, and scan rate | [runs/visual/slam_seed-123_summary.json](../runs/visual/slam_seed-123_summary.json), [ros_ws/src/g1_mujoco_bridge/g1_mujoco_bridge/slam_artifact_collector.py](../ros_ws/src/g1_mujoco_bridge/g1_mujoco_bridge/slam_artifact_collector.py) |
| Nav2 validation | Nav2 was first tested in shadow/monitor mode before applying commands to the robot | [runs/visual/nav2_slam_seed-123_summary.json](../runs/visual/nav2_slam_seed-123_summary.json), [sim/nav2_motion_session.py](../sim/nav2_motion_session.py) |
| Exploration and fallback behavior | The run names show iterative fixes around scan persistence, side guards, rolling unknown fallback, red-goal fallback/sticky behavior, and m-explore watchdog handling | [ros_ws/src/g1_nav_bringup/g1_nav_bringup/frontier_explorer.py](../ros_ws/src/g1_nav_bringup/g1_nav_bringup/frontier_explorer.py), [ros_ws/src/g1_nav_bringup/g1_nav_bringup/maze_fallback_goal.py](../ros_ws/src/g1_nav_bringup/g1_nav_bringup/maze_fallback_goal.py), [ros_ws/src/g1_nav_bringup/g1_nav_bringup/run_termination.py](../ros_ws/src/g1_nav_bringup/g1_nav_bringup/run_termination.py) |
| Motion limits | Calibration measured which `vx` and yaw-rate commands the Unitree native G1 policy can actually hold | [sim/locomotion_calibration.py](../sim/locomotion_calibration.py), [scripts/run_g1_locomotion_calibration.py](../scripts/run_g1_locomotion_calibration.py), [configs/g1_locomotion_calibration.yaml](../configs/g1_locomotion_calibration.yaml) |
| Applying calibration | Calibration was translated into Nav2/oracle limits instead of hard-coded guesses | [scripts/render_navigation_config.py](../scripts/render_navigation_config.py), [scripts/run_g1_oracle_follow.py](../scripts/run_g1_oracle_follow.py), [runs/visual/g1_oracle_follow_calibrated-cap_seed-123_summary.json](../runs/visual/g1_oracle_follow_calibrated-cap_seed-123_summary.json) |
| PPO RL | A direct MuJoCo velocity controller was trained around oracle path features and wall raycasts | [scripts/train_maze_velocity_policy.py](../scripts/train_maze_velocity_policy.py), [scripts/evaluate_maze_velocity_policy.py](../scripts/evaluate_maze_velocity_policy.py), [rl_velocity/env.py](../rl_velocity/env.py), [configs/rl_velocity_controller.yaml](../configs/rl_velocity_controller.yaml) |

## Training and Calibration Provenance

| Training / calibration | Make target | Main script | Config | Output artifacts |
|---|---|---|---|---|
| Single-seed G1 locomotion calibration | `make locomotion-calibrate SEED=123` | [scripts/run_g1_locomotion_calibration.py](../scripts/run_g1_locomotion_calibration.py) | [configs/g1_locomotion_calibration.yaml](../configs/g1_locomotion_calibration.yaml) | [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/) |
| Multi-seed locomotion calibration batch | `make locomotion-calibrate-batch SEED=123 CALIBRATION_BATCH_COUNT=100` | [scripts/run_g1_locomotion_calibration_seed_batch.py](../scripts/run_g1_locomotion_calibration_seed_batch.py) | [configs/g1_locomotion_calibration.yaml](../configs/g1_locomotion_calibration.yaml) | [runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/](../runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/) |
| PPO velocity controller, seed 123 | `make rl-train SEED=123 RL_TIMESTEPS=200000 RL_NUM_ENVS=1` | [scripts/train_maze_velocity_policy.py](../scripts/train_maze_velocity_policy.py) | [configs/rl_velocity_controller.yaml](../configs/rl_velocity_controller.yaml) | [runs/rl_velocity/train/20260622_112956/](../runs/rl_velocity/train/20260622_112956/) |
| PPO velocity controller, seed 9130 | `make rl-train SEED=9130 RL_TIMESTEPS=200000 RL_NUM_ENVS=1` | [scripts/train_maze_velocity_policy.py](../scripts/train_maze_velocity_policy.py) | [configs/rl_velocity_controller.yaml](../configs/rl_velocity_controller.yaml) | [runs/rl_velocity/train/20260622_192403/](../runs/rl_velocity/train/20260622_192403/) |
| PPO 100-episode evaluation | `make rl-eval ...` | [scripts/evaluate_maze_velocity_policy.py](../scripts/evaluate_maze_velocity_policy.py) | [configs/rl_velocity_controller.yaml](../configs/rl_velocity_controller.yaml) | [runs/rl_velocity/eval/20260622_121953/](../runs/rl_velocity/eval/20260622_121953/) |
| PPO fixed corridor sweep | `make rl-eval-corridor-sweep ...` | [scripts/evaluate_maze_velocity_policy.py](../scripts/evaluate_maze_velocity_policy.py) | [configs/rl_velocity_eval_corridor_sweep_100.yaml](../configs/rl_velocity_eval_corridor_sweep_100.yaml) | [runs/rl_velocity/eval/20260622_133522/](../runs/rl_velocity/eval/20260622_133522/) |

## Calibration Results

Single balanced calibration:

- Summary: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/summary.json](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/summary.json)
- Command CSV: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/command_results.csv](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/command_results.csv)
- Calibration JSON: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/locomotion_calibration.json](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/locomotion_calibration.json)
- Dashboard: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/dashboard.html](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/dashboard.html)

Measured results:

- 67 stable commands out of 87 total: 77.01% stable.
- 0 falls, 0 stuck commands, 2 non-floor contacts.
- Straight commands: 7/10 stable.
- Reverse recovery commands: 9/9 stable.
- Pure rotation commands: 18/20 stable.
- Arc commands: 33/48 stable.
- Recommended safe limits: `max_safe_vx=1.4 m/s`, `max_safe_wz=2.0 rad/s`, turn slowdown starts at `0.7 rad/s`, full slowdown at `1.6 rad/s`.

Multi-seed batch:

- Summary: [runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/summary.json](../runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/summary.json)
- Seed metrics: [runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/seed_metrics.csv](../runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/seed_metrics.csv)
- Dashboard: [runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/dashboard.html](../runs/calibration/g1_locomotion_seed_batch/seed-123/20260622T184810.021+0000__count-100__profile-balanced/dashboard.html)

Measured results:

- 100 seeds requested, 81 evaluated, 78 passed the calibration goal.
- Success rate among evaluated seeds: 96.30%.
- Average stable rate: 76.25%.
- Average max safe forward velocity: 1.40 m/s.
- Average max safe yaw rate: 1.91 rad/s.
- Average time to calibration goal: 84.94 s, median 69.52 s.

## PPO RL Results

Training artifacts:

- Seed 123 model: [runs/rl_velocity/train/20260622_112956/final_model.zip](../runs/rl_velocity/train/20260622_112956/final_model.zip)
- Seed 123 normalization: [runs/rl_velocity/train/20260622_112956/vec_normalize.pkl](../runs/rl_velocity/train/20260622_112956/vec_normalize.pkl)
- Seed 9130 model: [runs/rl_velocity/train/20260622_192403/final_model.zip](../runs/rl_velocity/train/20260622_192403/final_model.zip)
- Seed 9130 normalization: [runs/rl_velocity/train/20260622_192403/vec_normalize.pkl](../runs/rl_velocity/train/20260622_192403/vec_normalize.pkl)

Evaluation summary:

| Eval run | Checkpoint | Episodes | Success rate | Falls | Collisions | Avg goal time | Notes |
|---|---|---:|---:|---:|---:|---:|---|
| [20260622_121953](../runs/rl_velocity/eval/20260622_121953/) | seed-123 final | 100 | 67% | 0 | 0 | 50.85 s | Best initial result |
| [20260622_133522](../runs/rl_velocity/eval/20260622_133522/) | seed-123 final | 100 | 7% | 1 | 0 | 78.61 s | Fixed corridor sweep exposed stuck/timeouts |
| [20260622_153536](../runs/rl_velocity/eval/20260622_153536/) | seed-123 final | 100 | 0% | 100 | 0 | n/a | Unstable/falling run |
| [20260622_203214](../runs/rl_velocity/eval/20260622_203214/) | seed-9130 final | 100 | 0% | 96 | 0 | n/a | Lower jerk than prior unstable runs, but still unsafe |

The useful takeaway is not that PPO is ready. The measurable improvement is that the pipeline can now train, checkpoint, normalize, evaluate, and rank policies. The evidence also shows exactly where the learned policy fails: hard-suite generalization, straight-phase failures, stuck behavior, and falls under some action envelopes.

## Remaining Gaps

- The available held-out manifest summaries show 0/13 completed goals. The robot can now map and traverse much farther without ground-truth navigation, but it still needs better safety margins and termination/fallback behavior before claiming autonomy.
- Several held-out manifest seeds listed in [batch_manifest.json](../runs/heldout-20/batches/20260621T000619.052__navigate/batch_manifest.json) do not have final summary artifacts in the available tree.
- The most positive visual/oracle result uses oracle path following with calibrated command limits, not full SLAM/Nav2 autonomy.
- Calibration uses ground truth for offline measurement only. That is valid for command-envelope estimation, but it is not proof that live navigation localizes perfectly.
- PPO results are currently evidence of infrastructure and partial learning, not a production-ready controller.

## Best Files To Show

- Main visual scan proof: [runs/visual/d435i_scan_seed-123_scan_overlay.png](../runs/visual/d435i_scan_seed-123_scan_overlay.png)
- Main SLAM/Nav2 proof: [runs/visual/nav2_slam_seed-123_slam_vs_maze.png](../runs/visual/nav2_slam_seed-123_slam_vs_maze.png)
- Best no-ground-truth progress proof: [runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/slam_vs_maze.png](../runs/heldout-20/navigate/seed-795378426/20260621T002009.068+0000__cell_size-3m__duration-1000s/slam_vs_maze.png)
- Main calibration proof: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/report.md](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/report.md)
- Main calibration dashboard: [runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/dashboard.html](../runs/calibration/g1_locomotion/seed-123/20260622T182849.008+0000__profile-balanced/dashboard.html)
- Main PPO positive eval: [runs/rl_velocity/eval/20260622_121953/checkpoint_summary.json](../runs/rl_velocity/eval/20260622_121953/checkpoint_summary.json)
- Main PPO hard-suite eval: [runs/rl_velocity/eval/20260622_133522/grouped_summary.json](../runs/rl_velocity/eval/20260622_133522/grouped_summary.json)

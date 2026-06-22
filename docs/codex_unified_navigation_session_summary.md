# Codex Unified Session Summary — Robotics Maze G1 Navigation/Exploration

**Project:** `robotics-maze-g1`  
**Context:** MuJoCo + Unitree/Lucky G1 humanoid maze-navigation assignment  
**Main theme of these sessions:** turn the system from “Nav2/SLAM exists but is hard to diagnose” into a measurable, observable, safer navigation stack with explicit limitations.

---

## 1. One-paragraph executive summary

Across these Codex sessions, the navigation stack was pushed from a D435i/depth/IMU-oriented Phase 6 concept toward a more instrumented Nav2 + SLAM + Lucky Walker exploration pipeline. The work added full ROS bag recording, structured run directories, command-vs-motion analysis, path-progress KPIs, safer Nav2 command limits, reverse motion, backup-based recovery, and a simulated Livox Mid-360 `/scan` source. The biggest engineering finding is that the main failure mode is no longer only “Nav2 cannot see the walls.” The system can publish scan data, build maps, generate commands, and move the humanoid, but the Lucky Walker policy has physical momentum, command dead zones, and body-envelope issues that still cause wall contacts. The next session should focus on fixing the active odometry/localization mode boundary, adding a stronger locomotion-level emergency brake, increasing footprint/body clearance conservatism, and validating with longer 1200-second runs and multi-seed path-progress metrics.

---

## 2. Important source-of-truth warning

There is currently a **mode-definition conflict** that must be resolved before calling Phase 6 “complete.”

### Current README / demo-mode description
The README currently describes Phase 6 as a **Ground-Truth-Pose Livox Mid-360 SLAM Exploration** mode where exact MuJoCo pose intentionally supplies start-relative `/odom` and `odom → base_link`, while Livox point-cloud/raycast data remains the mapping observation. In that mode, ground truth is explicitly used for navigation localization by design.

### Earlier Phase 6 target from the odometry session
The earlier D435i/IMU odometry session targeted the opposite architecture: navigation should use D435i depth + IMU + point-cloud ICP + EKF for `/odom`, while MuJoCo ground truth is used only for `/ground_truth/odom` evaluation.

### Why this matters
Both modes can be valid if they are clearly labeled, but they cannot both be called the same final autonomous result.

Recommended naming:

- `MODE=gt_livox_exploration` or current `navigate-d435i`: current demo/debug mode using ground-truth odom + Livox mapping + Nav2 exploration.
- `MODE=d435i_imu_autonomous` or future mode: intended non-ground-truth odometry pipeline using depth/IMU/ICP/EKF.
- `/ground_truth/odom`: evaluation-only topic in the future autonomous mode.

The next Codex task should first make this distinction explicit in launch files, summaries, README, and dashboard fields.

---

## 3. Session goals consolidated

The combined work had five goals:

1. **Make ROS/Nav2 runs observable**
   - Record full ROS bags.
   - Keep structured run directories.
   - Preserve artifacts even when the run fails.
   - Make missing data produce warnings instead of crashes.

2. **Make the Nav2-to-robot command path easier to trust**
   - Remove downstream transformations after Nav2.
   - Treat `/cmd_vel` from Nav2 as the final command before Lucky Walker.
   - Mirror the applied command as `/applied_cmd_vel` during normal operation.
   - Measure commanded motion versus achieved robot motion.

3. **Adapt Nav2 commands to the real Lucky Walker policy limits**
   - Avoid unsupported low-speed commands.
   - Avoid pure spin recovery.
   - Slow the robot to a measured viable gait.
   - Enable reverse motion and backup recovery.

4. **Improve sensing for SLAM and obstacle awareness**
   - Start with D435i/depth/IMU-style odometry exploration.
   - Add custom point-cloud + IMU odometry and EKF configuration.
   - Later replace the D435i-derived `/scan` with a simulated Livox Mid-360 360-degree LaserScan.
   - Keep SLAM Toolbox and Nav2 wired to `/scan`.

5. **Change evaluation from raw motion to meaningful progress**
   - Raw distance traveled is not enough because the robot can wander a long distance without getting closer to the goal.
   - Add progress along the oracle/ground-truth path as the main KPI.

---

## 4. Chronological narrative of what happened

### 4.1 Phase 6 odometry and structured reporting foundation

Initial goal: build a non-ground-truth navigation/exploration stack using simulated D435i depth, IMU, and an odometry topic suitable for SLAM Toolbox and Nav2.

Implemented or added:

- D435i depth-to-3D-point-cloud generation.
- Custom full-3D point-cloud ICP + IMU + command-prior odometry node.
- EKF configuration through `robot_localization`.
- `/odometry/depth_icp_raw` and `/odometry/d435i_status`.
- Evaluation comparison against MuJoCo ground truth only in reports/tests.
- Structured run directories:

```text
runs/<command>/seed-<seed>/<timestamp>__corridor-Xm__duration-Ys/
```

Important result:

- A good non-ground-truth calibration run achieved roughly:
  - position RMSE: `0.3839 m`
  - position MAE: `0.2558 m`
  - p95 position error: `0.8508 m`
  - final position error: `0.4700 m`
  - yaw RMSE: `0.0079 rad`
  - distance scale: `1.185`

Earlier short run had better localization (`~0.0965 m` RMSE) but failed on odometry freshness.

Main blocker found:

- The active `navigate_d435i.launch.py` later drifted back to a ground-truth odometry launch path:

```python
ground_truth_odom = Node(
    package="g1_nav_bringup",
    executable="ground_truth_odometry",
    name="ground_truth_odometry"
)
```

and configuration included:

```text
ground_truth_navigation_odom: True
```

This violates the intended non-ground-truth Phase 6 rule unless explicitly labeled as a ground-truth demo/debug mode.

---

### 4.2 Nav2 command-path cleanup and Lucky Walker motion characterization

Goal: understand whether the robot is receiving the same command Nav2 emits, and whether the Lucky Walker policy actually performs those commands.

Implemented or changed:

- Full ROS bag recording:

```bash
ros2 bag record --all --include-hidden-topics
```

- Removed downstream `/cmd_vel` transformations.
- Made Nav2 the last command-generation stage before the robot policy.
- Added command/motion metrics to artifacts.
- Characterized the Lucky Walker policy’s usable command range.
- Slowed the robot to a stable command magnitude.
- Enabled reverse motion because the robot can use it.

Important Nav2 motion settings:

```yaml
nav2_navigation:
  max_forward_mps: 0.40
  min_forward_mps: 0.40
  max_reverse_mps: -0.40
  max_yaw_rate_radps: 0.40
  max_linear_accel_mps2: 4.00
  max_linear_decel_mps2: 4.00
```

Important DWB settings:

```yaml
min_vel_x: -0.4
max_vel_x: 0.4
min_speed_xy: 0.4
max_speed_xy: 0.4
max_vel_theta: 0.4
min_speed_theta: 1.1
```

Why this matters:

- The Lucky Walker policy does not reliably execute tiny low-speed commands.
- Low-speed/pure-spin commands can leave the robot in a dead zone or produce bad behavior near walls.
- Nav2 should not command velocities the policy cannot physically realize.

Recovery behavior changed:

```xml
<BackUp backup_dist="0.50" backup_speed="0.40"/>
```

Spin recovery was removed/avoided because pure rotation near walls is unsafe for this humanoid and policy.

Validation result:

- Focused Docker tests passed: `23 passed in 0.63s`.
- Live 40-second validation run:
  - `final_status: TIMEOUT`
  - `distance_traveled_m: 7.56`
  - `wall_contacts: 0`
- Command trace:
  - `nav2 min_vx=-0.400000 max_vx=0.400000`
  - reverse samples: `23`
  - forward samples: `239`
  - pure spin: `0`
  - deadzone: `0`
  - reverse interval: `13.82..16.04 s`, duration `2.22 s`
- Nav2 recovery log showed backup executed successfully:

```text
Running backup
backup completed successfully
```

Interpretation:

- The robot can now execute slower, cleaner, bidirectional commands.
- The 40-second run had zero wall contacts, which is a major safety improvement.
- It still timed out, so this does not prove full-maze completion.

---

### 4.3 Livox Mid-360 replacement for D435i-derived scan

Goal: use the robot’s simulated 360-degree Livox Mid-360 LiDAR instead of a forward D435i camera-derived point cloud for SLAM and obstacle detection.

Implemented or changed:

- Disabled D435 camera rendering during exploration.
- Added simulated Livox Mid-360 scan publishing directly from MuJoCo ray casts.
- Published `/scan` directly from the bridge.
- Removed D435 point-cloud-to-scan nodes from `navigate_d435i.launch.py`.
- Kept SLAM Toolbox and Nav2 consuming `/scan`.
- Updated RViz to show the Livox scan, map, costmaps, frontiers, and plan.

Livox configuration:

```yaml
livox_mid360:
  enabled: true
  frame_id: livox_mid360_frame
  mount_pos_m: [0.05, 0.0, 0.29]
  scan_rate_hz: 10.0
  horizontal_bins: 720
  range_min_m: 0.10
  range_max_m: 40.0
```

Launch-level behavior:

```python
"camera_enabled": False,
"livox_mid360_enabled": True,
"livox_scan_rate_hz": 10.0,
"livox_horizontal_bins": 720,
"livox_range_min_m": 0.10,
"livox_range_max_m": 40.0,
```

Bridge publishing behavior:

```python
self.livox_pub = (
    self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    if self.livox_enabled else None
)
```

SLAM Toolbox configuration:

```yaml
scan_topic: /scan
max_laser_range: 40.0
min_laser_range: 0.11
use_scan_matching: true
do_loop_closing: false
scan_buffer_maximum_scan_distance: 40.0
```

Costmap tuning retained:

```yaml
inflation_radius: 1.0
cost_scaling_factor: 5.0
```

Validation result:

- Focused Docker tests passed: `22 passed in 0.91s`.
- A 45-second run showed Livox active:

```text
publishing seed 123: clock 50 Hz, camera off, Livox Mid-360 on
Registering sensor: [Custom Described Lidar]
```

- Camera topics were disabled in the bag:
  - `/camera/depth/image_rect_raw Count: 0`
  - `/camera/color/image_raw Count: 0`
  - `/scan Count: 272`
- Mapping before collision:
  - `mapping_sensor: simulated_livox_mid360_360_degree_laserscan`
  - coverage fraction: `~0.60`
  - accuracy on known SLAM cells: `~0.954`

Remaining failure:

- The run ended with:

```text
final_status: COLLISION_ABORT
stop_reason: wall_contact_immediate_stop
wall contact time: 26.46 s
```

Interpretation:

- The sensing problem improved: side/rear walls are now visible to the navigation stack.
- The remaining collision is likely not due to missing obstacle sensing.
- The remaining problem is more likely physical locomotion: braking delay, body envelope, recovery behavior, arm/torso/foot swing, or command-to-motion mismatch.

---

### 4.4 Path-progress KPI implementation

Goal: replace raw distance as the main comparison metric with meaningful progress along the ground-truth/oracle path.

Problem solved:

- `distance_traveled_m` alone is misleading.
- A robot can walk 145 m and still not make meaningful progress toward the maze exit.
- The key metric should be: how far along the correct path did the robot get?

Added utility:

```python
def project_point_to_path(point, path):
    # project point onto nearest path segment
    # return distance-along-path, clamped to [0, path_length]
```

Added per-run metrics:

```text
ground_truth_path_length_m
progress_along_ground_truth_path_m
best_progress_along_path_m
final_progress_along_path_m
best_path_completion_fraction
final_path_completion_fraction
remaining_path_distance_m
path_efficiency
path_progress_warning
path_progress_error
```

Added aggregate rollup metrics:

```text
mean_best_path_completion_fraction
median_best_path_completion_fraction
max_best_path_completion_fraction
mean_final_path_completion_fraction
median_final_path_completion_fraction
max_final_path_completion_fraction
mean_path_efficiency
median_path_efficiency
runs_with_path_progress_count
```

Validation result:

- Full test suite passed: `116 passed in 4.00s`.
- Focused path-progress tests passed: `7 passed in 0.48s`.
- Real run analyzer sanity check:

```json
{
  "ground_truth_path_length_m": 112.0,
  "best_path_completion_fraction": 0.764740939302098,
  "final_path_completion_fraction": 0.764740939302098,
  "remaining_path_distance_m": 26.34901479816503,
  "path_efficiency": 1.0537000855482803
}
```

Backfilled 20-run rollup:

```text
runs_with_path_progress_count: 20
mean_best_path_completion_fraction: 0.09598
max_best_path_completion_fraction: 0.92206
mean_path_efficiency: 0.26299
```

Recommended main KPI now:

```text
best_path_completion_fraction
```

Use `path_efficiency` as the supporting metric: useful progress per meter walked.

---

## 5. Files changed across the sessions

### Core configuration and docs

```text
Makefile
README.md
configs/default.yaml
scripts/render_navigation_config.py
```

### ROS bridge, sensors, and odometry

```text
ros_ws/src/g1_mujoco_bridge/g1_mujoco_bridge/bridge_node.py
ros_ws/src/g1_mujoco_bridge/g1_mujoco_bridge/depth_point_cloud.py
ros_ws/src/g1_nav_bringup/g1_nav_bringup/pointcloud_imu_odometry.py
ros_ws/src/g1_nav_bringup/config/depth_imu_ekf.yaml
ros_ws/src/g1_nav_bringup/config/slam_toolbox_ground_truth.yaml
```

### Nav2, SLAM, exploration, and behavior trees

```text
ros_ws/src/g1_nav_bringup/launch/navigate_d435i.launch.py
ros_ws/src/g1_nav_bringup/config/nav2_exploration_params.yaml
ros_ws/src/g1_nav_bringup/behavior_trees/navigate_to_pose_no_spin.xml
ros_ws/src/g1_nav_bringup/behavior_trees/navigate_through_poses_no_spin.xml
ros_ws/src/g1_nav_bringup/rviz/nav2_slam.rviz
ros_ws/src/g1_nav_bringup/g1_nav_bringup/frontier_explorer.py
ros_ws/src/g1_nav_bringup/g1_nav_bringup/exploration_reporter.py
```

### Run structure and navigation session support

```text
sim/world_builder.py
sim/run_context.py
sim/nav2_motion_session.py
scripts/create_run_context.py
scripts/finalize_run_context.py
scripts/characterize_nav_locomotion.py
scripts/run_parallel_navigation.py
scripts/navigation_metrics.py
```

### Path-progress metrics

```text
nav/path_progress.py
scripts/navigation_metrics.py
ros_ws/src/g1_nav_bringup/g1_nav_bringup/exploration_reporter.py
```

### Tests

```text
tests/test_depth_point_cloud.py
tests/test_pointcloud_odometry.py
tests/test_nav2_navigation.py
tests/test_ros_bridge_support.py
tests/test_run_context.py
tests/test_path_progress.py
tests/test_navigation_metrics.py
```

---

## 6. Validation evidence collected

| Area | Evidence |
|---|---|
| D435i/IMU odometry foundation | Earlier full suite reached `103 passed` before later launch drift. |
| Command-path cleanup | Focused Docker tests: `23 passed in 0.63s`. |
| Livox replacement | Focused Docker tests: `22 passed in 0.91s`. |
| Path-progress metrics | Full suite: `116 passed in 4.00s`; focused path tests: `7 passed in 0.48s`. |
| 40-second motion validation | `TIMEOUT`, `7.56 m`, `0` wall contacts, reverse verified, no pure spin/deadzone. |
| Livox mapping run | `/scan` active, camera topics disabled, mapping coverage around `0.60`, map accuracy around `0.954`, but collision abort at `26.46 s`. |
| Batch KPI backfill | 20-run rollup had path-progress metrics, max best completion around `0.922`. |

---

## 7. What works now

### Observability and artifacts

- Full ROS bags can be recorded for navigation runs.
- Runs write structured directories under `runs/navigate-d435i/seed-.../`.
- Summaries, dashboards, rosbag logs, maps, trajectories, command traces, and run manifests exist.
- Missing optional data can be represented as warnings instead of fatal crashes.

### Command path

- `/cmd_vel` from Nav2 is no longer transformed downstream before the robot policy.
- `/applied_cmd_vel` can mirror the command applied to Lucky Walker.
- Commanded versus achieved motion can be measured.
- Reverse motion is enabled.
- Pure spin and dead-zone commands are excluded.

### Nav2 behavior

- DWB is constrained to the measured viable Lucky Walker gait.
- Backup recovery exists and can execute.
- Spin recovery is removed/avoided.
- Velocity limits are conservative: approximately `±0.40 m/s` and `0.40 rad/s` yaw.

### Sensing and mapping

- Simulated Livox Mid-360 `/scan` is implemented.
- D435 camera rendering can be disabled for exploration.
- SLAM Toolbox consumes `/scan`.
- Nav2 costmaps consume `/scan`.
- RViz can show scan, map, costmaps, frontiers, plan, and trajectory.
- Scan matching is enabled.

### Metrics

- Path-progress metrics exist at per-run and aggregate levels.
- The main comparison metric should be `best_path_completion_fraction`, not raw distance.
- `path_efficiency` gives useful progress per meter walked.

---

## 8. What still does not work or still needs caution

### 8.1 The current localization story is ambiguous

The project currently contains both:

- a ground-truth-odometry Livox exploration mode, and
- an intended D435i/IMU non-ground-truth odometry mode.

This must be cleaned up in names, configs, dashboards, and README before presentation.

### 8.2 Full-maze completion is not proven

- The 40-second run had zero wall contacts but ended by timeout.
- The Livox run mapped well but still collided around `26.46 s`.
- Longer `1200 s` runs are still needed.

### 8.3 Collision is probably physical, not only perceptual

The Livox sensor sees around the robot, and the map/costmap can consume that data. If the robot still collides, likely causes are:

- Lucky Walker physical momentum after commands drop to zero.
- Humanoid body envelope not represented conservatively enough in costmaps.
- Arms/hands/feet extending outside the circular footprint.
- Backup/recovery near walls still physically unsafe.
- Nav2 path or controller assumes a differential-drive robot more than a humanoid gait policy.
- Need for locomotion-level brake/disarm behavior, not just command zeroing.

### 8.4 Current `navigate_d435i.launch.py` must be audited

The launch file was reported to contain ground-truth odometry and static map/odom logic that violates the intended “SLAM owns `map → odom`” design for non-ground-truth mode.

Required target for non-ground-truth mode:

```text
D435i depth + IMU
  -> 3D point-cloud ICP odometry
  -> robot_localization EKF
  -> /odom + odom->base_link
  -> SLAM Toolbox owns map->odom
  -> Nav2 consumes /map, /scan, /odom
```

Ground truth should only be:

```text
/ground_truth/odom
```

for scoring/reporting.

---

## 9. Recommended next session: exact priority order

### Priority 1 — Decide and encode mode names

Do this before tuning anything else.

Recommended modes:

```text
gt_livox_exploration       # current working/demo mode; ground-truth odom is allowed and explicit
d435i_imu_autonomous       # future non-ground-truth mode; D435i/IMU/ICP/EKF odom
eval_only_ground_truth     # reporting/scoring only
```

Every summary/dashboard should include:

```json
{
  "ground_truth_used_for_navigation": true_or_false,
  "odom_source": "ground_truth" | "d435i_imu_ekf" | "other",
  "mapping_sensor": "simulated_livox_mid360_360_degree_laserscan" | "d435i_depth_scan",
  "slam_owns_map_to_odom": true_or_false
}
```

### Priority 2 — Add locomotion-level emergency stop/brake

Current command zeroing may not stop the physical body quickly enough.

Codex should inspect Lucky Walker adapter and add a clear emergency behavior:

- immediate command zero
- policy action hold or neutralization
- optional increased damping / safe pose target if available
- disarm on wall contact, unsafe path, fall, or watchdog
- report brake reason and braking distance/time

Acceptance evidence:

- artificial stop tests show reduced stopping distance.
- a near-wall stop does not continue drifting into the wall.
- dashboard reports braking delay.

### Priority 3 — Increase humanoid footprint conservatism

The robot is not a small circular base.

Try:

- larger circular radius, maybe `0.70–0.85 m` for safety tests.
- polygon footprint approximating shoulders/arms/feet if Nav2 supports it cleanly.
- larger near-wall penalty or no-go buffer near frontiers.
- special recovery rule: do not back up or arc near walls unless clearance is high.

### Priority 4 — Run longer tests

After the brake and footprint changes:

```bash
NAVIGATE_DURATION=1200 make navigate-d435i SEED=123
```

Then inspect:

```text
runs/navigate-d435i/seed-123/latest/summary.json
runs/navigate-d435i/seed-123/latest/dashboard.html
runs/navigate-d435i/seed-123/latest/rosbag/
```

Key fields:

```text
final_status
stop_reason
wall_contacts
best_path_completion_fraction
final_path_completion_fraction
path_efficiency
coverage_fraction
command_deadzone_count
pure_spin_count
reverse_samples
brake_events
```

### Priority 5 — Multi-seed path-progress validation

Run a small batch only after one seed is stable:

```bash
make navigate-d435i-random-parallel CORRIDOR_WIDTH=2.0 NAVIGATE_PARALLEL_COUNT=5 NAVIGATE_PARALLEL_JOBS=2
```

Judge by:

```text
mean_best_path_completion_fraction
max_best_path_completion_fraction
mean_path_efficiency
collision_abort_count
timeout_count
runs_with_path_progress_count
```

---

## 10. Suggested next Codex prompt

Use this prompt as the next handoff.

```text
You are working in my robotics-maze-g1 repository. Read the current README, worklog, this unified session summary, and the latest navigate-d435i run artifacts before editing.

Goal for this session: cleanly separate navigation modes and fix the immediate collision-risk failure mode.

Important context:
- Recent work added ROS bag recording, unchanged Nav2-to-Lucky /cmd_vel passthrough, command/motion metrics, reverse motion, backup recovery, no-spin behavior trees, Livox Mid-360 /scan, scan matching, and path-progress KPIs.
- A 40-second validation run had zero wall contacts but timed out.
- A Livox run mapped correctly but still ended with COLLISION_ABORT at about 26.46 s.
- The remaining issue is likely locomotion braking/body-envelope/recovery behavior, not missing side/rear sensing.
- There is also a mode conflict: current README describes ground-truth-odometry Livox exploration, while earlier Phase 6 target wanted D435i/IMU/ICP/EKF odometry with ground truth evaluation only.

First, inspect:
- git status --short
- README.md
- docs/worklog.md
- configs/default.yaml
- ros_ws/src/g1_nav_bringup/launch/navigate_d435i.launch.py
- ros_ws/src/g1_nav_bringup/config/nav2_exploration_params.yaml
- ros_ws/src/g1_nav_bringup/behavior_trees/*.xml
- ros_ws/src/g1_mujoco_bridge/g1_mujoco_bridge/bridge_node.py
- latest runs/navigate-d435i/seed-*/latest/summary.json if present

Do not edit yet. Produce a plan with:
1. How you will label/split ground-truth Livox exploration vs non-ground-truth D435i/IMU autonomous odometry.
2. Exactly which files need changes.
3. How you will add a locomotion-level emergency brake or stronger stop behavior.
4. How you will make the robot footprint/body envelope more conservative.
5. Validation commands and expected artifact fields.
6. Risks.

After I approve, implement only this scope:
- mode naming/reporting cleanup,
- emergency stop/brake behavior,
- safer footprint/recovery configuration,
- focused tests,
- one 45-second validation run.

Do not remove the existing working Livox mode. Do not silently claim autonomous non-ground-truth navigation unless Nav2/SLAM truly consume non-ground-truth odometry. Ground truth may be used in explicitly named debug/demo mode and in evaluation-only topics.
```

---

## 11. Short interview/demo explanation you can use

> At this stage I had moved from simple oracle walking into a ROS 2 navigation stack with SLAM, Nav2, a simulated 360-degree Livox scan, rosbag recording, and structured KPI reports. A major lesson was that making Nav2 “publish commands” is not enough for a humanoid. The walking policy has a real command envelope: tiny velocities and pure spins are unsafe or ineffective. I therefore measured the policy behavior, constrained Nav2 to viable ±0.40 m/s motion, removed spin recovery, enabled backup/reverse, and added command-vs-motion metrics. I also changed the evaluation metric from raw distance to progress along the ground-truth path, because walking far is not the same as solving the maze. The current remaining failure is physical: the robot can still collide because of body envelope and braking behavior even when the LiDAR sees the obstacle. My next engineering step would be a stronger locomotion-level brake, a more conservative humanoid footprint, and longer multi-seed validation.

---

## 12. What to put in the worklog after the next successful session

Use this template:

```markdown
## 2026-06-XX — Phase 6 navigation cleanup: mode boundary, braking, and collision risk

### Goal
Separate ground-truth Livox demo mode from future non-ground-truth D435i/IMU odometry mode, then reduce wall-contact risk by improving braking and humanoid footprint conservatism.

### Changes made
- ...

### Key decisions
- [GROUND_TRUTH_BOUNDARY] Decision: ...
- [LOCOMOTION] Decision: ...
- [DEMO_RISK] Decision: ...

### Validation performed
- Commands run:
- Tests passed/failed:
- Run artifact paths:
- Summary fields checked:

### Problems encountered
- Problem:
  - Symptom:
  - Suspected cause:
  - Fix/mitigation:
  - Remaining risk:

### Result
What works now, and what still does not.

### Next actions
Run longer 1200-second single-seed test, then multi-seed path-progress evaluation.
```

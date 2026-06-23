# Navigation Data Capture Schema

`make navigate-record` creates a dataset run with a documented schema, per-message timestamps, and repairable sidecar files. The normal `make navigate` target still records every ROS topic for broad debugging.

## What "Documented Schema" Means

The dataset schema has two parts:

- Human-readable contract: this document.
- Machine-readable contract: `configs/navigation_capture_topics.yaml`.

Each dataset run copies the schema into the run directory as `capture_schema.yaml` and writes `capture_manifest.json`, `capture_validation.json`, and `capture_samples.csv`.

## Recorded Topics

The dataset bag records an explicit allowlist instead of `--all`.

| Area | Topics |
| --- | --- |
| RGB-D | `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_rect_raw`, `/camera/depth/camera_info` |
| IMU, joints, base | `/imu/data`, `/joint_states`, `/odom`, `/tf`, `/tf_static` |
| Planning and map | `/plan`, `/received_global_plan`, `/local_plan`, `/map`, `/map_metadata`, `/map_updates` |
| Ground truth | `/ground_truth/odom`, `/ground_truth/achieved_velocity`, `/ground_truth/trajectory` |
| Timing and control | `/clock`, `/cmd_vel`, `/applied_cmd_vel`, `/navigation/status`, `/exploration/status`, `/odometry/d435i_status` |

Ground truth topics are evaluation-only. They are recorded for scoring, replay, and debugging, but they must not be used as Nav2 odometry or as a live planning input.

## Timestamp Semantics

Every row in `capture_samples.csv` has:

- `topic`: ROS topic name.
- `bag_time_ns`: rosbag message timestamp in nanoseconds. This is the primary sample timestamp and exists for every recorded message.
- `header_stamp_ns`: `msg.header.stamp` in nanoseconds when the message has a header and can be deserialized.
- `frame_id`: `msg.header.frame_id` when available.
- `message_type`: ROS message type.
- `bag_file`: split SQLite bag file that contains the message.

Use `/clock` as the simulation-time reference for replay/alignment. Headerless topics such as `/cmd_vel` use `bag_time_ns` as their sample time.

## Crash Safety

`make navigate-record` uses SQLite rosbag storage with 512 MB split files by default. On `Ctrl+C`, the Makefile sends SIGINT to rosbag so metadata and sidecars can finish cleanly. On `kill -9` or power loss, the active split may lose tail data, but closed split files should remain readable.

Repair a stale run with:

```bash
make repair-run RUN_DIR=runs/navigate-record/seed-123/<timestamp>
```

Repair validates SQLite integrity, reindexes if metadata is missing and ROS tooling is available, rebuilds `capture_validation.json`, rebuilds `capture_samples.csv`, and marks stale `RECORDING` manifests as `CRASHED_OR_INTERRUPTED`.

## Output Layout

Typical dataset run:

```text
runs/navigate-record/seed-123/<timestamp>/
  capture_schema.yaml
  capture_manifest.json
  capture_validation.json
  capture_samples.csv
  rosbag/
    metadata.yaml
    rosbag_0.db3
    rosbag_1.db3
  run_manifest.json
  resolved_config.yaml
  resolved_nav2_params.yaml
  summary.json
  dashboard.html
  live_dashboard/
```

The schema file is copied into each run so old runs remain interpretable even if the repository schema changes later.

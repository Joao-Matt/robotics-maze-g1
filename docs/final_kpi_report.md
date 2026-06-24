# Final KPI Report

Generated on 2026-06-24 for the current navigation stack. This report uses autonomous navigation runs only; oracle runs are not used to claim navigation success.

## Verdict

Would I trust this robot unsupervised? No, not yet.

The current stack produces complete telemetry and evaluation artifacts, but it did not solve the held-out navigation task. Autonomous solve remains future work for a later submission slice.

## Held-Out KPI Evidence

Artifacts:

- [heldout_summary.html](../runs/heldout-20-current/heldout_summary.html)
- [heldout_summary.json](../runs/heldout-20-current/heldout_summary.json)
- [heldout_summary.csv](../runs/heldout-20-current/heldout_summary.csv)

Result:

- Label: `held_out_current`
- Seeds: 20 expected, 20 found, 0 missing
- Solve rate: 0/20 = 0.0%
- 95% Wilson CI: 0.0% to 16.1%
- Verdict: No, held-out reliability or safety is not strong enough to trust the robot unsupervised.
- Dominant failure mode: collision/contact, reported as `hit` for 20/20 runs.
- Safety/motion: clean motion rate 0.0%, 27 total wall contacts, 157 recovery events.
- Localization: median ATE RMSE 0.634 m, p95 ATE RMSE 1.211 m, median final error 0.764 m.
- Mapping: median coverage 7.94%, p05 coverage 3.38%, median known cells 3166.5.
- Data quality/runtime: median realtime factor 0.988, minimum realtime factor 0.979, schema completeness minimum 1.0.

## Seen/Development Comparison

Artifacts:

- [seen_summary.html](../runs/seen-5-current/seen_summary.html)
- [seen_summary.json](../runs/seen-5-current/seen_summary.json)
- [seen_summary.csv](../runs/seen-5-current/seen_summary.csv)

Result:

- Label: `seen_current`
- Seeds: 5 expected, 5 found, 0 missing
- Solve rate: 0/5 = 0.0%
- 95% Wilson CI: 0.0% to 43.4%
- Seen-vs-held-out solve-rate gap: 0.0 percentage points.

This comparison shows no hidden tuned-set success; both seen and held-out batches currently fail by collision/contact.

## Clean Dataset Artifact

Official dataset run:

- [navigate-record seed 123 dataset run](../runs/navigate-record/seed-123/20260624T085454.964+0000__cell_size-2.0m__duration-300s/)
- [capture_validation.json](../runs/navigate-record/seed-123/20260624T085454.964+0000__cell_size-2.0m__duration-300s/capture_validation.json)
- [capture_manifest.json](../runs/navigate-record/seed-123/20260624T085454.964+0000__cell_size-2.0m__duration-300s/capture_manifest.json)
- [rosbag metadata](../runs/navigate-record/seed-123/20260624T085454.964+0000__cell_size-2.0m__duration-300s/rosbag/metadata.yaml)

Dataset validation:

- `capture_manifest.json` status: `FINALIZED`
- `capture_validation.json` ok: `true`
- `capture_samples.csv` rows: 23528, matching the bag message count.
- Split SQLite bag present: `rosbag/rosbag_0.db3`
- Required sidecars present: `run_manifest.json`, `resolved_config.yaml`, `resolved_nav2_params.yaml`, `rosbag/metadata.yaml`
- Required streams have nonzero messages, including RGB-D, IMU, joint states, odom, map, command, applied command, and ground-truth evaluation topics.
- Ground truth is documented and used as `evaluation_and_comparison_only`; it is not used for navigation.

The dataset run itself ended with `COLLISION_ABORT`, so it is not evidence of autonomous solve. It is evidence of a schema-valid dataset capture.

## Debug/Full-View Evidence

Current live/debug run:

- [navigate seed 8890 debug run](../runs/navigate/seed-8890/20260624T084038.183+0000__cell_size-2.0m__duration-1200s/)
- [dashboard.html](../runs/navigate/seed-8890/20260624T084038.183+0000__cell_size-2.0m__duration-1200s/dashboard.html)

This run is debug evidence only:

- Final status: `COLLISION_ABORT`
- Goal reached: false
- `rosbag/metadata.yaml` is absent.
- `capture_validation.json` is absent.
- Before old debug bags were pruned, RGB-D camera topics were observed with zero messages.

Do not use this full-view/debug rosbag as the assignment dataset artifact. The dataset source of truth is `make navigate-record`, because it enables the schema allowlist, RGB-D capture, split SQLite bags, sidecars, validation, and repair flow.

## Related Work

- [robot_improvement_report.md](robot_improvement_report.md)

The current submission slice is complete for KPI evidence and dataset packaging, but not for autonomous solve.

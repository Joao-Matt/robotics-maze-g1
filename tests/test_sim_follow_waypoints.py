from scripts.sim_follow_waypoints import _artifact_paths, write_dashboard


def test_sim_follow_dashboard_includes_required_status_and_links(tmp_path):
    paths = _artifact_paths(tmp_path, seed=123)
    for path in paths.values():
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix == ".png":
            path.write_bytes(b"\x89PNG\r\n\x1a\n")
        else:
            path.write_text("placeholder\n", encoding="utf-8")

    summary = {
        "seed": 123,
        "mode": "proxy_waypoint_follow",
        "final_status": "GOAL_REACHED",
        "g1_locomotion_implemented": False,
        "waypoints_reached": 61,
        "duration_s": 12.5,
        "rerun_live_command": "make sim-follow SEED=123",
    }

    write_dashboard(paths, summary)

    rendered = paths["dashboard"].read_text(encoding="utf-8")
    assert "proxy_waypoint_follow" in rendered
    assert "GOAL_REACHED" in rendered
    assert "G1 locomotion implemented" in rendered
    assert "sim_follow_seed-123_trajectory.csv" in rendered

import pytest

from ros_ws.src.g1_nav_bringup.g1_nav_bringup.nav2_probe import align_commands, command_metrics


def test_command_alignment_uses_shared_grid_and_fresh_samples():
    oracle = [(0.0, 0.2, 0.1), (0.1, 0.4, -0.1), (0.2, 0.6, -0.2)]
    nav2 = [(0.0, 0.1, 0.0), (0.1, 0.3, -0.2), (0.2, 0.5, -0.1)]
    rows = align_commands(oracle, nav2, start=0.0, rate_hz=10.0)
    assert len(rows) == 3
    assert rows[1] == pytest.approx((0.1, 0.4, -0.1, 0.3, -0.2))


def test_command_metrics_report_error_correlation_and_sign_agreement():
    rows = [(0.0, 0.2, 0.1, 0.1, 0.2), (0.1, 0.4, -0.1, 0.3, -0.2), (0.2, 0.6, -0.2, 0.5, -0.1)]
    metrics = command_metrics(rows)
    assert metrics["aligned_sample_count"] == 3
    assert metrics["linear_x"]["mae"] == pytest.approx(0.1)
    assert metrics["linear_x"]["rmse"] == pytest.approx(0.1)
    assert metrics["linear_x"]["correlation"] == pytest.approx(1.0)
    assert metrics["angular_z"]["sign_agreement"] == pytest.approx(1.0)

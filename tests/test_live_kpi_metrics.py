from __future__ import annotations

from pathlib import Path
import math
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws" / "src" / "g1_nav_bringup"))

from g1_nav_bringup.live_kpi_metrics import (  # noqa: E402
    command_smoothness,
    drop_fraction,
    localization_metrics,
    message_rate,
    occupancy_stats,
    scan_clearance,
)


class LiveKpiMetricsTest(unittest.TestCase):
    def test_rate_and_drop_fraction(self) -> None:
        self.assertAlmostEqual(message_rate([0.0, 0.5, 1.0], now=1.0, window_s=2.0), 2.0)
        self.assertAlmostEqual(drop_fraction(8.0, 10.0), 0.2)
        self.assertIsNone(drop_fraction(8.0, None))

    def test_occupancy_and_scan_stats(self) -> None:
        self.assertEqual(occupancy_stats([-1, 0, 0, 100])["coverage_fraction"], 0.75)
        clearance = scan_clearance([math.inf, 0.2, 2.5, 50.0], 0.1, 10.0)
        self.assertEqual(clearance["valid_ranges"], 2)
        self.assertEqual(clearance["invalid_ranges"], 2)
        self.assertAlmostEqual(clearance["min_clearance_m"], 0.2)

    def test_localization_metrics_align_initial_pose(self) -> None:
        odom = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
        truth = [(0.0, 10.0, 5.0, 0.0), (1.0, 11.0, 5.0, 0.0)]
        metrics = localization_metrics(odom, truth)
        self.assertTrue(metrics["available"])
        self.assertEqual(metrics["aligned_samples"], 2)
        self.assertAlmostEqual(metrics["position_rmse_m"], 0.0)

    def test_command_smoothness(self) -> None:
        metrics = command_smoothness([(0.0, 0.0, 0.0), (1.0, 1.0, 0.5), (2.0, 1.0, 0.0)])
        self.assertIsNotNone(metrics["linear_accel_rms_mps2"])
        self.assertIsNotNone(metrics["yaw_accel_rms_radps2"])


if __name__ == "__main__":
    unittest.main()

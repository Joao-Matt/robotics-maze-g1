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
    free_space_coverage_stats,
    localization_metrics,
    message_rate,
    occupancy_stats,
    projected_free_space_coverage_stats,
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

    def test_free_space_coverage_excludes_truth_walls(self) -> None:
        stats = free_space_coverage_stats(
            slam_data=[-1, 0, 100, 0, 100, -1],
            truth_data=[0, 0, 0, 100, 100, -1],
        )

        self.assertEqual(stats["coverage_scope"], "ground_truth_free_cells")
        self.assertEqual(stats["truth_free_cells"], 3)
        self.assertEqual(stats["known_free_space_cells"], 2)
        self.assertAlmostEqual(stats["coverage_fraction"], 2 / 3)
        self.assertEqual(stats["truth_wall_cells"], 2)
        self.assertEqual(stats["known_truth_wall_cells"], 2)
        self.assertEqual(stats["false_obstacle_in_free_space_cells"], 1)

    def test_free_space_coverage_requires_matching_grid_size(self) -> None:
        with self.assertRaises(ValueError):
            free_space_coverage_stats([0], [0, 0])

    def test_projected_free_space_coverage_uses_fixed_truth_denominator(self) -> None:
        truth = [
            0, 0,
            0, 100,
        ]
        first = projected_free_space_coverage_stats(
            [0],
            slam_width=1,
            slam_height=1,
            slam_origin_x=0.0,
            slam_origin_y=0.0,
            truth_data=truth,
            truth_width=2,
            truth_height=2,
            truth_origin_x=0.0,
            truth_origin_y=0.0,
            resolution=1.0,
        )
        expanded = projected_free_space_coverage_stats(
            [0, -1, -1, 100],
            slam_width=2,
            slam_height=2,
            slam_origin_x=0.0,
            slam_origin_y=0.0,
            truth_data=truth,
            truth_width=2,
            truth_height=2,
            truth_origin_x=0.0,
            truth_origin_y=0.0,
            resolution=1.0,
        )

        self.assertEqual(first["coverage_scope"], "full_ground_truth_free_cells")
        self.assertEqual(first["truth_free_cells"], 3)
        self.assertAlmostEqual(first["coverage_fraction"], 1 / 3)
        self.assertAlmostEqual(expanded["coverage_fraction"], 1 / 3)

    def test_localization_metrics_align_initial_pose(self) -> None:
        odom = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
        truth = [(0.0, 10.0, 5.0, 0.0), (1.0, 11.0, 5.0, 0.0)]
        metrics = localization_metrics(odom, truth)
        self.assertTrue(metrics["available"])
        self.assertEqual(metrics["aligned_samples"], 2)
        self.assertAlmostEqual(metrics["position_rmse_m"], 0.0)
        self.assertAlmostEqual(metrics["final_position_error_per_meter"], 0.0)
        self.assertAlmostEqual(metrics["yaw_rmse_deg"], 0.0)

    def test_localization_metrics_report_drift_per_meter_and_jumps(self) -> None:
        odom = [(0.0, 0.0, 0.0, 0.0), (0.1, 1.1, 0.0, 0.0), (1.0, 1.1, 0.0, 0.0)]
        truth = [(0.0, 0.0, 0.0, 0.0), (0.1, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
        metrics = localization_metrics(odom, truth)
        self.assertTrue(metrics["available"])
        self.assertAlmostEqual(metrics["final_position_error_per_meter"], 0.1)
        self.assertGreaterEqual(metrics["sudden_translation_jump_count"], 1)

    def test_localization_metrics_estimate_time_offset(self) -> None:
        truth = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 2.0, 0.0, 0.0), (3.0, 3.0, 0.0, 0.0)]
        odom = [(1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 0.0, 0.0), (3.0, 2.0, 0.0, 0.0)]
        metrics = localization_metrics(odom, truth, latency_search_s=1.0, latency_step_s=0.5)
        self.assertLess(metrics["estimated_time_offset_s"], 0.0)
        self.assertLess(metrics["latency_corrected_position_rmse_m"], metrics["position_rmse_m"])

    def test_command_smoothness(self) -> None:
        metrics = command_smoothness([(0.0, 0.0, 0.0), (1.0, 1.0, 0.5), (2.0, 1.0, 0.0)])
        self.assertIsNotNone(metrics["linear_accel_rms_mps2"])
        self.assertIsNotNone(metrics["yaw_accel_rms_radps2"])


if __name__ == "__main__":
    unittest.main()

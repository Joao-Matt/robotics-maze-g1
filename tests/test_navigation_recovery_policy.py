from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros_ws" / "src" / "g1_nav_bringup"))

from g1_nav_bringup.corridor_recenter import select_corridor_center_goal  # noqa: E402
from g1_nav_bringup.navigation_evaluation import frontier_goal, occupied_clearance_cells  # noqa: E402
from g1_nav_bringup.run_termination import (  # noqa: E402
    is_run_success_status,
    is_terminal_run_status,
    parse_duration_s,
    run_duration_s,
)


class RunTerminationPolicyTest(unittest.TestCase):
    def test_m_explore_complete_is_not_terminal(self) -> None:
        self.assertFalse(is_terminal_run_status("EXPLORATION_COMPLETE"))
        self.assertFalse(is_terminal_run_status("RETURNED_TO_ORIGIN"))

    def test_goal_collision_timeout_and_stuck_are_terminal(self) -> None:
        for status in ("GOAL_REACHED", "COLLISION_ABORT", "TIMEOUT", "STUCK"):
            with self.subTest(status=status):
                self.assertTrue(is_terminal_run_status(status))

    def test_only_goal_is_success(self) -> None:
        self.assertTrue(is_run_success_status("GOAL_REACHED"))
        self.assertFalse(is_run_success_status("TIMEOUT"))

    def test_duration_parser_accepts_numeric_and_seconds_suffix(self) -> None:
        self.assertEqual(parse_duration_s(900), 900.0)
        self.assertEqual(parse_duration_s("900s"), 900.0)
        self.assertEqual(parse_duration_s("12.5 seconds"), 12.5)
        self.assertIsNone(parse_duration_s("soon"))

    def test_run_duration_prefers_seed_run_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "run_manifest.json").write_text(
                '{"parameters":{"duration":"900s"}}\n',
                encoding="utf-8",
            )

            self.assertEqual(run_duration_s(root, 600.0), 900.0)

    def test_run_duration_uses_launch_value_without_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            self.assertEqual(run_duration_s(Path(temporary), "750"), 750.0)


class CorridorRecenterTest(unittest.TestCase):
    def test_selects_nearby_high_clearance_corridor_center(self) -> None:
        width = 11
        height = 11
        resolution = 0.2
        data = []
        for _row in range(height):
            for col in range(width):
                data.append(100 if col in {0, width - 1} else 0)

        goal = select_corridor_center_goal(
            data,
            width,
            height,
            resolution,
            0.0,
            0.0,
            (0.35, 1.10),
            search_radius_m=1.5,
            candidate_step_m=0.2,
            clearance_radius_m=1.2,
            clearance_sample_step_m=0.1,
            min_clearance_m=0.75,
            min_step_m=0.1,
        )

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertAlmostEqual(goal.x, 1.1, delta=0.25)
        self.assertGreaterEqual(goal.clearance_m, 0.75)

    def test_rejects_unknown_or_occupied_cells(self) -> None:
        width = 5
        height = 5
        resolution = 0.2
        data = [-1] * (width * height)
        data[2 * width + 2] = 100

        goal = select_corridor_center_goal(
            data,
            width,
            height,
            resolution,
            0.0,
            0.0,
            (0.5, 0.5),
            search_radius_m=1.0,
            min_clearance_m=0.1,
            min_step_m=0.0,
        )

        self.assertIsNone(goal)


class FrontierGoalClearanceTest(unittest.TestCase):
    def test_frontier_goal_prefers_corridor_center_over_nearest_wall_side(self) -> None:
        width = 9
        height = 9
        data = []
        for _row in range(height):
            for col in range(width):
                data.append(100 if col in {0, width - 1} else 0)
        cluster = [(2, 4), (2, 5), (2, 3)]

        goal = frontier_goal(cluster, data, width, height, setback_cells=4)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertGreaterEqual(goal[0], 3)
        self.assertGreater(
            occupied_clearance_cells(data, width, height, goal[0], goal[1]),
            occupied_clearance_cells(data, width, height, 1, goal[1]),
        )


if __name__ == "__main__":
    unittest.main()

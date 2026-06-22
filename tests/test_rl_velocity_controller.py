from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class RlVelocityMetricsTest(unittest.TestCase):
    def test_checkpoint_ranking_prefers_fast_safe_successful_average(self) -> None:
        from rl_velocity.metrics import rank_checkpoint_summaries

        ranked = rank_checkpoint_summaries(
            {
                "slow.zip": {"success_rate": 1.0, "crash_or_fall_rate": 0.0, "score": 20.0},
                "fast.zip": {"success_rate": 1.0, "crash_or_fall_rate": 0.0, "score": 12.0},
                "crashy.zip": {"success_rate": 1.0, "crash_or_fall_rate": 0.2, "score": 8.0},
            },
            min_success_rate=0.85,
            max_crash_or_fall_rate=0.05,
        )

        self.assertEqual(ranked[0]["checkpoint"], "fast.zip")
        self.assertTrue(ranked[0]["passes_gates"])
        self.assertFalse(ranked[-1]["passes_gates"])

    def test_summary_keeps_goal_time_success_only_and_phase_counts(self) -> None:
        from rl_velocity.metrics import summarize_metrics

        summary = summarize_metrics(
            [
                {
                    "success": True,
                    "goal_time_s": 12.0,
                    "final_status": "SUCCESS",
                    "collision_count": 0,
                    "fall_count": 0,
                    "turn_collision_count": 1,
                    "straight_collision_count": 0,
                    "turn_fall_count": 0,
                    "straight_fall_count": 0,
                    "distance_travelled_m": 6.0,
                    "average_speed_mps": 0.5,
                    "turn_average_speed_mps": 0.25,
                    "straight_average_speed_mps": 0.7,
                    "max_speed_mps": 0.9,
                    "turn_max_speed_mps": 0.4,
                    "straight_max_speed_mps": 0.9,
                    "route_turn_segment_count": 2,
                    "route_straight_segment_count": 3,
                    "completed_turn_segment_count": 2,
                    "completed_straight_segment_count": 3,
                    "failed_turn_segment_count": 0,
                    "failed_straight_segment_count": 0,
                    "backward_progress_events": 1,
                    "backward_progress_m": 0.2,
                    "reverse_command_steps": 2,
                    "reverse_command_time_s": 0.2,
                    "command_jerk": 0.1,
                },
                {
                    "success": False,
                    "goal_time_s": None,
                    "final_status": "STUCK",
                    "failure_phase": "turn",
                    "collision_count": 0,
                    "fall_count": 0,
                    "turn_collision_count": 0,
                    "straight_collision_count": 0,
                    "turn_fall_count": 0,
                    "straight_fall_count": 0,
                    "distance_travelled_m": 4.0,
                    "average_speed_mps": 0.25,
                    "turn_average_speed_mps": 0.2,
                    "straight_average_speed_mps": 0.3,
                    "max_speed_mps": 0.6,
                    "turn_max_speed_mps": 0.3,
                    "straight_max_speed_mps": 0.6,
                    "route_turn_segment_count": 2,
                    "route_straight_segment_count": 3,
                    "completed_turn_segment_count": 1,
                    "completed_straight_segment_count": 2,
                    "failed_turn_segment_count": 1,
                    "failed_straight_segment_count": 0,
                    "backward_progress_events": 2,
                    "backward_progress_m": 0.4,
                    "reverse_command_steps": 3,
                    "reverse_command_time_s": 0.3,
                    "command_jerk": 0.3,
                },
            ]
        )

        self.assertEqual(summary["episodes"], 2)
        self.assertEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["avg_goal_time_s"], 12.0)
        self.assertEqual(summary["turn_collision_count"], 1)
        self.assertEqual(summary["failed_turn_segment_count"], 1)
        self.assertEqual(summary["backward_progress_events"], 3)
        self.assertEqual(summary["failure_phase_counts"], {"turn": 1})


class RlVelocityEvalSuiteTest(unittest.TestCase):
    def test_corridor_sweep_suite_has_100_random_width_cases(self) -> None:
        from scripts.evaluate_maze_velocity_policy import load_episode_suite

        suite = load_episode_suite(ROOT / "configs" / "rl_velocity_eval_corridor_sweep_100.yaml")
        episodes = suite["episodes"]
        widths = sorted({episode["corridor_width_m"] for episode in episodes})
        stage_counts = {
            stage: sum(1 for episode in episodes if episode["stage"] == stage)
            for stage in {episode["stage"] for episode in episodes}
        }
        width_counts = {
            width: sum(1 for episode in episodes if episode["corridor_width_m"] == width)
            for width in widths
        }

        self.assertEqual(len(episodes), 100)
        self.assertEqual(widths, [2.0, 2.5, 3.0, 3.5, 4.0])
        self.assertEqual(stage_counts, {"full_small_maze": 50, "larger_random_maze": 50})
        self.assertEqual(width_counts, {2.0: 20, 2.5: 20, 3.0: 20, 3.5: 20, 4.0: 20})
        self.assertEqual(len({episode["id"] for episode in episodes}), 100)


@unittest.skipUnless(importlib.util.find_spec("numpy"), "numpy is required for maze fixtures")
class RlVelocityCurriculumTest(unittest.TestCase):
    def test_staged_corridor_mazes_have_free_start_goal(self) -> None:
        from maze.validator import validate_maze
        from rl_velocity.curriculum import StageSpec, maze_for_stage

        for kind in ("straight_corridor", "one_90_turn", "s_turns", "t_junctions"):
            stage = StageSpec(name=kind, kind=kind, width_cells=13, height_cells=13, cell_size_m=2.0)
            maze = maze_for_stage(stage, seed=7)
            result = validate_maze(maze, safety_radius_m=0.45, min_corridor_width_m=1.0)
            self.assertTrue(result.is_valid, result.errors)
            self.assertIsNotNone(result.path)


@unittest.skipUnless(
    importlib.util.find_spec("numpy") and importlib.util.find_spec("gymnasium"),
    "gymnasium and numpy are required for env helper tests",
)
class RlVelocityEnvHelperTest(unittest.TestCase):
    def test_action_scaling_matches_plan_limits(self) -> None:
        from rl_velocity.env import G1MazeVelocityEnv

        env = object.__new__(G1MazeVelocityEnv)
        env.command_limits = {
            "vx_min_mps": -0.35,
            "vx_max_mps": 0.80,
            "vy_min_mps": -0.20,
            "vy_max_mps": 0.20,
            "yaw_rate_max_radps": 1.20,
        }

        command = env._action_to_command([-1.0, 1.0, 0.0])

        self.assertAlmostEqual(command.vx, -0.35)
        self.assertAlmostEqual(command.vy, 0.20)
        self.assertAlmostEqual(command.yaw_rate, 0.0)


@unittest.skipUnless(
    all(importlib.util.find_spec(name) for name in ("numpy", "gymnasium", "mujoco", "torch")),
    "MuJoCo smoke test requires numpy, gymnasium, mujoco, and torch",
)
class RlVelocityMujocoSmokeTest(unittest.TestCase):
    def test_env_reset_and_random_steps_are_finite(self) -> None:
        import numpy as np
        from rl_velocity import OBSERVATION_DIM
        from rl_velocity.env import G1MazeVelocityEnv

        policy = ROOT / "third_party" / "unitree_rl_gym" / "deploy" / "pre_train" / "g1" / "motion.pt"
        model = ROOT / "third_party" / "unitree_rl_gym" / "resources" / "robots" / "g1_description" / "g1_12dof.xml"
        if not policy.exists() or not model.exists():
            self.skipTest("Unitree RL Gym assets are not available")

        env = G1MazeVelocityEnv(
            config_path=ROOT / "configs" / "default.yaml",
            rl_config_path=ROOT / "configs" / "rl_velocity_controller.yaml",
            run_dir=ROOT / ".tmp" / "rl-velocity-smoke",
            stage="straight_corridor",
            seed=5,
            unitree_rl_gym_repo=ROOT / "third_party" / "unitree_rl_gym",
            training=False,
        )
        obs, _ = env.reset()
        self.assertEqual(obs.shape, (OBSERVATION_DIM,))
        self.assertTrue(np.isfinite(obs).all())
        for _ in range(3):
            obs, reward, terminated, truncated, _ = env.step(env.action_space.sample())
            self.assertTrue(np.isfinite(obs).all())
            self.assertTrue(np.isfinite(reward))
            if terminated or truncated:
                break
        env.close()


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types
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

        for kind in (
            "straight_corridor",
            "one_90_turn_right",
            "one_90_turn_left",
            "s_turns_right_first",
            "s_turns_left_first",
            "t_junctions",
        ):
            stage = StageSpec(name=kind, kind=kind, width_cells=13, height_cells=13, cell_size_m=2.0)
            maze = maze_for_stage(stage, seed=7)
            result = validate_maze(maze, safety_radius_m=0.45, min_corridor_width_m=1.0)
            self.assertTrue(result.is_valid, result.errors)
            self.assertIsNotNone(result.path)

    def test_one_turn_curriculum_covers_left_and_right_arcs(self) -> None:
        from nav.oracle_follow import TurnAwareFollowerConfig, build_turn_aware_path
        from nav.planner import plan_oracle_path
        from rl_velocity.curriculum import StageSpec, maze_for_stage

        directions = {}
        for kind in ("one_90_turn_right", "one_90_turn_left"):
            stage = StageSpec(name=kind, kind=kind, width_cells=9, height_cells=9, cell_size_m=2.0)
            maze = maze_for_stage(stage, seed=7)
            plan = plan_oracle_path(maze, simplify=False, planner="heading_astar", turn_penalty_cost=2.0)
            path = build_turn_aware_path(maze, plan.cells, TurnAwareFollowerConfig())
            directions[kind] = [segment.turn_direction for segment in path.arc_segments]

        self.assertIn("right", directions["one_90_turn_right"])
        self.assertIn("left", directions["one_90_turn_left"])

    def test_stage_specs_keep_reverse_recovery_yaw_offsets(self) -> None:
        from rl_velocity.curriculum import load_stage_specs

        stages = load_stage_specs(
            {
                "curriculum": {
                    "stages": [
                        {
                            "name": "reverse_recovery_left",
                            "kind": "straight_corridor",
                            "start_yaw_offset_rad": -2.8,
                        }
                    ]
                }
            }
        )

        self.assertAlmostEqual(stages[0].start_yaw_offset_rad, -2.8)


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

    def test_locomotion_calibration_caps_rl_command_limits(self) -> None:
        from rl_velocity.env import apply_locomotion_calibration_to_command_limits

        limits = {
            "vx_min_mps": -2.0,
            "vx_max_mps": 2.0,
            "vy_min_mps": -1.5,
            "vy_max_mps": 1.5,
            "yaw_rate_max_radps": 3.0,
        }
        calibration = {
            "selected_max_forward_mps": 1.4,
            "command_limits": {"max_reverse_mps": -0.6},
            "recommended_safe_limits": {"max_safe_vx": 1.4, "max_safe_wz": 2.0},
        }

        capped = apply_locomotion_calibration_to_command_limits(limits, calibration)

        self.assertEqual(capped["vx_max_mps"], 1.4)
        self.assertEqual(capped["vx_min_mps"], -0.6)
        self.assertEqual(capped["yaw_rate_max_radps"], 2.0)
        self.assertEqual(capped["vy_min_mps"], -1.5)
        self.assertEqual(capped["vy_max_mps"], 1.5)

    def test_odom_pose_applies_bias_noise_and_wraps_yaw(self) -> None:
        from nav.controller import Pose2D
        from rl_velocity.env import G1MazeVelocityEnv

        env = object.__new__(G1MazeVelocityEnv)
        env.odom_bias_x_m = 0.10
        env.odom_bias_y_m = -0.20
        env.odom_bias_yaw_rad = 0.30
        env.odom_noise_x_m = 0.01
        env.odom_noise_y_m = 0.02
        env.odom_noise_yaw_rad = 0.04

        pose = env._odom_pose_from_truth(Pose2D(x=1.0, y=2.0, yaw=3.10))

        self.assertAlmostEqual(pose.x, 1.11)
        self.assertAlmostEqual(pose.y, 1.82)
        self.assertGreaterEqual(pose.yaw, -3.141593)
        self.assertLessEqual(pose.yaw, 3.141593)

    def test_odom_bias_clip_bounds_translation_and_yaw(self) -> None:
        from rl_velocity.env import G1MazeVelocityEnv

        env = object.__new__(G1MazeVelocityEnv)
        env.odometry_training = {"max_xy_bias_m": 0.25, "max_yaw_bias_rad": 0.5}
        env.odom_bias_x_m = 3.0
        env.odom_bias_y_m = 4.0
        env.odom_bias_yaw_rad = 2.0

        env._clip_odom_bias()

        self.assertAlmostEqual((env.odom_bias_x_m**2 + env.odom_bias_y_m**2) ** 0.5, 0.25)
        self.assertAlmostEqual(env.odom_bias_yaw_rad, 0.5)

    def test_reverse_reward_reduces_backtrack_penalty_when_misaligned(self) -> None:
        import numpy as np
        from rl_velocity.env import G1MazeVelocityEnv
        from sim.locomotion_policy_adapter import VelocityCommand

        env = object.__new__(G1MazeVelocityEnv)
        env.reward_weights = {
            "progress": 8.0,
            "reverse_backtrack_penalty_scale": 0.35,
            "reverse_heading_error_threshold_rad": 1.75,
            "reverse_misaligned": 0.0,
            "aligned_speed": 0.0,
            "tilt": 0.0,
            "command_jerk": 0.0,
        }
        env.termination = {"spin_yaw_rate_radps": 0.75}
        env.action_dt = 0.1
        env.previous_action = np.zeros(3, dtype=np.float32)
        env.wall_contact_this_step = False
        env.stuck_active_since = None
        env.data = types.SimpleNamespace(qpos=[0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0])
        projection = types.SimpleNamespace(heading_error_rad=2.2)

        reverse_reward = env._reward(
            projection=projection,
            progress_delta=-0.1,
            action=np.zeros(3, dtype=np.float32),
            command=VelocityCommand(vx=-0.2),
            status="RUNNING",
            terminated=False,
            truncated=False,
        )
        forward_reward = env._reward(
            projection=projection,
            progress_delta=-0.1,
            action=np.zeros(3, dtype=np.float32),
            command=VelocityCommand(vx=0.2),
            status="RUNNING",
            terminated=False,
            truncated=False,
        )

        self.assertGreater(reverse_reward, forward_reward)


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

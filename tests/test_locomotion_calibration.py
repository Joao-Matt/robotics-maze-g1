from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import importlib.util
import sys
import tempfile
from types import SimpleNamespace
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class LocomotionCalibrationConfigTest(unittest.TestCase):
    def test_balanced_config_expands_expected_command_grid(self) -> None:
        from sim.locomotion_calibration import load_calibration_suite

        suite = load_calibration_suite(ROOT / "configs" / "g1_locomotion_calibration.yaml", "balanced")
        groups = {group: sum(1 for command in suite.commands if command.group == group) for group in {
            command.group for command in suite.commands
        }}

        self.assertEqual(len(suite.commands), 87)
        self.assertEqual(groups["straight"], 10)
        self.assertEqual(groups["pure_rotation"], 20)
        self.assertEqual(groups["reverse_recovery"], 9)
        self.assertEqual(groups["arc"], 48)

    def test_arc_radius_uses_inf_for_near_zero_yaw(self) -> None:
        from sim.locomotion_calibration import arc_radius

        self.assertEqual(arc_radius(0.8, 0.0), "inf")
        self.assertAlmostEqual(arc_radius(0.8, 0.4), 2.0)


class LocomotionCalibrationScoringTest(unittest.TestCase):
    def test_fall_stuck_and_non_floor_contact_are_unsafe(self) -> None:
        from sim.locomotion_calibration import CalibrationCommand, summarize_trial

        row = summarize_trial(
            command=CalibrationCommand("straight", 1.0, 0.0),
            duration_s=8.0,
            samples=[
                {
                    "actual_vx": 0.0,
                    "actual_wz": 0.0,
                    "base_z": 0.4,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "distance_m": 0.0,
                    "yaw_changed_rad": 0.0,
                    "lateral_drift_m": 0.0,
                    "contact": True,
                    "non_floor_contact": True,
                    "low_motion_s": 2.0,
                }
            ],
            stability={"stable_score_threshold": 75.0, "stuck_expected_fraction": 0.25, "stuck_duration_s": 2.0},
            fell=True,
        )

        self.assertFalse(row["stable"])
        self.assertEqual(row["failure_reason"], "fall")
        self.assertEqual(row["stability_score"], 0.0)

    def test_recommendations_select_safe_limits_and_exclude_unsafe(self) -> None:
        from sim.locomotion_calibration import recommend_safe_limits

        rows = [
            _row("straight", 0.8, 0.0, True, 92.0),
            _row("straight", 1.0, 0.0, False, 55.0, reason="low_stability_score"),
            _row("pure_rotation", 0.0, 0.6, True, 90.0),
            _row("pure_rotation", 0.0, -0.8, True, 91.0),
            _row("arc", 0.4, 0.3, True, 88.0, actual_vx=0.35, actual_wz=0.25, radius=1.4),
            _row("reverse_recovery", -0.2, 0.0, True, 86.0),
        ]

        recommendations = recommend_safe_limits(
            rows,
            {
                "preferred_max_vx_error_mps": 0.35,
                "preferred_max_wz_error_radps": 0.35,
                "preferred_max_lateral_drift_m": 0.5,
            },
        )

        self.assertEqual(recommendations["max_safe_vx"], 0.8)
        self.assertEqual(recommendations["max_safe_wz"], 0.6)
        self.assertEqual(len(recommendations["unsafe_commands"]), 1)
        self.assertEqual(recommendations["unsafe_commands"][0]["cmd_vx"], 1.0)
        self.assertTrue(recommendations["recovery_safe_commands"])


class NavigationCalibrationCompatibilityTest(unittest.TestCase):
    def test_navigation_limit_modes_cap_or_use_calibrated_envelope(self) -> None:
        from scripts.render_navigation_config import calibrated_navigation_limits

        configured = {
            "max_forward_mps": 0.45,
            "min_forward_mps": 0.12,
            "max_reverse_mps": -0.30,
            "max_yaw_rate_radps": 1.20,
            "turn_slowdown_start_radps": 0.45,
            "turn_slowdown_full_radps": 1.10,
            "turn_slowdown_min_forward_mps": 0.12,
        }
        calibration = {
            "selected_max_forward_mps": 1.4,
            "command_limits": {"max_forward_mps": 1.4, "max_reverse_mps": -0.6, "max_yaw_rate_radps": 2.0},
            "recommended_safe_limits": {
                "max_safe_vx": 1.4,
                "max_safe_wz": 2.0,
                "turn_slowdown_start_radps": 0.7,
                "turn_slowdown_full_radps": 1.6,
            },
        }

        capped = calibrated_navigation_limits(calibration, configured, limit_mode="cap")
        active = calibrated_navigation_limits(calibration, configured, limit_mode="use-calibration")

        self.assertEqual(capped["max_forward_mps"], 0.45)
        self.assertEqual(capped["max_yaw_rate_radps"], 1.20)
        self.assertEqual(capped["max_reverse_mps"], -0.30)
        self.assertEqual(capped["turn_slowdown_start_radps"], 0.45)
        self.assertEqual(active["max_forward_mps"], 1.4)
        self.assertEqual(active["max_yaw_rate_radps"], 2.0)
        self.assertEqual(active["max_reverse_mps"], -0.6)
        self.assertEqual(active["turn_slowdown_start_radps"], 0.7)

    def test_use_calibration_mode_applies_configured_maze_caps(self) -> None:
        from scripts.render_navigation_config import calibrated_navigation_limits

        configured = {
            "max_forward_mps": 0.45,
            "max_yaw_rate_radps": 1.20,
            "calibrated_nav2_forward_scale": 0.75,
            "calibrated_nav2_yaw_rate_scale": 0.65,
            "calibrated_nav2_max_forward_mps": 0.80,
            "calibrated_nav2_max_yaw_rate_radps": 1.20,
        }
        calibration = {
            "selected_max_forward_mps": 1.4,
            "command_limits": {"max_yaw_rate_radps": 2.0},
            "recommended_safe_limits": {"max_safe_vx": 1.4, "max_safe_wz": 2.0},
        }

        active = calibrated_navigation_limits(calibration, configured, limit_mode="use-calibration")

        self.assertEqual(active["max_forward_mps"], 0.80)
        self.assertEqual(active["max_yaw_rate_radps"], 1.20)

    def test_render_config_consumes_rich_calibration_without_pose_data(self) -> None:
        from scripts.render_navigation_config import render_configs

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = {
                "maze": {"cell_size_m": 2.0, "cell_width_m": 2.0, "cell_length_m": 2.0},
                "nav2_navigation": {
                    "locomotion_policy": "unitree_rl_gym_native",
                    "max_forward_mps": 1.5,
                    "max_reverse_mps": -0.3,
                    "min_forward_mps": 0.1,
                    "max_yaw_rate_radps": 2.0,
                    "turn_slowdown_start_radps": 0.8,
                    "turn_slowdown_full_radps": 1.5,
                    "turn_slowdown_min_forward_mps": 0.1,
                },
            }
            nav = {
                "controller_server": {"ros__parameters": {"FollowPath": {}}},
                "global_costmap": {"global_costmap": {"ros__parameters": {"inflation_layer": {}}}},
                "local_costmap": {"local_costmap": {"ros__parameters": {"inflation_layer": {}}}},
            }
            calibration = {
                "schema_version": 2,
                "source": "direct_mujoco_command_sweep",
                "status": "passed",
                "cache_key": "abc",
                "policy": "unitree_rl_gym_native",
                "selected_max_forward_mps": 0.8,
                "recommended_safe_limits": {
                    "max_safe_vx": 0.8,
                    "max_safe_wz": 0.7,
                    "turn_slowdown_start_radps": 0.25,
                    "turn_slowdown_full_radps": 0.55,
                },
                "recovery_safe_commands": [{"group": "reverse_recovery", "cmd_vx": -0.2, "cmd_wz": 0.0}],
                "ground_truth_used_for_calibration_metrics": True,
                "final_pose": {"x": 1.0},
                "trajectory": [{"x": 1.0}],
            }
            config_path = root / "config.yaml"
            nav_path = root / "nav.yaml"
            cal_path = root / "calibration.json"
            config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
            nav_path.write_text(yaml.safe_dump(nav), encoding="utf-8")
            cal_path.write_text(__import__("json").dumps(calibration), encoding="utf-8")

            render_configs(
                config_path=config_path,
                nav2_template_path=nav_path,
                calibration_path=cal_path,
                output_dir=root / "out",
                limit_mode="use-calibration",
            )

            resolved = yaml.safe_load((root / "out" / "resolved_config.yaml").read_text(encoding="utf-8"))
            rendered_text = (root / "out" / "resolved_config.yaml").read_text(encoding="utf-8")
            follow = yaml.safe_load((root / "out" / "resolved_nav2_params.yaml").read_text(encoding="utf-8"))[
                "controller_server"
            ]["ros__parameters"]["FollowPath"]
            self.assertEqual(resolved["nav2_navigation"]["max_forward_mps"], 0.8)
            self.assertEqual(resolved["nav2_navigation"]["max_yaw_rate_radps"], 0.7)
            self.assertEqual(follow["max_vel_x"], 0.8)
            self.assertEqual(follow["min_vel_x"], -0.3)
            self.assertEqual(follow["max_vel_theta"], 0.7)
            self.assertNotIn("final_pose", rendered_text)
            self.assertNotIn("trajectory", rendered_text)

    def test_render_config_scales_inflation_to_quarter_corridor_width(self) -> None:
        from scripts.render_navigation_config import render_configs

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = {
                "maze": {"cell_size_m": 2.0, "cell_width_m": 2.0, "cell_length_m": 2.0},
                "nav2_navigation": {
                    "locomotion_policy": "unitree_rl_gym_native",
                    "max_forward_mps": 0.45,
                    "max_yaw_rate_radps": 1.0,
                    "inflation_radius_cell_width_fraction": 0.25,
                },
            }
            nav = {
                "controller_server": {"ros__parameters": {"FollowPath": {}}},
                "global_costmap": {"global_costmap": {"ros__parameters": {"inflation_layer": {}}}},
                "local_costmap": {"local_costmap": {"ros__parameters": {"inflation_layer": {}}}},
            }
            calibration = {
                "selected_max_forward_mps": 0.45,
                "recommended_safe_limits": {"max_safe_vx": 0.45, "max_safe_wz": 1.0},
            }
            config_path = root / "config.yaml"
            nav_path = root / "nav.yaml"
            cal_path = root / "calibration.json"
            config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
            nav_path.write_text(yaml.safe_dump(nav), encoding="utf-8")
            cal_path.write_text(__import__("json").dumps(calibration), encoding="utf-8")

            render_configs(
                config_path=config_path,
                nav2_template_path=nav_path,
                calibration_path=cal_path,
                output_dir=root / "out",
                cell_size_m=4.0,
            )

            rendered = yaml.safe_load((root / "out" / "resolved_nav2_params.yaml").read_text(encoding="utf-8"))
            for section in ("global_costmap", "local_costmap"):
                params = rendered[section][section]["ros__parameters"]
                self.assertEqual(params["inflation_layer"]["inflation_radius"], 1.0)


class OracleCalibrationCompatibilityTest(unittest.TestCase):
    def test_oracle_use_safe_mode_uses_preferred_arc_and_safe_limits(self) -> None:
        from nav.oracle_follow import TurnAwareFollowerConfig
        from scripts.run_g1_oracle_follow import _apply_locomotion_calibration_to_follower_config

        base = TurnAwareFollowerConfig(
            forward_speed_mps=1.0,
            max_yaw_rate_radps=0.8,
            arc_turn_forward_speed_mps=0.4,
            arc_turn_yaw_rate_radps=0.8,
            recovery_reverse_speed_mps=-0.18,
        )
        calibration = {
            "status": "passed",
            "selected_max_forward_mps": 1.4,
            "recommended_safe_limits": {"max_safe_vx": 1.4, "max_safe_wz": 2.0},
            "preferred_arc_commands": {
                "tight_turn": [{"cmd_vx": 0.2, "cmd_wz": 1.0}],
            },
            "recovery_safe_commands": [{"cmd_vx": -0.4, "cmd_wz": 0.0}],
            "ground_truth_used_for_calibration_metrics": True,
        }

        config, summary = _apply_locomotion_calibration_to_follower_config(
            base,
            calibration,
            mode="use-safe",
        )

        self.assertAlmostEqual(config.forward_speed_mps, 1.4)
        self.assertAlmostEqual(config.max_yaw_rate_radps, 2.0)
        self.assertAlmostEqual(config.arc_turn_forward_speed_mps, 0.2)
        self.assertAlmostEqual(config.arc_turn_yaw_rate_radps, 1.0)
        self.assertAlmostEqual(config.recovery_reverse_speed_mps, -0.4)
        self.assertEqual(summary["mode"], "use-safe")

    def test_oracle_cap_mode_only_reduces_existing_speeds(self) -> None:
        from nav.oracle_follow import TurnAwareFollowerConfig
        from scripts.run_g1_oracle_follow import _apply_locomotion_calibration_to_follower_config

        base = TurnAwareFollowerConfig(
            forward_speed_mps=1.0,
            max_yaw_rate_radps=0.8,
            arc_turn_forward_speed_mps=0.4,
            arc_turn_yaw_rate_radps=0.8,
        )
        calibration = {
            "recommended_safe_limits": {"max_safe_vx": 0.6, "max_safe_wz": 0.5},
            "preferred_arc_commands": {"tight_turn": [{"cmd_vx": 0.3, "cmd_wz": 0.4}]},
        }

        config, _summary = _apply_locomotion_calibration_to_follower_config(base, calibration, mode="cap")

        self.assertAlmostEqual(config.forward_speed_mps, 0.6)
        self.assertAlmostEqual(config.max_yaw_rate_radps, 0.5)
        self.assertAlmostEqual(config.arc_turn_forward_speed_mps, 0.4)
        self.assertAlmostEqual(config.arc_turn_yaw_rate_radps, 0.5)


class LocomotionCalibrationBatchTest(unittest.TestCase):
    def test_random_seed_generation_is_deterministic_and_unique(self) -> None:
        module = _load_batch_module()

        first = module.generate_random_seeds(123, 100)
        second = module.generate_random_seeds(123, 100)

        self.assertEqual(first, second)
        self.assertEqual(len(first), 100)
        self.assertEqual(len(set(first)), 100)

    def test_seed_metric_records_gate_failures_and_group_counts(self) -> None:
        module = _load_batch_module()
        gates = module.GateSettings(
            min_stable_rate=0.70,
            max_fall_count=0,
            max_stuck_count=0,
            max_non_floor_contact_count=5,
            min_safe_vx=0.4,
            min_safe_wz=0.4,
        )
        rows = [
            _batch_row("straight", stable=True, score=90.0),
            _batch_row("arc", stable=False, score=40.0, stuck=True, reason="stuck"),
        ]

        metric = module.build_seed_metric(
            index=1,
            seed=99,
            profile="balanced",
            friction_scale=1.0,
            run_dir=ROOT / ".tmp" / "fake-calibration",
            elapsed_wall_s=12.0,
            suite_timing={"warmup_s": 1.0, "command_s": 8.0, "settle_s": 0.5},
            gates=gates,
            summary={
                "status": "passed",
                "total_commands": 2,
                "stable_commands": 1,
                "fall_count": 0,
                "stuck_count": 1,
                "non_floor_contact_count": 0,
                "recommended_safe_limits": {"max_safe_vx": 0.8, "max_safe_wz": 0.6},
                "locomotion_calibration": {"selected_max_forward_mps": 0.8},
            },
            command_rows=rows,
        )

        self.assertFalse(metric["reached_goal"])
        self.assertIn("low_stable_rate", metric["failure_reason"])
        self.assertIn("stuck", metric["failure_reason"])
        self.assertEqual(metric["straight_stable"], 1)
        self.assertEqual(metric["arc_stuck"], 1)
        self.assertIsNone(metric["goal_time_s"])

    def test_batch_report_averages_goal_time_only_for_successful_seeds(self) -> None:
        module = _load_batch_module()
        gates = module.GateSettings(
            min_stable_rate=0.70,
            max_fall_count=0,
            max_stuck_count=0,
            max_non_floor_contact_count=5,
            min_safe_vx=0.4,
            min_safe_wz=0.4,
        )
        args = SimpleNamespace(
            profile="balanced",
            seed=123,
            friction_scale_min=0.75,
            friction_scale_max=1.15,
        )
        rows = [
            {
                "seed": 1,
                "suite_completed": True,
                "reached_goal": True,
                "goal_time_s": 10.0,
                "elapsed_wall_s": 10.0,
                "stable_rate": 0.8,
                "fall_count": 0,
                "stuck_count": 0,
                "collision_count": 0,
                "non_floor_contact_count": 0,
                "max_safe_vx": 0.8,
                "max_safe_wz": 0.6,
                "avg_stability_score": 90.0,
            },
            {
                "seed": 2,
                "suite_completed": True,
                "reached_goal": False,
                "failure_reason": "fall",
                "goal_time_s": None,
                "elapsed_wall_s": 20.0,
                "stable_rate": 0.5,
                "fall_count": 1,
                "stuck_count": 0,
                "collision_count": 0,
                "non_floor_contact_count": 0,
                "max_safe_vx": 0.2,
                "max_safe_wz": 0.1,
                "avg_stability_score": 60.0,
            },
        ]

        report = module.build_batch_report(
            batch_dir=ROOT / ".tmp" / "fake-batch",
            seeds=[1, 2],
            rows=rows,
            args=args,
            gates=gates,
        )

        self.assertEqual(report["goal_reached_count"], 1)
        self.assertEqual(report["success_rate"], 0.5)
        self.assertEqual(report["average_time_to_goal_s"], 10.0)
        self.assertEqual(report["total_fall_count"], 1)


def _mujoco_smoke_available() -> bool:
    if not all(importlib.util.find_spec(name) for name in ("mujoco", "torch", "yaml")):
        return False
    numpy_module = sys.modules.get("numpy")
    return numpy_module is None or hasattr(numpy_module, "__version__")


@unittest.skipUnless(_mujoco_smoke_available(), "MuJoCo smoke test requires real numpy, mujoco, torch, and yaml")
class LocomotionCalibrationMujocoSmokeTest(unittest.TestCase):
    def test_smoke_profile_writes_artifacts(self) -> None:
        from sim.locomotion_calibration import G1LocomotionCalibrationRunner, load_calibration_suite

        numpy_module = sys.modules.get("numpy")
        if numpy_module is not None and not hasattr(numpy_module, "__version__"):
            self.skipTest("MuJoCo smoke test requires real numpy")
        policy = ROOT / "third_party" / "unitree_rl_gym" / "deploy" / "pre_train" / "g1" / "motion.pt"
        model = ROOT / "third_party" / "unitree_rl_gym" / "resources" / "robots" / "g1_description" / "scene.xml"
        if not policy.exists() or not model.exists():
            self.skipTest("Unitree RL Gym assets are not available")

        with tempfile.TemporaryDirectory() as temporary:
            suite = load_calibration_suite(ROOT / "configs" / "g1_locomotion_calibration.yaml", "smoke")
            suite = replace(suite, commands=suite.commands[:3])
            runner = G1LocomotionCalibrationRunner(
                project_config_path=ROOT / "configs" / "default.yaml",
                calibration_config_path=ROOT / "configs" / "g1_locomotion_calibration.yaml",
                suite=suite,
                run_dir=Path(temporary),
                seed=7,
                unitree_rl_gym_repo=ROOT / "third_party" / "unitree_rl_gym",
            )

            summary = runner.run()

            self.assertEqual(summary["total_commands"], 3)
            self.assertTrue((Path(temporary) / "command_results.csv").is_file())
            self.assertTrue((Path(temporary) / "locomotion_calibration.json").is_file())
            self.assertTrue((Path(temporary) / "report.md").is_file())
            self.assertTrue((Path(temporary) / "dashboard.html").is_file())


def _row(
    group: str,
    vx: float,
    wz: float,
    stable: bool,
    score: float,
    *,
    reason: str = "",
    actual_vx: float | None = None,
    actual_wz: float | None = None,
    radius: float | str = "inf",
) -> dict:
    return {
        "group": group,
        "cmd_vx": vx,
        "cmd_wz": wz,
        "actual_vx_mean": vx if actual_vx is None else actual_vx,
        "actual_wz_mean": wz if actual_wz is None else actual_wz,
        "actual_arc_radius_m": radius,
        "vx_tracking_error_mean_abs": 0.05,
        "wz_tracking_error_mean_abs": 0.05,
        "lateral_drift_m": 0.02,
        "stability_score": score,
        "stable": stable,
        "failure_reason": reason,
    }


def _batch_row(
    group: str,
    *,
    stable: bool,
    score: float,
    stuck: bool = False,
    fell: bool = False,
    non_floor_contact: bool = False,
    reason: str = "",
) -> dict:
    return {
        "group": group,
        "stable": stable,
        "stability_score": score,
        "stuck": stuck,
        "fell": fell,
        "contact": True,
        "non_floor_contact": non_floor_contact,
        "failure_reason": reason,
    }


def _load_batch_module():
    path = ROOT / "scripts" / "run_g1_locomotion_calibration_seed_batch.py"
    spec = importlib.util.spec_from_file_location("run_g1_locomotion_calibration_seed_batch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


if __name__ == "__main__":
    unittest.main()

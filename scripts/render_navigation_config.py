#!/usr/bin/env python3
"""Render per-run robot and Nav2 configs from locomotion calibration."""

from __future__ import annotations

from pathlib import Path
import argparse
import json

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--nav2-template", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cell-size-m", type=float)
    parser.add_argument("--limit-mode", choices=("cap", "use-calibration"), default="cap")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    render_configs(
        config_path=args.config,
        nav2_template_path=args.nav2_template,
        calibration_path=args.calibration,
        output_dir=args.output_dir,
        cell_size_m=args.cell_size_m,
        limit_mode=args.limit_mode,
    )
    return 0


def render_configs(
    *,
    config_path: Path,
    nav2_template_path: Path,
    calibration_path: Path,
    output_dir: Path,
    cell_size_m: float | None = None,
    limit_mode: str = "cap",
) -> None:
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    maze = config.setdefault("maze", {})
    if cell_size_m is not None:
        if cell_size_m <= 0.0:
            raise ValueError("--cell-size-m must be positive")
        maze["cell_size_m"] = float(cell_size_m)
        maze["cell_width_m"] = float(cell_size_m)
        maze["cell_length_m"] = float(cell_size_m)

    limits = config.setdefault("nav2_navigation", {})
    policy = str(limits.get("locomotion_policy", "")).strip()
    if policy and policy != "unitree_rl_gym_native":
        raise ValueError(f"Production navigation supports only unitree_rl_gym_native, got {policy!r}.")

    nav_limits = calibrated_navigation_limits(calibration, limits, limit_mode=limit_mode)
    speed = nav_limits["max_forward_mps"]
    yaw_rate = nav_limits["max_yaw_rate_radps"]
    limits.update(nav_limits)
    limits["locomotion_calibration_limit_mode"] = limit_mode
    limits["locomotion_calibration"] = calibration_summary_for_navigation(calibration)

    nav = yaml.safe_load(nav2_template_path.read_text(encoding="utf-8"))
    controller = nav["controller_server"]["ros__parameters"]
    controller["controller_frequency"] = float(
        limits.get("controller_frequency_hz", controller.get("controller_frequency", 12.0))
    )
    follow = controller["FollowPath"]
    follow["min_vel_x"] = float(limits.get("dwb_min_vel_x_mps", 0.0))
    follow["min_speed_xy"] = 0.0
    follow["max_vel_x"] = speed
    follow["max_speed_xy"] = speed
    follow["max_vel_theta"] = yaw_rate
    follow["min_speed_theta"] = float(limits.get("min_yaw_rate_radps", 0.0))
    follow["acc_lim_x"] = float(limits.get("max_linear_accel_mps2", 0.20))
    follow["acc_lim_theta"] = float(limits.get("max_yaw_accel_radps2", 0.40))
    follow["decel_lim_x"] = -float(limits.get("max_linear_decel_mps2", 1.20))
    follow["decel_lim_theta"] = -float(limits.get("max_yaw_decel_radps2", 1.20))
    follow["BaseObstacle.scale"] = float(limits.get("obstacle_critic_scale", follow.get("BaseObstacle.scale", 0.20)))
    follow["PathAlign.scale"] = float(limits.get("path_align_scale", follow.get("PathAlign.scale", 18.0)))
    follow["GoalAlign.scale"] = float(limits.get("goal_align_scale", follow.get("GoalAlign.scale", 8.0)))
    follow["PathDist.scale"] = float(limits.get("path_dist_scale", follow.get("PathDist.scale", 24.0)))
    follow["GoalDist.scale"] = float(limits.get("goal_dist_scale", follow.get("GoalDist.scale", 10.0)))
    follow["vx_samples"] = int(limits.get("dwb_vx_samples", follow.get("vx_samples", 16)))
    follow["vtheta_samples"] = int(limits.get("dwb_vtheta_samples", follow.get("vtheta_samples", 32)))
    follow["sim_time"] = float(limits.get("dwb_sim_time_s", follow.get("sim_time", 1.2)))
    follow["linear_granularity"] = float(
        limits.get("dwb_linear_granularity_m", follow.get("linear_granularity", 0.05))
    )
    follow["angular_granularity"] = float(
        limits.get("dwb_angular_granularity_rad", follow.get("angular_granularity", 0.025))
    )
    forward_point = float(limits.get("dwb_forward_point_distance_m", 0.325))
    follow["PathAlign.forward_point_distance"] = forward_point
    follow["GoalAlign.forward_point_distance"] = forward_point

    costmap_resolution = float(limits.get("costmap_resolution_m", 0.05))
    robot_radius = float(limits.get("costmap_robot_radius_m", 0.60))
    cell_width = float(maze.get("cell_width_m", maze.get("cell_size_m", 1.0)))
    inflation_fraction = limits.get("inflation_radius_cell_width_fraction")
    inflation_radius = (
        float(limits.get("inflation_radius_m", 1.20))
        if inflation_fraction is None
        else cell_width * float(inflation_fraction)
    )
    inflation_min = limits.get("inflation_radius_min_m")
    if inflation_min is not None:
        inflation_radius = max(inflation_radius, float(inflation_min))
    inflation_max = limits.get("inflation_radius_max_m")
    if inflation_max is not None:
        inflation_radius = min(inflation_radius, float(inflation_max))
    limits["inflation_radius_m"] = inflation_radius
    inflation_scale = float(limits.get("inflation_cost_scaling", 1.25))
    for section in ("global_costmap", "local_costmap"):
        params = nav[section][section]["ros__parameters"]
        params["resolution"] = costmap_resolution
        params["robot_radius"] = robot_radius
        inflation = params.get("inflation_layer", {})
        inflation["inflation_radius"] = inflation_radius
        inflation["cost_scaling_factor"] = inflation_scale
        params["inflation_layer"] = inflation

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "resolved_config.yaml").write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    (output_dir / "resolved_nav2_params.yaml").write_text(yaml.safe_dump(nav, sort_keys=False), encoding="utf-8")


def calibrated_navigation_limits(
    calibration: dict,
    configured_limits: dict,
    *,
    limit_mode: str = "cap",
) -> dict[str, float | list[dict[str, float | str]]]:
    """Return scalar Nav2 limits from calibration.

    cap keeps the old conservative behavior: config values are upper-bounded by
    measured calibration. use-calibration promotes measured safe values to the
    active command envelope.
    """
    if limit_mode not in {"cap", "use-calibration"}:
        raise ValueError(f"unsupported Nav2 calibration limit mode: {limit_mode!r}")
    recommended = calibration.get("recommended_safe_limits", {}) if isinstance(calibration, dict) else {}
    command_limits = calibration.get("command_limits", {}) if isinstance(calibration, dict) else {}
    if not isinstance(command_limits, dict):
        command_limits = {}
    calibrated_speed = _first_number(
        recommended.get("max_safe_vx"),
        calibration.get("selected_max_forward_mps"),
        command_limits.get("max_forward_mps", None),
        default=float(configured_limits.get("max_forward_mps", 0.60)),
    )
    configured_speed = float(configured_limits.get("max_forward_mps", 0.60))
    if limit_mode == "use-calibration":
        speed = max(0.0, calibrated_speed) * float(configured_limits.get("calibrated_nav2_forward_scale", 1.0))
        max_nav2_speed = configured_limits.get("calibrated_nav2_max_forward_mps")
        if max_nav2_speed is not None:
            speed = min(speed, max(0.0, float(max_nav2_speed)))
    else:
        speed = min(configured_speed, max(0.0, calibrated_speed))
    configured_minimum = float(configured_limits.get("min_forward_mps", 0.40))
    minimum_speed = min(configured_minimum, speed)

    configured_yaw = float(configured_limits.get("max_yaw_rate_radps", 0.40))
    calibrated_yaw = _first_number(
        recommended.get("max_safe_wz"),
        command_limits.get("max_yaw_rate_radps", None),
        default=configured_yaw,
    )
    if limit_mode == "use-calibration":
        yaw_rate = max(0.0, calibrated_yaw) * float(configured_limits.get("calibrated_nav2_yaw_rate_scale", 1.0))
        max_nav2_yaw = configured_limits.get("calibrated_nav2_max_yaw_rate_radps")
        if max_nav2_yaw is not None:
            yaw_rate = min(yaw_rate, max(0.0, float(max_nav2_yaw)))
    else:
        yaw_rate = min(configured_yaw, max(0.0, calibrated_yaw))
    configured_reverse = float(configured_limits.get("max_reverse_mps", -0.30))
    calibrated_reverse = _first_number(command_limits.get("max_reverse_mps"), default=configured_reverse)
    reverse = (
        min(0.0, calibrated_reverse)
        if limit_mode == "use-calibration"
        else max(configured_reverse, min(0.0, calibrated_reverse))
    )
    configured_start = float(configured_limits.get("turn_slowdown_start_radps", 0.45))
    configured_full = float(configured_limits.get("turn_slowdown_full_radps", 1.10))
    measured_start = _first_number(recommended.get("turn_slowdown_start_radps"), default=configured_start)
    measured_full = _first_number(recommended.get("turn_slowdown_full_radps"), default=configured_full)
    if limit_mode == "use-calibration":
        turn_start = min(max(0.0, measured_start), yaw_rate) if yaw_rate > 0.0 else 0.0
        turn_full = min(max(0.0, measured_full), yaw_rate) if yaw_rate > 0.0 else 0.0
    else:
        turn_start = min(configured_start, measured_start, yaw_rate) if yaw_rate > 0.0 else 0.0
        turn_full = min(configured_full, measured_full, yaw_rate) if yaw_rate > 0.0 else 0.0
    if yaw_rate > 0.0 and turn_full <= turn_start:
        fallback_full = measured_full if limit_mode == "use-calibration" else configured_full
        turn_full = min(yaw_rate, max(turn_start + 1e-6, fallback_full))

    values: dict[str, float | list[dict[str, float | str]]] = {
        "max_forward_mps": speed,
        "min_forward_mps": minimum_speed,
        "max_reverse_mps": reverse,
        "max_yaw_rate_radps": yaw_rate,
        "turn_slowdown_start_radps": turn_start,
        "turn_slowdown_full_radps": turn_full,
        "turn_slowdown_min_forward_mps": min(
            float(configured_limits.get("turn_slowdown_min_forward_mps", minimum_speed)),
            speed,
        ),
    }
    recovery_hints = command_hints(calibration.get("recovery_safe_commands", []))
    if recovery_hints:
        values["recovery_safe_commands"] = recovery_hints
    return values


def calibration_summary_for_navigation(calibration: dict) -> dict[str, object]:
    """Expose only scalar calibration metadata to navigation config."""
    recommended = calibration.get("recommended_safe_limits", {}) if isinstance(calibration, dict) else {}
    return {
        "schema_version": calibration.get("schema_version"),
        "source": calibration.get("source"),
        "status": calibration.get("status"),
        "cache_key": calibration.get("cache_key"),
        "policy": calibration.get("policy"),
        "ground_truth_used_for_calibration_metrics": bool(calibration.get("ground_truth_used_for_calibration_metrics")),
        "max_safe_vx": recommended.get("max_safe_vx", calibration.get("selected_max_forward_mps")),
        "max_safe_wz": recommended.get("max_safe_wz", calibration.get("command_limits", {}).get("max_yaw_rate_radps")),
    }


def command_hints(rows: object) -> list[dict[str, float | str]]:
    hints: list[dict[str, float | str]] = []
    if not isinstance(rows, list):
        return hints
    for row in rows[:6]:
        if not isinstance(row, dict):
            continue
        hints.append(
            {
                "group": str(row.get("group", "")),
                "cmd_vx": float(row.get("cmd_vx", 0.0)),
                "cmd_wz": float(row.get("cmd_wz", 0.0)),
            }
        )
    return hints


def _first_number(*values: object, default: float) -> float:
    for value in values:
        if value is None:
            continue
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return float(default)


if __name__ == "__main__":
    raise SystemExit(main())

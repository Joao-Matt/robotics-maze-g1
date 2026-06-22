#!/usr/bin/env python3
"""Write per-run locomotion limits for Nav2 navigation."""

from __future__ import annotations

from pathlib import Path
import argparse
import hashlib
import json
import shutil

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=Path("third_party/unitree_rl_gym"))
    parser.add_argument("--config", type=Path, default=Path("configs/default.yaml"))
    parser.add_argument("--cache-root", type=Path, default=Path("runs/calibration"))
    parser.add_argument(
        "--calibration",
        type=Path,
        default=None,
        help="Optional measured locomotion_calibration.json to copy into this navigation run.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    target = args.output_dir / "locomotion_calibration.json"
    if args.calibration is not None:
        values = json.loads(args.calibration.read_text(encoding="utf-8"))
        if not isinstance(values, dict):
            raise ValueError(f"Calibration must be a JSON object: {args.calibration}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.calibration, target)
        _update_manifest(args.output_dir, values, source=str(args.calibration))
        print(target)
        return 0

    values = _static_calibration(args)
    target.write_text(json.dumps(values, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _update_manifest(args.output_dir, values, source="native_policy_static_limits")
    print(target)
    return 0


def _static_calibration(args: argparse.Namespace) -> dict:
    config = yaml.safe_load(args.config.read_text(encoding="utf-8")) if args.config.is_file() else {}
    limits = config.get("nav2_navigation", {})
    policy = str(limits.get("locomotion_policy", "unitree_rl_gym_native"))
    if policy != "unitree_rl_gym_native":
        raise ValueError(f"Production supports only unitree_rl_gym_native, got {policy!r}.")
    policy_path = args.unitree_rl_gym_repo / "deploy" / "pre_train" / "g1" / "motion.pt"
    model_path = args.unitree_rl_gym_repo / "resources" / "robots" / "g1_description" / "g1_12dof.xml"
    deploy_config = args.unitree_rl_gym_repo / "deploy" / "deploy_mujoco" / "configs" / "g1.yaml"
    missing = [path for path in (policy_path, model_path, deploy_config) if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing Unitree RL Gym assets: "
            + ", ".join(str(path) for path in missing)
            + ". Run `make fetch-unitree-rl-gym-policy`."
        )
    policy_hash = hashlib.sha256(policy_path.read_bytes()).hexdigest()
    model_hash = hashlib.sha256(model_path.read_bytes() + deploy_config.read_bytes()).hexdigest()
    digest = hashlib.sha256((policy + policy_hash + model_hash).encode("utf-8")).hexdigest()[:16]
    selected = float(limits.get("max_forward_mps", 1.0))
    yaw = float(limits.get("max_yaw_rate_radps", 1.0))
    return {
        "schema_version": 1,
        "policy": policy,
        "policy_hash": policy_hash,
        "model_config_hash": model_hash,
        "cache_key": digest,
        "selected_max_forward_mps": selected,
        "command_limits": {
            "min_forward_mps": float(limits.get("min_forward_mps", 0.1)),
            "max_forward_mps": selected,
            "max_reverse_mps": float(limits.get("max_reverse_mps", -0.4)),
            "min_yaw_rate_radps": float(limits.get("min_yaw_rate_radps", 0.0)),
            "max_yaw_rate_radps": yaw,
        },
        "recommended_safe_limits": {
            "max_safe_vx": selected,
            "max_safe_wz": yaw,
            "turn_slowdown_start_radps": float(limits.get("turn_slowdown_start_radps", 0.45)),
            "turn_slowdown_full_radps": float(limits.get("turn_slowdown_full_radps", 1.10)),
        },
        "status": "passed",
        "source": "native_policy_static_limits",
        "criteria": {
            "native_policy_usable_yaw_rate_radps": 1.0,
            "native_policy_forward_range_mps": [0.1, 1.0],
            "native_policy_reverse_mps": -0.4,
        },
    }


def _update_manifest(output_dir: Path, values: dict, *, source: str) -> None:
    manifest = output_dir / "run_manifest.json"
    if not manifest.is_file():
        return
    manifest_values = json.loads(manifest.read_text(encoding="utf-8"))
    manifest_values["policy_sha256"] = values.get("policy_hash")
    manifest_values["model_config_sha256"] = values.get("model_config_hash")
    manifest_values["locomotion_policy"] = values.get("policy")
    manifest_values["locomotion_calibration_source"] = source
    manifest.write_text(json.dumps(manifest_values, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())

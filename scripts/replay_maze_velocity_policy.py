#!/usr/bin/env python3
"""Replay one trained direct MuJoCo velocity-controller episode."""

from __future__ import annotations

from pathlib import Path
import argparse
import csv
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rl_velocity.config import load_rl_config  # noqa: E402
from rl_velocity.metrics import write_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--vec-normalize", type=Path, default=None)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--rl-config", type=Path, default=PROJECT_ROOT / "configs" / "rl_velocity_controller.yaml")
    parser.add_argument("--run-root", type=Path, default=PROJECT_ROOT / "runs" / "rl_velocity")
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--stage", default=None)
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=PROJECT_ROOT / "third_party" / "unitree_rl_gym")
    parser.add_argument("--locomotion-calibration", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    from rl_velocity.sb3_utils import build_vec_env

    load_rl_config(args.rl_config)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_root / "replay" / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    env = build_vec_env(
        config_path=args.config,
        rl_config_path=args.rl_config,
        run_dir=run_dir,
        stage=args.stage,
        seed=args.seed,
        num_envs=1,
        unitree_rl_gym_repo=args.unitree_rl_gym_repo,
        training=False,
        normalize_observations=False,
        normalize_rewards=False,
        record_trajectory=True,
        locomotion_calibration_path=args.locomotion_calibration,
    )
    vec_path = _find_vec_normalize(args.checkpoint, args.vec_normalize)
    if vec_path is not None and vec_path.exists():
        env = VecNormalize.load(str(vec_path), env)
        env.training = False
        env.norm_reward = False
    model = PPO.load(str(args.checkpoint), env=env)

    obs = env.reset()
    metrics = {}
    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, _reward, done, infos = env.step(action)
        if bool(done[0]):
            metrics = dict(infos[0].get("episode_metrics", {}))
            break
    trajectory = env.get_attr("trajectory")[0]
    if trajectory:
        _write_trajectory(run_dir / "trajectory.csv", trajectory)
    try:
        env.env_method("save_final_render", run_dir / "final_render.png")
    except Exception as exc:
        metrics["render_error"] = str(exc)
    env.close()
    metrics.update({"checkpoint": str(args.checkpoint), "seed": args.seed, "stage": args.stage})
    write_json(run_dir / "summary.json", metrics)
    print(f"summary: {run_dir / 'summary.json'}")
    print(f"trajectory: {run_dir / 'trajectory.csv'}")
    print(f"final_render: {run_dir / 'final_render.png'}")
    return 0


def _write_trajectory(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _find_vec_normalize(checkpoint: Path, explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    candidates = [
        checkpoint.with_name("vec_normalize.pkl"),
        checkpoint.parent / "vec_normalize.pkl",
        checkpoint.parent.parent / "vec_normalize.pkl",
        checkpoint.with_name(checkpoint.stem + "_vecnormalize.pkl"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    matches = sorted(checkpoint.parent.glob("*vecnormalize*.pkl"))
    return matches[-1] if matches else None


if __name__ == "__main__":
    raise SystemExit(main())

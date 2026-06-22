#!/usr/bin/env python3
"""Evaluate and rank direct MuJoCo PPO velocity-controller checkpoints."""

from __future__ import annotations

from pathlib import Path
import argparse
import sys
import time

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rl_velocity.config import load_rl_config  # noqa: E402
from rl_velocity.metrics import rank_checkpoint_summaries, summarize_metrics, write_json, write_metrics_csv  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, nargs="*", default=[])
    parser.add_argument("--checkpoint-dir", type=Path, default=None)
    parser.add_argument("--vec-normalize", type=Path, default=None)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--rl-config", type=Path, default=PROJECT_ROOT / "configs" / "rl_velocity_controller.yaml")
    parser.add_argument("--run-root", type=Path, default=PROJECT_ROOT / "runs" / "rl_velocity")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--episode-suite", type=Path, default=None)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--stage", default=None)
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=PROJECT_ROOT / "third_party" / "unitree_rl_gym")
    parser.add_argument("--locomotion-calibration", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
    from rl_velocity.sb3_utils import build_vec_env

    checkpoints = _checkpoint_paths(args.checkpoint, args.checkpoint_dir)
    if not checkpoints:
        raise SystemExit("No checkpoints supplied. Use --checkpoint or --checkpoint-dir.")
    rl_config = load_rl_config(args.rl_config)
    eval_config = rl_config.get("evaluation", {})
    suite = load_episode_suite(args.episode_suite) if args.episode_suite else None
    episode_plan = suite["episodes"] if suite else None
    episodes = int(args.episodes or (len(episode_plan) if episode_plan else eval_config.get("episodes", 20)))
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_root / "eval" / timestamp
    run_dir.mkdir(parents=True, exist_ok=True)
    if suite:
        write_json(run_dir / "episode_suite.json", suite)

    all_rows = []
    summaries = {}
    for checkpoint in checkpoints:
        checkpoint_rows = []
        env = build_vec_env(
            config_path=args.config,
            rl_config_path=args.rl_config,
            run_dir=run_dir / checkpoint.stem,
            stage=args.stage,
            seed=args.seed,
            num_envs=1,
            unitree_rl_gym_repo=args.unitree_rl_gym_repo,
            training=False,
            normalize_observations=False,
            normalize_rewards=False,
            episode_plan=episode_plan,
            locomotion_calibration_path=args.locomotion_calibration,
        )
        vec_path = _find_vec_normalize(checkpoint, args.vec_normalize)
        if vec_path is not None and vec_path.exists():
            env = VecNormalize.load(str(vec_path), env)
            env.training = False
            env.norm_reward = False
        model = PPO.load(str(checkpoint), env=env)
        obs = env.reset()
        finished = 0
        while finished < episodes:
            action, _ = model.predict(obs, deterministic=True)
            obs, _reward, done, infos = env.step(action)
            if bool(done[0]):
                metrics = dict(infos[0].get("episode_metrics", {}))
                if metrics:
                    metrics["checkpoint"] = str(checkpoint)
                    checkpoint_rows.append(metrics)
                    all_rows.append(metrics)
                    finished += 1
                    if finished == episodes or finished % 10 == 0:
                        print(f"{checkpoint.name}: evaluated {finished}/{episodes} episodes", flush=True)
        env.close()
        summaries[str(checkpoint)] = summarize_metrics(checkpoint_rows)

    ranking = rank_checkpoint_summaries(
        summaries,
        min_success_rate=float(eval_config.get("min_success_rate", 0.85)),
        max_crash_or_fall_rate=float(eval_config.get("max_crash_or_fall_rate", 0.05)),
    )
    write_metrics_csv(run_dir / "episode_metrics.csv", all_rows)
    write_json(run_dir / "episode_metrics.json", all_rows)
    write_json(run_dir / "checkpoint_summary.json", summaries)
    write_json(run_dir / "checkpoint_ranking.json", ranking)
    write_json(run_dir / "grouped_summary.json", grouped_summary(all_rows))
    print(f"episode_metrics: {run_dir / 'episode_metrics.csv'}")
    print(f"grouped_summary: {run_dir / 'grouped_summary.json'}")
    print(f"checkpoint_ranking: {run_dir / 'checkpoint_ranking.json'}")
    if ranking:
        print(f"best_checkpoint: {ranking[0]['checkpoint']}")
        print(
            "best_summary: "
            f"success_rate={ranking[0].get('success_rate', 0.0):.3f}, "
            f"avg_goal_time_s={ranking[0].get('avg_goal_time_s')}, "
            f"collision_count={ranking[0].get('collision_count', 0)}, "
            f"fall_count={ranking[0].get('fall_count', 0)}"
        )
    return 0


def _checkpoint_paths(explicit: list[Path], checkpoint_dir: Path | None) -> list[Path]:
    paths = [path for path in explicit if path.exists()]
    if checkpoint_dir is not None:
        paths.extend(sorted(checkpoint_dir.glob("*.zip")))
        paths.extend(sorted((checkpoint_dir / "checkpoints").glob("*.zip")))
    seen = set()
    unique = []
    for path in paths:
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(path)
    return unique


def load_episode_suite(path: Path) -> dict:
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict) or not isinstance(values.get("episodes"), list):
        raise ValueError(f"Episode suite must be a mapping with an episodes list: {path}")
    episodes = []
    for index, raw in enumerate(values["episodes"]):
        if not isinstance(raw, dict):
            raise ValueError(f"Episode suite row {index} must be a mapping.")
        row = dict(raw)
        row.setdefault("id", f"episode_{index:03d}")
        row.setdefault("suite_index", index)
        row["stage"] = str(row["stage"])
        row["seed"] = int(row["seed"])
        row["corridor_width_m"] = float(row["corridor_width_m"])
        if row["corridor_width_m"] < 2.0 or row["corridor_width_m"] > 4.0:
            raise ValueError(f"Episode suite row {index} corridor_width_m must be in [2.0, 4.0].")
        episodes.append(row)
    return {**values, "episodes": episodes}


def grouped_summary(rows: list[dict]) -> dict:
    groups: dict[str, list[dict]] = {}
    for row in rows:
        for key in (
            f"stage={row.get('stage', '')}",
            f"corridor_width_m={row.get('corridor_width_m', '')}",
            f"stage={row.get('stage', '')}|corridor_width_m={row.get('corridor_width_m', '')}",
            f"failure_phase={row.get('failure_phase', '') or 'none'}",
            f"final_status={row.get('final_status', '')}",
        ):
            groups.setdefault(key, []).append(row)
    return {key: summarize_metrics(value) for key, value in sorted(groups.items())}


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

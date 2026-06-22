#!/usr/bin/env python3
"""Train a direct MuJoCo PPO velocity controller for G1 maze navigation."""

from __future__ import annotations

from pathlib import Path
import argparse
import json
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rl_velocity.config import load_rl_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "default.yaml")
    parser.add_argument("--rl-config", type=Path, default=PROJECT_ROOT / "configs" / "rl_velocity_controller.yaml")
    parser.add_argument("--run-root", type=Path, default=PROJECT_ROOT / "runs" / "rl_velocity")
    parser.add_argument("--total-timesteps", type=int, default=None)
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--stage", default=None, help="Optional fixed curriculum stage name.")
    parser.add_argument("--unitree-rl-gym-repo", type=Path, default=PROJECT_ROOT / "third_party" / "unitree_rl_gym")
    parser.add_argument("--progress", action="store_true", help="Show the SB3 progress bar.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from rl_velocity.sb3_utils import build_vec_env

    rl_config = load_rl_config(args.rl_config)
    ppo_config = rl_config.get("ppo", {})
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.run_root / "train" / timestamp
    checkpoint_dir = run_dir / "checkpoints"
    run_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    normalize_obs = bool(ppo_config.get("normalize_observations", True))
    normalize_rewards = bool(ppo_config.get("normalize_rewards", True))
    env = build_vec_env(
        config_path=args.config,
        rl_config_path=args.rl_config,
        run_dir=run_dir,
        stage=args.stage,
        seed=args.seed,
        num_envs=max(1, int(args.num_envs)),
        unitree_rl_gym_repo=args.unitree_rl_gym_repo,
        training=True,
        normalize_observations=normalize_obs,
        normalize_rewards=normalize_rewards,
    )

    hidden_sizes = list(ppo_config.get("policy_hidden_sizes", [256, 256]))
    policy_kwargs = {"net_arch": {"pi": hidden_sizes, "vf": hidden_sizes}}
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=float(ppo_config.get("learning_rate", 3e-4)),
        n_steps=int(ppo_config.get("n_steps", 512)),
        batch_size=int(ppo_config.get("batch_size", 256)),
        gamma=float(ppo_config.get("gamma", 0.99)),
        gae_lambda=float(ppo_config.get("gae_lambda", 0.95)),
        clip_range=float(ppo_config.get("clip_range", 0.2)),
        ent_coef=float(ppo_config.get("ent_coef", 0.005)),
        vf_coef=float(ppo_config.get("vf_coef", 0.5)),
        max_grad_norm=float(ppo_config.get("max_grad_norm", 0.5)),
        policy_kwargs=policy_kwargs,
        verbose=1,
        seed=args.seed,
        tensorboard_log=str(run_dir / "tensorboard"),
    )

    checkpoint = CheckpointCallback(
        save_freq=max(1, int(ppo_config.get("checkpoint_freq", 25000)) // max(1, int(args.num_envs))),
        save_path=str(checkpoint_dir),
        name_prefix="ppo_maze_velocity",
        save_replay_buffer=False,
        save_vecnormalize=True,
    )
    total_timesteps = int(args.total_timesteps or ppo_config.get("total_timesteps", 200000))
    metadata = {
        "config": str(args.config),
        "rl_config": str(args.rl_config),
        "run_dir": str(run_dir),
        "seed": args.seed,
        "stage": args.stage,
        "num_envs": int(args.num_envs),
        "total_timesteps": total_timesteps,
        "unitree_rl_gym_repo": str(args.unitree_rl_gym_repo),
        "ppo": ppo_config,
    }
    (run_dir / "run_config.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    model.learn(total_timesteps=total_timesteps, callback=checkpoint, progress_bar=bool(args.progress))
    final_model = run_dir / "final_model.zip"
    model.save(str(final_model))
    if hasattr(env, "save"):
        env.save(str(run_dir / "vec_normalize.pkl"))
    env.close()
    print(f"trained_model: {final_model}")
    print(f"vec_normalize: {run_dir / 'vec_normalize.pkl'}")
    print(f"run_dir: {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

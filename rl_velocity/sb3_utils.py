"""Stable-Baselines3 helpers for the MuJoCo velocity-controller scripts."""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from rl_velocity.env import G1MazeVelocityEnv


def make_env_factory(
    *,
    config_path: Path,
    rl_config_path: Path,
    run_dir: Path,
    stage: str | None,
    seed: int,
    rank: int,
    unitree_rl_gym_repo: Path | None,
    training: bool,
    record_trajectory: bool = False,
    episode_plan: list[dict] | None = None,
) -> Callable[[], G1MazeVelocityEnv]:
    """Return a picklable factory for one SB3 vector-env worker."""

    def _factory() -> G1MazeVelocityEnv:
        return G1MazeVelocityEnv(
            config_path=config_path,
            rl_config_path=rl_config_path,
            run_dir=run_dir / f"env-{rank}",
            stage=stage,
            seed=seed + rank,
            unitree_rl_gym_repo=unitree_rl_gym_repo,
            training=training,
            record_trajectory=record_trajectory,
            episode_plan=episode_plan,
        )

    return _factory


def build_vec_env(
    *,
    config_path: Path,
    rl_config_path: Path,
    run_dir: Path,
    stage: str | None,
    seed: int,
    num_envs: int,
    unitree_rl_gym_repo: Path | None,
    training: bool,
    normalize_observations: bool,
    normalize_rewards: bool,
    record_trajectory: bool = False,
    episode_plan: list[dict] | None = None,
):
    """Build a monitored SB3 vector environment."""
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor, VecNormalize

    factories = [
        make_env_factory(
            config_path=config_path,
            rl_config_path=rl_config_path,
            run_dir=run_dir,
            stage=stage,
            seed=seed,
            rank=index,
            unitree_rl_gym_repo=unitree_rl_gym_repo,
            training=training,
            record_trajectory=record_trajectory,
            episode_plan=episode_plan,
        )
        for index in range(num_envs)
    ]
    vec_env = DummyVecEnv(factories) if num_envs == 1 else SubprocVecEnv(factories, start_method="spawn")
    vec_env = VecMonitor(vec_env)
    if normalize_observations or normalize_rewards:
        vec_env = VecNormalize(
            vec_env,
            norm_obs=normalize_observations,
            norm_reward=normalize_rewards,
            clip_obs=10.0,
        )
        vec_env.training = training
        vec_env.norm_reward = normalize_rewards and training
    return vec_env

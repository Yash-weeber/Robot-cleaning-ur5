"""
SAC_baseline/callbacks.py
=========================
Stable-Baselines3 callbacks for the ball-clearing SAC training run.

EpisodeLoggerCallback
    Writes one CSV row per completed episode:
        episode, timestep, reward, balls_swept, balls_remaining,
        initial_count, waypoints, ee_pos_x, ee_pos_y, ee_pos_z

CheckpointCallback  (thin wrapper around SB3's built-in one)
    Re-exported here so train_sac.py only needs to import from this module.
"""

from __future__ import annotations

import csv
import os
from typing import Any

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback  # noqa: F401


class EpisodeLoggerCallback(BaseCallback):
    """
    Logs per-episode metrics to a CSV file and optionally to stdout.

    Parameters
    ----------
    log_path : str
        Full path to the CSV file (parent directory is created if needed).
    print_freq : int
        Print a summary line every ``print_freq`` episodes (0 = never).
    verbose : int
        SB3 verbosity level (passed to BaseCallback).
    """

    def __init__(
        self,
        log_path: str,
        print_freq: int = 10,
        verbose: int = 0,
    ):
        super().__init__(verbose=verbose)
        self.log_path   = log_path
        self.print_freq = print_freq

        self._episode_count = 0
        self._csv_file      = None
        self._csv_writer    = None

    # ------------------------------------------------------------------
    def _on_training_start(self) -> None:
        log_dir = os.path.dirname(self.log_path)
        if log_dir:                                    # guard against bare filenames
            os.makedirs(log_dir, exist_ok=True)
        self._csv_file = open(self.log_path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "episode", "timestep", "reward",
            "balls_swept", "balls_remaining", "initial_count",
            "waypoints", "ee_pos_x", "ee_pos_y", "ee_pos_z",
        ])

    def _on_step(self) -> bool:
        """Called after every env.step().  We look at 'dones' to detect episode ends."""
        infos   = self.locals.get("infos",   [])
        dones   = self.locals.get("dones",   [])
        rewards = self.locals.get("rewards", [])

        for idx, (done, info) in enumerate(zip(dones, infos)):
            if not done:
                continue   # episode not finished yet

            self._episode_count += 1

            # SB3 ≥ 2.0 + gymnasium: when the VecEnv auto-resets after
            # terminated=True, the real terminal info dict is stashed under
            # info["final_info"].  Fall back to info itself for older setups
            # or when Monitor is the outermost wrapper without a VecEnv.
            terminal_info = info.get("final_info", info)

            # Episode reward: prefer Monitor's cumulative "episode.r",
            # fall back to the raw step reward from self.locals.
            ep_stats = terminal_info.get("episode", {})
            step_r   = rewards[idx] if idx < len(rewards) else 0.0
            reward   = float(ep_stats.get("r", step_r))

            balls_swept     = terminal_info.get("balls_swept",     0)
            balls_remaining = terminal_info.get("balls_remaining", 0)
            initial_count   = terminal_info.get("initial_count",  0)
            waypoints       = terminal_info.get("waypoints",       0)
            ee_pos          = terminal_info.get("ee_pos",          [0.0, 0.0, 0.0])

            self._csv_writer.writerow([
                self._episode_count,
                self.num_timesteps,
                f"{reward:.6f}",
                balls_swept,
                balls_remaining,
                initial_count,
                waypoints,
                f"{ee_pos[0]:.4f}",
                f"{ee_pos[1]:.4f}",
                f"{ee_pos[2]:.4f}",
            ])
            self._csv_file.flush()

            if self.print_freq > 0 and self._episode_count % self.print_freq == 0:
                print(
                    f"[SAC] ep={self._episode_count:5d}  "
                    f"t={self.num_timesteps:7d}  "
                    f"swept={balls_swept}/{initial_count}  "
                    f"reward={reward:.4f}"
                )

        return True   # returning False would abort training

    def _on_training_end(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()

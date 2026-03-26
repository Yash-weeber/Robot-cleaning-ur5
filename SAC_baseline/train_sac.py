"""
SAC_baseline/train_sac.py
=========================
Train a Soft Actor-Critic (SAC) agent on the UR5e ball-clearing task
using Stable-Baselines3.

Usage
-----
    # from the repo root:
    python -m SAC_baseline.train_sac                          # default config
    python -m SAC_baseline.train_sac --config config/sac_config.yaml
    python -m SAC_baseline.train_sac --config config/sac_config.yaml \\
                                     --run-name my_run \\
                                     --total-timesteps 5000

Key design decisions
--------------------
* One SB3 "timestep"   = one complete DMP trajectory = one RL episode.
  (BallClearingEnv.step() always returns terminated=True.)
* ``learning_starts``   controls how many random rollouts fill the replay
  buffer before gradient updates begin (set in sac_config.yaml).
* All hyper-parameters are read from the YAML config so that the same
  config file drives both the LLM-runner and this SAC baseline.
* Model checkpoints and episode logs are written under
    Results/SAC_baseline/<run_name>/
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# ── project root on path ───────────────────────────────────────────────────
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor

from config.loader import load_config
from SAC_baseline.cleaning_env import BallClearingEnv
from SAC_baseline.callbacks import EpisodeLoggerCallback, CheckpointCallback


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Train SAC on the UR5e ball-clearing task."
    )
    p.add_argument(
        "--config", "-c",
        default="config/sac_config.yaml",
        help="Path to YAML config file (default: config/sac_config.yaml).",
    )
    p.add_argument(
        "--run-name", "-n",
        default=None,
        help="Sub-directory name under Results/SAC_baseline/ for this run. "
             "Defaults to a timestamp string.",
    )
    p.add_argument(
        "--total-timesteps", "-t",
        type=int,
        default=None,
        help="Override total training timesteps from config.",
    )
    p.add_argument(
        "--checkpoint-freq", "-k",
        type=int,
        default=100,
        help="Save a checkpoint every N timesteps (default: 100).",
    )
    p.add_argument(
        "--grid-reward",
        action="store_true",
        default=False,
        help="Add shaped grid-uniformity bonus to the reward.",
    )
    p.add_argument(
        "--w-bound",
        type=float,
        default=None,
        help="Override symmetric bound for DMP weight action space.",
    )
    p.add_argument(
        "--verbose", "-v",
        type=int,
        default=1,
        help="Verbosity: 0=silent, 1=SB3 progress, 2=episode details.",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _parse_args()

    # ── load config ────────────────────────────────────────────────────────
    config_path = args.config
    # If the path doesn't exist as-is, try prepending "config/"
    if not os.path.exists(config_path):
        alt = os.path.join("config", config_path)
        if os.path.exists(alt):
            config_path = alt
        else:
            raise FileNotFoundError(
                f"Config not found: '{config_path}' (also tried '{alt}')."
            )

    config = load_config(config_path)
    sac_cfg = config.get("sac_params", {})

    # ── output directories ─────────────────────────────────────────────────
    run_name = args.run_name or time.strftime("run_%Y%m%d_%H%M%S")
    out_dir  = os.path.join(
        config["simulation"].get("base_dir", "Results"),
        "SAC_baseline",
        run_name,
    )
    ckpt_dir = os.path.join(out_dir, "checkpoints")
    log_csv  = os.path.join(out_dir, "episode_log.csv")
    tb_dir   = os.path.join(out_dir, "tensorboard")
    os.makedirs(ckpt_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f"  SAC baseline  —  run: {run_name}")
    print(f"  Output:  {out_dir}")
    print(f"{'='*60}\n")

    # ── build environment ──────────────────────────────────────────────────
    print("Initialising BallClearingEnv …")
    env = BallClearingEnv(
        config         = config,
        w_bound        = args.w_bound,
        use_grid_reward= args.grid_reward,
        fast_physics   = True,                  # no real-time sleep during training
        verbose        = max(0, args.verbose - 1),
    )
    env = Monitor(env, filename=os.path.join(out_dir, "monitor"))

    print(f"  Observation space : {env.observation_space}")
    print(f"  Action space      : {env.action_space}")
    print(f"  n_bfs             : {env.unwrapped.n_bfs}")
    print(f"  action_dim        : {env.unwrapped.action_dim}")
    print(f"  w_bound           : ±{env.unwrapped.w_bound:.1f}\n")

    # ── SAC hyper-parameters from config ──────────────────────────────────
    total_timesteps = (
        args.total_timesteps
        or int(sac_cfg.get("num_iterations", 400))
            * int(sac_cfg.get("episodes_per_iteration", 1))
    )
    learning_starts = int(sac_cfg.get("learning_starts", 20))
    batch_size      = int(sac_cfg.get("batch_size",  256))
    buffer_size     = int(sac_cfg.get("buffer_size", 100_000))
    hidden_dim      = int(sac_cfg.get("hidden_dim",  256))
    gamma           = float(sac_cfg.get("gamma",     0.99))
    tau             = float(sac_cfg.get("tau",        0.005))
    lr_actor        = float(sac_cfg.get("lr_actor",  3e-4))
    lr_critic       = float(sac_cfg.get("lr_critic", 3e-4))
    auto_entropy    = bool(sac_cfg.get("automatic_entropy_tuning", True))
    target_entropy  = sac_cfg.get("target_entropy", "auto")
    if isinstance(target_entropy, (int, float)):
        target_entropy = float(target_entropy)

    # SB3 SAC uses a single lr for actor + critic;
    # we honour lr_actor and note any difference.
    if abs(lr_actor - lr_critic) > 1e-9:
        print(
            f"  Note: SB3 SAC uses one learning_rate parameter. "
            f"Using lr_actor={lr_actor}. "
            f"Set both to the same value in config to silence this."
        )

    policy_kwargs = dict(net_arch=[hidden_dim, hidden_dim])

    print(f"  total_timesteps : {total_timesteps}")
    print(f"  learning_starts : {learning_starts}")
    print(f"  batch_size      : {batch_size}")
    print(f"  buffer_size     : {buffer_size}")
    print(f"  hidden_dim      : {hidden_dim}")
    print(f"  gamma           : {gamma}")
    print(f"  tau             : {tau}")
    print(f"  lr              : {lr_actor}")
    print(f"  auto_entropy    : {auto_entropy}")
    print(f"  target_entropy  : {target_entropy}\n")

    # ── build SAC model ────────────────────────────────────────────────────
    model = SAC(
        policy          = "MlpPolicy",
        env             = env,
        learning_rate   = lr_actor,
        buffer_size     = buffer_size,
        learning_starts = learning_starts,
        batch_size      = batch_size,
        tau             = tau,
        gamma           = gamma,
        policy_kwargs   = policy_kwargs,
        ent_coef        = "auto" if auto_entropy else 0.1,
        target_entropy  = target_entropy if auto_entropy else "auto",
        tensorboard_log = tb_dir,
        verbose         = args.verbose,
        seed            = int(config.get("llm_settings", {}).get("seed_number", 42)),
    )

    # ── callbacks ─────────────────────────────────────────────────────────
    episode_logger = EpisodeLoggerCallback(
        log_path  = log_csv,
        print_freq= 10,
        verbose   = 0,
    )
    checkpointer = CheckpointCallback(
        save_freq  = args.checkpoint_freq,
        save_path  = ckpt_dir,
        name_prefix= "sac_ball_clearing",
        verbose    = 0,
    )

    # ── train ─────────────────────────────────────────────────────────────
    print(f"Starting SAC training for {total_timesteps} timesteps …\n")
    t0 = time.time()
    model.learn(
        total_timesteps = total_timesteps,
        callback        = [episode_logger, checkpointer],
        progress_bar    = True,
    )
    elapsed = time.time() - t0
    print(f"\nTraining complete in {elapsed/60:.1f} min.")

    # ── save final model ───────────────────────────────────────────────────
    final_path = os.path.join(out_dir, "sac_ball_clearing_final")
    model.save(final_path)
    print(f"Final model saved → {final_path}.zip")
    print(f"Episode log       → {log_csv}")
    print(f"TensorBoard       → tensorboard --logdir {tb_dir}\n")

    env.close()


if __name__ == "__main__":
    main()

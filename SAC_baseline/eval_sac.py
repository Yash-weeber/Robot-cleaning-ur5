"""
SAC_baseline/eval_sac.py
========================
Load a trained SAC checkpoint and run evaluation episodes on the
UR5e ball-clearing task.

Usage
-----
    # from the repo root:
    python -m SAC_baseline.eval_sac \\
        --model Results/SAC_baseline/run_XYZ/sac_ball_clearing_final.zip \\
        --config config/sac_config.yaml \\
        --episodes 20

Output
------
  * Per-episode summary printed to stdout.
  * CSV with columns:
        episode, reward, balls_swept, balls_remaining,
        initial_count, waypoints, ee_pos_x, ee_pos_y, ee_pos_z
    written to the same directory as the loaded model checkpoint.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import numpy as np
from stable_baselines3 import SAC

from config.loader import load_config
from SAC_baseline.cleaning_env import BallClearingEnv


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Evaluate a trained SAC model on the ball-clearing task."
    )
    p.add_argument(
        "--model", "-m",
        required=True,
        help="Path to the saved SB3 SAC model (.zip).",
    )
    p.add_argument(
        "--config", "-c",
        default="config/sac_config.yaml",
        help="YAML config file (default: config/sac_config.yaml).",
    )
    p.add_argument(
        "--episodes", "-e",
        type=int,
        default=20,
        help="Number of evaluation episodes (default: 20).",
    )
    p.add_argument(
        "--deterministic",
        action="store_true",
        default=True,
        help="Use deterministic (mean) policy (default: True).",
    )
    p.add_argument(
        "--stochastic",
        action="store_true",
        default=False,
        help="Override --deterministic and sample stochastically.",
    )
    p.add_argument(
        "--w-bound",
        type=float,
        default=None,
        help="Override w_bound used to build the env (must match training).",
    )
    p.add_argument(
        "--render",
        action="store_true",
        default=False,
        help="Attempt to render each step (requires a display).",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _parse_args()
    deterministic = args.deterministic and not args.stochastic

    # ── config ────────────────────────────────────────────────────────────
    config_path = args.config
    if not os.path.exists(config_path):
        alt = os.path.join("config", config_path)
        if os.path.exists(alt):
            config_path = alt
        else:
            raise FileNotFoundError(
                f"Config not found: '{config_path}' (also tried '{alt}')."
            )
    config = load_config(config_path)

    # ── environment ───────────────────────────────────────────────────────
    print("Initialising BallClearingEnv …")
    env = BallClearingEnv(
        config      = config,
        w_bound     = args.w_bound,
        fast_physics= not args.render,
        verbose     = 1,
    )

    # ── model ─────────────────────────────────────────────────────────────
    model_path = args.model
    if not model_path.endswith(".zip"):
        model_path += ".zip"
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model checkpoint not found: {model_path}")

    print(f"Loading model: {model_path}")
    model = SAC.load(model_path, env=env)

    # ── output CSV ────────────────────────────────────────────────────────
    out_dir  = os.path.dirname(model_path)
    csv_path = os.path.join(
        out_dir,
        f"eval_{time.strftime('%Y%m%d_%H%M%S')}.csv",
    )

    print(f"\nRunning {args.episodes} evaluation episode(s) …")
    print(f"  deterministic={deterministic}")
    print(f"  results → {csv_path}\n")

    rewards       = []
    balls_swepts  = []

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "episode", "reward",
            "balls_swept", "balls_remaining", "initial_count",
            "waypoints", "ee_pos_x", "ee_pos_y", "ee_pos_z",
        ])

        for ep in range(1, args.episodes + 1):
            obs, _ = env.reset()
            action, _ = model.predict(obs, deterministic=deterministic)
            obs, reward, terminated, truncated, info = env.step(action)

            if args.render:
                env.render()

            bs  = info.get("balls_swept",     0)
            br  = info.get("balls_remaining", 0)
            ic  = info.get("initial_count",   0)
            wp  = info.get("waypoints",       0)
            ep_ = info.get("ee_pos",          [0.0, 0.0, 0.0])

            rewards.append(reward)
            balls_swepts.append(bs)

            writer.writerow([
                ep, f"{reward:.6f}",
                bs, br, ic, wp,
                f"{ep_[0]:.4f}", f"{ep_[1]:.4f}", f"{ep_[2]:.4f}",
            ])
            f.flush()

            print(
                f"  ep={ep:4d}  swept={bs:4d}/{ic}  "
                f"reward={reward:.4f}  waypoints={wp}"
            )

    # ── summary ───────────────────────────────────────────────────────────
    print(f"\n{'─'*50}")
    print(f"  Episodes       : {args.episodes}")
    print(f"  Mean reward    : {np.mean(rewards):.4f}  ±{np.std(rewards):.4f}")
    print(f"  Mean swept     : {np.mean(balls_swepts):.1f}")
    print(f"  Best  swept    : {np.max(balls_swepts)}")
    print(f"  Worst swept    : {np.min(balls_swepts)}")
    print(f"  Results CSV    : {csv_path}")
    print(f"{'─'*50}\n")

    env.close()


if __name__ == "__main__":
    main()

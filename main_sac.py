# import sys
# import os
# import argparse
# import torch
# import numpy as np
# import random
# import traceback

# # Ensure project root is in path for imports
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# # Importing your internal project utilities to maintain structure consistency
# from config.loader import load_config, setup_logging_dirs
# from runner.sac_main_runner import run_sac_optimization

# def _parse_args():
#     parser = argparse.ArgumentParser(description="SAC DMP Optimization Runner")
#     parser.add_argument(
#         "-c",
#         "--config",
#         default="sac_config.yaml",
#         help="Path to SAC YAML config (default: sac_config.yaml)",
#     )
#     return parser.parse_args()

# def main():
#     args = _parse_args()
    
#     # Path handling to match your project structure
#     config_path = f"config/{args.config}" if not args.config.startswith("config/") else args.config

#     try:
#         # 1. Verification of config existence
#         if not os.path.exists(config_path):
#             print(f"Error: Configuration file {config_path} not found.")
#             return

#         # 2. Load SAC-specific configuration via your standard loader
#         config = load_config(config_path)
        
#         # 3. Setup versioned logging directories (1, 2, 3...)
#         # This replaces manual folder creation to ensure PPO/SAC consistency
#         config = setup_logging_dirs(config)

#         # 4. Set Random Seeds for Thesis Reproducibility
#         seed = config.get('llm_settings', {}).get('seed_number', 42)
#         torch.manual_seed(seed)
#         np.random.seed(seed)
#         random.seed(seed)
        
#         if torch.cuda.is_available():
#             torch.cuda.manual_seed(seed)
#             torch.backends.cudnn.deterministic = True
#             torch.backends.cudnn.benchmark = False

#         # 5. Device Selection & Performance Optimizations (sg014/sg023 cluster)
#         torch.set_num_threads(1) # Avoid CPU contention on cluster nodes
#         device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
#         print("\n" + "="*60)
#         print(f"ALGORITHM: Soft Actor-Critic (SAC)")
#         print(f"DEVICE: {device}")
#         if torch.cuda.is_available():
#             print(f"GPU MODEL: {torch.cuda.get_device_name(0)}")
#         print(f"RESULTS SAVING TO: {config['logs']['root']}")
#         print("="*60 + "\n")

#         # 6. Execute SAC Optimization Loop
#         run_sac_optimization(config)

#     except KeyboardInterrupt:
#         print("\n[User Interrupt] Optimization stopped. Data saved to CSV.")
#     except Exception as e:
#         print(f"\n[Critical Error] SAC failed: {e}")
#         traceback.print_exc()
#         raise e

# if __name__ == "__main__":
#     main()










# import sys
# import os
# import argparse
# import torch
# import numpy as np
# import random
# import traceback
# import time

# # Ensure project root is in path
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# from config.loader import load_config, setup_logging_dirs
# from runner.pure_sac_main_runner import run_sac_optimization


# # =========================
# # Seed utility
# # =========================
# def set_all_seeds(seed: int):
#     os.environ["PYTHONHASHSEED"] = str(seed)

#     random.seed(seed)
#     np.random.seed(seed)

#     torch.manual_seed(seed)
#     torch.cuda.manual_seed_all(seed)

#     # Allow stochastic behavior for exploration
#     torch.backends.cudnn.deterministic = False
#     torch.backends.cudnn.benchmark = True


# # =========================
# # CLI args
# # =========================
# def _parse_args():
#     parser = argparse.ArgumentParser(description="SAC DMP Optimization Runner")

#     parser.add_argument(
#         "-c",
#         "--config",
#         default="sac_config.yaml",
#         help="Path to SAC YAML config (default: sac_config.yaml)",
#     )

#     parser.add_argument(
#         "--seed",
#         type=int,
#         default=None,
#         help="Use a fixed seed for reproducibility",
#     )

#     parser.add_argument(
#         "--random-seed",
#         action="store_true",
#         help="Force a different random seed every run",
#     )

#     return parser.parse_args()


# def main():
#     args = _parse_args()

#     config_path = (
#         f"config/{args.config}"
#         if not args.config.startswith("config/")
#         else args.config
#     )

#     try:
#         if not os.path.exists(config_path):
#             print(f"Error: Configuration file {config_path} not found.")
#             return

#         config = load_config(config_path)
#         config = setup_logging_dirs(config)

#         # =========================
#         # Determine seed
#         # =========================
#         if args.random_seed:
#             seed = int.from_bytes(os.urandom(4), "little")

#         elif args.seed is not None:
#             seed = args.seed

#         else:
#             seed = int(time.time()) ^ os.getpid()

#         set_all_seeds(seed)

#         # Save seed into config
#         config.setdefault("experiment", {})
#         config["experiment"]["seed"] = seed

#         # =========================
#         # Device setup
#         # =========================
#         torch.set_num_threads(1)
#         device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

#         print("\n" + "=" * 60)
#         print("ALGORITHM: Soft Actor-Critic (SAC)")
#         print(f"DEVICE: {device}")
#         if torch.cuda.is_available():
#             print(f"GPU MODEL: {torch.cuda.get_device_name(0)}")
#         print(f"SEED USED: {seed}")
#         print(f"RESULTS SAVING TO: {config['logs']['root']}")
#         print("=" * 60 + "\n")

#         run_sac_optimization(config)

#     except KeyboardInterrupt:
#         print("\n[User Interrupt] Optimization stopped. Data saved to CSV.")
#     except Exception as e:
#         print(f"\n[Critical Error] SAC failed: {e}")
#         traceback.print_exc()
#         raise e


# if __name__ == "__main__":
#     main()







#!/usr/bin/env python3
"""
main_sac.py

Entry-point script to run Soft Actor-Critic (SAC) optimization.

What this file does:
1) Loads YAML config from config/<name>.yaml (or a direct path if provided)
2) Creates logging directories via setup_logging_dirs()
3) Sets all random seeds (Python, NumPy, Torch) for controlled randomness
4) Prints a clear run header (device, GPU, seed, output dir, state/action dims)
5) Calls run_sac_optimization(config)

Important:
- This script does NOT define the SAC algorithm itself.
- The SAC logic is inside runner/pure_sac_main_runner.py (or runner/sac_main_runner.py).
- This script tries to import from runner.pure_sac_main_runner first, and falls back
  to runner.sac_main_runner automatically to avoid filename mismatch issues.
"""

import sys
import os
import argparse
import torch
import numpy as np
import random
import traceback
import time


# =========================
# Ensure project root is in PYTHONPATH
# =========================
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(THIS_DIR)


# =========================
# Project imports
# =========================
from config.loader import load_config, setup_logging_dirs

# Try both possible runner filenames to avoid import mismatch headaches.
try:
    from runner.pure_sac_main_runner import run_sac_optimization
    _RUNNER_IMPORT = "runner.pure_sac_main_runner"
except ImportError:
    from runner.sac_main_runner import run_sac_optimization
    _RUNNER_IMPORT = "runner.sac_main_runner"


# =========================
# Seed utility
# =========================
def set_all_seeds(seed: int):
    """
    Sets seeds for reproducibility while still allowing stochastic exploration.
    """
    os.environ["PYTHONHASHSEED"] = str(seed)

    random.seed(seed)
    np.random.seed(seed)

    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

    # Stochastic behavior for exploration (not fully deterministic)
    torch.backends.cudnn.deterministic = False
    torch.backends.cudnn.benchmark = True


# =========================
# CLI args
# =========================
def _parse_args():
    parser = argparse.ArgumentParser(description="SAC DMP Optimization Runner")

    parser.add_argument(
        "-c",
        "--config",
        default="sac_config.yaml",
        help="Path to SAC YAML config (default: sac_config.yaml). "
             "If you pass a name without 'config/', it loads from config/<name>.",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Use a fixed seed for reproducibility (overrides time-based seed).",
    )

    parser.add_argument(
        "--random-seed",
        action="store_true",
        help="Force a different random seed every run (stronger than time-based seed).",
    )

    return parser.parse_args()


def _resolve_config_path(arg_config: str) -> str:
    """
    Resolves config path:
    - If user passed something starting with 'config/', use it as-is.
    - Else assume it is under the config/ directory.
    """
    if arg_config.startswith("config/") or arg_config.startswith("config\\"):
        return arg_config
    return os.path.join("config", arg_config)


def _choose_seed(args) -> int:
    """
    Seed priority:
    1) --random-seed
    2) --seed <int>
    3) time-based (time ^ pid)
    """
    if args.random_seed:
        return int.from_bytes(os.urandom(4), "little")
    if args.seed is not None:
        return int(args.seed)
    return int(time.time()) ^ os.getpid()


def main():
    args = _parse_args()
    config_path = _resolve_config_path(args.config)

    try:
        if not os.path.exists(config_path):
            print(f"Error: Configuration file not found: {config_path}")
            return

        # -------------------------
        # Load config + setup logging dirs
        # -------------------------
        config = load_config(config_path)
        config = setup_logging_dirs(config)

        # -------------------------
        # Seed setup
        # -------------------------
        seed = _choose_seed(args)
        set_all_seeds(seed)

        # Save seed into config for record-keeping
        config.setdefault("experiment", {})
        config["experiment"]["seed"] = seed

        # -------------------------
        # Device setup
        # -------------------------
        torch.set_num_threads(1)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # -------------------------
        # Sanity prints: state/action dims (trajectory-state runner)
        # -------------------------
        n_bfs = int(config["dmp_params"]["n_bfs"])
        traj_state_len = int(config.get("sac_params", {}).get("traj_state_len", 100))

        # state_dim = 2 * traj_state_len
        # action_dim = 2 * n_bfs
        state_dim  = 2 * traj_state_len + 2 * n_bfs   # (2*100) + (2*10) = 220
        action_dim = 2 * n_bfs                          # 20

        # and update the print line too:
        print(f"STATE DIM (trajectory + delta_w): {state_dim}  "
            f"(traj_state_len={traj_state_len}, n_bfs={n_bfs})")

        # -------------------------
        # Run header
        # -------------------------
        print("\n" + "=" * 60)
        print("ALGORITHM: Soft Actor-Critic (SAC)")
        print(f"RUNNER MODULE: {_RUNNER_IMPORT}")
        print(f"DEVICE: {device}")
        if torch.cuda.is_available():
            print(f"GPU MODEL: {torch.cuda.get_device_name(0)}")
        print(f"SEED USED: {seed}")
        print(f"CONFIG PATH: {config_path}")
        print(f"RESULTS SAVING TO: {config['logs']['root']}")
        print(f"STATE DIM (trajectory): {state_dim}  (traj_state_len={traj_state_len})")
        print(f"ACTION DIM (DMP weights): {action_dim}  (n_bfs={n_bfs})")
        print("=" * 60 + "\n")

        # -------------------------
        # Run SAC optimization
        # -------------------------
        run_sac_optimization(config)

    except KeyboardInterrupt:
        print("\n[User Interrupt] Optimization stopped. Data saved to CSV.")
    except Exception as e:
        print(f"\n[Critical Error] SAC failed: {e}")
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()


























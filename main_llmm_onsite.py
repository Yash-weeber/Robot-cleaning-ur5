import sys
import os
import dotenv
import argparse
# os.environ["MUJOCO_GL"] = "egl"
# os.environ["PYOPENGL_PLATFORM"] = "egl"
# Ensure project root is in path for modular imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config, setup_logging_dirs
from runner.llm_main_runner import run_llm_optimization

def _parse_args():
    parser = argparse.ArgumentParser(description="LLM-driven DMP optimization runner")
    parser.add_argument(
        "-c",
        "--config",
        default="config.yaml",
        help="Path to YAML config file (default: config.yaml)",
    )
    parser.add_argument(
        "-k",
        "--keys",
        default="./keys.env",
        help="Path to env file containing API keys (default: ./keys.env)",
    )
    return parser.parse_args()


def main():
    """
    Main entry point for LLM-driven DMP optimization.
    """
    args = _parse_args()
    args.config = f"config/{args.config}" if not args.config.startswith("config/") else args.config

    # Load API keys from keys.env (overrideable via --keys)
    if args.keys and os.path.exists(args.keys):
        dotenv.load_dotenv(args.keys)
    else:
        print(f"Warning: {args.keys} not found.")

    try:
        # 1. Load configuration from YAML (overrideable via --config)
        if not os.path.exists(args.config):
            raise FileNotFoundError(f"Config file not found: {args.config}")

        config = load_config(args.config)
        setup_logging_dirs(config)

        # 2. Run the optimization loop
        run_llm_optimization(config)

    except Exception as e:
        print(f"Critical error during LLM optimization: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
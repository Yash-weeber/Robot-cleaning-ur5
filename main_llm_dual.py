import sys
import os
import dotenv
import argparse

# Ensure project root is in path for modular imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config, setup_logging_dirs
# Match the function name in your runner/llm_main_runner_dual.py
from runner.llm_main_runner_dual import run_llm_optimization_dual

def _parse_args():
    parser = argparse.ArgumentParser(description="Dual-LLM (Critic + Optimizer) DMP optimization runner")
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
    Main entry point for Dual-LLM driven DMP optimization.
    Calls the optimization loop function directly.
    """
    args = _parse_args()
    
    # Path logic for config folder
    if not args.config.startswith("config/"):
        args.config = f"config/{args.config}"
    
    # Load API keys for Gemini rotation and Ollama access
    if args.keys and os.path.exists(args.keys):
        dotenv.load_dotenv(args.keys)
    else:
        print(f"Warning: API Key file {args.keys} not found.")

    try:
        # 1. Load configuration from YAML
        if not os.path.exists(args.config):
            raise FileNotFoundError(f"Config file not found: {args.config}")

        config = load_config(args.config)
        setup_logging_dirs(config)

        print(f"--- Launching Dual-LLM Session ---")
        print(f"Optimizer: {config['llm_settings']['llm_model']}")
        print(f"Critic: {config['llm_settings']['critic_model']}")

        # 2. Run the optimization loop function
        # This function handles the it in range(...) loop internally.
        run_llm_optimization_dual(config)

    except Exception as e:
        print(f"Critical error during Dual-LLM optimization: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
import sys
import os
import dotenv
os.environ["MUJOCO_GL"] = "egl"
os.environ["PYOPENGL_PLATFORM"] = "egl"
# Ensure project root is in path for modular imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config
from runner.llm_main_runner import run_llm_optimization


def main():
    """
    Main entry point for LLM-driven DMP optimization.
    """
    # Load API keys from keys.env as per original logic
    keys_path = "./keys.env"
    if os.path.exists(keys_path):
        dotenv.load_dotenv(keys_path)
    else:
        print(f"Warning: {keys_path} not found.")

    try:
        # 1. Load configuration from YAML
        config = load_config("config/config.yaml")

        # 2. Run the optimization loop
        run_llm_optimization(config)

    except Exception as e:
        print(f"Critical error during LLM optimization: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
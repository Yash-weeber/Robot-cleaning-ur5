import sys
import os

# Ensure the project root is in the python path for modular imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config
from runner.main_runner import EnhancedDMPController


def main():
    """
    Main entry point for the Factorized DMP Controller.
    """
    try:
        # 1. Load configuration
        config = load_config("config/config.yaml")

        # 2. Initialize the controller with the config dictionary
        controller = EnhancedDMPController(config)

        # 3. Start the main menu loop
        controller.run()

    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
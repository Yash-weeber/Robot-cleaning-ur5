import sys
import os

os.environ["MUJOCO_GL"] = "egl"
os.environ["PYOPENGL_PLATFORM"] = "egl"


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config
from runner.cmaes_runner import run_cmaes_optimization

def main():

    try:
 
        config = load_config("config/config.yaml")


        run_cmaes_optimization(config)

    except Exception as e:
        print(f"Critical error during CMA-ES optimization: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
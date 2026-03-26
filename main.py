import sys
import os
import pandas as pd
import numpy as np
# from kdl import urdf
# Ensure the project root is in the python path for modular imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.loader import load_config
from runner.main_runner import EnhancedDMPController
import pathlib as Path


def main():
    """
    Main entry point for the Factorized DMP Controller.
    """
    # iterations = [1, 50, 100, 150, 200, 250, 300, 350, 400] # Example iterations to visualize
    iterations = [100, 150, 200, 250, 300, 350, 400]
    for iteration in iterations:
        print(f"\n=== Executing trajectory for iteration {iteration} ===")
        try:
            # 1. Load configuration
            config = load_config("config/config.yaml")

            # 2. Initialize the controller with the config dictionary
            controller = EnhancedDMPController(config)
            
            df_traj = pd.read_csv("dmp_trajectory_feedback-sinusoid-x-run-7.csv")
            # iteration = 50
            df_traj_iter = df_traj[df_traj['iter'] == iteration].copy()
            x_orig = df_traj_iter['x'].values
            y_orig = df_traj_iter['y'].values
            xy_traj = np.vstack([x_orig, y_orig]).T
            video_dir = Path.Path("videos/sinusoid-x-run-7")
            video_dir.mkdir(parents=True, exist_ok=True)
            video_file = video_dir / f"iteration_{iteration}.mp4"
            controller.execute_xy_trajectory(xy_traj, record_path=video_file)

            # 3. Start the main menu loop
            # controller.run()

        except FileNotFoundError as e:
            print(f"Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
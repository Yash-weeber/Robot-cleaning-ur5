import numpy as np
import mujoco
from utils.draw_shapes import (
    circle_trajectory, rectangle_trajectory,
    elipsoid_trajectory, triangle_trajectory
)
from utils.obstacle_avoidance import avoid_obstacles


def generate_warmup_trajectory(n_counter):

    if n_counter == 0:
        x_traj, y_traj = circle_trajectory(center=(0.0, -0.1), radius=0.4, num_points=200, plot=False)
    elif n_counter == 1:
        x_traj, y_traj = rectangle_trajectory(center=(0.0, -0.1), width=1.0, height=0.4, num_points=200, plot=False)
    elif n_counter == 2:
        x_traj, y_traj = elipsoid_trajectory(center=(0, 0), axes_lengths=(1.0, 0.3), angle=np.pi/6, num_points=200, plot=False)
    elif n_counter == 3:
        x_traj, y_traj = triangle_trajectory(center=(0, -0.2), side_length=1.25, num_points=200, plot=False)
    else:
        return None

    trajectory = np.vstack((x_traj, y_traj))

    trajectory = np.hstack((np.zeros((2, 1)), trajectory)).T
    return trajectory

def get_dmp_step_with_obstacles(dmp):

    y, _, _ = dmp.step(
        tau=2.0,
        external_force=avoid_obstacles(
            dmp.y, dmp.dy, dmp.goal,
            rect_d0=0.05,
            rect_eta=25.0,
            obs_d0=0.1,
            obs_eta=25.0,
            max_force=220.0
        )
    )
    return y


def log_iteration_data(iter_idx, grid_mat, total_balls, traj_len, out_csv):

    import csv
    import os
    import time

    flat = list(map(int, grid_mat.flatten()))
    file_exists = os.path.exists(out_csv)
    with open(out_csv, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "timestamp", "traj_waypoints", "total_balls"] +
                       [f"cell{i}" for i in range(len(flat))])
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), traj_len, total_balls] + flat)
import numpy as np
import mujoco
from utils.draw_shapes import (
    circle_trajectory, rectangle_trajectory,
    elipsoid_trajectory, triangle_trajectory
)
from utils.obstacle_avoidance import avoid_obstacles


def generate_warmup_trajectory(n_counter, config):
    ws_center = config["simulation"]["ws_center"]
    ws_width = config["simulation"]["ws_width"]
    ws_length = config["simulation"]["ws_length"]
    if n_counter == 0:
        x_traj, y_traj = circle_trajectory(center=(ws_center[0], ws_center[1]), radius=0.35*ws_width, num_points=200, plot=False)
    elif n_counter == 1:
        x_traj, y_traj = rectangle_trajectory(center=(ws_center[0], ws_center[1]), width=0.6*ws_width, height=0.8*ws_length, num_points=200, plot=False)
    elif n_counter == 2:
        x_traj, y_traj = elipsoid_trajectory(center=(ws_center[0], ws_center[1]), axes_lengths=(1.0/2.1*ws_width, 0.6*ws_width), angle=0.0, num_points=200, plot=False)
    elif n_counter == 3:
        x_traj, y_traj = triangle_trajectory(center=(ws_center[0], ws_center[1]), side_length=0.8*ws_width, num_points=200, plot=False)
    #     x_traj, y_traj = circle_trajectory(center=(ws_center[0], ws_center[1]), radius=0.2*ws_width, num_points=200, plot=False)
    # elif n_counter == 1:
    #     x_traj, y_traj = rectangle_trajectory(center=(ws_center[0], ws_center[1]), width=0.2*ws_width, height=0.35*ws_length, num_points=200, plot=False)
    # elif n_counter == 2:
    #     x_traj, y_traj = elipsoid_trajectory(center=(ws_center[0], ws_center[1]), axes_lengths=(0.2*ws_width, 0.4*ws_width), angle=0.0, num_points=200, plot=False)
    # elif n_counter == 3:
    #     x_traj, y_traj = triangle_trajectory(center=(ws_center[0], ws_center[1]), side_length=0.3*ws_width, num_points=200, plot=False)
    else:
        return None

    trajectory = np.vstack((x_traj, y_traj))

    trajectory = np.hstack((np.array(ws_center).reshape(2, 1), trajectory)).T
    return trajectory

def get_dmp_step_with_obstacles(dmp):

    y, _, _ = dmp.step(
        tau=2.0,
        external_force=avoid_obstacles(
            dmp.y, dmp.dy, dmp.goal,
            rect_d0_x=0.06,
            rect_d0_y=0.14,
            rect_eta=30.0,
            obs_d0=0.1,
            obs_eta=30.0,
            max_force=220.0
        )
    )
    return y


def log_iteration_data(iter_idx, grid_mat, total_balls, traj_len, out_csv):

    import csv
    import os
    import time

    flat = list(map(int, grid_mat.flatten())) if grid_mat is not None else ''
    file_exists = os.path.exists(out_csv)
    with open(out_csv, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            if grid_mat is not None:
                w.writerow(["iter", "timestamp", "traj_waypoints", "total_balls"] +
                       [f"cell{i}" for i in range(len(flat))])
            else:
                w.writerow(["iter", "timestamp", "traj_waypoints", "total_balls"])
        if grid_mat is None:
            w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), traj_len, total_balls])
        else:
            w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), traj_len, total_balls] + flat)
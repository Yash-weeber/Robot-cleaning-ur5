import os
import json
import numpy as np
import pandas as pd


def load_trajectory_history(csv_path, max_iters=20):

    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        trajectory_history = {}
        rows = np.atleast_1d(data)
        for row in rows:
            iter_num = int(row["iter"])
            if iter_num not in trajectory_history:
                trajectory_history[iter_num] = []
            trajectory_history[iter_num].append({
                "step": int(row["step"]),
                "x": float(row["x"]),
                "y": float(row["y"])
            })
        sorted_iters = sorted(trajectory_history.keys())
        if len(sorted_iters) > max_iters:
            sorted_iters = sorted_iters[-max_iters:]
        return {k: trajectory_history[k] for k in sorted_iters}
    except Exception as e:
        print(f"Warning: Could not load trajectory history: {e}")
        return {}


def load_ik_error_history(csv_path, max_iters=20):

    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        rows = np.atleast_1d(data)
        history = {}
        for row in rows:
            it = int(row["iter"])
            entry = {
                "step": int(row["step"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "error_m": float(row["error_m"]),
            }
            history.setdefault(it, []).append(entry)
        keys = sorted(history.keys())
        if len(keys) > max_iters:
            keys = keys[-max_iters:]
        return {k: history[k] for k in keys}
    except Exception as e:
        print(f"Warning: Could not load IK error history: {e}")
        return {}


def summarize_ik_errors(error_history):

    summary = {}
    for it, entries in error_history.items():
        if not entries:
            continue
        errs = [e["error_m"] for e in entries]
        summary[it] = {
            "num_failures": int(len(errs)),
            "max_error_m": float(np.max(errs)),
            "mean_error_m": float(np.mean(errs)),
            "sample": entries[:3]
        }
    return summary


def analyze_trajectory_performance(trajectory_data, bounds):

    if not trajectory_data:
        return {}
    analysis = {}
    for iter_num, traj_points in trajectory_data.items():
        if not traj_points or len(traj_points) < 2:
            continue
        xs = [p["x"] for p in traj_points]
        ys = [p["y"] for p in traj_points]
        x_in_bounds = all(bounds["xmin"] <= x <= bounds["xmax"] for x in xs)
        y_in_bounds = all(bounds["ymin"] <= y <= bounds["ymax"] for y in ys)
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        bounds_width = bounds["xmax"] - bounds["xmin"]
        bounds_height = bounds["ymax"] - bounds["ymin"]
        x_range_covered = x_range / bounds_width if bounds_width > 0 else 0
        y_range_covered = y_range / bounds_height if bounds_height > 0 else 0
        path_length = sum(np.sqrt((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2)
                          for i in range(len(xs) - 1))
        direct_distance = np.sqrt((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2)
        smoothness = direct_distance / path_length if path_length > 0 else 0
        analysis[iter_num] = {
            "bounds_compliant": x_in_bounds and y_in_bounds,
            "x_coverage": x_range_covered,
            "y_coverage": y_range_covered,
            "smoothness": smoothness,
            "path_length": path_length,
            "waypoint_count": len(traj_points)
        }
    return analysis


def load_iteration_log(csv_path, n_x_seg, n_y_seg):

    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        log_data = {}
        for row in np.atleast_1d(data):
            it = int(row["iter"]) # "cells": cells,
            # cells = [int(row[f"cell{i}"]) for i in range(n_x_seg * n_y_seg)]
            log_data[it] = {
                "traj_waypoints": int(row["traj_waypoints"]),
                "total_balls": round(row["total_balls"],1),
            }
        return log_data
    except Exception as e:
        print(f"Warning: Could not load iteration log {csv_path}: {e}")
        return {}


def load_traj_feedback(csv_path):

    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        traj_data = {}
        for row in np.atleast_1d(data):
            it = int(row["iter"])
            if it not in traj_data:
                traj_data[it] = []
            traj_data[it].append({
                "step": int(row["step"]),
                "x": float(row["x"]),
                "y": float(row["y"])
            })
        return traj_data
    except Exception as e:
        print(f"Warning: Could not load trajectory feedback {csv_path}: {e}")
        return {}

def build_llm_feedback(iter_idx, w_df, iter_log_data, traj_feedback_data, ee_traj_df, config, bounds):

    STRICT_X_MIN, STRICT_X_MAX = bounds['xmin'], bounds['xmax']
    STRICT_Y_MIN, STRICT_Y_MAX = bounds['ymin'], bounds['ymax']
    guided = config['llm_settings'].get('guided', False)

    n_warmup = config['llm_settings']['n_warmup']
    feedback_window = config['llm_settings']['feedback_window']
    traj_in_prompt = config['llm_settings']['traj_in_prompt']
    grid_coverage_in_prompt = config['llm_settings'].get('grid_coverage_in_prompt', False)
    grid_reward_enabled = config['llm_settings'].get('grid_reward', False)
    resample_rate = config['llm_settings'].get('resample_rate', 30)
    y_window = config['llm_settings'].get('y_window', 0)
    x_window = config['llm_settings'].get('x_window', 0)

    # Grid parameters for labeling markdown tables
    n_x_seg = config['dmp_params']['num_x_segments']
    n_y_seg = config['dmp_params']['num_y_segments']
    x_edges = np.linspace(bounds['xmin'], bounds['xmax'], n_x_seg + 1)
    y_edges = np.linspace(bounds['ymin'], bounds['ymax'], n_y_seg + 1)
    
    def _trajectory_coverage_grid(df_points: pd.DataFrame) -> np.ndarray:
        """
        Returns a (n_y_seg, n_x_seg) binary grid where grid[y, x] == 1
        if any trajectory point falls into that (x,y) segment.
        """
        if df_points is None or df_points.empty:
            return np.zeros((n_y_seg, n_x_seg), dtype=int)

        x_idx = np.digitize(df_points["x"].to_numpy(dtype=float), x_edges, right=False) - 1
        y_idx = np.digitize(df_points["y"].to_numpy(dtype=float), y_edges, right=False) - 1

        # Clamp points exactly at max edge into the last bin
        x_idx = np.where(x_idx == n_x_seg, n_x_seg - 1, x_idx)
        y_idx = np.where(y_idx == n_y_seg, n_y_seg - 1, y_idx)

        in_bounds = (x_idx >= 0) & (x_idx < n_x_seg) & (y_idx >= 0) & (y_idx < n_y_seg)
        x_idx = x_idx[in_bounds]
        y_idx = y_idx[in_bounds]

        grid = np.zeros((int(n_y_seg), int(n_x_seg)), dtype=int)

        yw = int(y_window)
        if yw < 0:
            raise ValueError("y_window must be >= 0")

        xw = int(x_window)
        if xw < 0:
            raise ValueError("x_window must be >= 0")

        if x_idx.size == 0:
            return grid

        if yw == 0 and xw == 0:
            grid[y_idx, x_idx] = 1
        else:
            y_shifts = np.arange(-yw, yw + 1, dtype=int)
            x_shifts = np.arange(-xw, xw + 1, dtype=int)

            # create meshgrid of shifts
            sy, sx = np.meshgrid(y_shifts, x_shifts, indexing='ij')
            sy = sy.ravel()
            sx = sx.ravel()

            # Broadcast to shape (N, K)
            y_expanded = (y_idx[:, None] + sy[None, :]).clip(0, int(n_y_seg) - 1)
            x_expanded = (x_idx[:, None] + sx[None, :]).clip(0, int(n_x_seg) - 1)

            grid[y_expanded.ravel(), x_expanded.ravel()] = 1

        return grid

    feedback_text = ""

    guidance_text = "The optimal policy must result in a sinusoidal trajectory that covers the workspace, while avoiding going out of bounds. The sinusoidal sweeping motion must be along the y-axis (sweeping side to side along the x-axis), smooth, and continuous. The optimal policy must aim to reach the global optimum and cover as much as possible of the workspace." if not traj_in_prompt else "The policy should result in a sinusoidal trajectory that covers the workspace, while avoiding going out of bounds. The sinusoidal motion should sweep up and down the y-axis smoothly and continuously. Analyze the impact of each weight on the trajectory, then use the analysis to inform your weight adjustments."

    if w_df is not None and not w_df.empty:
        # Get recent executed iterations
        executed_df = w_df[(w_df['tag'] == 'proposed') & (w_df['iter'] < iter_idx)].copy()
        recent = executed_df.sort_values(by='iter', ascending=False).head(feedback_window)

        for _, row in recent.sort_values(by='iter').iterrows():
            it_num = int(row['iter'])
            w_cols = [c for c in w_df.columns if c.startswith('w')]
            weights = pd.to_numeric(row[w_cols]).dropna().tolist()

            # Fetch performance metrics
            log_entry = iter_log_data.get(it_num, {})
            current_f = log_entry.get('total_balls', 'N/A')
            cells_list = log_entry.get('cells', [])

            # Check Bounds compliance
            bounds_info = ""
            if it_num in traj_feedback_data:
                pts = traj_feedback_data[it_num]
                x_vals = [p['x'] for p in pts]
                y_vals = [p['y'] for p in pts]
                is_failed = (min(x_vals) < STRICT_X_MIN or max(x_vals) > STRICT_X_MAX or
                             min(y_vals) < STRICT_Y_MIN or max(y_vals) > STRICT_Y_MAX)
                bounds_info = f"x_range=[{min(x_vals):.4f}, {max(x_vals):.4f}], y_range=[{min(y_vals):.4f}, {max(y_vals):.4f}]"
                # if is_failed:
                #     bounds_info += " (FAILED)"

            # Construct iteration block with exact separators
            # iter_label = f" Examples {it_num + n_warmup} " if it_num < 1 else f" Iteration {it_num} "
            iter_label = f" Example {-it_num} " if it_num < 1 else f" Iteration {it_num} "
            separator = "-" * 50
            feedback_text += f"{separator}{iter_label}{separator}\n"
            feedback_text += f"weights={json.dumps([round(w, 4) for w in weights])}\n{bounds_info}\n"

            # Trajectory Resampling (every 30 steps)
            if traj_in_prompt and ee_traj_df is not None:
                it_traj = ee_traj_df[ee_traj_df["iter"] == it_num].copy()
                if not it_traj.empty:
                    it_traj.drop(columns=['iter', 'timestamp'], inplace=True, errors='ignore')
                    resampled = it_traj.iloc[::resample_rate, :].reset_index(drop=True)
                    resampled.set_index('step', inplace=True)
                    feedback_text += f"Resampled 2D Trajectory:\n{resampled.to_markdown()}\n"
                    
            if grid_coverage_in_prompt and ee_traj_df is not None:
                it_traj = ee_traj_df[ee_traj_df["iter"] == it_num].copy()
                if not it_traj.empty and ("x" in it_traj.columns) and ("y" in it_traj.columns):
                    coverage_grid = _trajectory_coverage_grid(it_traj)

                    cov_df = pd.DataFrame(coverage_grid)
                    cov_df.index = [f"y:[{y_edges[j]:.2f},{y_edges[j+1]:.2f}]" for j in range(n_y_seg)]
                    cov_df.columns = [f"x:[{x_edges[i]:.2f},{x_edges[i+1]:.2f}]" for i in range(n_x_seg)]

                    feedback_text += "Trajectory coverage (1=visited, 0=not visited):\n"
                    feedback_text += f"{cov_df.to_markdown(index=True)}\n\n"

            # Grid Reward Markdown Table
            if grid_reward_enabled and cells_list:
                # Reshape flat list to 2D grid: n_x_seg columns, n_y_seg rows
                cells_arr = np.array(cells_list).reshape(n_x_seg, n_y_seg).T
                cells_df = pd.DataFrame(cells_arr)
                cells_df.index = [f"y:[{y_edges[j]:.2f},{y_edges[j+1]:.2f}]" for j in range(n_y_seg)]
                cells_df.columns = [f"x:[{x_edges[i]:.2f},{x_edges[i+1]:.2f}]" for i in range(n_x_seg)]
                feedback_text += f"f(weights):\n{cells_df.to_markdown(index=True)}\n\n"
            else:
                feedback_text += f"f(weights)={current_f}\n\n"

    return feedback_text, guidance_text
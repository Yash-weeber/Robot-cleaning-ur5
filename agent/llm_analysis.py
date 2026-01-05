import os
import json
import numpy as np
import pandas as pd


def load_trajectory_history(csv_path, max_iters=20):
    """Exact copy of original logic to load last max_iters of trajectory data."""
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
    """Exact copy of original logic to return dict {iter: [ {step,x,y,z,error_m}, ... ]}."""
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
    """Exact copy of original logic to compute per-iter IK failure stats."""
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
    """Exact copy of original trajectory analysis logic including coverage and smoothness."""
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


def load_iteration_log(csv_path):
    """Exact copy of original logic to load llm_iteration_log.csv."""
    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        log_data = {}
        for row in np.atleast_1d(data):
            it = int(row["iter"])
            cells = [int(row[f"cell{i}"]) for i in range(6)]
            log_data[it] = {
                "traj_waypoints": int(row["traj_waypoints"]),
                "total_balls": int(row["total_balls"]),
                "cells": cells,
            }
        return log_data
    except Exception as e:
        print(f"Warning: Could not load iteration log {csv_path}: {e}")
        return {}


def load_traj_feedback(csv_path):
    """Exact copy of original logic to load trajectory_feedback.csv."""
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


def build_llm_feedback(iter_idx, w_df, iter_log_data, traj_feedback_data, feedback_window, n_warmup):
    """
    Modularized exact logic from enhanced_ollama_prompt to build historical feedback string.
    Includes strict bounds checks: X [-1.05, 1.05], Y [-0.65, 0.65].
    """
    STRICT_X_MIN, STRICT_X_MAX = -1.050, 1.050
    STRICT_Y_MIN, STRICT_Y_MAX = -0.650, 0.650
    feedback_text = ""

    if w_df is not None and not w_df.empty:
        try:
            executed_df = w_df[(w_df['tag'] == 'executed') & (w_df['iter'] < iter_idx)].copy()
            executed_df['iter'] = executed_df['iter'].astype(int)
            recent_executed = executed_df.sort_values(by='iter', ascending=False).head(feedback_window)

            for _, row in recent_executed.sort_values(by='iter').iterrows():
                iter_num = int(row['iter'])
                weight_cols = [col for col in w_df.columns if col.startswith('w')]
                weights = pd.to_numeric(row[weight_cols], errors='coerce').dropna().tolist()
                current_f_weights = iter_log_data.get(iter_num, {}).get('total_balls', 'N/A')
                bounds_info = ""

                if iter_num in traj_feedback_data:
                    pts = traj_feedback_data[iter_num]
                    if pts:
                        x_vals = [p['x'] for p in pts]
                        y_vals = [p['y'] for p in pts]
                        x_min_traj, x_max_traj = round(min(x_vals), 4), round(max(x_vals), 4)
                        y_min_traj, y_max_traj = round(min(y_vals), 4), round(max(y_vals), 4)

                        is_failed = (x_min_traj < STRICT_X_MIN or x_max_traj > STRICT_X_MAX or
                                     y_min_traj < STRICT_Y_MIN or y_max_traj > STRICT_Y_MAX)

                        bounds_info = f", x_range=[{x_min_traj}, {x_max_traj}], y_range=[{y_min_traj}, {y_max_traj}]"
                        if is_failed:
                            bounds_info += " (FAILED)"

                if weights:
                    rounded_weights = [round(w, 4) for w in weights]
                    iter_string = f"Examples {iter_num + n_warmup}:" if iter_num < 1 else f"Iteration {iter_num}:"
                    feedback_text += (f"{iter_string} weights={json.dumps(rounded_weights)}"
                                      f"{bounds_info}, f(weights)={current_f_weights}\n")
        except Exception as e:
            feedback_text += f"# Error processing history: {str(e)}\n"

    return feedback_text
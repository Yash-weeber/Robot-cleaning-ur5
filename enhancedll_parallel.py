#!/usr/bin/env python3
"""
Enhanced DMP Controller - Parallel Execution Wrapper
Preserves ALL enhancedll.py logic, adds headless mode via environment variables
NO changes to testiing_2.py needed!

Usage:
    # Normal mode (with viewer):
    python enhancedll_parallel.py

    # Headless mode:
    set HEADLESS=1
    set EXPERIMENT_BASE_DIR=Y:/path/to/experiment/
    python enhancedll_parallel.py
"""

import os
import sys
import mujoco
import builtins

builtins.input = lambda *a, **k: "7"
# ============================================
# STEP 1: Monkey-patch viewer BEFORE any imports
# ============================================

if os.environ.get("HEADLESS", "0") == "1":
    print(" HEADLESS MODE ENABLED - Patching MuJoCo viewer...")
    # Windows doesn't support osmesa - leave MUJOCO_GL unset or use 'glfw' (but we'll disable viewer anyway)
    # Only set MUJOCO_GL if user explicitly provided it
    if "MUJOCO_GL" not in os.environ:
        # Don't set MUJOCO_GL - let MuJoCo use default, we're disabling viewer anyway
        pass


    # Create a dummy viewer that does nothing
    class DummyViewer:
        def __init__(self, *args, **kwargs):
            self.backend = "none"
            self.closed = False
            self._dm_context_mgr = self

        def is_running(self):
            return True

        def sync(self):
            pass

        def render(self):
            pass

        def draw(self):
            pass

        def close(self):
            self.closed = True

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass


    # Patch mujoco.viewer module
    class DummyViewerModule:
        @staticmethod
        def launch_passive(*args, **kwargs):
            print("[Viewer] Headless mode - viewer disabled")
            return DummyViewer()


    sys.modules['mujoco.viewer'] = DummyViewerModule()

    # Patch community viewer if present
    try:
        import mujoco_viewer

        mujoco_viewer.MujocoViewer = DummyViewer
    except ImportError:
        pass

# ============================================
# STEP 2: Override paths from environment
# ============================================

# Check if we're running in experiment-specific mode
EXPERIMENT_BASE_DIR = os.environ.get("EXPERIMENT_BASE_DIR", None)

if EXPERIMENT_BASE_DIR:
    print(f" Using experiment directory: {EXPERIMENT_BASE_DIR}")

    # Override all paths to use this base directory
    BASE_DIR = EXPERIMENT_BASE_DIR
    LOGDIR = os.path.join(BASE_DIR, "logs")

    CSV_MOVE_PATH = os.path.join(LOGDIR, "move1.csv")
    WEIGHTS_TXT = os.path.join(LOGDIR, "weight.txt")
    WEIGHTS_TXT2 = os.path.join(LOGDIR, "weight2.txt")
    TRAJ_TXT1 = os.path.join(LOGDIR, "traject1.txt")
    TRAJ_TXT2 = os.path.join(LOGDIR, "traject2.txt")
    TOTAL1 = os.path.join(LOGDIR, "total1.txt")
    TOTAL2 = os.path.join(LOGDIR, "total2.txt")
    GRID1 = os.path.join(LOGDIR, "gridlist1.txt")
    GRID2 = os.path.join(LOGDIR, "gridlist2.txt")

    WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
    ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
    DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
    WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
    TRAJECTORY_CSV = os.path.join(LOGDIR, "trajectory_feedback.csv")
    IK_ERROR_CSV = os.path.join(LOGDIR, "ik_errors.csv")
else:
    # Use default paths from original enhancedll.py
    BASE_DIR = "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/"
    LOGDIR = os.path.join(BASE_DIR, "logs")

    CSV_MOVE_PATH = os.path.join(LOGDIR, "move1.csv")
    WEIGHTS_TXT = os.path.join(LOGDIR, "weight.txt")
    WEIGHTS_TXT2 = os.path.join(LOGDIR, "weight2.txt")
    TRAJ_TXT1 = os.path.join(LOGDIR, "traject1.txt")
    TRAJ_TXT2 = os.path.join(LOGDIR, "traject2.txt")
    TOTAL1 = os.path.join(LOGDIR, "total1.txt")
    TOTAL2 = os.path.join(LOGDIR, "total2.txt")
    GRID1 = os.path.join(LOGDIR, "gridlist1.txt")
    GRID2 = os.path.join(LOGDIR, "gridlist2.txt")

    WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
    ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
    DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
    WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
    TRAJECTORY_CSV = os.path.join(LOGDIR, "trajectory_feedback.csv")
    IK_ERROR_CSV = os.path.join(LOGDIR, "ik_errors.csv")

# ============================================
# STEP 3: Now import everything else
# ============================================

import re
import csv
import json
import time
import uuid
import builtins
import subprocess
import numpy as np
import threading
from google import genai
import pandas as pd

# Import from testiing_2 (viewer is already patched if headless)
from testiing_2 import (
    EnhancedDMPController, MOP_Z_HEIGHT,
    enhanced_ik_solver, get_joint_positions, set_joint_positions
)
from pydmps.dmp_rhythmic import DMPs_rhythmic

# ============================================
# Constants from original enhancedll.py
# ============================================

HISTORY_WINDOW = 25
TRAJECTORY_HISTORY_WINDOW = 20
IK_ERROR_HISTORY_WINDOW = 40

N_BFS = 10
MAX_ITERS = int(os.environ.get("MAX_ITERS", "400"))
IK_MAX_ITERS = 50
DECI_BUILD = 2

GEMINI_MODEL = "gemini-2.5-flash"
GEMINI = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY"))

builtins.input = lambda *a, **k: "7"


# ============================================
# All your original utility functions
# ============================================

def ensure_dirs():
    os.makedirs(LOGDIR, exist_ok=True)
    os.makedirs(DIALOG_DIR, exist_ok=True)


def parse_weights_text(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?\d*\.?\d+", txt)
    if not nums:
        raise ValueError(f"No numeric weights found in {path}")
    return np.array([float(x) for x in nums], dtype=float)


def row_to_2x50(arr):
    """Accepts any even-length flat weight vector and returns shape (2, N_BFS),
    resizing per-axis weights if needed via linear interpolation."""
    a = np.asarray(arr, dtype=float).flatten()
    if a.size % 2 != 0:
        raise ValueError(f"Expected even number of weights, got {a.size}")

    cur_n_bfs = a.size // 2
    w2 = a.reshape(2, cur_n_bfs)

    if cur_n_bfs == N_BFS:
        return w2

    src_x = np.linspace(0.0, 1.0, cur_n_bfs)
    dst_x = np.linspace(0.0, 1.0, N_BFS)
    w_resized = np.empty((2, N_BFS), dtype=float)
    for d in range(2):
        w_resized[d] = np.interp(dst_x, src_x, w2[d])
    return w_resized


def write_weights_csv(path, w2):
    row = w2.reshape(-1)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerow(list(row))


def read_weights_csv(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", txt)
    vals = [float(x) for x in nums]

    need = 2 * N_BFS
    if len(vals) < need:
        raise ValueError(f"{path} has only {len(vals)} numbers, need {need} (2*N_BFS).")

    if len(vals) % 2 != 0:
        print(f"Warning: {path} has odd length ({len(vals)}). Dropping last value.")
        vals = vals[:-1]

    if len(vals) > need:
        print(f"Warning: {path} has {len(vals)} values. Trimming to the first {need}.")
        vals = vals[:need]

    return row_to_2x50(vals)


def read_move_csv(path):
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        if data.dtype.names and {"x", "y"}.issubset(data.dtype.names):
            xy = np.column_stack([data["x"], data["y"]]).astype(float)
            if xy.ndim == 2 and xy.shape[1] >= 2:
                return xy
    except Exception:
        pass

    xy = np.loadtxt(path, delimiter=",", dtype=float)
    if xy.ndim != 2 or xy.shape[1] < 2:
        raise ValueError(f"{path} must have at least two columns (x,y)")
    return xy[:, :2].astype(float)


def log_iteration(iter_idx, grid_mat, total_balls, traj_len, out_csv):
    flat = list(map(int, grid_mat.flatten()))
    file_exists = os.path.exists(out_csv)
    with open(out_csv, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "timestamp", "traj_waypoints", "total_balls"] +
                       [f"cell{i}" for i in range(len(flat))])
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), traj_len, total_balls] + flat)


def save_trajectory_data(iter_idx, task_trajectory, csv_path):
    """Save X,Y trajectory coordinates for this iteration."""
    file_exists = os.path.exists(csv_path)

    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "step", "x", "y", "timestamp"])

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        for step_idx, target in enumerate(task_trajectory):
            x, y = target[0], target[1]
            w.writerow([iter_idx, step_idx, float(x), float(y), timestamp])


def load_trajectory_history(csv_path, max_iters=20):
    """Load last max_iters iterations of trajectory data."""
    if not os.path.exists(csv_path):
        return {}

    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}

        trajectory_history = {}
        for row in data:
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


def save_ik_error(iter_idx, step_idx, target_3d, error_val, csv_path):
    """Append a single IK failure row."""
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "step", "x", "y", "z", "error_m", "timestamp"])
        x, y, z = float(target_3d[0]), float(target_3d[1]), float(target_3d[2])
        w.writerow([
            int(iter_idx), int(step_idx), x, y, z, float(error_val),
            time.strftime("%Y-%m-%d %H:%M:%S")
        ])


def load_ik_error_history(csv_path, max_iters=20):
    """Return dict of IK errors for last max_iters iterations."""
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
    """Compute per-iter IK failure stats."""
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
    """Analyze trajectory quality."""
    if not trajectory_data:
        return {}

    analysis = {}

    for iter_num, traj_points in trajectory_data.items():
        if not traj_points:
            continue

        xs = [p["x"] for p in traj_points]
        ys = [p["y"] for p in traj_points]

        x_in_bounds = all(bounds["xmin"] <= x <= bounds["xmax"] for x in xs)
        y_in_bounds = all(bounds["ymin"] <= y <= bounds["ymax"] for y in ys)

        x_range_covered = (max(xs) - min(xs)) / (bounds["xmax"] - bounds["xmin"])
        y_range_covered = (max(ys) - min(ys)) / (bounds["ymax"] - bounds["ymin"])

        path_length = sum(np.sqrt((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2)
                          for i in range(len(xs) - 1)) if len(xs) > 1 else 0
        direct_distance = np.sqrt((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2) if len(xs) > 1 else 0
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
    """Load llm_iteration_log.csv."""
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
    """Load trajectory_feedback.csv."""
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


# Your complete enhanced_ollama_prompt function
def enhanced_ollama_prompt(prev_w_flat, grid_mat, total_balls, iter_idx, history,
                           trajectory_history, trajectory_analysis, bounds, ik_error_summary=None,
                           iter_log_data=None, traj_feedback_data=None, feedback_window=40000):
    try:
        w_example1 = parse_weights_text(WEIGHTS_TXT).tolist()
    except Exception:
        w_example1 = []
    try:
        w_example2 = parse_weights_text(WEIGHTS_TXT2).tolist()
    except Exception:
        w_example2 = []
    try:
        trajectoy_1 = parse_weights_text(TRAJ_TXT1).tolist()
        trajectoy_2 = parse_weights_text(TRAJ_TXT2).tolist()
        total1 = parse_weights_text(TOTAL1).tolist()
        total2 = parse_weights_text(TOTAL2).tolist()
        grid1 = parse_weights_text(GRID1).tolist()
        grid2 = parse_weights_text(GRID2).tolist()
    except:
        trajectoy_1 = []
        trajectoy_2 = []
        total1 = []
        total2 = []
        grid1 = []
        grid2 = []

    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]
    grid_list = grid_mat.tolist()

    STRICT_X_MIN = -1.050
    STRICT_X_MAX = 1.050
    STRICT_Y_MIN = -0.650
    STRICT_Y_MAX = 0.650

    best_iter_summary = ""
    best_weight_feedback = ""
    best_iter_data = None
    best_weights = None
    w_df = None

    try:
        if os.path.exists(ITER_LOG_CSV) and os.path.exists(WEIGHT_HISTORY_CSV):
            iter_df = pd.read_csv(ITER_LOG_CSV)
            w_df = pd.read_csv(WEIGHT_HISTORY_CSV)

            iter_df = iter_df.dropna(subset=["total_balls", "traj_waypoints"])
            iter_df["iter"] = iter_df["iter"].astype(int)

            waypoint_median = iter_df["traj_waypoints"].median()
            waypoint_cutoff = max(waypoint_median * 0.9, waypoint_median - 150)

            valid_df = iter_df[iter_df["traj_waypoints"] >= waypoint_cutoff]

            if not valid_df.empty:
                best_iter = valid_df.loc[valid_df["total_balls"].idxmin()]
                best_iter_num = int(best_iter["iter"])
                best_iter_data = best_iter

                weight_row = w_df[w_df["iter"] == best_iter_num]
                if not weight_row.empty:
                    best_weights = [
                        float(x) for x in weight_row.iloc[0].values[1:]
                        if str(x).replace('.', '', 1).replace('-', '', 1).isdigit()
                    ]

                if best_weights:
                    w_min, w_max = np.min(best_weights), np.max(best_weights)
                    w_range = w_max - w_min
                    wrange = 340
                    max_step = round(wrange / 12, 3)
                    best_weight_feedback = (
                        f"# Weight Range Insight:\n"
                        f"  Weight range = [-170,170]  "
                        f"(Δ = {wrange:.3f}) → Suggested MAX_STEP = {max_step}\n"
                    )

    except Exception as e:
        best_iter_summary = f"\n# Error analyzing historical weights: {str(e)}\n"
    traj_feedback = {}
    for iter_num, analysis in trajectory_analysis.items():
        if iter_num in trajectory_history:
            traj_points = trajectory_history[iter_num]
            n_points = len(traj_points)
            sample_indices = [0, n_points // 2, n_points - 1] if n_points > 2 else list(range(n_points))
            sampled_points = [traj_points[i] for i in sample_indices if i < n_points]

            traj_feedback[iter_num] = {
                "performance": analysis,
                "sample_trajectory": sampled_points,
                "total_waypoints": n_points
            }

    def _ordinal(n):
        return f"{n}{'th' if 11 <= n % 100 <= 13 else {1: 'st', 2: 'nd', 3: 'rd'}.get(n % 10, 'th')}"

    feedback_text = ""

    def is_bounds_failed(x_min, x_max, y_min, y_max):
        return (x_min < STRICT_X_MIN or x_max > STRICT_X_MAX or
                y_min < STRICT_Y_MIN or y_max > STRICT_Y_MAX)

    if iter_log_data:
        recent_iters = sorted([k for k in iter_log_data.keys() if k < iter_idx])[-feedback_window:]
        if recent_iters:
            feedback_text += f"\n# Iteration Performance Summary (last {len(recent_iters)} iterations):\n"
            for i in recent_iters:
                entry = iter_log_data[i]
                current_f_weights = entry['total_balls']
                is_failed_iter = False
                if i in traj_feedback_data:
                    pts = traj_feedback_data[i]
                    if pts:
                        x_values = [p['x'] for p in pts]
                        y_values = [p['y'] for p in pts]

                        x_min_traj = min(x_values)
                        x_max_traj = max(x_values)
                        y_min_traj = min(y_values)
                        y_max_traj = max(y_values)

                        if is_bounds_failed(x_min_traj, x_max_traj, y_min_traj, y_max_traj):
                            is_failed_iter = True

                if is_failed_iter:
                    feedback_text += (
                        f"  {_ordinal(i)} iteration: f(weights)={current_f_weights} (FAILURE: Out of Bounds ❌)\n"
                    )
                else:
                    feedback_text += (
                        f"  {_ordinal(i)} iteration: f(weights)={current_f_weights}, "
                    )

    if w_df is not None and not w_df.empty:
        try:
            executed_df = w_df[(w_df['tag'] == 'executed') & (w_df['iter'] < iter_idx)].copy()
            executed_df['iter'] = executed_df['iter'].astype(int)
            recent_executed = executed_df.sort_values(by='iter', ascending=False).head(feedback_window)

            if not recent_executed.empty:
                feedback_text += f"\n# Executed Weights History (last {len(recent_executed)} executed iterations):\n"

                for _, row in recent_executed.sort_values(by='iter').iterrows():
                    iter_num = int(row['iter'])
                    weight_cols = [col for col in w_df.columns if col.startswith('w')]
                    weights = pd.to_numeric(row[weight_cols], errors='coerce').dropna().tolist()
                    current_f_weights = iter_log_data.get(iter_num, {}).get('total_balls', 'N/A')
                    bounds_info = ""
                    is_failed_iter = False
                    if iter_num in traj_feedback_data:
                        pts = traj_feedback_data[iter_num]
                        if pts:
                            x_values = [p['x'] for p in pts]
                            y_values = [p['y'] for p in pts]

                            x_min_traj = round(min(x_values), 4)
                            x_max_traj = round(max(x_values), 4)
                            y_min_traj = round(min(y_values), 4)
                            y_max_traj = round(max(y_values), 4)
                            is_failed_iter = is_bounds_failed(min(x_values), max(x_values), min(y_values),
                                                              max(y_values))

                            if is_failed_iter:
                                bounds_info = (
                                    f", **OUT-OF-BOUNDS FAILURE**: x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )
                            else:
                                bounds_info = (
                                    f", x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )

                    if weights:
                        rounded_weights = [round(w, 4) for w in weights]
                        failure_tag = " (FAILED)" if is_failed_iter else ""
                        feedback_text += (
                            f"  {_ordinal(iter_num)} iteration: weights={json.dumps(rounded_weights)}\n"
                            f"{bounds_info}\n"
                            f" f(weights) : {current_f_weights}"
                        )
        except Exception as e:
            feedback_text += f"# ️ Error processing executed weights history: {str(e)}\n"

    if best_iter_summary or best_weight_feedback:
        feedback_text += "\n# Historical Weight Performance Feedback:\n"
        feedback_text += best_iter_summary + best_weight_feedback

    return f"""
    You are a good global optimizer, helping me find the global minimum of a mathematical function
    f(weights). I will give you the function evaluation f(weights) and the current iteration number at each step. Your
    goal is to propose input values that efficiently lead us to the global minimum within a limited number
    of iterations (400).

    # Here's how we will interact :
        1) I will provide you max steps ({MAX_ITERS}) along with a couple of training examples which includes weights for the policy, and its corresponding function value f(weights) for each example. 
        2) You will provide the response in exact following format: 
            Output: STRICTLY one JSON object on a single line (no code fences). Include a brief "reason" and the "weights":
            {{"reason": "one sentence explaining coverage, bounds adherence", "weights": [exactly {2 * N_BFS} floats]}}
        3) I will then provide the function evaluation f(weights) at that point and the current iteration.
        4) You will repeat the steps from 2-3 until we will reach a maximum number of iteration.


    # Remember :
        1) XY WORKSPACE LIMITS (meters): x ∈ [{xmin:.3f}, {xmax:.3f}], y ∈ [{ymin:.3f}, {ymax:.3f}]. Any path implied by your weights must keep the 2D path strictly within these bounds.
        2) Balance between exploration and exploitation. 
        3) Search both the positive and the negative values.
        4) The global optimum corresponds to the minimum function value, which is close to f(weights). If your current f(weights) is significantly higher than that, you should prioritize exploration over exploitation.
        5)You must avoid proposing weights that result in trajectories going outside the defined XY WORKSPACE LIMITS (meters): x ∈ [{xmin:.3f}, {xmax:.3f}], y ∈ [{ymin:.3f}, {ymax:.3f}], as trajectories with waypoints out of bounds are invalid and fail to execute correctly. Analyze the `Historical Feedback Summary` to understand which past weights caused poor bounds compliance and avoid similar solutions.

    Next :
         You will see examples of the dmp weights and their corresponding function value f(weights):
          (WORKSPACE LIMITS (meters): x ∈ [-0.976, 1.175], y ∈ [-0.327, 0.198])
          # Example 1:
         weights =  {json.dumps(w_example1)} 
        f(weights) =  {json.dumps(total1)} 


          # Example 2:
          (WORKSPACE LIMITS (meters): x ∈ [-0.697, 0.695], y ∈ [-0.213, 0.116])
         weights =  {json.dumps(w_example2)},
        f(weights) =  {json.dumps(total2)}



# Historical Feedback Summary (Full History Up to Iteration {iter_idx - 1}):
     {feedback_text}


    Now you are at iteration {iter_idx} out of {MAX_ITERS}.  Please provide the results in the indicated format. Do not provide any additional texts.
    """


# Your complete call_gemini function with rotation
import random

_call_gemini_lock = threading.Lock()


def call_gemini(prompt: str) -> str:
    """
    Gemini call with exponential backoff and persistent round-robin key rotation.
    """
    API_KEYS = [
        "GOOGLE_API_KEY_1",
        "GOOGLE_API_KEY_2",
        "GOOGLE_API_KEY_6",
        "GOOGLE_API_KEY_5",
        "GOOGLE_API_KEY_3",
        "GOOGLE_API_KEY_4",
    ]

    max_retries_per_key = 7
    base_wait_time = 4
    backoff_factor = 2
    max_sleep_cap = 45

    n = len(API_KEYS)
    if n == 0:
        raise RuntimeError("No API key variables configured.")

    if not hasattr(call_gemini, "_active_idx"):
        call_gemini._active_idx = 0

    with _call_gemini_lock:
        start_idx = call_gemini._active_idx % n

    def _retryable(err: Exception) -> bool:
        s = str(err).lower()
        return (
                "429" in s or
                "503" in s or
                "502" in s or
                "504" in s or
                "temporarily unavailable" in s or
                "timeout" in s or
                "timed out" in s or
                "connection reset" in s or
                "econnreset" in s or
                "unavailable" in s
        )

    last_error = None

    for offset in range(n):
        api_index = (start_idx + offset) % n
        api_var = API_KEYS[api_index]
        api_key = os.environ.get(api_var)

        if not api_key:
            print(f"️ {api_var} not found in environment, skipping...")
            continue

        print(f" Using {api_var} (index {api_index + 1}/{n})")
        try:
            client = genai.Client(api_key=api_key)
        except Exception as e:
            print(f" Failed to initialize client for {api_var}: {e}")
            last_error = e
            continue

        for attempt in range(max_retries_per_key):
            try:
                resp = client.models.generate_content(
                    model=GEMINI_MODEL,
                    contents=prompt,
                    config={"temperature": 0.2},
                )
                text = getattr(resp, "text", None) or str(resp)

                with _call_gemini_lock:
                    call_gemini._active_idx = api_index
                return text.strip()

            except Exception as e:
                last_error = e
                if _retryable(e):
                    sleep_time = min(base_wait_time * (backoff_factor ** attempt) + random.uniform(0, 1.5),
                                     max_sleep_cap)
                    print(
                        f" Transient Gemini error on {api_var}: {e}. "
                        f"Retrying in {sleep_time:.1f}s... ({attempt + 1}/{max_retries_per_key})"
                    )
                    time.sleep(sleep_time)
                    continue
                else:
                    print(f"❌ Non-retryable error on {api_var}: {e}")
                    break

        print(f" {api_var} exhausted after {max_retries_per_key} retries. Trying next key...")

    with _call_gemini_lock:
        call_gemini._active_idx = (start_idx + 1) % n

    raise RuntimeError(f"All Gemini API keys failed after rotation. Last error: {last_error}")


def parse_ollama_weights(out_text):
    """
    Parse the LLM response to extract a 2xN_BFS weight matrix.
    """
    text = out_text.strip()

    if text.startswith("```"):
        text = re.sub(r"^```[^\n]*\n|\n```$", "", text, flags=re.MULTILINE).strip()

    try:
        obj = json.loads(text)
        cand = obj.get("weights", None)
        if isinstance(cand, list):
            return row_to_2x50(cand)
    except Exception:
        pass

    nums = re.findall(r"[-+]?\d*\.?\d+", text)
    if len(nums) >= 2 * N_BFS:
        return row_to_2x50([float(x) for x in nums[:2 * N_BFS]])

    raise ValueError("Could not parse weights from LLM output")


def save_dialog(iter_idx, prompt, response):
    pid = f"iter_{iter_idx:03d}_{uuid.uuid4().hex[:8]}"
    with open(os.path.join(DIALOG_DIR, pid + "_prompt.txt"), "w", encoding="utf-8") as f:
        f.write(prompt)
    with open(os.path.join(DIALOG_DIR, pid + "_response.txt"), "w", encoding="utf-8") as f:
        f.write(response)


def append_weight_history(csv_path, iter_idx, tag, w2):
    """Append a single row to weight_history.csv."""
    flat = list(map(float, w2.reshape(-1)))
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            header = ["iter", "timestamp", "tag"] + [f"w{i}" for i in range(2 * N_BFS)]
            w.writerow(header)
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), tag] + flat)


# ============================================
# MAIN FUNCTION - Complete from enhancedll.py
# ============================================

def main():
    # Print configuration
    exp_name = os.environ.get("EXPERIMENT_NAME", "default")
    is_headless = os.environ.get("HEADLESS", "0") == "1"

    print("\n" + "=" * 70)
    print(f" Enhanced DMP Controller - {exp_name}")
    print("=" * 70)
    print(f" Base Directory: {BASE_DIR}")
    print(f" Max Iterations: {MAX_ITERS}")
    print(f"  Viewer Mode: {'Headless ' if is_headless else 'Visual'}")
    api_keys = [k for k in os.environ if k.startswith('GOOGLE_API_KEY_')]
    print(f" API Keys Available: {len(api_keys)}")
    print("=" * 70 + "\n")

    ensure_dirs()

    weight_history = []

    # Bootstrap weights.csv from weights.txt if missing
    if not os.path.exists(WEIGHTS_CSV):
        if not os.path.exists(WEIGHTS_TXT):
            raise FileNotFoundError(f"Missing {WEIGHTS_CSV} and {WEIGHTS_TXT}")

        flat0 = parse_weights_text(WEIGHTS_TXT)
        w0 = row_to_2x50(flat0)
        write_weights_csv(WEIGHTS_CSV, w0)
        print(f" Initialized {WEIGHTS_CSV} from {WEIGHTS_TXT} → shape {w0.shape}")
        append_weight_history(WEIGHT_HISTORY_CSV, 0, "executed", w0)

    # Controller (viewer is already patched if headless)
    print(" Initializing robot controller...")
    controller = EnhancedDMPController()

    # Get bounds for prompt
    bounds = {
        "xmin": controller.x_min, "xmax": controller.x_max,
        "ymin": controller.y_min, "ymax": controller.y_max,
    }

    if not os.path.exists(CSV_MOVE_PATH):
        raise FileNotFoundError(f"Demo path not found: {CSV_MOVE_PATH}")
    demo_xy = read_move_csv(CSV_MOVE_PATH)

    # Prime DMP ONCE from move.csv
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)
    dmp.imitate_path(y_des=demo_xy.T)

    print(" Controller initialized successfully\n")

    # Main optimization loop
    for it in range(1, MAX_ITERS + 1):
        print(f"\n{'=' * 70}")
        print(f" Iteration {it}/{MAX_ITERS} - {exp_name}")
        print(f"{'=' * 70}")

        # Reset simulation
        controller.hard_reset_from_home()

        # Read current weights
        w2 = read_weights_csv(WEIGHTS_CSV)
        w_flat = w2.reshape(-1)
        append_weight_history(WEIGHT_HISTORY_CSV, it, "executed", w2)

        # Apply weights
        dmp.w = w2.copy()
        dmp.reset_state()

        # Convert one full cycle to joint trajectory
        model, data = controller.model, controller.data
        site_id = controller.site_id
        joint_names = controller.joint_names
        start_joints = get_joint_positions(model, data, joint_names)

        joint_traj = []
        task_trajectory = []
        steps = int(dmp.timesteps)
        keep_every = max(1, int(DECI_BUILD))

        print(f" Generating trajectory with {steps} DMP steps...")

        for i in range(steps):
            y, _, _ = dmp.step()
            target_3d = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
            task_trajectory.append(target_3d)

            ok, err_val = enhanced_ik_solver(
                model, data, site_id, target_3d, joint_names,
                max_iters_per_wp=IK_MAX_ITERS, print_every=1000000
            )
            if not ok:
                save_ik_error(it, i, target_3d, (err_val if err_val is not None else float("nan")), IK_ERROR_CSV)
                continue
            if i % keep_every == 0:
                joint_traj.append(get_joint_positions(model, data, joint_names).copy())

        if not joint_traj:
            print(f"  iter {it}: No joints generated, skipping execution.")
        else:
            # Restore start joints so playback is clean
            set_joint_positions(model, data, joint_names, start_joints)

            print(f"️  iter {it}: Executing {len(joint_traj)} joint waypoints...")
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt * 2)

        # Save trajectory data for this iteration
        save_trajectory_data(it, task_trajectory, TRAJECTORY_CSV)

        # Compute cost via grid counter
        grid = controller.count_balls_in_grid()
        controller.grid_count = grid
        total_balls = int(np.sum(grid))

        log_iteration(it, grid, total_balls, len(joint_traj), ITER_LOG_CSV)
        print(f" iter {it}: Cost (total balls) = {total_balls}")
        print(f"   Grid distribution: {grid.tolist()}")

        # Flush output for monitoring
        sys.stdout.flush()

        if total_balls == 0:
            print(f" iter {it}: Success! No balls left.")
            break

        # Load and analyze trajectory history
        trajectory_history = load_trajectory_history(TRAJECTORY_CSV, TRAJECTORY_HISTORY_WINDOW)
        trajectory_analysis = analyze_trajectory_performance(trajectory_history, bounds)

        iter_log_data = load_iteration_log(ITER_LOG_CSV)
        traj_feedback_data = load_traj_feedback(TRAJECTORY_CSV)

        # Ask LLM for NEW weights
        hist_slice = weight_history[-HISTORY_WINDOW:] if HISTORY_WINDOW > 0 else weight_history

        prompt = enhanced_ollama_prompt(
            w_flat, grid, total_balls, it, hist_slice,
            trajectory_history, trajectory_analysis, bounds,
            ik_error_summary=None,
            iter_log_data=iter_log_data,
            traj_feedback_data=traj_feedback_data,
            feedback_window=40000
        )

        try:
            print(f" Querying LLM for next weights...")
            response = call_gemini(prompt)

        except Exception as e:
            print(f" iter {it}: Gemini error: {e}. Reusing previous weights.")
            time.sleep(1.0)
            continue

        save_dialog(it, prompt, response)

        try:
            w_next = parse_ollama_weights(response)  # (2,50)
            print(f" Successfully parsed new weights from LLM")
        except Exception as e:
            print(f" iter {it}: Failed to parse LLM weights: {e}. Reusing previous weights.")
            time.sleep(1.0)
            continue

        append_weight_history(WEIGHT_HISTORY_CSV, it, "proposed", w_next)
        write_weights_csv(WEIGHTS_CSV, w_next)
        print(f" iter {it}: Updated {WEIGHTS_CSV} with new weights from LLM.")

        weight_history.append(w_next.reshape(-1).tolist())

        # Sleep between iterations (configurable via env)
        iter_sleep = int(os.environ.get("ITER_SLEEP", "40"))
        print(f" Sleeping {iter_sleep}s before next iteration...")
        time.sleep(iter_sleep)

    print(f"\n{'=' * 70}")
    print(f"Experiment {exp_name} completed!")
    print(f"{'=' * 70}\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n️  Interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n Fatal error: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)
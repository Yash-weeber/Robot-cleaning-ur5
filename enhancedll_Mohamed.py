# !/usr/bin/env python3
#%%
import os
import re
import csv
import json
import time
import uuid
import builtins
import subprocess
import numpy as np
import pdb
import mujoco
import threading
from google import genai
import pandas as pd
from utils.draw_shapes import infinity_trajectory, square_trajectory, triangle_trajectory, circle_trajectory, elipsoid_trajectory, rectangle_trajectory
from utils.obstacle_avoidance import *
import dotenv
import ollama

keys_file_path = "./keys.env"
API_KEYS = dotenv.dotenv_values(keys_file_path)
dotenv.load_dotenv(keys_file_path)
# print("Loaded API keys:", API_KEYS)

os.environ["MUJOCO_GL"] = "egl"


d = time.strftime("%Y-%m-%d %H-%M-%S")
optimum = 0.0

OLLAMA_MODEL = "gpt-oss:120b"
#%%
# # ====== EDIT THESE PATHS ======
CSV_MOVE_PATH = f"./Results/logs/{d}/move.csv"
WEIGHTS_TXT = f"./Results/logs/{d}/weight.txt"
WEIGHTS_TXT2 = f"./Results/logs/{d}/weight2.txt"
BASE_DIR = f"./Results/"
TRAJ_TXT1 = f"./Results/logs/{d}/traject1.txt"
TRAJ_TXT2 = f"./Results/logs/{d}/traject2.txt"
TOTAL1 = f"./Results/logs/{d}/total1.txt"
TOTAL2 = f"./Results/logs/{d}/total2.txt"
GRID1 = f"./Results/logs/{d}/gridlist1.txt"
GRID2 = f"./Results/logs/{d}/gridlist2.txt"
# # ==============================
# ====== EDIT THESE PATHS ======
# CSV_MOVE_PATH   = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/logs/move.csv"
# WEIGHTS_TXT     = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/logs/weight.txt"
# BASE_DIR        = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/"
# ==============================

HISTORY_WINDOW = 25
TRAJECTORY_HISTORY_WINDOW = 40  # NEW: Track last 20 iterations of X,Y trajectories
n_warmup = 20 # number of initial ICL examples
seed_number = 42
feedback_window = 30  # number of recent iterations to summarize for feedback
step_size = 50
random_scale = 10.0

LOGDIR = os.path.join(BASE_DIR, "logs", "best_prompt-walled-stepsize-20-hist", d)
WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
DMP_TRAJECTORY_CSV = os.path.join(LOGDIR, "dmp_trajectory_feedback.csv")  # NEW: Store X,Y trajectories per iteration
EE_TRAJECTORY_CSV = os.path.join(LOGDIR, "ee_trajectory.csv")
IK_ERROR_CSV = os.path.join(LOGDIR, "ik_errors.csv")
IK_ERROR_HISTORY_WINDOW = 40 # how many past iterations of IK errors to summarize

N_BFS = 10
MAX_ITERS = 400
IK_MAX_ITERS = 50
DECI_BUILD = 2  # keep every k-th DMP step when building joints (1=all)
INIT_LAMBDA = 0.15
TOL = 1e-3
PRINT_EVERY = 60
ANIMATION_DURATION = 4.0
ANIMATION_FPS = 75

GEMINI_MODEL = "gemini-2.5-flash"  # use Pro or gemini-2.0-flash if you want faster/cheaper
GEMINI = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY_1"))

# builtins.input = lambda *a, **k: "7"

from testiing_2 import (
    EnhancedDMPController, MOP_Z_HEIGHT,
    enhanced_ik_solver, get_joint_positions, set_joint_positions
)
from pydmps.dmp_rhythmic import DMPs_rhythmic

# controller = EnhancedDMPController()
# bounds = {
#     "xmin": controller.x_min, "xmax": controller.x_max,
#     "ymin": controller.y_min, "ymax": controller.y_max,
# }

# DELTA_ABS = 5.0
# DELTA_REL = 0.08


# ----------------- utils -----------------

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

    # Resize each axis from cur_n_bfs -> N_BFS using linear interpolation
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


# def read_weights_csv(path):
#     with open(path, "r", encoding="utf-8") as f:
#         txt = f.read()
#     nums = re.findall(r"[-+]?\d*\.?\d+", txt)
#     if not nums:
#         raise ValueError(f"No numbers in {path}")
#     return row_to_2x50([float(x) for x in nums])
def read_weights_csv(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", txt)
    vals = [float(x) for x in nums]

    need = 2 * N_BFS
    if len(vals) < need:
        raise ValueError(f"{path} has only {len(vals)} numbers, need {need} (2*N_BFS).")

    if len(vals) % 2 != 0:
        # odd length -> drop the last stray token
        print(f"Warning: {path} has odd length ({len(vals)}). Dropping last value.")
        vals = vals[:-1]

    if len(vals) > need:
        # avoid mixing in accidental extra numbers; keep the first exact set
        print(f"Warning: {path} has {len(vals)} values. Trimming to the first {need}.")
        vals = vals[:need]

    return row_to_2x50(vals)



def read_move_csv(path):
    try:
        # Try named columns
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        if data.dtype.names and {"x", "y"}.issubset(data.dtype.names):
            xy = np.column_stack([data["x"], data["y"]]).astype(float)
            if xy.ndim == 2 and xy.shape[1] >= 2:
                return xy
    except Exception:
        pass

    # Fallback: two columns
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


# NEW: Functions for trajectory feedback
def save_trajectory_data(iter_idx, task_trajectory, csv_path):
    """Save X,Y trajectory coordinates for this iteration."""
    file_exists = os.path.exists(csv_path)
    # if iter_idx % 2 == 1:

    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "step", "x", "y", "timestamp"])

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        for step_idx, target in enumerate(task_trajectory):
            x, y = target[0], target[1]  # Extract X,Y (Z is constant)
            w.writerow([iter_idx, step_idx, float(x), float(y), timestamp])



def load_trajectory_history(csv_path, max_iters=20):
    """Load last max_iters iterations of trajectory data."""
    if not os.path.exists(csv_path):
        return {}

    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}

        # Group by iteration
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

        # Return only last max_iters iterations
        sorted_iters = sorted(trajectory_history.keys())
        if len(sorted_iters) > max_iters:
            sorted_iters = sorted_iters[-max_iters:]

        return {k: trajectory_history[k] for k in sorted_iters}

    except Exception as e:
        print(f"Warning: Could not load trajectory history: {e}")
        return {}

def save_ik_error(iter_idx, step_idx, target_3d, error_val, csv_path):
    """Append a single IK failure row: (iter, step, x, y, z, error_m, timestamp)."""
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
    """Return dict {iter: [ {step,x,y,z,error_m}, ... ]} for last max_iters iterations."""
    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        # Ensure we can iterate even when a single row is present
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
    """Compute per-iter IK failure stats and include a few sample failures."""
    summary = {}
    for it, entries in error_history.items():
        if not entries:
            continue
        errs = [e["error_m"] for e in entries]
        summary[it] = {
            "num_failures": int(len(errs)),
            "max_error_m": float(np.max(errs)),
            "mean_error_m": float(np.mean(errs)),
            "sample": entries[:3]  # first few failures (step,x,y,z,error_m)
        }
    return summary

def analyze_trajectory_performance(trajectory_data, bounds):
    """Analyze trajectory quality: coverage, bounds compliance, smoothness."""
    if not trajectory_data:
        return {}

    analysis = {}

    for iter_num, traj_points in trajectory_data.items():
        if not traj_points:
            continue

        xs = [p["x"] for p in traj_points]
        ys = [p["y"] for p in traj_points]

        # Bounds compliance
        x_in_bounds = all(bounds["xmin"] <= x <= bounds["xmax"] for x in xs)
        y_in_bounds = all(bounds["ymin"] <= y <= bounds["ymax"] for y in ys)

        # Coverage metrics
        x_range_covered = (max(xs) - min(xs)) / (bounds["xmax"] - bounds["xmin"])
        y_range_covered = (max(ys) - min(ys)) / (bounds["ymax"] - bounds["ymin"])

        # Smoothness (path length vs direct distance)
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
    """Load llm_iteration_log.csv into a dictionary keyed by iteration number."""
    if not os.path.exists(csv_path):
        return {}
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        if data.size == 0:
            return {}
        log_data = {}
        for row in np.atleast_1d(data):
            it = int(row["iter"])
            cells = [int(row[f"cell{i}"]) for i in range(6)]  # 2x3 grid = 6 cells
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
    """Load trajectory_feedback.csv into a dictionary keyed by iteration number."""
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
# delta_abs=5.0, delta_rel=0.08
# IK ERROR FEEDBACK  which you will use to reduce the movement that you did so the error is less overtime for the motion(last {IK_ERROR_HISTORY_WINDOW} iters):
# {json.dumps(ik_error_summary or {}, ensure_ascii=False)} this is only used when

# - TRAJECTORY QUALITY: Based on the trajectory history, optimize for:
#   * Bounds compliance (keep all X,Y within workspace)
#   * Good coverage (high x_coverage and y_coverage values)
#   * Smooth motion (reasonable smoothness metric)
#   * Effective cleaning (correlation between trajectory path and ball reduction)
def enhanced_ollama_prompt(prev_w_flat, grid_mat, total_balls, iter_idx, history,
                           trajectory_history, trajectory_analysis, bounds,  ik_error_summary=None,iter_log_data=None,traj_feedback_data=None, feedback_window=50
                           ):
    try:
        w_example1 = parse_weights_text(WEIGHTS_TXT).tolist()
    except Exception:
        w_example1 = []
    try:
        w_example2 = parse_weights_text(WEIGHTS_TXT2).tolist()
    except Exception:
        w_example2 = []
    try :
        trajectoy_1 = parse_weights_text(TRAJ_TXT1).tolist()
        trajectoy_2 = parse_weights_text(TRAJ_TXT2).tolist()
        total1=parse_weights_text(TOTAL1).tolist()
        total2 = parse_weights_text(TOTAL2).tolist()
        grid1 = parse_weights_text(GRID1).tolist()
        grid2 = parse_weights_text(GRID2).tolist()
    except:
        trajectoy_1 =[]
        trajectoy_2=[]
        total1=[]
        total2 =[]
        grid1 =[]
        grid2 = []

    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]
    grid_list = grid_mat.tolist()

    # --- NEW: Define Strict Global Limits for Failure Check (Based on your request) ---
    STRICT_X_MIN = -1.050
    STRICT_X_MAX = 1.050
    STRICT_Y_MIN = -0.650
    STRICT_Y_MAX = 0.650
    # === 2. NEW SECTION: Analyze historical iteration + weight data ===
    best_iter_summary = ""
    best_weight_feedback = ""
    best_iter_data = None
    best_weights = None
    # The code `w_df` is not doing anything as it is just a variable name without any associated
    # operation or assignment.
    w_df = None

    try:
        # Load iteration and weight data if present
        if os.path.exists(ITER_LOG_CSV) and os.path.exists(WEIGHT_HISTORY_CSV):
            iter_df = pd.read_csv(ITER_LOG_CSV)
            w_df = pd.read_csv(WEIGHT_HISTORY_CSV)

            # Expect columns: iter, traj_waypoints, total_balls, cell0...cell5
            # and weights_history: iter, w0...wn + maybe 'executed'
            iter_df = iter_df.dropna(subset=["total_balls", "traj_waypoints"])
            iter_df["iter"] = iter_df["iter"].astype(int)

            # Define a waypoint sanity threshold (exclude truncated or invalid runs)
            waypoint_median = iter_df["traj_waypoints"].median()
            waypoint_cutoff = max(waypoint_median * 0.9, waypoint_median - 150)

            valid_df = iter_df[iter_df["traj_waypoints"] >= waypoint_cutoff]

            # Find the best run (lowest f(weights) among valid)
            # if not valid_df.empty:
            #     best_iter = valid_df.loc[valid_df["total_balls"].idxmin()]
            #     best_iter_num = int(best_iter["iter"])
            #     best_iter_data = best_iter

            #     # Fetch matching weights from weights_history
            #     weight_row = w_df[w_df["iter"] == best_iter_num]
            #     if not weight_row.empty:
            #         # Flatten weight columns only (ignore non-float columns)
            #         best_weights = [
            #             float(x) for x in weight_row.iloc[0].values[1:]
            #             if str(x).replace('.', '', 1).replace('-', '', 1).isdigit()
            #         ]

                # # Construct readable text
                # best_iter_summary = (
                #     f"\n# Best Historical Policy (Iteration {best_iter_num}):\n"
                #     f" f(weights): {best_iter['total_balls']} | "
                #     #
                #     # f"Grid: {[best_iter.get(f'cell{i}', 'N/A') for i in range(6)]}\n"
                # )

                # if best_weights:
                #     w_min, w_max = np.min(best_weights), np.max(best_weights)
                #     w_range = w_max - w_min
                #     wrange = 340
                #     max_step = round(wrange / 12, 3)
                #     best_weight_feedback = (
                #         f"# Weight Range Insight:\n"
                #         f"  Weight range = [-170,170]  "
                #         f"(Δ = {wrange:.3f}) → Suggested MAX_STEP = {max_step}\n"
                #     )

    except Exception as e:
        best_iter_summary = f"\n# ⚠️ Error analyzing historical weights: {str(e)}\n"

    # Format trajectory history for LLM
    traj_feedback = {}
    for iter_num, analysis in trajectory_analysis.items():
        if iter_num in trajectory_history:
            # Sample trajectory points (first, middle, last)
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
        """Checks if the trajectory range violates the strict global limits."""
        return (x_min < STRICT_X_MIN or x_max > STRICT_X_MAX or
                y_min < STRICT_Y_MIN or y_max > STRICT_Y_MAX)



    # Construct recent iteration performance summary
    # if iter_log_data:
    #     recent_iters = sorted([k for k in iter_log_data.keys() if k < iter_idx])[-feedback_window:]
    #     if recent_iters:
    #         feedback_text += f"\n# Iteration Performance Summary (last {len(recent_iters)} iterations):\n"
    #         for i in recent_iters:
    #             entry = iter_log_data[i]
    #             current_f_weights = entry['total_balls']
    #             is_failed_iter = False
    #             if i in traj_feedback_data:
    #                 pts = traj_feedback_data[i]
    #                 if pts:
    #                     x_values = [p['x'] for p in pts]
    #                     y_values = [p['y'] for p in pts]

    #                     x_min_traj = min(x_values)
    #                     x_max_traj = max(x_values)
    #                     y_min_traj = min(y_values)
    #                     y_max_traj = max(y_values)

    #                     if is_bounds_failed(x_min_traj, x_max_traj, y_min_traj, y_max_traj):
    #                         is_failed_iter = True

    #                 # Check for explicit failure cost if one was applied (for IK failures not caught by bounds)
    #                 # We prioritize bounds check, but keep the high cost check as a secondary signal for other hard errors.

    #             if is_failed_iter:

    #                 feedback_text += (
    #                     f"  {_ordinal(i)} iteration: f(weights)={current_f_weights} (FAILURE: Out of Bounds ❌)\n"
    #                 )
    #             else:
    #                 feedback_text += (
    #                     f"  {_ordinal(i)} iteration: f(weights)={current_f_weights}, "
    #                 )
                # feedback_text += (
                #     f"  {_ordinal(i)} iteration: f(weights)={entry['total_balls']}, "
                #
                #
                # )

    # if traj_feedback_data:
    #     recent_traj_iters = sorted([k for k in traj_feedback_data.keys() if k < iter_idx])[-feedback_window:]
    #     if recent_traj_iters:
    #         feedback_text += f"\n# Trajectory Path Analysis (last {len(recent_traj_iters)} iterations):\n"
    #         for i in recent_traj_iters:
    #             pts = traj_feedback_data[i]
    #             if pts:
    #                 # Sample first 5 points to show path start
    #                 sample_start = [(round(p['x'], 3), round(p['y'], 3)) for p in pts[:500]]
    #                 # Sample last 5 points to show path end
    #                 sample_end = [(round(p['x'], 3), round(p['y'], 3)) for p in pts[-500:]]
    #                 feedback_text += (
    #                     f"  {_ordinal(i)} iteration: path_start={sample_start}, "
    #                     f"path_end={sample_end}, total_steps={len(pts)}\n"
    #                 )
    if w_df is not None and not w_df.empty:
        try:
            executed_df = w_df[(w_df['tag'] == 'executed') & (w_df['iter'] < iter_idx)].copy()
            executed_df['iter'] = executed_df['iter'].astype(int)
            recent_executed = executed_df.sort_values(by='iter', ascending=False).head(feedback_window)

            if not recent_executed.empty:
                # feedback_text += f"\n# Executed Weights History (last {len(recent_executed)} executed iterations):\n"

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
                                # Highlight the specific failure coordinates
                                bounds_info = (
                                    # f", **OUT-OF-BOUNDS FAILURE**: x_range=[{x_min_traj}, {x_max_traj}], "
                                    f", x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )
                            else:
                                bounds_info = (
                                    f", x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )

                            # bounds_info = (
                            #     f", x_range=[{x_min_traj}, {x_max_traj}], "
                            #     f"y_range=[{y_min_traj}, {y_max_traj}]"
                            # )


                    if weights:
                        # Convert the list of weights into a JSON string to include in the prompt
                        # Rounding to 4 decimal places keeps it clean
                        rounded_weights = [round(w, 4) for w in weights]
                        failure_tag = " (FAILED)" if is_failed_iter else ""
                        iter_string = f"Examples {iter_num + n_warmup}:" if iter_num < 1 else f"Iteration {iter_num}:"
                        feedback_text += (
                            f"{iter_string} weights={json.dumps(rounded_weights)}"
                            f"{bounds_info}, "
                            f" f(weights)={current_f_weights}\n"
                        )
        except Exception as e:
            feedback_text += f"# ⚠️ Error processing executed weights history: {str(e)}\n"


    # # Add performance trends analysis if we have enough data
    # if iter_log_data and len([k for k in iter_log_data.keys() if k < iter_idx]) >= 3:
    #     recent_3 = sorted([k for k in iter_log_data.keys() if k < iter_idx])[-10:]
    #     ball_trend = [iter_log_data[k]['total_balls'] for k in recent_3]
    #     if ball_trend[0] > ball_trend[-1]:
    #         trend = "IMPROVING (f(weights) decreasing)"
    #     elif ball_trend[0] < ball_trend[-1]:
    #         trend = "WORSENING (f(weights) increasing)"
    #     else:
    #         trend = "STABLE (f(weights) unchanged)"
    #     # feedback_text += f"\n# Recent Trend: {trend} - Last 3 iterations: {ball_trend}\n"

    # if best_iter_summary or best_weight_feedback:
    #     feedback_text += "\n# Historical Weight Performance Feedback:\n"
    #     feedback_text += best_iter_summary + best_weight_feedback

    return f"""
You are a good global optimizer, helping me find the global minimum of a mathematical function f(weights). I will give you the function evaluation f(weights) and the current iteration number at each step. Your goal is to propose input values that efficiently lead us to the global minimum within a limited number of iterations ({MAX_ITERS}).

# Regarding the policy and weights:
    policy is parameterized by a set of weights that define a 2D trajectory via Dynamic Movement Primitives (DMPs). 
    There are {N_BFS} basis functions per dimension, resulting in a total of {2 * N_BFS} weights.
    Weight values should be floats, and can be both positive and negative.
    The policy defines the trajectory in the XY workspace.
    The generated trajectory must strictly stay within the defined XY workspace limits.
    The function f(weights) evaluates the cost of the policy.
    
# Here's how we will interact :
    1. I will provide you max steps ({MAX_ITERS}) along with training examples which includes weights for the policy, the ranges of the trajecotry in the XY workspace and its corresponding function value f(weights) for each example. 
    2. You will provide the response in exact following format: 
        * Line 1: a new set of {2 * N_BFS} float weights as an array, aiming to minimize the functions value f(weights).
        * Line 2: details explanation of why you chose the weights.
    3. I will then provide the function's f(weights) at that point and the current iteration.
    4. You will repeat the steps from 2-3 until we will reach a maximum number of iteration.
    

# Remember :
    1. **XY workspace limits: x ∈ [{xmin:.3f}, {xmax:.3f}], y ∈ [{ymin:.3f}, {ymax:.3f}]. Any proposed weights must keep the trajectory strictly within these bounds.**
    2. **The global optimum should be around {optimum}.** If you are higher than that, this is a local optimum. You should  explore instead of exploiting.
    3. Search both the positive and the negative values. **During exploration, use search step size of {step_size}**
    

Next, You will see examples of the weights and their corresponding function value f(weights) and XY workspace range:
{feedback_text}


Now you are at iteration {iter_idx} out of {MAX_ITERS}.  Please provide the results in the indicated format.
"""



import time
import random

_call_gemini_lock = threading.Lock()

def call_gemini(prompt: str) -> str:
    """
    Gemini call with exponential backoff and persistent round-robin key rotation.
    - Starts from the last successful key across calls.
    - On success: pointer stays on that key.
    - If all keys fail in this call: pointer advances by 1 (keeps the cycle moving).
    - Adds thread safety, broader transient error detection, and capped backoff.
    """
    API_KEYS = [

        "GOOGLE_API_KEY_1",
        "GOOGLE_API_KEY_2",
        "GOOGLE_API_KEY_6",



        "GOOGLE_API_KEY_3",
        "GOOGLE_API_KEY_4",
        "GOOGLE_API_KEY_5",

    ]

    # Tuning: keep retries modest to avoid long stalls on a single key
    max_retries_per_key = 7
    base_wait_time = 4
    backoff_factor = 2
    max_sleep_cap = 45  # cap each sleep to avoid runaway waits

    n = len(API_KEYS)
    if n == 0:
        raise RuntimeError("No API key variables configured.")

    # Initialize persistent pointer if missing
    if not hasattr(call_gemini, "_active_idx"):
        call_gemini._active_idx = 0

    with _call_gemini_lock:
        start_idx = call_gemini._active_idx % n

    # Helper: decide if transient
    def _retryable(err: Exception) -> bool:
        s = str(err).lower()
        # Include rate limits, common 5xx/service errors, and timeouts/resets
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
            print(f"⚠️ {api_var} not found in environment, skipping...")
            continue

        print(f"🔑 Using {api_var} (index {api_index + 1}/{n})")
        try:
            client = genai.Client(api_key=api_key)
        except Exception as e:
            print(f"❌ Failed to initialize client for {api_var}: {e}")
            last_error = e
            continue

        for attempt in range(max_retries_per_key):
            try:
                resp = client.models.generate_content(
                    model=GEMINI_MODEL,
                    contents=prompt,
                    # config={"temperature": 0.2},
                )
                text = getattr(resp, "text", None) or str(resp)

                # Record success pointer
                with _call_gemini_lock:
                    call_gemini._active_idx = api_index
                return text.strip()

            except Exception as e:
                last_error = e
                if _retryable(e):
                    sleep_time = min(base_wait_time * (backoff_factor ** attempt) + random.uniform(0, 1.5),
                                     max_sleep_cap)
                    print(
                        f"⚠️ Transient Gemini error on {api_var}: {e}. "
                        f"Retrying in {sleep_time:.1f}s... ({attempt + 1}/{max_retries_per_key})"
                    )
                    time.sleep(sleep_time)
                    continue
                else:
                    print(f"❌ Non-retryable error on {api_var}: {e}")
                    break

        print(f"🔁 {api_var} exhausted after {max_retries_per_key} retries. Trying next key...")

    # All keys failed in this call; advance the pointer so the next call starts at the next key
    with _call_gemini_lock:
        call_gemini._active_idx = (start_idx + 1) % n

    raise RuntimeError(f"All Gemini API keys failed after rotation. Last error: {last_error}")

def call_ollama(prompt: str, model:str = OLLAMA_MODEL, **kwargs) -> str:
    """
    Call Ollama LLM with the given prompt and return the text response.
    """
    try:
        # pdb.set_trace()
        # breakpoint()
        response = ollama.chat(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            **kwargs
            )
        # Ollama returns a dict with a 'message' key containing another dict with 'content'
        # breakpoint()
        return response["message"]["content"].strip()
    except Exception as e:
        raise RuntimeError(f"Ollama API call failed: {e}")


def parse_ollama_weights(out_text):
    """
    Parse the Ollama LLM response to extract a 2xN_BFS weight matrix.
    Handles both pure JSON and messy text (code fences or extra commentary).
    """
    text = out_text.strip()

    # 🧹 Clean up possible code fences like ```json ... ```
    if text.startswith("```"):
        text = re.sub(r"^```[^\n]*\n|\n```$", "", text, flags=re.MULTILINE).strip()

    # 🧠 Try direct JSON parsing first
    try:
        obj = json.loads(text)
        cand = obj.get("weights", None)
        if isinstance(cand, list):
            return row_to_2x50(cand)
    except Exception:
        pass  # not pure JSON, fall back below

    # 🔢 Fallback: extract all floats from the text
    nums = re.findall(r"[-+]?\d*\.?\d+", text)
    if len(nums) >= 2 * N_BFS:
        return row_to_2x50([float(x) for x in nums[:2 * N_BFS]])

    # ❌ If nothing worked
    raise ValueError("Could not parse weights from LLM output")

def save_dialog(iter_idx, prompt, response):
    pid = f"iter_{iter_idx:03d}_{uuid.uuid4().hex[:8]}"
    with open(os.path.join(DIALOG_DIR, pid + "_prompt.txt"), "w", encoding="utf-8") as f:
        f.write(prompt)
    with open(os.path.join(DIALOG_DIR, pid + "_response.txt"), "w", encoding="utf-8") as f:
        f.write(response)


def append_weight_history(csv_path, iter_idx, tag, w2):
    """Append a single row to weight_history.csv.

    Parameters
    ----------
    csv_path : str
    iter_idx : int
        iteration number (use 0 for initial)
    tag : str
        "executed" or "proposed"
    w2 : np.ndarray
        shape (2, N_BFS)
    """
    flat = list(map(float, w2.reshape(-1)))
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            header = ["iter", "timestamp", "tag"] + [f"w{i}" for i in range(2 * N_BFS)]
            w.writerow(header)
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), tag] + flat)


# --------------- main loop ---------------

def main():
    ensure_dirs()

    weight_history = []
    
    # Controller (one viewer, no reset later)
    controller = EnhancedDMPController()
    bounds = {
        "xmin": controller.x_min, "xmax": controller.x_max,
        "ymin": controller.y_min, "ymax": controller.y_max,
    }

    # # Tune PID Gains
    # kp = [3000, 3000, 1500, 800, 500, 500]
    # kd = [150, 150, 80, 40, 20, 20]
    # controller.set_joint_pid_gains(controller.joint_names, kp, kd)
    # print("Updated PID gains.")

    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)

    # Bootstrap weights.csv from weights.txt if missing
    # if not os.path.exists(WEIGHTS_CSV):
    #     if not os.path.exists(WEIGHTS_TXT):
    # raise FileNotFoundError(f"Missing {WEIGHTS_CSV} and {WEIGHTS_TXT}")
    # x_traj, y_traj = infinity_trajectory(center=(0.0, 0.0), size=(2.0, 2.5), num_points=400, plot=False)
    # trajectory = np.vstack((x_traj, y_traj)).T
    # dmp.imitate_path(trajectory.T, plot=False)
    # # print(dmp.w)
    # write_weights_csv(WEIGHTS_CSV, dmp.w.copy())
        
        # else:
        #     flat0 = parse_weights_text(WEIGHTS_TXT)
        #     w0 = row_to_2x50(flat0)
        #     write_weights_csv(WEIGHTS_CSV, w0)
        #     print(f"Initialized {WEIGHTS_CSV} from {WEIGHTS_TXT} → shape {w0.shape}")
        #     append_weight_history(WEIGHT_HISTORY_CSV, 0, "executed", w0)

    # x_traj, y_traj = infinity_trajectory(center=(0.0, 0.0), size=(2.0, 2.5), num_points=400, plot=False)
    # trajectory = np.vstack((x_traj, y_traj)).T
    # dmp.imitate_path(trajectory.T, plot=False)

    # if not os.path.exists(CSV_MOVE_PATH):
    #     raise FileNotFoundError(f"Demo path not found: {CSV_MOVE_PATH}")
    # trajectory = read_move_csv(CSV_MOVE_PATH)

    # # Prime DMP ONCE from move.csv (y0, etc.)
    # # dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)
    # dmp.imitate_path(y_des=trajectory.T)
    
    
    n_counter = 0
    # Main optimization loop
    for it in range(1 - n_warmup, MAX_ITERS + 1):
        # Iterations
        controller.hard_reset_from_home(redraw=False)
        if it < 0:
            if (it - 1) % 5 != 0:
                pass
            else:
                # Warmup: use predefined trajectories
                if n_counter == 0:
                    x_traj, y_traj = circle_trajectory(center=(0.0, -0.1), radius=0.4, num_points=200, plot=False)
                elif n_counter == 1:
                    x_traj, y_traj = rectangle_trajectory(center=(0.0, -0.1), width=1.0, height=0.4, num_points=200, plot=False)
                elif n_counter == 2:
                    x_traj, y_traj = elipsoid_trajectory(center=(0, 0), axes_lengths=(1.0, 0.3), angle=np.pi/6, num_points=200, plot=False)
                elif n_counter == 3:
                    x_traj, y_traj = triangle_trajectory(center=(0, -0.2), side_length=1.25, num_points=200, plot=False)
                trajectory = np.vstack((x_traj, y_traj))
                trajectory = np.hstack((np.zeros((2,1)), trajectory)).T
                dmp.imitate_path(trajectory.T, plot=False)
                # print(dmp.w)
                write_weights_csv(WEIGHTS_CSV, dmp.w.copy())
                n_counter += 1

        # Read current weights
        w2 = read_weights_csv(WEIGHTS_CSV)
        
        # pdb.set_trace()
        w_flat = w2.reshape(-1)
        
        # Apply weights
        print(f"iteration {it}: w2 = {w2}")
        dmp.w = w2.copy()
        dmp.reset_state()
        append_weight_history(WEIGHT_HISTORY_CSV, it, "executed", w2.copy())

        # # --- NEW: Move to start of trajectory first ---
        # start_target_3d = np.array([dmp.y0[0], dmp.y0[1], MOP_Z_HEIGHT], dtype=float)
        # # print(f"iter {it}: Moving to start position {start_target_3d}...")
        
        # ok_start, err_start = enhanced_ik_solver(
        #     controller.model, controller.data, controller.site_id, start_target_3d, controller.joint_names,
        #     max_iters_per_wp=IK_MAX_ITERS, print_every=1000000
        # )
        
        # if not ok_start:
        #      print(f"iter {it}: Failed to move to start position (error={err_start}). Skipping.")
        #      continue
        # ----------------------------------------------

        # Convert one full cycle to joint trajectory
        model, data = controller.model, controller.data
        site_id = controller.site_id
        joint_names = controller.joint_names
        start_joints = get_joint_positions(model, data, joint_names)

        joint_traj = []
        dmp_task_trajectory = []  # NEW: Store task-space trajectory for feedback
        steps = int(dmp.timesteps)
        keep_every = max(1, int(DECI_BUILD))

        for i in range(steps):
            y, _, _ = dmp.step(tau=2.0, 
                               external_force=avoid_obstacles(dmp.y, dmp.dy, dmp.goal, 
                                                              rect_eta=0.5, obs_d0=0.25, obs_eta=25))
            target_3d = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
            dmp_task_trajectory.append(target_3d)  # NEW: Save for trajectory analysis

            ok, err_val = enhanced_ik_solver(
                model, data, site_id, target_3d, joint_names,
                max_iters_per_wp=50, print_every=1000
            )
            if not ok:
                save_ik_error(it, i, target_3d, (err_val if err_val is not None else float("nan")), IK_ERROR_CSV)
                continue
            if i % keep_every == 0:
                joint_traj.append(get_joint_positions(model, data, joint_names).copy())

        if not joint_traj:
            print(f"iter {it}: No joints generated, skipping execution.")
        else:
            # Restore start joints so playback is clean
            set_joint_positions(model, data, joint_names, start_joints)

            print(f"iter {it}: Executing {len(joint_traj)} joint waypoints...")
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt*2)

        # NEW: Save trajectory data for this iteration
        save_trajectory_data(it, dmp_task_trajectory, DMP_TRAJECTORY_CSV)
        save_trajectory_data(it, controller.ee_trajectory, EE_TRAJECTORY_CSV)

        # Compute cost via your grid counter
        grid = controller.count_balls_in_grid()
        controller.grid_count = grid  # (ny, nx), after their transpose
        total_balls = int(np.sum(grid))

        log_iteration(it, grid, total_balls, len(joint_traj), ITER_LOG_CSV)
        print(f"iter {it}: Cost (total balls) = {total_balls}, per-cell = {grid.tolist()}")

        if total_balls == 0:
            print(f"iter {it}: Done, no balls left.")
            break

        # NEW: Load and analyze trajectory history
        dmp_trajectory_history = load_trajectory_history(DMP_TRAJECTORY_CSV, TRAJECTORY_HISTORY_WINDOW)
        trajectory_analysis = analyze_trajectory_performance(dmp_trajectory_history, bounds)
        # NEW: IK failure feedback (history + summary)
        # ik_error_history = load_ik_error_history(IK_ERROR_CSV, IK_ERROR_HISTORY_WINDOW)
        # ik_error_summary = summarize_ik_errors(ik_error_history)


        
        iter_log_data = load_iteration_log(ITER_LOG_CSV)
        traj_feedback_data = load_traj_feedback(DMP_TRAJECTORY_CSV)
        # pdb.set_trace()

        # Ask LLM for NEW weights given cost, grid, prev weights, AND trajectory feedback
        hist_slice = weight_history[-HISTORY_WINDOW:] if HISTORY_WINDOW > 0 else weight_history

        # prompt = enhanced_ollama_prompt(
        #     w_flat, grid, total_balls, it, hist_slice,
        #     trajectory_history, trajectory_analysis, bounds,    ik_error_summary=ik_error_summary,
        #     max_changed=MAX_CHANGED
        #
        # )
        

        if it < 0:
            np.random.seed(seed_number+it)
            w_next = w2 + np.random.randn(2, N_BFS) * random_scale
            write_weights_csv(WEIGHTS_CSV, w_next)
        elif it >= 0:
            prompt = enhanced_ollama_prompt(
                w_flat, grid, total_balls, it+1, hist_slice,
                dmp_trajectory_history, trajectory_analysis, bounds,
                ik_error_summary=None,
                iter_log_data=iter_log_data,
                traj_feedback_data=traj_feedback_data,
                feedback_window=feedback_window
            )
            
            # save_dialog(it, prompt, '')
            # return
            try:
                # response = call_gemini(prompt)
                response = call_ollama(prompt)

            except Exception as e:

                print(f"iter {it}: API error: {e}. Reusing previous weights.")
                time.sleep(1.0)
                continue

            save_dialog(it+1, prompt, response)

            try:
                w_next = parse_ollama_weights(response)  # (2,50)
            except Exception as e:
                print(f"iter {it}: Failed to parse LLM weights: {e}. Reusing previous weights.")
                time.sleep(1.0)
                continue
        # if it > 0:
        #     return

        append_weight_history(WEIGHT_HISTORY_CSV, it+1, "proposed", w_next)
        write_weights_csv(WEIGHTS_CSV, w_next)
        print(f"iter {it}: Updated {WEIGHTS_CSV} with new weights from LLM.")

        weight_history.append(w_next.reshape(-1).tolist())
            # time.sleep(40)

    print("Loop finished. Close the viewer to exit.")


if __name__ == "__main__":
    main()

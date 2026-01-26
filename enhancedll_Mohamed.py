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
from jinja2 import Template, Environment, FileSystemLoader
from utils.call_llms import *
import traceback
import argparse



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
BASE_DIR = f"/home/melmisti/GitHub/Robot-cleaning-ur5/Results/"
TEMPLATE_DIR = "/home/melmisti/GitHub/Robot-cleaning-ur5/agent/policy/templates"

# --- Defaults (kept as defaults, but overridable via CLI) ---
HISTORY_WINDOW = 25
TRAJECTORY_HISTORY_WINDOW = 40
n_warmup = 20
seed_number = 42

feedback_window = 100
resample_rate = None
step_size = 100
random_scale = 10.0

run_type = "semantics-RL-optimizer"
template_number = 2
traj_in_prompt = False
GRID_REWARD = True
GUIDED = False
n_x_seg = 3
n_y_seg = 2

# These depend on n_x_seg/n_y_seg; will be recomputed after CLI parse in main()
x_edges = None
y_edges = None
raw_cell_cols = None

# These depend on save_results_file; will be set in main() after CLI parse
LOG_PARENT = None
LOGDIR = None
WEIGHTS_CSV = None
ITER_LOG_CSV = None
DIALOG_DIR = None
WEIGHT_HISTORY_CSV = None
DMP_TRAJECTORY_CSV = None
EE_TRAJECTORY_CSV = None
IK_ERROR_CSV = None

template_name = None
save_results_file = None

#%%
JINJA_ENV = Environment(
    loader=FileSystemLoader(TEMPLATE_DIR),
    autoescape=False,
    trim_blocks=True,
    lstrip_blocks=True,
    )

def _make_next_numeric_run_dir(parent_dir: str) -> str:
    """
    Create and return a new run directory with an incrementing integer name: 1, 2, 3, ...
    - Scans existing subfolders of parent_dir
    - Ignores non-numeric folder names
    - Creates the next available numeric folder (race-safe-ish via mkdir loop)
    """
    os.makedirs(parent_dir, exist_ok=True)

    existing = []
    for name in os.listdir(parent_dir):
        full = os.path.join(parent_dir, name)
        if os.path.isdir(full) and name.isdigit():
            existing.append(int(name))

    next_id = (max(existing) + 1) if existing else 1

    # In case two processes start at the same time, try until we succeed.
    while True:
        run_dir = os.path.join(parent_dir, str(next_id))
        try:
            os.mkdir(run_dir)  # do not use exist_ok here
            return run_dir
        except FileExistsError:
            next_id += 1

# Put runs under: ./Results/logs/best_prompt-walled-stepsize-20-hist/{N}/
# LOG_PARENT = os.path.join(BASE_DIR, "logs", save_results_file)
# LOGDIR = _make_next_numeric_run_dir(LOG_PARENT)
# WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
# ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
# DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
# WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
# DMP_TRAJECTORY_CSV = os.path.join(LOGDIR, "dmp_trajectory_feedback.csv")  # NEW: Store X,Y trajectories per iteration
# EE_TRAJECTORY_CSV = os.path.join(LOGDIR, "ee_trajectory.csv")
# IK_ERROR_CSV = os.path.join(LOGDIR, "ik_errors.csv")
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

# ----------------- utils -----------------

def _str2bool(v):
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in {"true", "t", "1", "yes", "y", "on"}:
        return True
    if s in {"false", "f", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected true/false, got: {v}")

def _parse_args():
    p = argparse.ArgumentParser(description="UR5 cleaning optimizer run config")
    p.add_argument("--run-type", type=str, default=run_type)
    p.add_argument("--step-size", type=int, default=step_size)
    p.add_argument("--feedback-window", type=int, default=feedback_window)
    p.add_argument("--template-number", type=int, default=template_number)

    p.add_argument("--traj_in_prompt", "--traj-in-prompt", type=_str2bool, default=traj_in_prompt)
    p.add_argument("--resample-rate", type=int, default=resample_rate)

    # Grid reward toggle
    p.add_argument("--grid_reward", "--grid-reward", type=_str2bool, default=GRID_REWARD)
    p.add_argument("--guided", "--guided", type=_str2bool, default=GUIDED)

    p.add_argument("--n-x-seg", type=int, default=n_x_seg)
    p.add_argument("--n-y-seg", type=int, default=n_y_seg)

    # Optional: output folder override (otherwise uses ./Results/logs/<save_results_file>/)
    p.add_argument("--log-parent", type=str, default=BASE_DIR, help="Override ./Results/logs/<save_results_file>")

    return p.parse_args()


def _compute_names(args):
    """
    Matches your existing naming logic, but driven by CLI args.
    Returns (template_name, save_results_file).
    """
    rt = args.run_type
    if args.traj_in_prompt:
        rt += f"-traj"
    if args.guided:
        rt += f"-guided"
    tmpl = (
        f"{rt}-totalcost-{args.template_number}.j2"
        if not args.grid_reward
        else f"{rt}-gridreward-{args.template_number}.j2"
    )
    
    if args.resample_rate is not None and args.traj_in_prompt:
        rt = f"{args.run_type}-traj-{args.resample_rate}"
        if args.guided:
            rt += f"-guided"
    save = (
        f"{rt}-walled-stepsize-{args.step_size}-hist-{args.feedback_window}-{args.template_number}"
        if not args.grid_reward
        else f"{rt}-walled-stepsize-{args.step_size}-hist-{args.feedback_window}-gridreward-{args.n_x_seg}x{args.n_y_seg}-{args.template_number}"
    )
    return tmpl, save


def _init_paths(save_results_file_local: str, log_parent_override: str | None = None):
    """Initialize global log paths based on save_results_file."""
    global LOG_PARENT, LOGDIR, WEIGHTS_CSV, ITER_LOG_CSV, DIALOG_DIR
    global WEIGHT_HISTORY_CSV, DMP_TRAJECTORY_CSV, EE_TRAJECTORY_CSV, IK_ERROR_CSV

    parent = (
        log_parent_override
        if log_parent_override is not None
        else os.path.join(BASE_DIR, "logs", save_results_file_local)
    )
    LOG_PARENT = parent
    LOGDIR = _make_next_numeric_run_dir(LOG_PARENT)

    WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
    ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
    DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
    WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
    DMP_TRAJECTORY_CSV = os.path.join(LOGDIR, "dmp_trajectory_feedback.csv")
    EE_TRAJECTORY_CSV = os.path.join(LOGDIR, "ee_trajectory.csv")
    IK_ERROR_CSV = os.path.join(LOGDIR, "ik_errors.csv")


def ensure_dirs():
    if LOGDIR is None or DIALOG_DIR is None:
        raise RuntimeError("LOGDIR/DIALOG_DIR not initialized. Call _init_paths(...) first.")
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

def enhanced_ollama_prompt(prev_w_flat, grid_mat, total_balls, iter_idx, history,
                           trajectory_history, trajectory_analysis, bounds,  ik_error_summary=None,iter_log_data=None,traj_feedback_data=None, feedback_window=50
                           ):
    """Generate enhanced LLM prompt with historical data and trajectory feedback."""
    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]
    grid_list = grid_mat.tolist()

    guidance_text = "The policy should result in a sinusoidal trajectory that covers the workspace, while avoiding going out of bounds. The sinusoidal sweeping motion should be along the x-axis (sweeping up and down the y-axis), smooth, and continuous." if not traj_in_prompt else "The policy should result in a sinusoidal trajectory that covers the workspace, while avoiding going out of bounds. The sinusoidal motion should sweep up and down the y-axis smoothly and continuously. Analyze the impact of each weight on the trajectory, then use the analysis to inform your weight adjustments."

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
            ee_traj_df = pd.read_csv(EE_TRAJECTORY_CSV)

            # Expect columns: iter, traj_waypoints, total_balls, cell0...cell5
            # and weights_history: iter, w0...wn + maybe 'executed'
            iter_df = iter_df.dropna(subset=["total_balls", "traj_waypoints"])
            iter_df["iter"] = iter_df["iter"].astype(int)

            # Define a waypoint sanity threshold (exclude truncated or invalid runs)
            waypoint_median = iter_df["traj_waypoints"].median()
            waypoint_cutoff = max(waypoint_median * 0.9, waypoint_median - 150)

            valid_df = iter_df[iter_df["traj_waypoints"] >= waypoint_cutoff]

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
                    cells_arr = iter_df.loc[iter_df["iter"] == iter_num, raw_cell_cols].to_numpy().reshape(-1, n_y_seg).T
                    cells_df = pd.DataFrame(cells_arr)
                    cells_df.index = [f"y:[{y_edges[j]:.2f},{y_edges[j + 1]:.2f}]" for j in range(n_y_seg)]
                    cells_df.columns = [f"x:[{x_edges[i]:.2f},{x_edges[i + 1]:.2f}]" for i in range(n_x_seg)]
                    bounds_info = ""
                    is_failed_iter = False
                    if iter_num in traj_feedback_data:
                        pts = traj_feedback_data[iter_num]
                        if pts:
                            x_values = [p['x'] for p in pts]
                            y_values = [p['y'] for p in pts]
                            # breakpoint()

                            x_min_traj = round(min(x_values), 4)
                            x_max_traj = round(max(x_values), 4)
                            y_min_traj = round(min(y_values), 4)
                            y_max_traj = round(max(y_values), 4)
                            is_failed_iter = is_bounds_failed(min(x_values), max(x_values), min(y_values), max(y_values))

                            if is_failed_iter:
                                # Highlight the specific failure coordinates
                                bounds_info = (
                                    # f", **OUT-OF-BOUNDS FAILURE**: x_range=[{x_min_traj}, {x_max_traj}], "
                                    f", x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )
                            else:
                                bounds_info = (
                                    f"x_range=[{x_min_traj}, {x_max_traj}], "
                                    f"y_range=[{y_min_traj}, {y_max_traj}]"
                                )

                    if weights:
                        # Convert the list of weights into a JSON string to include in the prompt
                        # Rounding to 4 decimal places keeps it clean
                        rounded_weights = [round(w, 4) for w in weights]
                        failure_tag = " (FAILED)" if is_failed_iter else ""
                        iter_string = f" Examples {iter_num + n_warmup} " if iter_num < 1 else f" Iteration {iter_num} "
                        iter_string = "-" * 50 + iter_string + "-" * 50
                        feedback_text += (
                            f"{iter_string}\n"
                            f"weights={json.dumps(rounded_weights)}\n"
                            f"{bounds_info}\n"
                        )
                        if traj_in_prompt:
                            ee_traj_it_df = ee_traj_df[ee_traj_df["iter"] == iter_num].copy()
                            ee_traj_it_df.drop(columns=['iter', 'timestamp'], inplace=True)
                            ee_traj_it_df = ee_traj_it_df.iloc[::resample_rate, :].reset_index(drop=True)
                            ee_traj_it_df.set_index('step', inplace=True)
                            feedback_text += f"Resampled 2D Trajectory:\n{ee_traj_it_df.to_markdown()}\n"
                        if GRID_REWARD:
                            feedback_text += f"f(weights):\n{cells_df.to_markdown(index=True)}\n\n"
                        else:
                            feedback_text += f"f(weights)={current_f_weights}\n\n"
        except Exception as e:
            tb = traceback.extract_tb(e.__traceback__)
            if tb:
                last = tb[-1]  # where the exception was raised
                feedback_text += (
                    f"# ⚠️ Error processing executed weights history: {e}\n"
                    f"#    at {last.filename}:{last.lineno} in {last.name}\n"
                    f"#    code: {last.line}\n"
                )
            else:
                feedback_text += f"# ⚠️ Error processing executed weights history: {e}\n"
    # === 3. Render the Jinja2 template with all data ===
    try:
        tmpl = JINJA_ENV.get_template(template_name)
    except Exception as e:
        raise RuntimeError(
            f"Failed to load Jinja2 template '{template_name}' from '{TEMPLATE_DIR}': {e}"
        )

    prompt = tmpl.render(
        MAX_ITERS=MAX_ITERS,
        N_BFS=N_BFS,
        xmin=xmin,
        xmax=xmax,
        ymin=ymin,
        ymax=ymax,
        optimum=optimum,
        step_size=step_size,
        feedback_text=feedback_text,
        iter_idx=iter_idx,
        n_x_seg=n_x_seg,
        n_y_seg=n_y_seg,
        guidance_text=guidance_text,
        # If you later add more placeholders to the template, pass them here.
        # grid_list=grid_list,
        # total_balls=total_balls,
    )
    return prompt


import time
import random

_call_gemini_lock = threading.Lock()


def parse_ollama_weights(out_text):
    """
    Parse the Ollama LLM response to extract a 2xN_BFS weight matrix.
    Handles both pure JSON and messy text (code fences or extra commentary).
    """
    text = out_text.strip()
    

    # 🧹 Clean up possible code fences like ```json ... ```
    if text.startswith("```"):
        text = re.sub(r"^```[^\n]*\n|\n```$", "", text, flags=re.MULTILINE).strip()

    text = text.split("<weights>")[1].split("</weights>")[0].strip() if "<weights>" in text and "</weights>" in text else text
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
    global feedback_window, resample_rate, step_size, run_type, template_number
    global traj_in_prompt, GRID_REWARD, n_x_seg, n_y_seg, GUIDED
    global x_edges, y_edges, raw_cell_cols
    global template_name, save_results_file

    args = _parse_args()

    # Apply CLI config to globals used throughout the file
    run_type = args.run_type
    step_size = args.step_size
    feedback_window = args.feedback_window
    template_number = args.template_number
    traj_in_prompt = args.traj_in_prompt
    resample_rate = args.resample_rate
    GRID_REWARD = args.grid_reward
    n_x_seg = args.n_x_seg
    n_y_seg = args.n_y_seg
    GUIDED = args.guided

    # Recompute derived globals
    x_edges = np.linspace(-1, 1, n_x_seg + 1)
    y_edges = np.linspace(-0.6, 0.6, n_y_seg + 1)
    raw_cell_cols = [f"cell{i}" for i in range(n_x_seg * n_y_seg)]

    template_name, save_results_file = _compute_names(args)
    _init_paths(save_results_file)

    print(f"template_name: {template_name}")
    print(f"save_results_file: {save_results_file}")
    print(f"LOGDIR: {LOGDIR}")

    ensure_dirs()

    weight_history = []
    
    # Controller (one viewer, no reset later)
    controller = EnhancedDMPController()
    controller.num_x_segments = n_x_seg
    controller.num_y_segments = n_y_seg
    controller.grid_count = np.zeros((n_y_seg, n_x_seg), dtype=int)
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
                                                              rect_d0=0.05, rect_eta=25, obs_d0=0.1, obs_eta=25, max_force=220))
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

        # if total_balls == 0:
        #     print(f"iter {it}: Done, no balls left.")
        #     break

        # NEW: Load and analyze trajectory history
        dmp_trajectory_history = load_trajectory_history(DMP_TRAJECTORY_CSV, TRAJECTORY_HISTORY_WINDOW)
        trajectory_analysis = analyze_trajectory_performance(dmp_trajectory_history, bounds)
        # NEW: IK failure feedback (history + summary)
        # ik_error_history = load_ik_error_history(IK_ERROR_CSV, IK_ERROR_HISTORY_WINDOW)
        # ik_error_summary = summarize_ik_errors(ik_error_history)


        
        iter_log_data = load_iteration_log(ITER_LOG_CSV)
        dmp_traj_feedback_data = load_traj_feedback(DMP_TRAJECTORY_CSV)
        ee_traj_feedback_data = load_traj_feedback(EE_TRAJECTORY_CSV)
        # pdb.set_trace()

        # Ask LLM for NEW weights given cost, grid, prev weights, AND trajectory feedback
        hist_slice = weight_history[-HISTORY_WINDOW:] if HISTORY_WINDOW > 0 else weight_history

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
                traj_feedback_data=ee_traj_feedback_data,
                feedback_window=feedback_window
            )
            
            # save_dialog(it, prompt, '')
            # return
            try:
                # response = call_gemini(prompt)
                response = call_ollama(prompt, token_limit=100000)

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

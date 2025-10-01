#!/usr/bin/env python3


import os
import re
import csv
import json
import time
import uuid
import builtins
import subprocess
import numpy as np

# ====== EDIT THESE PATHS ======
CSV_MOVE_PATH   = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/logs/move.csv"
WEIGHTS_TXT     = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/logs/weight.txt"
BASE_DIR        = "/home/flash/Assign 1/yash/meshes (3)/Robot-cleaning-ur5/"
# ==============================

LOGDIR          = os.path.join(BASE_DIR, "logs")
WEIGHTS_CSV     = os.path.join(LOGDIR, "weights.csv")
ITER_LOG_CSV    = os.path.join(LOGDIR, "llm_iteration_log.csv")
DIALOG_DIR      = os.path.join(LOGDIR, "llm_dialog")

N_BFS           = 50
MAX_ITERS       = 50
IK_MAX_ITERS    = 60
DECI_BUILD      = 1           # keep every k-th DMP step when building joints (1 = all)
OLLAMA_MODEL    = "gpt-oss:120b"
OLLAMA_BIN      = "ollama"

# Defuse any accidental input() in imported code
builtins.input = lambda *a, **k: "7"

# Import your controller + helpers
from testiing_2 import (
    EnhancedDMPController, MOP_Z_HEIGHT,
    enhanced_ik_solver, get_joint_positions, set_joint_positions
)
from pydmps.dmp_rhythmic import DMPs_rhythmic


# ----------------- utils -----------------
def _ensure_dirs():
    os.makedirs(LOGDIR, exist_ok=True)
    os.makedirs(DIALOG_DIR, exist_ok=True)

def _parse_weights_text(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", txt)
    if not nums:
        raise ValueError(f"No numeric weights found in {path}")
    return np.array([float(x) for x in nums], dtype=float)

def _row_to_2x50(arr):
    a = np.asarray(arr, dtype=float).flatten()
    if a.size != 2 * N_BFS:
        raise ValueError(f"Expected {2*N_BFS} weights, got {a.size}")
    return a.reshape(2, N_BFS)

def _write_weights_csv(path, w2):
    row = w2.reshape(-1)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerow(list(row))

def _read_weights_csv(path):
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", txt)
    if not nums:
        raise ValueError(f"No numbers in {path}")
    return _row_to_2x50([float(x) for x in nums])

def _read_move_csv(path):
    # Try named columns
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        if data.dtype.names and {'x','y'}.issubset(data.dtype.names):
            xy = np.column_stack([data['x'], data['y']]).astype(float)
            if xy.ndim == 2 and xy.shape[1] == 2:
                return xy
    except Exception:
        pass
    # Fallback two columns
    xy = np.loadtxt(path, delimiter=",", dtype=float)
    if xy.ndim != 2 or xy.shape[1] < 2:
        raise ValueError(f"{path} must have at least two columns: x,y")
    return xy[:, :2].astype(float)

def _log_iteration(iter_idx, grid_mat, total_balls, traj_len, out_csv):
    flat = list(map(int, grid_mat.flatten()))
    file_exists = os.path.exists(out_csv)
    with open(out_csv, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter","timestamp","traj_waypoints","total_balls"] +
                       [f"cell_{i}" for i in range(len(flat))])
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"),
                    traj_len, total_balls] + flat)

def _ollama_prompt(prev_w_flat, grid_mat, total_balls, iter_idx):
    # Keep prompt compact but strict on output format.
    grid_list = grid_mat.tolist()
    return f"""You are a good global RL policy optimizer, helping me find an optimal policy in the following environment:

    1. Environment:
    - A 2-DoF end-effector sweeps a tabletop using a controller that parameterizes motion with {N_BFS} basis functions per axis (x,y), for a total of {2 * N_BFS} weights flattened as [x then y].
    - One rollout executes a single rhythmic sweep. After the sweep, the cost is the number of balls remaining on the table.
    - Spatial feedback is provided as a grid of ball counts (rows = y high→low, cols = x left→right): {json.dumps(grid_list)}

    2. Regarding the parameters (policy weights):
    - Current iteration: {iter_idx}
    - Current cost (total balls on table): {total_balls}
    - Current weights (length {2 * N_BFS}, flattened [x then y]):
    {json.dumps([float(x) for x in prev_w_flat])}

    3. Here’s how we’ll interact:
    - You will return a NEW weight vector (length {2 * N_BFS}) that preserves the overall trajectory style of the very first (initial) policy while improving ball removal.
    - Choose a random K ∈ [5, 10] distinct indices across the full flattened vector and modify ONLY those K components; leave all other components unchanged.
    - Use the grid feedback to bias subtle adjustments that push balls outward/off the table (e.g., nudge weights to sweep into higher-count regions), but keep the motion similar to the initial trajectory.

    4. Remember:
    - Keep each change small and stable; do NOT radically alter the trajectory. Prefer conservative deltas that maintain the original motion pattern.
    - Objective: reduce the total balls remaining (aim toward 0).
    - Output STRICTLY one JSON object on a single line and nothing else:
      {{"weights":[<exactly {2 * N_BFS} floats here>]}}
    - Do not add comments, prose, or extra keys."""
#     return f"""You are optimizing rhythmic DMP weights for a 2-DoF end-effector sweeping task.
# We use pydmps.DMPs_rhythmic with n_bfs={N_BFS}. The weight matrix shape is (2,{N_BFS}) for x,y.
# Current iteration: {iter_idx}
# Recent cost (total balls remaining): {total_balls}
# Grid counts (rows = y-bins high->low, columns = x-bins left->right): {json.dumps(grid_list)}
#
# Current weights (flattened [x then y], {2*N_BFS} floats):
# {json.dumps([float(x) for x in prev_w_flat])}
#
# Goal: Propose a NEW set of {2*N_BFS} weights that reduces the cost. Keep changes modest and stable.
# IMPORTANT:
# - Output ONLY this JSON object on a single line:
#   {{"weights": [<exactly {2*N_BFS} floats here>]}}
# Do not include explanations or extra text.
# """

def _call_ollama(prompt):
    # Use stdin to pass the prompt; capture stdout.
    try:
        res = subprocess.run(
            [OLLAMA_BIN, "run", OLLAMA_MODEL],
            input=prompt,
            text=True,
            capture_output=True,
            check=True,
        )
        return res.stdout.strip()
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Ollama call failed: {e.stderr or e.stdout}")

def _parse_ollama_weights(out_text):
    # Try strict JSON first
    try:
        obj = json.loads(out_text)
        cand = obj.get("weights", None)
        if cand is not None and isinstance(cand, list):
            return _row_to_2x50(cand)
    except Exception:
        pass
    # Fallback: extract all floats and reshape
    nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", out_text)
    if len(nums) >= 2*N_BFS:
        return _row_to_2x50([float(x) for x in nums[:2*N_BFS]])
    raise ValueError("Could not parse weights from LLM output")

def _save_dialog(iter_idx, prompt, response):
    pid = f"iter_{iter_idx:03d}_{uuid.uuid4().hex[:8]}"
    with open(os.path.join(DIALOG_DIR, pid + "_prompt.txt"), "w", encoding="utf-8") as f:
        f.write(prompt)
    with open(os.path.join(DIALOG_DIR, pid + "_response.txt"), "w", encoding="utf-8") as f:
        f.write(response)


# --------------- main loop ---------------
def main():
    _ensure_dirs()

    # Bootstrap weights.csv from weights.txt if missing
    if not os.path.exists(WEIGHTS_CSV):
        if not os.path.exists(WEIGHTS_TXT):
            raise FileNotFoundError(f"Missing {WEIGHTS_CSV} and {WEIGHTS_TXT}")
        flat0 = _parse_weights_text(WEIGHTS_TXT)
        w0 = _row_to_2x50(flat0)
        _write_weights_csv(WEIGHTS_CSV, w0)
        print(f"Initialized {WEIGHTS_CSV} from {WEIGHTS_TXT} -> shape {w0.shape}")

    # Controller (one viewer, no reset later)
    controller = EnhancedDMPController()

    # Prime DMP ONCE from move.csv (y0/etc)
    if not os.path.exists(CSV_MOVE_PATH):
        raise FileNotFoundError(f"Demo path not found: {CSV_MOVE_PATH}")
    demo_xy = _read_move_csv(CSV_MOVE_PATH)
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)
    dmp.imitate_path(y_des=demo_xy.T)

    # Iterations
    for it in range(1, MAX_ITERS + 1):
        controller.hard_reset_from_home()
        # Read current weights
        w2 = _read_weights_csv(WEIGHTS_CSV)
        w_flat = w2.reshape(-1)

        # Apply weights
        dmp.w = w2.copy()
        dmp.reset_state()

        # Convert one full cycle to joint trajectory
        model, data = controller.model, controller.data
        site_id     = controller.site_id
        joint_names = controller.joint_names

        start_joints = get_joint_positions(model, data, joint_names)
        joint_traj = []
        steps = int(dmp.timesteps)
        keep_every = max(1, int(DECI_BUILD))


        for i in range(steps):
            y, _, _ = dmp.step()
            target_3d = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
            ok, _ = enhanced_ik_solver(
                model, data, site_id, target_3d, joint_names,
                max_iters_per_wp=IK_MAX_ITERS, print_every=1_000_000
            )
            if not ok:
                continue
            if (i % keep_every) == 0:
                joint_traj.append(get_joint_positions(model, data, joint_names).copy())

        if not joint_traj:
            print(f"[iter {it}] ⚠️ No joints generated; skipping execution.")
        else:
            # restore start joints so playback is clean
            set_joint_positions(model, data, joint_names, start_joints)
            # Execute whole motion once (same viewer, no reset)
            print(f"[iter {it}] Executing {len(joint_traj)} joint waypoints...")
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt)

        # Compute cost via your grid counter
        controller.count_balls_in_grid()
        grid = controller.grid_count       # (ny, nx) after their transpose
        total_balls = int(np.sum(grid))
        _log_iteration(it, grid, total_balls, len(joint_traj), ITER_LOG_CSV)
        print(f"[iter {it}] Cost (total balls): {total_balls} | per-cell: {grid.tolist()}")

        if total_balls == 0:
            print(f"[iter {it}] ✅ Done (no balls left).")
            break

        # Ask Ollama for NEW weights (given cost + grid + prev weights)
        prompt = _ollama_prompt(w_flat, grid, total_balls, it)
        try:
            response = _call_ollama(prompt)
        except Exception as e:
            print(f"[iter {it}] Ollama error: {e}. Reusing previous weights.")
            time.sleep(1.0)
            continue

        _save_dialog(it, prompt, response)

        try:
            w_next = _parse_ollama_weights(response)  # (2,50)
        except Exception as e:
            print(f"[iter {it}] Failed to parse LLM weights: {e}. Reusing previous weights.")
            time.sleep(1.0)
            continue

        # Write new weights for transparency & possible external inspection
        _write_weights_csv(WEIGHTS_CSV, w_next)
        print(f"[iter {it}] Updated {WEIGHTS_CSV} with new weights from LLM.")

    print("Loop finished. Close the viewer to exit.")

if __name__ == "__main__":
    main()

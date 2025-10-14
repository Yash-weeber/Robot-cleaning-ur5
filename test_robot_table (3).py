# #!/usr/bin/env python3
# """
# Replay DMP weights -> generate & execute trajectories -> log ALL waypoints
# - Viewer: persistent EnhancedDMPController from testiing_2.py (so you can see runs)
# - Looks for: logs/llm_dialog/iter_###_*_response.txt, where ### = 001..050 and * is random
# - DMP: rhythmic, n_bfs = 25 (2 DoF: x,y; Z fixed by controller)
# - Output CSV (append): logs/trajectory_way.csv
#     Columns:
#       iter_idx, step_idx, timestamp, x, y, z, q0, q1, q2, q3, q4, q5
# """

# import os
# import re
# import csv
# import glob
# import json
# import time
# import numpy as np

# # ====== EDIT THIS PATH TO YOUR PROJECT ROOT ======
# BASE_DIR ="F:/Robotics Automation system AI/SEM 3/APPLIED PROJECT/NEW XMLS FILESSSSSSSSSSSSSSSSSSSSSSSSSSSS/Robot-cleaning-ur5-testing_IK/"

# # =================================================

# LOGS_ROOT = os.path.join(BASE_DIR, "logs")
# DIALOG_DIR = os.path.join(LOGS_ROOT, "llm_dialog")
# TRAJ_CSV  = os.path.join(LOGS_ROOT, "trajectory_way.csv")  # <-- fixed to logs/, not llm_dialog/
# MOVE_CSV  = os.path.join(LOGS_ROOT, "move.csv")            # optional, if you have a demo path

# # DMP / IK config
# N_BFS        = 25                       # 25 per axis → 50 total weights
# IK_MAX_ITERS = 60

# # Import your viewer/controller + helpers (kept exactly as in your project)
# from testiing_2 import (  # type: ignore
#     EnhancedDMPController, MOP_Z_HEIGHT,
#     enhanced_ik_solver, get_joint_positions, set_joint_positions
# )
# from pydmps.dmp_rhythmic import DMPs_rhythmic


# # ---------------- helpers ----------------
# def _ensure_dirs():
#     os.makedirs(LOGS_ROOT, exist_ok=True)
#     os.makedirs(DIALOG_DIR, exist_ok=True)

# def _iter_files_001_to_050(resp_dir: str):
#     """
#     Return list of (iter_idx:int, filepath:str) for exact iterations 001..050.
#     Primary pattern: iter_###_*_response.txt (zero-padded).
#     Fallback pattern if none for that iter: iter_#_*_response.txt (no padding).
#     If multiple files match, pick the newest (by mtime).
#     """
#     out = []
#     for i in range(1, 51):
#         patt_main = os.path.join(resp_dir, f"iter_{i:03d}_*_response.txt")
#         matches = glob.glob(patt_main)
#         if not matches:
#             patt_fallback = os.path.join(resp_dir, f"iter_{i}_*_response.txt")
#             matches = glob.glob(patt_fallback)

#         if matches:
#             matches.sort(key=lambda p: os.path.getmtime(p))
#             out.append((i, matches[-1]))
#     return out

# def _debug_list_candidates(resp_dir: str):
#     """Prints what candidate files are visible to help diagnose glob issues."""
#     cands = sorted(glob.glob(os.path.join(resp_dir, "iter_*_response.txt")))
#     print(f"[DEBUG] RESP_DIR: {resp_dir}")
#     print(f"[DEBUG] Found {len(cands)} candidate file(s):")
#     for p in cands:
#         print("   -", os.path.basename(p))

# def _parse_weights_from_file(path):
#     """
#     Parse 50 floats from a response file.
#     Primary: JSON array after "weights": [...]
#     Fallback: all floats in the file.
#     If > 50 numbers found, take the first 50; if < 50, raise.
#     Returns ndarray shaped (2, N_BFS).
#     """
#     with open(path, "r", encoding="utf-8") as f:
#         txt = f.read()

#     # try JSON array after "weights":
#     m = re.search(r'"weights"\s*:\s*\[(.*?)\]', txt, flags=re.S)
#     vals = []
#     if m:
#         arr_txt = "[" + m.group(1) + "]"
#         try:
#             vals = [float(x) for x in json.loads(arr_txt)]
#         except Exception:
#             vals = []

#     if not vals:
#         # fallback: collect all floats (ints/floats, with optional signs)
#         nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", txt)
#         vals = [float(x) for x in nums]

#     if len(vals) < 50:
#         raise ValueError(f"{os.path.basename(path)} has only {len(vals)} numbers; need at least 50.")
#     if len(vals) > 50:
#         vals = vals[:50]

#     w = np.asarray(vals, dtype=float).reshape(2, N_BFS)  # (2,25)
#     return w

# def _append_traj_rows(csv_path, rows):
#     """
#     rows: iterable of [iter_idx, step_idx, timestamp, x,y,z, q0..q5]
#     Creates header on first write and appends.
#     """
#     new_file = not os.path.exists(csv_path)
#     with open(csv_path, "a", newline="") as f:
#         w = csv.writer(f)
#         if new_file:
#             w.writerow([
#                 "iter_idx","step_idx","timestamp",
#                 "x","y","z","q0","q1","q2","q3","q4","q5"
#             ])
#         w.writerows(rows)

# def _read_move_csv_optional(path):
#     """Return (2, T) demo path or None if missing/invalid."""
#     if not os.path.exists(path):
#         return None
#     try:
#         # Try named columns x,y
#         data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
#         if data.dtype.names and {'x','y'}.issubset(data.dtype.names):
#             xy = np.column_stack([data['x'], data['y']]).astype(float)
#         else:
#             xy = np.loadtxt(path, delimiter=",", dtype=float)
#             if xy.ndim != 2 or xy.shape[1] < 2:
#                 return None
#             xy = xy[:, :2]
#         return xy.T  # shape (2, T)
#     except Exception:
#         return None


# # ---------------- main ----------------
# def main():
#     _ensure_dirs()

#     # Keep a single viewer running the whole time
#     controller = EnhancedDMPController()

#     # DMP
#     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)

#     # Optional: prime DMP canonical system using a demo path if you have one
#     demo = _read_move_csv_optional(MOVE_CSV)
#     if demo is not None and demo.shape[0] == 2 and demo.shape[1] >= 2:
#         dmp.imitate_path(y_des=demo)

#     files = _iter_files_001_to_050(DIALOG_DIR)
#     if not files:
#         print(f"No matching files like iter_###_*_response.txt found in {DIALOG_DIR}")
#         _debug_list_candidates(DIALOG_DIR)
#         return

#     print(f"Using {len(files)} file(s) from {DIALOG_DIR}:")
#     for i, p in files:
#         print(f"  iter {i:03d} -> {os.path.basename(p)}")

#     for iter_idx, path in files:
#         print(f"\n=== Iteration {iter_idx:03d} → {os.path.basename(path)} ===")
#         # 1) Parse weights (2x25)
#         try:
#             w2 = _parse_weights_from_file(path)
#         except Exception as e:
#             print(f"  ! Failed parsing weights: {e}")
#             continue

#         # 2) Program DMP
#         dmp.w = w2.copy()
#         dmp.reset_state()

#         # 3) Generate task-space waypoints (one rhythmic cycle using dmp.timesteps)
#         task_traj = []
#         for _ in range(dmp.timesteps):
#             y, _, _ = dmp.step()          # y: (2,) -> (x, y)
#             target = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
#             task_traj.append(target)

#         # 4) IK to joint space, collect successful points
#         joint_traj = []
#         for target in task_traj:
#             ok, _err = enhanced_ik_solver(
#                 controller.model, controller.data, controller.site_id,
#                 target, controller.joint_names,
#                 max_iters_per_wp=IK_MAX_ITERS, print_every=1000
#             )
#             if ok:
#                 q = get_joint_positions(controller.model, controller.data, controller.joint_names)
#                 joint_traj.append(q.copy())

#         print(f"  DMP waypoints: {len(task_traj)} | IK waypoints: {len(joint_traj)}")

#         # 5) Execute (viewer is constant)
#         if joint_traj:
#             # reset to first waypoint for a clean start
#             set_joint_positions(controller.model, controller.data, controller.joint_names, joint_traj[0])
#             controller.execute_joint_trajectory(joint_traj, dt=controller.dt)

#         # 6) Append to trajectory_way.csv (full history across all iterations)
#         rows = []
#         tstamp = time.strftime("%Y-%m-%d %H:%M:%S")
#         n = min(len(task_traj), len(joint_traj))
#         for step_idx in range(n):
#             x, y, z = task_traj[step_idx]
#             q = joint_traj[step_idx]
#             rows.append([
#                 iter_idx, step_idx, tstamp,
#                 float(x), float(y), float(z),
#                 float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4]), float(q[5])
#             ])

#         if rows:
#             _append_traj_rows(TRAJ_CSV, rows)
#             print(f"  Appended {len(rows)} rows to {TRAJ_CSV}")

#     print("\nDone. Close the viewer when you're finished.")

# if __name__ == "__main__":
#     main()





#!/usr/bin/env python3
"""
Replay DMP weights -> generate & execute trajectories -> log ALL waypoints
- Viewer: persistent EnhancedDMPController from testiing_2.py (so you can see runs)
- Looks for: logs/llm_dialog/iter_###_*_response.txt, where ### = 001..050 and * is random
- DMP: rhythmic, n_bfs = 25 (2 DoF: x,y; Z fixed by controller)
- Output CSV (append): logs/trajectory_way.csv
    Columns:
      iter_idx, step_idx, timestamp, x, y, z, q0, q1, q2, q3, q4, q5

New:
- Uses controller.hard_reset_from_home() to hard-reset the world before each iteration.
"""

import os
import re
import csv
import glob
import json
import time
import numpy as np
import builtins

# ====== EDIT THIS PATH TO YOUR PROJECT ROOT ======
BASE_DIR ="F:/Robotics Automation system AI/SEM 3/APPLIED PROJECT/NEW XMLS FILESSSSSSSSSSSSSSSSSSSSSSSSSSSS/Robot-cleaning-ur5-testing_IK/"
# =================================================

LOGS_ROOT  = os.path.join(BASE_DIR, "logs")
DIALOG_DIR = os.path.join(LOGS_ROOT, "llm_dialog")
TRAJ_CSV   = os.path.join(LOGS_ROOT, "trajectory_way.csv")
MOVE_CSV   = os.path.join(LOGS_ROOT, "move.csv")  # optional

# DMP / IK config
N_BFS        = 25                       # 25 per axis → 50 total weights
IK_MAX_ITERS = 60

# Auto-select menu option 7 (Quit) inside testiing_2's menu loop, so the viewer stays but menu doesn't block
builtins.input = lambda *a, **k: "7"

# Import your viewer/controller + helpers (from testiing_2.py)
from testiing_2 import (  # type: ignore
    EnhancedDMPController, MOP_Z_HEIGHT,
    enhanced_ik_solver, get_joint_positions, set_joint_positions
)
from pydmps.dmp_rhythmic import DMPs_rhythmic

# ---------------- helpers ----------------
def _ensure_dirs():
    os.makedirs(LOGS_ROOT, exist_ok=True)
    os.makedirs(DIALOG_DIR, exist_ok=True)

def _iter_files_001_to_050(resp_dir: str):
    """
    Return list of (iter_idx:int, filepath:str) for exact iterations 001..050.
    Primary pattern: iter_###_*_response.txt (zero-padded).
    Fallback pattern if none for that iter: iter_#_*_response.txt (no padding).
    If multiple files match, pick the newest (by mtime).
    """
    out = []
    for i in range(1, 51):
        patt_main = os.path.join(resp_dir, f"iter_{i:03d}_*_response.txt")
        matches = glob.glob(patt_main)
        if not matches:
            patt_fallback = os.path.join(resp_dir, f"iter_{i}_*_response.txt")
            matches = glob.glob(patt_fallback)

        if matches:
            matches.sort(key=lambda p: os.path.getmtime(p))
            out.append((i, matches[-1]))
    return out

def _debug_list_candidates(resp_dir: str):
    """Prints what candidate files are visible to help diagnose glob issues."""
    cands = sorted(glob.glob(os.path.join(resp_dir, "iter_*_response.txt")))
    print(f"[DEBUG] RESP_DIR: {resp_dir}")
    print(f"[DEBUG] Found {len(cands)} candidate file(s):")
    for p in cands:
        print("   -", os.path.basename(p))

def _parse_weights_from_file(path):
    """
    Parse 50 floats from a response file.
    Primary: JSON array after "weights": [...]
    Fallback: all floats in the file.
    If > 50 numbers found, take the first 50; if < 50, raise.
    Returns ndarray shaped (2, N_BFS).
    """
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()

    # try JSON array after "weights":
    m = re.search(r'"weights"\s*:\s*\[(.*?)\]', txt, flags=re.S)
    vals = []
    if m:
        arr_txt = "[" + m.group(1) + "]"
        try:
            vals = [float(x) for x in json.loads(arr_txt)]
        except Exception:
            vals = []

    if not vals:
        # fallback: collect all floats (ints/floats, with optional signs)
        nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", txt)
        vals = [float(x) for x in nums]

    if len(vals) < 50:
        raise ValueError(f"{os.path.basename(path)} has only {len(vals)} numbers; need at least 50.")
    if len(vals) > 50:
        vals = vals[:50]

    w = np.asarray(vals, dtype=float).reshape(2, N_BFS)  # (2,25)
    return w

def _append_traj_rows(csv_path, rows):
    """
    rows: iterable of [iter_idx, step_idx, timestamp, x,y,z, q0..q5]
    Creates header on first write and appends.
    """
    new_file = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow([
                "iter_idx","step_idx","timestamp",
                "x","y","z","q0","q1","q2","q3","q4","q5"
            ])
        w.writerows(rows)

def _read_move_csv_optional(path):
    """Return (2, T) demo path or None if missing/invalid."""
    if not os.path.exists(path):
        return None
    try:
        # Try named columns x,y
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        if data.dtype.names and {'x','y'}.issubset(data.dtype.names):
            xy = np.column_stack([data['x'], data['y']]).astype(float)
        else:
            xy = np.loadtxt(path, delimiter=",", dtype=float)
            if xy.ndim != 2 or xy.shape[1] < 2:
                return None
            xy = xy[:, :2]
        return xy.T  # shape (2, T)
    except Exception:
        return None

# ---------------- main ----------------
def main():
    _ensure_dirs()

    # Keep a single viewer running the whole time
    controller = EnhancedDMPController()

    # DMP
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)

    # Optional: prime DMP canonical system using a demo path if you have one
    demo = _read_move_csv_optional(MOVE_CSV)
    if demo is not None and demo.shape[0] == 2 and demo.shape[1] >= 2:
        dmp.imitate_path(y_des=demo)

    files = _iter_files_001_to_050(DIALOG_DIR)
    if not files:
        print(f"No matching files like iter_###_*_response.txt found in {DIALOG_DIR}")
        _debug_list_candidates(DIALOG_DIR)
        return

    print(f"Using {len(files)} file(s) from {DIALOG_DIR}:")
    for i, p in files:
        print(f"  iter {i:03d} -> {os.path.basename(p)}")

    for iter_idx, path in files:
        print(f"\n=== Iteration {iter_idx:03d} → {os.path.basename(path)} ===")

        # A) HARD RESET ENVIRONMENT so every iteration starts clean
        #    (provided by EnhancedDMPController in testiing_2.py)
        try:
            controller.hard_reset_from_home(redraw=True)
        except Exception as e:
            print(f"[reset] hard_reset_from_home() raised: {e}")

        # B) Parse weights (2x25)
        try:
            w2 = _parse_weights_from_file(path)
        except Exception as e:
            print(f"  ! Failed parsing weights: {e}")
            continue

        # C) Program DMP
        dmp.w = w2.copy()
        dmp.reset_state()

        # D) Generate task-space waypoints (one rhythmic cycle using dmp.timesteps)
        task_traj = []
        for _ in range(dmp.timesteps):
            y, _, _ = dmp.step()          # y: (2,) -> (x, y)
            target = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
            task_traj.append(target)

        # E) IK to joint space, collect successful points
        joint_traj = []
        for target in task_traj:
            ok, _err = enhanced_ik_solver(
                controller.model, controller.data, controller.site_id,
                target, controller.joint_names,
                max_iters_per_wp=IK_MAX_ITERS, print_every=1000
            )
            if ok:
                q = get_joint_positions(controller.model, controller.data, controller.joint_names)
                joint_traj.append(q.copy())

        print(f"  DMP waypoints: {len(task_traj)} | IK waypoints: {len(joint_traj)}")

        # F) Execute (viewer is constant)
        if joint_traj:
            # reset to first waypoint for a clean start
            set_joint_positions(controller.model, controller.data, controller.joint_names, joint_traj[0])
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt)

        # G) Append to trajectory_way.csv (full history across all iterations)
        rows = []
        tstamp = time.strftime("%Y-%m-%d %H:%M:%S")
        n = min(len(task_traj), len(joint_traj))
        for step_idx in range(n):
            x, y, z = task_traj[step_idx]
            q = joint_traj[step_idx]
            rows.append([
                iter_idx, step_idx, tstamp,
                float(x), float(y), float(z),
                float(q[0]), float(q[1]), float(q[2]), float(q[3]), float(q[4]), float(q[5])
            ])

        if rows:
            _append_traj_rows(TRAJ_CSV, rows)
            print(f"  Appended {len(rows)} rows to {TRAJ_CSV}")

    print("\nDone. Close the viewer when you're finished.")

if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# """
# plot_grid_counts.py
#
# Reads the LLM loop log (e.g., ./logs/llm_iteration_log.csv), computes per-iteration
# changes in grid (cell) ball counts, saves a deltas CSV, and produces PNG plots.
#
# - Input CSV is expected to have columns like:
#   iter, timestamp, traj_waypoints, total_balls, cell_0, cell_1, ..., cell_M
#
# - Outputs:
#   1) <outdir>/grid_counts_deltas.csv
#   2) <outdir>/grid_counts_heatmap.png
#   3) <outdir>/grid_counts_delta_heatmap.png
#   4) <outdir>/total_balls_over_iterations.png
#   5) <outdir>/grid_counts_stacked.png
#
# Usage:
#   python3 plot_grid_counts.py \
#     --log ./logs/llm_iteration_log.csv \
#     --outdir ./logs
# """
#
# import os
# import re
# import argparse
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
#
#
# def _natural_sort_cell_cols(cols):
#     # sort cell_0, cell_1, ..., cell_10 numerically by suffix
#     def key(c):
#         m = re.search(r"cell[_\- ]?(\d+)$", c)
#         return int(m.group(1)) if m else float('inf')
#     return sorted(cols, key=key)
#
#
# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("--log", default="./logs/llm_iteration_log.csv", help="Path to llm_iteration_log.csv")
#     ap.add_argument("--outdir", default="./logs/", help="Directory to save CSV/PNGs")
#     args = ap.parse_args()
#
#     os.makedirs(args.outdir, exist_ok=True)
#
#     # Load and sanitize
#     df = pd.read_csv(args.log)
#     if "iter" not in df.columns:
#         raise ValueError("Missing 'iter' column in log CSV.")
#
#     # Ensure iter is numeric and unique per row (keep last if duplicates)
#     df["iter"] = pd.to_numeric(df["iter"], errors="coerce")
#     df = df.dropna(subset=["iter"]).astype({"iter": int})
#     df = df.sort_values("iter")
#     df = df.groupby("iter", as_index=False).last()  # de-dup by taking last record per iter
#
#     # Find cell columns
#     cell_cols = [c for c in df.columns if str(c).startswith("cell_")]
#     if not cell_cols:
#         raise ValueError("No 'cell_*' columns found in log CSV.")
#     cell_cols = _natural_sort_cell_cols(cell_cols)
#
#     # Get total balls or compute if missing
#     if "total_balls" in df.columns:
#         total = pd.to_numeric(df["total_balls"], errors="coerce").fillna(0).astype(int).to_numpy()
#     else:
#         total = df[cell_cols].to_numpy().sum(axis=1)
#
#     iters = df["iter"].to_numpy()
#     C = df[cell_cols].to_numpy()  # shape: (T, M) where T=iterations, M=cells
#
#     # Save per-iteration deltas (current - previous)
#     if C.shape[0] >= 2:
#         dC = np.diff(C, axis=0)          # shape: (T-1, M)
#         iters_delta = iters[1:]
#         deltas_df = pd.DataFrame(dC, columns=cell_cols)
#         deltas_df.insert(0, "iter", iters_delta)
#     else:
#         dC = np.zeros_like(C)
#         iters_delta = iters
#         deltas_df = pd.DataFrame(C, columns=cell_cols)
#         deltas_df.insert(0, "iter", iters_delta)
#
#     deltas_csv = os.path.join(args.outdir, "grid_counts_deltas.csv")
#     deltas_df.to_csv(deltas_csv, index=False)
#     print(f"✅ Saved deltas CSV: {deltas_csv}")
#
#     # ----------------- Plot 1: counts heatmap -----------------
#     # Display as cells (rows) x iterations (cols) for readability
#     plt.figure(figsize=(max(8, C.shape[0] * 0.2), max(4, len(cell_cols) * 0.25)))
#     plt.imshow(C.T, aspect="auto", interpolation="nearest")
#     plt.colorbar(label="balls in cell")
#     plt.xlabel("iteration")
#     plt.ylabel("cell index")
#     # X ticks (iterations)
#     xticks = np.arange(C.shape[0])
#     # Show fewer x tick labels if many iterations
#     step = max(1, int(np.ceil(C.shape[0] / 10)))
#     plt.xticks(ticks=xticks[::step], labels=iters[::step])
#     # Y ticks (cells)
#     yticks = np.arange(len(cell_cols))
#     plt.yticks(ticks=yticks, labels=[c.split("_")[-1] for c in cell_cols])
#     plt.title("Grid Cell Counts per Iteration (Heatmap)")
#     plt.tight_layout()
#     heatmap_path = os.path.join(args.outdir, "grid_counts_heatmap.png")
#     plt.savefig(heatmap_path, dpi=180)
#     plt.close()
#     print(f"✅ Saved heatmap: {heatmap_path}")
#
#     # ----------------- Plot 2: delta heatmap -----------------
#     plt.figure(figsize=(max(8, dC.shape[0] * 0.2), max(4, len(cell_cols) * 0.25)))
#     plt.imshow(dC.T, aspect="auto", interpolation="nearest")
#     plt.colorbar(label="Δ balls (current - previous)")
#     plt.xlabel("iteration (from 2nd row)")
#     plt.ylabel("cell index")
#     xticks = np.arange(dC.shape[0])
#     step = max(1, int(np.ceil(dC.shape[0] / 10)))
#     plt.xticks(ticks=xticks[::step], labels=iters_delta[::step])
#     yticks = np.arange(len(cell_cols))
#     plt.yticks(ticks=yticks, labels=[c.split("_")[-1] for c in cell_cols])
#     plt.title("Change in Grid Cell Counts per Iteration (Heatmap)")
#     plt.tight_layout()
#     delta_heatmap_path = os.path.join(args.outdir, "grid_counts_delta_heatmap.png")
#     plt.savefig(delta_heatmap_path, dpi=180)
#     plt.close()
#     print(f"✅ Saved delta heatmap: {delta_heatmap_path}")
#
#     # ----------------- Plot 3: total balls vs iteration -----------------
#     x_iter = iters  # numpy already
#     y_total = total
#     plt.figure(figsize=(10, 4))
#     plt.plot(x_iter, y_total, marker="o", linewidth=1)
#     plt.xlabel("iteration")
#     plt.ylabel("total balls")
#     plt.title("Total Balls Remaining per Iteration")
#     plt.grid(True, alpha=0.3)
#     plt.tight_layout()
#     total_plot_path = os.path.join(args.outdir, "total_balls_over_iterations.png")
#     plt.savefig(total_plot_path, dpi=180)
#     plt.close()
#     print(f"✅ Saved total balls plot: {total_plot_path}")
#
#     # ----------------- Plot 4: stacked area per-cell (optional but useful) -----------------
#     # Use NumPy arrays explicitly to avoid pandas indexing quirks.
#     plt.figure(figsize=(12, 6))
#     # stackplot expects lists of series; transpose C to (M, T)
#     series = [C[:, j] for j in range(C.shape[1])]
#     plt.stackplot(x_iter, series, labels=[c.split("_")[-1] for c in cell_cols], baseline='zero')
#     plt.xlabel("iteration")
#     plt.ylabel("balls per cell")
#     plt.title("Grid Cell Counts (Stacked Area)")
#     # Show only a few legends if many cells
#     if len(cell_cols) <= 12:
#         plt.legend(title="cell", loc="upper right", ncol=2, fontsize=8)
#     plt.tight_layout()
#     stacked_path = os.path.join(args.outdir, "grid_counts_stacked.png")
#     plt.savefig(stacked_path, dpi=180)
#     plt.close()
#     print(f"✅ Saved stacked area plot: {stacked_path}")
#
#
# if __name__ == "__main__":
#     main()
#
#
# #
# # #!/usr/bin/env python3
# # """
# # plot_cells_lines.py
# #
# # Make a *line plot like your L2 screenshot*:
# #   x-axis = iteration
# #   y-axis = balls in cell
# #   one line per grid cell.
# #
# # Usage examples:
# #   python3 plot_cells_lines.py --log ./logs/llm_iteration_log.csv --out ./logs/grid_cells_lines.png
# #   python3 plot_cells_lines.py --log ./logs/llm_iteration_log.csv --cells 0,1,2 --out ./logs/cells_0_1_2.png
# #   python3 plot_cells_lines.py --log ./logs/llm_iteration_log.csv --topk 6 --smooth 1 --out ./logs/cells_top6.png
# # """
# #
# # import os
# # import re
# # import argparse
# # import numpy as np
# # import pandas as pd
# # import matplotlib.pyplot as plt
# #
# #
# # def _natural_sort_cell_cols(cols):
# #     def key(c):
# #         m = re.search(r"cell[_\- ]?(\d+)$", c)
# #         return int(m.group(1)) if m else float("inf")
# #     return sorted(cols, key=key)
# #
# #
# # def _moving_average(x, k):
# #     if k <= 1:
# #         return x
# #     k = int(k)
# #     w = np.ones(k) / k
# #     # pad at ends to keep length
# #     pad = k // 2
# #     xpad = np.pad(x, (pad, k - 1 - pad), mode="edge")
# #     return np.convolve(xpad, w, mode="valid")
# #
# #
# # def main():
# #     ap = argparse.ArgumentParser()
# #     ap.add_argument("--log", default="./logs/llm_iteration_log.csv", help="Path to llm_iteration_log.csv")
# #     ap.add_argument("--out", default="./logs/grid_cells_lines.png", help="Output PNG")
# #     ap.add_argument("--cells", default=None,
# #                     help="Comma-separated cell indices to plot (e.g., '0,3,5'). Overrides --topk if set.")
# #     ap.add_argument("--topk", type=int, default=None,
# #                     help="If set, plot only the top-K most populated cells (by total over all iterations).")
# #     ap.add_argument("--smooth", type=int, default=1,
# #                     help="Moving-average window (>=1). 1 = no smoothing.")
# #     ap.add_argument("--figw", type=float, default=14.0)
# #     ap.add_argument("--figh", type=float, default=5.0)
# #     args = ap.parse_args()
# #
# #     os.makedirs(os.path.dirname(args.out), exist_ok=True)
# #
# #     # Load & sanitize
# #     df = pd.read_csv(args.log)
# #     if "iter" not in df.columns:
# #         raise ValueError("Missing 'iter' column in log CSV.")
# #     df["iter"] = pd.to_numeric(df["iter"], errors="coerce")
# #     df = df.dropna(subset=["iter"]).astype({"iter": int}).sort_values("iter")
# #     df = df.groupby("iter", as_index=False).last()   # if duplicates per iter, keep last
# #
# #     # Cell columns
# #     cell_cols = [c for c in df.columns if str(c).startswith("cell_")]
# #     if not cell_cols:
# #         raise ValueError("No 'cell_*' columns found in the log.")
# #     cell_cols = _natural_sort_cell_cols(cell_cols)
# #
# #     # Choose which cells to plot
# #     if args.cells:
# #         sel_ids = [int(x.strip()) for x in args.cells.split(",") if x.strip()]
# #         sel_cols = []
# #         for cid in sel_ids:
# #             cname = f"cell_{cid}"
# #             if cname not in cell_cols:
# #                 raise ValueError(f"Requested cell index {cid} not found (have: {cell_cols})")
# #             sel_cols.append(cname)
# #     elif args.topk:
# #         # choose top-K by total count across iterations
# #         totals = df[cell_cols].sum(axis=0)
# #         sel_cols = list(totals.sort_values(ascending=False).head(args.topk).index)
# #     else:
# #         sel_cols = cell_cols  # all
# #
# #     # Data
# #     iters = df["iter"].to_numpy()
# #     plt.figure(figsize=(args.figw, args.figh))
# #
# #     for cname in sel_cols:
# #         y = pd.to_numeric(df[cname], errors="coerce").fillna(0).to_numpy()
# #         y_s = _moving_average(y, args.smooth)
# #         plt.plot(iters, y_s, marker="o", linewidth=1, label=cname.split("_")[-1])
# #
# #     plt.xlabel("iteration")
# #     plt.ylabel("balls in cell")
# #     plt.title("Grid Cell vs. Balls per Iteration")
# #     plt.grid(True, alpha=0.3)
# #     # Show legend only if not too many lines
# #     if len(sel_cols) <= 18:
# #         plt.legend(title="cell", ncol=2, fontsize=9)
# #     plt.tight_layout()
# #     plt.savefig(args.out, dpi=180)
# #     plt.close()
# #
# #     print(f"✅ Saved line chart: {args.out}")
# #
# #
# # if __name__ == "__main__":
# #     main()
# #
# #
#
#




# #!/usr/bin/env python3
#
# import os
# import re
# import csv
# import json
# import time
# import uuid
# import builtins
# import subprocess
# import numpy as np
# import mujoco
# from google import genai
# import imageio
#
# # ====== EDIT THESE PATHS ======
# CSV_MOVE_PATH = "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/logs/move.csv"
# WEIGHTS_TXT = "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/logs/weight.txt"
# BASE_DIR = "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/"
# # ==============================
#
# HISTORY_WINDOW = 25
# TRAJECTORY_HISTORY_WINDOW = 20  # NEW: Track last 20 iterations of X,Y trajectories
#
# LOGDIR = os.path.join(BASE_DIR, "logs")
# WEIGHTS_CSV = os.path.join(LOGDIR, "weights.csv")
# ITER_LOG_CSV = os.path.join(LOGDIR, "llm_iteration_log.csv")
# DIALOG_DIR = os.path.join(LOGDIR, "llm_dialog")
# WEIGHT_HISTORY_CSV = os.path.join(LOGDIR, "weights_history.csv")
# TRAJECTORY_CSV = os.path.join(LOGDIR, "trajectory_feedback.csv")  # NEW: Store X,Y trajectories per iteration
#
# N_BFS = 25
# MAX_ITERS = 50
# IK_MAX_ITERS = 60
# DECI_BUILD = 1  # keep every k-th DMP step when building joints (1=all)
#
# GEMINI_MODEL = "gemini-2.0-flash"  # use Pro or gemini-2.0-flash if you want faster/cheaper
# GEMINI = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY"))
#
# builtins.input = lambda *a, **k: "7"
#
# from testiing_2 import (
#     EnhancedDMPController, MOP_Z_HEIGHT,
#     enhanced_ik_solver, get_joint_positions, set_joint_positions
# )
# from pydmps.dmp_rhythmic import DMPs_rhythmic
#
# controller = EnhancedDMPController()
# bounds = {
#     "xmin": controller.x_min, "xmax": controller.x_max,
#     "ymin": controller.y_min, "ymax": controller.y_max,
# }
#
# MAX_CHANGED = 12
# DELTA_ABS = 5.0
# DELTA_REL = 0.08
#
# # ----------------- utils -----------------
#
# def ensure_dirs():
#     os.makedirs(LOGDIR, exist_ok=True)
#     os.makedirs(DIALOG_DIR, exist_ok=True)
#
# def parse_weights_text(path):
#     with open(path, "r", encoding="utf-8") as f:
#         txt = f.read()
#     nums = re.findall(r"[-+]?\d*\.?\d+", txt)
#     if not nums:
#         raise ValueError(f"No numeric weights found in {path}")
#     return np.array([float(x) for x in nums], dtype=float)
#
# def row_to_2x50(arr):
#     """Accepts any even-length flat weight vector and returns shape (2, N_BFS),
#     resizing per-axis weights if needed via linear interpolation."""
#     a = np.asarray(arr, dtype=float).flatten()
#     if a.size % 2 != 0:
#         raise ValueError(f"Expected even number of weights, got {a.size}")
#
#     cur_n_bfs = a.size // 2
#     w2 = a.reshape(2, cur_n_bfs)
#
#     if cur_n_bfs == N_BFS:
#         return w2
#
#     # Resize each axis from cur_n_bfs -> N_BFS using linear interpolation
#     src_x = np.linspace(0.0, 1.0, cur_n_bfs)
#     dst_x = np.linspace(0.0, 1.0, N_BFS)
#     w_resized = np.empty((2, N_BFS), dtype=float)
#     for d in range(2):
#         w_resized[d] = np.interp(dst_x, src_x, w2[d])
#     return w_resized
#
# def write_weights_csv(path, w2):
#     row = w2.reshape(-1)
#     with open(path, "w", newline="") as f:
#         csv.writer(f).writerow(list(row))
#
# def read_weights_csv(path):
#     with open(path, "r", encoding="utf-8") as f:
#         txt = f.read()
#     nums = re.findall(r"[-+]?\d*\.?\d+", txt)
#     if not nums:
#         raise ValueError(f"No numbers in {path}")
#     return row_to_2x50([float(x) for x in nums])
#
# def read_move_csv(path):
#     try:
#         # Try named columns
#         data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
#         if data.dtype.names and {"x", "y"}.issubset(data.dtype.names):
#             xy = np.column_stack([data["x"], data["y"]]).astype(float)
#             if xy.ndim == 2 and xy.shape[1] >= 2:
#                 return xy
#     except Exception:
#         pass
#
#     # Fallback: two columns
#     xy = np.loadtxt(path, delimiter=",", dtype=float)
#     if xy.ndim != 2 or xy.shape[1] < 2:
#         raise ValueError(f"{path} must have at least two columns (x,y)")
#     return xy[:, :2].astype(float)
#
# def log_iteration(iter_idx, grid_mat, total_balls, traj_len, out_csv):
#     flat = list(map(int, grid_mat.flatten()))
#     file_exists = os.path.exists(out_csv)
#     with open(out_csv, "a", newline="") as f:
#         w = csv.writer(f)
#         if not file_exists:
#             w.writerow(["iter", "timestamp", "traj_waypoints", "total_balls"] +
#                       [f"cell{i}" for i in range(len(flat))])
#         w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), traj_len, total_balls] + flat)
#
# # NEW: Functions for trajectory feedback
# def save_trajectory_data(iter_idx, task_trajectory, csv_path):
#     """Save X,Y trajectory coordinates for this iteration."""
#     file_exists = os.path.exists(csv_path)
#     with open(csv_path, "a", newline="") as f:
#         w = csv.writer(f)
#         if not file_exists:
#             w.writerow(["iter", "step", "x", "y", "timestamp"])
#
#         timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
#         for step_idx, target in enumerate(task_trajectory):
#             x, y = target[0], target[1]  # Extract X,Y (Z is constant)
#             w.writerow([iter_idx, step_idx, float(x), float(y), timestamp])
#
# def load_trajectory_history(csv_path, max_iters=20):
#     """Load last max_iters iterations of trajectory data."""
#     if not os.path.exists(csv_path):
#         return {}
#
#     try:
#         data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding="utf-8")
#         if data.size == 0:
#             return {}
#
#         # Group by iteration
#         trajectory_history = {}
#         for row in data:
#             iter_num = int(row["iter"])
#             if iter_num not in trajectory_history:
#                 trajectory_history[iter_num] = []
#             trajectory_history[iter_num].append({
#                 "step": int(row["step"]),
#                 "x": float(row["x"]),
#                 "y": float(row["y"])
#             })
#
#         # Return only last max_iters iterations
#         sorted_iters = sorted(trajectory_history.keys())
#         if len(sorted_iters) > max_iters:
#             sorted_iters = sorted_iters[-max_iters:]
#
#         return {k: trajectory_history[k] for k in sorted_iters}
#
#     except Exception as e:
#         print(f"Warning: Could not load trajectory history: {e}")
#         return {}
#
# def analyze_trajectory_performance(trajectory_data, bounds):
#     """Analyze trajectory quality: coverage, bounds compliance, smoothness."""
#     if not trajectory_data:
#         return {}
#
#     analysis = {}
#
#     for iter_num, traj_points in trajectory_data.items():
#         if not traj_points:
#             continue
#
#         xs = [p["x"] for p in traj_points]
#         ys = [p["y"] for p in traj_points]
#
#         # Bounds compliance
#         x_in_bounds = all(bounds["xmin"] <= x <= bounds["xmax"] for x in xs)
#         y_in_bounds = all(bounds["ymin"] <= y <= bounds["ymax"] for y in ys)
#
#         # Coverage metrics
#         x_range_covered = (max(xs) - min(xs)) / (bounds["xmax"] - bounds["xmin"])
#         y_range_covered = (max(ys) - min(ys)) / (bounds["ymax"] - bounds["ymin"])
#
#         # Smoothness (path length vs direct distance)
#         path_length = sum(np.sqrt((xs[i+1] - xs[i])**2 + (ys[i+1] - ys[i])**2)
#                          for i in range(len(xs)-1)) if len(xs) > 1 else 0
#         direct_distance = np.sqrt((xs[-1] - xs[0])**2 + (ys[-1] - ys[0])**2) if len(xs) > 1 else 0
#         smoothness = direct_distance / path_length if path_length > 0 else 0
#
#         analysis[iter_num] = {
#             "bounds_compliant": x_in_bounds and y_in_bounds,
#             "x_coverage": x_range_covered,
#             "y_coverage": y_range_covered,
#             "smoothness": smoothness,
#             "path_length": path_length,
#             "waypoint_count": len(traj_points)
#         }
#
#     return analysis
#
# def enhanced_ollama_prompt(prev_w_flat, grid_mat, total_balls, iter_idx, history,
#                           trajectory_history, trajectory_analysis, bounds,
#                           max_changed=12, delta_abs=5.0, delta_rel=0.08):
#
#     xmin, xmax = bounds["xmin"], bounds["xmax"]
#     ymin, ymax = bounds["ymin"], bounds["ymax"]
#     grid_list = grid_mat.tolist()
#
#     # Format trajectory history for LLM
#     traj_feedback = {}
#     for iter_num, analysis in trajectory_analysis.items():
#         if iter_num in trajectory_history:
#             # Sample trajectory points (first, middle, last)
#             traj_points = trajectory_history[iter_num]
#             n_points = len(traj_points)
#             sample_indices = [0, n_points//2, n_points-1] if n_points > 2 else list(range(n_points))
#             sampled_points = [traj_points[i] for i in sample_indices if i < n_points]
#
#             traj_feedback[iter_num] = {
#                 "performance": analysis,
#                 "sample_trajectory": sampled_points,
#                 "total_waypoints": n_points
#             }
#
#     return f"""You are optimizing rhythmic DMP weights for a 2-DoF end-effector sweeping task with ENHANCED trajectory feedback.
#
# System: You are a global RL policy optimizer, helping me find the global optimal policy in the following environment.
#
# Environment:
# - A planar end-effector (2-DoF in x,y) performs a rhythmic sweep over a tabletop covered with light balls.
# - One rollout = one full sweep. After the sweep, we observe remaining balls via a coarse grid.
# - Controller uses Rhythmic DMPs with n_bfs={N_BFS}. Weights are flattened [x then y] with total length {2*N_BFS}.
# - A single rollout executes one full sweep; cost = total balls remaining on the table.
# - Spatial feedback is a 3x2 grid of ball counts (rows=y high→low, cols=x left→right): {json.dumps(grid_list)}
#
# NEW ENHANCED FEEDBACK: X,Y Trajectory Performance
# - You now receive detailed trajectory data from the last {TRAJECTORY_HISTORY_WINDOW} iterations
# - This includes actual X,Y coordinates the robot followed, coverage analysis, and bounds compliance
# - Use this to understand HOW the robot moved, not just the final cleaning result
#
# Trajectory History (last {len(traj_feedback)} iterations): {json.dumps(traj_feedback)}
#
# Hard constraints (must follow):
# - XY WORKSPACE LIMITS (meters): x ∈ [{xmin:.3f}, {xmax:.3f}], y ∈ [{ymin:.3f}, {ymax:.3f}]
#   Any path implied by your weights must keep the 2D end-effector trajectory strictly within these bounds.
#   If you propose weights that drive points outside these bounds, the IK will fail → waypoints rejected.
#
# - FULL-GRID COVERAGE PER CYCLE: design the sweep so it passes through ALL grid sectors.
#   Do NOT tunnel on a single side. Prioritize cells with higher ball counts, but ensure at least one pass over each sector every cycle.
#
# - TRAJECTORY QUALITY: Based on the trajectory history, optimize for:
#   * Bounds compliance (keep all X,Y within workspace)
#   * Good coverage (high x_coverage and y_coverage values)
#   * Smooth motion (reasonable smoothness metric)
#   * Effective cleaning (correlation between trajectory path and ball reduction)
#
# - When creating weights, use the trajectory feedback to understand:
#   * If previous motions stayed in bounds
#   * Which areas were actually covered vs missed
#   * How smooth/erratic the motion was
#   * Where the robot spent time vs where balls remain
#
# Stability and continuity:
# - You will return a NEW weight vector (length {2*N_BFS}) that may adjust some or all components as needed to improve ball removal AND trajectory quality.
# - Prefer small, smooth, coordinated updates that improve both cleaning performance AND motion quality.
# - Modify at most {max_changed} weights out of {2*N_BFS} (subset updates).
# - For any changed weight w_i:
#   * Absolute delta limit: |Δw_i| ≤ {delta_abs}
#   * Relative delta limit: |Δw_i| ≤ {delta_rel} * max(1e-6, |w_prev_i|)
#
# - History of past weights (oldest→newest), for all previous iterations: {json.dumps(history)}
#
# Current state:
# - Iteration: {iter_idx}
# - Current cost (total balls): {total_balls}
# - Current weights (length {2*N_BFS}, flattened [x then y]): {json.dumps([float(x) for x in prev_w_flat])}
# - Recent weight history (oldest→newest), up to last {HISTORY_WINDOW} proposals/executions: {json.dumps(history)}
#
# Objective:
# - Reduce total balls toward 0 while satisfying bounds and full-grid coverage
# - Improve trajectory quality: better coverage, bounds compliance, and smoothness
# - Keep the trajectory coherent and rhythmic; avoid large amplitude jumps or drift
# - Use trajectory feedback to understand robot behavior and optimize accordingly
#
# Output: STRICTLY one JSON object on a single line (no code fences). Include a brief "reason" and the "weights":
# {{"reason": "one sentence explaining coverage, bounds adherence, and trajectory improvements", "weights": [exactly {2*N_BFS} floats]}}"""
#
# def call_gemini(prompt: str) -> str:
#     try:
#         resp = GEMINI.models.generate_content(
#             model=GEMINI_MODEL,
#             contents=prompt,
#             config={"temperature": 0.2}  # stable, low-noise updates
#         )
#         text = getattr(resp, "text", None) or str(resp)
#         return text.strip()
#     except Exception as e:
#         raise RuntimeError(f"Gemini call failed: {e}")
#
# def parse_ollama_weights(out_text):
#     text = out_text.strip()
#     json_start = text.find("{")
#
#     try:
#         obj = json.loads(out_text)
#         cand = obj.get("weights", None)
#         if cand is not None and isinstance(cand, list):
#             return row_to_2x50(cand)
#     except Exception:
#         pass
#
#     # Fallback: extract all floats and reshape
#     nums = re.findall(r"[-+]?\d*\.?\d+", out_text)
#     if len(nums) >= 2*N_BFS:
#         return row_to_2x50([float(x) for x in nums[:2*N_BFS]])
#
#     raise ValueError("Could not parse weights from LLM output")
#
# def save_dialog(iter_idx, prompt, response):
#     pid = f"iter_{iter_idx:03d}_{uuid.uuid4().hex[:8]}"
#     with open(os.path.join(DIALOG_DIR, pid + "_prompt.txt"), "w", encoding="utf-8") as f:
#         f.write(prompt)
#     with open(os.path.join(DIALOG_DIR, pid + "_response.txt"), "w", encoding="utf-8") as f:
#         f.write(response)
#
# def append_weight_history(csv_path, iter_idx, tag, w2):
#     """Append a single row to weight_history.csv.
#
#     Parameters
#     ----------
#     csv_path : str
#     iter_idx : int
#         iteration number (use 0 for initial)
#     tag : str
#         "executed" or "proposed"
#     w2 : np.ndarray
#         shape (2, N_BFS)
#     """
#     flat = list(map(float, w2.reshape(-1)))
#     file_exists = os.path.exists(csv_path)
#     with open(csv_path, "a", newline="") as f:
#         w = csv.writer(f)
#         if not file_exists:
#             header = ["iter", "timestamp", "tag"] + [f"w{i}" for i in range(2 * N_BFS)]
#             w.writerow(header)
#         w.writerow([iter_idx, time.strftime("%Y-%m-%d %H:%M:%S"), tag] + flat)
#
# #
# # def execute_and_record_joint_trajectory(controller, joint_traj, video_path, fps=60, width=2560, height=1440):
# #     """Execute joint trajectory while recording to MP4, auto-detecting framebuffer limits."""
# #
# #     # Read offscreen framebuffer size from model (defaults: 640x480)
# #     try:
# #         offw = int(getattr(controller.model.vis.global_, 'offwidth', 640))
# #         offh = int(getattr(controller.model.vis.global_, 'offheight', 480))
# #         if offw <= 0 or offh <= 0:
# #             offw, offh = 640, 480
# #     except Exception:
# #         offw, offh = 640, 480
# #
# #     # Clamp requested size to available framebuffer
# #     width = min(width, offw)
# #     height = min(height, offh)
# #     print(f"[recording] Using resolution {width}x{height} (framebuffer: {offw}x{offh})")
# #
# #     renderer = None
# #     writer = None
# #
# #     try:
# #         renderer = mujoco.Renderer(controller.model, height, width)
# #         writer = imageio.get_writer(video_path, fps=fps)
# #
# #         # Execute trajectory with recording
# #         for joints in joint_traj:
# #             controller.data.ctrl[:] = joints
# #             mujoco.mj_step(controller.model, controller.data)
# #
# #             # Keep interactive viewer in sync (if present)
# #             try:
# #                 controller.viewer.draw()
# #             except Exception:
# #                 pass
# #
# #             # Capture frame for video
# #             frame = renderer.render()
# #             writer.append_data(frame)
# #             time.sleep(controller.dt)
# #
# #         print(f"[recording] Saved video to: {video_path}")
# #
# #     except Exception as e:
# #         print(f"[recording] Recording failed: {e}")
# #
# #     finally:
# #         # Clean shutdown to prevent Windows AttributeError
# #         try:
# #             if writer is not None:
# #                 writer.close()
# #         except Exception:
# #             pass
# #         try:
# #             if renderer is not None:
# #                 renderer.close()
# #         except Exception:
# #             pass
#
#
# def execute_and_record_joint_trajectory(controller, joint_traj, base_path, fps=60):
#     """
#     Execute trajectory and dump PNG frames to base_path + '_frames'.
#     For example, base_path='iter_005_record' → frames in 'iter_005_record_frames/'.
#     """
#     # Ensure an offscreen renderer (clamped to framebuffer)
#     offw = int(getattr(controller.model.vis.global_, 'offwidth', 640))
#     offh = int(getattr(controller.model.vis.global_, 'offheight', 480))
#     width, height = min(offw, 640), min(offh, 480)
#     try:
#         renderer = mujoco.Renderer(controller.model, height, width)
#     except Exception as e:
#         print(f"[recording] Offscreen renderer init failed: {e}")
#         return
#
#     frames_dir = f"{base_path}_frames"
#     os.makedirs(frames_dir, exist_ok=True)
#
#     # Step through trajectory and save PNGs
#     for idx, joints in enumerate(joint_traj):
#         controller.data.ctrl[:] = joints
#         mujoco.mj_step(controller.model, controller.data)
#         # Capture frame
#         frame = renderer.render()
#         png_path = os.path.join(frames_dir, f"frame_{idx:05d}.png")
#         imageio.imwrite(png_path, frame)
#         time.sleep(controller.dt)
#
#     # Clean up renderer
#     try:
#         renderer.close()
#     except Exception:
#         pass
#
#     print(f"[recording] Saved frames to: {frames_dir}")
#     print(f"[recording] Run this to make MP4:")
#     print(f"ffmpeg -r {fps} -i {frames_dir}/frame_%05d.png -c:v libx264 {base_path}.mp4")
#
#
#
# # --------------- main loop ---------------
#
# def main():
#     ensure_dirs()
#
#     weight_history = []
#
#     # Bootstrap weights.csv from weights.txt if missing
#     if not os.path.exists(WEIGHTS_CSV):
#         if not os.path.exists(WEIGHTS_TXT):
#             raise FileNotFoundError(f"Missing {WEIGHTS_CSV} and {WEIGHTS_TXT}")
#
#         flat0 = parse_weights_text(WEIGHTS_TXT)
#         w0 = row_to_2x50(flat0)
#         write_weights_csv(WEIGHTS_CSV, w0)
#         print(f"Initialized {WEIGHTS_CSV} from {WEIGHTS_TXT} → shape {w0.shape}")
#         append_weight_history(WEIGHT_HISTORY_CSV, 0, "executed", w0)
#
#     # Controller (one viewer, no reset later)
#     controller = EnhancedDMPController()
#
#     if not os.path.exists(CSV_MOVE_PATH):
#         raise FileNotFoundError(f"Demo path not found: {CSV_MOVE_PATH}")
#     demo_xy = read_move_csv(CSV_MOVE_PATH)
#
#     # Prime DMP ONCE from move.csv (y0, etc.)
#     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=N_BFS, dt=controller.dt)
#     dmp.imitate_path(y_des=demo_xy.T)
#
#     # Main optimization loop
#     for it in range(1, MAX_ITERS + 1):
#         # Iterations
#         controller.hard_reset_from_home()
#
#         # Read current weights
#         w2 = read_weights_csv(WEIGHTS_CSV)
#         w_flat = w2.reshape(-1)
#         append_weight_history(WEIGHT_HISTORY_CSV, it, "executed", w2)
#
#         # Apply weights
#         dmp.w = w2.copy()
#         dmp.reset_state()
#
#         # Convert one full cycle to joint trajectory
#         model, data = controller.model, controller.data
#         site_id = controller.site_id
#         joint_names = controller.joint_names
#         start_joints = get_joint_positions(model, data, joint_names)
#
#         joint_traj = []
#         task_trajectory = []  # NEW: Store task-space trajectory for feedback
#         steps = int(dmp.timesteps)
#         keep_every = max(1, int(DECI_BUILD))
#
#         for i in range(steps):
#             y, _, _ = dmp.step()
#             target_3d = np.array([y[0], y[1], MOP_Z_HEIGHT], dtype=float)
#             task_trajectory.append(target_3d)  # NEW: Save for trajectory analysis
#
#             ok, _ = enhanced_ik_solver(
#                 model, data, site_id, target_3d, joint_names,
#                 max_iters_per_wp=IK_MAX_ITERS, print_every=1000000
#             )
#             if not ok:
#                 continue
#             if i % keep_every == 0:
#                 joint_traj.append(get_joint_positions(model, data, joint_names).copy())
#
#         if not joint_traj:
#             print(f"iter {it}: No joints generated, skipping execution.")
#         else:
#             # Restore start joints so playback is clean
#             set_joint_positions(model, data, joint_names, start_joints)
#
#             if it % 5 == 0:
#                 base = os.path.join(BASE_DIR, f"iter_{it:03d}_record")
#                 print(f"iter {it}: Recording frames to {base}_frames/")
#                 execute_and_record_joint_trajectory(controller, joint_traj, base, fps=60)
#             else:
#                 controller.execute_joint_trajectory(joint_traj, dt=controller.dt)
#
#         # NEW: Save trajectory data for this iteration
#         save_trajectory_data(it, task_trajectory, TRAJECTORY_CSV)
#
#         # Compute cost via your grid counter
#         grid = controller.count_balls_in_grid()
#         controller.grid_count = grid  # (ny, nx), after their transpose
#         total_balls = int(np.sum(grid))
#
#         log_iteration(it, grid, total_balls, len(joint_traj), ITER_LOG_CSV)
#         print(f"iter {it}: Cost (total balls) = {total_balls}, per-cell = {grid.tolist()}")
#
#         if total_balls == 0:
#             print(f"iter {it}: Done, no balls left.")
#             break
#
#         # NEW: Load and analyze trajectory history
#         trajectory_history = load_trajectory_history(TRAJECTORY_CSV, TRAJECTORY_HISTORY_WINDOW)
#         trajectory_analysis = analyze_trajectory_performance(trajectory_history, bounds)
#
#         # Ask LLM for NEW weights given cost, grid, prev weights, AND trajectory feedback
#         hist_slice = weight_history[-HISTORY_WINDOW:] if HISTORY_WINDOW > 0 else weight_history
#
#         prompt = enhanced_ollama_prompt(
#             w_flat, grid, total_balls, it, hist_slice,
#             trajectory_history, trajectory_analysis, bounds,
#             max_changed=MAX_CHANGED, delta_abs=DELTA_ABS, delta_rel=DELTA_REL
#         )
#
#         try:
#             response = call_gemini(prompt)
#         except Exception as e:
#             print(f"iter {it}: Gemini error: {e}. Reusing previous weights.")
#             time.sleep(1.0)
#             continue
#
#         save_dialog(it, prompt, response)
#
#         try:
#             w_next = parse_ollama_weights(response)  # (2,50)
#         except Exception as e:
#             print(f"iter {it}: Failed to parse LLM weights: {e}. Reusing previous weights.")
#             time.sleep(1.0)
#             continue
#
#         append_weight_history(WEIGHT_HISTORY_CSV, it, "proposed", w_next)
#         write_weights_csv(WEIGHTS_CSV, w_next)
#         print(f"iter {it}: Updated {WEIGHTS_CSV} with new weights from LLM.")
#
#         weight_history.append(w_next.reshape(-1).tolist())
#         time.sleep(40)
#
#     print("Loop finished. Close the viewer to exit.")
#
# if __name__ == "__main__":
#     main()



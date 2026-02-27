# import os
# from collections import defaultdict

# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt

# from minisom import MiniSom


# # =========================================================
# # CONFIG (DYNAMIC)
# # Put this script next to your "output" folder
# # =========================================================
# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")   # filtered outputs live here
# RESULT_DIR = os.path.join(SCRIPT_DIR, "result")   # save SOM results here
# os.makedirs(RESULT_DIR, exist_ok=True)

# # SOM params
# SOM_SIGMA = 1.5
# SOM_LR = 0.5
# SOM_ITERS = 5000

# # How many best examples to export per SOM cell
# TOPK_PER_CELL = 10

# # Downsample each iteration trajectory for feature building (speed + stability)
# RESAMPLE_N = 200

# # Z-score normalize features before SOM
# EPS = 1e-9

# # =========================================================
# # CSV helpers
# # =========================================================
# def list_csvs(folder_path):
#     if not os.path.isdir(folder_path):
#         return []
#     return [
#         os.path.join(folder_path, f)
#         for f in os.listdir(folder_path)
#         if f.lower().endswith(".csv")
#     ]

# def pick_file(csv_paths, keyword):
#     keyword = keyword.lower()
#     for p in csv_paths:
#         if keyword in os.path.basename(p).lower():
#             return p
#     return None

# def find_iter_col(df):
#     for c in ["iter", "iteration", "Iteration", "ITER", "step", "idx", "step_idx"]:
#         if c in df.columns:
#             return c
#     raise ValueError(f"Iteration column not found. Columns: {list(df.columns)}")

# def find_xy_cols(df):
#     for xcol, ycol in [("x", "y"), ("X", "Y")]:
#         if xcol in df.columns and ycol in df.columns:
#             return xcol, ycol
#     raise ValueError(f"x/y columns not found. Columns: {list(df.columns)}")

# def zscore(X):
#     mu = np.nanmean(X, axis=0)
#     sd = np.nanstd(X, axis=0) + EPS
#     return (X - mu) / sd

# def ensure_dir(p):
#     os.makedirs(p, exist_ok=True)
#     return p


# # =========================================================
# # Discovery: output/<run>/<exp>/*.csv
# # =========================================================
# def discover_all_experiments(output_dir):
#     items = []
#     if not os.path.isdir(output_dir):
#         return items

#     run_names = [d for d in os.listdir(output_dir) if os.path.isdir(os.path.join(output_dir, d))]
#     run_names = sorted(run_names)

#     for run in run_names:
#         run_path = os.path.join(output_dir, run)
#         exp_nums = [d for d in os.listdir(run_path)
#                     if os.path.isdir(os.path.join(run_path, d)) and d.isdigit()]
#         exp_nums = sorted(exp_nums, key=lambda x: int(x))

#         for exp in exp_nums:
#             exp_path = os.path.join(run_path, exp)
#             csvs = list_csvs(exp_path)

#             llm = pick_file(csvs, "llm_iteration_log")
#             dmp = pick_file(csvs, "dmp_trajectory_feedback")
#             ee  = pick_file(csvs, "ee_trajectory")

#             if (dmp is None) and (ee is None):
#                 continue

#             items.append({
#                 "run": run,
#                 "exp_num": int(exp),
#                 "exp_path": exp_path,
#                 "llm_csv": llm,
#                 "dmp_csv": dmp,
#                 "ee_csv": ee,
#             })

#     return items


# # =========================================================
# # Trajectory utilities
# # =========================================================
# def resample_xy(xy: np.ndarray, n: int) -> np.ndarray:
#     """Arc-length resample to n points."""
#     if xy.shape[0] < 2:
#         return np.repeat(xy[:1], n, axis=0)

#     d = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))
#     s = np.insert(np.cumsum(d), 0, 0.0)
#     if s[-1] <= 0:
#         return np.repeat(xy[:1], n, axis=0)

#     t_new = np.linspace(0, s[-1], n)
#     x_new = np.interp(t_new, s, xy[:, 0])
#     y_new = np.interp(t_new, s, xy[:, 1])
#     return np.column_stack([x_new, y_new])

# def traj_features(xy: np.ndarray) -> dict:
#     """
#     Simple strong features for shape clustering:
#       - bbox (min/max/width/height)
#       - path length
#       - mean abs heading change (smooth_turn)
#     """
#     if xy.shape[0] < 6:
#         return None

#     xs = xy[:, 0]
#     ys = xy[:, 1]
#     x_min, x_max = float(xs.min()), float(xs.max())
#     y_min, y_max = float(ys.min()), float(ys.max())
#     width = x_max - x_min
#     height = y_max - y_min

#     dx = np.diff(xs)
#     dy = np.diff(ys)
#     seg = np.sqrt(dx*dx + dy*dy)
#     path_len = float(np.sum(seg))

#     heading = np.arctan2(dy, dx)
#     dhead = np.diff(heading)
#     dhead = (dhead + np.pi) % (2*np.pi) - np.pi
#     smooth_turn = float(np.mean(np.abs(dhead))) if len(dhead) else 0.0

#     return {
#         "x_min": x_min, "x_max": x_max,
#         "y_min": y_min, "y_max": y_max,
#         "width": width, "height": height,
#         "path_len": path_len,
#         "smooth_turn": smooth_turn
#     }

# def load_iter_map_xy(csv_path: str):
#     """
#     Returns dict: iter -> (N,2) raw xy array
#     """
#     if csv_path is None or (not os.path.isfile(csv_path)):
#         return {}

#     df = pd.read_csv(csv_path)
#     itc = find_iter_col(df)
#     xcol, ycol = find_xy_cols(df)

#     df[itc] = pd.to_numeric(df[itc], errors="coerce").astype("Int64")
#     df[xcol] = pd.to_numeric(df[xcol], errors="coerce")
#     df[ycol] = pd.to_numeric(df[ycol], errors="coerce")
#     df = df.dropna(subset=[itc, xcol, ycol])

#     out = {}
#     for it, g in df.groupby(itc):
#         out[int(it)] = g[[xcol, ycol]].to_numpy(dtype=float)
#     return out


# # =========================================================
# # Build GLOBAL dataset: one row per iteration
# # (Uses BOTH DMP and EE features if available)
# # =========================================================
# def load_llm_balls_map(llm_csv):
#     if llm_csv is None or (not os.path.isfile(llm_csv)):
#         return {}

#     llm = pd.read_csv(llm_csv)
#     if "total_balls" not in llm.columns:
#         return {}

#     itc = find_iter_col(llm)
#     llm[itc] = pd.to_numeric(llm[itc], errors="coerce").astype("Int64")
#     llm["total_balls"] = pd.to_numeric(llm["total_balls"], errors="coerce")
#     llm = llm.dropna(subset=[itc])

#     return dict(zip(llm[itc].astype(int), llm["total_balls"]))

# def build_global_rows(items):
#     """
#     Returns:
#       X_all: (M,D) feature matrix
#       meta_all: list[dict] with run, exp, iteration, total_balls
#       sources: list[dict] with dmp_csv/ee_csv paths for plotting
#       feat_names: list[str]
#     """
#     rows = []
#     meta_all = []
#     sources = []

#     feat_names = [
#         # DMP features
#         "dmp_x_min","dmp_x_max","dmp_y_min","dmp_y_max","dmp_width","dmp_height","dmp_path_len","dmp_smooth_turn",
#         # EE features
#         "ee_x_min","ee_x_max","ee_y_min","ee_y_max","ee_width","ee_height","ee_path_len","ee_smooth_turn",
#         # DMP-EE mismatch
#         "dmp_ee_mean_dist"
#     ]

#     for it in items:
#         balls_map = load_llm_balls_map(it["llm_csv"])

#         dmp_map = load_iter_map_xy(it["dmp_csv"])
#         ee_map  = load_iter_map_xy(it["ee_csv"])

#         all_iters = sorted(set(dmp_map.keys()) | set(ee_map.keys()))
#         if not all_iters:
#             continue

#         for iteration in all_iters:
#             dmp_xy = dmp_map.get(iteration, None)
#             ee_xy  = ee_map.get(iteration, None)

#             # resample for consistent feature calc
#             dmp_rs = resample_xy(dmp_xy, RESAMPLE_N) if (dmp_xy is not None and len(dmp_xy) > 1) else None
#             ee_rs  = resample_xy(ee_xy, RESAMPLE_N)  if (ee_xy  is not None and len(ee_xy)  > 1) else None

#             dmp_f = traj_features(dmp_rs) if dmp_rs is not None else None
#             ee_f  = traj_features(ee_rs)  if ee_rs  is not None else None

#             # require at least one trajectory to be usable
#             if (dmp_f is None) and (ee_f is None):
#                 continue

#             # fill missing side with NaNs
#             def pack(prefix, f):
#                 if f is None:
#                     return [np.nan]*8
#                 return [
#                     f["x_min"], f["x_max"], f["y_min"], f["y_max"],
#                     f["width"], f["height"], f["path_len"], f["smooth_turn"]
#                 ]

#             dmp_vec = pack("dmp", dmp_f)
#             ee_vec  = pack("ee", ee_f)

#             # DMP-EE mismatch (mean pointwise dist after resample)
#             if dmp_rs is not None and ee_rs is not None:
#                 dist = np.sqrt(np.sum((dmp_rs - ee_rs)**2, axis=1))
#                 mean_dist = float(np.mean(dist))
#             else:
#                 mean_dist = np.nan

#             vec = dmp_vec + ee_vec + [mean_dist]
#             rows.append(vec)

#             meta_all.append({
#                 "run": it["run"],
#                 "exp": f"exp{it['exp_num']}",
#                 "iteration": int(iteration),
#                 "total_balls": float(balls_map.get(int(iteration), np.nan))
#             })
#             sources.append(it)

#     if not rows:
#         return None, [], [], feat_names

#     X_all = np.array(rows, dtype=float)
#     return X_all, meta_all, sources, feat_names


# # =========================================================
# # SOM: train + graphs
# # =========================================================
# def choose_grid(n_samples: int) -> int:
#     # simple heuristic: bigger data => bigger grid
#     g = int(np.sqrt(np.sqrt(max(n_samples, 1))) * 10)
#     return max(10, min(g, 30))

# def train_som(X_all):
#     Xn = zscore(X_all)
#     grid = choose_grid(Xn.shape[0])

#     som = MiniSom(x=grid, y=grid, input_len=Xn.shape[1], sigma=1.5, learning_rate=0.5)
#     som.random_weights_init(Xn)
#     som.train_random(Xn, num_iteration=SOM_ITERS)

#     winners = [som.winner(v) for v in Xn]
#     return som, winners, grid, Xn

# def save_umatrix(som, out_png):
#     um = som.distance_map()
#     plt.figure(figsize=(7,6))
#     plt.imshow(um.T, origin="lower")
#     plt.colorbar()
#     plt.title("SOM U-Matrix (distance map)")
#     plt.tight_layout()
#     plt.savefig(out_png, dpi=220)
#     plt.close()

# def save_hitmap(grid, winners, out_png):
#     counts = np.zeros((grid, grid), dtype=int)
#     for (x, y) in winners:
#         counts[x, y] += 1
#     plt.figure(figsize=(7,6))
#     plt.imshow(counts.T, origin="lower")
#     plt.colorbar()
#     plt.title("SOM Hit Map (samples per cell)")
#     plt.tight_layout()
#     plt.savefig(out_png, dpi=220)
#     plt.close()
#     return counts

# def save_median_balls_map(grid, winners, meta_all, out_png):
#     cell_vals = defaultdict(list)
#     for w, m in zip(winners, meta_all):
#         b = m.get("total_balls", np.nan)
#         if np.isfinite(b):
#             cell_vals[w].append(b)

#     med = np.full((grid, grid), np.nan, dtype=float)
#     for (x, y), vals in cell_vals.items():
#         med[x, y] = float(np.median(vals))

#     plt.figure(figsize=(7,6))
#     plt.imshow(med.T, origin="lower")
#     plt.colorbar()
#     plt.title("SOM Median total_balls per cell")
#     plt.tight_layout()
#     plt.savefig(out_png, dpi=220)
#     plt.close()
#     return med


# # =========================================================
# # Plot overlay (DMP + EE in same plot) for a single iteration
# # =========================================================
# def plot_overlay(dmp_csv, ee_csv, iteration, out_png, title):
#     dmp_map = load_iter_map_xy(dmp_csv)
#     ee_map  = load_iter_map_xy(ee_csv)

#     dmp_xy = dmp_map.get(int(iteration), None)
#     ee_xy  = ee_map.get(int(iteration), None)

#     if dmp_xy is None and ee_xy is None:
#         return False

#     plt.figure(figsize=(7.5, 6.5))
#     if dmp_xy is not None and len(dmp_xy) > 1:
#         plt.plot(dmp_xy[:, 0], dmp_xy[:, 1], color="green", linewidth=2.2, alpha=0.85, label="DMP")
#     if ee_xy is not None and len(ee_xy) > 1:
#         plt.plot(ee_xy[:, 0], ee_xy[:, 1], color="green", linewidth=2.2, alpha=0.35, label="EE")

#     plt.title(title)
#     plt.xlabel("x")
#     plt.ylabel("y")
#     plt.grid(True, alpha=0.25)
#     plt.legend()
#     plt.axis("equal")

#     plt.tight_layout()
#     plt.savefig(out_png, dpi=220)
#     plt.close()
#     return True


# # =========================================================
# # MAIN
# # =========================================================
# def main():
#     print("\n==================== GLOBAL SOM CLUSTERING ====================")
#     print("[INFO] OUTPUT_DIR:", OUTPUT_DIR)
#     print("[INFO] RESULT_DIR:", RESULT_DIR)
#     print("[INFO] OUTPUT_DIR exists:", os.path.isdir(OUTPUT_DIR))

#     if not os.path.isdir(OUTPUT_DIR):
#         raise RuntimeError(f"Filtered output folder not found: {OUTPUT_DIR}")

#     items = discover_all_experiments(OUTPUT_DIR)
#     if not items:
#         raise RuntimeError(f"No experiments found under: {OUTPUT_DIR}")

#     X_all, meta_all, sources, feat_names = build_global_rows(items)
#     if X_all is None or X_all.shape[0] < 2:
#         raise RuntimeError("Not enough iterations globally to cluster. Check that filtered CSVs contain iterations.")

#     print(f"[INFO] Total iterations (global samples): {X_all.shape[0]}")
#     print(f"[INFO] Feature dim confirm: {X_all.shape[1]}")

#     som, winners, grid, Xn = train_som(X_all)
#     print(f"[INFO] Trained SOM grid: {grid} x {grid}")

#     out_root = ensure_dir(os.path.join(RESULT_DIR, "global"))
#     plots_dir = ensure_dir(os.path.join(out_root, "plots"))
#     clusters_dir = ensure_dir(os.path.join(out_root, "clusters"))

#     # --- Global plots ---
#     save_umatrix(som, os.path.join(plots_dir, "som_umatrix.png"))
#     hit_counts = save_hitmap(grid, winners, os.path.join(plots_dir, "som_hitmap.png"))
#     med_map = save_median_balls_map(grid, winners, meta_all, os.path.join(plots_dir, "som_median_balls_map.png"))

#     # --- Assignments ---
#     assign_rows = []
#     for m, w in zip(meta_all, winners):
#         # naming convention you asked:
#         # "<run> exp3 iter 21"
#         name = f"{m['run']} {m['exp']} iter {m['iteration']}"
#         assign_rows.append({
#             "name": name,
#             "run": m["run"],
#             "exp": m["exp"],
#             "iteration": m["iteration"],
#             "total_balls": m.get("total_balls", np.nan),
#             "cell_x": w[0],
#             "cell_y": w[1],
#         })

#     assign_df = pd.DataFrame(assign_rows)
#     assign_df.to_csv(os.path.join(out_root, "som_assignments_all.csv"), index=False)

#     # --- Cluster summary per cell ---
#     clusters = defaultdict(list)
#     for row in assign_rows:
#         clusters[(row["cell_x"], row["cell_y"])].append(row)

#     summ = []
#     for (cx, cy), members in clusters.items():
#         balls = np.array([r["total_balls"] for r in members], dtype=float)
#         summ.append({
#             "cell_x": cx,
#             "cell_y": cy,
#             "count": len(members),
#             "median_balls": float(np.nanmedian(balls)) if np.isfinite(balls).any() else np.nan,
#             "best_balls": float(np.nanmin(balls)) if np.isfinite(balls).any() else np.nan,
#             "worst_balls": float(np.nanmax(balls)) if np.isfinite(balls).any() else np.nan,
#         })

#     summ_df = pd.DataFrame(summ).sort_values(
#         ["median_balls", "best_balls", "count"],
#         ascending=[True, True, False]
#     )
#     summ_df.to_csv(os.path.join(out_root, "som_cluster_summary_all.csv"), index=False)

#     # --- Export top-K plots per SOM cell (best balls first) ---
#     print("\n[INFO] Saving top trajectories per SOM cell...")
#     for (cx, cy), members in clusters.items():
#         # sort by balls (NaN last)
#         members_sorted = sorted(
#             members,
#             key=lambda r: (np.inf if not np.isfinite(r["total_balls"]) else r["total_balls"])
#         )
#         top = members_sorted[:TOPK_PER_CELL]
#         if not top:
#             continue

#         cell_dir = ensure_dir(os.path.join(clusters_dir, f"cell_{cx}_{cy}"))

#         for r in top:
#             # locate the correct source exp to load its dmp/ee CSVs
#             run = r["run"]
#             exp_num = int(str(r["exp"]).replace("exp", ""))
#             src = None
#             for it in items:
#                 if it["run"] == run and it["exp_num"] == exp_num:
#                     src = it
#                     break
#             if src is None:
#                 continue

#             fname = f"{r['run']} {r['exp']} iter {r['iteration']}.png"
#             out_png = os.path.join(cell_dir, fname)

#             title = f"{r['run']} {r['exp']} iter {r['iteration']} | cell({cx},{cy}) | balls={r['total_balls']}"
#             ok = plot_overlay(src["dmp_csv"], src["ee_csv"], r["iteration"], out_png, title)
#             if not ok:
#                 continue

#     print("\n✅ DONE.")
#     print("Saved GLOBAL SOM results here:")
#     print(out_root)
#     print("\nKey outputs:")
#     print("  - plots/som_umatrix.png")
#     print("  - plots/som_hitmap.png")
#     print("  - plots/som_median_balls_map.png")
#     print("  - som_assignments_all.csv")
#     print("  - som_cluster_summary_all.csv")
#     print("  - clusters/cell_x_y/<run exp iter>.png")


# if __name__ == "__main__":
#     main()








# ranking_som_weights_only.py
# =========================================================
# GLOBAL SOM CLUSTERING USING ONLY weights_history.csv
# Keeps the rest of the pipeline same idea:
#   - Detect output/<run_name>/<exp_num>/ folders
#   - Rank experiments best->worst using llm_iteration_log total_balls
#   - Build SOM dataset from weights_history.csv (tag == executed preferred)
#   - Save SOM plots + assignments + cluster summary
#   - Export example trajectory overlay plots (DMP + EE on same figure) per SOM cell
#
# Output:
#   output/result/
#     ranking_best_to_worst.csv
#     som_assignments_all.csv
#     som_cluster_summary_all.csv
#     plots/som_umatrix.png
#     plots/som_hitmap.png
#     plots/som_median_balls_map.png
#     clusters/cell_x_y/members.csv
#     clusters/cell_x_y/<run exp iter>.png
#
# Install:
#   python -m pip install minisom numpy pandas matplotlib
# =========================================================

import os
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from minisom import MiniSom

# =========================================================
# CONFIG (DYNAMIC)
# Put this script next to "output/" folder.
# =========================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
RESULT_DIR = os.path.join(OUTPUT_DIR, "result")
os.makedirs(RESULT_DIR, exist_ok=True)

RANDOM_SEED = 42

# SOM hyperparameters
SOM_ITERS = 6000
SOM_SIGMA = 1.8
SOM_LR = 0.45

# Export control
TOPK_PER_CELL = 20  # number of plotted trajectories per SOM cell

EPS = 1e-9


# =========================================================
# HELPERS
# =========================================================
def list_dirs(path):
    if not os.path.isdir(path):
        return []
    return [os.path.join(path, d) for d in os.listdir(path)
            if os.path.isdir(os.path.join(path, d))]


def list_csvs(path):
    if not os.path.isdir(path):
        return []
    return [os.path.join(path, f) for f in os.listdir(path)
            if f.lower().endswith(".csv")]


def pick_file(csvs, keyword):
    keyword = keyword.lower()
    for p in csvs:
        if keyword in os.path.basename(p).lower():
            return p
    return None


def find_iter_col(df):
    for c in ["iter", "iteration", "Iteration", "ITER", "step", "idx", "step_idx"]:
        if c in df.columns:
            return c
    return None


def find_xy_cols(df):
    for xcol, ycol in [("x", "y"), ("X", "Y")]:
        if xcol in df.columns and ycol in df.columns:
            return xcol, ycol
    return None, None


def zscore(X):
    mu = np.nanmean(X, axis=0)
    sd = np.nanstd(X, axis=0) + EPS
    return (X - mu) / sd


def safe_label(run_name, exp_num, it):
    # your requested naming convention
    return f"{run_name} exp{exp_num} iter {it}"


# =========================================================
# DISCOVERY: output/<run>/<exp>/  (exp is numeric)
# =========================================================
def discover_all_experiments(output_dir):
    items = []
    if not os.path.isdir(output_dir):
        return items

    for run_dir in list_dirs(output_dir):
        run_name = os.path.basename(run_dir)
        if run_name.lower() in ["result", "global_clusters"]:
            continue

        for exp_dir in list_dirs(run_dir):
            exp_name = os.path.basename(exp_dir)
            if not exp_name.isdigit():
                continue

            csvs = list_csvs(exp_dir)

            llm = pick_file(csvs, "llm_iteration_log")
            w = pick_file(csvs, "weights_history")
            dmp = pick_file(csvs, "dmp_trajectory_feedback")
            ee = pick_file(csvs, "ee_trajectory")

            # weights + llm are required for this pipeline
            if llm is None or w is None:
                continue

            items.append({
                "run_name": run_name,
                "exp_num": int(exp_name),
                "exp_dir": exp_dir,
                "llm_csv": llm,
                "w_csv": w,
                "dmp_csv": dmp,
                "ee_csv": ee,
            })

    return items


# =========================================================
# RANKING: best->worst experiments using llm_iteration_log
# =========================================================
def experiment_best_ball(llm_csv):
    df = pd.read_csv(llm_csv)
    if "total_balls" not in df.columns:
        return np.nan
    balls = pd.to_numeric(df["total_balls"], errors="coerce").dropna()
    if len(balls) == 0:
        return np.nan
    return float(balls.min())


def load_llm_balls_map(llm_csv):
    """
    Returns dict: iter -> total_balls
    """
    df = pd.read_csv(llm_csv)
    if "total_balls" not in df.columns:
        return {}

    itc = find_iter_col(df)
    if itc is None:
        return {}

    df[itc] = pd.to_numeric(df[itc], errors="coerce").astype("Int64")
    df["total_balls"] = pd.to_numeric(df["total_balls"], errors="coerce")
    df = df.dropna(subset=[itc])

    m = {}
    for _, row in df.iterrows():
        m[int(row[itc])] = float(row["total_balls"]) if pd.notna(row["total_balls"]) else np.nan
    return m


# =========================================================
# DATASET: weights_history.csv ONLY (tag executed preferred)
# Format you showed:
#   iter, timestamp, tag, w0..wN
# =========================================================
def build_dataset_from_weights(items, prefer_tag="executed"):
    X_all = []
    meta_all = []

    per_exp_counts = []

    for it in items:
        w_path = it["w_csv"]
        llm_path = it["llm_csv"]

        if w_path is None or not os.path.isfile(w_path):
            per_exp_counts.append((it["run_name"], it["exp_num"], 0))
            continue

        wdf = pd.read_csv(w_path)
        itc = find_iter_col(wdf)
        if itc is None:
            per_exp_counts.append((it["run_name"], it["exp_num"], 0))
            continue

        if "tag" not in wdf.columns:
            per_exp_counts.append((it["run_name"], it["exp_num"], 0))
            continue

        # weight columns
        w_cols = [c for c in wdf.columns if c.lower().startswith("w")]
        if not w_cols:
            per_exp_counts.append((it["run_name"], it["exp_num"], 0))
            continue

        # numeric conversions
        wdf[itc] = pd.to_numeric(wdf[itc], errors="coerce").astype("Int64")
        wdf["tag"] = wdf["tag"].astype(str).str.lower()
        for c in w_cols:
            wdf[c] = pd.to_numeric(wdf[c], errors="coerce")

        wdf = wdf.dropna(subset=[itc])

        # pick one row per iteration:
        # prefer executed, else proposed
        chosen_rows = []
        for iter_val, g in wdf.groupby(itc):
            g_exec = g[g["tag"] == prefer_tag].dropna(subset=w_cols)
            if len(g_exec) > 0:
                chosen_rows.append(g_exec.iloc[-1])
                continue
            g_prop = g[g["tag"] == "proposed"].dropna(subset=w_cols)
            if len(g_prop) > 0:
                chosen_rows.append(g_prop.iloc[-1])

        if not chosen_rows:
            per_exp_counts.append((it["run_name"], it["exp_num"], 0))
            continue

        balls_map = load_llm_balls_map(llm_path)

        added = 0
        for row in chosen_rows:
            iter_int = int(row[itc])
            vec = row[w_cols].to_numpy(dtype=float)

            # skip if any nan
            if np.any(~np.isfinite(vec)):
                continue

            X_all.append(vec)
            meta_all.append({
                "run_name": it["run_name"],
                "exp_num": it["exp_num"],
                "iter": iter_int,
                "total_balls": balls_map.get(iter_int, np.nan),
                "dmp_csv": it["dmp_csv"],
                "ee_csv": it["ee_csv"],
            })
            added += 1

        per_exp_counts.append((it["run_name"], it["exp_num"], added))

    print("\n[INFO] Weight vectors extracted per experiment:")
    for rn, en, cnt in per_exp_counts:
        print(f"  - {rn} exp{en}: {cnt}")

    if len(X_all) < 2:
        return None, None

    X = np.vstack(X_all)
    return X, meta_all


# =========================================================
# SOM utilities + plots
# =========================================================
def choose_grid(n_samples):
    # Similar heuristic as before (10..30)
    g = int(np.sqrt(np.sqrt(n_samples)) * 10)
    return max(10, min(g, 30))


def save_umatrix(som, out_png):
    um = som.distance_map()
    plt.figure(figsize=(7, 6))
    plt.imshow(um.T, origin="lower")
    plt.colorbar()
    plt.title("SOM U-Matrix (weights)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()


def save_hitmap(grid, winners, out_png):
    counts = np.zeros((grid, grid), dtype=int)
    for (x, y) in winners:
        counts[x, y] += 1
    plt.figure(figsize=(7, 6))
    plt.imshow(counts.T, origin="lower")
    plt.colorbar()
    plt.title("SOM Hit Map (weights)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()
    return counts


def save_median_balls_map(grid, winners, meta_all, out_png):
    cell_vals = defaultdict(list)
    for (x, y), m in zip(winners, meta_all):
        b = m.get("total_balls", np.nan)
        if np.isfinite(b):
            cell_vals[(x, y)].append(float(b))

    med = np.full((grid, grid), np.nan, dtype=float)
    for (x, y), vals in cell_vals.items():
        med[x, y] = float(np.median(vals))

    plt.figure(figsize=(7, 6))
    plt.imshow(med.T, origin="lower")
    plt.colorbar()
    plt.title("SOM Median total_balls per cell")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()

    return med


# =========================================================
# Trajectory overlay plots per (run,exp,iter)
# (kept same: DMP+EE on one plot, green)
# =========================================================
def load_iter_xy(csv_path, iteration):
    if csv_path is None or not os.path.isfile(csv_path):
        return None
    df = pd.read_csv(csv_path)
    itc = find_iter_col(df)
    if itc is None:
        return None
    xcol, ycol = find_xy_cols(df)
    if xcol is None:
        return None

    df[itc] = pd.to_numeric(df[itc], errors="coerce").astype("Int64")
    df[xcol] = pd.to_numeric(df[xcol], errors="coerce")
    df[ycol] = pd.to_numeric(df[ycol], errors="coerce")
    df = df.dropna(subset=[itc, xcol, ycol])

    g = df[df[itc].astype(int) == int(iteration)]
    if len(g) < 2:
        return None
    return g[[xcol, ycol]].to_numpy(dtype=float)


def plot_overlay(dmp_csv, ee_csv, iteration, out_png, title):
    dmp = load_iter_xy(dmp_csv, iteration)
    ee = load_iter_xy(ee_csv, iteration)

    if dmp is None and ee is None:
        return False

    plt.figure(figsize=(7.5, 6.5))
    if dmp is not None:
        plt.plot(dmp[:, 0], dmp[:, 1], color="green", lw=2.4, alpha=0.85, label="DMP")
    if ee is not None:
        plt.plot(ee[:, 0], ee[:, 1], color="green", lw=2.4, alpha=0.35, label="EE")

    plt.title(title)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()
    return True


# =========================================================
# MAIN
# =========================================================
def main():
    print("\n==================== GLOBAL SOM CLUSTERING (WEIGHTS ONLY) ====================")
    print("[INFO] OUTPUT_DIR:", OUTPUT_DIR)
    print("[INFO] RESULT_DIR:", RESULT_DIR)
    print("[INFO] OUTPUT_DIR exists:", os.path.isdir(OUTPUT_DIR))

    items = discover_all_experiments(OUTPUT_DIR)
    if not items:
        raise RuntimeError("No experiments found under output/<run>/<exp>/")

    # 1) Ranking experiments best->worst
    rank_rows = []
    for it in items:
        rank_rows.append({
            "run_name": it["run_name"],
            "exp_num": it["exp_num"],
            "best_total_balls": experiment_best_ball(it["llm_csv"]),
            "exp_dir": it["exp_dir"]
        })

    rank_df = pd.DataFrame(rank_rows).dropna(subset=["best_total_balls"])
    rank_df = rank_df.sort_values("best_total_balls", ascending=True)
    rank_csv = os.path.join(RESULT_DIR, "ranking_best_to_worst.csv")
    rank_df.to_csv(rank_csv, index=False)
    print("\n[RANK] Saved:", rank_csv)

    # 2) Build dataset from weights_history.csv only
    X_all, meta_all = build_dataset_from_weights(items, prefer_tag="executed")
    if X_all is None or X_all.shape[0] < 2:
        raise RuntimeError("Not enough weight vectors for SOM after parsing weights_history.csv.")

    Xn = zscore(X_all)

    # 3) Train SOM
    grid = choose_grid(Xn.shape[0])
    som = MiniSom(grid, grid, Xn.shape[1], sigma=SOM_SIGMA, learning_rate=SOM_LR, random_seed=RANDOM_SEED)
    som.random_weights_init(Xn)

    print(f"\n[SOM] Training SOM {grid}x{grid} on {Xn.shape[0]} weight vectors...")
    som.train_random(Xn, SOM_ITERS)

    winners = [som.winner(v) for v in Xn]

    # 4) Save SOM plots
    plots_dir = os.path.join(RESULT_DIR, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    um_png = os.path.join(plots_dir, "som_umatrix.png")
    hit_png = os.path.join(plots_dir, "som_hitmap.png")
    med_png = os.path.join(plots_dir, "som_median_balls_map.png")

    save_umatrix(som, um_png)
    save_hitmap(grid, winners, hit_png)
    save_median_balls_map(grid, winners, meta_all, med_png)

    print("[SOM] Saved plots:")
    print("  -", um_png)
    print("  -", hit_png)
    print("  -", med_png)

    # 5) Assignments CSV
    assign_rows = []
    for m, (cx, cy) in zip(meta_all, winners):
        assign_rows.append({
            "label": safe_label(m["run_name"], m["exp_num"], m["iter"]),
            "run_name": m["run_name"],
            "exp_num": m["exp_num"],
            "iter": m["iter"],
            "total_balls": m.get("total_balls", np.nan),
            "cell_x": cx,
            "cell_y": cy,
        })

    assign_df = pd.DataFrame(assign_rows)
    assign_csv = os.path.join(RESULT_DIR, "som_assignments_all.csv")
    assign_df.to_csv(assign_csv, index=False)
    print("[SOM] Saved assignments:", assign_csv)

    # 6) Cluster summary CSV
    summary_rows = []
    for (cx, cy), g in assign_df.groupby(["cell_x", "cell_y"]):
        balls = pd.to_numeric(g["total_balls"], errors="coerce").dropna()
        summary_rows.append({
            "cell_x": cx,
            "cell_y": cy,
            "count": int(len(g)),
            "median_balls": float(np.median(balls)) if len(balls) else np.nan,
            "best_balls": float(np.min(balls)) if len(balls) else np.nan,
            "worst_balls": float(np.max(balls)) if len(balls) else np.nan,
        })

    summary_df = pd.DataFrame(summary_rows).sort_values(["median_balls", "count"], ascending=[True, False])
    summary_csv = os.path.join(RESULT_DIR, "som_cluster_summary_all.csv")
    summary_df.to_csv(summary_csv, index=False)
    print("[SOM] Saved cluster summary:", summary_csv)

    # 7) Export per-cell examples (trajectory overlay plots)
    clusters_root = os.path.join(RESULT_DIR, "clusters")
    os.makedirs(clusters_root, exist_ok=True)

    # Build meta lookup for plotting
    meta_lookup = {(m["run_name"], int(m["exp_num"]), int(m["iter"])): m for m in meta_all}

    print("\n[EXPORT] Saving overlay plots inside each cell folder...")
    for (cx, cy), g in assign_df.groupby(["cell_x", "cell_y"]):
        cell_dir = os.path.join(clusters_root, f"cell_{cx}_{cy}")
        os.makedirs(cell_dir, exist_ok=True)

        # Save full members list
        g_sorted = g.sort_values("total_balls", ascending=True)
        g_sorted.to_csv(os.path.join(cell_dir, "members.csv"), index=False)

        # Plot top K
        topk = g_sorted.head(TOPK_PER_CELL)
        for _, r in topk.iterrows():
            key = (r["run_name"], int(r["exp_num"]), int(r["iter"]))
            m = meta_lookup.get(key)
            if m is None:
                continue

            out_png = os.path.join(cell_dir, f"{r['label']}.png")
            plot_overlay(
                dmp_csv=m.get("dmp_csv"),
                ee_csv=m.get("ee_csv"),
                iteration=int(r["iter"]),
                out_png=out_png,
                title=r["label"]
            )

    print("\n✅ DONE.")
    print("Best->worst:", rank_csv)
    print("All results saved under:", RESULT_DIR)


if __name__ == "__main__":
    main()

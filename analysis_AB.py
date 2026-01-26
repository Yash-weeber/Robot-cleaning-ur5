import os
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from minisom import MiniSom


# =========================================================
# CONFIG (DYNAMIC)
# Put this script next to your "output" folder
# =========================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")   # filtered outputs live here
RESULT_DIR = os.path.join(SCRIPT_DIR, "result")   # save SOM results here
os.makedirs(RESULT_DIR, exist_ok=True)

# SOM params
SOM_SIGMA = 1.5
SOM_LR = 0.5
SOM_ITERS = 5000

# How many best examples to export per SOM cell
TOPK_PER_CELL = 10

# Downsample each iteration trajectory for feature building (speed + stability)
RESAMPLE_N = 200

# Z-score normalize features before SOM
EPS = 1e-9

# =========================================================
# CSV helpers
# =========================================================
def list_csvs(folder_path):
    if not os.path.isdir(folder_path):
        return []
    return [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.lower().endswith(".csv")
    ]

def pick_file(csv_paths, keyword):
    keyword = keyword.lower()
    for p in csv_paths:
        if keyword in os.path.basename(p).lower():
            return p
    return None

def find_iter_col(df):
    for c in ["iter", "iteration", "Iteration", "ITER", "step", "idx", "step_idx"]:
        if c in df.columns:
            return c
    raise ValueError(f"Iteration column not found. Columns: {list(df.columns)}")

def find_xy_cols(df):
    for xcol, ycol in [("x", "y"), ("X", "Y")]:
        if xcol in df.columns and ycol in df.columns:
            return xcol, ycol
    raise ValueError(f"x/y columns not found. Columns: {list(df.columns)}")

def zscore(X):
    mu = np.nanmean(X, axis=0)
    sd = np.nanstd(X, axis=0) + EPS
    return (X - mu) / sd

def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p


# =========================================================
# Discovery: output/<run>/<exp>/*.csv
# =========================================================
def discover_all_experiments(output_dir):
    items = []
    if not os.path.isdir(output_dir):
        return items

    run_names = [d for d in os.listdir(output_dir) if os.path.isdir(os.path.join(output_dir, d))]
    run_names = sorted(run_names)

    for run in run_names:
        run_path = os.path.join(output_dir, run)
        exp_nums = [d for d in os.listdir(run_path)
                    if os.path.isdir(os.path.join(run_path, d)) and d.isdigit()]
        exp_nums = sorted(exp_nums, key=lambda x: int(x))

        for exp in exp_nums:
            exp_path = os.path.join(run_path, exp)
            csvs = list_csvs(exp_path)

            llm = pick_file(csvs, "llm_iteration_log")
            dmp = pick_file(csvs, "dmp_trajectory_feedback")
            ee  = pick_file(csvs, "ee_trajectory")

            if (dmp is None) and (ee is None):
                continue

            items.append({
                "run": run,
                "exp_num": int(exp),
                "exp_path": exp_path,
                "llm_csv": llm,
                "dmp_csv": dmp,
                "ee_csv": ee,
            })

    return items


# =========================================================
# Trajectory utilities
# =========================================================
def resample_xy(xy: np.ndarray, n: int) -> np.ndarray:
    """Arc-length resample to n points."""
    if xy.shape[0] < 2:
        return np.repeat(xy[:1], n, axis=0)

    d = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))
    s = np.insert(np.cumsum(d), 0, 0.0)
    if s[-1] <= 0:
        return np.repeat(xy[:1], n, axis=0)

    t_new = np.linspace(0, s[-1], n)
    x_new = np.interp(t_new, s, xy[:, 0])
    y_new = np.interp(t_new, s, xy[:, 1])
    return np.column_stack([x_new, y_new])

def traj_features(xy: np.ndarray) -> dict:
    """
    Simple strong features for shape clustering:
      - bbox (min/max/width/height)
      - path length
      - mean abs heading change (smooth_turn)
    """
    if xy.shape[0] < 6:
        return None

    xs = xy[:, 0]
    ys = xy[:, 1]
    x_min, x_max = float(xs.min()), float(xs.max())
    y_min, y_max = float(ys.min()), float(ys.max())
    width = x_max - x_min
    height = y_max - y_min

    dx = np.diff(xs)
    dy = np.diff(ys)
    seg = np.sqrt(dx*dx + dy*dy)
    path_len = float(np.sum(seg))

    heading = np.arctan2(dy, dx)
    dhead = np.diff(heading)
    dhead = (dhead + np.pi) % (2*np.pi) - np.pi
    smooth_turn = float(np.mean(np.abs(dhead))) if len(dhead) else 0.0

    return {
        "x_min": x_min, "x_max": x_max,
        "y_min": y_min, "y_max": y_max,
        "width": width, "height": height,
        "path_len": path_len,
        "smooth_turn": smooth_turn
    }

def load_iter_map_xy(csv_path: str):
    """
    Returns dict: iter -> (N,2) raw xy array
    """
    if csv_path is None or (not os.path.isfile(csv_path)):
        return {}

    df = pd.read_csv(csv_path)
    itc = find_iter_col(df)
    xcol, ycol = find_xy_cols(df)

    df[itc] = pd.to_numeric(df[itc], errors="coerce").astype("Int64")
    df[xcol] = pd.to_numeric(df[xcol], errors="coerce")
    df[ycol] = pd.to_numeric(df[ycol], errors="coerce")
    df = df.dropna(subset=[itc, xcol, ycol])

    out = {}
    for it, g in df.groupby(itc):
        out[int(it)] = g[[xcol, ycol]].to_numpy(dtype=float)
    return out


# =========================================================
# Build GLOBAL dataset: one row per iteration
# (Uses BOTH DMP and EE features if available)
# =========================================================
def load_llm_balls_map(llm_csv):
    if llm_csv is None or (not os.path.isfile(llm_csv)):
        return {}

    llm = pd.read_csv(llm_csv)
    if "total_balls" not in llm.columns:
        return {}

    itc = find_iter_col(llm)
    llm[itc] = pd.to_numeric(llm[itc], errors="coerce").astype("Int64")
    llm["total_balls"] = pd.to_numeric(llm["total_balls"], errors="coerce")
    llm = llm.dropna(subset=[itc])

    return dict(zip(llm[itc].astype(int), llm["total_balls"]))

def build_global_rows(items):
    """
    Returns:
      X_all: (M,D) feature matrix
      meta_all: list[dict] with run, exp, iteration, total_balls
      sources: list[dict] with dmp_csv/ee_csv paths for plotting
      feat_names: list[str]
    """
    rows = []
    meta_all = []
    sources = []

    feat_names = [
        # DMP features
        "dmp_x_min","dmp_x_max","dmp_y_min","dmp_y_max","dmp_width","dmp_height","dmp_path_len","dmp_smooth_turn",
        # EE features
        "ee_x_min","ee_x_max","ee_y_min","ee_y_max","ee_width","ee_height","ee_path_len","ee_smooth_turn",
        # DMP-EE mismatch
        "dmp_ee_mean_dist"
    ]

    for it in items:
        balls_map = load_llm_balls_map(it["llm_csv"])

        dmp_map = load_iter_map_xy(it["dmp_csv"])
        ee_map  = load_iter_map_xy(it["ee_csv"])

        all_iters = sorted(set(dmp_map.keys()) | set(ee_map.keys()))
        if not all_iters:
            continue

        for iteration in all_iters:
            dmp_xy = dmp_map.get(iteration, None)
            ee_xy  = ee_map.get(iteration, None)

            # resample for consistent feature calc
            dmp_rs = resample_xy(dmp_xy, RESAMPLE_N) if (dmp_xy is not None and len(dmp_xy) > 1) else None
            ee_rs  = resample_xy(ee_xy, RESAMPLE_N)  if (ee_xy  is not None and len(ee_xy)  > 1) else None

            dmp_f = traj_features(dmp_rs) if dmp_rs is not None else None
            ee_f  = traj_features(ee_rs)  if ee_rs  is not None else None

            # require at least one trajectory to be usable
            if (dmp_f is None) and (ee_f is None):
                continue

            # fill missing side with NaNs
            def pack(prefix, f):
                if f is None:
                    return [np.nan]*8
                return [
                    f["x_min"], f["x_max"], f["y_min"], f["y_max"],
                    f["width"], f["height"], f["path_len"], f["smooth_turn"]
                ]

            dmp_vec = pack("dmp", dmp_f)
            ee_vec  = pack("ee", ee_f)

            # DMP-EE mismatch (mean pointwise dist after resample)
            if dmp_rs is not None and ee_rs is not None:
                dist = np.sqrt(np.sum((dmp_rs - ee_rs)**2, axis=1))
                mean_dist = float(np.mean(dist))
            else:
                mean_dist = np.nan

            vec = dmp_vec + ee_vec + [mean_dist]
            rows.append(vec)

            meta_all.append({
                "run": it["run"],
                "exp": f"exp{it['exp_num']}",
                "iteration": int(iteration),
                "total_balls": float(balls_map.get(int(iteration), np.nan))
            })
            sources.append(it)

    if not rows:
        return None, [], [], feat_names

    X_all = np.array(rows, dtype=float)
    return X_all, meta_all, sources, feat_names


# =========================================================
# SOM: train + graphs
# =========================================================
def choose_grid(n_samples: int) -> int:
    # simple heuristic: bigger data => bigger grid
    g = int(np.sqrt(np.sqrt(max(n_samples, 1))) * 10)
    return max(10, min(g, 30))

def train_som(X_all):
    Xn = zscore(X_all)
    grid = choose_grid(Xn.shape[0])

    som = MiniSom(x=grid, y=grid, input_len=Xn.shape[1], sigma=1.5, learning_rate=0.5)
    som.random_weights_init(Xn)
    som.train_random(Xn, num_iteration=SOM_ITERS)

    winners = [som.winner(v) for v in Xn]
    return som, winners, grid, Xn

def save_umatrix(som, out_png):
    um = som.distance_map()
    plt.figure(figsize=(7,6))
    plt.imshow(um.T, origin="lower")
    plt.colorbar()
    plt.title("SOM U-Matrix (distance map)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()

def save_hitmap(grid, winners, out_png):
    counts = np.zeros((grid, grid), dtype=int)
    for (x, y) in winners:
        counts[x, y] += 1
    plt.figure(figsize=(7,6))
    plt.imshow(counts.T, origin="lower")
    plt.colorbar()
    plt.title("SOM Hit Map (samples per cell)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()
    return counts

def save_median_balls_map(grid, winners, meta_all, out_png):
    cell_vals = defaultdict(list)
    for w, m in zip(winners, meta_all):
        b = m.get("total_balls", np.nan)
        if np.isfinite(b):
            cell_vals[w].append(b)

    med = np.full((grid, grid), np.nan, dtype=float)
    for (x, y), vals in cell_vals.items():
        med[x, y] = float(np.median(vals))

    plt.figure(figsize=(7,6))
    plt.imshow(med.T, origin="lower")
    plt.colorbar()
    plt.title("SOM Median total_balls per cell")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()
    return med


# =========================================================
# Plot overlay (DMP + EE in same plot) for a single iteration
# =========================================================
def plot_overlay(dmp_csv, ee_csv, iteration, out_png, title):
    dmp_map = load_iter_map_xy(dmp_csv)
    ee_map  = load_iter_map_xy(ee_csv)

    dmp_xy = dmp_map.get(int(iteration), None)
    ee_xy  = ee_map.get(int(iteration), None)

    if dmp_xy is None and ee_xy is None:
        return False

    plt.figure(figsize=(7.5, 6.5))
    if dmp_xy is not None and len(dmp_xy) > 1:
        plt.plot(dmp_xy[:, 0], dmp_xy[:, 1], color="green", linewidth=2.2, alpha=0.85, label="DMP")
    if ee_xy is not None and len(ee_xy) > 1:
        plt.plot(ee_xy[:, 0], ee_xy[:, 1], color="green", linewidth=2.2, alpha=0.35, label="EE")

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
    print("\n==================== GLOBAL SOM CLUSTERING ====================")
    print("[INFO] OUTPUT_DIR:", OUTPUT_DIR)
    print("[INFO] RESULT_DIR:", RESULT_DIR)
    print("[INFO] OUTPUT_DIR exists:", os.path.isdir(OUTPUT_DIR))

    if not os.path.isdir(OUTPUT_DIR):
        raise RuntimeError(f"Filtered output folder not found: {OUTPUT_DIR}")

    items = discover_all_experiments(OUTPUT_DIR)
    if not items:
        raise RuntimeError(f"No experiments found under: {OUTPUT_DIR}")

    X_all, meta_all, sources, feat_names = build_global_rows(items)
    if X_all is None or X_all.shape[0] < 2:
        raise RuntimeError("Not enough iterations globally to cluster. Check that filtered CSVs contain iterations.")

    print(f"[INFO] Total iterations (global samples): {X_all.shape[0]}")
    print(f"[INFO] Feature dim confirm: {X_all.shape[1]}")

    som, winners, grid, Xn = train_som(X_all)
    print(f"[INFO] Trained SOM grid: {grid} x {grid}")

    out_root = ensure_dir(os.path.join(RESULT_DIR, "global"))
    plots_dir = ensure_dir(os.path.join(out_root, "plots"))
    clusters_dir = ensure_dir(os.path.join(out_root, "clusters"))

    # --- Global plots ---
    save_umatrix(som, os.path.join(plots_dir, "som_umatrix.png"))
    hit_counts = save_hitmap(grid, winners, os.path.join(plots_dir, "som_hitmap.png"))
    med_map = save_median_balls_map(grid, winners, meta_all, os.path.join(plots_dir, "som_median_balls_map.png"))

    # --- Assignments ---
    assign_rows = []
    for m, w in zip(meta_all, winners):
        # naming convention you asked:
        # "<run> exp3 iter 21"
        name = f"{m['run']} {m['exp']} iter {m['iteration']}"
        assign_rows.append({
            "name": name,
            "run": m["run"],
            "exp": m["exp"],
            "iteration": m["iteration"],
            "total_balls": m.get("total_balls", np.nan),
            "cell_x": w[0],
            "cell_y": w[1],
        })

    assign_df = pd.DataFrame(assign_rows)
    assign_df.to_csv(os.path.join(out_root, "som_assignments_all.csv"), index=False)

    # --- Cluster summary per cell ---
    clusters = defaultdict(list)
    for row in assign_rows:
        clusters[(row["cell_x"], row["cell_y"])].append(row)

    summ = []
    for (cx, cy), members in clusters.items():
        balls = np.array([r["total_balls"] for r in members], dtype=float)
        summ.append({
            "cell_x": cx,
            "cell_y": cy,
            "count": len(members),
            "median_balls": float(np.nanmedian(balls)) if np.isfinite(balls).any() else np.nan,
            "best_balls": float(np.nanmin(balls)) if np.isfinite(balls).any() else np.nan,
            "worst_balls": float(np.nanmax(balls)) if np.isfinite(balls).any() else np.nan,
        })

    summ_df = pd.DataFrame(summ).sort_values(
        ["median_balls", "best_balls", "count"],
        ascending=[True, True, False]
    )
    summ_df.to_csv(os.path.join(out_root, "som_cluster_summary_all.csv"), index=False)

    # --- Export top-K plots per SOM cell (best balls first) ---
    print("\n[INFO] Saving top trajectories per SOM cell...")
    for (cx, cy), members in clusters.items():
        # sort by balls (NaN last)
        members_sorted = sorted(
            members,
            key=lambda r: (np.inf if not np.isfinite(r["total_balls"]) else r["total_balls"])
        )
        top = members_sorted[:TOPK_PER_CELL]
        if not top:
            continue

        cell_dir = ensure_dir(os.path.join(clusters_dir, f"cell_{cx}_{cy}"))

        for r in top:
            # locate the correct source exp to load its dmp/ee CSVs
            run = r["run"]
            exp_num = int(str(r["exp"]).replace("exp", ""))
            src = None
            for it in items:
                if it["run"] == run and it["exp_num"] == exp_num:
                    src = it
                    break
            if src is None:
                continue

            fname = f"{r['run']} {r['exp']} iter {r['iteration']}.png"
            out_png = os.path.join(cell_dir, fname)

            title = f"{r['run']} {r['exp']} iter {r['iteration']} | cell({cx},{cy}) | balls={r['total_balls']}"
            ok = plot_overlay(src["dmp_csv"], src["ee_csv"], r["iteration"], out_png, title)
            if not ok:
                continue

    print("\n✅ DONE.")
    print("Saved GLOBAL SOM results here:")
    print(out_root)
    print("\nKey outputs:")
    print("  - plots/som_umatrix.png")
    print("  - plots/som_hitmap.png")
    print("  - plots/som_median_balls_map.png")
    print("  - som_assignments_all.csv")
    print("  - som_cluster_summary_all.csv")
    print("  - clusters/cell_x_y/<run exp iter>.png")


if __name__ == "__main__":
    main()

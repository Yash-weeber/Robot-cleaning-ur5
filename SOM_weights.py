import os
import re
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from minisom import MiniSom

from config import load_config


# =========================================================
# CONFIG
# =========================================================
config = load_config("./config/semantics-gridcoverage-hist-30.yaml")
SCRIPT_DIR = config["simulation"]["base_dir"]

# Expected: <OUTPUT_DIR>/<run_type>/<run_name>/<exp_num>/*.csv
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "SOM_Results", "filtered_results-2")
RESULT_DIR = os.path.join(SCRIPT_DIR, "SOM_Results", "Summary-weights-2")
os.makedirs(RESULT_DIR, exist_ok=True)

SOM_SIGMA = 1.5
SOM_LR = 0.5
SOM_ITERS = 5000
TOPK_PER_CELL = 10
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
# Discovery: output/<run_type>/<run_name>/<exp>/*.csv
# =========================================================
def discover_all_experiments(output_dir):
    items = []
    if not os.path.isdir(output_dir):
        return items

    run_types = sorted(
        d for d in os.listdir(output_dir)
        if os.path.isdir(os.path.join(output_dir, d))
    )

    for run_type in run_types:
        run_type_path = os.path.join(output_dir, run_type)
        run_names = sorted(
            d for d in os.listdir(run_type_path)
            if os.path.isdir(os.path.join(run_type_path, d))
        )

        for run_name in run_names:
            run_path = os.path.join(run_type_path, run_name)
            exp_nums = sorted(
                [
                    d for d in os.listdir(run_path)
                    if os.path.isdir(os.path.join(run_path, d)) and d.isdigit()
                ],
                key=lambda x: int(x),
            )

            for exp in exp_nums:
                exp_path = os.path.join(run_path, exp)
                csvs = list_csvs(exp_path)

                llm = pick_file(csvs, "llm_iteration_log")
                w_csv = pick_file(csvs, "dmp_weights_history") or pick_file(csvs, "weights_history")
                dmp = pick_file(csvs, "dmp_trajectory_feedback")
                ee = pick_file(csvs, "ee_trajectory")

                if w_csv is None:
                    continue

                items.append({
                    "run": f"{run_type}_{run_name}",
                    "run_type": run_type,
                    "run_name": run_name,
                    "exp_num": int(exp),
                    "exp_path": exp_path,
                    "llm_csv": llm,
                    "w_csv": w_csv,
                    "dmp_csv": dmp,  # optional, only for overlay export
                    "ee_csv": ee,    # optional, only for overlay export
                })

    return items


# =========================================================
# Optional overlay support (not used for SOM features)
# =========================================================
def load_iter_map_xy(csv_path: str):
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


def plot_overlay(dmp_csv, ee_csv, iteration, out_png, title):
    dmp_map = load_iter_map_xy(dmp_csv)
    ee_map = load_iter_map_xy(ee_csv)

    dmp_xy = dmp_map.get(int(iteration))
    ee_xy = ee_map.get(int(iteration))
    if dmp_xy is None and ee_xy is None:
        return False

    plt.figure(figsize=(6, 6))
    if dmp_xy is not None:
        plt.plot(dmp_xy[:, 0], dmp_xy[:, 1], label="DMP", linewidth=1.8)
    if ee_xy is not None:
        plt.plot(ee_xy[:, 0], ee_xy[:, 1], label="EE", linewidth=1.2, alpha=0.9)

    plt.title(title)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()
    return True


# =========================================================
# Build GLOBAL dataset: one row per iteration (FROM DMP WEIGHTS)
#   - prefer tag='executed'
#   - fallback tag='proposed'
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


def _weight_sort_key(col_name: str):
    c = str(col_name).lower()
    m = re.search(r"\d+", c)
    if m:
        return (0, int(m.group(0)))
    return (1, c)


def _find_weight_cols(df):
    # common: w0, w1, ..., or w_0, w_1, ...
    cols = [c for c in df.columns if str(c).lower().startswith("w")]
    return sorted(cols, key=_weight_sort_key)


def build_global_rows(items, prefer_tag="executed"):
    rows = []
    meta_all = []
    sources = []
    feat_names = None

    for it in items:
        w_csv = it.get("w_csv")
        if w_csv is None or (not os.path.isfile(w_csv)):
            continue

        try:
            wdf = pd.read_csv(w_csv)
            itc = find_iter_col(wdf)
        except Exception:
            continue

        local_w_cols = _find_weight_cols(wdf)
        if not local_w_cols:
            continue

        # lock schema from first valid file
        if feat_names is None:
            feat_names = list(local_w_cols)

        # strict schema consistency for SOM vectors
        if any(c not in wdf.columns for c in feat_names):
            print(f"[WARN] Skipping {w_csv}: missing expected weight columns.")
            continue

        wdf[itc] = pd.to_numeric(wdf[itc], errors="coerce").astype("Int64")
        for c in feat_names:
            wdf[c] = pd.to_numeric(wdf[c], errors="coerce")

        has_tag = "tag" in wdf.columns
        if has_tag:
            wdf["tag"] = wdf["tag"].astype(str).str.lower()

        wdf = wdf.dropna(subset=[itc])
        if wdf.empty:
            continue

        balls_map = load_llm_balls_map(it.get("llm_csv"))

        for iteration, g in wdf.groupby(itc):
            chosen = None

            if has_tag:
                g_exec = g[g["tag"] == str(prefer_tag).lower()].dropna(subset=feat_names)
                if len(g_exec) > 0:
                    chosen = g_exec.iloc[-1]
                else:
                    g_prop = g[g["tag"] == "proposed"].dropna(subset=feat_names)
                    if len(g_prop) > 0:
                        chosen = g_prop.iloc[-1]
            else:
                g_valid = g.dropna(subset=feat_names)
                if len(g_valid) > 0:
                    chosen = g_valid.iloc[-1]

            if chosen is None:
                continue

            vec = chosen[feat_names].to_numpy(dtype=float)
            if np.any(~np.isfinite(vec)):
                continue

            rows.append(vec)
            meta_all.append({
                "run": it["run"],
                "exp": f"exp{it['exp_num']}",
                "iteration": int(iteration),
                "total_balls": float(balls_map.get(int(iteration), np.nan)),
            })
            sources.append(it)

    if not rows:
        return None, [], [], (feat_names if feat_names is not None else [])

    X_all = np.vstack(rows).astype(float)
    return X_all, meta_all, sources, feat_names


# =========================================================
# SOM: train + graphs
# =========================================================
def choose_grid(n_samples: int) -> int:
    g = int(np.sqrt(np.sqrt(max(n_samples, 1))) * 10)
    return max(10, min(g, 30))


def train_som(X_all):
    Xn = zscore(X_all)
    grid = choose_grid(Xn.shape[0])

    som = MiniSom(
        x=grid,
        y=grid,
        input_len=Xn.shape[1],
        sigma=SOM_SIGMA,
        learning_rate=SOM_LR,
    )
    som.random_weights_init(Xn)
    som.train_random(Xn, num_iteration=SOM_ITERS)

    winners = [som.winner(v) for v in Xn]
    return som, winners, grid, Xn


def save_umatrix(som, out_png):
    um = som.distance_map()
    plt.figure(figsize=(7, 6))
    plt.imshow(um.T, origin="lower")
    plt.colorbar()
    plt.title("SOM U-Matrix (distance map)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()


def save_hitmap(grid, winners, out_png):
    hit = np.zeros((grid, grid), dtype=int)
    for x, y in winners:
        hit[x, y] += 1

    plt.figure(figsize=(7, 6))
    plt.imshow(hit.T, origin="lower")
    plt.colorbar()
    plt.title("SOM Hit Map")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()


def save_median_balls_map(grid, winners, meta_all, out_png):
    per_cell = defaultdict(list)
    for i, (x, y) in enumerate(winners):
        b = meta_all[i]["total_balls"]
        if np.isfinite(b):
            per_cell[(x, y)].append(float(b))

    med = np.full((grid, grid), np.nan, dtype=float)
    for (x, y), vals in per_cell.items():
        med[x, y] = float(np.median(vals))

    plt.figure(figsize=(7, 6))
    im = plt.imshow(med.T, origin="lower")
    plt.colorbar(im)
    plt.title("SOM Cell Median total_balls")
    plt.tight_layout()
    plt.savefig(out_png, dpi=220)
    plt.close()


# =========================================================
# MAIN
# =========================================================
def main():
    items = discover_all_experiments(OUTPUT_DIR)
    print(f"[INFO] discovered experiments with weight history: {len(items)}")

    X_all, meta_all, sources, feat_names = build_global_rows(items, prefer_tag="executed")
    if X_all is None or X_all.shape[0] < 2:
        raise RuntimeError(
            "Not enough weight-history iterations globally to cluster. "
            "Check dmp_weights_history/weights_history CSVs."
        )

    print(f"[INFO] global rows: {X_all.shape[0]}, feature dim: {X_all.shape[1]}")

    som, winners, grid, _ = train_som(X_all)
    print(f"[INFO] SOM trained: {grid}x{grid}")

    # Mirror SOM_Commented.py layout: RESULT_DIR/global/{plots,clusters}/
    out_root     = ensure_dir(os.path.join(RESULT_DIR, "global"))
    plots_dir    = ensure_dir(os.path.join(out_root, "plots"))
    clusters_dir = ensure_dir(os.path.join(out_root, "clusters"))

    save_umatrix(som, os.path.join(plots_dir, "som_umatrix.png"))
    save_hitmap(grid, winners, os.path.join(plots_dir, "som_hitmap.png"))
    save_median_balls_map(grid, winners, meta_all, os.path.join(plots_dir, "som_median_balls.png"))

    # Save assignment table
    assign_rows = []
    for i, (wx, wy) in enumerate(winners):
        m = meta_all[i]
        assign_rows.append({
            "name":        f"{m['run']} {m['exp']} iter {m['iteration']}",
            "run":         m["run"],
            "exp":         m["exp"],
            "iteration":   m["iteration"],
            "total_balls": m["total_balls"],
            "cell_x":      wx,
            "cell_y":      wy,
        })
    pd.DataFrame(assign_rows).to_csv(
        os.path.join(out_root, "som_assignments_all.csv"), index=False
    )

    # Per-cell summary statistics
    per_cell_idx = defaultdict(list)
    for i, w in enumerate(winners):
        per_cell_idx[w].append(i)

    summ = []
    for (cx, cy), idxs in per_cell_idx.items():
        balls = np.array([meta_all[i]["total_balls"] for i in idxs], dtype=float)
        summ.append({
            "cell_x":       cx,
            "cell_y":       cy,
            "count":        len(idxs),
            "median_balls": float(np.nanmedian(balls)) if np.isfinite(balls).any() else np.nan,
            "best_balls":   float(np.nanmin(balls))    if np.isfinite(balls).any() else np.nan,
            "worst_balls":  float(np.nanmax(balls))    if np.isfinite(balls).any() else np.nan,
        })

    pd.DataFrame(summ).sort_values(
        ["median_balls", "best_balls", "count"],
        ascending=[True, True, False]
    ).to_csv(os.path.join(out_root, "som_cluster_summary_all.csv"), index=False)

    # Top-K overlay plots per cell: clusters/cell_{cx}_{cy}/{run} {exp} iter {iteration}.png
    print("\n[INFO] Saving top trajectories per SOM cell...")

    for (cx, cy), idxs in per_cell_idx.items():
        idxs_sorted = sorted(
            idxs,
            key=lambda i: meta_all[i]["total_balls"] if np.isfinite(meta_all[i]["total_balls"]) else np.inf
        )[:TOPK_PER_CELL]

        cell_dir = ensure_dir(os.path.join(clusters_dir, f"cell_{cx}_{cy}"))

        for i in idxs_sorted:
            m = meta_all[i]
            s = sources[i]

            dmp_csv = s.get("dmp_csv")
            ee_csv  = s.get("ee_csv")

            # Filename matches SOM_Commented.py convention exactly
            fname   = f"{m['run']} {m['exp']} iter {m['iteration']}.png"
            out_png = os.path.join(cell_dir, fname)
            title   = (
                f"{m['run']} {m['exp']} iter {m['iteration']} "
                f"| cell({cx},{cy}) | balls={m['total_balls']}"
            )

            plot_overlay(dmp_csv, ee_csv, m["iteration"], out_png, title)

    print(f"\n✅ DONE. Results saved in: {out_root}")
    print("\nKey outputs:")
    print("  - plots/som_umatrix.png")
    print("  - plots/som_hitmap.png")
    print("  - plots/som_median_balls.png")
    print("  - som_assignments_all.csv")
    print("  - som_cluster_summary_all.csv")
    print("  - clusters/cell_x_y/<run exp iter>.png")


if __name__ == "__main__":
    main()
import os
import shutil
import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# CONFIG (DYNAMIC)
# Put this script in the directory that contains your run folders
# =========================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = SCRIPT_DIR
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Keep iterations with total_balls <= MAX_BALLS
MAX_BALLS = 99
CHUNK_SIZE = 250_000

# =========================================================
# STRICT BOUNDS (from enhancedll_Mohamed.py)
# =========================================================
STRICT_X_MIN = -1.050
STRICT_X_MAX =  1.050
STRICT_Y_MIN = -0.650
STRICT_Y_MAX =  0.650


# =========================================================
# HELPERS
# =========================================================
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


def is_experiment_folder(path):
    """
    Experiment folder = numeric folder that contains llm_iteration_log + trajectory files.
    """
    if not os.path.isdir(path):
        return False
    name = os.path.basename(path)
    if not name.isdigit():
        return False

    csvs = list_csvs(path)
    has_llm = pick_file(csvs, "llm_iteration_log") is not None
    has_dmp = pick_file(csvs, "dmp_trajectory_feedback") is not None
    has_ee  = pick_file(csvs, "ee_trajectory") is not None
    # weights is optional but expected
    return has_llm and (has_dmp or has_ee)


def find_run_folders(base_dir):
    """
    A "run folder" is any folder that (at any depth<=2) contains numeric experiment folders.
    We'll scan base_dir immediate subfolders and keep those that contain experiment folders.
    """
    run_folders = []
    for d in os.listdir(base_dir):
        p = os.path.join(base_dir, d)
        if not os.path.isdir(p):
            continue
        if d.lower() == "output":
            continue

        # check for numeric exp folders directly inside
        found_any = False
        for sub in os.listdir(p):
            subp = os.path.join(p, sub)
            if is_experiment_folder(subp):
                found_any = True
                break

        # also check one level deeper (common pattern)
        if not found_any:
            for sub in os.listdir(p):
                subp = os.path.join(p, sub)
                if not os.path.isdir(subp):
                    continue
                for sub2 in os.listdir(subp):
                    sub2p = os.path.join(subp, sub2)
                    if is_experiment_folder(sub2p):
                        found_any = True
                        break
                if found_any:
                    break

        if found_any:
            run_folders.append(d)

    return sorted(run_folders)


def find_experiments_in_run(run_path):
    """
    Return list of experiment folder paths (absolute) under a run folder (depth<=2).
    """
    exp_paths = []

    # depth 1
    for sub in os.listdir(run_path):
        subp = os.path.join(run_path, sub)
        if is_experiment_folder(subp):
            exp_paths.append(subp)

    # depth 2
    for sub in os.listdir(run_path):
        subp = os.path.join(run_path, sub)
        if not os.path.isdir(subp) or os.path.basename(subp).isdigit():
            continue
        for sub2 in os.listdir(subp):
            sub2p = os.path.join(subp, sub2)
            if is_experiment_folder(sub2p):
                exp_paths.append(sub2p)

    # sort by numeric experiment name
    exp_paths = sorted(exp_paths, key=lambda p: int(os.path.basename(p)))
    return exp_paths


# =========================================================
# FILTER RULES
# =========================================================
def read_bad_iters_from_llm(llm_path):
    llm = pd.read_csv(llm_path)
    if "total_balls" not in llm.columns:
        raise ValueError(f"'total_balls' missing in {llm_path}")

    iter_col = find_iter_col(llm)
    llm[iter_col] = pd.to_numeric(llm[iter_col], errors="coerce").astype("Int64")
    balls = pd.to_numeric(llm["total_balls"], errors="coerce")

    bad_mask = balls > MAX_BALLS
    bad_iters = set(llm.loc[bad_mask, iter_col].dropna().astype(int).unique().tolist())

    keep_mask = (~bad_mask) | balls.isna()
    llm_filtered = llm.loc[keep_mask].copy()

    return bad_iters, llm_filtered


def bad_iters_from_bounds(traj_path):
    """
    Mark iteration bad if ANY row has x or y outside strict bounds.
    """
    bad = set()
    first = True
    iter_col = None
    xcol = ycol = None

    for chunk in pd.read_csv(traj_path, chunksize=CHUNK_SIZE):
        if first:
            iter_col = find_iter_col(chunk)
            xcol, ycol = find_xy_cols(chunk)
            first = False

        chunk[iter_col] = pd.to_numeric(chunk[iter_col], errors="coerce").astype("Int64")
        xs = pd.to_numeric(chunk[xcol], errors="coerce")
        ys = pd.to_numeric(chunk[ycol], errors="coerce")

        oob = (
            (xs < STRICT_X_MIN) | (xs > STRICT_X_MAX) |
            (ys < STRICT_Y_MIN) | (ys > STRICT_Y_MAX)
        )

        if oob.any():
            bad.update(chunk.loc[oob, iter_col].dropna().astype(int).unique().tolist())

    return bad


def filter_small_csv_by_iters(in_path, out_path, bad_iters):
    df = pd.read_csv(in_path)
    iter_col = find_iter_col(df)
    df[iter_col] = pd.to_numeric(df[iter_col], errors="coerce").astype("Int64")

    before = len(df)
    df_f = df[~df[iter_col].isin(bad_iters)].copy()
    df_f.to_csv(out_path, index=False)
    return before, len(df_f)


def filter_large_csv_by_iters(in_path, out_path, bad_iters):
    first = True
    wrote_any = False
    total_in = 0
    total_out = 0

    for chunk in pd.read_csv(in_path, chunksize=CHUNK_SIZE):
        iter_col = find_iter_col(chunk)
        chunk[iter_col] = pd.to_numeric(chunk[iter_col], errors="coerce").astype("Int64")

        total_in += len(chunk)
        chunk_f = chunk[~chunk[iter_col].isin(bad_iters)].copy()
        total_out += len(chunk_f)

        if chunk_f.empty:
            continue

        chunk_f.to_csv(out_path, mode="w" if first else "a", header=first, index=False)
        first = False
        wrote_any = True

    if not wrote_any:
        pd.read_csv(in_path, nrows=0).to_csv(out_path, index=False)

    return total_in, total_out


# =========================================================
# PLOTTING (ONLY from FILTERED OUTPUT CSVs)
# =========================================================
def load_xy_by_iter(csv_path):
    df = pd.read_csv(csv_path)
    iter_col = find_iter_col(df)
    xcol, ycol = find_xy_cols(df)

    df[iter_col] = pd.to_numeric(df[iter_col], errors="coerce").astype("Int64")
    df[xcol] = pd.to_numeric(df[xcol], errors="coerce")
    df[ycol] = pd.to_numeric(df[ycol], errors="coerce")
    df = df.dropna(subset=[iter_col, xcol, ycol])

    out = {}
    for it, g in df.groupby(iter_col):
        out[int(it)] = g[[xcol, ycol]].to_numpy(dtype=float)
    return out


def save_overlay_plot_green(dmp_xy, ee_xy, it, save_path):
    """
    DMP + EE on SAME plot. Both green (EE lighter).
    """
    plt.figure(figsize=(7.5, 6.5))

    if dmp_xy is not None and len(dmp_xy) > 1:
        plt.plot(dmp_xy[:, 0], dmp_xy[:, 1],
                 color="green", linewidth=2.2, alpha=0.85, label="DMP")

    if ee_xy is not None and len(ee_xy) > 1:
        plt.plot(ee_xy[:, 0], ee_xy[:, 1],
                 color="green", linewidth=2.2, alpha=0.35, label="EE")

    # bounds box (visual check)
    plt.axvline(STRICT_X_MIN, linestyle="--", linewidth=1.0, alpha=0.35)
    plt.axvline(STRICT_X_MAX, linestyle="--", linewidth=1.0, alpha=0.35)
    plt.axhline(STRICT_Y_MIN, linestyle="--", linewidth=1.0, alpha=0.35)
    plt.axhline(STRICT_Y_MAX, linestyle="--", linewidth=1.0, alpha=0.35)

    plt.title(f"DMP + EE Overlay | iter {it}")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True, alpha=0.25)
    plt.legend()

    plt.text(
        0.02, 0.98, f"iter = {it}",
        transform=plt.gca().transAxes,
        va="top",
        fontsize=12,
        bbox=dict(boxstyle="round", alpha=0.2)
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=220)
    plt.close()


def make_iteration_plots_only_new(exp_out_dir, dmp_filtered_path, ee_filtered_path):
    """
    Deletes any old plot folder, then makes plots ONLY for iterations
    in the FILTERED CSVs.
    """
    plot_dir = os.path.join(exp_out_dir, "iter_plots")

    # delete old plots
    if os.path.isdir(plot_dir):
        shutil.rmtree(plot_dir)
    os.makedirs(plot_dir, exist_ok=True)

    dmp_map = load_xy_by_iter(dmp_filtered_path) if os.path.isfile(dmp_filtered_path) else {}
    ee_map  = load_xy_by_iter(ee_filtered_path)  if os.path.isfile(ee_filtered_path)  else {}

    iters = sorted(set(dmp_map.keys()) | set(ee_map.keys()))
    if not iters:
        print("    [PLOT] No iterations left after filtering; no plots generated.")
        return

    print(f"    [PLOT] Generating {len(iters)} NEW plots -> {plot_dir}")
    for it in iters:
        out_img = os.path.join(plot_dir, f"iter_{it}.png")
        save_overlay_plot_green(
            dmp_xy=dmp_map.get(it, None),
            ee_xy=ee_map.get(it, None),
            it=it,
            save_path=out_img
        )


# =========================================================
# MAIN: process all run folders -> all experiments
# =========================================================
print("=========================================================")
print("AUTO-DETECT run folders + experiments")
print(f"Base INPUT_DIR: {INPUT_DIR}")
print(f"OUTPUT_DIR:     {OUTPUT_DIR}")
print("Filtering rules:")
print(f"  1) Keep only iters where total_balls <= {MAX_BALLS} (from llm_iteration_log)")
print("  2) Keep only iters inside bounds in BOTH trajectories (bad in DMP or EE removed)")
print(f"     X ∈ [{STRICT_X_MIN}, {STRICT_X_MAX}], Y ∈ [{STRICT_Y_MIN}, {STRICT_Y_MAX}]")
print("=========================================================\n")

run_folders = find_run_folders(INPUT_DIR)
if not run_folders:
    raise RuntimeError(f"No run folders found under: {INPUT_DIR}")

print("[INFO] Found run folders:")
for rf in run_folders:
    print("  -", rf)
print()

for run_name in run_folders:
    run_path = os.path.join(INPUT_DIR, run_name)
    exp_paths = find_experiments_in_run(run_path)

    if not exp_paths:
        print(f"[SKIP] No experiments found in run folder: {run_name}")
        continue

    print(f"\n==================== RUN FOLDER: {run_name} ====================")
    print(f"[INFO] Experiments found: {len(exp_paths)}")

    for exp_path in exp_paths:
        exp_num = os.path.basename(exp_path)  # numeric folder name
        print(f"\n  -------- Experiment {exp_num} --------")
        csvs = list_csvs(exp_path)

        llm_in = pick_file(csvs, "llm_iteration_log")
        w_in   = pick_file(csvs, "weights_history")
        dmp_in = pick_file(csvs, "dmp_trajectory_feedback")
        ee_in  = pick_file(csvs, "ee_trajectory")

        if llm_in is None:
            print("  [SKIP] Missing llm_iteration_log*.csv")
            continue

        # output mirrors: output/<run_name>/<exp_num>/
        exp_out_dir = os.path.join(OUTPUT_DIR, run_name, exp_num)
        os.makedirs(exp_out_dir, exist_ok=True)

        llm_out = os.path.join(exp_out_dir, os.path.basename(llm_in))
        w_out   = os.path.join(exp_out_dir, os.path.basename(w_in)) if w_in else None
        dmp_out = os.path.join(exp_out_dir, os.path.basename(dmp_in)) if dmp_in else None
        ee_out  = os.path.join(exp_out_dir, os.path.basename(ee_in)) if ee_in else None

        # ---- filtering ----
        bad_balls, llm_filtered = read_bad_iters_from_llm(llm_in)
        bad_bounds_dmp = bad_iters_from_bounds(dmp_in) if dmp_in else set()
        bad_bounds_ee  = bad_iters_from_bounds(ee_in) if ee_in else set()
        bad_all = set(bad_balls) | set(bad_bounds_dmp) | set(bad_bounds_ee)

        print(f"  Bad iters by balls   (> {MAX_BALLS}): {len(bad_balls)}")
        print(f"  Bad iters by bounds  (DMP):           {len(bad_bounds_dmp)}")
        print(f"  Bad iters by bounds  (EE):            {len(bad_bounds_ee)}")
        print(f"  TOTAL bad iters removed:              {len(bad_all)}")

        # save llm filtered (also remove bound-bad iters)
        llm_df = llm_filtered.copy()
        llm_iter_col = find_iter_col(llm_df)
        llm_df[llm_iter_col] = pd.to_numeric(llm_df[llm_iter_col], errors="coerce").astype("Int64")
        llm_before = pd.read_csv(llm_in).shape[0]
        llm_df = llm_df[~llm_df[llm_iter_col].isin(set(bad_bounds_dmp) | set(bad_bounds_ee))].copy()
        llm_df.to_csv(llm_out, index=False)
        print(f"  LLM log rows:        {llm_before} -> {len(llm_df)}")

        # weights (small)
        if w_in and w_out:
            w_before, w_after = filter_small_csv_by_iters(w_in, w_out, bad_all)
            print(f"  Weights rows:        {w_before} -> {w_after}")
        else:
            print("  [WARN] weights_history*.csv not found")

        # dmp (large)
        if dmp_in and dmp_out:
            dmp_before, dmp_after = filter_large_csv_by_iters(dmp_in, dmp_out, bad_all)
            print(f"  DMP traj rows:       {dmp_before} -> {dmp_after}")
        else:
            print("  [WARN] dmp_trajectory_feedback*.csv not found")

        # ee (large)
        if ee_in and ee_out:
            ee_before, ee_after = filter_large_csv_by_iters(ee_in, ee_out, bad_all)
            print(f"  EE traj rows:        {ee_before} -> {ee_after}")
        else:
            print("  [WARN] ee_trajectory*.csv not found")

        # ---- plots ONLY for passing iters, from filtered CSVs ----
        if dmp_out and ee_out and os.path.isfile(dmp_out) and os.path.isfile(ee_out):
            make_iteration_plots_only_new(exp_out_dir, dmp_out, ee_out)
        else:
            print("    [PLOT] Skipping plots (missing filtered dmp/ee CSVs).")

        print(f"  Saved to: {exp_out_dir}")

print("\n✅ DONE — All runs processed. Output saved under:")
print(OUTPUT_DIR)

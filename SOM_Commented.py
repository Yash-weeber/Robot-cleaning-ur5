# Standard library imports for file/directory operations and data grouping
import os
from collections import defaultdict

# Numerical computing library for array and math operations
import numpy as np

# Data manipulation library for reading and processing CSV files
import pandas as pd

# Plotting library for generating and saving figures
import matplotlib.pyplot as plt

# Self-Organising Map library used for unsupervised trajectory clustering
from minisom import MiniSom

from config import load_config


# =========================================================
# CONFIG (DYNAMIC)
# Put this script next to your "output" folder
# =========================================================

# Absolute path to the directory containing this script
# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
config = load_config("./config/semantics-gridcoverage-hist-30.yaml") 
SCRIPT_DIR = config["simulation"]["base_dir"]

# Path to the filtered output folder produced by sorttinggood_AB.py
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "SOM_Results", "filtered_results")  # Output will be saved in a subfolder to keep things organized

# Path to the directory where SOM results and plots will be saved
RESULT_DIR = os.path.join(SCRIPT_DIR, "SOM_Results", "Summary-1")

# Create the result directory if it does not already exist
os.makedirs(RESULT_DIR, exist_ok=True)

# SOM neighbourhood radius; controls how far influence spreads during training
SOM_SIGMA = 1.5

# SOM learning rate; controls how much weights are updated per step
SOM_LR = 0.5

# Total number of random training iterations for the SOM
SOM_ITERS = 5000

# Number of top-scoring (lowest ball count) examples to export per SOM cell
TOPK_PER_CELL = 10

# Number of points each trajectory is resampled to before feature extraction
RESAMPLE_N = 200

# Small constant added to standard deviation to prevent division by zero in z-score
EPS = 1e-9


# =========================================================
# CSV helpers
# =========================================================

def list_csvs(folder_path):
    """
    Return a list of absolute paths to all CSV files in the given folder.

    Returns an empty list if the path does not point to a valid directory.

    Args:
        folder_path (str): Path to the directory to scan.

    Returns:
        list[str]: Absolute paths to all '.csv' files in the folder.
    """
    # Return an empty list if the folder does not exist
    if not os.path.isdir(folder_path):
        return []
    # Build and return full paths for each CSV file found in the folder
    return [
        os.path.join(folder_path, f)       # Construct the full file path
        for f in os.listdir(folder_path)   # Iterate over all entries in the folder
        if f.lower().endswith(".csv")      # Keep only files with a .csv extension
    ]


def pick_file(csv_paths, keyword):
    """
    Return the first file path whose filename contains the given keyword.

    The comparison is case-insensitive. Returns None if no match is found.

    Args:
        csv_paths (list[str]): List of file paths to search through.
        keyword (str): Substring to look for in each filename.

    Returns:
        str or None: First matching file path, or None if no match exists.
    """
    # Normalise keyword to lowercase for case-insensitive matching
    keyword = keyword.lower()
    # Check each path and return the first one whose filename contains the keyword
    for p in csv_paths:
        if keyword in os.path.basename(p).lower():
            return p
    # No match was found
    return None


def find_iter_col(df):
    """
    Find and return the name of the iteration column in a DataFrame.

    Searches for several common column name variants. Raises ValueError if none found.

    Args:
        df (pd.DataFrame): DataFrame to inspect for an iteration column.

    Returns:
        str: Name of the iteration column found in the DataFrame.

    Raises:
        ValueError: If no recognised iteration column is present.
    """
    # Try each known variant of the iteration column name
    for c in ["iter", "iteration", "Iteration", "ITER", "step", "idx", "step_idx"]:
        if c in df.columns:
            return c  # Return the first matching column name
    # None of the expected names were found; raise an informative error
    raise ValueError(f"Iteration column not found. Columns: {list(df.columns)}")


def find_xy_cols(df):
    """
    Find and return the x and y coordinate column names in a DataFrame.

    Checks both lowercase and uppercase variants. Raises ValueError if none found.

    Args:
        df (pd.DataFrame): DataFrame to inspect for coordinate columns.

    Returns:
        tuple[str, str]: Tuple of (x_column_name, y_column_name).

    Raises:
        ValueError: If no recognised x/y column pair is present.
    """
    # Try both lowercase and uppercase x/y column pairs
    for xcol, ycol in [("x", "y"), ("X", "Y")]:
        if xcol in df.columns and ycol in df.columns:
            return xcol, ycol  # Return the first matching pair
    # No valid pair found; raise an informative error
    raise ValueError(f"x/y columns not found. Columns: {list(df.columns)}")


def zscore(X):
    """
    Apply z-score normalisation to a 2-D feature matrix column-wise.

    Each column is shifted by its mean and divided by its standard deviation.
    EPS is added to the standard deviation to prevent division by zero.

    Args:
        X (np.ndarray): (N, D) array of raw feature values.

    Returns:
        np.ndarray: (N, D) array of z-score normalised feature values.
    """
    # Compute the column-wise mean, ignoring NaN values
    mu = np.nanmean(X, axis=0)
    # Compute the column-wise standard deviation, ignoring NaN values; add EPS for safety
    sd = np.nanstd(X, axis=0) + EPS
    # Subtract the mean and divide by the standard deviation for each column
    return (X - mu) / sd


def ensure_dir(p):
    """
    Create a directory (and any missing parents) if it does not already exist.

    Args:
        p (str): Path to the directory to create.

    Returns:
        str: The same path that was passed in, for convenient chaining.
    """
    # Create the directory hierarchy; do nothing if it already exists
    os.makedirs(p, exist_ok=True)
    # Return the path so this function can be used inline
    return p


# =========================================================
# Discovery: output/<run>/<exp>/*.csv
# =========================================================

def discover_all_experiments(output_dir):
    """
    Scan filtered output and return metadata for every experiment.

    Expected layout:
      <output_dir>/<run_type>/<run_name>/<exp_num>/*.csv
    Example:
      filtered_results/n_warmup-20/semantics-RL-.../1/

    The returned "run" field includes run_type so names are unique across warmup groups.
    """
    items = []
    if not os.path.isdir(output_dir):
        return items

    # Level 1: run types (e.g., n_warmup-5, n_warmup-20)
    run_types = [
        d for d in os.listdir(output_dir)
        if os.path.isdir(os.path.join(output_dir, d))
    ]
    run_types = sorted(run_types)

    for run_type in run_types:
        run_type_path = os.path.join(output_dir, run_type)

        # Level 2: concrete run folders under each run_type
        run_names = [
            d for d in os.listdir(run_type_path)
            if os.path.isdir(os.path.join(run_type_path, d))
        ]
        run_names = sorted(run_names)

        for run_name in run_names:
            run_path = os.path.join(run_type_path, run_name)

            # Level 3: numeric experiment folders
            exp_nums = [
                d for d in os.listdir(run_path)
                if os.path.isdir(os.path.join(run_path, d)) and d.isdigit()
            ]
            exp_nums = sorted(exp_nums, key=lambda x: int(x))

            for exp in exp_nums:
                exp_path = os.path.join(run_path, exp)
                csvs = list_csvs(exp_path)

                llm = pick_file(csvs, "llm_iteration_log")
                dmp = pick_file(csvs, "dmp_trajectory_feedback")
                ee  = pick_file(csvs, "ee_trajectory")

                if (dmp is None) and (ee is None):
                    continue

                # Include run_type in run label (no "/" to avoid path issues in filenames)
                run_label = f"{run_type}__{run_name}"

                items.append({
                    "run": run_label,          # used everywhere downstream
                    "run_type": run_type,      # optional extra metadata
                    "run_name": run_name,      # optional extra metadata
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
    """
    Resample a 2-D trajectory to exactly n evenly-spaced points by arc length.

    Computes cumulative arc length along the trajectory and uses linear
    interpolation to place n points at equal arc-length intervals.
    Handles degenerate inputs (single point or zero total length) gracefully.

    Args:
        xy (np.ndarray): (M, 2) array of original [x, y] trajectory points.
        n  (int):        Target number of points after resampling.

    Returns:
        np.ndarray: (n, 2) array of resampled [x, y] points.
    """
    # A single point cannot be resampled; repeat it n times
    if xy.shape[0] < 2:
        return np.repeat(xy[:1], n, axis=0)

    # Compute Euclidean distance between consecutive points (segment lengths)
    d = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))

    # Build cumulative arc-length array, starting from 0
    s = np.insert(np.cumsum(d), 0, 0.0)

    # If total arc length is zero (all points identical), repeat the first point
    if s[-1] <= 0:
        return np.repeat(xy[:1], n, axis=0)

    # Create n evenly spaced parameter values from 0 to the total arc length
    t_new = np.linspace(0, s[-1], n)

    # Interpolate x and y independently at the new parameter values
    x_new = np.interp(t_new, s, xy[:, 0])
    y_new = np.interp(t_new, s, xy[:, 1])

    # Stack interpolated x and y into an (n, 2) array
    return np.column_stack([x_new, y_new])


def traj_features(xy: np.ndarray) -> dict:
    """
    Extract a compact set of shape-descriptive features from a 2-D trajectory.

    Computes bounding-box dimensions, total path length, and mean absolute
    heading change (smoothness proxy). Returns None for very short trajectories.

    Features returned:
        - x_min, x_max     : Spatial extent along x-axis.
        - y_min, y_max     : Spatial extent along y-axis.
        - width            : Bounding box width  (x_max - x_min).
        - height           : Bounding box height (y_max - y_min).
        - path_len         : Total arc length of the trajectory.
        - smooth_turn      : Mean absolute heading change between consecutive segments.

    Args:
        xy (np.ndarray): (N, 2) array of [x, y] trajectory points.

    Returns:
        dict or None: Feature dictionary, or None if the trajectory is too short (<6 points).
    """
    # Require at least 6 points for meaningful feature extraction
    if xy.shape[0] < 6:
        return None

    # Separate x and y coordinate arrays
    xs = xy[:, 0]
    ys = xy[:, 1]

    # Compute bounding-box extents along each axis
    x_min, x_max = float(xs.min()), float(xs.max())
    y_min, y_max = float(ys.min()), float(ys.max())

    # Bounding-box dimensions
    width  = x_max - x_min
    height = y_max - y_min

    # Compute consecutive segment displacement vectors
    dx = np.diff(xs)
    dy = np.diff(ys)

    # Compute Euclidean length of each segment
    seg = np.sqrt(dx*dx + dy*dy)

    # Total path length is the sum of all segment lengths
    path_len = float(np.sum(seg))

    # Compute heading angle of each segment in radians
    heading = np.arctan2(dy, dx)

    # Compute heading change between consecutive segments
    dhead = np.diff(heading)

    # Wrap heading changes to the range [-π, π] to avoid discontinuities
    dhead = (dhead + np.pi) % (2*np.pi) - np.pi

    # Mean absolute heading change; 0.0 for trajectories too short to compute
    smooth_turn = float(np.mean(np.abs(dhead))) if len(dhead) else 0.0

    return {
        "x_min": x_min, "x_max": x_max,          # Bounding box x extents
        "y_min": y_min, "y_max": y_max,          # Bounding box y extents
        "width": width, "height": height,        # Bounding box dimensions
        "path_len": path_len,                    # Total arc length
        "smooth_turn": smooth_turn               # Mean absolute heading change
    }


def load_iter_map_xy(csv_path: str):
    """
    Load a trajectory CSV and return a per-iteration dictionary of x/y arrays.

    Reads the CSV, converts columns to the correct types, drops invalid rows,
    and groups the remaining data by iteration ID.

    Args:
        csv_path (str or None): Absolute path to the trajectory CSV, or None.

    Returns:
        dict[int, np.ndarray]: Mapping from integer iteration ID to an (N, 2)
            NumPy array of [x, y] coordinate pairs. Empty dict if file is missing.
    """
    # Return empty dict if no path was provided or the file does not exist
    if csv_path is None or (not os.path.isfile(csv_path)):
        return {}

    # Load the entire trajectory CSV into a DataFrame
    df = pd.read_csv(csv_path)

    # Identify iteration and coordinate column names
    itc = find_iter_col(df)
    xcol, ycol = find_xy_cols(df)

    # Convert iteration column to nullable integer, coercing errors to NaN
    df[itc] = pd.to_numeric(df[itc], errors="coerce").astype("Int64")

    # Convert x coordinate column to numeric, coercing errors to NaN
    df[xcol] = pd.to_numeric(df[xcol], errors="coerce")

    # Convert y coordinate column to numeric, coercing errors to NaN
    df[ycol] = pd.to_numeric(df[ycol], errors="coerce")

    # Remove rows with NaN in any of the three key columns
    df = df.dropna(subset=[itc, xcol, ycol])

    # Build output dictionary: {iteration_id -> (N, 2) float array}
    out = {}
    for it, g in df.groupby(itc):                          # Group by iteration ID
        out[int(it)] = g[[xcol, ycol]].to_numpy(dtype=float)  # Store as float array
    return out


# =========================================================
# Build GLOBAL dataset: one row per iteration
# (Uses BOTH DMP and EE features if available)
# =========================================================

def load_llm_balls_map(llm_csv):
    """
    Load the LLM iteration log and return a mapping from iteration ID to total_balls.

    Reads the CSV, validates that the 'total_balls' column exists, and builds
    a dictionary keyed by integer iteration ID.

    Args:
        llm_csv (str or None): Absolute path to the llm_iteration_log CSV, or None.

    Returns:
        dict[int, float]: Mapping from integer iteration ID to its total_balls value.
            Returns an empty dict if the file is missing or lacks 'total_balls'.
    """
    # Return empty dict if no path was provided or the file does not exist
    if llm_csv is None or (not os.path.isfile(llm_csv)):
        return {}

    # Load the LLM iteration log CSV
    llm = pd.read_csv(llm_csv)

    # Return empty dict if the required 'total_balls' column is absent
    if "total_balls" not in llm.columns:
        return {}

    # Find the iteration column name within this DataFrame
    itc = find_iter_col(llm)

    # Convert iteration column to nullable integer, coercing errors to NaN
    llm[itc] = pd.to_numeric(llm[itc], errors="coerce").astype("Int64")

    # Convert total_balls column to numeric, coercing errors to NaN
    llm["total_balls"] = pd.to_numeric(llm["total_balls"], errors="coerce")

    # Drop rows where the iteration ID is NaN
    llm = llm.dropna(subset=[itc])

    # Build and return {integer_iteration_id -> total_balls} dictionary
    return dict(zip(llm[itc].astype(int), llm["total_balls"]))


def build_global_rows(items):
    """
    Build a global feature matrix with one row per iteration across all experiments.

    For each iteration, extracts DMP and EE trajectory features plus a DMP-EE
    mismatch metric. Returns the feature matrix, associated metadata, source
    experiment info, and the feature name list.

    Feature vector layout (17 features total):
        - Indices  0-7 : DMP trajectory features (bbox, path_len, smooth_turn).
        - Indices  8-15: EE  trajectory features (bbox, path_len, smooth_turn).
        - Index   16   : Mean pointwise Euclidean distance between DMP and EE.

    Args:
        items (list[dict]): Experiment metadata list from discover_all_experiments().

    Returns:
        tuple:
            - X_all      (np.ndarray or None): (M, 17) feature matrix, or None if empty.
            - meta_all   (list[dict]):  Per-row metadata (run, exp, iteration, total_balls).
            - sources    (list[dict]):  Per-row source experiment dict (dmp_csv, ee_csv, …).
            - feat_names (list[str]):   Ordered list of feature column names.
    """
    # Accumulators for feature vectors, metadata, and source references
    rows     = []
    meta_all = []
    sources  = []

    # Ordered list of feature names matching the vector layout
    feat_names = [
        # DMP trajectory bounding-box and shape features
        "dmp_x_min","dmp_x_max","dmp_y_min","dmp_y_max","dmp_width","dmp_height","dmp_path_len","dmp_smooth_turn",
        # EE trajectory bounding-box and shape features
        "ee_x_min","ee_x_max","ee_y_min","ee_y_max","ee_width","ee_height","ee_path_len","ee_smooth_turn",
        # Mean pointwise distance between resampled DMP and EE trajectories
        "dmp_ee_mean_dist"
    ]

    # Process each experiment in the input list
    for it in items:
        # Load iteration -> total_balls mapping from the LLM log
        balls_map = load_llm_balls_map(it["llm_csv"])

        # Load iteration -> xy array mappings for DMP and EE trajectories
        dmp_map = load_iter_map_xy(it["dmp_csv"])
        ee_map  = load_iter_map_xy(it["ee_csv"])

        # Collect the union of all iteration IDs present in either trajectory
        all_iters = sorted(set(dmp_map.keys()) | set(ee_map.keys()))

        # Skip this experiment if no valid iterations were found
        if not all_iters:
            continue

        # Process each iteration individually
        for iteration in all_iters:
            # Retrieve raw trajectory arrays for this iteration (None if absent)
            dmp_xy = dmp_map.get(iteration, None)
            ee_xy  = ee_map.get(iteration, None)

            # Resample DMP trajectory to RESAMPLE_N points if it has enough data
            dmp_rs = resample_xy(dmp_xy, RESAMPLE_N) if (dmp_xy is not None and len(dmp_xy) > 1) else None

            # Resample EE trajectory to RESAMPLE_N points if it has enough data
            ee_rs  = resample_xy(ee_xy, RESAMPLE_N)  if (ee_xy  is not None and len(ee_xy)  > 1) else None

            # Extract shape features from the resampled DMP trajectory (None if too short)
            dmp_f = traj_features(dmp_rs) if dmp_rs is not None else None

            # Extract shape features from the resampled EE trajectory (None if too short)
            ee_f  = traj_features(ee_rs)  if ee_rs  is not None else None

            # Skip this iteration if neither trajectory produced usable features
            if (dmp_f is None) and (ee_f is None):
                continue

            def pack(prefix, f):
                """
                Pack a feature dict into an 8-element list for one trajectory.

                Uses NaN placeholders for the entire 8-element block when the
                feature dict is None (i.e. the trajectory was missing/unusable).

                Args:
                    prefix (str): Unused label prefix (kept for clarity at call sites).
                    f (dict or None): Feature dictionary from traj_features(), or None.

                Returns:
                    list[float]: 8-element list of feature values or NaNs.
                """
                # Return 8 NaN values if no features are available for this side
                if f is None:
                    return [np.nan]*8
                # Return features in the canonical order matching feat_names
                return [
                    f["x_min"], f["x_max"], f["y_min"], f["y_max"],
                    f["width"], f["height"], f["path_len"], f["smooth_turn"]
                ]

            # Pack DMP features into a flat 8-element list
            dmp_vec = pack("dmp", dmp_f)

            # Pack EE features into a flat 8-element list
            ee_vec  = pack("ee", ee_f)

            # Compute mean pointwise Euclidean distance between DMP and EE trajectories
            if dmp_rs is not None and ee_rs is not None:
                # Compute per-point distances between resampled DMP and EE
                dist = np.sqrt(np.sum((dmp_rs - ee_rs)**2, axis=1))
                mean_dist = float(np.mean(dist))  # Average distance across all N points
            else:
                # Distance cannot be computed if one trajectory is missing
                mean_dist = np.nan

            # Concatenate DMP features, EE features, and mean distance into one vector
            vec = dmp_vec + ee_vec + [mean_dist]
            rows.append(vec)  # Add to feature matrix accumulator

            # Record metadata for this iteration row
            meta_all.append({
                "run":         it["run"],                                    # Run folder name
                "exp":         f"exp{it['exp_num']}",                        # Experiment label
                "iteration":   int(iteration),                               # Iteration number
                "total_balls": float(balls_map.get(int(iteration), np.nan))  # Ball count or NaN
            })

            # Record the source experiment dict for later plot generation
            sources.append(it)

    # Return None for X_all if no valid rows were produced
    if not rows:
        return None, [], [], feat_names

    # Convert accumulated rows to a float NumPy array
    X_all = np.array(rows, dtype=float)
    return X_all, meta_all, sources, feat_names


# =========================================================
# SOM: train + graphs
# =========================================================

def choose_grid(n_samples: int) -> int:
    """
    Choose a square SOM grid size based on the number of input samples.

    Uses a heuristic that scales the grid with the fourth root of the sample
    count, clamped to the range [10, 30].

    Args:
        n_samples (int): Total number of samples to be clustered.

    Returns:
        int: Side length of the square SOM grid (grid x grid neurons).
    """
    # Heuristic: scale grid by fourth root of sample count, multiplied by 10
    g = int(np.sqrt(np.sqrt(max(n_samples, 1))) * 10)
    # Clamp the result to the valid range [10, 30]
    return max(10, min(g, 30))


def train_som(X_all):
    """
    Z-score normalise the feature matrix, choose a grid size, and train a SOM.

    Initialises the SOM with random weights from the input data and trains
    it using random sample presentation for SOM_ITERS iterations.

    Args:
        X_all (np.ndarray): (M, D) raw feature matrix.

    Returns:
        tuple:
            - som     (MiniSom):        Trained SOM object.
            - winners (list[tuple]):    List of (x, y) winning neuron indices per sample.
            - grid    (int):            Side length of the square SOM grid.
            - Xn      (np.ndarray):     (M, D) z-score normalised feature matrix used for training.
    """
    # Z-score normalise all features to zero mean and unit variance
    Xn = zscore(X_all)

    # Determine the appropriate grid size for the number of samples
    grid = choose_grid(Xn.shape[0])

    # Instantiate the SOM with the chosen grid and feature dimensionality
    som = MiniSom(x=grid, y=grid, input_len=Xn.shape[1], sigma=1.5, learning_rate=0.5)

    # Initialise SOM weights randomly using samples from the input data
    som.random_weights_init(Xn)

    # Train the SOM using random sample presentation
    som.train_random(Xn, num_iteration=SOM_ITERS)

    # Compute the winning neuron (x, y) for every input sample
    winners = [som.winner(v) for v in Xn]

    return som, winners, grid, Xn


def save_umatrix(som, out_png):
    """
    Compute the SOM U-Matrix and save it as a PNG image.

    The U-Matrix shows the average distance between each neuron and its
    neighbours; high values indicate cluster boundaries.

    Args:
        som     (MiniSom): Trained SOM object.
        out_png (str):     Absolute path where the PNG image will be saved.

    Returns:
        None
    """
    # Compute the unified distance matrix from the trained SOM
    um = som.distance_map()

    # Create a new figure for the U-Matrix visualisation
    plt.figure(figsize=(7, 6))

    # Display the U-Matrix as a colour-mapped image (transpose for correct orientation)
    plt.imshow(um.T, origin="lower")

    # Add a colour bar to indicate distance magnitude
    plt.colorbar()

    # Set a descriptive title
    plt.title("SOM U-Matrix (distance map)")

    # Adjust layout to avoid label clipping
    plt.tight_layout()

    # Save the figure to disk at high resolution
    plt.savefig(out_png, dpi=220)

    # Close the figure to free memory
    plt.close()


def save_hitmap(grid, winners, out_png):
    """
    Count how many samples map to each SOM cell and save a hit-map image.

    The hit map is useful for identifying densely populated regions of the
    SOM grid that represent common trajectory patterns.

    Args:
        grid    (int):           Side length of the square SOM grid.
        winners (list[tuple]):   List of (x, y) winning neuron indices per sample.
        out_png (str):           Absolute path where the PNG image will be saved.

    Returns:
        np.ndarray: (grid, grid) integer array of sample counts per cell.
    """
    # Initialise a zero-filled count matrix of size grid x grid
    counts = np.zeros((grid, grid), dtype=int)

    # Increment the count for each winning cell
    for (x, y) in winners:
        counts[x, y] += 1

    # Create a new figure for the hit map
    plt.figure(figsize=(7, 6))

    # Display the hit count matrix as a colour-mapped image
    plt.imshow(counts.T, origin="lower")

    # Add a colour bar to show sample counts
    plt.colorbar()

    # Set a descriptive title
    plt.title("SOM Hit Map (samples per cell)")

    # Adjust layout to avoid label clipping
    plt.tight_layout()

    # Save the figure to disk at high resolution
    plt.savefig(out_png, dpi=220)

    # Close the figure to release memory
    plt.close()

    # Return the count matrix for downstream use
    return counts


def save_median_balls_map(grid, winners, meta_all, out_png):
    """
    Compute the median total_balls per SOM cell and save the result as an image.

    Cells with no valid ball count data are shown as NaN (typically displayed
    as a uniform background colour by matplotlib).

    Args:
        grid     (int):          Side length of the square SOM grid.
        winners  (list[tuple]):  List of (x, y) winning neuron indices per sample.
        meta_all (list[dict]):   Per-sample metadata containing 'total_balls'.
        out_png  (str):          Absolute path where the PNG image will be saved.

    Returns:
        np.ndarray: (grid, grid) float array of median total_balls per cell (NaN where empty).
    """
    # Accumulate ball counts per SOM cell using a defaultdict of lists
    cell_vals = defaultdict(list)
    for w, m in zip(winners, meta_all):
        b = m.get("total_balls", np.nan)  # Retrieve ball count; default to NaN if missing
        if np.isfinite(b):                # Only include finite (non-NaN, non-Inf) values
            cell_vals[w].append(b)

    # Initialise the median grid with NaN to handle empty cells
    med = np.full((grid, grid), np.nan, dtype=float)

    # Fill each cell with the median of its collected ball counts
    for (x, y), vals in cell_vals.items():
        med[x, y] = float(np.median(vals))

    # Create a new figure for the median balls map
    plt.figure(figsize=(7, 6))

    # Display the median matrix as a colour-mapped image
    plt.imshow(med.T, origin="lower")

    # Add a colour bar to show median ball count magnitude
    plt.colorbar()

    # Set a descriptive title
    plt.title("SOM Median total_balls per cell")

    # Adjust layout to avoid label clipping
    plt.tight_layout()

    # Save the figure to disk at high resolution
    plt.savefig(out_png, dpi=220)

    # Close the figure to release memory
    plt.close()

    # Return the median grid for downstream use
    return med


# =========================================================
# Plot overlay (DMP + EE in same plot) for a single iteration
# =========================================================

def plot_overlay(dmp_csv, ee_csv, iteration, out_png, title):
    """
    Generate and save an overlay plot of DMP and EE trajectories for one iteration.

    Loads trajectory data from CSV files, plots both paths on the same axes in
    green (DMP opaque, EE transparent), and saves the result as a PNG image.
    Returns False without writing a file if both trajectories are missing.

    Args:
        dmp_csv   (str or None): Path to the filtered DMP trajectory CSV.
        ee_csv    (str or None): Path to the filtered EE trajectory CSV.
        iteration (int):         Iteration number to retrieve and plot.
        out_png   (str):         Absolute path where the PNG image will be saved.
        title     (str):         Title string displayed at the top of the plot.

    Returns:
        bool: True if the plot was saved successfully, False if no data was available.
    """
    # Load per-iteration xy maps from DMP and EE trajectory CSVs
    dmp_map = load_iter_map_xy(dmp_csv)
    ee_map  = load_iter_map_xy(ee_csv)

    # Retrieve the xy array for the requested iteration (None if not present)
    dmp_xy = dmp_map.get(int(iteration), None)
    ee_xy  = ee_map.get(int(iteration), None)

    # Return False early if neither trajectory has data for this iteration
    if dmp_xy is None and ee_xy is None:
        return False

    # Create a new figure with specified dimensions
    plt.figure(figsize=(7.5, 6.5))

    # Plot DMP trajectory in solid opaque green if it has at least two points
    if dmp_xy is not None and len(dmp_xy) > 1:
        plt.plot(dmp_xy[:, 0], dmp_xy[:, 1], color="green", linewidth=2.2, alpha=0.85, label="DMP")

    # Plot EE trajectory in semi-transparent green if it has at least two points
    if ee_xy is not None and len(ee_xy) > 1:
        plt.plot(ee_xy[:, 0], ee_xy[:, 1], color="green", linewidth=2.2, alpha=0.35, label="EE")

    # Set the plot title from the caller-supplied string
    plt.title(title)

    # Label the horizontal axis
    plt.xlabel("x")

    # Label the vertical axis
    plt.ylabel("y")

    # Enable a light grid overlay for readability
    plt.grid(True, alpha=0.25)

    # Add a legend to distinguish DMP and EE lines
    plt.legend()

    # Force equal axis scaling so the trajectory shape is not distorted
    plt.axis("equal")

    # Adjust layout to prevent label clipping
    plt.tight_layout()

    # Save the figure as a PNG at high resolution
    plt.savefig(out_png, dpi=220)

    # Close the figure to free memory
    plt.close()

    # Indicate that the plot was saved successfully
    return True


# =========================================================
# MAIN
# =========================================================

def main():
    """
    Run the full global SOM clustering pipeline on filtered trajectory data.

    Steps performed:
        1. Validate that the filtered output directory exists.
        2. Discover all experiments and load feature matrices.
        3. Train a Self-Organising Map on the combined feature data.
        4. Save diagnostic plots (U-Matrix, hit map, median balls map).
        5. Write per-iteration SOM assignments to CSV.
        6. Write per-cell cluster summary statistics to CSV.
        7. Export top-K overlay plots per SOM cell sorted by lowest ball count.

    Returns:
        None

    Raises:
        RuntimeError: If the output directory is missing, no experiments are found,
                      or the global feature matrix has fewer than 2 rows.
    """
    # Print a banner to indicate the start of SOM clustering
    print("\n==================== GLOBAL SOM CLUSTERING ====================")

    # Print the path to the filtered data that will be analysed
    print("[INFO] OUTPUT_DIR:", OUTPUT_DIR)

    # Print the path where results will be saved
    print("[INFO] RESULT_DIR:", RESULT_DIR)

    # Print whether the filtered output directory actually exists on disk
    print("[INFO] OUTPUT_DIR exists:", os.path.isdir(OUTPUT_DIR))

    # Abort if the output directory from sorttinggood_AB.py does not exist
    if not os.path.isdir(OUTPUT_DIR):
        raise RuntimeError(f"Filtered output folder not found: {OUTPUT_DIR}")

    # Discover all valid experiments under the output directory
    items = discover_all_experiments(OUTPUT_DIR)
    

    # Abort if no experiments were found
    if not items:
        raise RuntimeError(f"No experiments found under: {OUTPUT_DIR}")

    # Build the global feature matrix from all discovered experiments
    X_all, meta_all, sources, feat_names = build_global_rows(items)

    # Abort if there are fewer than 2 iterations (SOM requires multiple samples)
    if X_all is None or X_all.shape[0] < 2:
        raise RuntimeError("Not enough iterations globally to cluster. Check that filtered CSVs contain iterations.")

    # Report the total number of samples and feature dimensionality
    print(f"[INFO] Total iterations (global samples): {X_all.shape[0]}")
    print(f"[INFO] Feature dim confirm: {X_all.shape[1]}")

    # Train the SOM and obtain per-sample winning cell assignments
    som, winners, grid, Xn = train_som(X_all)
    print(f"[INFO] Trained SOM grid: {grid} x {grid}")

    # Create the global output root directory for all SOM results
    out_root = ensure_dir(os.path.join(RESULT_DIR, "global"))

    # Create sub-directory for diagnostic SOM plots
    plots_dir = ensure_dir(os.path.join(out_root, "plots"))

    # Create sub-directory for per-cell cluster trajectory plots
    clusters_dir = ensure_dir(os.path.join(out_root, "clusters"))

    # --- Global diagnostic plots ---

    # Save the U-Matrix (inter-neuron distances) to a PNG file
    save_umatrix(som, os.path.join(plots_dir, "som_umatrix.png"))

    # Save the hit map (samples per cell) and retrieve the count matrix
    hit_counts = save_hitmap(grid, winners, os.path.join(plots_dir, "som_hitmap.png"))

    # Save the median total_balls map and retrieve the values matrix
    med_map = save_median_balls_map(grid, winners, meta_all, os.path.join(plots_dir, "som_median_balls_map.png"))

    # --- Build per-sample assignment rows for the output CSV ---
    assign_rows = []
    for m, w in zip(meta_all, winners):
        # Construct a human-readable name: e.g. "runA exp3 iter 21"
        name = f"{m['run']} {m['exp']} iter {m['iteration']}"
        assign_rows.append({
            "name":        name,                              # Human-readable identifier
            "run":         m["run"],                          # Run folder name
            "exp":         m["exp"],                          # Experiment label (e.g. exp3)
            "iteration":   m["iteration"],                    # Integer iteration number
            "total_balls": m.get("total_balls", np.nan),     # Ball count (NaN if missing)
            "cell_x":      w[0],                             # SOM winning cell x-coordinate
            "cell_y":      w[1],                             # SOM winning cell y-coordinate
        })

    # Convert assignment rows to a DataFrame and save to CSV
    assign_df = pd.DataFrame(assign_rows)
    assign_df.to_csv(os.path.join(out_root, "som_assignments_all.csv"), index=False)

    # --- Compute per-cell cluster summary statistics ---
    clusters = defaultdict(list)  # Map each (cell_x, cell_y) to a list of member dicts
    for row in assign_rows:
        clusters[(row["cell_x"], row["cell_y"])].append(row)  # Append row to its cell bucket

    # Accumulate summary statistics for each populated SOM cell
    summ = []
    for (cx, cy), members in clusters.items():
        # Collect ball counts for all members of this cell as a float array
        balls = np.array([r["total_balls"] for r in members], dtype=float)
        summ.append({
            "cell_x":       cx,                                                                   # Cell x-coordinate
            "cell_y":       cy,                                                                   # Cell y-coordinate
            "count":        len(members),                                                         # Number of samples in cell
            "median_balls": float(np.nanmedian(balls)) if np.isfinite(balls).any() else np.nan,  # Median ball count
            "best_balls":   float(np.nanmin(balls))    if np.isfinite(balls).any() else np.nan,  # Minimum ball count
            "worst_balls":  float(np.nanmax(balls))    if np.isfinite(balls).any() else np.nan,  # Maximum ball count
        })

    # Convert summary to DataFrame and sort by median -> best -> count descending
    summ_df = pd.DataFrame(summ).sort_values(
        ["median_balls", "best_balls", "count"],
        ascending=[True, True, False]   # Lower ball counts first; larger cells first on ties
    )

    # Save the cluster summary CSV to the output root
    summ_df.to_csv(os.path.join(out_root, "som_cluster_summary_all.csv"), index=False)

    # --- Export top-K overlay plots per SOM cell, best ball count first ---
    print("\n[INFO] Saving top trajectories per SOM cell...")

    for (cx, cy), members in clusters.items():
        # Sort members by total_balls ascending, placing NaN values last
        members_sorted = sorted(
            members,
            key=lambda r: (np.inf if not np.isfinite(r["total_balls"]) else r["total_balls"])
        )

        # Take only the top-K best members for this cell
        top = members_sorted[:TOPK_PER_CELL]

        # Skip cells with no members (should not occur, but guard anyway)
        if not top:
            continue

        # Create a dedicated output directory for this SOM cell
        cell_dir = ensure_dir(os.path.join(clusters_dir, f"cell_{cx}_{cy}"))

        # Generate one overlay plot per top member in this cell
        for r in top:
            # Identify the run and experiment number for this member
            run     = r["run"]
            exp_num = int(str(r["exp"]).replace("exp", ""))  # Strip 'exp' prefix to get int

            # Find the matching source experiment dict to retrieve CSV paths
            src = None
            for it in items:
                if it["run"] == run and it["exp_num"] == exp_num:
                    src = it   # Found the correct source experiment
                    break

            # Skip if no matching source was found
            if src is None:
                continue

            # Build the output image filename in the required naming convention
            fname   = f"{r['run']} {r['exp']} iter {r['iteration']}.png"
            out_png = os.path.join(cell_dir, fname)

            # Build an informative title string for the plot
            title = f"{r['run']} {r['exp']} iter {r['iteration']} | cell({cx},{cy}) | balls={r['total_balls']}"

            # Generate and save the overlay plot; skip if no trajectory data available
            ok = plot_overlay(src["dmp_csv"], src["ee_csv"], r["iteration"], out_png, title)
            if not ok:
                continue  # No trajectory data for this iteration; move on

    # Print a completion banner summarising where results were saved
    print("\n✅ DONE.")
    print("Saved GLOBAL SOM results here:")
    print(out_root)
    print("\nKey outputs:")
    print("  - plots/som_umatrix.png")           # U-Matrix diagnostic plot
    print("  - plots/som_hitmap.png")             # Hit map diagnostic plot
    print("  - plots/som_median_balls_map.png")   # Median balls per cell map
    print("  - som_assignments_all.csv")          # Per-iteration SOM cell assignments
    print("  - som_cluster_summary_all.csv")      # Per-cell summary statistics
    print("  - clusters/cell_x_y/<run exp iter>.png")  # Top-K trajectory plots per cell


# Entry point: run main() only when executed directly (not imported as a module)
if __name__ == "__main__":
    main()
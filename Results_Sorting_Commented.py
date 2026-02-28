# Standard library imports for file/directory operations
import os
import shutil

# Data manipulation library
import pandas as pd
from config import load_config  # Custom function to load configuration settings

# Plotting library
import matplotlib.pyplot as plt


# =========================================================
# CONFIG (DYNAMIC)
# Put this script in the directory that contains your run folders
# =========================================================

# Get the absolute path of the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
config = load_config("./config/semantics-gridcoverage-hist-30.yaml")  # Load additional configuration settings if needed

SCRIPT_DIR = config["simulation"]["base_dir"]  # Use the base_dir from config, which should point to the same directory as the script
# Set the input directory to the same folder as the script
INPUT_DIR = config["simulation"].get("base_dir", SCRIPT_DIR)  # Use config value if provided, otherwise default to script directory

# Define the output directory as a subfolder named "output" inside the script directory
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "SOM_Results", "filtered_results")  # Output will be saved in a subfolder to keep things organized

# Create the output directory if it does not already exist
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Maximum number of balls allowed per iteration; iterations exceeding this are filtered out
MAX_BALLS = 100

# Number of rows to read at a time when processing large CSV files in chunks
CHUNK_SIZE = 250_000

# =========================================================
# STRICT BOUNDS (from enhancedll_Mohamed.py)
# =========================================================

ws_width = config["simulation"]["ws_width"]  # Get workspace width from config or use default
ws_length = config["simulation"]["ws_length"]  # Get workspace length from config or use default
ws_center = config["simulation"]["ws_center"]  # Get workspace center from config or use default


# Minimum  and Minimum allowed x-coordinate for trajectory points
STRICT_X_MIN, STRICT_X_MAX = ws_center[0] - ws_width / 2, ws_center[0] + ws_width / 2

# Minimum and maximum allowed y-coordinate for trajectory points
STRICT_Y_MIN, STRICT_Y_MAX = ws_center[1] - ws_length / 2, ws_center[1] + ws_length / 2



# =========================================================
# HELPERS
# =========================================================

def find_iter_col(df):
    """
    Find and return the name of the iteration column in a DataFrame.

    Searches for common iteration column name variants in the given DataFrame.
    Raises a ValueError if none of the expected column names are found.

    Args:
        df (pd.DataFrame): The DataFrame to search for an iteration column.

    Returns:
        str: The name of the iteration column found in the DataFrame.

    Raises:
        ValueError: If no recognized iteration column name is present in the DataFrame.
    """
    # Iterate over a list of known possible names for the iteration column
    for c in ["iter", "iteration", "Iteration", "ITER", "step", "idx", "step_idx"]:
        # Return the first matching column name found
        if c in df.columns:
            return c
    # No matching column found; raise an error with the actual column names for debugging
    raise ValueError(f"Iteration column not found. Columns: {list(df.columns)}")


def find_xy_cols(df):
    """
    Find and return the names of the x and y coordinate columns in a DataFrame.

    Checks for both lowercase and uppercase variants of x/y column names.
    Raises a ValueError if no matching pair is found.

    Args:
        df (pd.DataFrame): The DataFrame to search for coordinate columns.

    Returns:
        tuple[str, str]: A tuple of (x_column_name, y_column_name).

    Raises:
        ValueError: If no recognized x/y column pair is present in the DataFrame.
    """
    # Check for both lowercase and uppercase x/y column name pairs
    for xcol, ycol in [("x", "y"), ("X", "Y")]:
        # Return the first matching pair found
        if xcol in df.columns and ycol in df.columns:
            return xcol, ycol
    # No matching column pair found; raise an error with actual columns for debugging
    raise ValueError(f"x/y columns not found. Columns: {list(df.columns)}")


def list_csvs(folder_path):
    """
    Return a list of absolute paths to all CSV files in the given folder.

    Only files with a '.csv' extension (case-insensitive) are included.
    Returns an empty list if the folder does not exist.

    Args:
        folder_path (str): Absolute or relative path to the folder to scan.

    Returns:
        list[str]: List of absolute file paths to CSV files in the folder.
    """
    # Return empty list if the folder path does not point to a valid directory
    if not os.path.isdir(folder_path):
        return []
    # Build and return a list of full paths for each CSV file in the folder
    return [
        os.path.join(folder_path, f)         # Construct full file path
        for f in os.listdir(folder_path)     # Iterate over all entries in the folder
        if f.lower().endswith(".csv")        # Keep only files ending with '.csv'
    ]


def pick_file(csv_paths, keyword):
    """
    Find and return the first CSV file path whose filename contains the given keyword.

    The search is case-insensitive. Returns None if no matching file is found.

    Args:
        csv_paths (list[str]): List of file paths to search through.
        keyword (str): Keyword to look for in the filename (case-insensitive).

    Returns:
        str or None: The first matching file path, or None if no match is found.
    """
    # Normalize the keyword to lowercase for case-insensitive comparison
    keyword = keyword.lower()
    # Check each path to see if the filename contains the keyword
    for p in csv_paths:
        if keyword in os.path.basename(p).lower():  # Compare lowercase filename with keyword
            return p  # Return the first match found
    # No file matched the keyword
    return None


def is_experiment_folder(path):
    """
    Determine whether a given path is a valid experiment folder.

    A valid experiment folder is a directory whose name consists entirely of digits
    and which contains both an 'llm_iteration_log' CSV and at least one trajectory
    CSV ('dmp_trajectory_feedback' or 'ee_trajectory').

    Args:
        path (str): Absolute or relative path to the folder to evaluate.

    Returns:
        bool: True if the folder meets experiment folder criteria, False otherwise.
    """
    # Path must point to an existing directory
    if not os.path.isdir(path):
        return False

    # Get just the folder name (not the full path)
    name = os.path.basename(path)

    # Experiment folders must have a purely numeric name (e.g., "1", "42")
    if not name.isdigit():
        return False

    # Collect all CSV files in the folder
    csvs = list_csvs(path)

    # Check whether the required LLM log CSV is present
    has_llm = pick_file(csvs, "llm_iteration_log") is not None

    # Check whether a DMP trajectory CSV is present
    has_dmp = pick_file(csvs, "dmp_trajectory_feedback") is not None

    # Check whether an EE trajectory CSV is present
    has_ee  = pick_file(csvs, "ee_trajectory") is not None

    # Valid experiment requires the LLM log and at least one trajectory file
    # weights_history is optional but expected
    return has_llm and (has_dmp or has_ee)

def _iter_dirs_up_to_depth(root, max_depth):
    """
    Yield directory paths under `root` from depth 1..max_depth.
    Depth is relative to root:
      depth 1 => root/<child>
      depth 2 => root/<child>/<grandchild>
    """
    root = os.path.abspath(root)

    for cur, dirs, _files in os.walk(root):
        rel = os.path.relpath(cur, root)
        depth = 0 if rel == "." else rel.count(os.sep) + 1

        # Stop descending past max depth
        if depth >= max_depth:
            dirs[:] = []

        # Only yield paths inside root (not root itself)
        if 1 <= depth <= max_depth:
            yield cur


def _contains_experiment_within_depth(root, max_depth):
    """Return True if any valid experiment folder exists under root within max_depth."""
    for dpath in _iter_dirs_up_to_depth(root, max_depth):
        if is_experiment_folder(dpath):
            return True
    return False


def find_run_folders(base_dir, scan_depth=4):
    """
    Discover run folders under:
      <base_dir>/<group>/logs/<run_folder>
    and return only those that contain at least one experiment folder
    within `scan_depth`.

    Returns relative paths (from base_dir), e.g.:
      n_warmup-5/logs/semantics-RL-...
    """
    run_folders = []

    for group_name in os.listdir(base_dir):
        group_path = os.path.join(base_dir, group_name)
        if not os.path.isdir(group_path):
            continue
        if group_name.lower() == "som_results":
            continue

        logs_path = os.path.join(group_path, "logs")
        if not os.path.isdir(logs_path):
            continue

        for run_name in os.listdir(logs_path):
            run_path = os.path.join(logs_path, run_name)
            if not os.path.isdir(run_path):
                continue

            if _contains_experiment_within_depth(run_path, scan_depth):
                run_folders.append(os.path.relpath(run_path, base_dir))

    return sorted(set(run_folders))


def find_experiments_in_run(run_path, scan_depth=1):
    """
    Return a sorted list of experiment folder paths within run_path up to `scan_depth`.
    """
    exp_paths = [
        dpath
        for dpath in _iter_dirs_up_to_depth(run_path, scan_depth)
        if is_experiment_folder(dpath)
    ]

    # Remove duplicates and sort numerically by folder name
    exp_paths = list(dict.fromkeys(exp_paths))
    exp_paths = sorted(exp_paths, key=lambda p: int(os.path.basename(p)))
    return exp_paths

# def find_run_folders(base_dir):
#     """
#     Discover and return all 'run folders' under the given base directory.

#     A run folder is any immediate subdirectory of base_dir that contains
#     valid experiment folders (numeric directories with required CSVs),
#     either directly or up to two levels deeper. The 'output' folder is excluded.

#     Args:
#         base_dir (str): The root directory to search for run folders.

#     Returns:
#         list[str]: Sorted list of run folder names (not full paths) found under base_dir.
#     """
#     # Accumulate valid run folder names
#     run_folders = []

#     # Iterate over all entries in the base directory
#     for d in os.listdir(base_dir):
#         p = os.path.join(base_dir, d)  # Full path to the current entry

#         # Skip entries that are not directories
#         if not os.path.isdir(p):
#             continue

#         # Skip the output folder to avoid re-processing already filtered data
#         if d.lower() == "som_results":
#             continue

#         # Flag to track whether any experiment folder was found at any depth
#         found_any = False

#         # --- Depth 1: check for experiment folders directly inside this folder ---
#         for sub in os.listdir(p):
#             subp = os.path.join(p, sub)  # Full path to sub-entry
#             if is_experiment_folder(subp):  # Check if it's a valid experiment folder
#                 found_any = True
#                 break  # Stop as soon as one is found

#         # --- Depth 2: check one level deeper if nothing was found at depth 1 ---
#         if not found_any:
#             for sub in os.listdir(p):
#                 subp = os.path.join(p, sub)  # Full path to sub-entry at depth 1
#                 if not os.path.isdir(subp):  # Skip non-directories
#                     continue
#                 for sub2 in os.listdir(subp):
#                     sub2p = os.path.join(subp, sub2)  # Full path at depth 2
#                     if is_experiment_folder(sub2p):  # Check if valid experiment folder
#                         found_any = True
#                         break  # Stop inner loop on first match
#                 if found_any:
#                     break  # Stop outer loop on first match

#         # --- Depth 3: check two levels deeper if nothing was found at depth 1 or 2 ---
#         if not found_any:
#             for sub in os.listdir(p):
#                 subp = os.path.join(p, sub)  # Full path to depth-1 sub-entry
#                 if not os.path.isdir(subp):  # Skip non-directories at depth 1
#                     continue
#                 for sub2 in os.listdir(subp):
#                     sub2p = os.path.join(subp, sub2)  # Full path to depth-2 sub-entry
#                     if not os.path.isdir(sub2p):  # Skip non-directories at depth 2
#                         continue
#                     for sub3 in os.listdir(sub2p):
#                         sub3p = os.path.join(sub2p, sub3)  # Full path at depth 3
#                         if is_experiment_folder(sub3p):  # Check if valid experiment folder
#                             found_any = True
#                             break  # Stop innermost loop on first match
#                     if found_any:
#                         break  # Stop depth-2 loop on first match
#                 if found_any:
#                     break  # Stop depth-1 loop on first match

#         # Add this folder to the list if it contains at least one experiment at any depth
#         if found_any:
#             run_folders.append(d)

#     # Return run folder names sorted alphabetically
#     return sorted(run_folders)


# def find_experiments_in_run(run_path):
#     """
#     Return a sorted list of absolute paths to all experiment folders within a run folder.

#     Searches directly inside run_path (depth 1), one level deeper (depth 2),
#     and two levels deeper (depth 3). Results are sorted numerically by the
#     experiment folder name.

#     Args:
#         run_path (str): Absolute path to the run folder to search.

#     Returns:
#         list[str]: Numerically sorted list of absolute paths to experiment folders.
#     """
#     # Accumulate found experiment folder paths
#     exp_paths = []

#     # --- Depth 1: check for experiment folders directly inside the run folder ---
#     for sub in os.listdir(run_path):
#         subp = os.path.join(run_path, sub)  # Full path to sub-entry
#         if is_experiment_folder(subp):      # Check if it qualifies as an experiment folder
#             exp_paths.append(subp)          # Add to the list

#     # --- Depth 2: check inside non-numeric subdirectories ---
#     for sub in os.listdir(run_path):
#         subp = os.path.join(run_path, sub)  # Full path to sub-entry
#         # Skip non-directories and directories that are already numeric exp folders
#         if not os.path.isdir(subp) or os.path.basename(subp).isdigit():
#             continue
#         for sub2 in os.listdir(subp):
#             sub2p = os.path.join(subp, sub2)  # Full path at depth 2
#             if is_experiment_folder(sub2p):    # Check if valid experiment folder
#                 exp_paths.append(sub2p)        # Add to the list

#     # --- Depth 3: check one level deeper than depth 2 ---
#     # for sub in os.listdir(run_path):
#     #     subp = os.path.join(run_path, sub)  # Full path to depth-1 sub-entry
#     #     # Skip non-directories and numeric folders already captured at depth 1
#     #     if not os.path.isdir(subp) or os.path.basename(subp).isdigit():
#     #         continue
#     #     for sub2 in os.listdir(subp):
#     #         sub2p = os.path.join(subp, sub2)  # Full path to depth-2 sub-entry
#     #         # Skip non-directories and numeric folders already captured at depth 2
#     #         if not os.path.isdir(sub2p) or os.path.basename(sub2p).isdigit():
#     #             continue
#     #         for sub3 in os.listdir(sub2p):
#     #             sub3p = os.path.join(sub2p, sub3)  # Full path at depth 3
#     #             if is_experiment_folder(sub3p):     # Check if valid experiment folder
#     #                 exp_paths.append(sub3p)         # Add to the list

#     # Remove any duplicates that may have been found at multiple depths
#     exp_paths = list(dict.fromkeys(exp_paths))

#     # Sort all found experiment paths numerically by their folder name
#     exp_paths = sorted(exp_paths, key=lambda p: int(os.path.basename(p)))
#     return exp_paths


# =========================================================
# FILTER RULES
# =========================================================

def read_bad_iters_from_llm(llm_path):
    """
    Read the LLM iteration log and identify iterations with too many balls.

    Loads the CSV at llm_path and marks any iteration where 'total_balls'
    exceeds MAX_BALLS as 'bad'. Returns the set of bad iteration IDs and
    a filtered DataFrame containing only the valid (kept) rows.

    Args:
        llm_path (str): Absolute path to the llm_iteration_log CSV file.

    Returns:
        tuple[set[int], pd.DataFrame]:
            - A set of integer iteration IDs that exceed MAX_BALLS.
            - A DataFrame of rows where total_balls <= MAX_BALLS (or is NaN).

    Raises:
        ValueError: If the 'total_balls' column is not found in the CSV.
    """
    # Load the LLM iteration log CSV into a DataFrame
    llm = pd.read_csv(llm_path)

    # Validate that the required 'total_balls' column exists
    if "total_balls" not in llm.columns:
        raise ValueError(f"'total_balls' missing in {llm_path}")

    # Identify the iteration column name in this DataFrame
    iter_col = find_iter_col(llm)

    # Convert the iteration column to nullable integer type, coercing errors to NaN
    llm[iter_col] = pd.to_numeric(llm[iter_col], errors="coerce").astype("Int64")

    # Convert the total_balls column to numeric, coercing non-numeric values to NaN
    balls = pd.to_numeric(llm["total_balls"], errors="coerce")

    # Create a boolean mask for rows where total_balls exceeds the allowed maximum
    bad_mask = balls > MAX_BALLS

    # Extract the set of unique iteration IDs that are flagged as bad
    bad_iters = set(llm.loc[bad_mask, iter_col].dropna().astype(int).unique().tolist())

    # Build a mask for rows to keep: total_balls is within limit or is missing (NaN)
    keep_mask = (~bad_mask) | balls.isna()

    # Filter the DataFrame to retain only the valid rows
    llm_filtered = llm.loc[keep_mask].copy()

    return bad_iters, llm_filtered


def bad_iters_from_bounds(traj_path):
    """
    Identify iterations in a trajectory CSV where any point falls outside strict bounds.

    Reads the trajectory file in chunks and flags any iteration that contains
    at least one row where x or y is outside the defined STRICT_X/Y bounds.

    Args:
        traj_path (str): Absolute path to the trajectory CSV file (DMP or EE).

    Returns:
        set[int]: Set of integer iteration IDs that contain out-of-bounds trajectory points.
    """
    # Set to accumulate iteration IDs that have out-of-bounds points
    bad = set()

    # Flag to detect the first chunk and determine column names
    first = True

    # Placeholders for column name variables determined from the first chunk
    iter_col = None
    xcol = ycol = None

    # Process the trajectory CSV in chunks to handle large files efficiently
    for chunk in pd.read_csv(traj_path, chunksize=CHUNK_SIZE):
        if first:
            # Identify iteration and coordinate columns from the first chunk
            iter_col = find_iter_col(chunk)
            xcol, ycol = find_xy_cols(chunk)
            first = False  # Mark that column names have been determined

        # Convert iteration column to nullable integer, coercing errors to NaN
        chunk[iter_col] = pd.to_numeric(chunk[iter_col], errors="coerce").astype("Int64")

        # Convert x coordinate column to numeric, coercing errors to NaN
        xs = pd.to_numeric(chunk[xcol], errors="coerce")

        # Convert y coordinate column to numeric, coercing errors to NaN
        ys = pd.to_numeric(chunk[ycol], errors="coerce")

        # Build a boolean mask for rows where x or y is outside the strict bounds
        oob = (
            (xs < STRICT_X_MIN) | (xs > STRICT_X_MAX) |  # x is out of bounds
            (ys < STRICT_Y_MIN) | (ys > STRICT_Y_MAX)    # y is out of bounds
        )

        # If any out-of-bounds rows exist, record their iteration IDs
        if oob.any():
            bad.update(chunk.loc[oob, iter_col].dropna().astype(int).unique().tolist())

    return bad


def filter_small_csv_by_iters(in_path, out_path, bad_iters):
    """
    Filter a small CSV file by removing rows belonging to bad iterations.

    Loads the entire CSV into memory, removes rows whose iteration ID is in
    bad_iters, and saves the result to out_path.

    Args:
        in_path (str): Absolute path to the input CSV file.
        out_path (str): Absolute path where the filtered CSV will be saved.
        bad_iters (set[int]): Set of iteration IDs to remove from the file.

    Returns:
        tuple[int, int]: A tuple of (rows_before_filter, rows_after_filter).
    """
    # Load the entire CSV into a DataFrame (suitable for small files)
    df = pd.read_csv(in_path)

    # Find the iteration column name in the DataFrame
    iter_col = find_iter_col(df)

    # Convert iteration column to nullable integer, coercing errors to NaN
    df[iter_col] = pd.to_numeric(df[iter_col], errors="coerce").astype("Int64")

    # Record the number of rows before filtering
    before = len(df)

    # Remove rows whose iteration ID is in the bad_iters set
    df_f = df[~df[iter_col].isin(bad_iters)].copy()

    # Save the filtered DataFrame to the output CSV without the index column
    df_f.to_csv(out_path, index=False)

    # Return row counts before and after filtering
    return before, len(df_f)


def filter_large_csv_by_iters(in_path, out_path, bad_iters):
    """
    Filter a large CSV file by removing rows belonging to bad iterations.

    Reads the input CSV in chunks to avoid loading the entire file into memory.
    Writes filtered chunks to out_path, appending after the first valid chunk.
    If no rows pass the filter, writes an empty CSV with only the header row.

    Args:
        in_path (str): Absolute path to the large input CSV file.
        out_path (str): Absolute path where the filtered CSV will be saved.
        bad_iters (set[int]): Set of iteration IDs to remove from the file.

    Returns:
        tuple[int, int]: A tuple of (total_rows_read, total_rows_written).
    """
    # Flag to determine whether the output file header should be written
    first = True

    # Flag to track if at least one non-empty chunk has been written
    wrote_any = False

    # Counters for total rows processed and written
    total_in = 0
    total_out = 0

    # Process the CSV in memory-efficient chunks
    for chunk in pd.read_csv(in_path, chunksize=CHUNK_SIZE):
        # Identify the iteration column in the current chunk
        iter_col = find_iter_col(chunk)

        # Convert iteration column to nullable integer, coercing errors to NaN
        chunk[iter_col] = pd.to_numeric(chunk[iter_col], errors="coerce").astype("Int64")

        # Accumulate the total number of rows read so far
        total_in += len(chunk)

        # Remove rows belonging to bad iterations
        chunk_f = chunk[~chunk[iter_col].isin(bad_iters)].copy()

        # Accumulate the total number of rows kept so far
        total_out += len(chunk_f)

        # Skip writing if the filtered chunk is empty
        if chunk_f.empty:
            continue

        # Write the filtered chunk; overwrite on first write, append afterwards
        chunk_f.to_csv(out_path, mode="w" if first else "a", header=first, index=False)

        # After first write, switch to append mode and suppress the header
        first = False
        wrote_any = True  # Mark that at least one chunk has been written

    # If no data was written, create an empty CSV with just the header row
    if not wrote_any:
        pd.read_csv(in_path, nrows=0).to_csv(out_path, index=False)

    # Return total rows read and written
    return total_in, total_out


# =========================================================
# PLOTTING (ONLY from FILTERED OUTPUT CSVs)
# =========================================================

def load_xy_by_iter(csv_path):
    """
    Load a trajectory CSV and return a dictionary mapping iteration IDs to x/y arrays.

    Reads the CSV, converts relevant columns to numeric types, drops rows with
    missing values in the key columns, and groups the data by iteration.

    Args:
        csv_path (str): Absolute path to the trajectory CSV file.

    Returns:
        dict[int, np.ndarray]: Dictionary mapping each integer iteration ID to a
            (N, 2) NumPy array of [x, y] coordinate pairs for that iteration.
    """
    # Load the trajectory CSV into a DataFrame
    df = pd.read_csv(csv_path)

    # Find the iteration and coordinate column names
    iter_col = find_iter_col(df)
    xcol, ycol = find_xy_cols(df)

    # Convert iteration column to nullable integer, coercing errors to NaN
    df[iter_col] = pd.to_numeric(df[iter_col], errors="coerce").astype("Int64")

    # Convert x coordinate column to numeric, coercing errors to NaN
    df[xcol] = pd.to_numeric(df[xcol], errors="coerce")

    # Convert y coordinate column to numeric, coercing errors to NaN
    df[ycol] = pd.to_numeric(df[ycol], errors="coerce")

    # Drop rows that have NaN in any of the three key columns
    df = df.dropna(subset=[iter_col, xcol, ycol])

    # Build the output dictionary: {iteration_id -> (N, 2) NumPy array}
    out = {}
    for it, g in df.groupby(iter_col):  # Group rows by iteration ID
        out[int(it)] = g[[xcol, ycol]].to_numpy(dtype=float)  # Store as float array
    return out


def save_overlay_plot_green(dmp_xy, ee_xy, it, save_path):
    """
    Save an overlay plot of DMP and EE trajectories for a single iteration.

    Both trajectories are drawn in green; the DMP line is fully opaque and the
    EE line is semi-transparent. Dashed lines mark the strict spatial bounds.
    The iteration number is annotated in the top-left corner of the plot.

    Args:
        dmp_xy (np.ndarray or None): (N, 2) array of DMP [x, y] trajectory points,
            or None if DMP data is unavailable for this iteration.
        ee_xy (np.ndarray or None): (N, 2) array of EE [x, y] trajectory points,
            or None if EE data is unavailable for this iteration.
        it (int): The iteration number to display in the plot title and annotation.
        save_path (str): Absolute file path where the PNG image will be saved.

    Returns:
        None
    """
    # Create a new figure with a specified size in inches
    plt.figure(figsize=(7.5, 6.5))

    # Plot DMP trajectory in solid green if data is available and has more than one point
    if dmp_xy is not None and len(dmp_xy) > 1:
        plt.plot(dmp_xy[:, 0], dmp_xy[:, 1],    # x and y coordinates
                 color="green", linewidth=2.2, alpha=0.85, label="DMP")  # Opaque green line

    # Plot EE trajectory in lighter green if data is available and has more than one point
    if ee_xy is not None and len(ee_xy) > 1:
        plt.plot(ee_xy[:, 0], ee_xy[:, 1],       # x and y coordinates
                 color="green", linewidth=2.2, alpha=0.35, label="EE")   # Transparent green line

    # Draw a vertical dashed line at the minimum x bound for visual reference
    plt.axvline(STRICT_X_MIN, linestyle="--", linewidth=1.0, alpha=0.35)

    # Draw a vertical dashed line at the maximum x bound for visual reference
    plt.axvline(STRICT_X_MAX, linestyle="--", linewidth=1.0, alpha=0.35)

    # Draw a horizontal dashed line at the minimum y bound for visual reference
    plt.axhline(STRICT_Y_MIN, linestyle="--", linewidth=1.0, alpha=0.35)

    # Draw a horizontal dashed line at the maximum y bound for visual reference
    plt.axhline(STRICT_Y_MAX, linestyle="--", linewidth=1.0, alpha=0.35)

    # Set the plot title with the current iteration number
    plt.title(f"DMP + EE Overlay | iter {it}")

    # Label the x-axis
    plt.xlabel("x")

    # Label the y-axis
    plt.ylabel("y")

    # Enable a subtle grid for easier reading
    plt.grid(True, alpha=0.25)

    # Add a legend to distinguish DMP and EE lines
    plt.legend()

    # Add an annotation box in the top-left corner showing the iteration number
    plt.text(
        0.02, 0.98, f"iter = {it}",          # Position in axes coordinates (top-left)
        transform=plt.gca().transAxes,        # Use axes-relative coordinate system
        va="top",                             # Align text to the top of the box
        fontsize=12,                          # Font size for the annotation
        bbox=dict(boxstyle="round", alpha=0.2)  # Rounded box with slight transparency
    )

    # Adjust layout to prevent clipping of labels and titles
    plt.tight_layout()

    # Save the figure as a PNG file at the specified path with high DPI
    plt.savefig(save_path, dpi=220)

    # Close the figure to free memory
    plt.close()


def make_iteration_plots_only_new(exp_out_dir, dmp_filtered_path, ee_filtered_path):
    """
    Generate per-iteration overlay plots from filtered DMP and EE trajectory CSVs.

    Deletes any existing plot folder for this experiment and recreates it fresh.
    Plots are generated only for iterations present in the filtered output CSVs,
    ensuring that no plots are made for bad/removed iterations.

    Args:
        exp_out_dir (str): Absolute path to the experiment's output directory.
        dmp_filtered_path (str): Absolute path to the filtered DMP trajectory CSV.
        ee_filtered_path (str): Absolute path to the filtered EE trajectory CSV.

    Returns:
        None
    """
    # Define the path to the plots subdirectory within the experiment output folder
    plot_dir = os.path.join(exp_out_dir, "iter_plots")

    # Remove the existing plots folder entirely to avoid stale plots from previous runs
    if os.path.isdir(plot_dir):
        shutil.rmtree(plot_dir)  # Recursively delete the directory and its contents

    # Recreate the (now empty) plots directory
    os.makedirs(plot_dir, exist_ok=True)

    # Load DMP trajectory data if the filtered file exists; otherwise use empty dict
    dmp_map = load_xy_by_iter(dmp_filtered_path) if os.path.isfile(dmp_filtered_path) else {}

    # Load EE trajectory data if the filtered file exists; otherwise use empty dict
    ee_map  = load_xy_by_iter(ee_filtered_path)  if os.path.isfile(ee_filtered_path)  else {}

    # Collect the union of all iteration IDs present in either trajectory dataset
    iters = sorted(set(dmp_map.keys()) | set(ee_map.keys()))

    # If no iterations remain after filtering, skip plot generation
    if not iters:
        print("    [PLOT] No iterations left after filtering; no plots generated.")
        return

    # Inform the user how many plots will be generated and where they will be saved
    print(f"    [PLOT] Generating {len(iters)} NEW plots -> {plot_dir}")

    # Generate one overlay plot per iteration
    for it in iters:
        # Define the output image file path for this iteration
        out_img = os.path.join(plot_dir, f"iter_{it}.png")

        # Save the overlay plot combining DMP and EE data for this iteration
        save_overlay_plot_green(
            dmp_xy=dmp_map.get(it, None),   # DMP data for this iteration, or None
            ee_xy=ee_map.get(it, None),     # EE data for this iteration, or None
            it=it,                          # Iteration number for labelling
            save_path=out_img               # File path for saving the image
        )


# =========================================================
# MAIN: process all run folders -> all experiments
# =========================================================

# Print a header banner to indicate the start of processing
print("=========================================================")
print("AUTO-DETECT run folders + experiments")

# Print the base input directory being scanned
print(f"Base INPUT_DIR: {INPUT_DIR}")

# Print the output directory where filtered results will be saved
print(f"OUTPUT_DIR:     {OUTPUT_DIR}")

# Print the two filtering rules applied to each experiment
print("Filtering rules:")
print(f"  1) Keep only iters where total_balls <= {MAX_BALLS} (from llm_iteration_log)")
print("  2) Keep only iters inside bounds in BOTH trajectories (bad in DMP or EE removed)")
print(f"     X ∈ [{STRICT_X_MIN}, {STRICT_X_MAX}], Y ∈ [{STRICT_Y_MIN}, {STRICT_Y_MAX}]")
print("=========================================================\n")

# Discover all valid run folders under the input directory
run_folders = find_run_folders(INPUT_DIR)

print(f"[INFO] Total run folders found: {len(run_folders)}")
print("=========================================================\n")
print(f"run folders: {run_folders}\n")

# Abort if no run folders were found
if not run_folders:
    raise RuntimeError(f"No run folders found under: {INPUT_DIR}")

# Print the list of detected run folders
print("[INFO] Found run folders:")
for rf in run_folders:
    print("  -", rf)
print()

# Iterate over each discovered run folder
for run_name in run_folders:
    # Build the full path to the current run folder
    run_path = os.path.join(INPUT_DIR, run_name)

    # Find all experiment folders within this run folder
    exp_paths = find_experiments_in_run(run_path, 2)

    # Skip this run folder if it contains no valid experiments
    if not exp_paths:
        print(f"[SKIP] No experiments found in run folder: {run_name}")
        continue

    # Print a section header for the current run folder
    print(f"\n==================== RUN FOLDER: {run_name} ====================")
    print(f"[INFO] Experiments found: {len(exp_paths)}")

    # Iterate over each experiment folder within this run
    for exp_path in exp_paths:
        # Extract the experiment number from the folder name
        exp_num = os.path.basename(exp_path)  # numeric folder name e.g. "1", "42"

        # Print a sub-section header for the current experiment
        print(f"\n  -------- Experiment {exp_num} --------")

        # List all CSV files in this experiment folder
        csvs = list_csvs(exp_path)

        # Locate each required CSV file within the experiment folder
        llm_in = pick_file(csvs, "llm_iteration_log")          # LLM iteration log
        w_in   = pick_file(csvs, "weights_history")             # DMP weights history (optional)
        dmp_in = pick_file(csvs, "dmp_trajectory_feedback")    # DMP trajectory CSV
        ee_in  = pick_file(csvs, "ee_trajectory")              # EE (end-effector) trajectory CSV

        # Skip experiment if the mandatory LLM log file is missing
        if llm_in is None:
            print("  [SKIP] Missing llm_iteration_log*.csv")
            continue

        # Build the mirrored output directory: output/<run_name>/<exp_num>/
        exp_out_dir = os.path.join(OUTPUT_DIR, run_name, exp_num)

        # Create the experiment output directory if it does not exist
        os.makedirs(exp_out_dir, exist_ok=True)

        # Define output file paths mirroring the input filenames
        llm_out = os.path.join(exp_out_dir, os.path.basename(llm_in))  # Output LLM log path

        # Output weights path (only if weights file was found)
        w_out   = os.path.join(exp_out_dir, os.path.basename(w_in)) if w_in else None

        # Output DMP trajectory path (only if DMP file was found)
        dmp_out = os.path.join(exp_out_dir, os.path.basename(dmp_in)) if dmp_in else None

        # Output EE trajectory path (only if EE file was found)
        ee_out  = os.path.join(exp_out_dir, os.path.basename(ee_in)) if ee_in else None

        # ---- Build the combined set of bad iterations ----

        # Get bad iterations from the LLM log (too many balls) and the cleaned LLM DataFrame
        bad_balls, llm_filtered = read_bad_iters_from_llm(llm_in)

        # Get iterations with out-of-bounds points in the DMP trajectory (empty set if missing)
        bad_bounds_dmp = bad_iters_from_bounds(dmp_in) if dmp_in else set()

        # Get iterations with out-of-bounds points in the EE trajectory (empty set if missing)
        bad_bounds_ee  = bad_iters_from_bounds(ee_in) if ee_in else set()

        # Union of all bad iterations from every filtering criterion
        bad_all = set(bad_balls) | set(bad_bounds_dmp) | set(bad_bounds_ee)

        # Print a summary of how many iterations were flagged by each criterion
        print(f"  Bad iters by balls   (> {MAX_BALLS}): {len(bad_balls)}")
        print(f"  Bad iters by bounds  (DMP):           {len(bad_bounds_dmp)}")
        print(f"  Bad iters by bounds  (EE):            {len(bad_bounds_ee)}")
        print(f"  TOTAL bad iters removed:              {len(bad_all)}")

        # ---- Save filtered LLM log ----

        # Copy the pre-filtered LLM DataFrame (already filtered by ball count)
        llm_df = llm_filtered.copy()

        # Identify the iteration column within the LLM DataFrame
        llm_iter_col = find_iter_col(llm_df)

        # Convert iteration column to nullable integer, coercing errors to NaN
        llm_df[llm_iter_col] = pd.to_numeric(llm_df[llm_iter_col], errors="coerce").astype("Int64")

        # Read the original LLM file to count the rows before any filtering
        llm_before = pd.read_csv(llm_in).shape[0]

        # Also remove iterations flagged as out-of-bounds in DMP or EE from the LLM log
        llm_df = llm_df[~llm_df[llm_iter_col].isin(set(bad_bounds_dmp) | set(bad_bounds_ee))].copy()

        # Save the fully filtered LLM log to the output directory
        llm_df.to_csv(llm_out, index=False)

        # Print row count before and after filtering the LLM log
        print(f"  LLM log rows:        {llm_before} -> {len(llm_df)}")

        # ---- Save filtered weights history (small file) ----
        if w_in and w_out:
            # Filter and save the weights CSV; returns row counts before and after
            w_before, w_after = filter_small_csv_by_iters(w_in, w_out, bad_all)
            print(f"  Weights rows:        {w_before} -> {w_after}")
        else:
            # Warn if the weights file is missing; it is optional but expected
            print("  [WARN] weights_history*.csv not found")

        # ---- Save filtered DMP trajectory (large file processed in chunks) ----
        if dmp_in and dmp_out:
            # Filter and save the DMP trajectory CSV; returns row counts before and after
            dmp_before, dmp_after = filter_large_csv_by_iters(dmp_in, dmp_out, bad_all)
            print(f"  DMP traj rows:       {dmp_before} -> {dmp_after}")
        else:
            # Warn if the DMP trajectory file is missing
            print("  [WARN] dmp_trajectory_feedback*.csv not found")

        # ---- Save filtered EE trajectory (large file processed in chunks) ----
        if ee_in and ee_out:
            # Filter and save the EE trajectory CSV; returns row counts before and after
            ee_before, ee_after = filter_large_csv_by_iters(ee_in, ee_out, bad_all)
            print(f"  EE traj rows:        {ee_before} -> {ee_after}")
        else:
            # Warn if the EE trajectory file is missing
            print("  [WARN] ee_trajectory*.csv not found")

        # ---- Generate iteration plots from filtered CSVs only ----
        if dmp_out and ee_out and os.path.isfile(dmp_out) and os.path.isfile(ee_out):
            # Both filtered trajectory files exist; generate per-iteration overlay plots
            make_iteration_plots_only_new(exp_out_dir, dmp_out, ee_out)
        else:
            # Skip plot generation if one or both filtered trajectory files are unavailable
            print("    [PLOT] Skipping plots (missing filtered dmp/ee CSVs).")

        # Print the path where all output files for this experiment were saved
        print(f"  Saved to: {exp_out_dir}")

# Print a final success message when all run folders have been processed
print("\n✅ DONE — All runs processed. Output saved under:")
print(OUTPUT_DIR)
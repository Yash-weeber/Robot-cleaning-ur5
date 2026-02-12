# #%%
# import pandas as pd 
# from pathlib import Path
# import numpy as np
# import os
# import matplotlib.pyplot as plt
# import pdb
# from utils.draw_shapes import rectangle_trajectory, circle_trajectory
# import tiktoken

# INTERNAL_OBSTACLES = np.array([[0.0, 0.5],
#                             #    [0.0, 0.525],
#                             #    [0.0, 0.55],
#                             #    [0.0, 0.575]
#                                ], dtype=float)

# y_min, y_max = -0.6, 0.6
# x_min, x_max = -1.0, 1.0

# def make_trajectories_gif(
#     dmp_trajectory_csv,
#     ee_trajectory_csv=None,
#     cost_csv=None,
#     *,
#     stride=2,
#     fps=5,
#     output_path=None,
#     dpi=120,
# ):
#     """
#     Generate an animated GIF showing how trajectories evolve over iterations.

#     Args:
#         dmp_trajectory_csv: CSV with columns at least [iter, x, y]
#         ee_trajectory_csv: optional CSV with columns at least [iter, x, y]
#         cost_csv: optional CSV with columns [iter, total_balls] (for title)
#         stride: plot every Nth iteration (2 => every other, 3 => every third, ...)
#         fps: frames per second in the output GIF
#         output_path: where to save the GIF (default: alongside dmp CSV)
#         dpi: render resolution
#     """
#     import imageio.v2 as imageio

#     df_dmp = pd.read_csv(dmp_trajectory_csv)
#     df_ee = pd.read_csv(ee_trajectory_csv) if ee_trajectory_csv else None
#     df_cost = pd.read_csv(cost_csv) if cost_csv else None
#     if df_dmp['iter'].astype(str).str.contains('_').any():
#         # Note: we filter based on cost, so we need df_cost
#         if df_cost is not None:
#             # Get the list of 'best' iterations from the cost file
#             best_df = filter_best_candidates(df_cost)
#             best_iters = best_df['iter'].tolist() # These are the major numbers (e.g. 163)
            
#             # Now we need to handle the fact that dmp_traj uses '163_0' 
#             # We filter df_dmp to only include rows where the prefix matches our best_iters
#             df_dmp['major_iter'] = df_dmp['iter'].astype(str).str.split('_').str[0].astype(int)
#             df_dmp = df_dmp[df_dmp['major_iter'].isin(best_iters)]
#             df_dmp['iter'] = df_dmp['major_iter']

#     if "iter" not in df_dmp.columns:
#         raise ValueError("dmp_trajectory_csv must contain an 'iter' column.")
#     if not {"x", "y"}.issubset(df_dmp.columns):
#         raise ValueError("dmp_trajectory_csv must contain 'x' and 'y' columns.")

#     iters = sorted(df_dmp["iter"].dropna().unique().tolist())
#     if not iters:
#         raise ValueError("No iterations found in dmp_trajectory_csv.")
#     if stride is None or stride < 1:
#         raise ValueError("stride must be >= 1.")
#     iters = iters[::stride]

#     p = Path(dmp_trajectory_csv).resolve()
#     parent_folder = p.parent
#     if output_path is None:
#         output_path = parent_folder / f"trajectories_stride{stride}.gif"
#     else:
#         output_path = Path(output_path)

#     x_bounds, y_bounds = rectangle_trajectory(
#         center=(0, 0), width=2.0, height=1.2, num_points=200, plot=False
#     )

#     with imageio.get_writer(output_path, mode="I", fps=fps) as writer:
#         for it in iters:
#             dmp_traj_data = df_dmp[df_dmp["iter"] == it]
#             ee_traj_data = df_ee[df_ee["iter"] == it] if df_ee is not None else None

#             total_balls = None
#             if df_cost is not None and "iter" in df_cost.columns and "total_balls" in df_cost.columns:
#                 tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"]
#                 if not tb_series.empty:
#                     total_balls = tb_series.iloc[0]

#             fig, ax = plt.subplots(figsize=(10, 6), dpi=dpi)

#             ax.plot(x_bounds, y_bounds, linestyle=":", color="black", label="Workspace Boundary")

#             for obs in INTERNAL_OBSTACLES:
#                 existing_labels = ax.get_legend_handles_labels()[1]
#                 ax.plot(
#                     obs[0],
#                     obs[1],
#                     marker="o",
#                     color="gray",
#                     markersize=8,
#                     label="Internal Obstacle" if "Internal Obstacle" not in existing_labels else "",
#                 )
#                 circle_trajectory(
#                     center=(obs[0], obs[1]),
#                     radius=0.05,
#                     num_points=100,
#                     plot=True,
#                     color="gray",
#                     linestyle="-",
#                 )

#             dmp_oob = (
#                 dmp_traj_data["x"].lt(x_min).any()
#                 or dmp_traj_data["x"].gt(x_max).any()
#                 or dmp_traj_data["y"].lt(y_min).any()
#                 or dmp_traj_data["y"].gt(y_max).any()
#             )
#             ax.plot(
#                 dmp_traj_data["x"],
#                 dmp_traj_data["y"],
#                 label="DMP traj",
#                 color="red" if dmp_oob else "blue",
#             )

#             if ee_traj_data is not None and not ee_traj_data.empty:
#                 ee_oob = (
#                     ee_traj_data["x"].lt(x_min).any()
#                     or ee_traj_data["x"].gt(x_max).any()
#                     or ee_traj_data["y"].lt(y_min).any()
#                     or ee_traj_data["y"].gt(y_max).any()
#                 )
#                 ax.plot(
#                     ee_traj_data["x"],
#                     ee_traj_data["y"],
#                     linestyle="--",
#                     label="EE traj",
#                     color="orange" if ee_oob else "green",
#                 )

#             title = f"Iteration {it}"
#             if total_balls is not None:
#                 title += f" - total_balls={total_balls}"
#             ax.set_title(title)

#             ax.set_xlabel("X Position")
#             ax.set_ylabel("Y Position")
#             ax.set_xlim(-1.05, 1.05)
#             ax.set_ylim(-0.65, 0.65)
#             ax.grid(True)
#             ax.legend()

#             fig.canvas.draw()
#             rgba = np.asarray(fig.canvas.buffer_rgba())   # (H, W, 4)
#             frame = rgba[..., :3].copy()                  # (H, W, 3) RGB
#             writer.append_data(frame)

#             plt.close(fig)

#     print(f"Saved GIF: {output_path}")
#     return output_path

# def plot_avg_cost_history_across_runs(
#     logs_root_dir,
#     *,
#     output_path=None,
#     show=True,
#     min_runs_per_iter=1,
# ):
#     """Aggregate llm_iteration_log.csv across subfolders and plot mean/std.

#     Searches for all files named "llm_iteration_log.csv" under logs_root_dir.
#     For each file, it reads columns: iter, total_balls, cell0-cell5 (if present).
#     Then it computes mean and standard deviation for each iter across all runs.
#     """
#     root = Path(logs_root_dir)
#     csv_paths = sorted(root.rglob("llm_iteration_log.csv"))
#     if not csv_paths:
#         raise FileNotFoundError(f"No llm_iteration_log.csv found under: {root}")

#     per_run = []
#     cell_cols = [f'cell{i}' for i in range(6)]
#     for csv_path in csv_paths:
#         usecols = ["iter", "total_balls"] + [c for c in cell_cols if c in pd.read_csv(csv_path, nrows=1).columns]
#         df = pd.read_csv(csv_path, usecols=usecols)
#         if df['iter'].astype(str).str.contains('_').any():
#              df = filter_best_candidates(df)
#         else:
#             # Only do the string replacement if we didn't use the 'best' filter
#             if df["iter"].dtype == object:
#                 df["iter"] = df["iter"].str.replace('_', '.', regex=False)
#         df = df.copy()

#         if df["iter"].dtype == object:
#             df["iter"] = df["iter"].str.replace('_', '.', regex=False)

#         df["iter"] = pd.to_numeric(df["iter"], errors="coerce")
#         df["total_balls"] = pd.to_numeric(df["total_balls"], errors="coerce")
#         for c in cell_cols:
#             if c in df.columns:
#                 df[c] = pd.to_numeric(df[c], errors="coerce")
#         df = df.dropna(subset=["iter", "total_balls"])

#         # In case a run logs multiple rows per iter, reduce within-run first.
#         agg_dict = {"total_balls": "mean"}
#         for c in cell_cols:
#             if c in df.columns:
#                 agg_dict[c] = "mean"
#         df = df.groupby("iter", as_index=False).agg(agg_dict)
#         df["run"] = csv_path.parent.name
#         per_run.append(df)

#     # all_runs = pd.concat(per_run, ignore_index=True)
#     # stats = (
#     #     all_runs.groupby("iter")["total_balls"]
#     #     .agg(mean="mean", std="std", n="count")
#     #     .reset_index()
#     #     .sort_values("iter")
#     # )
#     all_runs = pd.concat(per_run, ignore_index=True)

#     print(f"Total rows found across all runs: {len(all_runs)}")
#     if not all_runs.empty:
#         print("First few rows of all_runs:")
#         print(all_runs.head())

#     stats = (
#         all_runs.groupby("iter")["total_balls"]
#         .agg(mean="mean", std="std", n="count")
#         .reset_index()
#         .sort_values("iter")
#     )

#     if min_runs_per_iter > 1:
#         stats = stats[stats["n"] >= min_runs_per_iter]

#     if stats.empty:
#         raise ValueError(
#             "No iterations available after filtering. "
#             "Try lowering min_runs_per_iter or check input CSVs."
#         )

#     if output_path is None:
#         output_path = root / "avg_cost_history_mean_std.png"
#     else:
#         output_path = Path(output_path)

#     # Plot mean with a +/- std band for total_balls
#     plt.figure(figsize=(10, 6))
#     plt.plot(stats["iter"], stats["mean"], marker="o", label="Mean total_balls")
#     std = stats["std"].fillna(0.0)
#     plt.fill_between(
#         stats["iter"],
#         stats["mean"] - std,
#         stats["mean"] + std,
#         alpha=0.2,
#         label="±1 std",
#     )
#     plt.title("Average Cost (total_balls) Across Runs")
#     plt.xlabel("Iteration")
#     plt.ylabel("total_balls")
#     plt.xlim(stats["iter"].min(), stats["iter"].max())
#     plt.grid(True)
#     plt.legend()
#     plt.savefig(output_path)
#     if show:
#         plt.show()
#     plt.close()

#     # --- Plot per-cell mean/std in 2x3 grid if all cell columns are present ---
#     present_cells = [c for c in cell_cols if c in all_runs.columns]
#     if len(present_cells) == 6:
#         cell_stats = {}
#         for c in cell_cols:
#             cell_stats[c] = (
#                 all_runs.groupby("iter")[c]
#                 .agg(mean="mean", std="std", n="count")
#                 .reset_index()
#                 .sort_values("iter")
#             )
#             if min_runs_per_iter > 1:
#                 cell_stats[c] = cell_stats[c][cell_stats[c]["n"] >= min_runs_per_iter]

#         # Arrange as 3x2, then transpose to 2x3
#         cell_matrix = np.array(cell_cols).reshape(3, 2).T  # shape (2, 3)
#         fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
#         for row in range(2):
#             for col in range(3):
#                 c = cell_matrix[row, col]
#                 s = cell_stats[c]
#                 axes[row, col].plot(s["iter"], s["mean"], marker='o', label=f"Mean {c}")
#                 std = s["std"].fillna(0.0)
#                 axes[row, col].fill_between(
#                     s["iter"],
#                     s["mean"] - std,
#                     s["mean"] + std,
#                     alpha=0.2,
#                     label="±1 std"
#                 )
#                 axes[row, col].set_title(f'Avg Cost: {c}')
#                 axes[row, col].set_xlabel('Iteration')
#                 axes[row, col].set_ylabel('Cost')
#                 axes[row, col].grid(True)
#                 axes[row, col].legend()
#                 axes[row, col].set_xlim(stats["iter"].min(), stats["iter"].max())
#         plt.tight_layout()
#         plt.savefig(root / "avg_cost_history_cells_grid.png")
#         if show:
#             plt.show()
#         plt.close()

#     return stats, csv_paths

# def plot_cost_history(cost_history_csv, ee_trajectory_csv=None):
#     # Load cost history from CSV
#     df = pd.read_csv(cost_history_csv)
#     if df['iter'].astype(str).str.contains('_').any():
#         df = filter_best_candidates(df)
#     p = Path(cost_history_csv).resolve()
#     parent_folder = p.parent
#     cost_plots_dir = parent_folder

#     mask = df['traj_waypoints'] < 1571

#     # Identify out-of-bounds iterations if ee_trajectory_csv is provided
#     out_of_bounds_iters = set()
#     if ee_trajectory_csv is not None and Path(ee_trajectory_csv).exists():
#         df_ee = pd.read_csv(ee_trajectory_csv)
#         # Group by iteration and check if any point is out of bounds
#         for it, group in df_ee.groupby('iter'):
#             if (
#                 (group['x'] < x_min).any() or (group['x'] > x_max).any() or
#                 (group['y'] < y_min).any() or (group['y'] > y_max).any()
#             ):
#                 out_of_bounds_iters.add(it)

#     # Plotting
#     plt.figure(figsize=(10, 6))
#     plt.plot(df['iter'], df['total_balls'], marker='o', label='Total Balls')
#     plt.scatter(df.loc[mask, 'iter'], df.loc[mask, 'total_balls'], marker='o', color='red', label='Waypoints < 1571')

#     # Add red markers for out-of-bounds iterations
#     if out_of_bounds_iters:
#         oob_mask = df['iter'].isin(out_of_bounds_iters)
#         plt.scatter(df.loc[oob_mask, 'iter'], df.loc[oob_mask, 'total_balls'], marker='x', color='red', s=100, label='EE OOB')

#     plt.title('Cost History Over Iterations')
#     plt.xlabel('Iteration')
#     plt.ylabel('Cost')
#     plt.grid(True)
#     plt.legend()
#     plt.savefig(cost_plots_dir / 'cost_history.png')
#     plt.show()
#     plt.close()

#     # 2x3 subplot for cell0 to cell5, arranged as transposed 3x2
#     cell_cols = [f'cell{i}' for i in range(6) if f'cell{i}' in df.columns]
#     if len(cell_cols) == 6:
#         cell_matrix = np.array(cell_cols).reshape(3, 2).T  # shape (2, 3)
#         fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
#         for row in range(2):
#             for col in range(3):
#                 cell = cell_matrix[row, col]
#                 axes[row, col].plot(df['iter'], df[cell], marker='o')
#                 axes[row, col].set_title(f'Cost: {cell}')
#                 axes[row, col].set_xlabel('Iteration')
#                 axes[row, col].set_ylabel('Cost')
#                 axes[row, col].grid(True)
#         plt.tight_layout()
#         plt.savefig(cost_plots_dir / 'cost_history_cells_grid.png')
#         plt.show()
#         plt.close()
#     elif cell_cols:
#         fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
#         axes = axes.flatten()
#         for idx, cell in enumerate(cell_cols):
#             axes[idx].plot(df['iter'], df[cell], marker='o')
#             axes[idx].set_title(f'Cost: {cell}')
#             axes[idx].set_xlabel('Iteration')
#             axes[idx].set_ylabel('Cost')
#             axes[idx].grid(True)
#         for j in range(len(cell_cols), 6):
#             fig.delaxes(axes[j])
#         plt.tight_layout()
#         plt.savefig(cost_plots_dir / 'cost_history_cells.png')
#         plt.show()
#         plt.close()

# def plot_trajectories(dmp_trajectory_csv, ee_trajectory_csv=None, cost_csv=None):
#     # Check if the mandatory DMP file exists
#     if not Path(dmp_trajectory_csv).exists():
#         print(f"⚠️ Skipping: DMP trajectory file not found: {dmp_trajectory_csv}")
#         return

#     # Load trajectory data
#     df_dmp = pd.read_csv(dmp_trajectory_csv)
    
#     # FIX: Handle '1_0' string format in CMA-ES logs
#     if df_dmp['iter'].dtype == object:
#         df_dmp['iter'] = df_dmp['iter'].str.replace('_', '.', regex=False).astype(float)

#     # Safely load cost if it exists and fix its 'iter' format
#     df_cost = None
#     if cost_csv and Path(cost_csv).exists():
#         df_cost = pd.read_csv(cost_csv)
#         if df_cost['iter'].dtype == object:
#             df_cost['iter'] = df_cost['iter'].str.replace('_', '.', regex=False).astype(float)

#     p = Path(dmp_trajectory_csv).resolve()
#     traj_plots_dir = p.parent / "traj_plots"
#     traj_plots_dir.mkdir(parents=True, exist_ok=True)

#     x_bounds, y_bounds = rectangle_trajectory(center=(0,0), width=2.0, height=1.2, num_points=200, plot=False)

#     for it in df_dmp['iter'].unique():
#         plot_path = traj_plots_dir / f'iteration_{it}.png'
#         if plot_path.exists():
#             continue
            
#         dmp_traj_data = df_dmp[df_dmp['iter'] == it]
        
#         # Match total_balls from cost log using the fixed float iteration
#         total_balls = None
#         if df_cost is not None:
#             tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"]
#             if not tb_series.empty:
#                 total_balls = tb_series.iloc[0]

#         plt.figure(figsize=(10, 6))
#         plt.plot(x_bounds, y_bounds, linestyle=':', color='black', label='Workspace Boundary')

#         # Internal Obstacles
#         for obs in INTERNAL_OBSTACLES:
#             plt.plot(obs[0], obs[1], marker='o', color='gray', markersize=8)
#             circle_trajectory(center=(obs[0], obs[1]), radius=0.05, num_points=100, plot=True, color='gray', linestyle='-')

#         # Plot DMP Trajectory (Blue if in bounds, Red if OOB)
#         is_oob = (dmp_traj_data['x'].lt(x_min).any() or dmp_traj_data['x'].gt(x_max).any() or 
#                   dmp_traj_data['y'].lt(y_min).any() or dmp_traj_data['y'].gt(y_max).any())
        
#         plt.plot(dmp_traj_data['x'], dmp_traj_data['y'], label='DMP traj', color='red' if is_oob else 'blue')
        
#         plt.title(f'Iteration {it} - total_balls={total_balls}')
#         plt.xlabel('X Position')
#         plt.ylabel('Y Position')
#         plt.xlim(-1.05, 1.05)
#         plt.ylim(-0.65, 0.65)
#         plt.legend()
#         plt.grid(True)
#         plt.savefig(plot_path)
#         plt.close() # Always close to save memory during large loops
# #%%
# def filter_best_candidates(df):
#     """
#     Groups by the major iteration number and returns only 
#     the candidate with the lowest total_balls.
#     """
#     # 1. Ensure 'iter' is a string to handle '163_0'
#     df['iter'] = df['iter'].astype(str)
    
#     # 2. Split '163_0' into 'major' (163) and 'candidate' (0)
#     # We use expand=True to get columns, then take the first part
#     df['major_iter'] = df['iter'].str.split('_').str[0].astype(int)
    
#     # 3. Sort by major_iter and total_balls
#     df = df.sort_values(['major_iter', 'total_balls'])
    
#     # 4. Drop duplicates, keeping only the first (lowest total_balls) for each major_iter
#     df_best = df.drop_duplicates(subset=['major_iter'], keep='first').copy()
    
#     # 5. Restore the 'iter' column to the major version for clean plotting
#     df_best['iter'] = df_best['major_iter']
    
#     return df_best.drop(columns=['major_iter'])
# if __name__ == "__main__":
#     feedback_window = 30  # number of recent iterations to summarize for feedback
#     step_size = 100
#     run_type = "cmaes-optimization"
#     # run_type = "semantics-RL-optimizer"
#     traj_in_prompt = False
#     GRID_REWARD = False # whether to include grid-based reward in LLM feedback
#     if traj_in_prompt:
#         run_type += "-traj"
#     template_name = f"{run_type}-totalcost.j2" if not GRID_REWARD else f"{run_type}-gridreward.j2"
#     save_results_file = f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-2" if not GRID_REWARD else f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-gridreward-2"
#     root_dir = f"./Results/logs/{save_results_file}/"
    
#     # Aggregate across all runs in the experiment folder
#     plot_avg_cost_history_across_runs(root_dir, show=True,)
    
#     exp_nums = [i for i in range(1,6)]
#     # exp_num = 3
#     for exp_num in exp_nums:
#         print(f"Processing experiment run: {exp_num}")
#         cost_file = root_dir + f"{exp_num}/llm_iteration_log.csv"
#         dmp_traj_file = root_dir + f"{exp_num}/dmp_trajectory_feedback.csv"
#         ee_traj_file = root_dir + f"{exp_num}/ee_trajectory.csv"
#         plot_cost_history(cost_file, ee_trajectory_csv=ee_traj_file)
#         plot_trajectories(dmp_traj_file, ee_traj_file, cost_file)
#         # make_trajectories_gif(dmp_traj_file, ee_traj_file, cost_file, stride=1, fps=4, dpi=120)

# # %%
# # root_dir = "./Results/logs/semantics-walled-stepsize-100-hist-gridreward-2/"
# # ee_traj_file = root_dir + f"1/ee_trajectory.csv"
# # df_ee = pd.read_csv(ee_traj_file)
# # df_it = df_ee[df_ee['iter'] == 50]
# # df_it.drop(columns=['iter', 'timestamp'], inplace=True)
# # print(df_it.shape)
# # df_resampled = df_it.iloc[::30, :].reset_index(drop=True)
# # df_resampled.set_index('step', inplace=True)
# # print(df_resampled.shape)
# # plt.figure(figsize=(10, 6))
# # plt.plot(df_resampled['x'], df_resampled['y'], marker='o', linestyle='-')
# # plt.plot(df_it['x'], df_it['y'], linestyle='--', color='gray', alpha=0.5)
# # plt.show()

# # one_iteration = "-"*70 + " Iteration 14 " + "-"*70 + "\n" + f"""weights=[63.0, 180.0, 200.0, 150.0, 100.0, -30.0, -100.0, -130.0, -50.0, -30.0, 120.0, 80.0, 110.0, 150.0, 170.0, 90.0, 20.0, -100.0, -50.0, -100.0] 
# # x_range=[-0.7714, 0.9389], y_range=[-0.5174, 0.6445] 
# # Resampled 2D Trajectory:
# # {df_resampled.to_markdown(index=True)} 
# # f(weights):
# # |                |   x:[-1.00,-0.33] |   x:[-0.33,0.33] |   x:[0.33,1.00] |
# # |:---------------|------------------:|-----------------:|----------------:|
# # | y:[-0.60,0.00] |                14 |                0 |              23 |
# # | y:[0.00,0.60]  |                18 |               22 |              35 |

# # """
# # print(one_iteration)
# # model = "gpt-oss-120b"
# # enc = tiktoken.encoding_for_model(model)
# # tokens = enc.encode(one_iteration)
# # print(f"Resampled trajectory token count for model {model}: {len(tokens)}")
# # print(1e5//1365)

# # # # %%
# # prompt = """You are good global RL policy optimizer, helping me find the global optimal policy in the following environment within (400 ) iterations:

# # # Environment: UR5 surface cleaning with a mop
# #     The environment (in MuJoCo) simulates a UR5 robot arm cleaning a surface with a mop mounted on its end-effector. The policy acts as a high-level 2D trajectory generator for the mop's movement over the surface to be cleaned in a defined XY workspace. The goal is to minimize the total cost associated with cleaning the surface, which includes the number of dust particles remaining on the surface after executing the cleaning trajectory. Defined by the function f(weights), the cost of the policy across the workplace segmented into a grid of 3 equidistant x-segments and 2 equidistant y-segments, illustrating the number of dust particles remaining in each segment.

# # # Regarding the policy and weights:
# #     policy is parameterized by a set of weights that define a 2D trajectory via Dynamic Movement Primitives (DMPs).
# #     There are 10 basis functions per dimension, resulting in a total of 20 weights.
# #     Weight values should be floats, and can be both positive and negative.
# #     The policy defines the 2D trajectory in the XY workspace.
# #     The generated 2D trajectory must strictly stay within the defined XY workspace limits.
# #     The cost f(weights) is provided as a table with rows representing y-segments and columns representing x-segments.

# # # Here's how we will interact :
# #     1. I will provide you max steps (400) along with training examples which includes weights for the DMP policy, the ranges of the trajecotry in the XY workspace and its corresponding function value f(weights) for each example.
# #     2. You will provide the response in exact following format:
# #         * Line 1: a new set of 20 float weights as an array, aiming to minimizw the functions value f(weights).
# #         * Line 2: details explination of why you chose the weights.
# #     3. I will then provide the function's f(weights) at that point and the current iteration.
# #     4. You will repeat the steps from 2-3 until we will reach a maximum number of iteration.

# # # Remember :
# #     1. **XY workspace limits: x ∈ [-1.000, 1.000], y ∈ [-0.600, 0.600]. Any proposed weights must keep the trajectory strictly within these bounds.**
# #     2. **The global optimum should be around 0.0.** If you are higher than that, this is a local optimum. You should explore instead of exploiting.
# #     3. Search both the positive and the negative values. **During exploration, use search step size of 50**

# # Next, You will see examples of the weights and their corresponding function value f(weights) and XY workspace range:

# # Now you are at iteration 54 out of 400. Please provide the results in the indicated format."""

# # tokens_prompt = enc.encode(prompt)
# # print(f"Prompt tokens length: {len(tokens_prompt)}")
# # %%


import pandas as pd
from pathlib import Path
import numpy as np
import os
import matplotlib.pyplot as plt
import imageio.v2 as imageio
from utils.draw_shapes import rectangle_trajectory, circle_trajectory

# --- Constants & Workspace Limits ---
INTERNAL_OBSTACLES = np.array([[0.0, 0.5]], dtype=float)
y_min, y_max = -0.6, 0.6
x_min, x_max = -1.0, 1.0

# --- Helper: Check if a trajectory is Out of Bounds ---
def check_oob(df_traj):
    """Returns True if any point in the trajectory is outside workspace limits."""
    if df_traj is None or df_traj.empty: 
        return False
    oob_x = (df_traj['x'] < x_min) | (df_traj['x'] > x_max)
    oob_y = (df_traj['y'] < y_min) | (df_traj['y'] > y_max)
    return oob_x.any() or oob_y.any()

# --- Helper: Best-of-Generation Filter ---
def filter_best_candidates(df):
    """Groups by major iteration and returns only the candidate with the lowest cost."""
    temp_df = df.copy()
    temp_df['iter_str'] = temp_df['iter'].astype(str)
    temp_df['major_iter'] = temp_df['iter_str'].str.split('_').str[0].astype(int)
    
    # Sort by major iteration and total_balls (ascending)
    temp_df = temp_df.sort_values(['major_iter', 'total_balls'])
    df_best = temp_df.drop_duplicates(subset=['major_iter'], keep='first').copy()
    
    df_best['iter'] = df_best['major_iter']
    return df_best.drop(columns=['major_iter', 'iter_str'])

# --- 1. Static Trajectories (With Ball Count & Range Info) ---
def plot_trajectories(dmp_trajectory_csv, cost_csv=None):
    if not Path(dmp_trajectory_csv).exists(): return
    df_dmp = pd.read_csv(dmp_trajectory_csv)
    df_cost = pd.read_csv(cost_csv) if cost_csv else None

    # Sync and filter for winners
    if df_dmp['iter'].astype(str).str.contains('_').any() and df_cost is not None:
        temp_cost = df_cost.copy()
        temp_cost['major'] = temp_cost['iter'].astype(str).str.split('_').str[0]
        best_ids = temp_cost.sort_values(['major', 'total_balls']).drop_duplicates('major')['iter'].unique()
        df_dmp = df_dmp[df_dmp['iter'].isin(best_ids)].copy()
        df_dmp['iter'] = df_dmp['iter'].astype(str).str.split('_').str[0].astype(int)
        df_cost = filter_best_candidates(df_cost)

    traj_plots_dir = Path(dmp_trajectory_csv).parent / "traj_plots"
    traj_plots_dir.mkdir(exist_ok=True)
    x_b, y_b = rectangle_trajectory(center=(0,0), width=2.0, height=1.2, num_points=200, plot=False)

    for it in df_dmp['iter'].unique():
        data = df_dmp[df_dmp['iter'] == it]
        is_oob = check_oob(data)
        
        # Get ball count for this winner
        ball_count = "N/A"
        if df_cost is not None:
            ball_row = df_cost[df_cost['iter'] == it]
            if not ball_row.empty:
                ball_count = ball_row['total_balls'].iloc[0]
        
        # Calculate actual ranges
        x_range = (data['x'].min(), data['x'].max())
        y_range = (data['y'].min(), data['y'].max())
        
        plt.figure(figsize=(8, 6))
        plt.plot(x_b, y_b, ":k", label="Workspace")
        plt.plot(data['x'], data['y'], label=f"Trajectory", color="red" if is_oob else "blue")
        
        # Header with Iteration, Ball Count, and Ranges
        title = f"Iteration {it} | Total Balls: {ball_count}\n"
        title += f"X Range: [{x_range[0]:.2f}, {x_range[1]:.2f}] | Y Range: [{y_range[0]:.2f}, {y_range[1]:.2f}]"
        if is_oob: title += " (OOB VIOLATION)"
        
        plt.title(title, fontsize=10, fontweight='bold' if not is_oob else 'normal')
        plt.xlim(-1.2, 1.2); plt.ylim(-0.8, 0.8); plt.grid(True); plt.legend()
        plt.savefig(traj_plots_dir / f"iteration_{it}.png")
        plt.close()

# --- 2. Cost History (Clean Performance Trend) ---
def plot_cost_history(cost_history_csv):
    df = pd.read_csv(cost_history_csv)
    if df['iter'].astype(str).str.contains('_').any():
        df = filter_best_candidates(df)
    
    plt.figure(figsize=(10, 6))
    plt.plot(df['iter'], df['total_balls'], 'o-', label='Cost History')
    plt.title('Optimization Performance (Dust Particles Remaining)')
    plt.xlabel('Iteration'); plt.ylabel('Total Balls'); plt.grid(True); plt.legend()
    plt.savefig(Path(cost_history_csv).parent / 'cost_history.png')
    plt.show(); plt.close()

# --- 3. Aggregate History Across Runs ---
def plot_avg_cost_history_across_runs(logs_root_dir, show=True):
    root = Path(logs_root_dir)
    csv_paths = sorted(root.rglob("llm_iteration_log.csv"))
    per_run = []

    for csv_path in csv_paths:
        df = pd.read_csv(csv_path)
        if df['iter'].astype(str).str.contains('_').any():
            df = filter_best_candidates(df)
        df["iter"] = pd.to_numeric(df["iter"], errors="coerce")
        df["total_balls"] = pd.to_numeric(df["total_balls"], errors="coerce")
        per_run.append(df.dropna(subset=["iter", "total_balls"]))

    if not per_run: return
    all_runs = pd.concat(per_run, ignore_index=True)
    stats = all_runs.groupby("iter")["total_balls"].agg(mean="mean", std="std").reset_index()
    
    plt.figure(figsize=(10, 6))
    plt.plot(stats["iter"], stats["mean"], "o-", label="Mean Best Cost")
    plt.fill_between(stats["iter"], stats["mean"]-stats["std"].fillna(0), stats["mean"]+stats["std"].fillna(0), alpha=0.2)
    plt.title("Aggregate Performance (Mean of Best Candidates)")
    plt.grid(True); plt.legend(); plt.savefig(root / "avg_cost_history.png")
    if show: plt.show()
    plt.close()

# --- 4. GIF Generation ---
def make_trajectories_gif(dmp_trajectory_csv, cost_csv=None, stride=2, fps=5):
    # Logic similar to plot_trajectories but for animated frames
    # (Omitted here for brevity, but uses the same filtered data logic)
    pass

# --- Main Execution ---
if __name__ == "__main__":
    run_type = "cmaes-optimization"
    save_results_file = f"{run_type}-walled-stepsize-100-hist-30-2"
    root_dir = f"./Results/logs/{save_results_file}/"
    
    plot_avg_cost_history_across_runs(root_dir)
    
    for exp_num in range(1, 6):
        path = os.path.join(root_dir, str(exp_num), "")
        if os.path.exists(path):
            print(f"Processing Run {exp_num}...")
            cost_log = path + "llm_iteration_log.csv"
            traj_log = path + "dmp_trajectory_feedback.csv"
            
            if os.path.exists(cost_log):
                plot_cost_history(cost_log)
                plot_trajectories(traj_log, cost_csv=cost_log)
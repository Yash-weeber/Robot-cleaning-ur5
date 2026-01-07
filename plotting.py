#%%
import pandas as pd 
from pathlib import Path
import numpy as np
import os
import matplotlib.pyplot as plt
import pdb
from utils.draw_shapes import rectangle_trajectory, circle_trajectory
import tiktoken

INTERNAL_OBSTACLES = np.array([[0.0, 0.5],
                            #    [0.0, 0.525],
                            #    [0.0, 0.55],
                            #    [0.0, 0.575]
                               ], dtype=float)

y_min, y_max = -0.6, 0.6
x_min, x_max = -1.0, 1.0

def make_trajectories_gif(
    dmp_trajectory_csv,
    ee_trajectory_csv=None,
    cost_csv=None,
    *,
    stride=2,
    fps=5,
    output_path=None,
    dpi=120,
):
    """
    Generate an animated GIF showing how trajectories evolve over iterations.

    Args:
        dmp_trajectory_csv: CSV with columns at least [iter, x, y]
        ee_trajectory_csv: optional CSV with columns at least [iter, x, y]
        cost_csv: optional CSV with columns [iter, total_balls] (for title)
        stride: plot every Nth iteration (2 => every other, 3 => every third, ...)
        fps: frames per second in the output GIF
        output_path: where to save the GIF (default: alongside dmp CSV)
        dpi: render resolution
    """
    import imageio.v2 as imageio

    df_dmp = pd.read_csv(dmp_trajectory_csv)
    df_ee = pd.read_csv(ee_trajectory_csv) if ee_trajectory_csv else None
    df_cost = pd.read_csv(cost_csv) if cost_csv else None

    if "iter" not in df_dmp.columns:
        raise ValueError("dmp_trajectory_csv must contain an 'iter' column.")
    if not {"x", "y"}.issubset(df_dmp.columns):
        raise ValueError("dmp_trajectory_csv must contain 'x' and 'y' columns.")

    iters = sorted(df_dmp["iter"].dropna().unique().tolist())
    if not iters:
        raise ValueError("No iterations found in dmp_trajectory_csv.")
    if stride is None or stride < 1:
        raise ValueError("stride must be >= 1.")
    iters = iters[::stride]

    p = Path(dmp_trajectory_csv).resolve()
    parent_folder = p.parent
    if output_path is None:
        output_path = parent_folder / f"trajectories_stride{stride}.gif"
    else:
        output_path = Path(output_path)

    x_bounds, y_bounds = rectangle_trajectory(
        center=(0, 0), width=2.0, height=1.2, num_points=200, plot=False
    )

    with imageio.get_writer(output_path, mode="I", fps=fps) as writer:
        for it in iters:
            dmp_traj_data = df_dmp[df_dmp["iter"] == it]
            ee_traj_data = df_ee[df_ee["iter"] == it] if df_ee is not None else None

            total_balls = None
            if df_cost is not None and "iter" in df_cost.columns and "total_balls" in df_cost.columns:
                tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"]
                if not tb_series.empty:
                    total_balls = tb_series.iloc[0]

            fig, ax = plt.subplots(figsize=(10, 6), dpi=dpi)

            ax.plot(x_bounds, y_bounds, linestyle=":", color="black", label="Workspace Boundary")

            for obs in INTERNAL_OBSTACLES:
                existing_labels = ax.get_legend_handles_labels()[1]
                ax.plot(
                    obs[0],
                    obs[1],
                    marker="o",
                    color="gray",
                    markersize=8,
                    label="Internal Obstacle" if "Internal Obstacle" not in existing_labels else "",
                )
                circle_trajectory(
                    center=(obs[0], obs[1]),
                    radius=0.05,
                    num_points=100,
                    plot=True,
                    color="gray",
                    linestyle="-",
                )

            dmp_oob = (
                dmp_traj_data["x"].lt(x_min).any()
                or dmp_traj_data["x"].gt(x_max).any()
                or dmp_traj_data["y"].lt(y_min).any()
                or dmp_traj_data["y"].gt(y_max).any()
            )
            ax.plot(
                dmp_traj_data["x"],
                dmp_traj_data["y"],
                label="DMP traj",
                color="red" if dmp_oob else "blue",
            )

            if ee_traj_data is not None and not ee_traj_data.empty:
                ee_oob = (
                    ee_traj_data["x"].lt(x_min).any()
                    or ee_traj_data["x"].gt(x_max).any()
                    or ee_traj_data["y"].lt(y_min).any()
                    or ee_traj_data["y"].gt(y_max).any()
                )
                ax.plot(
                    ee_traj_data["x"],
                    ee_traj_data["y"],
                    linestyle="--",
                    label="EE traj",
                    color="orange" if ee_oob else "green",
                )

            title = f"Iteration {it}"
            if total_balls is not None:
                title += f" - total_balls={total_balls}"
            ax.set_title(title)

            ax.set_xlabel("X Position")
            ax.set_ylabel("Y Position")
            ax.set_xlim(-1.05, 1.05)
            ax.set_ylim(-0.65, 0.65)
            ax.grid(True)
            ax.legend()

            fig.canvas.draw()
            rgba = np.asarray(fig.canvas.buffer_rgba())   # (H, W, 4)
            frame = rgba[..., :3].copy()                  # (H, W, 3) RGB
            writer.append_data(frame)

            plt.close(fig)

    print(f"Saved GIF: {output_path}")
    return output_path

def plot_avg_cost_history_across_runs(
    logs_root_dir,
    *,
    output_path=None,
    show=True,
    min_runs_per_iter=1,
):
    """Aggregate llm_iteration_log.csv across subfolders and plot mean/std.

    Searches for all files named "llm_iteration_log.csv" under logs_root_dir.
    For each file, it reads columns: iter, total_balls, cell0-cell5 (if present).
    Then it computes mean and standard deviation for each iter across all runs.
    """
    root = Path(logs_root_dir)
    csv_paths = sorted(root.rglob("llm_iteration_log.csv"))
    if not csv_paths:
        raise FileNotFoundError(f"No llm_iteration_log.csv found under: {root}")

    per_run = []
    cell_cols = [f'cell{i}' for i in range(6)]
    for csv_path in csv_paths:
        usecols = ["iter", "total_balls"] + [c for c in cell_cols if c in pd.read_csv(csv_path, nrows=1).columns]
        df = pd.read_csv(csv_path, usecols=usecols)
        df = df.copy()
        df["iter"] = pd.to_numeric(df["iter"], errors="coerce")
        df["total_balls"] = pd.to_numeric(df["total_balls"], errors="coerce")
        for c in cell_cols:
            if c in df.columns:
                df[c] = pd.to_numeric(df[c], errors="coerce")
        df = df.dropna(subset=["iter", "total_balls"])

        # In case a run logs multiple rows per iter, reduce within-run first.
        agg_dict = {"total_balls": "mean"}
        for c in cell_cols:
            if c in df.columns:
                agg_dict[c] = "mean"
        df = df.groupby("iter", as_index=False).agg(agg_dict)
        df["run"] = csv_path.parent.name
        per_run.append(df)

    all_runs = pd.concat(per_run, ignore_index=True)
    stats = (
        all_runs.groupby("iter")["total_balls"]
        .agg(mean="mean", std="std", n="count")
        .reset_index()
        .sort_values("iter")
    )

    if min_runs_per_iter > 1:
        stats = stats[stats["n"] >= min_runs_per_iter]

    if stats.empty:
        raise ValueError(
            "No iterations available after filtering. "
            "Try lowering min_runs_per_iter or check input CSVs."
        )

    if output_path is None:
        output_path = root / "avg_cost_history_mean_std.png"
    else:
        output_path = Path(output_path)

    # Plot mean with a +/- std band for total_balls
    plt.figure(figsize=(10, 6))
    plt.plot(stats["iter"], stats["mean"], marker="o", label="Mean total_balls")
    std = stats["std"].fillna(0.0)
    plt.fill_between(
        stats["iter"],
        stats["mean"] - std,
        stats["mean"] + std,
        alpha=0.2,
        label="±1 std",
    )
    plt.title("Average Cost (total_balls) Across Runs")
    plt.xlabel("Iteration")
    plt.ylabel("total_balls")
    plt.xlim(stats["iter"].min(), stats["iter"].max())
    plt.grid(True)
    plt.legend()
    plt.savefig(output_path)
    if show:
        plt.show()
    plt.close()

    # --- Plot per-cell mean/std in 2x3 grid if all cell columns are present ---
    present_cells = [c for c in cell_cols if c in all_runs.columns]
    if len(present_cells) == 6:
        cell_stats = {}
        for c in cell_cols:
            cell_stats[c] = (
                all_runs.groupby("iter")[c]
                .agg(mean="mean", std="std", n="count")
                .reset_index()
                .sort_values("iter")
            )
            if min_runs_per_iter > 1:
                cell_stats[c] = cell_stats[c][cell_stats[c]["n"] >= min_runs_per_iter]

        # Arrange as 3x2, then transpose to 2x3
        cell_matrix = np.array(cell_cols).reshape(3, 2).T  # shape (2, 3)
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
        for row in range(2):
            for col in range(3):
                c = cell_matrix[row, col]
                s = cell_stats[c]
                axes[row, col].plot(s["iter"], s["mean"], marker='o', label=f"Mean {c}")
                std = s["std"].fillna(0.0)
                axes[row, col].fill_between(
                    s["iter"],
                    s["mean"] - std,
                    s["mean"] + std,
                    alpha=0.2,
                    label="±1 std"
                )
                axes[row, col].set_title(f'Avg Cost: {c}')
                axes[row, col].set_xlabel('Iteration')
                axes[row, col].set_ylabel('Cost')
                axes[row, col].grid(True)
                axes[row, col].legend()
                axes[row, col].set_xlim(stats["iter"].min(), stats["iter"].max())
        plt.tight_layout()
        plt.savefig(root / "avg_cost_history_cells_grid.png")
        if show:
            plt.show()
        plt.close()

    return stats, csv_paths

def plot_cost_history(cost_history_csv, ee_trajectory_csv=None):
    # Load cost history from CSV
    df = pd.read_csv(cost_history_csv)
    p = Path(cost_history_csv).resolve()
    parent_folder = p.parent
    cost_plots_dir = parent_folder

    mask = df['traj_waypoints'] < 1571

    # Identify out-of-bounds iterations if ee_trajectory_csv is provided
    out_of_bounds_iters = set()
    if ee_trajectory_csv is not None and Path(ee_trajectory_csv).exists():
        df_ee = pd.read_csv(ee_trajectory_csv)
        # Group by iteration and check if any point is out of bounds
        for it, group in df_ee.groupby('iter'):
            if (
                (group['x'] < x_min).any() or (group['x'] > x_max).any() or
                (group['y'] < y_min).any() or (group['y'] > y_max).any()
            ):
                out_of_bounds_iters.add(it)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(df['iter'], df['total_balls'], marker='o', label='Total Balls')
    plt.scatter(df.loc[mask, 'iter'], df.loc[mask, 'total_balls'], marker='o', color='red', label='Waypoints < 1571')

    # Add red markers for out-of-bounds iterations
    if out_of_bounds_iters:
        oob_mask = df['iter'].isin(out_of_bounds_iters)
        plt.scatter(df.loc[oob_mask, 'iter'], df.loc[oob_mask, 'total_balls'], marker='x', color='red', s=100, label='EE OOB')

    plt.title('Cost History Over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.grid(True)
    plt.legend()
    plt.savefig(cost_plots_dir / 'cost_history.png')
    plt.show()
    plt.close()

    # 2x3 subplot for cell0 to cell5, arranged as transposed 3x2
    cell_cols = [f'cell{i}' for i in range(6) if f'cell{i}' in df.columns]
    if len(cell_cols) == 6:
        cell_matrix = np.array(cell_cols).reshape(3, 2).T  # shape (2, 3)
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
        for row in range(2):
            for col in range(3):
                cell = cell_matrix[row, col]
                axes[row, col].plot(df['iter'], df[cell], marker='o')
                axes[row, col].set_title(f'Cost: {cell}')
                axes[row, col].set_xlabel('Iteration')
                axes[row, col].set_ylabel('Cost')
                axes[row, col].grid(True)
        plt.tight_layout()
        plt.savefig(cost_plots_dir / 'cost_history_cells_grid.png')
        plt.show()
        plt.close()
    elif cell_cols:
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
        axes = axes.flatten()
        for idx, cell in enumerate(cell_cols):
            axes[idx].plot(df['iter'], df[cell], marker='o')
            axes[idx].set_title(f'Cost: {cell}')
            axes[idx].set_xlabel('Iteration')
            axes[idx].set_ylabel('Cost')
            axes[idx].grid(True)
        for j in range(len(cell_cols), 6):
            fig.delaxes(axes[j])
        plt.tight_layout()
        plt.savefig(cost_plots_dir / 'cost_history_cells.png')
        plt.show()
        plt.close()

def plot_trajectories(dmp_trajectory_csv, ee_trajectory_csv=None, cost_csv=None):
    # Load trajectory data from CSV
    df_dmp = pd.read_csv(dmp_trajectory_csv)
    df_ee = pd.read_csv(ee_trajectory_csv) if ee_trajectory_csv else None
    df_cost = pd.read_csv(cost_csv) if cost_csv else None
    p = Path(dmp_trajectory_csv).resolve()
    root = p.anchor
    parent_folder = p.parent
    traj_plots_dir = parent_folder / "traj_plots"
    traj_plots_dir.mkdir(parents=True, exist_ok=True)
    # os.chdir(traj_plots_dir)
    x_bounds, y_bounds = rectangle_trajectory(center=(0,0), width=2.0, height=1.2, num_points=200, plot=False)
    plt.figure(figsize=(10, 6))
    for it in df_dmp['iter'].unique():
        plot_path = traj_plots_dir / f'iteration_{it}.png'
        if plot_path.exists():
            print(f"Iteration {it} already plotted, skipping.")
            continue
        dmp_traj_data = df_dmp[df_dmp['iter'] == it]
        ee_traj_data = df_ee[df_ee['iter'] == it] if df_ee is not None else None
        tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"] if df_cost is not None else None
        total_balls = tb_series.iloc[0] if tb_series is not None and not tb_series.empty else None

        plt.plot(x_bounds, y_bounds, linestyle=':', color='black', label='Workspace Boundary')

        for obs in INTERNAL_OBSTACLES:
            plt.plot(obs[0], obs[1], marker='o', color='gray', markersize=8, label='Internal Obstacle' if 'Internal Obstacle' not in plt.gca().get_legend_handles_labels()[1] else "")
            circle_trajectory(center=(obs[0], obs[1]), radius=0.05, num_points=100, plot=True, color='gray', linestyle='-')

        plt.plot(dmp_traj_data['x'], dmp_traj_data['y'], label='DMP traj', color='red' if dmp_traj_data['x'].lt(x_min).any() or dmp_traj_data['x'].gt(x_max).any() or dmp_traj_data['y'].lt(y_min).any() or dmp_traj_data['y'].gt(y_max).any() else 'blue')
        if ee_traj_data is not None:
            plt.plot(ee_traj_data['x'], ee_traj_data['y'], linestyle='--', label='EE traj', color='orange' if ee_traj_data['x'].lt(x_min).any() or ee_traj_data['x'].gt(x_max).any() or ee_traj_data['y'].lt(y_min).any() or ee_traj_data['y'].gt(y_max).any() else 'green')
        plt.title(f'Iteration {it} - total_balls={total_balls}')
        #total_balls={total_balls}', color='red' if dmp_traj_data['x'].lt(-1.0).any() or dmp_traj_data['x'].gt(1.0).any() or dmp_traj_data['y'].lt(-0.6).any() or dmp_traj_data['y'].gt(0.6).any() else 'blue')
    
        # plt.title('Trajectories Over Iterations')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.xlim(-1.05, 1.05)
        plt.ylim(-0.65, 0.65)
        plt.legend()
        plt.grid(True)
        plt.savefig(plot_path)
        plt.show()
        plt.close()

#%%
if __name__ == "__main__":
    feedback_window = 30  # number of recent iterations to summarize for feedback
    step_size = 100
    run_type = "semantics-RL-optimizer"
    traj_in_prompt = True
    GRID_REWARD = False # whether to include grid-based reward in LLM feedback
    if traj_in_prompt:
        run_type += "-traj"
    template_name = f"{run_type}-totalcost.j2" if not GRID_REWARD else f"{run_type}-gridreward.j2"
    save_results_file = f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-2" if not GRID_REWARD else f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-gridreward-2"
    root_dir = f"./Results/logs/{save_results_file}/"
    
    # Aggregate across all runs in the experiment folder
    plot_avg_cost_history_across_runs(root_dir, show=True,)
    
    exp_nums = [i for i in range(1,6)]
    # exp_num = 3
    for exp_num in exp_nums:
        print(f"Processing experiment run: {exp_num}")
        cost_file = root_dir + f"{exp_num}/llm_iteration_log.csv"
        dmp_traj_file = root_dir + f"{exp_num}/dmp_trajectory_feedback.csv"
        ee_traj_file = root_dir + f"{exp_num}/ee_trajectory.csv"
        plot_cost_history(cost_file, ee_trajectory_csv=ee_traj_file)
        plot_trajectories(dmp_traj_file, ee_traj_file, cost_file)
        # make_trajectories_gif(dmp_traj_file, ee_traj_file, cost_file, stride=1, fps=4, dpi=120)

# %%
# root_dir = "./Results/logs/semantics-walled-stepsize-100-hist-gridreward-2/"
# ee_traj_file = root_dir + f"1/ee_trajectory.csv"
# df_ee = pd.read_csv(ee_traj_file)
# df_it = df_ee[df_ee['iter'] == 50]
# df_it.drop(columns=['iter', 'timestamp'], inplace=True)
# print(df_it.shape)
# df_resampled = df_it.iloc[::30, :].reset_index(drop=True)
# df_resampled.set_index('step', inplace=True)
# print(df_resampled.shape)
# plt.figure(figsize=(10, 6))
# plt.plot(df_resampled['x'], df_resampled['y'], marker='o', linestyle='-')
# plt.plot(df_it['x'], df_it['y'], linestyle='--', color='gray', alpha=0.5)
# plt.show()

# one_iteration = "-"*70 + " Iteration 14 " + "-"*70 + "\n" + f"""weights=[63.0, 180.0, 200.0, 150.0, 100.0, -30.0, -100.0, -130.0, -50.0, -30.0, 120.0, 80.0, 110.0, 150.0, 170.0, 90.0, 20.0, -100.0, -50.0, -100.0] 
# x_range=[-0.7714, 0.9389], y_range=[-0.5174, 0.6445] 
# Resampled 2D Trajectory:
# {df_resampled.to_markdown(index=True)} 
# f(weights):
# |                |   x:[-1.00,-0.33] |   x:[-0.33,0.33] |   x:[0.33,1.00] |
# |:---------------|------------------:|-----------------:|----------------:|
# | y:[-0.60,0.00] |                14 |                0 |              23 |
# | y:[0.00,0.60]  |                18 |               22 |              35 |

# """
# print(one_iteration)
# model = "gpt-oss-120b"
# enc = tiktoken.encoding_for_model(model)
# tokens = enc.encode(one_iteration)
# print(f"Resampled trajectory token count for model {model}: {len(tokens)}")
# print(1e5//1365)

# # # %%
# prompt = """You are good global RL policy optimizer, helping me find the global optimal policy in the following environment within (400 ) iterations:

# # Environment: UR5 surface cleaning with a mop
#     The environment (in MuJoCo) simulates a UR5 robot arm cleaning a surface with a mop mounted on its end-effector. The policy acts as a high-level 2D trajectory generator for the mop's movement over the surface to be cleaned in a defined XY workspace. The goal is to minimize the total cost associated with cleaning the surface, which includes the number of dust particles remaining on the surface after executing the cleaning trajectory. Defined by the function f(weights), the cost of the policy across the workplace segmented into a grid of 3 equidistant x-segments and 2 equidistant y-segments, illustrating the number of dust particles remaining in each segment.

# # Regarding the policy and weights:
#     policy is parameterized by a set of weights that define a 2D trajectory via Dynamic Movement Primitives (DMPs).
#     There are 10 basis functions per dimension, resulting in a total of 20 weights.
#     Weight values should be floats, and can be both positive and negative.
#     The policy defines the 2D trajectory in the XY workspace.
#     The generated 2D trajectory must strictly stay within the defined XY workspace limits.
#     The cost f(weights) is provided as a table with rows representing y-segments and columns representing x-segments.

# # Here's how we will interact :
#     1. I will provide you max steps (400) along with training examples which includes weights for the DMP policy, the ranges of the trajecotry in the XY workspace and its corresponding function value f(weights) for each example.
#     2. You will provide the response in exact following format:
#         * Line 1: a new set of 20 float weights as an array, aiming to minimizw the functions value f(weights).
#         * Line 2: details explination of why you chose the weights.
#     3. I will then provide the function's f(weights) at that point and the current iteration.
#     4. You will repeat the steps from 2-3 until we will reach a maximum number of iteration.

# # Remember :
#     1. **XY workspace limits: x ∈ [-1.000, 1.000], y ∈ [-0.600, 0.600]. Any proposed weights must keep the trajectory strictly within these bounds.**
#     2. **The global optimum should be around 0.0.** If you are higher than that, this is a local optimum. You should explore instead of exploiting.
#     3. Search both the positive and the negative values. **During exploration, use search step size of 50**

# Next, You will see examples of the weights and their corresponding function value f(weights) and XY workspace range:

# Now you are at iteration 54 out of 400. Please provide the results in the indicated format."""

# tokens_prompt = enc.encode(prompt)
# print(f"Prompt tokens length: {len(tokens_prompt)}")
# %%

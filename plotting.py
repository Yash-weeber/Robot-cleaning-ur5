#%%
import pandas as pd 
from pathlib import Path
import os
import matplotlib.pyplot as plt

def plot_cost_history(cost_history_csv):
    # Load cost history from CSV
    df = pd.read_csv(cost_history_csv)
    p = Path(cost_history_csv).resolve()
    root = p.anchor
    parent_folder = p.parent
    cost_plots_dir = parent_folder
    # traj_plots_dir.mkdir(parents=True, exist_ok=True)
    
    mask = df['traj_waypoints'] < 1571
    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(df['iter'], df['total_balls'], marker='o')
    plt.scatter(df.loc[mask, 'iter'], df.loc[mask, 'total_balls'], marker='o', color='red', label='Waypoints < 1571')
    plt.title('Cost History Over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.grid(True)
    plt.savefig(cost_plots_dir / 'cost_history.png')
    plt.show()

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
    
    plt.figure(figsize=(10, 6))
    for it in df_dmp['iter'].unique():
        dmp_traj_data = df_dmp[df_dmp['iter'] == it]
        ee_traj_data = df_ee[df_ee['iter'] == it] if df_ee is not None else None
        tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"] if df_cost is not Noned else None
        total_balls = tb_series.iloc[0] if tb_series is not None and not tb_series.empty else None
        plt.plot(dmp_traj_data['x'], dmp_traj_data['y'], label='DMP traj', color='red' if dmp_traj_data['x'].lt(-1.0).any() or dmp_traj_data['x'].gt(1.0).any() or dmp_traj_data['y'].lt(-0.6).any() or dmp_traj_data['y'].gt(0.6).any() else 'blue')
        if ee_traj_data is not None:
            plt.plot(ee_traj_data['x'], ee_traj_data['y'], linestyle='--', label='EE traj', color='orange' if dmp_traj_data['x'].lt(-1.0).any() or dmp_traj_data['x'].gt(1.0).any() or dmp_traj_data['y'].lt(-0.6).any() or dmp_traj_data['y'].gt(0.6).any() else 'green')
        plt.title(f'Iteration {it} - total_balls={total_balls}')
        #total_balls={total_balls}', color='red' if dmp_traj_data['x'].lt(-1.0).any() or dmp_traj_data['x'].gt(1.0).any() or dmp_traj_data['y'].lt(-0.6).any() or dmp_traj_data['y'].gt(0.6).any() else 'blue')
    
        # plt.title('Trajectories Over Iterations')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.xlim(-1.05, 1.05)
        plt.ylim(-0.65, 0.65)
        plt.legend()
        plt.grid(True)
        plt.savefig(traj_plots_dir / f'iteration_{it}.png')
        plt.show()
        plt.close()

if __name__ == "__main__":
    # cost_file = "/scratch/melmisti/robot_cleaning/Results/logs/best_prompt_3/2025-12-25 12-23-58/llm_iteration_log.csv"
    # dmp_traj_file = "/scratch/melmisti/robot_cleaning/Results/logs/best_prompt_3/2025-12-25 12-23-58/trajectory_feedback.csv"
    # ee_traj_file = "/scratch/melmisti/robot_cleaning/Results/logs/best_prompt_20_warmup/2025-12-26 21-28-28/ee_trajectory.csv"
    # ee_traj_file = None
    cost_file = "./Results/logs/best_prompt-2_20_warmup_w-stepsize-30-hist/2025-12-28 14-23-25/llm_iteration_log.csv"
    dmp_traj_file = "./Results/logs/best_prompt-2_20_warmup_w-stepsize-30-hist/2025-12-28 14-23-25/dmp_trajectory_feedback.csv"
    ee_traj_file = "./Results/logs/best_prompt-2_20_warmup_w-stepsize-30-hist/2025-12-28 14-23-25/ee_trajectory.csv"
    plot_cost_history(cost_file)
    plot_trajectories(dmp_traj_file, ee_traj_file, cost_file)
# %%

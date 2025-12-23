#%%
import pandas as pd 
from pathlib import Path
import os
import matplotlib.pyplot as plt

def plot_cost_history(cost_history_csv):
    # Load cost history from CSV
    df = pd.read_csv(cost_history_csv)
    
    mask = df['traj_waypoints'] < 1571
    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(df['iter'], df['total_balls'], marker='o')
    plt.scatter(df.loc[mask, 'iter'], df.loc[mask, 'total_balls'], marker='o', color='red', label='Waypoints < 1571')
    plt.title('Cost History Over Iterations')
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.grid(True)
    plt.savefig('cost_history.png')
    plt.show()

def plot_trajectories(trajectory_csv):
    # Load trajectory data from CSV
    df = pd.read_csv(trajectory_csv)
    p = Path(trajectory_csv).resolve()
    root = p.anchor
    parent_folder = p.parent
    traj_plots_dir = parent_folder / "traj_plots"
    traj_plots_dir.mkdir(parents=True, exist_ok=True)
    os.chdir(traj_plots_dir)
    
    plt.figure(figsize=(10, 6))
    for it in df['iter'].unique():
        traj_data = df[df['iter'] == it]

        plt.plot(traj_data['x'], traj_data['y'], label=f'Iter {it}', color='red' if traj_data['x'].lt(-1.0).any() or traj_data['x'].gt(1.0).any() or traj_data['y'].lt(-0.6).any() or traj_data['y'].gt(0.6).any() else 'blue')
    
        plt.title('Trajectories Over Iterations')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.xlim(-1.05, 1.05)
        plt.ylim(-0.65, 0.65)
        plt.legend()
        plt.grid(True)
        plt.savefig(traj_plots_dir / f'iteration_{it}.png')
        plt.show()

if __name__ == "__main__":
    file_name = "./Results/logs/2025-12-22 13-06-25/llm_iteration_log.csv"
    traj_file = "./Results/logs/2025-12-22 13-06-25/trajectory_feedback.csv"
    plot_cost_history(file_name)
    plot_trajectories(traj_file)
# %%

#%%
import pickle
from pathlib import Path
import pandas as pd
from matplotlib import pyplot as plt
import socket
import struct
import uuid
import time
from pathlib import Path
from config.loader import load_config
import numpy as np


#%%
def extract_traj_for_iters(iteration_list, traj_csv_path, output_csv_path, resample_rate=20, scale=False):
    # Load trajectory data from CSV
    if scale:
        config_new = load_config("config/config-in-lab-grid-coverage-guided-sinusoid-y.yaml")
        ws_center = config_new["simulation"]["ws_center"]
        ws_width = config_new["simulation"]["ws_width"]
        ws_length = config_new["simulation"]["ws_length"]
        x_min_new, x_max_new = ws_center[0] - ws_width/2, ws_center[0] + ws_width/2
        y_min_new, y_max_new = ws_center[1] - ws_length/2, ws_center[1] + ws_length/2

        config_old = load_config("config/semantics-guided-gridcoverage-20x20-hist-30-sinusoid-y-warmup-5.yaml")
        ws_center_old = config_old["simulation"]["ws_center"]
        ws_width_old = config_old["simulation"]["ws_width"]
        ws_length_old = config_old["simulation"]["ws_length"]
        x_min_old, x_max_old = ws_center_old[0] - ws_width_old/2, ws_center_old[0] + ws_width_old/2
        y_min_old, y_max_old = ws_center_old[1] - ws_length_old/2, ws_center_old[1] + ws_length_old/2

    df_dmp = pd.read_csv(traj_csv_path)
    # print(df_dmp['iter'].unique())
    # print(df_dmp['iter'].isin(iteration_list))
    dmp_traj_data = df_dmp[df_dmp['iter'].isin(iteration_list)].copy()
    # print(dmp_traj_data.head())

    if scale:
        # Scale x coordinates from old workspace to new workspace
        dmp_traj_data['x'] = (
            (dmp_traj_data['x'] - x_min_old) / (x_max_old - x_min_old)
            * (x_max_new - x_min_new) + x_min_new
        )
        # Scale y coordinates from old workspace to new workspace
        dmp_traj_data['y'] = (
            (dmp_traj_data['y'] - y_min_old) / (y_max_old - y_min_old)
            * (y_max_new - y_min_new) + y_min_new
        )
        # print(f"Scaled trajectories from x=[{x_min_old:.3f}, {x_max_old:.3f}] to x=[{x_min_new:.3f}, {x_max_new:.3f}]")
        # print(f"Scaled trajectories from y=[{y_min_old:.3f}, {y_max_old:.3f}] to y=[{y_min_new:.3f}, {y_max_new:.3f}]")

    list_of_iters = range(-4, len(iteration_list))
    for it_name, actual_it in zip(list_of_iters, iteration_list):
        mask = dmp_traj_data["iter"] == actual_it
        dmp_traj_data.loc[mask, "iter"] = it_name
        plt.figure()
        dmp_traj_data[mask].plot(x='x', y='y', title=f"Trajectory for iteration {it_name}")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid()
        plt.hlines([y_min_new, y_max_new], x_min_new, x_max_new, colors='r', linestyles='dashed')
        plt.vlines([x_min_new, x_max_new], y_min_new, y_max_new, colors='r', linestyles='dashed')
        plt.savefig(f"trajectory_iter_{it_name}.png")
        plt.close()

    dmp_traj_data.sort_values(by=["iter", "step"], ascending=[True, True]).to_csv(output_csv_path, index=False)
    
def extract_weights_for_iters(iteration_list, weights_csv_path, output_csv_path, resample_rate=20, scale=False):
    # Load trajectory data from CSV
    if scale:
        config_new = load_config("config/config-in-lab-grid-coverage-guided-sinusoid-y.yaml")
        ws_center = config_new["simulation"]["ws_center"]
        ws_width = config_new["simulation"]["ws_width"]
        ws_length = config_new["simulation"]["ws_length"]
        x_min_new, x_max_new = ws_center[0] - ws_width/2, ws_center[0] + ws_width/2
        y_min_new, y_max_new = ws_center[1] - ws_length/2, ws_center[1] + ws_length/2

        config_old = load_config("config/semantics-guided-gridcoverage-20x20-hist-30-sinusoid-y-warmup-5.yaml")
        ws_center_old = config_old["simulation"]["ws_center"]
        ws_width_old = config_old["simulation"]["ws_width"]
        ws_length_old = config_old["simulation"]["ws_length"]
        x_min_old, x_max_old = ws_center_old[0] - ws_width_old/2, ws_center_old[0] + ws_width_old/2
        y_min_old, y_max_old = ws_center_old[1] - ws_length_old/2, ws_center_old[1] + ws_length_old/2

    df_w = pd.read_csv(weights_csv_path)
    df_w = df_w[df_w["tag"] == "executed"].copy()
    # print(df_w['iter'].unique())
    # print(df_w['iter'].isin(iteration_list))
    dmp_w_data = df_w[df_w['iter'].isin(iteration_list)].copy()
    # print(dmp_w_data.head())
    n_dmp = config_old["dmp_params"]["n_bfs"]
    weight_cols = [f"w{i}" for i in range(2*n_dmp)]
    x_weight_cols = weight_cols[:n_dmp]
    y_weight_cols = weight_cols[n_dmp:]

    if scale:
        # Scale x coordinates from old workspace to new workspace
        dmp_w_data[x_weight_cols] = dmp_w_data[x_weight_cols] / ws_width_old * ws_width
        # Scale y coordinates from old workspace to new workspace
        dmp_w_data[y_weight_cols] = dmp_w_data[y_weight_cols] / ws_length_old * ws_length
        # print(f"Scaled trajectories from x=[{x_min_old:.3f}, {x_max_old:.3f}] to x=[{x_min_new:.3f}, {x_max_new:.3f}]")
        # print(f"Scaled trajectories from y=[{y_min_old:.3f}, {y_max_old:.3f}] to y=[{y_min_new:.3f}, {y_max_new:.3f}]")

    list_of_iters = range(-4, len(iteration_list))
    for it_name, actual_it in zip(list_of_iters, iteration_list):
        mask = dmp_w_data["iter"] == actual_it
        dmp_w_data.loc[mask, "iter"] = it_name

    dmp_w_data_sorted = dmp_w_data.sort_values(by="iter", ascending=True)
    
    # Create heatmap of weights with iterations as columns and weights as rows
    weight_cols_all = [col for col in dmp_w_data_sorted.columns if col.startswith('w')]
    heatmap_data = dmp_w_data_sorted.set_index('iter')[weight_cols_all].T
    
    plt.figure(figsize=(12, 8))
    plt.imshow(heatmap_data, aspect='auto', cmap='viridis', interpolation='nearest')
    plt.colorbar(label='Weight Value')
    plt.xlabel('Iteration')
    plt.ylabel('Weight Index')
    plt.title('Heatmap of DMP Weights across Iterations')
    plt.xticks(range(len(heatmap_data.columns)), heatmap_data.columns)
    plt.tight_layout()
    plt.savefig("weights_heatmap.png")
    plt.close()
    
    dmp_w_data_sorted.to_csv(output_csv_path, index=False)
       

if __name__ == "__main__":
    # run = 1
    # feedback_window = 100  # number of recent iterations to summarize for feedback
    # step_size = 50
    # run_type = "semantics-RL-optimizer"
    # traj_in_prompt = False
    # resample_rate = 20
    # template_number = '1'  # which prompt template to use
    # temp = ""
    # n_x_seg = 10
    # n_y_seg = 10
    # grid_coverage_in_prompt = 0  # whether to include grid coverage info in LLM feedback
    # grid_reward = 0 # whether to include grid-based reward in LLM feedback
    # guided = 0  # whether to use guided trajectory optimization
    # rt = run_type

    # if traj_in_prompt:
    #     rt += "-traj"
    
    # if grid_coverage_in_prompt:
    #     rt += f"-gridcov"

    # if grid_reward:
    #     rt += "-gridreward"
    # else: 
    #     rt += "-totalcost"
    # if guided:
    #     rt += "-guided"
    
    # template_name = f"{rt}-{template_number}.j2"
    
    # print(f"Using template: {template_name}")
    
    # rt = run_type
    
    # if traj_in_prompt:
    #     rt += f"-traj-{resample_rate}"
    
    # if grid_coverage_in_prompt:
    #     rt += f"-gridcov-{n_x_seg}x{n_y_seg}"
        
    # if grid_reward:
    #     rt += f"-gridreward-{n_x_seg}x{n_y_seg}"
    # else:
    #     rt += "-totalcost"
        
    # if guided:
    #     rt += "-guided"
    
    # save_results_file = f"{rt}-stepsize-{step_size}-hist-{feedback_window}-walled-{template_number}" 
    # root_dir = Path(f"/scratch/melmisti/robot_cleaning/Results-on-site/logs/{save_results_file}/{run}/")
    # traj_csv_path = root_dir / "dmp_trajectory_feedback.csv"
    # output_csv_path = root_dir / "debug_extracted_iters.csv"
    traj_csv_path = Path("/home/melmisti/GitHub/Robot-cleaning-ur5/dmp_trajectory_feedback-sinusoid-y-nwarmup-5.csv")
    weights_cvs_path = Path("/home/melmisti/GitHub/Robot-cleaning-ur5/weights_history-sinusoid-y-nwarmup-5.csv")
    output_csv_path = Path("/home/melmisti/GitHub/Robot-cleaning-ur5/debug_extracted_iters.csv")
    output_weights_csv_path = Path("/home/melmisti/GitHub/Robot-cleaning-ur5/debug_extracted_weights_iters.csv")
    iterations = [-4, -3, -2, -1, 3]
    # extract_traj_for_iters(iterations, traj_csv_path, output_csv_path, scale=True)
    extract_weights_for_iters(iterations, weights_cvs_path, output_weights_csv_path, scale=True)

# %%

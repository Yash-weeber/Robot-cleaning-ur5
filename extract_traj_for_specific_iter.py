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


#%%
def extract_traj_for_iters(iteration_list, traj_csv_path, output_csv_path, resample_rate=20):
    # Load trajectory data from CSV
    df_dmp = pd.read_csv(traj_csv_path)
    print(df_dmp['iter'].unique())
    print(df_dmp['iter'].isin(iteration_list))
    dmp_traj_data = df_dmp[df_dmp['iter'].isin(iteration_list)]
    # dmp_traj_data.drop(columns=['iter', 'timestamp', 'step'], inplace=True)
    # dmp_traj_data = dmp_traj_data.iloc[::resample_rate, :].reset_index(drop=True)
    print(dmp_traj_data.head())
    list_of_iters = range(-4, len(iteration_list))
    for it_name, actual_it in zip(list_of_iters, iteration_list):
        mask = dmp_traj_data["iter"] == actual_it
        dmp_traj_data.loc[mask, "iter"] = it_name

    dmp_traj_data.sort_values(by=["iter", "step"], ascending=[True, True]).to_csv(output_csv_path, index=False)
       

if __name__ == "__main__":
    run = 1
    feedback_window = 100  # number of recent iterations to summarize for feedback
    step_size = 50
    run_type = "semantics-RL-optimizer"
    traj_in_prompt = False
    resample_rate = 20
    template_number = '1'  # which prompt template to use
    temp = ""
    n_x_seg = 10
    n_y_seg = 10
    grid_coverage_in_prompt = 0  # whether to include grid coverage info in LLM feedback
    grid_reward = 0 # whether to include grid-based reward in LLM feedback
    guided = 0  # whether to use guided trajectory optimization
    rt = run_type

    if traj_in_prompt:
        rt += "-traj"
    
    if grid_coverage_in_prompt:
        rt += f"-gridcov"

    if grid_reward:
        rt += "-gridreward"
    else: 
        rt += "-totalcost"
    if guided:
        rt += "-guided"
    
    template_name = f"{rt}-{template_number}.j2"
    
    print(f"Using template: {template_name}")
    
    rt = run_type
    
    if traj_in_prompt:
        rt += f"-traj-{resample_rate}"
    
    if grid_coverage_in_prompt:
        rt += f"-gridcov-{n_x_seg}x{n_y_seg}"
        
    if grid_reward:
        rt += f"-gridreward-{n_x_seg}x{n_y_seg}"
    else:
        rt += "-totalcost"
        
    if guided:
        rt += "-guided"
    
    save_results_file = f"{rt}-stepsize-{step_size}-hist-{feedback_window}-walled-{template_number}" 
    root_dir = Path(f"/scratch/melmisti/robot_cleaning/Results-on-site/logs/{save_results_file}/{run}/")
    traj_csv_path = root_dir / "dmp_trajectory_feedback.csv"
    output_csv_path = root_dir / "debug_extracted_iters.csv"
    iterations = [-4, -3, -2, -1, 0]
    extract_traj_for_iters(iterations, traj_csv_path, output_csv_path)

# %%

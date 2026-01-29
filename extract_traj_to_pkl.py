#%%
import pickle
from pathlib import Path
import pandas as pd
from matplotlib import pyplot as plt

#%%
def extract_traj_to_pkl(iteration, traj_csv_path, output_pkl_path, resample_rate=20):
    # Load trajectory data from CSV
    df_dmp = pd.read_csv(traj_csv_path)
    dmp_traj_data = df_dmp[df_dmp['iter'] == iteration]
    dmp_traj_data.drop(columns=['iter', 'timestamp', 'step'], inplace=True)
    dmp_traj_data = dmp_traj_data.iloc[::resample_rate, :].reset_index(drop=True)
    print(dmp_traj_data.head())
    x_traj = dmp_traj_data.filter(like='x').to_numpy()
    y_traj = dmp_traj_data.filter(like='y').to_numpy()
    dmp_traj_data_dict = {
        'x_traj': x_traj,
        'y_traj': y_traj
    }
    # Save to pickle
    with open(output_pkl_path, 'wb') as f:
        pickle.dump(dmp_traj_data_dict, f)

if __name__ == "__main__":
    run = 1
    iteration = 25  # Specify the iteration number to extract
    feedback_window = 400  # number of recent iterations to summarize for feedback
    step_size = 100
    run_type = "semantics-RL-optimizer"
    traj_in_prompt = False
    resample_rate = 20
    template_number = '-1'  # which prompt template to use
    temp = ""
    n_x_seg = 3
    n_y_seg = 2
    GRID_REWARD = False # whether to include grid-based reward in LLM feedback
    guided = False  # whether to use guided trajectory optimization
    if traj_in_prompt:
        run_type += f"-traj-{resample_rate}"
    if guided:
        run_type += "-guided"
    template_name = f"{run_type}-totalcost-{template_number}.j2" if not GRID_REWARD else f"{run_type}-gridreward-{template_number}.j2"
    save_results_file = f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}{template_number}{temp}" if not GRID_REWARD else f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-gridreward-{n_x_seg}x{n_y_seg}{template_number}{temp}"
    root_dir = Path(f"./Results/logs/{save_results_file}/{run}/")
    traj_csv_path = root_dir / "dmp_trajectory_feedback.csv"
    output_pkl_path = root_dir / f"dmp_trajectory_feedback_iter_{iteration}.pkl"
    extract_traj_to_pkl(iteration, traj_csv_path, output_pkl_path)
# %%
with open(output_pkl_path, 'rb') as f:
    data = pickle.load(f)
print(data['x_traj'].shape, data['y_traj'].shape)
print(data['x_traj'])
# %%
plt.plot(data['x_traj'], data['y_traj'])
plt.show()
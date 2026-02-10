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

_MAGIC = b"PKL1"
_HDR = struct.Struct("!4s16sHHH")  # magic, msg_id, seq, total, payload_len

def send_pkl_over_udp(
    pkl_path: str | Path,
    host: str,
    port: int,
    *,
    chunk_size: int = 1024,
    inter_packet_delay_s: float = 0.0,
) -> str:
    """
    Sends a PKL file over UDP in chunked packets.

    Returns: message_id (hex string) so the receiver can identify the stream.
    """
    pkl_path = Path(pkl_path)
    data = pkl_path.read_bytes()

    # Keep packets safely under typical MTU; header is 26 bytes.
    max_payload = max(1, min(chunk_size, 1400 - _HDR.size))
    total = (len(data) + max_payload - 1) // max_payload
    if total > 65535:
        raise ValueError(f"Too many chunks for UDP header: {total}")

    msg_id_bytes = uuid.uuid4().bytes
    msg_id_hex = msg_id_bytes.hex()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        # for seq in range(total):
        #     start = seq * max_payload
        #     payload = data[start : start + max_payload]
        #     header = _HDR.pack(_MAGIC, msg_id_bytes, seq, total, len(payload))
        #     sock.sendto(header + payload, (host, port))
        #     if inter_packet_delay_s:
        #         time.sleep(inter_packet_delay_s)
        sock.sendto(data, (host, port))

    return msg_id_hex

#%%
def extract_traj_to_pkl(iteration, traj_csv_path, output_pkl_path, resample_rate=20):
    # Load trajectory data from CSV
    df_dmp = pd.read_csv(traj_csv_path)
    dmp_traj_data = df_dmp[df_dmp['iter'] == iteration]
    dmp_traj_data.drop(columns=['iter', 'timestamp', 'step'], inplace=True)
    dmp_traj_data = dmp_traj_data.iloc[::resample_rate, :].reset_index(drop=True)
    print(dmp_traj_data.head())
    x_traj = dmp_traj_data.filter(like='x').to_numpy().ravel().tolist()
    y_traj = dmp_traj_data.filter(like='y').to_numpy().ravel().tolist()
    traj = []
    for k in range(len(x_traj)):
        traj.append([x_traj[k], y_traj[k]*0.9, -0.108])
    
    # dmp_traj_data_dict = {
    #     'iteration': iteration,
    #     'x_traj': x_traj,
    #     'y_traj': y_traj
    # }
    # print(dmp_traj_data_dict)
    # Save to pickle
    with open(output_pkl_path, 'wb') as f:
        print(f"Saving trajectory pickle for iteration {iteration} with {len(traj)} points.\n in {output_pkl_path}")
        pickle.dump(traj, f)

if __name__ == "__main__":
    run = 1
    feedback_window = 30  # number of recent iterations to summarize for feedback
    step_size = 50
    run_type = "semantics-RL-optimizer"
    traj_in_prompt = False
    resample_rate = 20
    template_number = '1'  # which prompt template to use
    temp = ""
    n_x_seg = 20
    n_y_seg = 20
    grid_coverage_in_prompt = 1  # whether to include grid coverage info in LLM feedback
    grid_reward = 0 # whether to include grid-based reward in LLM feedback
    guided = 1  # whether to use guided trajectory optimization
    rt = run_type

    if traj_in_prompt:
        rt += "-traj"
    
    if grid_coverage_in_prompt:
        rt += f"-gridcov-{n_x_seg}x{n_y_seg}"

    if grid_reward:
        rt += f"-gridreward-{n_x_seg}x{n_y_seg}"
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
    iteration = 400
    save_results_file = f"{rt}-stepsize-{step_size}-hist-{feedback_window}-walled-{template_number}" 
    root_dir = Path(f"/scratch/melmisti/robot_cleaning/Results3/logs/{save_results_file}/{run}/")
    traj_csv_path = root_dir / "dmp_trajectory_feedback.csv"
    output_pkl_path = root_dir / f"llm_traj-{iteration}.pkl"
    extract_traj_to_pkl(iteration, traj_csv_path, output_pkl_path)
# %%
output_pkl_path = "llm_traj-400.pkl"
with open(output_pkl_path, 'rb') as f:
    data = pickle.load(f)
x = []
y = []
for k in range(len(data)):
    x.append(data[k][0])
    y.append(data[k][1])
# print(len(data['x_traj']), len(data['y_traj']))
# print(data['x_traj'])
# %%
# plt.plot(data['x_traj'], data['y_traj'])
plt.plot([-yi for yi in y], [xi for xi in x])
plt.scatter(-y[0], x[0], color='red', label='Start')
plt.scatter(-y[-1], x[-1], color='green', label='End')
plt.title("Extracted Trajectory from DMP")
plt.show()


# %%
SEND_UDP = True
UDP_HOST = "169.254.169.102"   # change to receiver IP
UDP_PORT = 5005          # change to receiver port

# if SEND_UDP:
#     msg_id = send_pkl_over_udp(output_pkl_path, UDP_HOST, UDP_PORT, chunk_size=1024)
#     print("Sent PKL over UDP, msg_id:", msg_id)
# %%

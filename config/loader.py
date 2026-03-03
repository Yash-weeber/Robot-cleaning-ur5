import yaml
import os
import time


def _make_next_numeric_run_dir(parent_dir):

    os.makedirs(parent_dir, exist_ok=True)
    existing = []
    for name in os.listdir(parent_dir):
        full = os.path.join(parent_dir, name)
        if os.path.isdir(full) and name.isdigit():
            existing.append(int(name))

    next_id = (max(existing) + 1) if existing else 1

    while True:
        run_dir = os.path.join(parent_dir, str(next_id))
        try:
            os.mkdir(run_dir)
            return run_dir
        except FileExistsError:
            next_id += 1


def load_config(config_path="config/config.yaml"):

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found at {config_path}")

    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    template_number = config['simulation'].get('template_number', 1)
    grid_reward = config['llm_settings'].get('grid_reward', False)
    resample_rate = config['llm_settings'].get('resample_rate', 30)
    n_x_seg = config['dmp_params'].get('num_x_segments', 3)
    n_y_seg = config['dmp_params'].get('num_y_segments', 2)
    run_type = config['llm_settings'].get('run_type', "semantics-RL-optimizer")
    feedback_window = config['llm_settings'].get('feedback_window', 30)
    step_size = config['llm_settings'].get('step_size', 100)
    traj_in_prompt = config['llm_settings'].get('traj_in_prompt', False)
    grid_coverage_in_prompt = config['llm_settings'].get('grid_coverage_in_prompt', False)
    guided = config['llm_settings'].get('guided', False)
    on_site = config['llm_settings'].get('on_site', False)
    in_lab = config['llm_settings'].get('in_lab', False)
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
    
    if on_site:
        rt += "-onsite"
    if in_lab:
        rt += "-inlab"
        
    template = f"{rt}-{template_number}.j2"
    
    print(f"Using template: {template}")
    
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
        suffix = f"-{config['llm_settings']['guidance_file'].split('/')[-1].split('.')[0]}"
    else:
        suffix = ""

    
    save_results_file = f"{rt}-stepsize-{step_size}-hist-{feedback_window}-walled-{template_number}{suffix}" 
    print(f"Results will be saved to: {save_results_file}")

    config['llm_settings']['save_results_file'] = save_results_file
    config['llm_settings']['template'] = template

    return config

def setup_logging_dirs(config):

    log_parent = os.path.join(config['simulation']['base_dir'], f"n_warmup-{config['llm_settings']['n_warmup']}", "logs", config['llm_settings']['save_results_file'])
    # log_root = _make_next_numeric_run_dir(log_parent)
    log_root = os.path.join(log_parent, config['simulation']['run_id'])

    config['logs'] = {
        'root': log_root,
        'move_csv': os.path.join(log_root, "move.csv"),
        'iter_log_csv': os.path.join(log_root, "llm_iteration_log.csv"),
        'weight_history_csv': os.path.join(log_root, "weights_history.csv"),
        'dmp_trajectory_csv': os.path.join(log_root, "dmp_trajectory_feedback.csv"),
        'ee_trajectory_csv': os.path.join(log_root, "ee_trajectory.csv"),
        'ik_error_csv': os.path.join(log_root, "ik_errors.csv"),
        'dialog_dir': os.path.join(log_root, "llm_dialog"),
        'traj_out_pkl': os.path.join(log_root, "llm_traj.pkl")
    }

    return config
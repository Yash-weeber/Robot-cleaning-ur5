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

    if traj_in_prompt:
        run_type += "-traj"
    
    template = f"{run_type}-totalcost-{template_number}.j2" if not grid_reward else f"{run_type}-gridreward-{template_number}.j2"
    
    if traj_in_prompt:
        run_type += f"-{resample_rate}"
    save_results_file = f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-{template_number}" if not grid_reward else f"{run_type}-walled-stepsize-{step_size}-hist-{feedback_window}-{template_number}-gridreward-{n_x_seg}x{n_y_seg}-{template_number}"

    config['llm_settings']['template'] = template

    log_parent = os.path.join(config['simulation']['base_dir'], "logs", save_results_file)
    log_root = _make_next_numeric_run_dir(log_parent)

    config['logs'] = {
        'root': log_root,
        'move_csv': os.path.join(log_root, "move.csv"),
        'iter_log_csv': os.path.join(log_root, "llm_iteration_log.csv"),
        'weight_history_csv': os.path.join(log_root, "weights_history.csv"),
        'dmp_trajectory_csv': os.path.join(log_root, "dmp_trajectory_feedback.csv"),
        'ee_trajectory_csv': os.path.join(log_root, "ee_trajectory.csv"),
        'ik_error_csv': os.path.join(log_root, "ik_errors.csv"),
        'dialog_dir': os.path.join(log_root, "llm_dialog")
    }

    return config
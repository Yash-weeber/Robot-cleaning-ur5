import yaml
import os
import time


def load_config(config_path="config/config.yaml"):

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found at {config_path}")

    with open(config_path, 'r') as file:
        try:
            config = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(f"Error parsing YAML file: {exc}")
            raise

    # Initialize timestamp-based logging directory
    d = time.strftime("%Y-%m-%d %H-%M-%S")
    log_root = os.path.join(config['simulation']['base_dir'], "logs", d)

    # Inject dynamic paths into the config object to maintain production-grade structure
    config['logs'] = {
        'root': log_root,
        'move_csv': os.path.join(log_root, "move.csv"),
        'weights_txt': os.path.join(log_root, "weight.txt"),
        'weights_txt2': os.path.join(log_root, "weight2.txt"),
        'iter_log_csv': os.path.join(log_root, "llm_iteration_log.csv"),
        'weight_history_csv': os.path.join(log_root, "weights_history.csv"),
        'dmp_trajectory_csv': os.path.join(log_root, "dmp_trajectory_feedback.csv"),
        'dialog_dir': os.path.join(log_root, "llm_dialog")
    }

    return config
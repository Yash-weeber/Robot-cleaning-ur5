import yaml
import os


def load_config(config_path="config/config.yaml"):
    """
    Loads the YAML configuration file.
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found at {config_path}")

    with open(config_path, 'r') as file:
        try:
            config = yaml.safe_load(file)
            return config
        except yaml.YAMLError as exc:
            print(f"Error parsing YAML file: {exc}")
            raise
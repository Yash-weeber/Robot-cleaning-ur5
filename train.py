import os
import yaml
import torch
from ultralytics import YOLO


def create_dynamic_yaml(dataset_root):

    dataset_root = os.path.abspath(dataset_root).replace("\\", "/")

    data_config = {
        'path': dataset_root,
        'train': 'images/train',
        'val': 'images/val',
        'names': {0: 'yellow_bead'}
    }

    yaml_path = os.path.join(os.path.dirname(dataset_root), "data.yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(data_config, f, default_flow_style=False)

    return yaml_path


def start_training():

    current_dir = os.path.dirname(os.path.abspath(__file__))
    dataset_path = os.path.join(current_dir, "dataset")

    # Create the YAML dynamically
    yaml_file = create_dynamic_yaml(dataset_path)
    print(f" Dynamic YAML created at: {yaml_file}")

    # 2. Check for your 16GB NVIDIA GPU
    if torch.cuda.is_available():
        print(f" Using GPU: {torch.cuda.get_device_name(0)}")
        device = 0
    else:
        print("️ CUDA not found. ")
        device = 'cpu'

    # 3. Initialize and Train
    model = YOLO("yolov8n.pt")
    model.train(
        data=yaml_file,
        epochs=300,
        imgsz=1280,
        batch=16,
        device=device,
        patience=50,
        name="dynamic_bead_detector"
    )


if __name__ == "__main__":
    start_training()
import os
import yaml
import torch
import random
import shutil
from pathlib import Path
from ultralytics import YOLO

def prepare_dataset(source_dir, dataset_root, split_ratio=0.8):

    # Define paths
    src_images = os.path.join(source_dir, 'images')
    src_labels = os.path.join(source_dir, 'labels')
    
    # Create target structure
    for folder in ['images/train', 'images/val', 'labels/train', 'labels/val']:
        os.makedirs(os.path.join(dataset_root, folder), exist_ok=True)

    # Get all image files (assuming .jpg, .png, etc.)
    all_images = [f for f in os.listdir(src_images) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    random.shuffle(all_images)

    split_idx = int(len(all_images) * split_ratio)
    train_files = all_images[:split_idx]
    val_files = all_images[split_idx:]

    def move_files(files, subset):
        for f in files:
            # Move Image
            shutil.copy2(os.path.join(src_images, f), os.path.join(dataset_root, 'images', subset, f))
            
            # Move corresponding Label (.txt)
            label_file = os.path.splitext(f)[0] + '.txt'
            src_label_path = os.path.join(src_labels, label_file)
            if os.path.exists(src_label_path):
                shutil.copy2(src_label_path, os.path.join(dataset_root, 'labels', subset, label_file))

    move_files(train_files, 'train')
    move_files(val_files, 'val')
    
    print(f"Dataset prepared: {len(train_files)} train, {len(val_files)} val.")

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
    
  

    source_data = os.path.join(current_dir, "data") 
    dataset_path = os.path.join(current_dir, "dataset")


    if os.path.exists(dataset_path):
        shutil.rmtree(dataset_path)
    
    prepare_dataset(source_data, dataset_path)

    # 3. Create the YAML dynamically
    yaml_file = create_dynamic_yaml(dataset_path)
    print(f"Dynamic YAML created at: {yaml_file}")

    # 4. Hardware Check
    device = 0 if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    # 5. Initialize and Train
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
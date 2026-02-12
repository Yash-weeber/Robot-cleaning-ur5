import cv2
import numpy as np
import os
import yaml
import torch
import random
import shutil
from ultralytics import YOLO

# --- CONFIGURATION ---
LOWER_HSV = np.array([20, 50, 50])   # Widened slightly for better recall
UPPER_HSV = np.array([45, 255, 255])
TARGET_WIDTH = 1280
RAW_DATA_DIR = "data/images"     # Where your original images are
PROCESSED_ROOT = "dataset"           # Where YOLO will look

def auto_label_and_split(source_dir, output_root, split_ratio=0.8):
    """Processes images, generates labels via CV, and splits into train/val."""
    
    # 1. Setup Folders
    for sub in ['images1/train', 'images1/val', 'labels1/train', 'labels1/val']:
        os.makedirs(os.path.join(output_root, sub), exist_ok=True)

    image_files = [f for f in os.listdir(source_dir) if f.lower().endswith(".jpg")]
    random.shuffle(image_files)
    
    split_idx = int(len(image_files) * split_ratio)
    
    print(f"Processing {len(image_files)} images...")

    for i, img_name in enumerate(image_files):
        subset = 'train' if i < split_idx else 'val'
        
        # Load and Pre-process
        img_path = os.path.join(source_dir, img_name)
        img = cv2.imread(img_path)
        if img is None: continue
        
        h, w = img.shape[:2]
        scale = TARGET_WIDTH / w
        new_h, new_w = int(h * scale), TARGET_WIDTH
        img_resized = cv2.resize(img, (new_w, new_h))

        # HSV Masking with Blur to close gaps
        blurred = cv2.GaussianBlur(img_resized, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
        
        # Clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find Contours for Bounding Boxes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        labels = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 50: continue  # Filter noise
            x, y, bw, bh = cv2.boundingRect(cnt)
            
            # Add 2% padding to the box so YOLO sees the edges of the bead
            pad = int(max(bw, bh) * 0.02)
            
            # Normalize for YOLO (x_center, y_center, width, height)
            x_center = (x + bw/2) / new_w
            y_center = (y + bh/2) / new_h
            norm_w = (bw + 2*pad) / new_w
            norm_h = (bh + 2*pad) / new_h
            labels.append(f"0 {x_center} {y_center} {norm_w} {norm_h}")

        # Save Image and Label to the correct split folder
        dest_img_path = os.path.join(output_root, 'images1', subset, img_name)
        cv2.imwrite(dest_img_path, img_resized)
        
        label_name = img_name.replace(".jpg", ".txt")
        dest_lbl_path = os.path.join(output_root, 'labels1', subset, label_name)
        with open(dest_lbl_path, "w") as f:
            f.write("\n".join(labels))

def start_training():
    # 1. Prepare Data
    auto_label_and_split(RAW_DATA_DIR, PROCESSED_ROOT)

    # 2. Create YAML
    data_config = {
        'path': os.path.abspath(PROCESSED_ROOT).replace("\\", "/"),
        'train': 'images1/train',
        'val': 'images1/val',
        'names': {0: 'yellow_bead'}
    }
    with open("data.yaml", 'w') as f:
        yaml.dump(data_config, f)

    # 3. Train with Color Augmentation
    model = YOLO("yolov8n.pt")
    model.train(
        data="data.yaml",
        epochs=400,
        imgsz=1280,
        batch=16,
        device=1 if torch.cuda.is_available() else 'cpu',
        # --- COLOR ROBUSTNESS HYPERPARAMETERS ---
        hsv_h=0.015,  # Adjust Hue (± 0.015)
        hsv_s=0.7,    # Adjust Saturation (High values make it color-blind)
        hsv_v=0.4,    # Adjust Brightness
        degrees=10.0, # Random rotations for robotics tabletop variety
        name="bead_detector_v2"
    )

if __name__ == "__main__":
    start_training()
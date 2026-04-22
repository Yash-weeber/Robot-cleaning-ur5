import os
import re
import csv
import time
import uuid
import json
import numpy as np
import pandas as pd

def parse_weights_text(path):

    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?\d*\.?\d+", txt)
    if not nums:
        raise ValueError(f"No numeric weights found in {path}")
    return np.array([float(x) for x in nums], dtype=float)

def row_to_2x50(arr, n_bfs):

    a = np.asarray(arr, dtype=float).flatten()
    if a.size % 2 != 0:
        raise ValueError(f"Expected even number of weights, got {a.size}")

    cur_n_bfs = a.size // 2
    w2 = a.reshape(2, cur_n_bfs)

    if cur_n_bfs == n_bfs:
        return w2

    src_x = np.linspace(0.0, 1.0, cur_n_bfs)
    dst_x = np.linspace(0.0, 1.0, n_bfs)
    w_resized = np.empty((2, n_bfs), dtype=float)
    for d in range(2):
        w_resized[d] = np.interp(dst_x, src_x, w2[d])
    return w_resized

def write_weights_csv(path, w2):
    """Exact copy of original write_weights_csv."""
    row = w2.reshape(-1)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerow(list(row))

def read_weights_csv(path, n_bfs):
    """Exact copy of original read_weights_csv with warning and trimming logic."""
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", txt)
    vals = [float(x) for x in nums]

    need = 2 * n_bfs
    if len(vals) < need:
        raise ValueError(f"{path} has only {len(vals)} numbers, need {need} (2*N_BFS).")

    if len(vals) % 2 != 0:
        print(f"Warning: {path} has odd length ({len(vals)}). Dropping last value.")
        vals = vals[:-1]

    if len(vals) > need:
        print(f"Warning: {path} has {len(vals)} values. Trimming to the first {need}.")
        vals = vals[:need]

    return row_to_2x50(vals, n_bfs)

def read_move_csv(path):
    """Exact copy of original read_move_csv with fallback logic."""
    try:
        data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        if data.dtype.names and {"x", "y"}.issubset(data.dtype.names):
            xy = np.column_stack([data["x"], data["y"]]).astype(float)
            if xy.ndim == 2 and xy.shape[1] >= 2:
                return xy
    except Exception:
        pass
    xy = np.loadtxt(path, delimiter=",", dtype=float)
    if xy.ndim != 2 or xy.shape[1] < 2:
        raise ValueError(f"{path} must have at least two columns (x,y)")
    return xy[:, :2].astype(float)

def save_trajectory_data(iter_idx, task_trajectory, csv_path):
    """Exact copy of original save_trajectory_data."""
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "step", "x", "y", "timestamp"])
        timestamp = time.strftime("%Y-%m-%d %H-%M-%S")
        for step_idx, target in enumerate(task_trajectory):
            x, y = target[0], target[1]
            w.writerow([iter_idx, step_idx, float(x), float(y), timestamp])

def save_ik_error(iter_idx, step_idx, target_3d, error_val, csv_path):
    """Exact copy of original save_ik_error."""
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            w.writerow(["iter", "step", "x", "y", "z", "error_m", "timestamp"])
        x, y, z = float(target_3d[0]), float(target_3d[1]), float(target_3d[2])
        w.writerow([int(iter_idx), int(step_idx), x, y, z, float(error_val), time.strftime("%Y-%m-%d %H-%M-%S")])

def parse_ollama_weights(out_text, n_bfs):
    """Exact copy of original parse_ollama_weights with code-fence cleaning."""
    text = out_text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```[^\n]*\n|\n```$", "", text, flags=re.MULTILINE).strip()
    
    text = text.split("<weights>")[1].split("</weights>")[0].strip() if "<weights>" in text and "</weights>" in text else text
    text = text.split("**Line 1:**")[1].strip() if "**Line 1:**" in text else text
    text = text.split("**Weights (20 floats):** ")[1].strip() if "**Weights (20 floats):**" in text else text
    
    try:
        obj = json.loads(text)
        cand = obj.get("weights", None)
        if isinstance(cand, list):
            return row_to_2x50(cand, n_bfs)
    except Exception:
        pass
    nums = re.findall(r"[-+]?\d*\.?\d+", text)
    if len(nums) >= 2 * n_bfs:
        return row_to_2x50([float(x) for x in nums[:2 * n_bfs]], n_bfs)
    raise ValueError("Could not parse weights from LLM output")

def save_dialog(dialog_dir, iter_idx, summary_prompt, summary, prompt, response):
    """Exact copy of original save_dialog."""
    pid = f"iter_{iter_idx:03d}"
    with open(os.path.join(dialog_dir, pid + "summary_prompt.txt"), "w", encoding="utf-8") as f:
        f.write(summary_prompt)
    with open(os.path.join(dialog_dir, pid + "_summary.txt"), "w", encoding="utf-8") as f:
        f.write(summary)
    with open(os.path.join(dialog_dir, pid + "_prompt.txt"), "w", encoding="utf-8") as f:
        f.write(prompt)
    with open(os.path.join(dialog_dir, pid + "_response.txt"), "w", encoding="utf-8") as f:
        f.write(response)

def append_weight_history(csv_path, iter_idx, tag, w2, n_bfs):
    """Exact copy of original append_weight_history."""
    flat = list(map(float, w2.reshape(-1)))
    file_exists = os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if not file_exists:
            header = ["iter", "timestamp", "tag"] + [f"w{i}" for i in range(2 * n_bfs)]
            w.writerow(header)
        w.writerow([iter_idx, time.strftime("%Y-%m-%d %H-%M-%S"), tag] + flat)
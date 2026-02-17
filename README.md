
---

#  UR5 Robot Cleaning Optimization with LLMs

This repository contains a framework for optimizing robot trajectories using **Large Language Models (LLMs)**. By combining **Dynamic Movement Primitives (DMPs)** with the reasoning capabilities of models like Gemini and Ollama, the system learns to clean a workspace in a **MuJoCo** simulation through iterative feedback.

##  Requirements

To run this project, ensure your environment meets the following specifications:

### **Software & Libraries**

* **Python 3.10+**
* **MuJoCo Physics Engine:** Used for the 3D robot simulation and ball-collision physics.
* **Core Dependencies:** `numpy`, `pandas`, `pyyaml`, `jinja2`, and `scipy`.
* **AI/LLM Tools:** `ollama` (for local models) and `google-genai` (for Gemini API).
* **GUI Tools:** `tkinter` (required for drawing paths or real-time mouse control).

### **API Keys & Environment**

* **Gemini API:** If using Google models, you must have a `keys.env` file in the root directory containing your `GOOGLE_API_KEY`.
* **Ollama:** If using local optimization, ensure the Ollama service is running and the model specified in `config.yaml` is downloaded.

---

##  How to Run the Code

There are three primary ways to interact with the system depending on your goal:

### **1. Automated LLM Optimization (`main_llm.py`)**

This is the standard research loop where the LLM acts as the primary optimizer.

* **Purpose:** The robot performs a task, the system automatically counts the remaining balls, and the LLM receives a "Report Card" to improve the next attempt.
* **Command:**
```bash
python main_llm.py --config config.yaml --keys keys.env

```



### **2. "On-Site" Human-in-the-Loop (`main_llm_onsite.py`)**

Designed for specific thesis work where human evaluation is required.

* **Purpose:** The system pauses after each trajectory. Instead of an automated ball count, it prompts **you** to enter a reward score (0-100) based on how well you think the robot performed.
* **Command:**
```bash
python main_llm_onsite.py

```



### **3. Manual Controller & Menu (`main.py`)**

A utility mode for manual testing and setup.

* **Purpose:** Access a menu to draw trajectories with your mouse, test predefined shapes (like the Infinity sign), or reset the robot home.
* **Command:**
```bash
python main.py

```

##  Simulation Display: GUI vs. Headless Mode

The system is configured to handle both **GUI (visual)** and **Headless (background)** simulations. This is useful for running large batches of iterations on a server or saving GPU resources.

### **1. Running Headless (No Window)**

If you want to run the optimization as fast as possible without a 3D window appearing:

* **Automatic Detection:** The `ViewerAdapter` will automatically switch to **HEADLESS mode (EGL)** if it detects no monitor or if the display initialization fails.
* **Manual Force:** You can force headless mode by uncommenting these lines in your main entry scripts (e.g., `main_llm.py`):
```python
import os
os.environ["MUJOCO_GL"] = "egl"
os.environ["PYOPENGL_PLATFORM"] = "egl"

```

### **2. Running with GUI (Visual Window)**

To watch the robot perform the cleaning task in real-time:

* **Requirements:** Ensure you are running on a machine with a monitor and that `has_display` is set to `True` in `env/adapter.py`.
* **Viewer Options:** The system will first try to use the official `mujoco.viewer` (DeepMind) and will fall back to `mujoco-python-viewer` (Community) if needed.

---


---

##  File Structure & Logic

| Folder / File | Description |
| --- | --- |
| **`agent/`** | Contains the LLM interface, DMP logic (Discrete & Rhythmic), and prompt templates. |
| **`config/`** | Stores `.yaml` files that control robot speed, table boundaries, and AI settings. |
| **`env/`** | The "Physics Hub" containing the MuJoCo viewer adapter and Inverse Kinematics (IK) solvers. |
| **`runner/`** | The execution scripts that manage the loops for both automated and onsite runs. |
| **`utils/`** | Helper scripts for generating shapes (circles, squares) and obstacle avoidance math. |

---

##  Logging & Data Analysis

Every experiment creates a unique, numbered folder within `/logs/`.

* **`weights_history.csv`**: Logs every set of weights proposed by the AI vs. those actually executed.
* **`ee_trajectory.csv`**: The high-resolution X, Y, Z path of the robot's "hand" (end-effector).
* **`llm_iteration_log.csv`**: The primary "Scoreboard" showing balls cleaned or human rewards per iteration.
* **`ik_errors.csv`**: Records any mathematical failures where the robot couldn't physically reach a target point.
* **`llm_dialog/`**: A "Transcript" folder containing every prompt sent to the AI and its raw response.

---

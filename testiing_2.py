# dmp_mujoco_system.py
# Dynamic Movement Primitives System for UR5e Robot in MuJoCo
# Modes: Discrete (XY waypoints), Rhythmic (draw XY), Real-time Mouse (XY)
# Z is held constant at CONSTANT_Z for all modes.
#%%
import math
import time
import numpy as np
import mujoco
import tkinter as tk
from tkinter import N, messagebox, simpledialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import queue
from utils.draw_shapes import circle_trajectory, elipsoid_trajectory, square_trajectory, infinity_trajectory
import os
os.environ["MUJOCO_GL"] = "egl"
# from waypoint_generator import generate_initial_pattern
# Movement Primitives Installation:
# pip install movement_primitives

try:
    # from movement_primitives.dmp import DMP, CartesianDMP
    # from movement_primitives.dmp_fast import RythmicDMP
    # from movement_primitives.dmp_fast import DiscreteDMP
    from pydmps.dmp_discrete import DMPs_discrete
    from pydmps.dmp_rhythmic import DMPs_rhythmic

    MOVEMENT_PRIMITIVES_AVAILABLE = True
except ImportError:
    print("movement_primitives not found. Using custom implementation.")
    MOVEMENT_PRIMITIVES_AVAILABLE = False

# ---------------------------------------------------------
# Configuration Constant
# ---------------------------------------------------------
XML_PATH = "ballmove.xml"
SITE_NAME = "ee_site"
UR5E_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

# 🎯 MOP CONFIGURATION
MOP_Z_HEIGHT = 0.51  # Constant Z coordinate for mop
# HOME_JOINT_POSITIONS = np.array([
#     -2.89,  # shoulder_pan
#     -1.07,  # shoulder_lift
#     0.377,  # elbow
#     -0.314,  # wrist_1
#     -0.0628,  # wrist_2c
#     -0.503  # wrist_3
# ])
HOME_JOINT_POSITIONS = np.array([0.16986918, -np.pi/1.5, -1.1,  -np.pi/9,  np.pi/2,  1.64722557])
# HOME_JOINT_POSITIONS = np.array([0.16986918, -1.99060853, -1.0536693,  -0.38275826,  1.59757099,  1.64722557])
N_BFs = 10  # Number of basis functions for DMPs

# Enhanced IK Parameters
INIT_LAMBDA = 0.15
TOL = 1e-3
PRINT_EVERY = 60
ANIMATION_DURATION = 4.0
ANIMATION_FPS = 75


# ---------------------------------------------------------
# Viewer Adapter
# ---------------------------------------------------------
class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo DMP Controller"):
        self.model = model
        self.data = data
        self.backend = None
        self.viewer = None
        self._dm_context_mgr = None

        # Try DeepMind's built-in viewer (MuJoCo >= 3.1)
        try:
            import mujoco.viewer as mview
            self.backend = "dm"
            self.viewer = mview.launch_passive(model, data)
            self._dm_context_mgr = self.viewer
            print("[Viewer] Using mujoco.viewer (DeepMind).")
            return
        except Exception:
            pass

        # Try community viewer
        try:
            import mujoco_viewer
            self.backend = "community"
            self.viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
            print("[Viewer] Using mujoco-python-viewer.")
            return
        except Exception:
            pass

        print("[Viewer] No viewer available. Running headless.")
        self.backend = "none"

    def is_running(self):
        if self.backend == "dm":
            return self.viewer.is_running()
        elif self.backend == "community":
            return not self.viewer.closed
        else:
            return False

    def draw(self):
        if self.backend == "dm":
            self.viewer.sync()
        elif self.backend == "community":
            self.viewer.render()
        else:
            pass

    def close(self):
        if self.backend == "dm":
            try:
                self._dm_context_mgr.close()
            except Exception:
                pass
        elif self.backend == "community":
            try:
                self.viewer.close()
            except Exception:
                pass


# ---------------------------------------------------------
# Enhanced Robot Control Functions (from test.py)
# ---------------------------------------------------------
def _clamp_limits(model, qpos, joint_names):
    """Clamp joint positions to their limits"""
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            qpos[qadr] = np.clip(qpos[qadr], lo, hi)


def set_joint_positions(model, data, joint_names, positions):
    """Set joint positions from numpy array"""
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = positions[i]


def get_joint_positions(model, data, joint_names):
    """Get current joint positions as numpy array"""
    positions = np.zeros(len(joint_names))
    mujoco.mj_forward(model, data)
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        positions[i] = data.qpos[qadr]
    return positions


def _interpolate_path(p0, p1, max_step=0.03):
    """Linear path from p0 to p1 with small steps for smooth movement"""
    # print(f"Interpolating path... from p0: {p0} to p1: {p1}")
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    dist = np.linalg.norm(p1 - p0)
    # print(f"Interpolating path: {p0} -> {p1}, distance {dist:.3f} m")
    if dist <= max_step:
        return [p1]
    if math.isnan(dist) or math.isinf(dist):
        integer_value = int(0.0)
    n = int(np.ceil(dist / max_step))
    alphas = np.linspace(0.0, 1.0, n + 1)[1:]  # skip p0
    return [p0 * (1 - a) + p1 * a for a in alphas]


def enhanced_ik_solver(model, data, site_id, goal_pos_3d, joint_names,
                       step_clip=0.2, max_wp_step=0.03, max_iters_per_wp=300,
                       lam_init=0.1, lam_inc=2.0, lam_dec=0.85,
                       tol=1e-3, print_every=60):
    """
    Enhanced 3D IK solver based on the working implementation from test.py
    """
    # Compute DOF columns for the joints
    dof_cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid == -1:
            raise RuntimeError(f"Joint '{jn}' not found.")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Joint '{jn}' must be a hinge.")
        dof_cols.append(model.jnt_dofadr[jid])
    dof_cols = np.asarray(dof_cols, int)

    def clamp_limits():
        _clamp_limits(model, data.qpos, joint_names)
        mujoco.mj_forward(model, data)

    start = np.array(data.site_xpos[site_id])
    waypoints = _interpolate_path(start, goal_pos_3d, max_step=max_wp_step)

    for wpi, wp in enumerate(waypoints, 1):
        lam = lam_init
        mujoco.mj_forward(model, data)
        prev_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))
        stalled = 0

        for it in range(1, max_iters_per_wp + 1):
            mujoco.mj_forward(model, data)

            # World-frame site Jacobian
            Jp = np.zeros((3, model.nv))
            Jr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Jp, Jr, site_id)
            J = Jp[:, dof_cols]  # (3,6)

            e = wp - np.array(data.site_xpos[site_id])  # (3,)

            # Levenberg-Marquardt step: dq = (J^T*J + λI)^(-1) * J^T * e
            A = J.T @ J + lam * np.eye(J.shape[1])
            b = J.T @ e

            try:
                dq = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(A) @ b

            # Trust-region: clip step size
            nq = np.linalg.norm(dq)
            if nq > step_clip:
                dq *= (step_clip / (nq + 1e-12))

            # Tentative update with rollback capability
            qpos_before = data.qpos.copy()
            for k, jn in enumerate(joint_names):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] += dq[k]

            clamp_limits()
            mujoco.mj_forward(model, data)
            new_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))

            if new_err < prev_err - 1e-6:
                # Accept the step
                prev_err = new_err
                lam = max(1e-6, lam * lam_dec)
                stalled = 0
            else:
                # Rollback and increase damping
                data.qpos[:] = qpos_before
                mujoco.mj_forward(model, data)
                lam *= lam_inc
                stalled += 1

            if it % print_every == 0:
                print(f" [wp {wpi:02d}/{len(waypoints)} | it {it:03d}] "
                      f"|e|={prev_err:.6f} m, λ={lam:.3g}")

            if prev_err < tol:
                break

            if stalled >= 30:
                break

        # If this waypoint failed, bail out
        if prev_err >= tol:
            print(f"⚠️ Waypoint {wpi} failed with error {prev_err:.6f} m")
            return False, prev_err

    # Check final goal accuracy
    final_err = np.linalg.norm(goal_pos_3d - np.array(data.site_xpos[site_id]))
    return True, final_err


def enhanced_interpolate(start_pos, end_pos, t):
    """Enhanced smooth interpolation using smootherstep function"""
    smooth_t = 6 * t ** 5 - 15 * t ** 4 + 10 * t ** 3
    return start_pos + smooth_t * (end_pos - start_pos)


def animate_robot_movement(model, data, viewer, joint_names, start_joints, target_joints,
                           duration=2.0, fps=60):
    """Smooth animation between joint configurations"""
    print(f"🎬 Animating robot movement over {duration:.1f} seconds...")

    total_frames = int(duration * fps)
    dt = 1.0 / fps

    for frame in range(total_frames + 1):
        t = frame / total_frames

        # Enhanced smooth interpolation
        current_joints = enhanced_interpolate(start_joints, target_joints, t)

        # Update robot position
        set_joint_positions(model, data, joint_names, current_joints)
        _clamp_limits(model, data.qpos, joint_names)

        # Update physics and render
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        viewer.draw()

        # Control frame rate
        time.sleep(dt)

        # Check if viewer is still open
        if not viewer.is_running():
            break

    print("✅ Animation complete!")


# ---------------------------------------------------------
# Simple DMP Implementation (fallback)
# ---------------------------------------------------------
class SimpleDMP:
    def __init__(self, n_dmps, n_bfs=50, dt=0.01, y0=None, goal=None):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt

        # DMP parameters
        self.alpha_y = 25.0
        self.beta_y = self.alpha_y / 4.0
        self.alpha_x = 1.0

        # Gaussian basis functions
        self.centers = np.exp(-self.alpha_x * np.linspace(0, 1, n_bfs))
        self.widths = np.ones(n_bfs) * n_bfs / (self.centers[1:] - self.centers[:-1]).mean()

        # Weights for each DMP and basis function
        self.weights = np.zeros((n_dmps, n_bfs))

        # State variables
        self.reset_state()

        if y0 is not None:
            self.y0 = np.array(y0)
        if goal is not None:
            self.goal = np.array(goal)

    def reset_state(self):
        self.x = 1.0
        self.y = np.zeros(self.n_dmps)
        self.dy = np.zeros(self.n_dmps)

        if hasattr(self, 'y0'):
            self.y = self.y0.copy()
        if hasattr(self, 'goal'):
            self.goal_current = self.goal.copy()

    def step(self, tau=1.0, external_force=None):
        # Canonical system
        dx = -self.alpha_x * self.x * tau

        # Forcing function
        psi = np.exp(-self.widths * (self.x - self.centers) ** 2)
        psi_norm = psi / (psi.sum() + 1e-10)

        f = np.dot(self.weights, psi_norm) * self.x * (self.goal_current - self.y0)

        # Transformation system
        ddy = self.alpha_y * (self.beta_y * (self.goal_current - self.y) - self.dy / tau) + f
        if external_force is not None:
            ddy += external_force

        # Integration
        self.x += dx * self.dt
        self.dy += ddy * tau * self.dt
        self.y += self.dy * self.dt

        return self.y.copy()

    def imitate_path(self, path):
        # Learn from demonstration
        path = np.array(path)
        if len(path.shape) == 1:
            path = path.reshape(-1, 1)

        n_points, n_dmps = path.shape
        self.n_dmps = n_dmps
        self.weights = np.zeros((n_dmps, self.n_bfs))

        self.y0 = path[0]
        self.goal = path[-1]

        # Generate target forcing function
        dt = 1.0 / n_points
        x_track = np.exp(-self.alpha_x * np.linspace(0, 1, n_points))

        # Calculate target accelerations
        velocity = np.gradient(path, axis=0) / dt
        acceleration = np.gradient(velocity, axis=0) / dt

        for d in range(n_dmps):
            f_target = acceleration[:, d] - self.alpha_y * (
                    self.beta_y * (self.goal[d] - path[:, d]) - velocity[:, d]
            )

            # Regression to find weights
            X = np.zeros((n_points, self.n_bfs))
            for i, x in enumerate(x_track):
                psi = np.exp(-self.widths * (x - self.centers) ** 2)
                X[i] = psi * x * (self.goal[d] - self.y0[d])

            self.weights[d] = np.linalg.pinv(X) @ f_target


class SimpleRythmicDMP:
    def __init__(self, n_dmps, n_bfs=50, dt=0.01):
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        self.dt = dt

        # Rhythmic DMP parameters
        self.alpha_y = 25.0
        self.beta_y = self.alpha_y / 4.0

        # Phase and frequency
        self.phi = 0.0
        self.freq = 1.0  # Hz

        # Basis functions for rhythmic patterns
        self.centers = np.linspace(0, 2 * np.pi, n_bfs)
        self.widths = np.ones(n_bfs) * n_bfs / (2 * np.pi)

        # Weights and amplitudes
        self.weights = np.zeros((n_dmps, n_bfs))
        self.r = np.ones(n_dmps)  # amplitude

        # State variables
        self.reset_state()

    def reset_state(self):
        self.phi = 0.0
        self.y = np.zeros(self.n_dmps)
        self.dy = np.zeros(self.n_dmps)
        self.r = np.ones(self.n_dmps)

    def step(self, tau=1.0):
        # Phase system
        dphi = 2 * np.pi * self.freq * tau

        # Forcing function
        psi = np.exp(-self.widths * np.cos(self.phi - self.centers))
        psi_norm = psi / (psi.sum() + 1e-10)

        f = np.dot(self.weights, psi_norm) * self.r

        # Rhythmic transformation system
        ddy = self.alpha_y * (self.beta_y * (-self.y) - self.dy / tau) + f

        # Integration
        self.phi += dphi * self.dt
        if self.phi > 2 * np.pi:
            self.phi -= 2 * np.pi

        self.dy += ddy * tau * self.dt
        self.y += self.dy * self.dt

        return self.y.copy()

    def imitate_path(self, path):
        # Learn rhythmic pattern from demonstration
        path = np.array(path)
        if len(path.shape) == 1:
            path = path.reshape(-1, 1)

        n_points, n_dmps = path.shape
        self.n_dmps = n_dmps
        self.weights = np.zeros((n_dmps, self.n_bfs))

        # Calculate amplitude
        self.r = np.std(path, axis=0)

        # Generate phase trajectory
        phi_track = np.linspace(0, 2 * np.pi, n_points)

        # Calculate target accelerations
        dt = 1.0 / n_points
        velocity = np.gradient(path, axis=0) / dt
        acceleration = np.gradient(velocity, axis=0) / dt

        for d in range(n_dmps):
            f_target = acceleration[:, d] - self.alpha_y * (
                    self.beta_y * (-path[:, d]) - velocity[:, d]
            )

            # Regression to find weights
            X = np.zeros((n_points, self.n_bfs))
            for i, phi in enumerate(phi_track):
                psi = np.exp(-self.widths * np.cos(phi - self.centers))
                X[i] = psi * self.r[d]

            self.weights[d] = np.linalg.pinv(X) @ f_target


# ---------------------------------------------------------
# Drawing Interface for Mouse Control
# ---------------------------------------------------------
class DrawingInterface:
    def __init__(self, width=400, height=300, title="Draw Trajectory"):
        self.width = width
        self.height = height
        self.title = title
        self.trajectory = []
        self.drawing = False
        self.completed = False

        # Create window
        self.root = tk.Toplevel()
        self.root.title(title)
        self.root.geometry(f"{width}x{height + 100}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Canvas for drawing
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg='white')
        self.canvas.pack(pady=10)

        # Bind mouse events
        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.stop_draw)

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        tk.Button(button_frame, text="Clear", command=self.clear).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Done", command=self.done).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Cancel", command=self.cancel).pack(side=tk.LEFT, padx=5)

        # Instructions
        instructions = tk.Label(self.root, text="Draw with mouse. Click 'Done' when finished.")
        instructions.pack()

        # Coordinate transformation parameters (robot workspace)
        self.x_min, self.x_max = -1.0, 1.0  # Robot workspace in meters
        self.y_min, self.y_max = -0.6, 0.6

    def canvas_to_robot_coords(self, canvas_x, canvas_y):
        # Convert canvas coordinates to robot workspace coordinates
        x = self.x_min + (canvas_x / self.width) * (self.x_max - self.x_min)
        y = self.y_max - (canvas_y / self.height) * (self.y_max - self.y_min)  # Flip Y
        return x, y

    def start_draw(self, event):
        self.drawing = True
        self.trajectory = []
        x, y = self.canvas_to_robot_coords(event.x, event.y)
        self.trajectory.append([x, y])

    def draw(self, event):
        if self.drawing:
            # Draw on canvas
            if len(self.trajectory) > 0:
                last_canvas_x = int((self.trajectory[-1][0] - self.x_min) / (self.x_max - self.x_min) * self.width)
                last_canvas_y = int((self.y_max - self.trajectory[-1][1]) / (self.y_max - self.y_min) * self.height)

                self.canvas.create_line(last_canvas_x, last_canvas_y, event.x, event.y,
                                        width=2, fill='blue', capstyle=tk.ROUND)

            # Add to trajectory
            x, y = self.canvas_to_robot_coords(event.x, event.y)
            self.trajectory.append([x, y])

    def stop_draw(self, event):
        self.drawing = False

    def clear(self):
        self.canvas.delete("all")
        self.trajectory = []

    def done(self):
        if len(self.trajectory) < 2:
            messagebox.showwarning("Warning", "Please draw a trajectory first!")
            return
        self.completed = True
        self.root.quit()

    def cancel(self):
        self.trajectory = []
        self.completed = False
        self.root.quit()

    def on_closing(self):
        self.completed = False
        self.root.quit()

    def get_trajectory(self):
        """Run the drawing interface and return the trajectory"""
        self.root.mainloop()
        if self.completed and len(self.trajectory) > 1:
            return np.array(self.trajectory)
        return None


# ---------------------------------------------------------
# Real-time Mouse Control Interface
# ---------------------------------------------------------
class RealTimeMouseControl:
    def __init__(self, width=600, height=600):
        self.width = width
        self.height = height
        self.active = False
        self.current_pos = np.array([-0.2, 0.0])  # Default position
        self.trajectory_log = []

        # Create window
        self.root = tk.Toplevel()
        self.root.title("Real-time Mouse Control")
        self.root.geometry(f"{width}x{height + 140}")  # +40 to fit sensitivity UI
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Canvas for mouse control
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg='lightgray')
        self.canvas.pack(pady=10)

        # Bind mouse events
        self.canvas.bind("<Motion>", self.mouse_move)
        self.canvas.bind("<Button-1>", self.toggle_control)

        # Control state
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(pady=10)

        self.status_label = tk.Label(self.control_frame, text="Click to start control")
        self.status_label.pack()

        tk.Button(self.control_frame, text="Stop Control", command=self.stop_control).pack(side=tk.LEFT, padx=5)
        tk.Button(self.control_frame, text="Save Log", command=self.save_trajectory).pack(side=tk.LEFT, padx=5)

        # Position display
        self.pos_label = tk.Label(self.root, text="Position: (0.30, 0.00)")
        self.pos_label.pack()

        # Coordinate transformation parameters
        self.x_min, self.x_max = -1.05, 1.05
        self.y_min, self.y_max = -0.65, 0.65

        # ====== SENSITIVITY (ADDED) ======
        self.sensitivity = 2.0  # 1.0 = original mapping
        sens_frame = tk.Frame(self.root)
        sens_frame.pack(pady=4)
        tk.Label(sens_frame, text="Sensitivity").pack(side=tk.LEFT, padx=6)
        self.sens_slider = tk.Scale(
            sens_frame, from_=0.5, to=5.0, resolution=0.1,
            orient=tk.HORIZONTAL, length=220, command=self._on_sens_change
        )
        self.sens_slider.set(self.sensitivity)
        self.sens_slider.pack(side=tk.LEFT)
        # =================================

        self.running = True

    # ====== SENSITIVITY CALLBACK (ADDED) ======
    def _on_sens_change(self, val):
        try:
            self.sensitivity = float(val)
        except Exception:
            pass
    # ==========================================

    def canvas_to_robot_coords(self, canvas_x, canvas_y):
        # --- Original mapping extended with sensitivity and clamped to bounds ---
        nx = np.clip(canvas_x / self.width, 0.0, 1.0)
        ny = np.clip(canvas_y / self.height, 0.0, 1.0)

        xrange = (self.x_max - self.x_min) * self.sensitivity
        yrange = (self.y_max - self.y_min) * self.sensitivity

        x = self.x_min + nx * xrange
        y = self.y_max - ny * yrange

        # Clamp to original workspace limits
        x = float(np.clip(x, min(self.x_min, self.x_max), max(self.x_min, self.x_max)))
        y = float(np.clip(y, min(self.y_min, self.y_max), max(self.y_min, self.y_max)))
        return np.array([y,x])

    def mouse_move(self, event):
        if self.active:
            self.current_pos = self.canvas_to_robot_coords(event.x, event.y)
            self.trajectory_log.append(self.current_pos.copy())
            self.pos_label.config(text=f"Position: ({self.current_pos[0]:.3f}, {self.current_pos[1]:.3f})")

    def toggle_control(self, event):
        self.active = not self.active
        if self.active:
            self.status_label.config(text="Control ACTIVE - Move mouse to control robot")
            self.canvas.config(bg='lightgreen')
            self.trajectory_log = []
        else:
            self.status_label.config(text="Control STOPPED - Click to resume")
            self.canvas.config(bg='lightgray')

    def stop_control(self):
        self.active = False
        self.running = False
        self.status_label.config(text="Control stopped")
        self.canvas.config(bg='lightgray')

    def save_trajectory(self):
        if len(self.trajectory_log) > 0:
            np.savetxt("mouse_trajectory.csv", self.trajectory_log, delimiter=",",
                       header="x,y", comments="")
            messagebox.showinfo("Saved", f"Trajectory saved with {len(self.trajectory_log)} points")

    def on_closing(self):
        self.running = False
        self.active = False
        self.root.quit()

    def get_current_position(self):
        return self.current_pos.copy()

    def is_active(self):
        return self.active and self.running


# ---------------------------------------------------------
# Main Enhanced DMP Controller Class
# ---------------------------------------------------------
class EnhancedDMPController:
    def __init__(self, xml_path="ballmove.xml", n_bfs=10):
        # Load model
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # number of basis functions per DMP
        self.n_bfs = n_bfs
        print(self.n_bfs)

        # Robot configuration
        self.joint_names = UR5E_JOINTS
        self.site_name = SITE_NAME
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        self.dt = 0.002  # Simulation timestep
        if self.site_id == -1:
            raise RuntimeError(f"Site '{self.site_name}' not found in model")

        # Initialize viewer
        self.viewer = ViewerAdapter(self.model, self.data)
        self._qpos0 = self.data.qpos.copy()
        self._qvel0 = self.data.qvel.copy()
        self._act0  = self.data.act.copy() if hasattr(self.data, "act") else None

        # DMP instances
        self.dmp = None
        self.dmp = None

        # Control variables
        self.mode = "menu"
        self.running = True

        # Initialize robot position
        self.reset_robot_to_home()
        self.x_min, self.x_max = -1.0, 1.0
        self.y_min, self.y_max = -0.6, 0.6
        self.num_balls = 150 # Number of balls in the environment
        self.num_x_segments = 3 
        self.num_y_segments = 2
        self.grid_count_log = []
        self.grid_count = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)

    def reset_robot_to_home(self):

        # Set home joint positions
        set_joint_positions(self.model, self.data, self.joint_names, HOME_JOINT_POSITIONS)
        _clamp_limits(self.model, self.data.qpos, self.joint_names)
        mujoco.mj_forward(self.model, self.data)

        # Get current end-effector position
        current_pos = self.data.site_xpos[self.site_id]
        print(f" Robot reset to home position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")

        # Update viewer
        self.viewer.draw()

        return True
    # ------------------------
    # World/Viewer Reset Utils
    # ------------------------
    def reset_grid_counters(self):
        """Clear per-iteration counters and logs."""
        try:
            self.grid_count[:] = 0
        except Exception:
            self.grid_count = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)
        self.grid_count_log.clear()

    def hard_reset_from_home(self, redraw: bool = True):
        """
        Reset world/balls to initial snapshot but place robot at HOME_JOINT_POSITIONS.
        - Restores qpos/qvel for everything from snapshot
        - Then overwrites robot joint qpos to HOME and zeros their qvel entries
        - Resets actuators and counters
        - Runs mj_forward and optionally redraws
        """
        # 1) Restore full snapshot (balls, free bodies, etc.)
        np.copyto(self.data.qpos, self._qpos0)
        np.copyto(self.data.qvel, self._qvel0)
        if self._act0 is not None and hasattr(self.data, "act"):
            np.copyto(self.data.act, self._act0)

        # 2) Overwrite robot joints to HOME and zero their qvel
        for i, jn in enumerate(self.joint_names):
            jid  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            qadr = self.model.jnt_qposadr[jid]       # index into qpos
            dadr = self.model.jnt_dofadr[jid]        # index into qvel (1 dof per hinge)
            self.data.qpos[qadr] = HOME_JOINT_POSITIONS[i]
            self.data.qvel[dadr] = 0.0

        # 3) Rebuild physics + clear per-iter counters
        mujoco.mj_forward(self.model, self.data)
        self.reset_grid_counters()

        # 4) Optional redraw
        if redraw and hasattr(self, "viewer") and self.viewer is not None:
            try:
                self.viewer.draw()
            except Exception:
                pass


    def move_to_3d_position(self, target_xy, animate=True):
        """
        Move robot to 3D position with constant Z (mop height)
        """
        # Create 3D target position with constant Z
        target_3d = np.array([target_xy[0], target_xy[1], MOP_Z_HEIGHT])

        print(f"🎯 Moving to 3D target: [{target_3d[0]:.3f}, {target_3d[1]:.3f}, {target_3d[2]:.3f}]")

        if animate:
            # Get current joint positions for animation
            start_joints = get_joint_positions(self.model, self.data, self.joint_names)

        # Solve 3D IK
        success, error = enhanced_ik_solver(
            self.model, self.data, self.site_id, target_3d, self.joint_names,
            step_clip=0.2, max_wp_step=0.03, max_iters_per_wp=300,
            lam_init=INIT_LAMBDA, tol=TOL, print_every=PRINT_EVERY
        )

        if success:
            print(f"✅ IK Success! Position error: {error:.6f} m")

            if animate:
                # Get target joint configuration
                target_joints = get_joint_positions(self.model, self.data, self.joint_names)

                # Reset to start position for animation
                set_joint_positions(self.model, self.data, self.joint_names, start_joints)
                # mujoco.mj_forward(self.model, self.data)
                # mujoco.mj_step(self.model, self.data)
                # self.viewer.draw()

                # Animate smooth movement
                animate_robot_movement(self.model, self.data, self.viewer, self.joint_names,
                                       start_joints, target_joints, duration=2.0, fps=60)

            return True
        else:
            print(f"❌ IK Failed! Final error: {error:.6f} m")
            return False

    # def count_balls_in_grid(self):
    #     x_edges = np.linspace(self.x_min, self.x_max, self.num_x_segments + 1)
    #     y_edges = np.linspace(self.y_min, self.y_max, self.num_y_segments + 1)
    #     grid_counts = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)
    #
    #     ball_names = [f"ball_{i+1}" for i in range(self.num_balls)]
    #     ball_positions = []
    #     for name in ball_names:
    #         body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
    #         if body_id != -1:
    #             pos = self.data.xpos[body_id][:2]  # x, y position
    #             ball_positions.append(pos)
    #     ball_positions = np.array(ball_positions)
    #
    #     for pos in ball_positions:
    #         x, y = pos
    #         # Find which grid cell (i, j) the ball is in
    #         i = np.searchsorted(x_edges, x, side='right') - 1
    #         j = np.searchsorted(y_edges, y, side='right') - 1
    #         # Clamp indices to valid range
    #         i = min(max(i, 0), self.num_x_segments - 1)
    #         j = min(max(j, 0), self.num_y_segments - 1)
    #         grid_counts[i, j] += 1
    #
    #     grid_counts = grid_counts[:, ::-1] # reverse columns then transpose to match visual layout
    #
    #     # Print results
    #     for i in range(self.num_x_segments):
    #         for j in range(self.num_y_segments):
    #             print(f"Grid cell ({i+1},{j+1}) x:[{x_edges[i]:.2f},{x_edges[i+1]:.2f}] y:[{y_edges[j]:.2f},{y_edges[j+1]:.2f}]: {grid_counts[i, j]} balls")
    #     self.grid_count_log.append(grid_counts.copy())
    #     self.grid_count = grid_counts.copy()
    #     return grid_counts



    def count_balls_in_grid(self):
        x_edges = np.linspace(self.x_min, self.x_max, self.num_x_segments + 1)
        y_edges = np.linspace(self.y_min, self.y_max, self.num_y_segments + 1)
        grid_counts = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)

        ball_names = [f"ball_{i + 1}" for i in range(self.num_balls)]
        ball_positions = []
        for name in ball_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id != -1:
                pos = self.data.xpos[body_id][:2]  # x, y position
                # Only include balls inside the grid bounds
                if (self.x_min <= pos[0] <= self.x_max) and (self.y_min <= pos[1] <= self.y_max):
                    ball_positions.append(pos)
        ball_positions = np.array(ball_positions)

        for pos in ball_positions:
            x, y = pos
            # Find which grid cell (i, j) the ball is in
            i = np.searchsorted(x_edges, x, side='right') - 1
            j = np.searchsorted(y_edges, y, side='right') - 1
            # Clamp indices to valid range
            i = min(max(i, 0), self.num_x_segments - 1)
            j = min(max(j, 0), self.num_y_segments - 1)
            grid_counts[i, j] += 1

        grid_counts = grid_counts[:, ::-1]  # reverse columns then transpose to match visual layout

        # Print results
        for i in range(self.num_x_segments):
            for j in range(self.num_y_segments):
                print(
                    f"Grid cell ({i + 1},{j + 1}) x:[{x_edges[i]:.2f},{x_edges[i + 1]:.2f}] y:[{y_edges[j]:.2f},{y_edges[j + 1]:.2f}]: {grid_counts[i, j]} balls")
        print("total balls counted:", np.sum(grid_counts))
        self.grid_count_log.append(grid_counts.copy())
        self.grid_count = grid_counts.copy()
        return grid_counts



    def get_discrete_waypoints(self):
        """Get waypoints for discrete mode with improved UI"""
        root = tk.Tk()
        root.withdraw()  # Hide main window

        # Ask for number of points
        num_points = simpledialog.askinteger("Discrete DMP",
                                             "How many waypoints? (2-10)",
                                             minvalue=2, maxvalue=10)
        if num_points is None:
            return None

        points = []
        point_names = []

        # Generate point names
        if num_points == 2:
            point_names = ["Start", "End"]
        elif num_points == 3:
            point_names = ["Start", "Middle", "End"]
        else:
            point_names = ["Start"] + [f"Mid{i}" for i in range(1, num_points - 1)] + ["End"]

        # Get current position as default start
        current_pos = self.data.site_xpos[self.site_id]

        for i, name in enumerate(point_names):
            if i == 0:  # Start point
                use_current = messagebox.askyesno("Start Point",
                                                  f"Use current position as start point?\n"
                                                  f"Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f})\n"
                                                  f"Z will be fixed at: {MOP_Z_HEIGHT:.4f}")
                if use_current:
                    points.append([current_pos[0], current_pos[1]])
                    continue

            # Get point coordinates (X, Y only - Z is constant)
            coord_str = simpledialog.askstring("Waypoint",
                                               f"Enter {name} point coordinates (x, y):\n"
                                               f"Z will be automatically set to {MOP_Z_HEIGHT:.4f}\n"
                                               f"Example: 0.3, 0.1")
            if coord_str is None:
                return None

            try:
                x, y = map(float, coord_str.split(','))
                points.append([x, y])
            except ValueError:
                messagebox.showerror("Error", "Invalid coordinates. Please use format: x, y")
                return None

        root.destroy()
        return np.array(points)

    def apply_dmp(self, pattern="discrete", draw_waypoints=False, shape="infinity"):
        """Generate joint trajectory from discrete DMP waypoints (does not execute)"""
        if pattern == "discrete":
            print("\n🎯 === DISCRETE DMP MODE (3D) ===")
            self.dmp = DMPs_discrete(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)
        elif pattern == "rhythmic":
            print("\n🎯 === RHYTHMIC DMP MODE (3D) ===")
            print(self.n_bfs)
            self.dmp = DMPs_rhythmic(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)
        else:
            print("\n🎯 === UNKNOWN DMP MODE (3D) ===")

        if draw_waypoints:
            # Get trajectory from drawing interface
            drawing_interface = DrawingInterface(title="Draw DMP 2D Trajectory")
            trajectory = drawing_interface.get_trajectory()

            if trajectory is None:
                print("Cancelled by user")
                return
            # Add current ee_site position as the first entry in the trajectory
            current_pos = self.data.site_xpos[self.site_id]
            trajectory = np.vstack(([current_pos[:2]], trajectory))
        else:
            if shape == "infinity":
                x_traj, y_traj = infinity_trajectory(center=(0.0, 0.0), size=(2.0, 2.5), num_points=400, plot=False, color='orange', linestyle='-')
                trajectory = np.vstack((x_traj, y_traj)).T
                start_x = x_traj[0]
                start_y = y_traj[0]
                start_target_3d = np.array([start_x, start_y, MOP_Z_HEIGHT])
                enhanced_ik_solver(
                    self.model, self.data, self.site_id, start_target_3d, self.joint_names,
                    max_iters_per_wp=50, print_every=1000
                )
                # joints = get_joint_positions(self.model, self.data, self.joint_names)
                # print(f"joint positions for starting point {start_target_3d}: {joints}")
                # set_joint_positions(self.model, self.data, self.joint_names, joints)
                # self.viewer.draw()
            else:
                trajectory = self.get_discrete_waypoints()
        if trajectory is None:
            print("Cancelled by user")
            return None

        print(f"Waypoints defined: {len(trajectory)} points")
        for i, point in enumerate(trajectory):
            print(f"  Point {i + 1}: ({point[0]:.3f}, {point[1]:.3f}, {MOP_Z_HEIGHT:.4f})")

        self.dmp.imitate_path(trajectory.T)
        self.dmp.reset_state()
        print("✅ DMP trained on waypoints")

        dt = self.dt
        max_steps = int(5.0 / dt)
        task_traj = []

        for step in range(self.dmp.timesteps):
            if not self.viewer.is_running():
                break
            dmp_pos_2d, _, _ = self.dmp.step(tau=2.0)
            target_3d = np.array([dmp_pos_2d[0], dmp_pos_2d[1], MOP_Z_HEIGHT])
            task_traj.append(target_3d)
            if hasattr(self.dmp, 'x') and self.dmp.x < 0.01:
                break
            # print(f"Step {step}: DMP pos: {dmp_pos_2d}, 3D target: {target_3d}" )

        print(f"Generated {len(task_traj)} task-space waypoints from DMP.")


        joint_traj = []
        for idx, target_3d in enumerate(task_traj):
            success, _ = enhanced_ik_solver(
                self.model, self.data, self.site_id, target_3d, self.joint_names,
                max_iters_per_wp=50, print_every=1000
            )
            if success:
                joints = get_joint_positions(self.model, self.data, self.joint_names)
                joint_traj.append(joints.copy())
            else:
                print(f"❌ IK failed for waypoint {idx}, skipping.")

        print(f"Generated {len(joint_traj)} joint-space waypoints.")
        return joint_traj
    
    def set_joint_pid_gains(self, joint_names, kp_values, kd_values):
        """
        Set kp and kd for each joint actuator in MuJoCo.
        kp_values and kd_values should be lists/arrays of same length as joint_names.
        """
        for i, jn in enumerate(joint_names):
            # Get joint ID and DOF ID
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            dof_id = self.model.jnt_dofadr[joint_id]

            # Find actuator ID by checking which joint it controls
            actuator_id = -1
            for aid in range(self.model.nu):
                # trnid[aid, 0] stores the joint index for the actuator
                if self.model.actuator_trnid[aid, 0] == joint_id:
                    actuator_id = aid
                    break
            
            if actuator_id != -1:
                # Set kp (proportional gain)
                self.model.actuator_gainprm[actuator_id, 0] = kp_values[i]
                # Set bias for position servo (standard MuJoCo position actuator formula)
                self.model.actuator_biasprm[actuator_id, 1] = -kp_values[i] 
            else:
                print(f"Warning: No actuator found for joint '{jn}'")
            self.model.dof_damping[dof_id] = kd_values[i]
    
    # def apply_rhythmic_mode(self):
    #     """Apply rhythmic DMP mode with 3D movement"""


    #     # Get trajectory from drawing interface
    #     drawing_interface = DrawingInterface(title="Draw Rhythmic Pattern")
    #     trajectory = drawing_interface.get_trajectory()

    #     if trajectory is None:
    #         print("Cancelled by user")
    #         return
    #     # Add current ee_site position as the first entry in the trajectory
    #     current_pos = self.data.site_xpos[self.site_id]
    #     trajectory = np.vstack(([current_pos[:2]], trajectory))
    #     print(f"Trajectory captured: {trajectory.shape} points")
    #     print(f"X range: {trajectory[:, 0].min():.3f} to {trajectory[:, 0].max():.3f}")
    #     print(f"Y range: {trajectory[:, 1].min():.3f} to {trajectory[:, 1].max():.3f}")
    #     print(f"Z constant: {MOP_Z_HEIGHT:.4f}")

    #     # Create rhythmic DMP for 2D movement
    #     # if MOVEMENT_PRIMITIVES_AVAILABLE:
    #     #     try:
    #     #         from movement_primitives.dmp import RhythmicDMP
    #     #         self.rhythmic_dmp = DMPs_rhythmic(n_dmps=2, n_bfs=100, dt=self.dt, tau=0.5)
    #     #     except ImportError:
    #     #         self.rhythmic_dmp = DMPs_rhythmic(n_dmps=2, n_bfs=100, dt=self.dt, )
    #     # else:
    #     #     self.rhythmic_dmp = DMPs_rhythmic(n_dmps=2, n_bfs=100, dt=self.dt)
    #     self.dmp = DMPs_rhythmic(n_dmps=2, n_bfs=50, dt=self.dt)

    #     # Train DMP on drawn trajectory
    #     self.dmp.imitate_path(trajectory.T)
    #     self.dmp.reset_state()
    #     print("✅ Rhythmic DMP trained on drawn pattern")


    #     # Generate rhythmic DMP trajectory
    #     dt = self.dt
    #     max_steps = int(5.0 / dt)  # 5 seconds of movement
    #     task_traj = []

    #     for step in range(self.dmp.timesteps):
    #         dmp_pos_2d, _, _ = self.dmp.step(tau=10.0)  # higher tau for faster execution
    #         target_3d = np.array([dmp_pos_2d[0], dmp_pos_2d[1], MOP_Z_HEIGHT])
    #         task_traj.append(target_3d)

    #     print(f"Generated {len(task_traj)} task-space waypoints from rhythmic DMP.")

    #     # Generate joint trajectory via IK
    #     joint_traj = []
    #     for idx, target_3d in enumerate(task_traj):
    #         success, _ = enhanced_ik_solver(
    #             self.model, self.data, self.site_id, target_3d, self.joint_names,
    #             max_iters_per_wp=30, print_every=1000
    #         )
    #         if success:
    #             joints = get_joint_positions(self.model, self.data, self.joint_names)
    #             joint_traj.append(joints.copy())
    #         else:
    #             print(f"❌ IK failed for waypoint {idx}, skipping.")

    #     print(f"Generated {len(joint_traj)} joint-space waypoints.")
    #     return joint_traj

    def execute_realtime_mode(self):
        """Execute real-time mouse control mode with 3D movement"""
        print(f"Z-coordinate fixed at: {MOP_Z_HEIGHT:.4f} m")

        # Create mouse control interface
        mouse_control = RealTimeMouseControl()


        dt = 0.01
        control_thread = threading.Thread(target=self._realtime_control_loop,
                                          args=(mouse_control, dt))
        control_thread.daemon = True
        control_thread.start()

        # Keep interface running
        try:
            while mouse_control.running and self.viewer.is_running():
                mouse_control.root.update()
                time.sleep(0.01)
        except tk.TclError:
            pass

        print("🔚 Real-time control mode ended")

    def _realtime_control_loop(self, mouse_control, dt):
        """Control loop for real-time mouse control with 3D movement"""
        while mouse_control.running and self.viewer.is_running():
            if mouse_control.is_active():
                target_pos_2d = mouse_control.get_current_position()

                # Create 3D target with constant Z
                target_3d = np.array([target_pos_2d[0], target_pos_2d[1], MOP_Z_HEIGHT])

                # Solve IK and move robot
                success, _ = enhanced_ik_solver(
                    self.model, self.data, self.site_id, target_3d, self.joint_names,
                     step_clip= 0.1 , max_iters_per_wp=20, print_every=1000  # Very reduced for real-time
                )

                if success:
                    mujoco.mj_forward(self.model, self.data)
                    mujoco.mj_step(self.model, self.data)

                # Update visualization

                self.viewer.draw()

            time.sleep(dt)

    def run(self):
        """Main control loop"""
        print("\n🚀 Enhanced DMP Controller Started!")
        print("=" * 60)

        while self.running and self.viewer.is_running():
            print("\n📋 MAIN MENU:")
            print("1. Discrete DMP Mode (waypoint navigation with constant Z)")
            print("2. Discrete DMP Mode (Draw waypoints)")
            print("3. Rhythmic DMP Mode (mouse-drawn patterns with constant Z)")
            print("4. Rhythmic DMP Mode (Predefined Patterns)")
            print("5. Real-time Mouse Control (X-Y plane with constant Z)")
            print("6. Reset Robot to Home Position")
            print("7. Move to Custom Position (X, Y)")
            print("8. Quit")
            try:
                choice = input("\nSelect mode (1-8): ").strip()

                if choice == '1':
                    joint_traj = self.apply_dmp(pattern="discrete", draw_waypoints=False)
                    if joint_traj is not None and len(joint_traj) > 0:
                        self.execute_joint_trajectory(joint_traj)
                elif choice == '2':
                    joint_traj = self.apply_dmp(pattern="discrete", draw_waypoints=True)
                    if joint_traj is not None and len(joint_traj) > 0:
                        self.execute_joint_trajectory(joint_traj)
                elif choice == '3':
                    joint_traj = self.apply_dmp(pattern="rhythmic", draw_waypoints=True)
                    if joint_traj is not None and len(joint_traj) > 0:
                        self.execute_joint_trajectory(joint_traj)
                elif choice == '4':
                    joint_traj = self.apply_dmp(pattern="rhythmic", draw_waypoints=False)
                    if joint_traj is not None and len(joint_traj) > 0:
                        self.execute_joint_trajectory(joint_traj)
                elif choice == '5':
                    self.execute_realtime_mode()
                elif choice == '6':
                    self.reset_robot_to_home()
                elif choice == '7':
                    self.manual_move_prompt()
                elif choice == '8':
                    self.running = False
                else:
                    print("Invalid choice. Please select 1-8.")

                # Brief pause between operations
                time.sleep(0.5)

            except KeyboardInterrupt:
                print("\n\n⏹️ Interrupted by user")
                break
            except EOFError:
                break
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.count_balls_in_grid()
        self.viewer.close()
        print("👋 Enhanced DMP Controller shut down")

    def manual_move_prompt(self):
        """Prompt user for manual movement to a specific position"""
        try:
            coord_str = input(f"Enter target coordinates (x, y) [Z fixed at {MOP_Z_HEIGHT:.4f}]: ").strip()
            if not coord_str:
                return

            x, y = map(float, coord_str.replace(',', ' ').split())
            target_xy = np.array([x, y])

            print(f"Moving to: ({x:.3f}, {y:.3f}, {MOP_Z_HEIGHT:.4f})")
            success = self.move_to_3d_position(target_xy, animate=True)

            if success:
                print("✅ Movement completed successfully")
            else:
                print("❌ Movement failed - target may be out of reach")

        except ValueError:
            print("Invalid coordinates. Please use format: x, y")
        except Exception as e:
            print(f"Error during movement: {e}")
            
    def rotate_base(self):
        """Rotate the robot base by a specified angle"""
        try:
            angle_str = input("Enter rotation angle in degrees (positive for CCW, negative for CW): ").strip()
            if not angle_str:
                return

            angle_deg = float(angle_str)
            angle_rad = np.deg2rad(angle_deg)

            # Get current joint positions
            current_joints = get_joint_positions(self.model, self.data, self.joint_names)

            # Rotate base joint (first joint)
            current_joints[0] += angle_rad

            # Set new joint positions
            set_joint_positions(self.model, self.data, self.joint_names, current_joints)
            _clamp_limits(self.model, self.data.qpos, self.joint_names)
            mujoco.mj_forward(self.model, self.data)

            print(f"✅ Base rotated by {angle_deg:.1f} degrees")

            # Update viewer
            self.viewer.draw()

        except ValueError:
            print("Invalid angle. Please enter a numeric value.")
        except Exception as e:
            print(f"Error during rotation: {e}")

    def execute_joint_trajectory(self, joint_traj, dt=0.01):
        print(f"Executing joint trajectory with {len(joint_traj)} waypoints...")
        self.ee_trajectory = []
        # Reset robot to initial joint configuration
        if len(joint_traj) > 0:
            set_joint_positions(self.model, self.data, self.joint_names, joint_traj[0])
            mujoco.mj_forward(self.model, self.data)
            # self.viewer.draw()
            time.sleep(self.dt)

        for joints in joint_traj:
            self.data.ctrl[:] = joints
            mujoco.mj_step(self.model, self.data)
            # self.viewer.draw()
            time.sleep(self.dt)
            cl_pos = self.data.site_xpos[self.site_id].copy()
            self.ee_trajectory.append(cl_pos)
        print("Discrete trajectory execution complete.")


def main():
    """Main function"""
    xml_path = XML_PATH
    n_bfs = N_BFs

    try:
        controller = EnhancedDMPController(xml_path=xml_path)
        controller.run()
    except FileNotFoundError:
        print(f"❌ Error: XML file '{xml_path}' not found!")
        print("Please ensure the XML file is in the current directory.")
    except Exception as e:
        print(f"❌ Error initializing controller: {e}")
        import traceback
        traceback.print_exc()


xml_path = XML_PATH

if __name__ == "__main__":
    main()
    # try:
    #     controller = EnhancedDMPController(xml_path)
    #     controller.run()
    # except FileNotFoundError:
    #     print(f"❌ Error: XML file '{xml_path}' not found!")
    #     print("Please ensure the XML file is in the current directory.")
    # except Exception as e:
    #     print(f"❌ Error initializing controller: {e}")
    #     import traceback
    #     traceback.print_exc()

#%%

# dmp_mujoco_system.py
# Dynamic Movement Primitives System for UR5e Robot in MuJoCo
# Implements three modes: Discrete, Rhythmic, and Real-time Control

import time
import numpy as np
import mujoco
import tkinter as tk
from tkinter import messagebox, simpledialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import queue

# Movement Primitives Installation:
# pip install movement_primitives

try:
    from movement_primitives.dmp import DMP, CartesianDMP
    from movement_primitives.dmp_fast import RythmicDMP

    MOVEMENT_PRIMITIVES_AVAILABLE = True
except ImportError:
    print("movement_primitives not found. Using custom implementation.")
    MOVEMENT_PRIMITIVES_AVAILABLE = False


# ---------------------------------------------------------
# Viewer adapter: tries mujoco.viewer first, falls back to mujoco-python-viewer
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
# Simple DMP Implementation (fallback if movement_primitives not available)
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

        # Coordinate transformation parameters (will be set based on robot workspace)
        self.x_min, self.x_max = 0.1, 0.6  # Robot workspace in meters
        self.y_min, self.y_max = -0.3, 0.3

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
    def __init__(self, width=400, height=300):
        self.width = width
        self.height = height
        self.active = False
        self.current_pos = np.array([0.3, 0.0])  # Default position
        self.trajectory_log = []

        # Create window
        self.root = tk.Toplevel()
        self.root.title("Real-time Mouse Control")
        self.root.geometry(f"{width}x{height + 100}")
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
        self.x_min, self.x_max = 0.1, 0.6
        self.y_min, self.y_max = -0.3, 0.3

        self.running = True

    def canvas_to_robot_coords(self, canvas_x, canvas_y):
        x = self.x_min + (canvas_x / self.width) * (self.x_max - self.x_min)
        y = self.y_max - (canvas_y / self.height) * (self.y_max - self.y_min)
        return np.array([x, y])

    def mouse_move(self, event):
        if self.active:
            self.current_pos = self.canvas_to_robot_coords(event.x, event.y)
            self.trajectory_log.append(self.current_pos.copy())
            self.pos_label.config(text=f"Position: ({self.current_pos[0]:.3f}, {self.current_pos[1]:.3f})")
            print(f"Mouse control: x={self.current_pos[0]:.3f}, y={self.current_pos[1]:.3f}")

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
# Robot Control Functions
# ---------------------------------------------------------
def get_joint_positions(model, data, joint_names):
    positions = np.zeros(len(joint_names))
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        positions[i] = data.qpos[qadr]
    return positions


def set_joint_positions(model, data, joint_names, positions):
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        data.qpos[qadr] = positions[i]


def _clamp_limits(model, qpos, joint_names):
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            qpos[qadr] = np.clip(qpos[qadr], lo, hi)


def solve_ik(model, data, site_id, target_pos, joint_names,
             max_iters=100, step_size=0.1, tolerance=1e-3):
    """Simple IK solver"""
    joint_ids = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)]
                 for jn in joint_names]

    for _ in range(max_iters):
        mujoco.mj_forward(model, data)

        # Get current end-effector position
        current_pos = data.site_xpos[site_id][:2]  # Only X,Y
        error = target_pos - current_pos

        if np.linalg.norm(error) < tolerance:
            return True

        # Compute Jacobian
        jac_pos = np.zeros((3, model.nv))
        jac_rot = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jac_pos, jac_rot, site_id)

        # Use only X,Y components and relevant joints
        jac_xy = jac_pos[:2, joint_ids]

        # Compute joint velocities using pseudoinverse
        try:
            dq = np.linalg.pinv(jac_xy) @ (error * step_size)
        except np.linalg.LinAlgError:
            return False

        # Update joint positions
        current_joints = get_joint_positions(model, data, joint_names)
        new_joints = current_joints + dq
        set_joint_positions(model, data, joint_names, new_joints)
        _clamp_limits(model, data.qpos, joint_names)

    return False


# ---------------------------------------------------------
# Main DMP Controller Class
# ---------------------------------------------------------
class DMPController:
    def __init__(self, xml_path="ballmove.xml"):
        # Load model
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Robot configuration
        self.joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        self.site_name = "ee_site"
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)

        if self.site_id == -1:
            raise RuntimeError(f"Site '{self.site_name}' not found in model")

        # Default start position
        self.start_joint_positions = np.array([
            -2.89, -1.07, 0.377, -0.314, -0.0628, -0.503
        ])

        # Initialize viewer
        self.viewer = ViewerAdapter(self.model, self.data)

        # DMP instances
        self.discrete_dmp = None
        self.rhythmic_dmp = None

        # Control variables
        self.mode = "menu"
        self.running = True

        # Initialize robot position
        self.reset_robot()

        print("🤖 DMP Controller initialized successfully!")
        print("Available modes:")
        print("  1. Discrete DMP (point-to-point movements)")
        print("  2. Rhythmic DMP (mouse-drawn patterns)")
        print("  3. Real-time Mouse Control")

    def reset_robot(self):
        """Reset robot to start position"""
        set_joint_positions(self.model, self.data, self.joint_names, self.start_joint_positions)
        _clamp_limits(self.model, self.data.qpos, self.joint_names)
        mujoco.mj_forward(self.model, self.data)

        current_pos = self.data.site_xpos[self.site_id]
        print(f"🏠 Robot reset to start position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")

    def get_discrete_waypoints(self):
        """Get waypoints for discrete mode"""
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
                                                  f"Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
                if use_current:
                    points.append([current_pos[0], current_pos[1]])
                    continue

            # Get point coordinates
            coord_str = simpledialog.askstring("Waypoint",
                                               f"Enter {name} point coordinates (x, y):\n"
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

    def execute_discrete_mode(self):
        """Execute discrete DMP mode"""
        print("\n🎯 === DISCRETE DMP MODE ===")

        # Get waypoints
        waypoints = self.get_discrete_waypoints()
        if waypoints is None:
            print("Cancelled by user")
            return

        print(f"Waypoints defined: {len(waypoints)} points")
        for i, point in enumerate(waypoints):
            print(f"  Point {i + 1}: ({point[0]:.3f}, {point[1]:.3f})")

        # Create discrete DMP
        if MOVEMENT_PRIMITIVES_AVAILABLE:
            self.discrete_dmp = CartesianDMP(n_dmps=2, dt=0.01)
        else:
            self.discrete_dmp = SimpleDMP(n_dmps=2, dt=0.01)

        # Train DMP on waypoints
        self.discrete_dmp.imitate_path(waypoints)
        print("✅ DMP trained on trajectory")

        # Execute trajectory
        self.discrete_dmp.reset_state()
        trajectory = []

        print("🎬 Executing discrete trajectory...")

        dt = 0.01
        max_steps = int(5.0 / dt)  # 5 seconds maximum

        for step in range(max_steps):
            if not self.viewer.is_running():
                break

            # Get next position from DMP
            dmp_pos = self.discrete_dmp.step()
            z_pos = 0.15  # Fixed Z height for table cleaning
            target_3d = np.array([dmp_pos[0], dmp_pos[1], z_pos])

            # Solve IK and move robot
            success = solve_ik(self.model, self.data, self.site_id, dmp_pos, self.joint_names)
            if success:
                mujoco.mj_forward(self.model, self.data)
                trajectory.append(dmp_pos.copy())

            # Update visualization
            self.viewer.draw()
            time.sleep(dt)

            # Check if trajectory is complete
            if hasattr(self.discrete_dmp, 'x') and self.discrete_dmp.x < 0.01:
                break

        print(f"✅ Discrete trajectory completed with {len(trajectory)} points")

    def execute_rhythmic_mode(self):
        """Execute rhythmic DMP mode"""
        print("\n🎵 === RHYTHMIC DMP MODE ===")

        # Get trajectory from drawing interface
        drawing_interface = DrawingInterface(title="Draw Rhythmic Pattern")
        trajectory = drawing_interface.get_trajectory()

        if trajectory is None:
            print("Cancelled by user")
            return

        print(f"Trajectory captured: {len(trajectory)} points")
        print(f"X range: {trajectory[:, 0].min():.3f} to {trajectory[:, 0].max():.3f}")
        print(f"Y range: {trajectory[:, 1].min():.3f} to {trajectory[:, 1].max():.3f}")

        # Create rhythmic DMP
        if MOVEMENT_PRIMITIVES_AVAILABLE:
            # Use movement_primitives library if available
            try:
                from movement_primitives.dmp import RhythmicDMP
                self.rhythmic_dmp = RhythmicDMP(n_dmps=2, dt=0.01)
            except ImportError:
                self.rhythmic_dmp = SimpleRythmicDMP(n_dmps=2, dt=0.01)
        else:
            self.rhythmic_dmp = SimpleRythmicDMP(n_dmps=2, dt=0.01)

        # Train DMP on drawn trajectory
        self.rhythmic_dmp.imitate_path(trajectory)
        print("✅ Rhythmic DMP trained on drawn pattern")

        # Execute rhythmic movement
        self.rhythmic_dmp.reset_state()

        print("🎬 Executing rhythmic trajectory...")
        print("Press Ctrl+C in terminal to stop rhythmic movement")

        dt = 0.01
        try:
            while self.viewer.is_running():
                # Get next position from rhythmic DMP
                dmp_pos = self.rhythmic_dmp.step()

                # Add offset to center the pattern on table
                center_offset = np.array([0.3, 0.0])  # Table center
                final_pos = center_offset + dmp_pos * 0.1  # Scale down pattern

                # Solve IK and move robot
                success = solve_ik(self.model, self.data, self.site_id, final_pos, self.joint_names)

                if success:
                    mujoco.mj_forward(self.model, self.data)

                # Update visualization
                self.viewer.draw()
                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n⏹️ Rhythmic movement stopped by user")

    def execute_realtime_mode(self):
        """Execute real-time mouse control mode"""
        print("\n🖱️ === REAL-TIME MOUSE CONTROL MODE ===")

        # Create mouse control interface
        mouse_control = RealTimeMouseControl()

        print("Mouse control interface opened. Click to activate control.")
        print("Move mouse to control robot end-effector position.")

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
        """Control loop for real-time mouse control"""
        while mouse_control.running and self.viewer.is_running():
            if mouse_control.is_active():
                target_pos = mouse_control.get_current_position()

                # Solve IK and move robot
                success = solve_ik(self.model, self.data, self.site_id, target_pos, self.joint_names)

                if success:
                    mujoco.mj_forward(self.model, self.data)

                # Update visualization
                self.viewer.draw()

            time.sleep(dt)

    def run(self):
        """Main control loop"""
        print("\n🚀 DMP Controller Started!")
        print("=" * 50)

        while self.running and self.viewer.is_running():
            print("\n📋 MAIN MENU:")
            print("1. Discrete DMP Mode (waypoint navigation)")
            print("2. Rhythmic DMP Mode (mouse-drawn patterns)")
            print("3. Real-time Mouse Control")
            print("4. Reset Robot Position")
            print("5. Quit")

            try:
                choice = input("\nSelect mode (1-5): ").strip()

                if choice == '1':
                    self.execute_discrete_mode()
                elif choice == '2':
                    self.execute_rhythmic_mode()
                elif choice == '3':
                    self.execute_realtime_mode()
                elif choice == '4':
                    self.reset_robot()
                elif choice == '5':
                    self.running = False
                else:
                    print("Invalid choice. Please select 1-5.")

                # Brief pause between operations
                time.sleep(0.5)

            except KeyboardInterrupt:
                print("\n\n⏹️ Interrupted by user")
                break
            except EOFError:
                break

        self.viewer.close()
        print("👋 DMP Controller shut down")


def main():
    """Main function"""
    xml_path = "ballmove.xml"

    try:
        controller = DMPController(xml_path)
        controller.run()
    except FileNotFoundError:
        print(f"❌ Error: XML file '{xml_path}' not found!")
        print("Please ensure the XML file is in the current directory.")
    except Exception as e:
        print(f"❌ Error initializing controller: {e}")


if __name__ == "__main__":
    main()
#%%
import numpy as np
import matplotlib.pyplot as plt
from pydmps.dmp_discrete import DMPs_discrete
from pydmps.dmp_rhythmic import DMPs_rhythmic
def circle_trajectory(center, radius, num_points=100, plot=True, color='b', linestyle='-'):
    """
    Generate and optionally plot a circular trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    if plot:
        plt.plot(x, y, color=color, linestyle=linestyle)
        plt.axis('equal')
        plt.grid(True)
    return x, y

def elipsoid_trajectory(center, axes_lengths, angle=0, num_points=100, plot=True, color='g', linestyle='-'):
    """
    Generate and optionally plot an ellipsoid trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = axes_lengths[0] * np.cos(t)
    y = axes_lengths[1] * np.sin(t)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    ellipse = R @ np.vstack((x, y))
    X = center[0] + ellipse[0, :]
    Y = center[1] + ellipse[1, :]
    if plot:
        plt.plot(X, Y, color=color, linestyle=linestyle)
        plt.axis('equal')
        plt.grid(True)
    return X, Y

def square_trajectory(center, side_length, num_points=100, plot=True, color='m', linestyle=':'):
    """
    Generate and optionally plot a square trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    half_side = side_length / 2
    corners = np.array([
        [center[0] - half_side, center[1] - half_side],
        [center[0] + half_side, center[1] - half_side],
        [center[0] + half_side, center[1] + half_side],
        [center[0] - half_side, center[1] + half_side],
        [center[0] - half_side, center[1] - half_side]
    ])
    points_per_edge = max(1, num_points // 4)
    x_traj, y_traj = [], []
    for i in range(4):
        start = corners[i]
        end = corners[i+1]
        xs = np.linspace(start[0], end[0], points_per_edge, endpoint=False)
        ys = np.linspace(start[1], end[1], points_per_edge, endpoint=False)
        x_traj.extend(xs)
        y_traj.extend(ys)
    x_traj.append(corners[0][0])
    y_traj.append(corners[0][1])
    if plot:
        plt.plot(x_traj, y_traj, color=color, linestyle=linestyle)
        plt.axis('equal')
        plt.grid(True)
    return np.array(x_traj), np.array(y_traj)

def triangle_trajectory(center, side_length, num_points=100, plot=True, color='c', linestyle='--'):
    """
    Generate and optionally plot a triangle trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    height = (np.sqrt(3) / 2) * side_length
    corners = np.array([
        [center[0] - side_length / 2, center[1] - height / 3],
        [center[0] + side_length / 2, center[1] - height / 3],
        [center[0], center[1] + 2 * height / 3],
        [center[0] - side_length / 2, center[1] - height / 3]
    ])
    points_per_edge = max(1, num_points // 3)
    x_traj, y_traj = [], []
    for i in range(3):
        start = corners[i]
        end = corners[i+1]
        xs = np.linspace(start[0], end[0], points_per_edge, endpoint=False)
        ys = np.linspace(start[1], end[1], points_per_edge, endpoint=False)
        x_traj.extend(xs)
        y_traj.extend(ys)
    x_traj.append(corners[0][0])
    y_traj.append(corners[0][1])
    if plot:
        plt.plot(x_traj, y_traj, color=color, linestyle=linestyle)
        plt.axis('equal')
        plt.grid(True)
    return np.array(x_traj), np.array(y_traj)

def infinity_trajectory(center, size=(1.0, 1.0), num_points=200, plot=True, color='orange', linestyle='-'):
    """
    Generate and optionally plot an infinity (figure-eight) trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    The parametric equation used is:
        x = a * sin(t)
        y = a * sin(t) * cos(t)
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    a = size[0] / 2.0  # scale to match other shapes
    b = size[1] / 2.0
    x = center[0] + a * np.sin(t)
    y = center[1] + b * np.sin(t) * np.cos(t)
    if plot:
        plt.plot(x, y, color=color, linestyle=linestyle)
        plt.axis('equal')
        plt.grid(True)
    return x, y

def plot_trajectory(traj, color='k', linestyle='-', label=None):
    """
    Plot a 2xN trajectory (traj[0,:]=x, traj[1,:]=y).
    """
    x = traj[0, :]
    y = traj[1, :]
    plt.plot(x, y, color=color, linestyle=linestyle, label=label)
    plt.axis('equal')
    plt.grid(True)
    if label is not None:
        plt.legend()

# Example usage:
circle_trajectory(center=(0, 0), radius=0.6, num_points=200, plot=True)
elipsoid_trajectory(center=(0, 0), axes_lengths=(1.0, 0.6), angle=0, num_points=200, plot=True)
square_trajectory(center=(0, 0), side_length=1.0, num_points=200, plot=True)
triangle_trajectory(center=(0, 0), side_length=1.0, num_points=200, plot=True)
infinity_trajectory(center=(0, 0), size=(2, 2.5), num_points=500, plot=True)
plt.show()
# %%
x_traj, y_traj = circle_trajectory(center=(0, 0), radius=1.0, num_points=200, plot=False)
dmp_traj = []
traj = np.vstack((x_traj, y_traj))
# dmp = DMPs_discrete(n_dmps=2, n_bfs=50, dt=0.001)
dmp = DMPs_rhythmic(n_dmps=2, n_bfs=10, dt=0.001)

dmp.imitate_path(traj, plot=True)

for step in range(dmp.timesteps):
    dmp_point, _, _ = dmp.step()
    # print(dmp_point)
    dmp_traj.append(dmp_point.copy())
dmp_traj = np.array(dmp_traj).T
plot_trajectory(traj, color='m', linestyle='-', label='Original Trajectory')
plot_trajectory(dmp_traj, color='k', linestyle='--', label='DMP Reproduction')
plt.title("DMP Reproduction of Circle Trajectory")
plt.legend()
plt.show()

plt.plot(np.linspace(0, 1, dmp_traj.shape[1]), dmp_traj[0, :], color='k', linestyle='--', label='DMP x')
plt.plot(np.linspace(0, 1, len(x_traj)), x_traj, color='m', linestyle='-', label='Original x')
plt.grid(True)
plt.legend()
plt.show()

plt.plot(np.linspace(0, 1, dmp_traj.shape[1]), dmp_traj[1, :], color='k', linestyle='--', label='DMP y')
plt.plot(np.linspace(0, 1, len(y_traj)), y_traj, color='m', linestyle='-', label='Original y')
plt.grid(True)
plt.legend()
plt.show()
#%%
import mujoco
import mujoco.viewer
import time

XML_PATH = "ballmove.xml"
SITE_NAME = "ee_site"
UR5E_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
MOP_Z_HEIGHT = 0.49
HOME_JOINT_POSITIONS = np.array([0.188, -2.18, -0.87, 0.0, np.pi / 2, np.pi / 2])
dt = 0.002

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


# --- 2. HELPER FUNCTIONS FROM TESTING_2.PY ---
def _clamp_limits(model, qpos, joint_names):

    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            qpos[qadr] = np.clip(qpos[qadr], lo, hi)


def _interpolate_path(p0, p1, max_step=0.03):

    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    dist = np.linalg.norm(p1 - p0)
    if dist <= max_step:
        return [p1]
    n = int(np.ceil(dist / max_step))
    alphas = np.linspace(0.0, 1.0, n + 1)[1:]
    return [p0 * (1 - a) + p1 * a for a in alphas]

def get_joint_positions(model, data, joint_names):
    """Get current joint positions as numpy array"""
    positions = np.zeros(len(joint_names))
    mujoco.mj_forward(model, data)
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        positions[i] = data.qpos[qadr]
    return positions

def enhanced_ik_solver(model, data, site_id, goal_pos_3d, joint_names,
                       step_clip=0.2, max_wp_step=0.03, max_iters_per_wp=50,
                       lam_init=0.1, lam_inc=2.0, lam_dec=0.85, tol=1e-3):
    # Identify degrees of freedom
    dof_cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        dof_cols.append(model.jnt_dofadr[jid])
    dof_cols = np.asarray(dof_cols, int)

    # Helper to enforce limits
    def clamp_limits():
        _clamp_limits(model, data.qpos, joint_names)
        mujoco.mj_forward(model, data)  # Kinematics update only

    start = np.array(data.site_xpos[site_id])
    waypoints = _interpolate_path(start, goal_pos_3d, max_step=max_wp_step)


    for wp in waypoints:
        lam = lam_init
        mujoco.mj_forward(model, data)
        prev_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))

        for it in range(max_iters_per_wp):
            # 1. Calculate Jacobian (The "Formula" requires this)
            mujoco.mj_forward(model, data)
            Jp = np.zeros((3, model.nv))
            Jr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Jp, Jr, site_id)
            J = Jp[:, dof_cols]
            e = wp - np.array(data.site_xpos[site_id])


            A = J.T @ J + lam * np.eye(J.shape[1])
            b = J.T @ e

            try:
                dq = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(A) @ b

            # 3. Apply changes
            nq = np.linalg.norm(dq)
            if nq > step_clip:
                dq *= (step_clip / (nq + 1e-12))

            qpos_before = data.qpos.copy()
            for k, jn in enumerate(joint_names):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] += dq[k]

            clamp_limits()
            mujoco.mj_forward(model, data)
            new_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))

            # 4. Adaptive Damping (Trust Region)
            if new_err < prev_err - 1e-6:
                prev_err = new_err
                lam = max(1e-6, lam * lam_dec)
            else:
                data.qpos[:] = qpos_before  # Rollback
                mujoco.mj_forward(model, data)
                lam *= lam_inc

            if prev_err < tol:
                break

    final_err = np.linalg.norm(goal_pos_3d - np.array(data.site_xpos[site_id]))
    return final_err < tol

def set_joint_positions(model, data, joint_names, positions):
    """
    Manually sets the robot's joint positions in the MuJoCo data structure.
    """
    for i, jn in enumerate(joint_names):
        # Find the ID of the joint by name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)

        # Get the address in the qpos array
        qadr = model.jnt_qposadr[jid]

        # Set the value
        data.qpos[qadr] = positions[i]

def run_ik_recreation(shape_name, x_traj, y_traj):
    print(f"--- Recreating {shape_name} using IK Formula ---")

    # 1. Load Model
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    except Exception as e:
        print(f"Error: Could not load {XML_PATH}. {e}")
        return

    # 2. Train DMP
    traj = np.vstack((x_traj, y_traj))
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=10, dt=0.001)
    dmp.imitate_path(traj, plot=False)

    # 3. CRITICAL STEP: Move Robot to the START of the trajectory first
    start_x = x_traj[0]
    start_y = y_traj[0]
    start_target_3d = np.array([start_x, start_y, MOP_Z_HEIGHT])

    # Reset to home first
    set_joint_positions(model, data, UR5E_JOINTS, HOME_JOINT_POSITIONS)
    mujoco.mj_forward(model, data)

    print(f"Moving robot to start position: {start_target_3d}...")
    # Run IK to get to start position (using the solver you defined earlier)
    enhanced_ik_solver(model, data, site_id, start_target_3d, UR5E_JOINTS,
                       max_iters_per_wp=100, tol=1e-4)

    # 4. Execute Full Loop
    dmp_desired_xy = []
    ik_actual_xy = []
    dmp.reset_state()

    print(f"Executing trajectory for {dmp.timesteps} steps...")

    for _ in range(dmp.timesteps):
        # A. Get DMP Target
        dmp_pos_2d, _, _ = dmp.step()
        target_3d = np.array([dmp_pos_2d[0], dmp_pos_2d[1], MOP_Z_HEIGHT])

        # B. Apply IK Formula
        enhanced_ik_solver(model, data, site_id, target_3d, UR5E_JOINTS,
                           max_iters_per_wp=20)

        # C. Read ACTUAL Position
        actual_pos = data.site_xpos[site_id].copy()

        # D. Store Data
        dmp_desired_xy.append(dmp_pos_2d.copy())
        ik_actual_xy.append(actual_pos[:2])


    dmp_desired_xy = np.array(dmp_desired_xy).T
    ik_actual_xy = np.array(ik_actual_xy).T

    plt.figure(figsize=(10, 10))
    plt.plot(dmp_desired_xy[0], dmp_desired_xy[1], 'k-', linewidth=3, label='DMP Target')
    plt.plot(ik_actual_xy[0], ik_actual_xy[1], 'r--', linewidth=1.5, label='IK Solution')

    plt.title(f"Comparison: DMP Target vs IK Solution ({shape_name})")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def set_joint_pid_gains(model, joint_names, kp_values, kd_values):
    """
    Set kp and kd for each joint actuator in MuJoCo.
    kp_values and kd_values should be lists/arrays of same length as joint_names.
    """
    for i, jn in enumerate(joint_names):
        # Get joint ID and DOF ID
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        dof_id = model.jnt_dofadr[joint_id]

        # Find actuator ID by checking which joint it controls
        actuator_id = -1
        for aid in range(model.nu):
            # trnid[aid, 0] stores the joint index for the actuator
            if model.actuator_trnid[aid, 0] == joint_id:
                actuator_id = aid
                break
        
        if actuator_id != -1:
            # Set kp (proportional gain)
            model.actuator_gainprm[actuator_id, 0] = kp_values[i]
            # Set bias for position servo (standard MuJoCo position actuator formula)
            model.actuator_biasprm[actuator_id, 1] = -kp_values[i] 
        else:
            print(f"Warning: No actuator found for joint '{jn}'")

def get_joint_pid_gains(model, joint_names):
    """
    Retrieve kp and kd for each joint actuator in MuJoCo.
    Returns a list of dicts: [{'joint': name, 'actuator_id': id, 'dof_id': id, 'kp': val, 'kd': val}, ...]
    """
    gains = []
    for jn in joint_names:
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jn)
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        dof_id = model.jnt_dofadr[joint_id]
        kp = model.actuator_gainprm[joint_id, 0]
        kd = model.dof_damping[dof_id]
        gains.append({
            'joint': jn,
            'actuator_id': actuator_id,
            'dof_id': dof_id,
            'kp': kp,
            'kd': kd
        })
    return gains

# x, y = circle_trajectory(center=(0.0, 0.0), radius=0.6, num_points=200, plot=False)
# run_ik_recreation("Circle", x, y)

# x, y = elipsoid_trajectory(center=(0.0, 0.0), axes_lengths=(0.6, 0.8), angle=0.5, num_points=200, plot=False)
# run_ik_recreation("Ellipsoid", x, y)


# x, y = square_trajectory(center=(0.0, 0.0), side_length=1.2, num_points=200, plot=False)
# run_ik_recreation("Square", x, y)

# x, y = triangle_trajectory(center=(0.0, 0.0), side_length=1.2, num_points=200, plot=False)
# run_ik_recreation("Triangle", x, y)


# x, y = infinity_trajectory(center=(0.0, 0.0), size=(2.8, 3.0), num_points=400, plot=False)
# run_ik_recreation("Infinity", x, y)

# %%
import os
results_dir = "./Results"
os.makedirs(results_dir, exist_ok=True)
if __name__ == "__main__":
    # 1. Load Model
    try:
        model = mujoco.MjModel.from_xml_path(XML_PATH)
        data = mujoco.MjData(model)
        viewer = ViewerAdapter(model, data)
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    except Exception as e:
        print(f"Error: Could not load {XML_PATH}. {e}")
        
    # Tune PID Gains
    # kp = [500, 500, 500, 500, 500, 500]  # proportional gains 
    # kd = [0, 0, 0, 0, 0, 0]         # damping values
    kp = [3000, 3000, 1500, 1000, 1000, 1000]
    kd = [150, 150, 80, 40, 10, 10]  
    set_joint_pid_gains(model, UR5E_JOINTS, kp, kd)
    
    # 2. Train DMP
    # get desired trajectory
    x_traj, y_traj = infinity_trajectory(center=(0.0, 0.0), size=(2.0, 2.5), num_points=400, plot=False)
    traj = np.vstack((x_traj, y_traj))
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=20, dt=0.001)
    dmp.imitate_path(traj, plot=False)

    # 3. CRITICAL STEP: Move Robot to the START of the trajectory first
    start_x = x_traj[0]
    start_y = y_traj[0]
    start_target_3d = np.array([start_x, start_y, MOP_Z_HEIGHT])

    # Reset to home first
    set_joint_positions(model, data, UR5E_JOINTS, HOME_JOINT_POSITIONS)
    mujoco.mj_forward(model, data)

    print(f"Moving robot to start position: {start_target_3d}...")
    # Run IK to get to start position (using the solver you defined earlier)
    enhanced_ik_solver(model, data, site_id, start_target_3d, UR5E_JOINTS,
                       max_iters_per_wp=100, tol=1e-4)

    # 4. Execute Full Loop
    dmp_desired_xy = []
    ik_xy = []
    actual_xy = []
    joint_traj = []
    actual_joint_traj = []

    print(f"Executing trajectory for {dmp.timesteps} steps...")

    for _ in range(dmp.timesteps):
        # A. Get DMP Target
        dmp_pos_2d, _, _ = dmp.step(tau=1.0)
        target_3d = np.array([dmp_pos_2d[0], dmp_pos_2d[1], MOP_Z_HEIGHT])

        # B. Apply IK Formula
        enhanced_ik_solver(model, data, site_id, target_3d, UR5E_JOINTS,
                           max_iters_per_wp=20)
        
        # C. Store Joint Positions
        joint_positions = get_joint_positions(model, data, UR5E_JOINTS)

        # D. Read IK Position
        ik_pos = data.site_xpos[site_id].copy()

        # D. Store Data
        dmp_desired_xy.append(dmp_pos_2d.copy())
        ik_xy.append(ik_pos[:2])
        joint_traj.append(joint_positions.copy())
        
    if len(joint_traj) > 0:
        set_joint_positions(model, data, UR5E_JOINTS, joint_traj[0])
        mujoco.mj_forward(model, data)
        viewer.draw()
        time.sleep(dt)

    for joints in joint_traj:
        data.ctrl[:] = joints
        mujoco.mj_step(model, data)
        viewer.draw()
        time.sleep(dt)
        cl_pos = data.site_xpos[site_id].copy()
        actual_xy.append(cl_pos[:2].copy())
        # Record actual joint positions after step
        actual_joint_traj.append(get_joint_positions(model, data, UR5E_JOINTS))

    print("Discrete trajectory execution complete.")
    dmp_desired_xy = np.array(dmp_desired_xy).T
    ik_xy = np.array(ik_xy).T
    actual_xy = np.array(actual_xy).T
    actual_joint_traj = np.array(actual_joint_traj)  # shape: (steps, joints)
    joint_traj = np.array(joint_traj)                # shape: (steps, joints)

    # Plot XY Trajectories
    plt.figure(figsize=(10, 10))
    plt.plot(x_traj, y_traj, 'g:', linewidth=1.5, label='Original Path')
    plt.plot(dmp_desired_xy[0], dmp_desired_xy[1], 'k-', linewidth=3, label='DMP Target')
    plt.plot(ik_xy[0], ik_xy[1], 'r--', linewidth=1.5, label='IK Solution')
    plt.plot(actual_xy[0], actual_xy[1], 'b-.', linewidth=1.5, label='Executed Path')

    plt.title(f"Comparison: DMP Target vs IK Solution vs Executed Path")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.savefig(os.path.join(results_dir, "xy_trajectories.png"))
    plt.show()

    # Plot joint trajectories
    plt.figure(figsize=(12, 8))
    for i, jn in enumerate(UR5E_JOINTS):
        plt.subplot(3, 2, i+1)
        plt.plot(joint_traj[:, i], label='Planned')
        plt.plot(actual_joint_traj[:, i], label='Actual', linestyle='--')
        plt.title(jn)
        plt.xlabel('Step')
        plt.ylabel('Joint Position (rad)')
        plt.legend()
        plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "joint_trajectories.png"))
    plt.show()
    
    gains = get_joint_pid_gains(model, UR5E_JOINTS)
    for g in gains:
        print(g)
# %%

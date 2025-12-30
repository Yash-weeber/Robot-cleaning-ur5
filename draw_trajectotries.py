#%%
import re
import numpy as np
import matplotlib.pyplot as plt
from pydmps.dmp_discrete import DMPs_discrete
from pydmps.dmp_rhythmic import DMPs_rhythmic
from utils.draw_shapes import *

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
circle_trajectory(center=(0, -0.1), radius=0.5, num_points=200, plot=False)
elipsoid_trajectory(center=(0, 0), axes_lengths=(1.0, 0.3), angle=np.pi/6, num_points=200, plot=True)
square_trajectory(center=(0, -0.1), side_length=1, num_points=200, plot=True)
triangle_trajectory(center=(0, -0.2), side_length=1.25, num_points=200, plot=True)
infinity_trajectory(center=(0, 0), size=(2, 2.5), num_points=500, plot=True)
x_, y_ = rectangle_trajectory(center=(0.0, 0.0), width=2.1, height=1.3, num_points=400, plot=True)
obs = np.vstack((x_, y_)).T
obs = np.vstack((np.array([0, 0.5]),obs))
plt.scatter(obs[:,0], obs[:,1], color='r', label='Rectangle Traj Points')
plt.show()
print(6//2)
#%% Obstacle avoidance test with DMPs
beta = 20.0 / np.pi
gamma = 500
R_halfpi = np.array(
        [
            [np.cos(np.pi / 2.0), -np.sin(np.pi / 2.0)],
            [np.sin(np.pi / 2.0), np.cos(np.pi / 2.0)],
        ]
    )

x_const, y_const = rectangle_trajectory(center=(0.0, 0.0), width=2.1, height=1.2, num_points=200, plot=False)
obstacles = np.vstack((x_const, y_const)).T
obstacles = np.vstack((np.array([0, 0.5]),obstacles))

# def avoid_obstacles(y, dy, goal):
#     p = np.zeros(2)

#     for obstacle in obstacles:
#         # based on (Hoffmann, 2009)

#         # if we're moving
#         # if np.linalg.norm(dy) > 1e-5:

#         # get the angle we're heading in
#         phi_dy = -np.arctan2(dy[1], dy[0])
#         R_dy = np.array(
#             [[np.cos(phi_dy), -np.sin(phi_dy)], [np.sin(phi_dy), np.cos(phi_dy)]]
#         )
#         # calculate vector to object relative to body
#         obj_vec = obstacle - y
#         # rotate it by the direction we're going
#         obj_vec = np.dot(R_dy, obj_vec)
#         # calculate the angle of obj relative to the direction we're going
#         phi = np.arctan2(obj_vec[1], obj_vec[0])

#         dphi = gamma * phi * np.exp(-beta * abs(phi))
#         R = np.dot(R_halfpi, np.outer(obstacle - y, dy))
#         pval = -np.nan_to_num(np.dot(R, dy) * dphi)

#         # check to see if the distance to the obstacle is further than
#         # the distance to the target, if it is, ignore the obstacle
#         if np.linalg.norm(obj_vec) > np.linalg.norm(goal - y):
#             pval = 0

#         p += pval
#     return p

# def avoid_obstacles(y, dy, goal, *, eta=0.5, d0=1.75, max_force=10.0, nearest_only=True):
#     """
#     Distance-based repulsive potential field.

#     For obstacle point o and current position y:
#         d = ||y - o||
#         if d < d0:
#             F_rep = eta * (1/d - 1/d0) * (1/d^2) * ( (y-o)/d )

#     Notes:
#     - d0 is the influence radius (meters).
#     - eta scales the strength.
#     - nearest_only=True is recommended when 'obstacles' is a dense point set.
#     """
#     y = np.asarray(y, dtype=float).reshape(2,)
#     goal = np.asarray(goal, dtype=float).reshape(2,)

#     p = np.zeros(2, dtype=float)
#     eps = 1e-9

#     # Select which obstacle points to consider
#     if nearest_only:
#         diffs = y[None, :] - obstacles               # vectors from obstacle -> y
#         dists = np.linalg.norm(diffs, axis=1) + eps
#         o_list = [obstacles[int(np.argmin(dists))]]
#     else:
#         o_list = obstacles

#     for o in o_list:
#         o = np.asarray(o, dtype=float).reshape(2,)
#         r = y - o
#         d = np.linalg.norm(r) + eps

#         # Outside influence radius -> no effect
#         if d >= d0:
#             continue

#         # Optional: ignore obstacles that are farther than the remaining distance to goal
#         if d > (np.linalg.norm(goal - y) + eps):
#             continue

#         # Repulsive magnitude (classic potential-field form)
#         mag = eta * (1.0 / d - 1.0 / d0) * (1.0 / (d * d))

#         # Direction away from obstacle
#         p += (r / d) * mag

#     # Clamp to keep DMP stable
#     n = np.linalg.norm(p)
#     if n > max_force:
#         p *= (max_force / (n + eps))

#     return p

# Rectangle keep-in zone boundaries/parameters
RECT_CENTER = np.array([0.0, 0.0], dtype=float)
RECT_WIDTH = 2.0
RECT_HEIGHT = 1.2

XMIN = RECT_CENTER[0] - RECT_WIDTH / 2.0
XMAX = RECT_CENTER[0] + RECT_WIDTH / 2.0
YMIN = RECT_CENTER[1] - RECT_HEIGHT / 2.0
YMAX = RECT_CENTER[1] + RECT_HEIGHT / 2.0

# Obstacle points inside the rectangle
INTERNAL_OBSTACLES = np.array([[0.0, 0.5]], dtype=float)


def _project_into_rect(y):
    """Hard projection into keep-in rectangle."""
    y = np.asarray(y, dtype=float).reshape(2,)
    return np.array([np.clip(y[0], XMIN, XMAX), np.clip(y[1], YMIN, YMAX)], dtype=float)


def _keep_in_rect_force(y, *, d0=0.10, eta=0.002, k_out=200.0):
    """
    Wall-based keep-in force for an axis-aligned rectangle.
    - Inside but within d0 of a wall: smooth repulsion away from the wall.
    - Outside: strong linear push back inside (k_out).
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)
    eps = 1e-9

    # X walls
    if y[0] < XMIN:
        p[0] += k_out * (XMIN - y[0])
    else:
        d = y[0] - XMIN
        if d < d0:
            dd = d + eps
            p[0] += eta * (1.0 / dd - 1.0 / d0) * (1.0 / (dd * dd))

    if y[0] > XMAX:
        p[0] -= k_out * (y[0] - XMAX)
    else:
        d = XMAX - y[0]
        if d < d0:
            dd = d + eps
            p[0] -= eta * (1.0 / dd - 1.0 / d0) * (1.0 / (dd * dd))

    # Y walls
    if y[1] < YMIN:
        p[1] += k_out * (YMIN - y[1])
    else:
        d = y[1] - YMIN
        if d < d0:
            dd = d + eps
            p[1] += eta * (1.0 / dd - 1.0 / d0) * (1.0 / (dd * dd))

    if y[1] > YMAX:
        p[1] -= k_out * (y[1] - YMAX)
    else:
        d = YMAX - y[1]
        if d < d0:
            dd = d + eps
            p[1] -= eta * (1.0 / dd - 1.0 / d0) * (1.0 / (dd * dd))

    return p


def _repulsive_point_obstacles_force(y, obstacles_xy, *, d0=0.20, eta=0.02):
    """
    Distance-based repulsive potential field for point obstacles.
    Only active within radius d0.
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)
    eps = 1e-9

    if obstacles_xy is None or len(obstacles_xy) == 0:
        return p

    obstacles_xy = np.asarray(obstacles_xy, dtype=float).reshape(-1, 2)

    for o in obstacles_xy:
        r = y - o
        d = np.linalg.norm(r) + eps
        if d >= d0:
            continue

        mag = eta * (1.0 / d - 1.0 / d0) * (1.0 / (d * d))
        p += (r / d) * mag

    return p


def avoid_obstacles(
    y,
    dy,
    goal,
    *,
    # keep-in rectangle params
    rect_d0=0.10,
    rect_eta=0.2,
    rect_k_out=200.0,
    # internal obstacle params
    obs_d0=0.25,
    obs_eta=5,
    # global clamp
    max_force=50.0,
):
    """
    Combined coupling term:
    - keep-in rectangle (walls)
    - keep-away internal point obstacles 
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)

    p += _keep_in_rect_force(y, d0=rect_d0, eta=rect_eta, k_out=rect_k_out)
    p += _repulsive_point_obstacles_force(y, INTERNAL_OBSTACLES, d0=obs_d0, eta=obs_eta)

    # Clamp for stability
    eps = 1e-9
    n = np.linalg.norm(p)
    if n > max_force:
        p *= (max_force / (n + eps))

    return p


# %%
x_traj, y_traj = rectangle_trajectory(center=(0.0, 0.0), width=2.4, height=1.0, num_points=100, plot=False)
dmp_traj = []
traj = np.vstack((x_traj, y_traj))
traj = np.vstack((np.array([0.0, 0.0]), traj.T)).T
# dmp = DMPs_discrete(n_dmps=2, n_bfs=50, dt=0.001)
dmp = DMPs_rhythmic(n_dmps=2, n_bfs=10, dt=0.001)

dmp.imitate_path(traj, plot=True)

for step in range(dmp.timesteps):
    dmp_point, _, _ = dmp.step(tau=2.0,external_force=avoid_obstacles(dmp.y, dmp.dy, dmp.goal))
    # print(dmp_point)
    dmp_traj.append(dmp_point.copy())
dmp_traj = np.array(dmp_traj).T
plot_trajectory(traj, color='m', linestyle='-', label='Original Trajectory')
plot_trajectory(dmp_traj, color='k', linestyle='--', label='DMP Reproduction')
plt.title("DMP Reproduction of Circle Trajectory")
plt.legend()
plt.show()

# plt.plot(np.linspace(0, 1, dmp_traj.shape[1]), dmp_traj[0, :], color='k', linestyle='--', label='DMP x')

for obstacle in obstacles:
    (plot_obs,) = plt.plot(obstacle[0], obstacle[1], "ro", mew=1, markersize=1)
plt.plot(dmp_traj[0, :], dmp_traj[1,:], color='k', linestyle='--', label='DMP traj')
# plt.plot(np.linspace(0, 1, len(x_traj)), x_traj, color='m', linestyle='-', label='Original x')
# plt.grid(True)
# plt.legend()
# plt.show()

# plt.plot(np.linspace(0, 1, dmp_traj.shape[1]), dmp_traj[1, :], color='k', linestyle='--', label='DMP y')
# plt.plot(np.linspace(0, 1, len(y_traj)), y_traj, color='m', linestyle='-', label='Original y')
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
    # kp = [3000, 3000, 1500, 1000, 1000, 1000]
    # kd = [150, 150, 80, 40, 10, 10]  
    # set_joint_pid_gains(model, UR5E_JOINTS, kp, kd)
    
    # 2. Train DMP
    # get desired trajectory
    x_traj, y_traj = infinity_trajectory(center=(0.0, 0.0), size=(2.0, 2.5), num_points=400, plot=False)
    traj = np.vstack((x_traj, y_traj))
    traj = np.vstack((np.array([0.0, 0.0]), traj.T)).T
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=20, dt=0.001)
    dmp.imitate_path(traj, plot=False)
    np.random.seed(42)
    dmp.w += np.random.randn(*dmp.w.shape) * 20  # add noise to weights for testing

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
        dmp_pos_2d, _, _ = dmp.step(tau=2.0, 
                                    external_force=avoid_obstacles(dmp.y, dmp.dy, dmp.goal, rect_eta=0.5, obs_d0=0.25, obs_eta=25))
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

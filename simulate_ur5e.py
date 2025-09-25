#%%
import random
import mujoco
import mujoco.viewer
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# --- Impedance Controller Class ---
class ImpedanceController:
    def __init__(self, model, data, ee_site_name, joint_names):
        self.model = model
        self.data = data
        self.ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
        self.robot_dof_indices = np.array([model.jnt_dofadr[jid] for jid in self.joint_ids])
        self.kp = 400
        self.ko = 5
        self.kv = 300
        self.kv_null = 100.0
        self.ctrlr_dof = np.array([True, True, True, False, False, False])
        self.torque_limit = 120.0

    def euler_rpy_to_quat(self, roll, pitch, yaw):
        cr, sr = np.cos(roll/2), np.sin(roll/2)
        cp, sp = np.cos(pitch/2), np.sin(pitch/2)
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        w = cy*cp*cr + sy*sp*sr
        x = cy*cp*sr - sy*sp*cr
        y = cy*sp*cr + sy*cp*sr
        z = sy*cp*cr - cy*sp*sr
        return np.array([w, x, y, z], dtype=float)

    def get_site_quat(self):
        q = np.zeros(4, dtype=float)
        mujoco.mju_mat2Quat(q, self.data.site_xmat[self.ee_site_id])
        return q

    def calc_task_space_error(self, target, current_pos, current_quat):
        u_task = np.zeros(6, dtype=float)
        u_task[:3] = self.kp * (target[0] - current_pos)
        q_conj = np.array([current_quat[0], -current_quat[1], -current_quat[2], -current_quat[3]])
        q_err = np.zeros(4)
        mujoco.mju_mulQuat(q_err, target[1], q_conj)
        u_task[3:] = self.ko * q_err[1:]
        return u_task

    def svd_solve(self, A):
        u, s, v = np.linalg.svd(A)
        for i in range(len(s)):
            if s[i] < 1e-10:
                s[i] = 0.0
            else:
                s[i] = 1.0 / s[i]
        Ainv = np.dot(v.transpose(), np.dot(np.diag(s), u.transpose()))
        return Ainv

    def get_Mx(self, J, M):
        M_inv = self.svd_solve(M)
        Mx_inv = J @ M_inv @ J.T
        threshold = 1e-4
        if abs(np.linalg.det(Mx_inv)) >= threshold:
            Mx = self.svd_solve(Mx_inv)
        else:
            Mx = np.linalg.pinv(Mx_inv, rcond=threshold*0.1)
        return Mx, M_inv

    def step(self, target_xyz, target_quat):
        # Current EE pose
        x = self.data.site_xpos[self.ee_site_id].copy()
        q_site = self.get_site_quat()

        # Jacobians
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.ee_site_id)
        J = np.vstack((jacp, jacr))
        J_robot = J[:, self.robot_dof_indices]

        # Mass matrix
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)
        M_robot = M_full[np.ix_(self.robot_dof_indices, self.robot_dof_indices)]

        # Joint velocities
        dq = self.data.qvel[self.robot_dof_indices]

        # Task-space error
        pos_target = [target_xyz, target_quat]
        u_task = self.calc_task_space_error(pos_target, x, q_site)
        # u_task[~self.ctrlr_dof] = 0.0 # Zero out orientation error 

        # Task-space inertia
        Mx, M_inv = self.get_Mx(J_robot, M_robot)

        # Joint-space control
        uv_all = np.dot(M_robot, dq)
        u = J_robot.T @ (Mx @ u_task)
        u += -1 * self.kv * uv_all
        u += self.data.qfrc_bias[self.robot_dof_indices]

        # Null-space damping
        u_null = M_robot @ (-self.kv_null * dq)
        J_bar = M_inv @ J_robot.T @ Mx
        null_filter = np.eye(len(self.robot_dof_indices)) - J_robot.T @ J_bar.T
        u += null_filter @ u_null

        # Torque limits
        u = np.clip(u, -self.torque_limit, self.torque_limit)

        # Apply torques
        self.data.qfrc_applied[:] = 0.0
        self.data.qfrc_applied[self.robot_dof_indices] = u

        mujoco.mj_step(self.model, self.data)
        

# --- Main Simulation Setup ---
XML_PATH = "robot-cleaning-scene/scene/robot_cleaning_scene.xml"
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)
model.opt.timestep = 0.002
model.opt.iterations = 50
model.opt.tolerance = 1e-10

ee_site_name = "ee_site"
robot_joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
joint_positions = [0.188, -2.2, -0.87, 0.0, np.pi/2, np.pi/2]
joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in robot_joint_names]
for jid, pos in zip(joint_ids, joint_positions):
    qpos_adr = model.jnt_qposadr[jid]
    data.qpos[qpos_adr] = pos

mujoco.mj_forward(model, data)

# Disable all actuators so qfrc_applied is the only actuation
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
mujoco.mj_forward(model, data)

# --- Instantiate Controller ---
controller = ImpedanceController(model, data, ee_site_name, robot_joint_names)
controller.kp = 200
controller.kv = 50
# --- Target Pose ---
target_xyz = np.array([0.5, 0.2, 0.51], dtype=float)
target_rpy = np.array([1.57079633, -0.07159265, -1.38279633])
target_quat = controller.euler_rpy_to_quat(*target_rpy)

# --- Control Loop ---
with mujoco.viewer.launch_passive(model, data) as viewer:
    time.sleep(2.0)
    while viewer.is_running():
        controller.step(target_xyz, target_quat)
        viewer.sync()

#%%
import random
import mujoco
import mujoco.viewer
import numpy as np
import threading 

model = mujoco.MjModel.from_xml_path("ballmove.xml")
data = mujoco.MjData(model)

for i in range(model.nsite):
    print(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i))


# Get end-effector site ID by name
ee_site_name = "ee_site"  
ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site_name)

robot_joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in robot_joint_names]
# Starting joint positions (radians) for the UR5e robot
joint_positions = [0.188, -2.15, -0.87, 0.0, np.pi/2, np.pi/2]

# Set the joint positions in the simulation data
for jid, pos in zip(joint_ids, joint_positions):
    qpos_adr = model.jnt_qposadr[jid]  # Address in qpos array
    data.qpos[qpos_adr] = pos

mujoco.mj_forward(model, data)

# Get the DOF indices for these joints
robot_dof_indices = []
for joint_id in joint_ids:
    dof_adr = model.jnt_dofadr[joint_id]
    robot_dof_indices.append(dof_adr)

# Allocate arrays for the Jacobian (position and rotation)
jacp = np.zeros((3, model.nv))  # Position Jacobian
jacr = np.zeros((3, model.nv))  # Rotation Jacobian

# Compute the Jacobian at the end-effector site
mujoco.mj_jacSite(model, data, jacp, jacr, ee_site_id)

# Stack to get the full 6D Jacobian (position + rotation)
J = np.vstack((jacp, jacr))  # Shape: (6, nv)

J_robot = J[:, robot_dof_indices]

print("Jacobian shape:", J_robot.shape)
print("Jacobian (robot joints):\n", J_robot)

# Allocate and compute the full joint-space inertia (mass) matrix
M_full = np.zeros((model.nv, model.nv))
mujoco.mj_fullM(model, M_full, data.qM)

# Extract the submatrix for the robot joints only
robot_dof_indices = np.array(robot_dof_indices)  # ensure it's a numpy array for indexing
M_robot = M_full[np.ix_(robot_dof_indices, robot_dof_indices)]

print("Robot mass matrix shape:", M_robot.shape)
print("Robot mass matrix:\n", M_robot)
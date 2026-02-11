import mujoco
import mujoco.viewer
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
import time

# Load model and data
model = mujoco.MjModel.from_xml_path("ballmove.xml")
data = mujoco.MjData(model)

# Get site ID
site_id = model.site('ee_site').id

# Target position for ee_site
target_pos = np.array([0.6, 0, 0.48])

# -----------------------------
# Inverse Kinematics to reach target
# -----------------------------
def solve_ik(target_pos, alpha=0.5, tol=1e-4, max_iter=1000):
    """
    Move end-effector to target position using Jacobian-based IK
    
    Args:
        target_pos: [x, y, z] target position
        alpha: step size for gradient descent
        tol: convergence tolerance
        max_iter: maximum iterations
    """
    for iteration in range(max_iter):
        # Forward kinematics
        mujoco.mj_forward(model, data)
        
        # Get current end-effector position
        current_pos = data.site_xpos[site_id].copy()
        
        # Calculate error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)
        
        if error_norm < tol:
            print(f"IK converged in {iteration} iterations. Error: {error_norm:.6f}")
            return True
        
        # Compute Jacobian
        jacp = np.zeros((3, model.nv))  # positional Jacobian
        jacr = np.zeros((3, model.nv))  # rotational Jacobian
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        
        # Compute joint velocities using damped least squares
        lambda_damping = 0.01
        J_T = jacp.T
        dq = J_T @ np.linalg.inv(jacp @ J_T + lambda_damping * np.eye(3)) @ error
        
        # Update joint positions (only robot joints)
        data.qpos[:6] += alpha * dq[:6]
        
        # Clamp to joint limits if available
        for i in range(min(6, model.njnt)):
            if model.jnt_limited[i]:
                data.qpos[i] = np.clip(data.qpos[i], 
                                       model.jnt_range[i, 0],
                                       model.jnt_range[i, 1])
    
    print(f"IK did not fully converge. Final error: {np.linalg.norm(error):.6f}")
    return False

# Solve IK to reach target position
print(f"Moving ee_site to target position: {target_pos}")
solve_ik(target_pos, alpha=0.3, max_iter=2000)

# Store the solved joint configuration
solved_qpos = [1.63, -1.51, -1.9, -0.88, 1.76, 0.0]

# Verify final position
mujoco.mj_forward(model, data)
final_pos = data.site_xpos[site_id].copy()
print(f"Final ee_site position: {final_pos}")
print(f"Target position: {target_pos}")
print(f"Position error: {np.linalg.norm(final_pos - target_pos):.6f}")

# -----------------------------
# Load and interpolate trajectory
# -----------------------------
df_traj = pd.read_csv("dmp_trajectory_feedback.csv")
iteration = 10
df_traj_iter = df_traj[df_traj['iter'] == iteration].copy()

# Original trajectory points
x_orig = df_traj_iter['x'].values
y_orig = df_traj_iter['y'].values

# Create parameter t for original points
t_orig = np.linspace(0, 1, len(x_orig))
# Create parameter t for interpolated points
t_interp = np.linspace(0, 1, len(x_orig))

# Interpolate x and y
interp_x = interp1d(t_orig, x_orig, kind='cubic')
interp_y = interp1d(t_orig, y_orig, kind='cubic')

x = interp_x(t_interp)
y = interp_y(t_interp)
z = np.ones_like(x) * 0.48

trajectory = np.vstack([x, y, z]).T

print(f"Interpolated trajectory from {len(x_orig)} to {len(x)} points")

# -----------------------------
# Launch passive viewer
# -----------------------------
def render_callback(viewer, scene, context):
    """Callback to add custom visualization to the scene"""
    # Add trajectory points
    for point in trajectory[::50]:  # Sample every 50th point for performance
        if scene.ngeom >= scene.maxgeom:
            break
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=[0.01, 0, 0],
                            pos=point,
                            mat=np.eye(3).flatten(),
                            rgba=[1, 0.549, 0, 0.5])
        scene.ngeom += 1
    
    # Add target position marker
    if scene.ngeom < scene.maxgeom:
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=[0.03, 0, 0],
                            pos=target_pos,
                            mat=np.eye(3).flatten(),
                            rgba=[1, 0, 0, 0.7])
        scene.ngeom += 1

# Set initial joint configuration
data.qpos[:6] = solved_qpos
data.ctrl[:6] = solved_qpos

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Optional: set camera position
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -70
    viewer.cam.distance = 2
    viewer.cam.lookat[:] = [0.6, 0, 0.5]
    
    # Run simulation loop
    while viewer.is_running():
        # Set control to maintain position
        data.ctrl[:6] = solved_qpos
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Manually add trajectory visualization to the scene
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
            
            # Add trajectory points
            for i in range(len(trajectory) - 1):
                if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                    break
                
                p1 = trajectory[i]
                p2 = trajectory[i + 1]
                midpoint = (p1 + p2) / 2
                
                direction = p2 - p1
                length = np.linalg.norm(direction)
                
                if length > 0:
                    direction = direction / length
                    
                    # Rotation matrix alignment
                    z_axis = direction
                    x_axis = np.array([1, 0, 0])
                    if abs(np.dot(z_axis, x_axis)) > 0.99:
                        x_axis = np.array([0, 1, 0])
                    y_axis = np.cross(z_axis, x_axis)
                    y_axis = y_axis / np.linalg.norm(y_axis)
                    x_axis = np.cross(y_axis, z_axis)
                    
                    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
                    
                    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[0.006, length/2, 0],  # radius, half-length
                        pos=midpoint,
                        mat=rotation_matrix.flatten(),
                        rgba=[1, 0.549, 0, 0.9]
                    )
                    viewer.user_scn.ngeom += 1
        
        # Sync viewer
        viewer.sync()
        
        # Control frame rate
        time.sleep(0.01)

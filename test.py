# ur5e_enhanced_animation.py  
# Enhanced UR5e IK REPL with custom start position and improved movement
import time
import numpy as np
import mujoco

# ---------------------------------------------------------
# Viewer adapter: tries mujoco.viewer first, falls back to mujoco-python-viewer
# ---------------------------------------------------------
class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo"):
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

# ---------------- Configuration ----------------
XML_PATH = "ballmove.xml"
SITE_NAME = "ee_site"
UR5E_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

# 🎯 YOUR CUSTOM START POSITION (in radians)
START_JOINT_POSITIONS = np.array([
    -2.89,    # shoulder_pan
    -1.07,    # shoulder_lift  
    0.377,    # elbow
    -0.314,   # wrist_1
    -0.0628,  # wrist_2
    -0.503    # wrist_3
])

# IK and Animation Parameters
INIT_LAMBDA = 0.15
TOL = 1e-3
PRINT_EVERY = 60
ANIMATION_DURATION = 4.0  # Slightly longer for smoother movement
ANIMATION_FPS = 75        # Higher FPS for very smooth motion

# -------------------------------------------

def _joint_cols(model, joint_names):
    cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid == -1:
            raise RuntimeError(f"Joint '{jn}' not found in model.")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Joint '{jn}' must be a hinge.")
        cols.append(model.jnt_dofadr[jid])
    return np.asarray(cols, dtype=int)

def _clamp_limits(model, qpos, joint_names):
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
    for i, jn in enumerate(joint_names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        positions[i] = data.qpos[qadr]
    return positions

def initialize_robot_position(model, data, joint_positions, joint_names):
    """Initialize robot to specific joint configuration"""
    print("🤖 Setting robot to initial position...")
    set_joint_positions(model, data, joint_names, joint_positions)
    _clamp_limits(model, data.qpos, joint_names)
    mujoco.mj_forward(model, data)
    
    # Print initial position
    print("📍 Initial joint configuration:")
    for i, jn in enumerate(joint_names):
        rad = joint_positions[i]
        deg = np.degrees(rad)
        print(f"  {jn:12s}: {rad:+.3f} rad ({deg:+7.2f}°)")

def _interpolate_path(p0, p1, max_step=0.05):
    """Linear path from p0 to p1 with <= max_step meters between waypoints."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    dist = np.linalg.norm(p1 - p0)
    if dist <= max_step:
        return [p1]
    n = int(np.ceil(dist / max_step))
    alphas = np.linspace(0.0, 1.0, n + 1)[1:]  # skip p0
    return [p0*(1-a) + p1*a for a in alphas]

def move_tip_to(model, data, site_id, goal_pos_world,
                joint_names=("shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"),
                step_clip=0.2,          # Slightly smaller steps for smoother convergence
                max_wp_step=0.03,       # Smaller waypoint steps
                max_iters_per_wp=300,   # More iterations for better accuracy
                lam_init=0.1, lam_inc=2.0, lam_dec=0.85,
                tol=1e-3, print_every=60):
    """
    Enhanced IK solver with improved convergence
    """
    # compute DOF columns for the six hinge joints
    dof_cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid == -1: raise RuntimeError(f"Joint '{jn}' not found.")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Joint '{jn}' must be a hinge.")
        dof_cols.append(model.jnt_dofadr[jid])
    dof_cols = np.asarray(dof_cols, int)
    
    def clamp_limits():
        _clamp_limits(model, data.qpos, joint_names)
    
    mujoco.mj_forward(model, data)
    start = np.array(data.site_xpos[site_id])
    waypoints = _interpolate_path(start, goal_pos_world, max_step=max_wp_step)
    
    print(f"🎯 Planning path with {len(waypoints)} waypoints")
    
    for wpi, wp in enumerate(waypoints, 1):
        lam = lam_init
        mujoco.mj_forward(model, data)
        prev_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))
        stalled = 0
        
        for it in range(1, max_iters_per_wp + 1):
            mujoco.mj_forward(model, data)
            # world-frame site Jacobian
            Jp = np.zeros((3, model.nv)); Jr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Jp, Jr, site_id)
            J = Jp[:, dof_cols]                                  # (3,6)
            e = wp - np.array(data.site_xpos[site_id])           # (3,)
            # LM step: dq = (JᵀJ + λI)⁻¹ Jᵀ e
            A = J.T @ J + lam * np.eye(J.shape[1])
            b = J.T @ e
            try:
                dq = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(A) @ b
            # trust-region: clip ‖Δq‖
            nq = np.linalg.norm(dq)
            if nq > step_clip:
                dq *= (step_clip / (nq + 1e-12))
            
            # tentative update with rollback
            qpos_before = data.qpos.copy()
            for k, jn in enumerate(joint_names):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] += dq[k]
            clamp_limits()
            mujoco.mj_forward(model, data)
            new_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))
            
            if new_err < prev_err - 1e-6:
                prev_err = new_err
                lam = max(1e-6, lam * lam_dec)
                stalled = 0
            else:
                # rollback, increase damping
                data.qpos[:] = qpos_before
                mujoco.mj_forward(model, data)
                lam *= lam_inc
                stalled += 1
            
            if it % print_every == 0:
                print(f"  [wp {wpi:02d}/{len(waypoints)} | it {it:03d}] "
                      f"|e|={prev_err:.6f} m, λ={lam:.3g}")
            if prev_err < tol:
                break
            if stalled >= 30:  # More patience for convergence
                break
        
        # if this waypoint failed, bail out
        if prev_err >= tol:
            print(f"⚠️  Waypoint {wpi} failed with error {prev_err:.6f} m")
            return False, prev_err
    
    # success on final goal
    final_err = np.linalg.norm(goal_pos_world - np.array(data.site_xpos[site_id]))
    return True, final_err

def enhanced_interpolate(start_pos, end_pos, t):
    """
    Enhanced smooth interpolation with multiple curve options
    """
    # Even smoother interpolation: smootherstep (6t⁵ - 15t⁴ + 10t³)
    smooth_t = 6 * t**5 - 15 * t**4 + 10 * t**3
    return start_pos + smooth_t * (end_pos - start_pos)

def animate_robot_movement(model, data, viewer, joint_names, start_joints, target_joints, 
                          duration=ANIMATION_DURATION, fps=ANIMATION_FPS):
    """
    Enhanced smooth animation with better interpolation
    """
    print(f"🎬 Animating robot movement over {duration:.1f} seconds at {fps} FPS...")
    
    total_frames = int(duration * fps)
    dt = 1.0 / fps
    
    # Show movement statistics
    joint_deltas = target_joints - start_joints
    max_joint_movement = np.max(np.abs(joint_deltas))
    print(f"   Maximum joint movement: {np.degrees(max_joint_movement):.1f}°")
    
    for frame in range(total_frames + 1):
        t = frame / total_frames
        
        # Enhanced smooth interpolation
        current_joints = enhanced_interpolate(start_joints, target_joints, t)
        
        # Update robot position
        set_joint_positions(model, data, joint_names, current_joints)
        _clamp_limits(model, data.qpos, joint_names)
        
        # Update physics and render
        mujoco.mj_forward(model, data)
        viewer.draw()
        
        # Control frame rate
        time.sleep(dt)
        
        # Progress indicator (every 25%)
        if frame % (total_frames // 4) == 0 and frame > 0:
            progress = (frame / total_frames) * 100
            print(f"   {progress:.0f}% complete...")
        
        # Check if viewer is still open
        if not viewer.is_running():
            break
    
    print("✅ Animation complete!")

def return_to_start_position(model, data, viewer, joint_names, start_joints, duration=2.0):
    """Return robot to starting position"""
    print("🏠 Returning to start position...")
    current_joints = get_joint_positions(model, data, joint_names)
    animate_robot_movement(model, data, viewer, joint_names, 
                         current_joints, start_joints, duration=duration, fps=60)

def _print_joint_solution(model, data):
    print("🔧 Current joint configuration:")
    for jn in UR5E_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        q = float(data.qpos[qadr])
        print(f"  {jn:12s}: {q:+.6f} rad ({np.degrees(q):+7.2f}°)")

def _hold_with_actuators_if_present(model, data):
    # If you have '<joint>_act' position actuators, set ctrl=qpos to "hold" pose.
    act_ids = []
    for jn in UR5E_JOINTS:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + "_act")
        if aid == -1:
            return
        act_ids.append(aid)
    
    for k, jn in enumerate(UR5E_JOINTS):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        data.ctrl[act_ids[k]] = float(data.qpos[qadr])

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    if site_id == -1:
        raise RuntimeError(f"Site '{SITE_NAME}' not found; update SITE_NAME to your tip site.")
    
    # 🚀 Initialize robot to your custom start position
    initialize_robot_position(model, data, START_JOINT_POSITIONS, UR5E_JOINTS)
    
    # Show initial end-effector position
    mujoco.mj_forward(model, data)
    start_pos = np.array(data.site_xpos[site_id])
    print(f"📍 Initial end-effector position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
    
    vw = ViewerAdapter(model, data, title="UR5e Enhanced Animation")
    
    print("\n🤖 UR5e ENHANCED IK REPL with SMOOTH ANIMATION 🤖")
    print("=" * 60)
    print("✨ Features:")
    print("  • Custom start position loaded")
    print("  • Ultra-smooth 75 FPS animation")
    print("  • Enhanced IK convergence")
    print("  • Progress indicators")
    print("")
    print("🎯 Suggested targets for mop-to-table movement:")
    print("  Table center:  0.3 0.0 0.15")
    print("  Table edge:    0.2 0.0 0.15") 
    print("  Above table:   0.3 0.0 0.25")
    print("")
    print("💡 Commands:")
    print("  • Enter xyz coordinates (e.g., '0.3 0.0 0.15')")
    print("  • Type 'home' to return to start position")
    print("  • Type 'q' to quit")
    print("")
    
    while True:
        try:
            line = input("xyz> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        
        if not line or line.lower() == "q":
            break
            
        if line.lower() == "home":
            return_to_start_position(model, data, vw, UR5E_JOINTS, START_JOINT_POSITIONS)
            continue
        
        parts = line.replace(",", " ").split()
        if len(parts) != 3:
            print("Please enter exactly 3 numbers: x y z (or 'home' or 'q')")
            continue
        
        try:
            goal = np.array(list(map(float, parts)), dtype=float)
        except ValueError:
            print("Could not parse numbers.")
            continue
        
        # Get starting position and joint configuration
        start = np.array(data.site_xpos[site_id])
        start_joints = get_joint_positions(model, data, UR5E_JOINTS)
        
        print(f"🎯 Start: [{start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}]  ->  Goal: [{goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}]")
        
        # Calculate distance
        distance = np.linalg.norm(goal - start)
        print(f"📏 Distance: {distance:.3f} m")
        
        # Solve IK to get target joint configuration
        print("🧠 Solving inverse kinematics...")
        ok, err = move_tip_to(model, data, site_id, goal,
                              step_clip=0.2,
                              max_wp_step=0.03,
                              max_iters_per_wp=300,
                              lam_init=INIT_LAMBDA,
                              tol=TOL,
                              print_every=PRINT_EVERY)
        
        if ok:
            print(f"✅ IK Success! Position error: {err:.6f} m")
            
            # Get target joint configuration
            target_joints = get_joint_positions(model, data, UR5E_JOINTS)
            
            # Reset to start position for animation
            set_joint_positions(model, data, UR5E_JOINTS, start_joints)
            mujoco.mj_forward(model, data)
            
            # Animate smooth movement
            animate_robot_movement(model, data, vw, UR5E_JOINTS, 
                                 start_joints, target_joints)
            
            # Print final solution
            _print_joint_solution(model, data)
            _hold_with_actuators_if_present(model, data)
            
        else:
            print(f"❌ IK Failed! Final error: {err:.6f} m")
            print("💡 Try a position closer to the robot or within workspace")
            # Reset to start position
            set_joint_positions(model, data, UR5E_JOINTS, start_joints)
            mujoco.mj_forward(model, data)
        
        # Brief pause after animation
        print("\n⏳ Ready for next target...\n")
        time.sleep(0.5)
        vw.draw()
    
    vw.close()

if __name__ == "__main__":
    main()


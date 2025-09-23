import random
import mujoco
import mujoco.viewer
import numpy as np
import threading  # NEW

def is_in_exclusion(x, y, x_center, y_center, excl_width, excl_length):
    """
    Check if (x, y) is inside the exclusion rectangle centered at (x_center, y_center).
    The rectangle has width excl_width (y direction) and length excl_length (x direction).
    Returns True if inside, False otherwise.
    """
    half_w = excl_width / 2
    half_l = excl_length / 2
    return (x_center - half_l <= x <= x_center + half_l) and (y_center - half_w <= y <= y_center + half_w)

def merge_xml(main_xml_path, balls_xml_path, output_xml_path):
    """
    Merge the balls.xml file into the main MuJoCo XML file.
    Inserts the contents of balls.xml just before </worldbody> in main_xml_path.
    Writes the result to output_xml_path.
    """
    with open(main_xml_path, "r") as f:
        main_xml = f.read()
    with open(balls_xml_path, "r") as f:
        balls_xml = f.read()

    # Find the <worldbody> section in the main XML
    start = main_xml.find("<worldbody>")
    end = main_xml.find("</worldbody>")
    if start == -1 or end == -1:
        raise ValueError("No <worldbody> section found in main XML.")

    # Insert balls.xml contents before </worldbody>
    merged = main_xml[:end] + balls_xml + main_xml[end:]
    with open(output_xml_path, "w") as f:
        f.write(merged)

def generate_balls_xml(num_balls, radii, positions, inertias, output_path="balls.xml"):
    """
    Generate balls.xml with specified parameters for each ball.

    Args:
        num_balls (int): Number of balls.
        radii (list of float): Radius for each ball.
        positions (list of tuple): Position (x, y, z) for each ball.
        inertias (list of tuple): Diagonal inertia (ix, iy, iz) for each ball.
        output_path (str): Output XML file path.
    """
    assert len(radii) == num_balls
    assert len(positions) == num_balls
    assert len(inertias) == num_balls

    xml = ""
    for i in range(num_balls):
        xml += (
            f'   <body name="ball_{i+1}" pos="{positions[i][0]} {positions[i][1]} {positions[i][2]}">\n'
            f'      <freejoint/>\n'
            f'      <inertial mass="0.03" diaginertia="{inertias[i][0]} {inertias[i][1]} {inertias[i][2]}" pos="0 0 0"/>\n'
            f'      <geom type="sphere" size="{radii[i]}" material="dust_material"/>\n'
            f'   </body>\n'
        )

    with open(output_path, "w") as f:
        f.write(xml)

# NEW: build an axis-aligned zig-zag waypoint list over the rectangle (x_min..x_max, y_min..y_max) at fixed z
def build_zigzag_path(p0, p1, y_step=0.05):
    """
    Generate waypoints that sweep along x back-and-forth, stepping along y (zig-zag).
    p0, p1: 3D numpy arrays defining opposite rectangle corners (must share same z).
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    assert abs(p0[2] - p1[2]) < 1e-6, "p0 and p1 must have same z"
    z = p0[2]
    x_min, x_max = sorted((p0[0], p1[0]))
    y_min, y_max = sorted((p0[1], p1[1]))

    waypoints = []
    y = y_min
    forward = True
    # ensure last strip reaches y_max
    while y <= y_max + 1e-9:
        if forward:
            waypoints.append(np.array([x_min, y, z]))
            waypoints.append(np.array([x_max, y, z]))
        else:
            waypoints.append(np.array([x_max, y, z]))
            waypoints.append(np.array([x_min, y, z]))
        forward = not forward
        y += y_step

    # Optional: start exactly at lower-left corner if present
    if len(waypoints) and not np.allclose(waypoints[0], [x_min, y_min, z]):
        waypoints.insert(0, np.array([x_min, y_min, z]))
    return waypoints

if __name__ == "__main__":
    # Simulation parameters
    num_balls = 100  # Number of balls to generate
    radius = 0.02  # Ball radius (meters)
    mass = 0.53    # Ball mass (kg)
    inertia = 2/5 * mass * radius**2  # Sphere inertia formula
    # inertia = 3e-3  # Approximate inertia value for small spheres
    radii = [radius] * num_balls
    inertias = [(inertia, inertia, inertia)] * num_balls

    # Position limits for random generation
    x_pos_low = -1.3
    x_pos_high = 1.3
    y_pos_low = -0.7
    y_pos_high = 0.7
    z_pos_low = 0.51
    z_pos_high = 0.51  # Fixed z position

    # Center of exclusion rectangle
    x_pos_center = (x_pos_low + x_pos_high) / 2
    y_pos_center = (y_pos_low + y_pos_high) / 2

    # Exclusion rectangle dimensions (centered in x/y range)
    swiffer_head_length = 0.3  # x direction
    swiffer_head_width = 0.15    # y direction

    # Generate random positions, excluding the rectangle
    positions = []
    while len(positions) < num_balls:
        x = round(random.uniform(x_pos_low, x_pos_high), 4)
        y = round(random.uniform(y_pos_low, y_pos_high), 4)
        z = round(random.uniform(z_pos_low, z_pos_high), 4)
        # Only accept positions outside the exclusion zone
        if not is_in_exclusion(x, y, x_pos_center, y_pos_center, swiffer_head_width, swiffer_head_length):
            positions.append((x, y, z))

    # Generate balls.xml with the random positions
    generate_balls_xml(num_balls, radii, positions, inertias, "balls.xml")

    # Merge balls.xml into main MuJoCo XML and write ballmove.xml
    main_xml = "world_ur5e_table.xml"
    balls_xml = "balls.xml"
    output_xml = "ballmove.xml"
    merge_xml(main_xml, balls_xml, output_xml)

    model = mujoco.MjModel.from_xml_path("ballmove.xml")
    data = mujoco.MjData(model)

    # Desired joint angles (radians) in UR5e joint order
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
    desired_pose = [0.188, -2.2, -0.87, 0.0, np.pi/2, np.pi/2]
    # from -1.45 to 1.76

    # Set qpos using qpos addresses (not joint IDs)
    for jname, q in zip(joint_names, desired_pose):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)  # joint id
        qadr = model.jnt_qposadr[jid]                                      # start index in qpos
        data.qpos[qadr] = q

    # Match actuator targets to the same angles so the controller holds the pose
    actuator_names = [
        "shoulder_pan_act", "shoulder_lift_act", "elbow_act",
        "wrist_1_act", "wrist_2_act", "wrist_3_act"
    ]
    for aname, q in zip(actuator_names, desired_pose):
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
        data.ctrl[aid] = q

    # Zero velocities and propagate state
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    # --- REMOVE shoulder_pan sinusoid; replace with Cartesian zig-zag IK control ---

    # Site to control
    ee_site = "ee_site"
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, ee_site)

    # Select only the 6 arm DOFs for IK (ignore unactuated mop ball joint DOFs)
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn) for jn in joint_names]
    dof_idx = np.array([model.jnt_dofadr[jid] for jid in jids], dtype=int)  # 6 indices in v/nv
    qpos_idx = np.array([model.jnt_qposadr[jid] for jid in jids], dtype=int)  # 6 indices in qpos

    # Actuators mapping in the same order
    actuator_names = ["shoulder_pan_act", "shoulder_lift_act", "elbow_act", "wrist_1_act", "wrist_2_act", "wrist_3_act"]
    aids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, an) for an in actuator_names]

    # Commanded joint positions (start from current)
    q_cmd = data.qpos[qpos_idx].copy()

    # Build zig-zag path between the two corners at fixed z
    p_start = np.array([-1.2, -0.6, 0.52])
    p_end   = np.array([-0.1, -0.1, 0.52])
    waypoints = build_zigzag_path(p_start, p_end, y_step=0.05)

    # Path traversal state (bounce back and forth for cyclical motion)
    wp_i = 0
    forward = True

    # IK parameters
    dt = model.opt.timestep
    pos_k = 1.5  # Cartesian proportional gain (m/s per m error)
    max_dq = 0.05  # rad per step clamp (safety)
    pos_tol = 0.005  # 5 mm waypoint tolerance

    # Pre-alloc Jacobians
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    # ---- Command listener (runs in background) ----
    start_event = threading.Event()
    quit_event = threading.Event()

    def command_loop():
        print("Commands: start | pause | resume | quit")
        while not quit_event.is_set():
            try:
                cmd = input("> ").strip().lower()
            except EOFError:
                break
            if cmd in ("start", "resume"):
                start_event.set()
                print("Motion: RUNNING")
            elif cmd == "pause":
                start_event.clear()
                print("Motion: PAUSED")
            elif cmd == "quit":
                quit_event.set()
                print("Quitting...")
            elif cmd:
                print("Unknown command. Use: start | pause | resume | quit")

    cmd_thread = threading.Thread(target=command_loop, daemon=True)
    cmd_thread.start()

    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and not quit_event.is_set():
            # Current EE position
            x = data.site_xpos[sid].copy()

            if start_event.is_set() and len(waypoints) > 0:
                # Current target waypoint
                xt = waypoints[wp_i]
                e = xt - x  # position error in world frame

                # Advance waypoint if close enough
                if np.linalg.norm(e) < pos_tol:
                    if forward:
                        if wp_i < len(waypoints) - 1:
                            wp_i += 1
                        else:
                            forward = False  # bounce
                            wp_i -= 1
                    else:
                        if wp_i > 0:
                            wp_i -= 1
                        else:
                            forward = True   # bounce
                            wp_i += 1
                    # Recompute target and error after switching index
                    xt = waypoints[wp_i]
                    e = xt - x

                # Compute position Jacobian at ee_site
                mujoco.mj_jacSite(model, data, jacp, jacr, sid)
                J = jacp[:, dof_idx]  # 3x6 for arm joints

                # Resolved-rate IK: qdot = J^+ * xdot, xdot = pos_k * e
                xdot = pos_k * e
                J_pinv = np.linalg.pinv(J, rcond=1e-4)
                dq = (J_pinv @ xdot) * dt

                # Clamp joint increments
                dq = np.clip(dq, -max_dq, max_dq)

                # Integrate command and send to position actuators
                q_cmd = q_cmd + dq
                for i, aid in enumerate(aids):
                    data.ctrl[aid] = q_cmd[i]
            else:
                # Hold current command
                for i, aid in enumerate(aids):
                    data.ctrl[aid] = q_cmd[i]

            mujoco.mj_step(model, data)
            viewer.sync()

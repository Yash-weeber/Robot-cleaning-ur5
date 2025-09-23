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
    x_pos_low = -2.3
    x_pos_high = 0.2
    y_pos_low = -1.35
    y_pos_high = -0.05
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

    # Cyclical motion setup for shoulder_pan
    shoulder_min = -1.5
    shoulder_max = 1.76
    shoulder_mid = 0.5 * (shoulder_min + shoulder_max)
    shoulder_amp = 0.5 * (shoulder_max - shoulder_min)
    shoulder_freq_hz = 0.3  # oscillation frequency (Hz)
    shoulder_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_pan_act")

    # Initialize shoulder target at mid to avoid a jump
    data.ctrl[shoulder_act] = shoulder_mid

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
            # Time (s) advances with mj_step
            t = data.time

            if start_event.is_set():
                q_target = shoulder_mid + shoulder_amp * np.sin(2 * np.pi * shoulder_freq_hz * t)
            else:
                q_target = shoulder_mid  # hold mid while paused/not started

            data.ctrl[shoulder_act] = q_target
            mujoco.mj_step(model, data)
            viewer.sync()

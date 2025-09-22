import random
import mujoco
import mujoco.viewer

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
            f'<body name="ball_{i+1}" pos="{positions[i][0]} {positions[i][1]} {positions[i][2]}">\n'
            f'  <freejoint/>\n'
            f'  <inertial mass="0.03" diaginertia="{inertias[i][0]} {inertias[i][1]} {inertias[i][2]}" pos="0 0 0"/>\n'
            f'  <geom type="sphere" size="{radii[i]}" rgba="1 0 0 1"/>\n'
            f'</body>\n'
        )

    with open(output_path, "w") as f:
        f.write(xml)

if __name__ == "__main__":
    # Simulation parameters
    num_balls = 100  # Number of balls to generate
    radius = 0.02  # Ball radius (meters)
    mass = 0.53    # Ball mass (kg)
    # inertia = 2/5 * mass * radius**2  # Sphere inertia formula
    inertia = 3e-3  # Approximate inertia value for small spheres
    radii = [radius] * num_balls
    inertias = [(inertia, inertia, inertia)] * num_balls

    # Position limits for random generation
    x_pos_low = -1.95
    x_pos_high = -0.1
    y_pos_low = -1.3
    y_pos_high = -0.2
    z_pos_low = 0.5
    z_pos_high = 0.5  # Fixed z position

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

    joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
    joint_indices = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]

    desired_pose = [0.377, -1.88, -1.04, -4.84, 0.0, 0.0]
    for idx, val in zip(joint_indices, desired_pose):
        data.qpos[idx] = val
    mujoco.mj_forward(model, data)

    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


import mujoco
import mujoco.viewer
import numpy as np
import time
import os

def run_table_dust_simulation():
    """Run interactive MuJoCo simulation with dust particles on the table"""

    xml_file = 'ur5e_with_mop_and_dust_on_table.xml'

    # Check if XML file exists
    if not os.path.exists(xml_file):
        print(f"❌ Error: {xml_file} not found!")
        print("Make sure the XML file is in the same directory as this script.")
        return

    # Check if STL file exists
    stl_file = os.path.join('assets', 'simple_small_ball.stl')
    if not os.path.exists(stl_file):
        print(f"❌ Error: {stl_file} not found!")
        print("Make sure 'simple_small_ball.stl' is in the 'assets' folder.")
        return

    # Load the model
    try:
        model = mujoco.MjModel.from_xml_path(xml_file)
        data = mujoco.MjData(model)
        print("✅ Model loaded successfully!")
        print(f"📊 Number of bodies: {model.nbody}")
        print(f"🦾 Number of joints: {model.njnt}")
        print(f"🔘 Number of geoms: {model.ngeom}")

        # Count dust particles
        dust_count = sum(1 for i in range(model.nbody)
                        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) and
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i).startswith('dust_particle_'))
        print(f"✨ Number of dust particles on table: {dust_count}")

        # Initialize robot to a cleaning position
        if model.njnt >= 6:
            data.qpos[0] = 1.2       # shoulder_pan - positioned over table
            data.qpos[1] = -0.8      # shoulder_lift - reaching down
            data.qpos[2] = 1.2       # elbow - bent to reach table
            data.qpos[3] = -1.8      # wrist_1 - angled down
            data.qpos[4] = -1.57     # wrist_2 - oriented
            data.qpos[5] = 0.0       # wrist_3 - straight

            # Forward kinematics to update positions
            mujoco.mj_forward(model, data)

    except Exception as e:
        print(f"❌ Error loading model: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure all mesh files (.obj, .stl) are in the 'assets' folder")
        print("2. Check that 'simple_small_ball.stl' exists")
        print("3. Verify all UR5 robot mesh files are present")
        return

    print("\n🎮 Starting table cleaning simulation...")
    print("Controls:")
    print("- Left click and drag: rotate view")
    print("- Right click and drag: translate view")
    print("- Scroll: zoom in/out")
    print("- Press TAB: enable/disable joint control")
    print("- Close window: exit simulation")
    print("\n🧹 The robot will clean dust particles on the table surface!")

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Enable visualization
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        # Set camera to focus on the table
        viewer.cam.lookat[0] = 0.3    # Table center X
        viewer.cam.lookat[1] = 0.0    # Table center Y
        viewer.cam.lookat[2] = 0.1    # Slightly above table
        viewer.cam.distance = 1.5     # Good viewing distance
        viewer.cam.elevation = -30    # Look down at table

        start_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Step physics
            mujoco.mj_step(model, data)

            # Table cleaning motion - sweep across the table surface
            current_time = data.time
            if model.njnt >= 6:
                # Sweeping motion across table
                sweep_amplitude = 0.4  # How far to sweep
                sweep_speed = 0.3      # Speed of sweeping

                # Base sweeping motion
                data.ctrl[0] = 1.2 + sweep_amplitude * np.sin(current_time * sweep_speed)
                data.ctrl[1] = -0.8 + 0.1 * np.cos(current_time * sweep_speed * 0.7)
                data.ctrl[2] = 1.2 + 0.1 * np.sin(current_time * sweep_speed * 0.5)
                data.ctrl[3] = -1.8 + 0.2 * np.cos(current_time * sweep_speed * 0.3)
                data.ctrl[4] = -1.57
                data.ctrl[5] = 0.2 * np.sin(current_time * sweep_speed * 2.0)  # Wrist rotation

            # Sync viewer
            viewer.sync()

            # Print cleaning progress every 5 seconds
            if int(current_time) % 5 == 0 and int(current_time) > 0:
                if int(current_time) != getattr(run_table_dust_simulation, 'last_print_time', -1):
                    print(f"🧹 Cleaning progress: {int(current_time)}s - Keep watching the dust particles!")
                    run_table_dust_simulation.last_print_time = int(current_time)

            # Maintain real-time execution
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("\n👋 Table cleaning simulation ended. Great job!")

if __name__ == "__main__":
    print("🤖 UR5 Robot Table Cleaning Simulation")
    print("🧹 Dust Particles on Table Surface")
    print("=" * 50)
    run_table_dust_simulation()

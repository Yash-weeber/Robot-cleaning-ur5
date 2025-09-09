# #
# # import mujoco
# # import mujoco.viewer
# # import numpy as np
# # import time
# #
# # def run_interactive_simulation():
# #     """Run interactive MuJoCo simulation with viewer"""
# #
# #     # Load the model
# #     try:
# #         model = mujoco.MjModel.from_xml_path('ur5e_with_mop_and_dust.xml')
# #         data = mujoco.MjData(model)
# #         print("Model loaded successfully!")
# #         print(f"Number of bodies: {model.nbody}")
# #         print(f"Number of dust particles: {model.nbody - 8}")  # Subtract robot + table bodies
# #
# #         # Initialize robot to a reasonable pose
# #         # Set joint positions (you may need to adjust these based on your robot)
# #         if model.njnt >= 6:  # Make sure we have enough joints
# #             # Set initial joint angles for better viewing
# #             data.qpos[0] = 0.0      # shoulder_pan
# #             data.qpos[1] = -1.57    # shoulder_lift
# #             data.qpos[2] = 1.57     # elbow
# #             data.qpos[3] = -1.57    # wrist_1
# #             data.qpos[4] = 0.0      # wrist_2
# #             data.qpos[5] = 0.0      # wrist_3
# #
# #             # Forward kinematics to update positions
# #             mujoco.mj_forward(model, data)
# #
# #     except Exception as e:
# #         print(f"Error loading model: {e}")
# #         return
# #
# #     print("\nStarting interactive simulation...")
# #     print("Controls:")
# #     print("- Left click and drag to rotate view")
# #     print("- Right click and drag to translate view")
# #     print("- Scroll to zoom")
# #     print("- Press TAB to enable/disable joint control")
# #     print("- Close window to exit")
# #
# #     # Launch the viewer
# #     with mujoco.viewer.launch_passive(model, data) as viewer:
# #         # Enable visualization of contacts
# #         viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
# #         viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
# #
# #         # Run simulation
# #         start_time = time.time()
# #         while viewer.is_running():
# #             step_start = time.time()
# #
# #             # Step physics
# #             mujoco.mj_step(model, data)
# #
# #             # Sync viewer
# #             viewer.sync()
# #
# #             # Simple control: make robot move slightly to interact with dust
# #             current_time = data.time
# #             if model.njnt >= 6:
# #                 # Add some gentle motion to demonstrate dust interaction
# #                 data.ctrl[0] = 0.1 * np.sin(current_time * 0.5)  # Slow shoulder pan
# #                 data.ctrl[1] = -1.5 + 0.2 * np.sin(current_time * 0.3)  # Gentle shoulder lift
# #
# #             # Maintain real-time execution
# #             time_until_next_step = model.opt.timestep - (time.time() - step_start)
# #             if time_until_next_step > 0:
# #                 time.sleep(time_until_next_step)
# #
# # if __name__ == "__main__":
# #     run_interactive_simulation()
#
#
#
#
#
# import mujoco
# import mujoco.viewer
# import numpy as np
# import time
# import os
#
# def run_dynamic_dust_simulation():
#     """Run interactive MuJoCo simulation with DYNAMIC dust particles on the table"""
#
#     xml_file = 'ur5e_with_dynamic_dust_on_table.xml'
#
#     # Check if XML file exists
#     if not os.path.exists(xml_file):
#         print(f"❌ Error: {xml_file} not found!")
#         print("Make sure the XML file is in the same directory as this script.")
#         return
#
#     # Load the model
#     try:
#         model = mujoco.MjModel.from_xml_path(xml_file)
#         data = mujoco.MjData(model)
#         print("✅ Model loaded successfully!")
#         print(f"📊 Number of bodies: {model.nbody}")
#         print(f"🦾 Number of joints: {model.njnt}")
#         print(f"🔘 Number of geoms: {model.ngeom}")
#
#         # Count dust particles
#         dust_count = sum(1 for i in range(model.nbody)
#                         if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) and
#                         mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i).startswith('dust_particle_'))
#         print(f"✨ Number of DYNAMIC dust particles: {dust_count}")
#
#         # Initialize robot to a cleaning position over the table
#         if model.njnt >= 6:
#             data.qpos[0] = 1.4       # shoulder_pan - positioned over table
#             data.qpos[1] = -0.9      # shoulder_lift - reaching down to table
#             data.qpos[2] = 1.3       # elbow - bent to reach table surface
#             data.qpos[3] = -1.9      # wrist_1 - angled down to table
#             data.qpos[4] = -1.57     # wrist_2 - oriented properly
#             data.qpos[5] = 0.0       # wrist_3 - straight
#
#             # Forward kinematics to update positions
#             mujoco.mj_forward(model, data)
#
#         # Add small random velocities to dust particles for initial movement
#         for i in range(model.nbody):
#             body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
#             if body_name and body_name.startswith('dust_particle_'):
#                 # Find the joint index for this dust particle
#                 joint_start = model.body_jntadr[i]
#                 if joint_start >= 0 and joint_start < model.njnt:
#                     # Add small random initial velocities
#                     data.qvel[joint_start:joint_start+6] = np.random.normal(0, 0.01, 6)
#
#     except Exception as e:
#         print(f"❌ Error loading model: {e}")
#         print("\nTroubleshooting tips:")
#         print("1. Make sure all mesh files (.obj, .stl) are in the 'assets' folder")
#         print("2. Verify all UR5 robot mesh files are present")
#         return
#
#     print("\n🎮 Starting DYNAMIC table cleaning simulation...")
#     print("🌟 FEATURES:")
#     print("   • Dust particles can MOVE and be pushed around!")
#     print("   • Realistic physics and collision detection")
#     print("   • Particles positioned ON the table surface")
#     print("   • Watch them scatter when the mop touches them!")
#     print("\n🎮 Controls:")
#     print("   • Left click + drag: rotate view")
#     print("   • Right click + drag: translate view")
#     print("   • Scroll: zoom in/out")
#     print("   • Press TAB: enable/disable joint control")
#     print("   • Close window: exit simulation")
#
#     # Launch the viewer
#     with mujoco.viewer.launch_passive(model, data) as viewer:
#         # Enable contact visualization
#         viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
#         viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
#
#         # Set camera to view the table area
#         viewer.cam.lookat[0] = 0.3    # Table center X
#         viewer.cam.lookat[1] = 0.0    # Table center Y
#         viewer.cam.lookat[2] = 0.05   # Table surface level
#         viewer.cam.distance = 1.2     # Good viewing distance
#         viewer.cam.elevation = -25    # Look down at table
#         viewer.cam.azimuth = 45       # Angled view
#
#         start_time = time.time()
#         cleaning_started = False
#
#         while viewer.is_running():
#             step_start = time.time()
#
#             # Step physics - this is where dust particles will move!
#             mujoco.mj_step(model, data)
#
#             # Interactive cleaning motion - sweep across the table surface
#             current_time = data.time
#             if model.njnt >= 6:
#                 # Start cleaning motion after 2 seconds
#                 if current_time > 2.0:
#                     if not cleaning_started:
#                         print("🧹 Starting table cleaning! Watch the dust particles move!")
#                         cleaning_started = True
#
#                     # Aggressive sweeping motion to push dust around
#                     sweep_amplitude = 0.5    # Wider sweeping
#                     sweep_speed = 0.4        # Faster motion
#                     vertical_motion = 0.05   # Up/down movement
#
#                     # Dynamic cleaning pattern
#                     data.ctrl[0] = 1.2 + sweep_amplitude * np.sin(current_time * sweep_speed)
#                     data.ctrl[1] = -0.8 + vertical_motion * np.cos(current_time * sweep_speed * 1.3)
#                     data.ctrl[2] = 1.2 + 0.2 * np.sin(current_time * sweep_speed * 0.7)
#                     data.ctrl[3] = -1.8 + 0.3 * np.cos(current_time * sweep_speed * 0.5)
#                     data.ctrl[4] = -1.57 + 0.1 * np.sin(current_time * sweep_speed * 2.0)
#                     data.ctrl[5] = 0.3 * np.sin(current_time * sweep_speed * 1.8)  # Wrist rotation
#                 else:
#                     # Hold position initially
#                     data.ctrl[0] = 1.4
#                     data.ctrl[1] = -0.9
#                     data.ctrl[2] = 1.3
#                     data.ctrl[3] = -1.9
#                     data.ctrl[4] = -1.57
#                     data.ctrl[5] = 0.0
#
#             # Sync viewer
#             viewer.sync()
#
#             # Progress updates
#             if int(current_time) % 5 == 0 and int(current_time) > 0:
#                 if int(current_time) != getattr(run_dynamic_dust_simulation, 'last_print_time', -1):
#                     if cleaning_started:
#                         print(f"🧹 Cleaning in progress: {int(current_time)}s - Look at those dust particles move!")
#                     else:
#                         print(f"⏱️  Getting ready to clean: {int(current_time)}s")
#                     run_dynamic_dust_simulation.last_print_time = int(current_time)
#
#             # Maintain real-time execution
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
#
#     print("\n🎉 Dynamic table cleaning simulation ended!")
#     print("💫 Hope you enjoyed watching the dust particles move around!")
#
# if __name__ == "__main__":
#     print("🤖 UR5 Robot DYNAMIC Table Cleaning Simulation")
#     print("✨ Dynamic Dust Particles ON Table Surface")
#     print("🎯 Particles can MOVE and be pushed around!")
#     print("=" * 55)
#     run_dynamic_dust_simulation()




import sys
import threading
import numpy as np
import mujoco_py
import glfw

#----------------------------------------
# Configuration
#----------------------------------------

XML_PATH = "ur5e_with_mop_and_dust.xml"

# Initial joint values
INIT_POS = {
    "shoulder_pan": -2.32,
    "shoulder_lift": -0.314,
    "elbow": -1.04,
    "wrist_1": -0.377,
    "wrist_2": 0.0,
    "wrist_3": 0.0
}

# Sweep limits for simulation mode (rad)
SWEEP_MIN = -1.74
SWEEP_MAX = -4.90
SWEEP_STEP = -0.01  # negative step to move toward SWEEP_MAX

#----------------------------------------
# Helper Functions
#----------------------------------------

def key_callback(window, key, scancode, action, mods):
    """Capture key presses for active control adjustments."""
    global control_mode, user_target
    if action != glfw.PRESS:
        return

    # Toggle between modes
    if key == glfw.KEY_M:
        control_mode = "simulation" if control_mode == "active" else "active"
        print(f"Switched to {control_mode} mode.")

    # In active mode, adjust joints with keys:
    if control_mode == "active":
        if key == glfw.KEY_1:
            user_target["shoulder_pan"] += 0.1
        elif key == glfw.KEY_Q:
            user_target["shoulder_pan"] -= 0.1
        elif key == glfw.KEY_2:
            user_target["shoulder_lift"] += 0.1
        elif key == glfw.KEY_W:
            user_target["shoulder_lift"] -= 0.1
        elif key == glfw.KEY_3:
            user_target["elbow"] += 0.1
        elif key == glfw.KEY_E:
            user_target["elbow"] -= 0.1
        elif key == glfw.KEY_4:
            user_target["wrist_1"] += 0.1
        elif key == glfw.KEY_R:
            user_target["wrist_1"] -= 0.1
        elif key == glfw.KEY_5:
            user_target["wrist_2"] += 0.1
        elif key == glfw.KEY_T:
            user_target["wrist_2"] -= 0.1
        elif key == glfw.KEY_6:
            user_target["wrist_3"] += 0.1
        elif key == glfw.KEY_Y:
            user_target["wrist_3"] -= 0.1

        print("Target positions:", user_target)

#----------------------------------------
# Main
#----------------------------------------

if __name__ == "__main__":
    # Load model and create simulator
    model = mujoco_py.load_model_from_path(XML_PATH)
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)

    # Initialize joint targets
    user_target = INIT_POS.copy()
    control_mode = "active"

    # Apply initial positions
    for name, val in INIT_POS.items():
        sim.data.qpos[sim.model.get_joint_qpos_addr(name)] = val
    sim.forward()

    # Set up GLFW for key input
    if not glfw.init():
        print("Could not initialize GLFW")
        sys.exit(1)
    glfw.set_key_callback(viewer.window, key_callback)

    # Simulation loop
    sweep_angle = SWEEP_MIN
    while True:
        # Mode behavior
        if control_mode == "active":
            # Apply user-set targets each step
            for j, angle in user_target.items():
                sim.data.ctrl[model.actuator_name2id(f"{j}_act")] = angle
        else:
            # Simulation mode: fix all except shoulder_pan
            # Keep other joint commands at their INIT positions
            for j, angle in INIT_POS.items():
                if j == "shoulder_pan":
                    sim.data.ctrl[model.actuator_name2id("shoulder_pan_act")] = sweep_angle
                else:
                    sim.data.ctrl[model.actuator_name2id(f"{j}_act")] = angle

            # Update sweep angle
            sweep_angle += SWEEP_STEP
            if sweep_angle < SWEEP_MAX or sweep_angle > SWEEP_MIN:
                SWEEP_STEP *= -1  # reverse direction

        # Step and render
        sim.step()
        viewer.render()

        # Ensure dust falls only on table and can roll freely
        # (Dynamics are defined in XML: contype=0 free, friction>0 allows rolling)

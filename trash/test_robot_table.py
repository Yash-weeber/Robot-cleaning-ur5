import mujoco as mj
import mujoco.viewer as viewer
import numpy as np
import time

def test_robot_table_scene():
    """Test the robot and table scene"""
    
    try:
        # Load the model
        model = mj.MjModel.from_xml_path('scene_with_table.xml')
        data = mj.MjData(model)
        
        print("✅ Model loaded successfully!")
        print(f"Number of bodies: {model.nbody}")
        print(f"Number of joints: {model.njnt}")
        print(f"Number of actuators: {model.nu}")
        
        # Set initial joint positions (home position)
        initial_qpos = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
        data.qpos[:len(initial_qpos)] = initial_qpos
        
        # Forward kinematics
        mj.mj_forward(model, data)
        
        print("\n🤖 Robot joint names:")
        for i in range(model.nu):
            joint_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"  {i}: {joint_name}")
        
        print("\n📦 Body names:")
        for i in range(model.nbody):
            body_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)
            if body_name:
                print(f"  {i}: {body_name}")
        
        # Launch viewer
        print("\n🚀 Launching MuJoCo viewer...")
        print("Press Ctrl+C to exit")
        
        with viewer.launch_passive(model, data) as v:
            # Set initial camera position for better view
            v.cam.distance = 3.0
            v.cam.azimuth = 45
            v.cam.elevation = -30
            
            # Simple animation - move the robot joints
            start_time = time.time()
            
            while v.is_running():
                current_time = time.time() - start_time
                
                # Simple joint animation
                data.ctrl[0] = -1.5708 + 0.5 * np.sin(current_time * 0.5)  # shoulder_pan
                data.ctrl[1] = -1.5708 + 0.3 * np.sin(current_time * 0.7)  # shoulder_lift
                data.ctrl[2] = 1.5708 + 0.3 * np.sin(current_time * 0.3)   # elbow
                
                # Step simulation
                mj.mj_step(model, data)
                v.sync()
                
                time.sleep(0.01)
    
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure your table.obj file is in the 'assets' folder")
        print("2. Check that all mesh files are in the correct location")
        print("3. Verify the table mesh file name in ur5e_with_table.xml")
        print("4. Adjust the robot Z position based on your table height")

if __name__ == "__main__":
    test_robot_table_scene()

import mujoco
import numpy as np
import time

def load_and_run_simulation():
    """Load the MuJoCo model and run simulation"""

    # Load the model
    try:
        model = mujoco.MjModel.from_xml_path('ur5e_with_mop_and_dust.xml')
        data = mujoco.MjData(model)
        print("Model loaded successfully!")
        print(f"Number of bodies: {model.nbody}")
        print(f"Number of joints: {model.njnt}")
        print(f"Number of geoms: {model.ngeom}")

    except Exception as e:
        print(f"Error loading model: {e}")
        return None, None

    return model, data

def simulate_with_rendering(model, data, duration=10.0):
    """Run simulation with basic physics"""

    # Set up simulation parameters
    timestep = model.opt.timestep
    steps = int(duration / timestep)

    print(f"Running simulation for {duration} seconds ({steps} steps)")
    print("Dust particles will be affected by gravity and physics")
    print("Move the robot to interact with dust particles!")

    # Simple simulation loop (without rendering - for rendering you'd need mujoco.viewer)
    for step in range(steps):
        # Step the physics
        mujoco.mj_step(model, data)

        # Print status every 1000 steps
        if step % 1000 == 0:
            print(f"Step {step}/{steps}, Time: {data.time:.2f}s")

    print("Simulation completed!")

if __name__ == "__main__":
    model, data = load_and_run_simulation()
    if model is not None:
        simulate_with_rendering(model, data)

import os
import time
import numpy as np
import pandas as pd
import mujoco
from agent.pydmps.dmp_rhythmic import DMPs_rhythmic

# Internal imports from the factorized codebase
from runner.main_runner import EnhancedDMPController
from env.robot_logic import (
    get_joint_positions, set_joint_positions, enhanced_ik_solver
)
from env.llm_robot_logic import (
    generate_warmup_trajectory, get_dmp_step_with_obstacles, log_iteration_data
)
from agent.llm_client import LLMInterface
from agent.llm_data_utils import (
    read_weights_csv, write_weights_csv, row_to_2x50,
    parse_ollama_weights, save_trajectory_data, save_dialog,
    append_weight_history, save_ik_error
)
from agent.llm_analysis import (
    load_trajectory_history, analyze_trajectory_performance,
    load_iteration_log, load_traj_feedback, build_llm_feedback
)


def run_llm_optimization(config):
    """
    Orchestrates the LLM optimization loop.
    Matches logic from enhancedll_Mohamed.py exactly.
    """
    # Initialize Controller and LLM Interface
    controller = EnhancedDMPController(config)
    llm = LLMInterface(config)

    # Setup directories as defined in original ensure_dirs
    os.makedirs(config['logs']['root'], exist_ok=True)
    os.makedirs(config['logs']['dialog_dir'], exist_ok=True)

    bounds = {
        "xmin": controller.x_min, "xmax": controller.x_max,
        "ymin": controller.y_min, "ymax": controller.y_max,
    }

    n_bfs = config['dmp_params']['n_bfs']
    max_iters = config['simulation']['max_iters']
    n_warmup = config['llm_settings']['n_warmup']
    feedback_window = config['llm_settings']['feedback_window']
    weights_csv_path = os.path.join(config['logs']['root'], "weights.csv")

    # Initialize DMP exactly as original script
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
    n_counter = 0

    print("\nStarting LLM-Driven Optimization Loop...")

    for it in range(1 - n_warmup, max_iters + 1):
        # Reset controller at start of every iteration
        controller.hard_reset_from_home(redraw=False)

        # Warmup: Use predefined trajectories and bootstrap weights
        if it < 0:
            if (it - 1) % 5 == 0:
                trajectory = generate_warmup_trajectory(n_counter)
                if trajectory is not None:
                    dmp.imitate_path(trajectory.T, plot=False)
                    write_weights_csv(weights_csv_path, dmp.w.copy())
                    n_counter += 1

        # Load weights for current iteration
        try:
            w2 = read_weights_csv(weights_csv_path, n_bfs)
        except Exception as e:
            print(f"Error loading weights at iter {it}: {e}")
            continue

        print(f"Iteration {it}: Applying Weights")
        dmp.w = w2.copy()
        dmp.reset_state()
        append_weight_history(config['logs']['weight_history_csv'], it, "executed", w2.copy(), n_bfs)

        # Physics Simulation Loop
        model, data = controller.model, controller.data
        joint_names = controller.joint_names
        start_joints = get_joint_positions(model, data, joint_names)

        joint_traj = []
        dmp_task_trajectory = []
        keep_every = max(1, int(config['dmp_params']['deci_build']))

        for i in range(int(dmp.timesteps)):
            y = get_dmp_step_with_obstacles(dmp)
            target_3d = np.array([y[0], y[1], config['robot']['mop_z_height']], dtype=float)
            dmp_task_trajectory.append(target_3d)

            ok, err_val = enhanced_ik_solver(
                model, data, controller.site_id, target_3d, joint_names,
                max_iters_per_wp=50, print_every=1000
            )

            if not ok:
                # Log IK failures as per original save_ik_error logic
                save_ik_error(it, i, target_3d, err_val or float("nan"), config['logs']['root'] + "/ik_errors.csv")
                continue

            if i % keep_every == 0:
                joint_traj.append(get_joint_positions(model, data, joint_names).copy())

        # Execute Trajectory if joints generated
        if joint_traj:
            set_joint_positions(model, data, joint_names, start_joints)
            # Original execute call with dt*2 logic
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt * 2)

        # Data Persistence
        save_trajectory_data(it, dmp_task_trajectory, config['logs']['dmp_trajectory_csv'])

        # World state cleanup before ball count
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

        grid = controller.count_balls_in_grid()
        total_balls = int(np.sum(grid))
        log_iteration_data(it, grid, total_balls, len(joint_traj), config['logs']['iter_log_csv'])

        if total_balls == 0:
            print("Cleanup complete. Optimization successful.")
            break

        # Load historical data for prompt construction
        iter_log_data = load_iteration_log(config['logs']['iter_log_csv'])
        traj_feedback_data = load_traj_feedback(config['logs']['dmp_trajectory_csv'])

        # Logic to decide next weights
        if it < 0:
            # Exploration during warmup
            np.random.seed(config['llm_settings']['seed_number'] + it)
            w_next = w2 + np.random.randn(2, n_bfs) * config['dmp_params']['random_scale']
        else:
            # Build detailed feedback text for the LLM prompt
            w_df = pd.read_csv(config['logs']['weight_history_csv']) if os.path.exists(
                config['logs']['weight_history_csv']) else None
            feedback_text = build_llm_feedback(
                it + 1, w_df, iter_log_data, traj_feedback_data, feedback_window, n_warmup
            )

            prompt = llm.render_prompt(it + 1, feedback_text, bounds)

            try:
                response = llm.call_ollama(prompt)
                w_next = parse_ollama_weights(response, n_bfs)
                save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
            except Exception as e:
                print(f"LLM Error at iteration {it}: {e}. Reusing current weights.")
                w_next = w2.copy()

        # Update weights for next iteration
        append_weight_history(config['logs']['weight_history_csv'], it + 1, "proposed", w_next, n_bfs)
        write_weights_csv(weights_csv_path, w_next)

    controller.viewer.close()
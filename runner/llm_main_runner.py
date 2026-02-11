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

    # Initialize Controller and LLM Interface
    controller = EnhancedDMPController(config)
    llm = LLMInterface(config)

    # Setup directories
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

    # Initialize DMP
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
    n_counter = 0

    print("\n Starting Synchronized LLM-Driven Optimization...")

    for it in range(1 - n_warmup, max_iters + 1):
        # Reset world snapshot but place robot at HOME
        controller.hard_reset_from_home(redraw=False)

        # Warmup: Predefined trajectories and weight bootstrapping
        if it < 0:
            if (it - 1) % 5 == 0:
                trajectory = generate_warmup_trajectory(n_counter, config)
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

        print(f"Iteration {it}: Executing Policy")
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
            # Step DMP with aggressive obstacle avoidance gains
            y = get_dmp_step_with_obstacles(dmp)
            target_3d = np.array([y[0], y[1], config['robot']['mop_z_height']], dtype=float)
            dmp_task_trajectory.append(target_3d)

            # High-speed IK solver settings for optimization runs
            ok, err_val = enhanced_ik_solver(
                model, data, controller.site_id, target_3d, joint_names,
                max_iters_per_wp=50, print_every=1000
            )

            if not ok:
                save_ik_error(it, i, target_3d, err_val or float("nan"), config['logs']['ik_error_csv'])
                continue

            if i % keep_every == 0:
                joint_traj.append(get_joint_positions(model, data, joint_names).copy())

        # Execute joint movements if successful
        if joint_traj:
            set_joint_positions(model, data, joint_names, start_joints)
            controller.execute_joint_trajectory(joint_traj, dt=controller.dt * 2)

        # Physics Settlement: Extra steps to let balls stop rolling before count
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

        # Data Persistence
        save_trajectory_data(it, dmp_task_trajectory, config['logs']['dmp_trajectory_csv'])
        save_trajectory_data(it, controller.ee_trajectory, config['logs']['ee_trajectory_csv'])

        # Spatial Ball Counting
        grid = controller.count_balls_in_grid()
        total_balls = int(np.sum(grid))
        log_iteration_data(it, grid, total_balls, len(joint_traj), config['logs']['iter_log_csv'])

        # LLM Feedback Construction
        iter_log_data = load_iteration_log(config['logs']['iter_log_csv'], config['dmp_params']['num_x_segments'], config['dmp_params']['num_y_segments'])
        # CRITICAL: Use Actual EE Trajectory for Bounds Analysis
        traj_feedback_data = load_traj_feedback(config['logs']['ee_trajectory_csv'])
        ee_traj_df = pd.read_csv(config['logs']['ee_trajectory_csv']) if os.path.exists(config['logs']['ee_trajectory_csv']) else None

        # Logic to decide next weights
        if it < 0:
            # Random exploration during warmup period
            np.random.seed(config['llm_settings']['seed_number'] + it)
            w_next = w2 + np.random.randn(2, n_bfs) * config['dmp_params']['random_scale']
        else:
            # Build detailed prompt with coordinate tables and grid markdown
            feedback_text, guidance_text = build_llm_feedback(
                it + 1, pd.read_csv(config['logs']['weight_history_csv']),
                iter_log_data, traj_feedback_data, ee_traj_df, config, bounds
            )

            prompt = llm.render_prompt(it + 1, feedback_text, bounds, guidance_text=guidance_text)
            # save_dialog(config['logs']['dialog_dir'], it + 1, prompt, "")

            try:
                # Use large token limit for coordinate tables
                if config['llm_settings']['llm_model'].startswith("gpt"):
                    response = llm.call_ollama(prompt, token_limit=118000)
                elif config['llm_settings']['llm_model'].startswith("gemini"):
                    response = llm.call_gemini(prompt)
                else:
                    raise ValueError(f"Unsupported LLM model: {config['llm_settings']['llm_model']}")
                w_next = parse_ollama_weights(response, n_bfs)
                save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
            except Exception as e:
                print(f"LLM Error at iteration {it}: {e}. Reusing current weights.")
                w_next = w2.copy()

        # Update for next iteration
        append_weight_history(config['logs']['weight_history_csv'], it + 1, "proposed", w_next, n_bfs)
        write_weights_csv(weights_csv_path, w_next)

    controller.viewer.close()
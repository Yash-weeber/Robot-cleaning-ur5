# import os
# import time
# import numpy as np
# import pandas as pd
# import mujoco
# import cma 
# from concurrent.futures import ProcessPoolExecutor

# # Import your custom modules
# from runner.main_runner import EnhancedDMPController
# from agent.dmp_logic import DMPs_rhythmic
# from env.robot_logic import get_joint_positions, set_joint_positions, enhanced_ik_solver
# from env.llm_robot_logic import generate_warmup_trajectory, get_dmp_step_with_obstacles, log_iteration_data
# from agent.llm_data_utils import (
#     write_weights_csv, append_weight_history, save_trajectory_data, save_ik_error
# )

# def evaluate_candidate_worker(args):

#     sol, config, gen, cand_idx = args
#     n_bfs = config['dmp_params']['n_bfs']
#     num_balls = config['dmp_params']['num_balls']
    
#     # Each process must create its own controller and DMP instance
#     local_controller = EnhancedDMPController(config)
#     local_dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=local_controller.dt)
    
#     # Setup weights
#     w2 = sol.reshape(2, n_bfs)
#     local_controller.hard_reset_from_home(redraw=False)
#     local_dmp.w = w2.copy()
#     local_dmp.reset_state()
    
#     joint_traj = []
#     dmp_task_trajectory = []
#     expected_steps = int(local_dmp.timesteps)

#     # 1. Trajectory Generation & IK
#     for step in range(expected_steps):
#         y = get_dmp_step_with_obstacles(local_dmp)
#         target_3d = np.array([y[0], y[1], config['robot']['mop_z_height']])
#         dmp_task_trajectory.append(target_3d)

#         ok, err = enhanced_ik_solver(
#             local_controller.model, local_controller.data, local_controller.site_id, 
#             target_3d, local_controller.joint_names, max_iters_per_wp=100
#         )
#         if ok:
#             joint_traj.append(get_joint_positions(local_controller.model, local_controller.data, local_controller.joint_names).copy())

#     # 2. Execution
#     if len(joint_traj) > (expected_steps * 0.85):
#         local_controller.execute_joint_trajectory(joint_traj)
    
#     # Settling steps
#     for _ in range(100):
#         mujoco.mj_step(local_controller.model, local_controller.data)
    
#     # 3. Reward/Fitness Calculation
#     grid = local_controller.count_balls_in_grid()
#     balls_remaining = int(np.sum(grid))
    
#     # Distance penalty calculation
#     table_diagonal = np.sqrt((local_controller.x_max - local_controller.x_min)**2 + 
#                              (local_controller.y_max - local_controller.y_min)**2)
#     max_possible_dist_penalty = num_balls * table_diagonal
    
#     dist_penalty = 0.0
#     for b_idx in range(1, num_balls + 1):
#         body_id = mujoco.mj_name2id(local_controller.model, mujoco.mjtObj.mjOBJ_BODY, f"ball_{b_idx}")
#         if body_id != -1:
#             pos = local_controller.data.xpos[body_id][:2]
#             if (local_controller.x_min <= pos[0] <= local_controller.x_max) and \
#                (local_controller.y_min <= pos[1] <= local_controller.y_max):
#                 dist_penalty += (1.2 - np.linalg.norm(pos))

#     fitness = (balls_remaining / num_balls) + 0.05 * (dist_penalty / max_possible_dist_penalty)
    
#     # We close the local viewer/context to free memory
#     if hasattr(local_controller, 'viewer') and local_controller.viewer is not None:
#         local_controller.viewer.close()

#     # Return results to main process for logging
#     return {
#         "fitness": fitness,
#         "balls_remaining": balls_remaining,
#         "grid": grid,
#         "dmp_task_trajectory": dmp_task_trajectory,
#         "joint_traj_len": len(joint_traj),
#         "weights": w2
#     }
# def get_unique_run_folder(base_root):
#     """Checks for existing folders and returns a new path like base_root/run_1, run_2..."""
#     if not os.path.exists(base_root):
#         os.makedirs(base_root, exist_ok=True)
#     i = 1
#     while os.path.exists(os.path.join(base_root, f"{i}")):
#         i += 1
#     return os.path.join(base_root, f"{i}")
# # def run_cmaes_optimization(config):
# #     # Phase 1: Bootstrapping remains sequential (only 4 runs)
# #     controller = EnhancedDMPController(config)
# #     # os.makedirs(config['logs']['root'], exist_ok=True)
# #     base_log_path = config['logs']['root']
# #     new_run_path = get_unique_run_folder(base_log_path)
# #     config['logs']['root'] = new_run_path
# #     n_bfs = config['dmp_params']['n_bfs']
# #     num_balls = config['dmp_params']['num_balls']
# #     max_generations = config['simulation']['max_iters']
# #     weights_csv_path = os.path.join(config['logs']['root'], "weights.csv")
# #     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
# #     if 'ik_error_csv' in config['logs']:
# #         config['logs']['ik_error_csv'] = os.path.join(new_run_path, "ik_errors.csv")

# #     os.makedirs(new_run_path, exist_ok=True)
# def run_cmaes_optimization(config):
#     # 1. Generate the unique folder for this specific run
#     base_log_path = config['logs']['root']
#     new_run_path = get_unique_run_folder(base_log_path)
    
#     # 2. Update the config so both the main process and workers use the new folder
#     config['logs']['root'] = new_run_path
#     config['logs']['iter_log_csv'] = os.path.join(new_run_path, "iteration_log.csv")
#     config['logs']['dmp_trajectory_csv'] = os.path.join(new_run_path, "dmp_trajectory.csv")
#     config['logs']['weight_history_csv'] = os.path.join(new_run_path, "weight_history.csv")
#     # Ensure any other specific logs like ik_errors follow the new path
#     if 'ik_error_csv' in config['logs']:
#         config['logs']['ik_error_csv'] = os.path.join(new_run_path, "ik_errors.csv")

#     # 3. Create the physical directory
#     os.makedirs(new_run_path, exist_ok=True)
    


#     controller = EnhancedDMPController(config)
#     n_bfs = config['dmp_params']['n_bfs']
#     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
#     num_balls = config['dmp_params']['num_balls']
#     max_generations = config['simulation']['cmaesmax_iters']
#     weights_csv_path = os.path.join(config['logs']['root'], "weights.csv")
#     consecutive_success_count = 0
#     SUCCESS_GOAL = 10
#     BALL_TARGET = 0
    
#     min_balls_remaining = float('inf')
#     best_warmup_weights = None

#     print("\n[CMA-ES] Phase 1: Bootstrapping from Warmup Trajectories...")
#     for n in range(4):
#         trajectory = generate_warmup_trajectory(n)
#         if trajectory is not None:
#             dmp.imitate_path(trajectory.T)
#             controller.hard_reset_from_home(redraw=False)
#             grid = controller.count_balls_in_grid()
#             remaining = np.sum(grid)
#             if remaining < min_balls_remaining:
#                 min_balls_remaining = remaining
#                 best_warmup_weights = dmp.w.copy().flatten()

#     x0 = best_warmup_weights if best_warmup_weights is not None else np.zeros(2 * n_bfs)
#     sigma0 = config['dmp_params'].get('random_scale', 10.0)
#     pop_size = 20
#     es = cma.CMAEvolutionStrategy(x0, sigma0, {'popsize': pop_size})
    
#     print(f"\n[CMA-ES] Phase 2: Starting Parallel Evolution ({os.cpu_count()} workers)...")

#     # The ProcessPool handles the workers
#     with ProcessPoolExecutor(max_workers=os.cpu_count()) as executor:
#         for gen in range(1, max_generations + 1):
#             solutions = es.ask()
            
#             # Map solutions to workers
#             task_args = [(sol, config, gen, i) for i, sol in enumerate(solutions)]
#             print(f"\n--- Generation {gen}: Evaluating {len(solutions)} candidates in parallel ---")
            
#             # This line runs all 14 candidates at once
#             results = list(executor.map(evaluate_candidate_worker, task_args))
            
#             # Process results for CMA-ES and Logging
#             fitness_list = []
#             gen_best_balls = float('inf')
#             gen_best_weights = None

#             for i, res in enumerate(results):
#                 fitness_list.append(res["fitness"])
                
#                 # Logging (Handled in main process to prevent file conflicts)
#                 log_iteration_data(f"{gen}_{i}", res["grid"], res["balls_remaining"], 
#                                    res["joint_traj_len"], config['logs']['iter_log_csv'])
#                 save_trajectory_data(f"{gen}_{i}", res["dmp_task_trajectory"], 
#                                      config['logs']['dmp_trajectory_csv'])
                
#                 # Track Success Streak (based on the best candidate of this generation)
#                 if res["balls_remaining"] < gen_best_balls:
#                     gen_best_balls = res["balls_remaining"]
#                     gen_best_weights = res["weights"]

#             print(f"  Gen {gen} Summary: Best remaining: {gen_best_balls}")

#             # Update streak based on best performance in generation
#             if gen_best_balls <= BALL_TARGET:
#                 consecutive_success_count += 1
#             else:
#                 consecutive_success_count = 0 

#             # Termination check
#             if consecutive_success_count >= SUCCESS_GOAL:
#                 print(f"\n[STABLE] Success reached {SUCCESS_GOAL} times consecutively. Exiting.")
#                 write_weights_csv(weights_csv_path, gen_best_weights)
#                 return

#             es.tell(solutions, fitness_list)
            
#             # Best of all time logging
#             best_w = es.result.xbest.reshape(2, n_bfs)
#             append_weight_history(config['logs']['weight_history_csv'], gen, "cmaes_best", best_w, n_bfs)

#             if es.stop():
#                 break

#     controller.viewer.close()



import os
import time
import json
import numpy as np
import pandas as pd
import mujoco
import cma 
from concurrent.futures import ProcessPoolExecutor

# Import your custom modules
from runner.main_runner import EnhancedDMPController
from agent.dmp_logic import DMPs_rhythmic
from env.robot_logic import get_joint_positions, set_joint_positions, enhanced_ik_solver
from env.llm_robot_logic import generate_warmup_trajectory, get_dmp_step_with_obstacles, log_iteration_data
from agent.llm_data_utils import (
    write_weights_csv, append_weight_history, save_trajectory_data, save_ik_error
)

def evaluate_candidate_worker(args):
    sol, config, gen, cand_idx = args
    n_bfs = config['dmp_params']['n_bfs']
    num_balls = config['dmp_params']['num_balls']
    
    # Each process must create its own controller and DMP instance
    local_controller = EnhancedDMPController(config)
    local_dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=local_controller.dt)
    
    # Setup weights from CMA-ES solution
    w2 = sol.reshape(2, n_bfs)
    local_controller.hard_reset_from_home(redraw=False)
    local_dmp.w = w2.copy()
    local_dmp.reset_state()
    
    joint_traj = []
    dmp_task_trajectory = []
    expected_steps = int(local_dmp.timesteps)

    # 1. Trajectory Generation & IK
    for step in range(expected_steps):
        y = get_dmp_step_with_obstacles(local_dmp)
        target_3d = np.array([y[0], y[1], config['robot']['mop_z_height']])
        dmp_task_trajectory.append(target_3d)

        ok, err = enhanced_ik_solver(
            local_controller.model, local_controller.data, local_controller.site_id, 
            target_3d, local_controller.joint_names, max_iters_per_wp=100
        )
        if ok:
            joint_traj.append(get_joint_positions(local_controller.model, local_controller.data, local_controller.joint_names).copy())

    # 2. Execution
    if len(joint_traj) > (expected_steps * 0.85):
        local_controller.execute_joint_trajectory(joint_traj)
    
    # Settling steps
    for _ in range(100):
        mujoco.mj_step(local_controller.model, local_controller.data)
    
    # 3. Reward/Fitness Calculation
    grid = local_controller.count_balls_in_grid()
    balls_remaining = int(np.sum(grid))
    
    # Distance penalty calculation
    table_diagonal = np.sqrt((local_controller.x_max - local_controller.x_min)**2 + 
                             (local_controller.y_max - local_controller.y_min)**2)
    max_possible_dist_penalty = num_balls * table_diagonal
    
    dist_penalty = 0.0
    for b_idx in range(1, num_balls + 1):
        body_id = mujoco.mj_name2id(local_controller.model, mujoco.mjtObj.mjOBJ_BODY, f"ball_{b_idx}")
        if body_id != -1:
            pos = local_controller.data.xpos[body_id][:2]
            if (local_controller.x_min <= pos[0] <= local_controller.x_max) and \
               (local_controller.y_min <= pos[1] <= local_controller.y_max):
                dist_penalty += (1.2 - np.linalg.norm(pos))

    # Lower fitness is better in CMA-ES
    fitness = (balls_remaining / num_balls) + 0.05 * (dist_penalty / max_possible_dist_penalty)
    
    if hasattr(local_controller, 'viewer') and local_controller.viewer is not None:
        local_controller.viewer.close()

    return {
        "fitness": fitness,
        "balls_remaining": balls_remaining,
        "grid": grid,
        "dmp_task_trajectory": dmp_task_trajectory,
        "joint_traj_len": len(joint_traj),
        "weights": w2
    }


def get_unique_run_folder(base_root):
    if not os.path.exists(base_root):
        os.makedirs(base_root, exist_ok=True)
    i = 1
    while os.path.exists(os.path.join(base_root, str(i))):
        i += 1
    return os.path.join(base_root, str(i))


def run_cmaes_optimization(config):
    # --- 1. Folder & Config Setup ---
    base_log_path = config['logs']['root']
    new_run_path = get_unique_run_folder(base_log_path)
    
    config['logs']['root'] = new_run_path
    config['logs']['iter_log_csv'] = os.path.join(new_run_path, "iteration_log.csv")
    config['logs']['dmp_trajectory_csv'] = os.path.join(new_run_path, "dmp_trajectory.csv")
    config['logs']['weight_history_csv'] = os.path.join(new_run_path, "weight_history.csv")
    if 'ik_error_csv' in config['logs']:
        config['logs']['ik_error_csv'] = os.path.join(new_run_path, "ik_errors.csv")

    os.makedirs(new_run_path, exist_ok=True)
    
    # Save a snapshot of the config for future reference
    with open(os.path.join(new_run_path, "config_used.json"), "w") as f:
        json.dump(config, f, indent=4)

    # --- 2. Initialize Hardware/Sim logic ---
    controller = EnhancedDMPController(config)
    n_bfs = config['dmp_params']['n_bfs']
    num_balls = config['dmp_params']['num_balls']
    max_generations = config['simulation'].get('cmaesmax_iters', 50)
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
    
    min_balls_remaining = float('inf')
    best_warmup_weights = None

    # --- PHASE 1: WARMUP (Sequential) ---
    # These are baseline runs to find a good starting point for CMA-ES
    n_warmup = config['llm_settings'].get('n_warmup', 4)
    print(f"\n[PHASE 1] Bootstrapping: Running {n_warmup} Warmup Trajectories...")
    
    for n in range(n_warmup):
        trajectory = generate_warmup_trajectory(n)
        if trajectory is not None:
            dmp.imitate_path(trajectory.T)
            controller.hard_reset_from_home(redraw=False)
            
            # Execute warmup
            grid = controller.count_balls_in_grid()
            remaining = int(np.sum(grid))
            
            # Log Warmup Data (distinct IDs)
            log_iteration_data(f"warmup_{n}", grid, remaining, 0, config['logs']['iter_log_csv'])
            append_weight_history(config['logs']['weight_history_csv'], n, "warmup", dmp.w, n_bfs)

            if remaining < min_balls_remaining:
                min_balls_remaining = remaining
                best_warmup_weights = dmp.w.copy().flatten()
            
            print(f" > Warmup {n+1}/{n_warmup} Complete. Balls remaining: {remaining}")

    # --- PHASE 2: CMA-ES EVOLUTION (Parallel) ---
    x0 = best_warmup_weights if best_warmup_weights is not None else np.zeros(2 * n_bfs)
    sigma0 = config['dmp_params'].get('random_scale', 5.0)
    pop_size = 20 
    es = cma.CMAEvolutionStrategy(x0, sigma0, {'popsize': pop_size})
    
    print(f"\n[PHASE 2] Starting CMA-ES Evolution (Gen 1 to {max_generations})...")
    print(f"Workers: {os.cpu_count()} | Population: {pop_size}")

    consecutive_success_count = 0
    SUCCESS_GOAL = 5

    with ProcessPoolExecutor(max_workers=os.cpu_count()) as executor:
        for gen in range(1, max_generations + 1):
            solutions = es.ask()
            task_args = [(sol, config, gen, i) for i, sol in enumerate(solutions)]
            
            print(f"\n--- Generation {gen}: Evaluating candidates in parallel ---")
            results = list(executor.map(evaluate_candidate_worker, task_args))
            
            fitness_list = []
            gen_best_balls = float('inf')
            gen_best_weights = None

            for i, res in enumerate(results):
                fitness_list.append(res["fitness"])
                
                # Persistence (Handled by main process)
                log_id = f"{gen}_{i}"
                log_iteration_data(log_id, res["grid"], res["balls_remaining"], 
                                   res["joint_traj_len"], config['logs']['iter_log_csv'])
                save_trajectory_data(log_id, res["dmp_task_trajectory"], 
                                     config['logs']['dmp_trajectory_csv'])
                
                if res["balls_remaining"] < gen_best_balls:
                    gen_best_balls = res["balls_remaining"]
                    gen_best_weights = res["weights"]

            print(f" Gen {gen} Summary: Best remaining: {gen_best_balls}")

            # Update Optimizer
            es.tell(solutions, fitness_list)
            
            # Log best weight history
            append_weight_history(config['logs']['weight_history_csv'], gen, "cma_best", gen_best_weights, n_bfs)

            # Success Streak Check
            if gen_best_balls <= 0:
                consecutive_success_count += 1
                if consecutive_success_count >= SUCCESS_GOAL:
                    print(f"\n[STABLE] Success goal reached. Table cleared consistently.")
                    break
            else:
                consecutive_success_count = 0 

            if es.stop():
                print("[SYSTEM] CMA-ES stop criteria reached.")
                break

    # Save final weights
    final_weights_path = os.path.join(new_run_path, "final_best_weights.csv")
    write_weights_csv(final_weights_path, gen_best_weights)
    print(f"\nOptimization Finished. Best weights saved to {final_weights_path}")
    
    controller.viewer.close()
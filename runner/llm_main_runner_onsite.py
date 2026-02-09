import os
import pickle
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

class LLM_Brain:
    '''Class that runs the llm optimization loop'''
    def __init__(self, config):
        self.config = config
        self.llm_interface = LLMInterface(config)

        # Setup directories
        os.makedirs(config['logs']['root'], exist_ok=True)
        os.makedirs(config['logs']['dialog_dir'], exist_ok=True)
        self.ws_center = config["simulation"]["ws_center"]
        ws_width = config["simulation"]["ws_width"]
        ws_length = config["simulation"]["ws_length"]
        self.x_min = self.ws_center[0] - ws_width / 2.0
        self.x_max = self.ws_center[0] + ws_width / 2.0
        self.y_min = self.ws_center[1] - ws_length / 2.0
        self.y_max = self.ws_center[1] + ws_length / 2.0
        self.num_x_segments = config['dmp_params']['num_x_segments']
        self.num_y_segments = config['dmp_params']['num_y_segments']
        self.grid_count = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)

        self.bounds = {
            "xmin": self.x_min, "xmax": self.x_max,
            "ymin": self.y_min, "ymax": self.y_max,
        }

        self.n_bfs = config['dmp_params']['n_bfs']
        self.dt = config['dmp_params']['dt']
        self.max_iters = config['simulation']['max_iters']
        self.n_warmup = config['llm_settings']['n_warmup']
        self.feedback_window = config['llm_settings']['feedback_window']
        self.weights_csv_path = os.path.join(config['logs']['root'], "weights.csv")

        # Initialize DMP
        self.dmp = DMPs_rhythmic(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)
        self.iteration = self._find_iteration_number()
    
    def _find_iteration_number(self):
        '''Find the current iteration number based on existing logs'''
        weight_history_path = self.config['logs']['weight_history_csv']
        if os.path.exists(weight_history_path):
            df = pd.read_csv(weight_history_path)
            if not df.empty:
                return int(df['iter'].max()) + 1
        return 0
    
    def _prompt_llm(self):
        """prompts the llm for new weights based on past performance"""
        iter_log_data = load_iteration_log(self.config['logs']['iter_log_csv'], self.config['dmp_params']['num_x_segments'], self.config['dmp_params']['num_y_segments'])
        traj_feedback_data = load_traj_feedback(self.config['logs']['dmp_trajectory_csv'])
        ee_traj_df = pd.read_csv(self.config['logs']['dmp_trajectory_csv'])
        feedback_text, guidance_text = build_llm_feedback(
                self.iteration + 1, pd.read_csv(self.config['logs']['weight_history_csv']),
                iter_log_data, traj_feedback_data, ee_traj_df, self.config, self.bounds
            )
        prompt = self.llm_interface.render_prompt(self.iteration, feedback_text, self.bounds, guidance_text=guidance_text)
        response = self.llm_interface.call_ollama(prompt, token_limit=118000)
        w_next = parse_ollama_weights(response, self.n_bfs)
        save_dialog(self.config['logs']['dialog_dir'], self.iteration, prompt, response)
        return w_next

    def _generate_dmp_trajectory(self, weights):
        """Generates a DMP trajectory given weights"""
        trajectory = generate_warmup_trajectory(0, self.config)
        self.dmp.imitate_path(trajectory.T, plot=False)
        self.dmp.w = weights.copy()
        self.dmp.reset_state()
        dmp_task_trajectory = []

        for i in range(int(self.dmp.timesteps)):
            # Step DMP with aggressive obstacle avoidance gains
            y = get_dmp_step_with_obstacles(self.dmp)
            target_3d = np.array([y[0], y[1], self.config['robot']['mop_z_height']], dtype=float)
            dmp_task_trajectory.append(target_3d)
        
        return dmp_task_trajectory
    
    def _check_trajectory_in_bounds(self, trajectory):
        """Checks if the trajectory stays within bounds"""
        for point in trajectory:
            if not (self.x_min <= point[0] <= self.x_max and self.y_min <= point[1] <= self.y_max):
                print(f"Trajectory point {point} out of bounds: {self.bounds}")
                return False
        return True
    
    def _obtain_reward_from_user(self):
        """Ask user to provide a reward score for the trajectory"""
        while True:
            try:
                reward = float(input(f"Please provide a reward score (0-100) for iteration {self.iteration}: "))
                reward = 100 - reward  # Convert to cost
                if 0 <= reward <= 100:
                    return reward
                else:
                    print("Reward must be between 0 and 100.")
            except ValueError:
                print("Invalid input. Please enter a numeric value between 0 and 100.")

    def extract_traj_to_pkl(self, resample_rate=20):
        # Load trajectory data from CSV
        df_dmp = pd.read_csv(self.config['logs']['dmp_trajectory_csv'])
        dmp_traj_data = df_dmp[df_dmp['iter'] == self.iteration]
        dmp_traj_data.drop(columns=['iter', 'timestamp', 'step'], inplace=True)
        dmp_traj_data = dmp_traj_data.iloc[::resample_rate, :].reset_index(drop=True)
        # print(dmp_traj_data.head())
        # x_traj = dmp_traj_data.filter(like='x').to_numpy()
        # y_traj = dmp_traj_data.filter(like='y').to_numpy()
        # dmp_traj_data_dict = {
        #     'iteration': self.iteration,
        #     'x_traj': x_traj,
        #     'y_traj': y_traj
        # }
        # # Save to pickle
        # with open(self.config['logs']['traj_out_pkl'], 'wb') as f:
        #     pickle.dump(dmp_traj_data_dict, f)
        x_traj = dmp_traj_data.filter(like='x').to_numpy().ravel().tolist()
        y_traj = dmp_traj_data.filter(like='y').to_numpy().ravel().tolist()
        traj = []
        for k in range(len(x_traj)):
            traj.append([x_traj[k], y_traj[k], -0.108])

        with open(self.config['logs']['traj_out_pkl'], 'wb') as f:
            print(f"Saving trajectory pickle for iteration {self.iteration} with {len(traj)} points.\n in {self.config['logs']['traj_out_pkl']}")
            pickle.dump(traj, f)

    def step(self):
        """Performs a single optimization step"""
        traj_in_bound = False
        while not traj_in_bound:
            w_next = self._prompt_llm()
            trajectory = self._generate_dmp_trajectory(w_next)
            traj_in_bound = self._check_trajectory_in_bounds(trajectory)
        
        save_trajectory_data(self.iteration, trajectory, self.config['logs']['dmp_trajectory_csv'])
        save_trajectory_data(self.iteration, trajectory, self.config['logs']['ee_trajectory_csv'])
        # Update for next iteration
        self.extract_traj_to_pkl(resample_rate=20)
        append_weight_history(self.config['logs']['weight_history_csv'], self.iteration, "proposed", w_next, self.n_bfs)
        write_weights_csv(self.weights_csv_path, w_next)
        reward = self._obtain_reward_from_user()
        grid_mat = None #np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)
        print(f"Received reward: {reward} for iteration {self.iteration}")
        
        log_iteration_data(self.iteration, grid_mat, reward, len(trajectory), self.config['logs']['iter_log_csv'])
        
        append_weight_history(self.config['logs']['weight_history_csv'], self.iteration, "executed", w_next, self.n_bfs)
        # write_weights_csv(self.weights_csv_path, w_next)
        
        self.iteration += 1
    
    
# def run_llm_optimization(config):

#     # Initialize Controller and LLM Interface
#     controller = EnhancedDMPController(config)
#     llm = LLMInterface(config)

#     # Setup directories
#     os.makedirs(config['logs']['root'], exist_ok=True)
#     os.makedirs(config['logs']['dialog_dir'], exist_ok=True)
    

#     bounds = {
#         "xmin": controller.x_min, "xmax": controller.x_max,
#         "ymin": controller.y_min, "ymax": controller.y_max,
#     }

#     n_bfs = config['dmp_params']['n_bfs']
#     max_iters = config['simulation']['max_iters']
#     n_warmup = config['llm_settings']['n_warmup']
#     feedback_window = config['llm_settings']['feedback_window']
#     weights_csv_path = os.path.join(config['logs']['root'], "weights.csv")

#     # Initialize DMP
#     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
#     n_counter = 0

#     print("\n Starting Synchronized LLM-Driven Optimization...")

#     for it in range(1 - n_warmup, max_iters + 1):
#         # Reset world snapshot but place robot at HOME
#         controller.hard_reset_from_home(redraw=False)

#         # Warmup: Predefined trajectories and weight bootstrapping
#         if it < 0:
#             if (it - 1) % 5 == 0:
#                 trajectory = generate_warmup_trajectory(n_counter, config)
#                 if trajectory is not None:
#                     dmp.imitate_path(trajectory.T, plot=False)
#                     write_weights_csv(weights_csv_path, dmp.w.copy())
#                     n_counter += 1

#         # Load weights for current iteration
#         try:
#             w2 = read_weights_csv(weights_csv_path, n_bfs)
#         except Exception as e:
#             print(f"Error loading weights at iter {it}: {e}")
#             continue

#         print(f"Iteration {it}: Executing Policy")
#         dmp.w = w2.copy()
#         dmp.reset_state()
#         append_weight_history(config['logs']['weight_history_csv'], it, "executed", w2.copy(), n_bfs)

#         # Physics Simulation Loop
#         model, data = controller.model, controller.data
#         joint_names = controller.joint_names
#         start_joints = get_joint_positions(model, data, joint_names)

#         joint_traj = []
#         dmp_task_trajectory = []
#         keep_every = max(1, int(config['dmp_params']['deci_build']))

#         for i in range(int(dmp.timesteps)):
#             # Step DMP with aggressive obstacle avoidance gains
#             y = get_dmp_step_with_obstacles(dmp)
#             target_3d = np.array([y[0], y[1], config['robot']['mop_z_height']], dtype=float)
#             dmp_task_trajectory.append(target_3d)

#             # High-speed IK solver settings for optimization runs
#             ok, err_val = enhanced_ik_solver(
#                 model, data, controller.site_id, target_3d, joint_names,
#                 max_iters_per_wp=50, print_every=1000
#             )

#             if not ok:
#                 save_ik_error(it, i, target_3d, err_val or float("nan"), config['logs']['ik_error_csv'])
#                 continue

#             if i % keep_every == 0:
#                 joint_traj.append(get_joint_positions(model, data, joint_names).copy())

#         # Execute joint movements if successful
#         if joint_traj:
#             set_joint_positions(model, data, joint_names, start_joints)
#             controller.execute_joint_trajectory(joint_traj, dt=controller.dt * 2)

#         # Physics Settlement: Extra steps to let balls stop rolling before count
#         mujoco.mj_step(model, data)
#         mujoco.mj_forward(model, data)

#         # Data Persistence
#         save_trajectory_data(it, dmp_task_trajectory, config['logs']['dmp_trajectory_csv'])
#         save_trajectory_data(it, controller.ee_trajectory, config['logs']['ee_trajectory_csv'])

#         # Spatial Ball Counting
#         grid = controller.count_balls_in_grid()
#         total_balls = int(np.sum(grid))
#         log_iteration_data(it, grid, total_balls, len(joint_traj), config['logs']['iter_log_csv'])

#         # LLM Feedback Construction
#         iter_log_data = load_iteration_log(config['logs']['iter_log_csv'], config['dmp_params']['num_x_segments'], config['dmp_params']['num_y_segments'])
#         # CRITICAL: Use Actual EE Trajectory for Bounds Analysis
#         traj_feedback_data = load_traj_feedback(config['logs']['ee_trajectory_csv'])
#         ee_traj_df = pd.read_csv(config['logs']['ee_trajectory_csv']) if os.path.exists(config['logs']['ee_trajectory_csv']) else None

#         # Logic to decide next weights
#         if it < 0:
#             # Random exploration during warmup period
#             np.random.seed(config['llm_settings']['seed_number'] + it)
#             w_next = w2 + np.random.randn(2, n_bfs) * config['dmp_params']['random_scale']
#         else:
#             # Build detailed prompt with coordinate tables and grid markdown
#             feedback_text, guidance_text = build_llm_feedback(
#                 it + 1, pd.read_csv(config['logs']['weight_history_csv']),
#                 iter_log_data, traj_feedback_data, ee_traj_df, config, bounds
#             )

#             prompt = llm.render_prompt(it + 1, feedback_text, bounds, guidance_text=guidance_text)
#             # save_dialog(config['logs']['dialog_dir'], it + 1, prompt, "")

#             try:
#                 # Use large token limit for coordinate tables
#                 response = llm.call_ollama(prompt, token_limit=118000)
#                 w_next = parse_ollama_weights(response, n_bfs)
#                 save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
#             except Exception as e:
#                 print(f"LLM Error at iteration {it}: {e}. Reusing current weights.")
#                 w_next = w2.copy()

#         # Update for next iteration
#         append_weight_history(config['logs']['weight_history_csv'], it + 1, "proposed", w_next, n_bfs)
#         write_weights_csv(weights_csv_path, w_next)

#     controller.viewer.close()
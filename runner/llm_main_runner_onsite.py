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
# This class manages the cycle of: Ask AI -> Try Move -> Get Score -> Repeat
class LLM_Brain:
    '''Class that runs the llm optimization loop'''
    def __init__(self, config):
        self.config = config
        self.llm_interface = LLMInterface(config)
        # Create folders to store all the data and AI conversations
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
        # Set up the grid system used to count objects on the table
        self.num_x_segments = config['dmp_params']['num_x_segments']
        self.num_y_segments = config['dmp_params']['num_y_segments']
        self.grid_count = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)

        self.bounds = {
            "xmin": self.x_min, "xmax": self.x_max,
            "ymin": self.y_min, "ymax": self.y_max,
        }
        # Initialize the movement system and time settings
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
        # Gather all the history: how it moved, where it went, and how many balls it cleared
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
        # Extract the numbers from the AI's response and save the conversation
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

        x_traj = dmp_traj_data.filter(like='x').to_numpy().ravel().tolist()
        y_traj = dmp_traj_data.filter(like='y').to_numpy().ravel().tolist()
        traj = []
        for k in range(len(x_traj)):
            traj.append([x_traj[k], y_traj[k], -0.108])
        # Save this simplified path to a special "pickle" file
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
    

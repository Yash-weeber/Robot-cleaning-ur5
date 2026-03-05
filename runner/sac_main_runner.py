# #!/usr/bin/env python3
# """
# sac_main_runner.py
#
# What this runner does:
# - Uses SAC to output DMP weights (action) that generate a 2D rhythmic trajectory.
# - Executes the trajectory in MuJoCo using your EnhancedDMPController + IK solver.
# - Computes reward based on how many balls were cleared (minimize balls_remaining).
# - Logs iteration data, EE trajectory, weight history, and IK errors.
#
# IMPORTANT CHANGE (requested):
# - State space is now the end-effector trajectory (fixed-length) instead of ball positions.
# - Action space is unchanged: action = flattened DMP weights of shape (2 * n_bfs).
#
# Episode structure (requested):
# - 10 episodes x 200 iterations = 2000 total training iterations (by default).
# - Warmup iterations run first (negative iteration indices up to 0), same as before.
#
# State definition (NEW):
# - state_t is a vector of length (2 * traj_state_len)
# - It is the PREVIOUS iteration's executed end-effector XY trajectory, resampled to traj_state_len points,
#   then normalized using workspace center and half-dimensions.
#
# Why "previous trajectory"?
# - In your loop, you only know the executed EE trajectory after you run the action.
# - So we use:
#     current_state  = previous_executed_trajectory_state
#     next_state     = current_executed_trajectory_state
#   This makes transitions well-defined without changing the action definition.
#
# You can switch to using the *planned* DMP trajectory instead of executed EE trajectory if you want,
# but I kept it tied to what actually happened in sim because it is typically more informative.
# """
#
# import os
# import torch
# import torch.nn as nn
# import torch.optim as optim
# import numpy as np
# import mujoco
# from torch.distributions import Normal
#
# # Internal imports
# from runner.main_runner import EnhancedDMPController
# from env.llm_robot_logic import generate_warmup_trajectory, log_iteration_data
# from env.robot_logic import get_joint_positions, enhanced_ik_solver
# from agent.dmp_logic import DMPs_rhythmic
# from utils.obstacle_avoidance import avoid_obstacles
# from agent.llm_data_utils import save_trajectory_data, append_weight_history, save_ik_error
#
#
# # -------------------------
# # Helper: Trajectory -> Fixed-Length State Vector
# # -------------------------
# def resample_2d_trajectory(traj_xy: np.ndarray, n_points: int) -> np.ndarray:
#     """
#     Resample a 2D trajectory to exactly n_points using linear interpolation.
#
#     Args:
#         traj_xy: np.ndarray of shape (T, 2) containing XY points.
#         n_points: desired number of points.
#
#     Returns:
#         np.ndarray of shape (n_points, 2)
#     """
#     if traj_xy is None or len(traj_xy) == 0:
#         return np.zeros((n_points, 2), dtype=np.float32)
#
#     traj_xy = np.asarray(traj_xy, dtype=np.float32)
#     if traj_xy.ndim != 2 or traj_xy.shape[1] != 2:
#         # If trajectory is not (T,2), fallback safely
#         return np.zeros((n_points, 2), dtype=np.float32)
#
#     T = traj_xy.shape[0]
#     if T == 1:
#         return np.repeat(traj_xy, n_points, axis=0)
#
#     # Original sample positions in [0, 1]
#     x_old = np.linspace(0.0, 1.0, T, dtype=np.float32)
#     x_new = np.linspace(0.0, 1.0, n_points, dtype=np.float32)
#
#     # Interpolate each dimension
#     x_interp = np.interp(x_new, x_old, traj_xy[:, 0]).astype(np.float32)
#     y_interp = np.interp(x_new, x_old, traj_xy[:, 1]).astype(np.float32)
#
#     return np.stack([x_interp, y_interp], axis=1)
#
#
# def normalize_xy(traj_xy: np.ndarray, ws_center, ws_width, ws_length) -> np.ndarray:
#     """
#     Normalize XY points using workspace center and half-dimensions.
#     This roughly maps workspace bounds to about [-1, 1] if motion stays in bounds.
#
#     Args:
#         traj_xy: (N,2)
#         ws_center: [cx, cy]
#         ws_width: width in X direction
#         ws_length: length in Y direction
#
#     Returns:
#         normalized traj (N,2)
#     """
#     cx, cy = float(ws_center[0]), float(ws_center[1])
#     half_w = max(float(ws_width) / 2.0, 1e-6)
#     half_l = max(float(ws_length) / 2.0, 1e-6)
#
#     out = np.empty_like(traj_xy, dtype=np.float32)
#     out[:, 0] = (traj_xy[:, 0] - cx) / half_w
#     out[:, 1] = (traj_xy[:, 1] - cy) / half_l
#     return out
#
#
# def build_traj_state_from_controller(controller, traj_state_len, ws_center, ws_width, ws_length) -> np.ndarray:
#     """
#     Build a fixed-length state vector from controller.ee_trajectory.
#
#     controller.ee_trajectory is assumed to be a list/array of 3D points [x,y,z]
#     or at least contains x,y in the first two entries.
#
#     Returns:
#         state_np: shape (2 * traj_state_len,), dtype float32
#     """
#     traj = getattr(controller, "ee_trajectory", None)
#     if traj is None or len(traj) == 0:
#         traj_xy = np.zeros((0, 2), dtype=np.float32)
#     else:
#         traj = np.asarray(traj, dtype=np.float32)
#         # If it is (T,3) or (T,>=2), keep XY
#         if traj.ndim == 2 and traj.shape[1] >= 2:
#             traj_xy = traj[:, :2]
#         else:
#             traj_xy = np.zeros((0, 2), dtype=np.float32)
#
#     traj_xy_rs = resample_2d_trajectory(traj_xy, traj_state_len)
#     traj_xy_norm = normalize_xy(traj_xy_rs, ws_center, ws_width, ws_length)
#
#     # Flatten (N,2) -> (2N,)
#     return traj_xy_norm.reshape(-1).astype(np.float32)
#
#
# # -------------------------
# # SAC Network Components
# # -------------------------
# class SACActor(nn.Module):
#     """
#     Actor maps state -> action distribution.
#     Action is still DMP weights (flattened), unchanged.
#
#     Note:
#     - This uses a Normal distribution around mu with learned std.
#     - mu is tanh-bounded and scaled, but sampled action can still exceed bounds
#       due to Gaussian sampling. This is your original behavior, kept unchanged.
#     """
#     def __init__(self, input_dim, action_dim, hidden_dim=512):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(input_dim, hidden_dim), nn.ReLU(),
#             nn.Linear(hidden_dim, hidden_dim), nn.ReLU()
#         )
#         self.mu = nn.Linear(hidden_dim, action_dim)
#         self.log_std = nn.Linear(hidden_dim, action_dim)
#
#     def forward(self, state):
#         x = self.net(state)
#         mu = torch.tanh(self.mu(x)) * 15.0  # bounded target range for DMP weights (kept same)
#         log_std = torch.clamp(self.log_std(x), -20, 2)
#         std = log_std.exp()
#         dist = Normal(mu, std)
#         action = dist.rsample()
#         logp = dist.log_prob(action).sum(-1, keepdim=True)
#         return action, logp, mu
#
#
# class SACCritic(nn.Module):
#     def __init__(self, input_dim, action_dim, hidden_dim=512):
#         super().__init__()
#
#         def q_net():
#             return nn.Sequential(
#                 nn.Linear(input_dim + action_dim, hidden_dim), nn.ReLU(),
#                 nn.Linear(hidden_dim, hidden_dim), nn.ReLU(),
#                 nn.Linear(hidden_dim, 1)
#             )
#
#         self.q1, self.q2 = q_net(), q_net()
#
#     def forward(self, state, action):
#         sa = torch.cat([state, action], dim=-1)
#         return self.q1(sa), self.q2(sa)
#
#
# class ReplayBuffer:
#     """
#     Replay buffer now stores (s, a, r, s2, done).
#     This is required once state is a proper trajectory transition.
#     Action space is unchanged.
#     """
#     def __init__(self, capacity, state_dim, action_dim):
#         self.capacity = int(capacity)
#         self.ptr = 0
#         self.size = 0
#
#         self.states = np.zeros((self.capacity, state_dim), dtype=np.float32)
#         self.actions = np.zeros((self.capacity, action_dim), dtype=np.float32)
#         self.rewards = np.zeros((self.capacity, 1), dtype=np.float32)
#         self.next_states = np.zeros((self.capacity, state_dim), dtype=np.float32)
#         self.dones = np.zeros((self.capacity, 1), dtype=np.float32)
#
#     def push(self, s, a, r, s2, done):
#         self.states[self.ptr] = s
#         self.actions[self.ptr] = a
#         self.rewards[self.ptr] = r
#         self.next_states[self.ptr] = s2
#         self.dones[self.ptr] = done
#
#         self.ptr = (self.ptr + 1) % self.capacity
#         self.size = min(self.size + 1, self.capacity)
#
#     def sample(self, batch_size):
#         bs = int(batch_size)
#         idx = np.random.randint(0, self.size, size=bs)
#         return (
#             torch.from_numpy(self.states[idx]),
#             torch.from_numpy(self.actions[idx]),
#             torch.from_numpy(self.rewards[idx]),
#             torch.from_numpy(self.next_states[idx]),
#             torch.from_numpy(self.dones[idx]),
#         )
#
#
# # -------------------------
# # SAC Main Runner
# # -------------------------
# def run_sac_optimization(config):
#     """
#     Main SAC loop.
#
#     Requested modifications:
#     - State is EE trajectory vector (2 * traj_state_len)
#     - Episodes = 10, iterations per episode = 200
#     - Action remains DMP weights (2 * n_bfs)
#     """
#     device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
#     controller = EnhancedDMPController(config)
#
#     # ---------- Action dims (UNCHANGED) ----------
#     n_balls = int(config["dmp_params"]["num_balls"])   # 500 in your case
#     n_bfs = int(config["dmp_params"]["n_bfs"])
#     action_dim = 2 * n_bfs
#
#     # ---------- State dims (CHANGED) ----------
#     # You can tune traj_state_len in config under sac_params if you want.
#     # Example: sac_params: { traj_state_len: 100 }
#     traj_state_len = int(config["sac_params"].get("traj_state_len", 100))
#     state_dim = 2 * traj_state_len
#
#     # Workspace normalization values
#     ws_center = config["simulation"]["ws_center"]
#     ws_width = float(config["simulation"]["ws_width"])
#     ws_length = float(config["simulation"]["ws_length"])
#
#     # ---------- Episode structure (requested) ----------
#     num_episodes = int(config["sac_params"].get("num_episodes", 10))
#     iters_per_episode = int(config["sac_params"].get("iters_per_episode", 200))
#     total_train_iters = num_episodes * iters_per_episode
#
#     # ---------- Networks ----------
#     actor = SACActor(state_dim, action_dim).to(device)
#     critic = SACCritic(state_dim, action_dim).to(device)
#     critic_target = SACCritic(state_dim, action_dim).to(device)
#     critic_target.load_state_dict(critic.state_dict())
#
#     # ---------- Entropy tuning ----------
#     target_entropy = -action_dim
#     log_alpha = torch.zeros(1, requires_grad=True, device=device)
#     alpha_opt = optim.Adam([log_alpha], lr=config["sac_params"]["lr_alpha"])
#
#     a_opt = optim.Adam(actor.parameters(), lr=config["sac_params"]["lr_actor"])
#     c_opt = optim.Adam(critic.parameters(), lr=config["sac_params"]["lr_critic"])
#
#     # ---------- Replay ----------
#     buffer = ReplayBuffer(
#         config["sac_params"]["buffer_size"],
#         state_dim,
#         action_dim
#     )
#
#     # ---------- DMP ----------
#     dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)
#
#     # Warmup count (kept as your structure)
#     n_warmup = int(config["llm_settings"]["n_warmup"])
#
#     print("\n============================================================")
#     print("SAC RUN CONFIG")
#     print(f"  Episodes:              {num_episodes}")
#     print(f"  Iters per episode:     {iters_per_episode}")
#     print(f"  Total train iters:     {total_train_iters}")
#     print(f"  Warmup iters:          {n_warmup}  (iterations -{n_warmup-1}..0)")
#     print(f"  State dim (trajectory):{state_dim}  (traj_state_len={traj_state_len})")
#     print(f"  Action dim (weights):  {action_dim}  (n_bfs={n_bfs})")
#     print("============================================================\n")
#
#     # This holds the previous iteration's executed trajectory state.
#     # At the start, we have no prior trajectory, so it is zeros.
#     prev_state_np = np.zeros((state_dim,), dtype=np.float32)
#
#     print(f"[PHASE 1] Starting Expert Warmup (-{n_warmup-1} to 0)")
#
#     # Global iteration index includes warmup (negative..0) then training (1..total_train_iters)
#     for it in range(1 - n_warmup, total_train_iters + 1):
#         if it == 1:
#             print(f"\n[PHASE 2] Starting SAC Exploration (1 to {total_train_iters})")
#
#         # Episode indexing only applies for training iters (it > 0)
#         if it > 0:
#             ep_idx = (it - 1) // iters_per_episode
#             step_in_ep = (it - 1) % iters_per_episode
#         else:
#             ep_idx = -1
#             step_in_ep = -1
#
#         # If a new episode starts, reset the "previous trajectory state"
#         # so each episode begins from a clean state representation.
#         if it > 0 and step_in_ep == 0:
#             prev_state_np = np.zeros((state_dim,), dtype=np.float32)
#
#         # Always reset robot from home for each iteration (your original behavior)
#         controller.hard_reset_from_home(redraw=False)
#
#         # -----------------------
#         # State (NEW: previous EE trajectory state)
#         # -----------------------
#         state_np = prev_state_np.copy()
#         state_t = torch.tensor(state_np, dtype=torch.float32, device=device).unsqueeze(0)
#
#         # -----------------------
#         # Action (UNCHANGED)
#         # -----------------------
#         if it <= 0:
#             # expert warmup: imitate a predefined warmup trajectory
#             traj = generate_warmup_trajectory((it + n_warmup - 1) % 4)
#             dmp.imitate_path(traj.T, plot=False)
#             action_np = dmp.w.flatten()
#             tag = "warmup_expert"
#
#             # supervised pretrain actor mu to match expert action
#             target_a_t = torch.tensor(action_np, dtype=torch.float32, device=device).unsqueeze(0)
#             for _ in range(5):
#                 a_opt.zero_grad()
#                 _, _, mu = actor(state_t)
#                 loss = nn.MSELoss()(mu, target_a_t)
#                 loss.backward()
#                 a_opt.step()
#         else:
#             # SAC exploration: sample action from actor
#             with torch.no_grad():
#                 action_t, _, _ = actor(state_t)
#                 action_np = action_t.squeeze(0).cpu().numpy()
#             dmp.w = action_np.reshape(2, n_bfs)
#             tag = f"sac_exploration_ep{ep_idx+1}_step{step_in_ep+1}"
#
#         # -----------------------
#         # Simulate trajectory (same as your original)
#         # -----------------------
#         dmp.reset_state()
#         joint_traj = []
#         last_err = 0.0
#         last_target_3d = np.zeros(3, dtype=float)
#
#         for step_i in range(int(dmp.timesteps)):
#             f = avoid_obstacles(
#                 dmp.y, dmp.dy, dmp.goal,
#                 rect_d0_x=config["obstacle_params"]["rect_d0"],
#                 rect_eta=config["obstacle_params"]["rect_eta"],
#                 max_force=config["obstacle_params"]["max_force"],
#             )
#
#             y, _, _ = dmp.step(tau=2.0, external_force=f)
#             last_target_3d = np.array([y[0], y[1], config["robot"]["mop_z_height"]], dtype=float)
#
#             ok, err = enhanced_ik_solver(
#                 controller.model,
#                 controller.data,
#                 controller.site_id,
#                 last_target_3d,
#                 controller.joint_names,
#             )
#             last_err = err
#
#             if ok:
#                 joint_traj.append(
#                     get_joint_positions(controller.model, controller.data, controller.joint_names).copy()
#                 )
#             elif step_i % 100 == 0:
#                 save_ik_error(it, step_i, last_target_3d, last_err, config["logs"]["ik_error_csv"])
#
#         if joint_traj:
#             controller.execute_joint_trajectory(joint_traj)
#
#         # -----------------------
#         # Balls + Reward (MINIMIZE balls_remaining)
#         # -----------------------
#         grid = controller.count_balls_in_grid()
#         balls_remaining = int(np.sum(grid))
#
#         # Reward scaled to [0,1]: best is 1.0 when balls_remaining = 0
#         reward = (n_balls - balls_remaining) / float(n_balls)
#
#         # -----------------------
#         # Next State (NEW: current executed EE trajectory)
#         # -----------------------
#         next_state_np = build_traj_state_from_controller(
#             controller,
#             traj_state_len=traj_state_len,
#             ws_center=ws_center,
#             ws_width=ws_width,
#             ws_length=ws_length
#         )
#
#         # Episode done flag
#         # Only meaningful for training iters (it > 0)
#         done = 0.0
#         if it > 0 and step_in_ep == (iters_per_episode - 1):
#             done = 1.0
#
#         # Push transition (s, a, r, s2, done)
#         buffer.push(state_np, action_np, reward, next_state_np, done)
#
#         # For the next iteration, "previous state" becomes this trajectory state
#         prev_state_np = next_state_np.copy()
#
#         # -----------------------
#         # Logging (same as your original)
#         # -----------------------
#         log_iteration_data(it, grid, balls_remaining, len(joint_traj), config["logs"]["iter_log_csv"])
#         save_trajectory_data(it, controller.ee_trajectory, config["logs"]["ee_trajectory_csv"])
#         append_weight_history(config["logs"]["weight_history_csv"], it, tag, dmp.w, n_bfs)
#         save_ik_error(it, int(dmp.timesteps), last_target_3d, last_err, config["logs"]["ik_error_csv"])
#
#         # -----------------------
#         # Off-policy updates (SAC)
#         # -----------------------
#         if buffer.size > int(config["sac_params"]["batch_size"]) and it > 0:
#             updates_per_iter = int(config["sac_params"].get("updates_per_iter", 5))
#             for _ in range(updates_per_iter):
#                 s, a, r, s2, d = buffer.sample(config["sac_params"]["batch_size"])
#                 s, a, r, s2, d = s.to(device), a.to(device), r.to(device), s2.to(device), d.to(device)
#
#                 curr_alpha = log_alpha.exp()
#
#                 # Critic target: r + gamma * (1-done) * (min(Q) - alpha * logp)
#                 with torch.no_grad():
#                     next_a, next_lp, _ = actor(s2)
#                     q1_t, q2_t = critic_target(s2, next_a)
#                     min_q_t = torch.min(q1_t, q2_t)
#                     target_q = r + config["sac_params"]["gamma"] * (1.0 - d) * (min_q_t - curr_alpha * next_lp)
#
#                 # Critic update
#                 q1, q2 = critic(s, a)
#                 c_loss = nn.MSELoss()(q1, target_q) + nn.MSELoss()(q2, target_q)
#                 c_opt.zero_grad()
#                 c_loss.backward()
#                 c_opt.step()
#
#                 # Actor update: maximize Q - alpha * logp  <=> minimize (alpha*logp - Q)
#                 new_a, lp, _ = actor(s)
#                 q1_new, q2_new = critic(s, new_a)
#                 min_q_new = torch.min(q1_new, q2_new)
#                 a_loss = (curr_alpha * lp - min_q_new).mean()
#                 a_opt.zero_grad()
#                 a_loss.backward()
#                 a_opt.step()
#
#                 # Alpha update (entropy temperature)
#                 alpha_loss = -(log_alpha * (lp + target_entropy).detach()).mean()
#                 alpha_opt.zero_grad()
#                 alpha_loss.backward()
#                 alpha_opt.step()
#
#                 # Soft update target critic
#                 tau = config["sac_params"]["tau"]
#                 for p, tp in zip(critic.parameters(), critic_target.parameters()):
#                     tp.data.copy_(tau * p.data + (1.0 - tau) * tp.data)
#
#         balls_cleared = n_balls - balls_remaining
#
#         # Print progress
#         if it <= 0:
#             print(
#                 f"Iter {it:4d} | Warmup | Remaining: {balls_remaining:4d} | Cleared: {balls_cleared:4d} | "
#                 f"Reward: {reward:.3f} | Alpha: {log_alpha.exp().item():.4f}"
#             )
#         else:
#             print(
#                 f"Iter {it:4d} | Ep {ep_idx+1:2d}/{num_episodes} Step {step_in_ep+1:3d}/{iters_per_episode} | "
#                 f"Remaining: {balls_remaining:4d} | Cleared: {balls_cleared:4d} | "
#                 f"Reward: {reward:.3f} | Alpha: {log_alpha.exp().item():.4f}"
#             )
#
#     # Close viewer if it exists
#     if hasattr(controller, "viewer") and controller.viewer is not None:
#         controller.viewer.close()







#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# !/usr/bin/env python3
"""
sac_main_runner.py

=============================================================
OVERVIEW
=============================================================
Pure SAC (Soft Actor-Critic) to drive DMP weights that move
a robot end-effector to sweep all 500 balls off the table.

GOAL: balls_remaining → 0   |   Reward → 500 (maximise pure reward)

=============================================================
STATE DESIGN (PRESERVED from 4:20 PM)
=============================================================
State at iteration t  =  [ ee_traj_sub_t  |  delta_w_t ]

  ee_traj_sub_t  :  Previous executed end-effector XY trajectory,
                    subsampled at every 2nd point (NO interpolation),
                    then padded/truncated to 'traj_state_len' points,
                    then workspace-normalised → shape (2*traj_state_len,)

  delta_w_t      :  w_t-1 - w_t-2  (change in DMP weights between the
                    last two iterations) → shape (2*n_bfs,)

=============================================================
EPISODE / RESET STRUCTURE (NEW NESTED LOGIC)
=============================================================
  - 1000 iterations x 2 episodes = 2000 total training tries.
  - Updates happen AFTER the 2 episodes are collected.
  - Warmup runs before iteration 1 (negative indices).

=============================================================
ENTROPY COLLAPSE FIXES (PRESERVED from 4:20 PM)
=============================================================
  FIX 1 — ALPHA FLOOR  (ALPHA_MIN = 0.1)
  FIX 2 — DELAYED UPDATES  (UPDATE_START_MULTIPLIER = 1)
  FIX 3 — RAMPED updates_per_iter (based on fill_ratio)
  FIX 4 — BETTER DEFAULT target_entropy (-n_bfs)
"""

import os
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import mujoco
from torch.distributions import Normal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# Internal project imports
from runner.main_runner import EnhancedDMPController
from env.llm_robot_logic import generate_warmup_trajectory, log_iteration_data
from env.robot_logic import get_joint_positions, enhanced_ik_solver
from agent.dmp_logic import DMPs_rhythmic
from utils.obstacle_avoidance import avoid_obstacles
from agent.llm_data_utils import (
    save_trajectory_data,
    append_weight_history,
    save_ik_error,
)

# -------------------------
# =============================================================
# ENTROPY COLLAPSE FIX — GLOBAL CONSTANTS
# =============================================================
ALPHA_MIN = 0.05
UPDATE_START_MULTIPLIER = 1
ACTION_SCALE = 300.0


# =============================================================
# TRAJECTORY SUBSAMPLING  (no interpolation)
# =============================================================
def subsample_2d_trajectory(traj_xy: np.ndarray, n_points: int) -> np.ndarray:
    if traj_xy is None or len(traj_xy) == 0:
        return np.zeros((n_points, 2), dtype=np.float32)

    traj_xy = np.asarray(traj_xy, dtype=np.float32)

    if traj_xy.ndim != 2 or traj_xy.shape[1] < 2:
        return np.zeros((n_points, 2), dtype=np.float32)

    traj_xy = traj_xy[:, :2]
    subsampled = traj_xy[::10]
    T_sub = subsampled.shape[0]

    if T_sub == 0:
        return np.zeros((n_points, 2), dtype=np.float32)

    if T_sub >= n_points:
        return subsampled[:n_points].astype(np.float32)
    else:
        pad_rows = n_points - T_sub
        last_pt = subsampled[-1:, :]
        padding = np.repeat(last_pt, pad_rows, axis=0)
        return np.vstack([subsampled, padding]).astype(np.float32)


def normalize_xy(traj_xy: np.ndarray, ws_center, ws_width, ws_length) -> np.ndarray:
    cx = float(ws_center[0])
    cy = float(ws_center[1])
    half_w = max(float(ws_width) / 2.0, 1e-6)
    half_l = max(float(ws_length) / 2.0, 1e-6)

    out = np.empty_like(traj_xy, dtype=np.float32)
    out[:, 0] = (traj_xy[:, 0] - cx) / half_w
    out[:, 1] = (traj_xy[:, 1] - cy) / half_l
    return out


def build_traj_state(controller, traj_state_len, ws_center, ws_width, ws_length) -> np.ndarray:
    raw_traj = getattr(controller, "ee_trajectory", None)

    if raw_traj is None or len(raw_traj) == 0:
        return np.zeros((2 * traj_state_len,), dtype=np.float32)

    raw_traj = np.asarray(raw_traj, dtype=np.float32)

    if raw_traj.ndim == 2 and raw_traj.shape[1] >= 2:
        traj_xy = raw_traj[:, :2]
    else:
        return np.zeros((2 * traj_state_len,), dtype=np.float32)

    traj_sub = subsample_2d_trajectory(traj_xy, traj_state_len)
    traj_norm = normalize_xy(traj_sub, ws_center, ws_width, ws_length)

    return traj_norm.reshape(-1).astype(np.float32)


# =============================================================
# STATE ASSEMBLY
# =============================================================
def build_full_state(traj_state_vec: np.ndarray, delta_w: np.ndarray) -> np.ndarray:
    return np.concatenate([traj_state_vec, delta_w], axis=0).astype(np.float32)


# =============================================================
# SAC NETWORK COMPONENTS
# =============================================================
class SACActor(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 512):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim), nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim), nn.ReLU(),
        )
        self.mu_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)


    def forward(self, state: torch.Tensor):
        x = self.net(state)
        mu_raw = self.mu_head(x)
        log_std = torch.clamp(self.log_std_head(x), -2, 2)
        std = log_std.exp()

        dist = Normal(mu_raw, std)
        u = dist.rsample()
        action = torch.tanh(u)

        log_prob = dist.log_prob(u) - torch.log(
            1.0 - action.pow(2) + 1e-6
        )
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        mu = torch.tanh(mu_raw)
        return action, log_prob, mu


class SACCritic(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 512):
        super().__init__()

        def q_net():
            return nn.Sequential(
                nn.Linear(state_dim + action_dim, hidden_dim), nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim), nn.ReLU(),
                nn.Linear(hidden_dim, 1),
            )

        self.q1 = q_net()
        self.q2 = q_net()

    def forward(self, state: torch.Tensor, action: torch.Tensor):
        sa = torch.cat([state, action], dim=-1)
        return self.q1(sa), self.q2(sa)


class ReplayBuffer:
    def __init__(self, capacity: int, state_dim: int, action_dim: int):
        cap = int(capacity)

        self.states = np.zeros((cap, state_dim), dtype=np.float32)
        self.actions = np.zeros((cap, action_dim), dtype=np.float32)
        self.rewards = np.zeros((cap, 1), dtype=np.float32)
        self.next_states = np.zeros((cap, state_dim), dtype=np.float32)
        self.dones = np.zeros((cap, 1), dtype=np.float32)

        self.capacity = cap
        self.ptr = 0
        self.size = 0

    def push(self, s, a, r, s2, done):
        self.states[self.ptr] = s
        self.actions[self.ptr] = a
        self.rewards[self.ptr] = r
        self.next_states[self.ptr] = s2
        self.dones[self.ptr] = done

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        idx = np.random.randint(0, self.size, size=int(batch_size))
        return (
            torch.from_numpy(self.states[idx]).to(torch.float32),
            torch.from_numpy(self.actions[idx]).to(torch.float32),
            torch.from_numpy(self.rewards[idx]).to(torch.float32),
            torch.from_numpy(self.next_states[idx]).to(torch.float32),
            torch.from_numpy(self.dones[idx]).to(torch.float32),
        )


# =============================================================
# MAIN SAC OPTIMISATION LOOP
# =============================================================
def run_sac_optimization(config):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[SAC] Using device: {device}")

    controller = EnhancedDMPController(config)

    # ----------------------------------------------------------
    # DIMENSIONS
    # ----------------------------------------------------------
    n_balls = int(config["dmp_params"]["num_balls"])
    n_bfs = int(config["dmp_params"].get("n_bfs", 10))
    action_dim = 2 * n_bfs

    traj_state_len = int(config["sac_params"].get("traj_state_len", 100))
    state_dim = 2 * traj_state_len + 2 * n_bfs

    ws_center = config["simulation"]["ws_center"]
    ws_width = float(config["simulation"]["ws_width"])
    ws_length = float(config["simulation"]["ws_length"])

    # ----------------------------------------------------------
    # EPISODE STRUCTURE (NEW NESTED LOGIC)
    # ----------------------------------------------------------
    num_iterations = int(config["sac_params"].get("num_iterations", 1000))
    episodes_per_iteration = int(config["sac_params"].get("episodes_per_iteration", 2))
    max_updates_per_iter = int(config["sac_params"].get("updates_per_iteration", 2))

    total_train_episodes = num_iterations * episodes_per_iteration
    n_warmup = int(config["llm_settings"]["n_warmup"])

    # ----------------------------------------------------------
    # NETWORKS
    # ----------------------------------------------------------
    hidden_dim = int(config["sac_params"].get("hidden_dim", 256))
    actor = SACActor(state_dim, action_dim, hidden_dim).to(device)
    critic = SACCritic(state_dim, action_dim, hidden_dim).to(device)
    critic_target = SACCritic(state_dim, action_dim, hidden_dim).to(device)
    critic_target.load_state_dict(critic.state_dict())

    a_opt = optim.Adam(actor.parameters(), lr=config["sac_params"]["lr_actor"])
    c_opt = optim.Adam(critic.parameters(), lr=config["sac_params"]["lr_critic"])

    # ----------------------------------------------------------
    # ENTROPY TEMPERATURE
    # ----------------------------------------------------------
    target_entropy = float(config["sac_params"].get("target_entropy", -n_bfs))
    init_alpha = float(config["sac_params"].get("alpha", 0.2))
    log_alpha = torch.tensor(
        [np.log(max(init_alpha, ALPHA_MIN))],
        dtype=torch.float32,
        requires_grad=True,
        device=device,
    )
    alpha_opt = optim.Adam([log_alpha], lr=config["sac_params"]["lr_alpha"])

    # ----------------------------------------------------------
    # REPLAY BUFFER + UPDATE THRESHOLDS
    # ----------------------------------------------------------
    batch_size = int(config["sac_params"]["batch_size"])
    buffer = ReplayBuffer(
        capacity=int(config["sac_params"]["buffer_size"]),
        state_dim=state_dim,
        action_dim=action_dim,
    )
    update_start_threshold = batch_size * UPDATE_START_MULTIPLIER

    # ----------------------------------------------------------
    # DMP
    # ----------------------------------------------------------
    dmp = DMPs_rhythmic(n_dmps=2, n_bfs=n_bfs, dt=controller.dt)

    print("\n" + "=" * 70)
    print("SAC CONFIGURATION (NESTED LOGIC - PURE REWARD)")
    print(f"  n_bfs                : {n_bfs}  (action_dim = {action_dim})")
    print(f"  traj_state_len       : {traj_state_len}")
    print(f"  state_dim            : {state_dim}  (2x{traj_state_len} traj + 2x{n_bfs} dw)")
    print(f"  Iterations           : {num_iterations}")
    print(f"  Episodes/Iter        : {episodes_per_iteration}")
    print(f"  Total train episodes : {total_train_episodes}")
    print(f"  n_warmup             : {n_warmup}  (FIXED)")
    print(f"  --- ENTROPY COLLAPSE FIXES ---")
    print(f"  ALPHA_MIN            : {ALPHA_MIN}")
    print(f"  target_entropy       : {target_entropy}")
    print(f"  update_start_thresh  : {update_start_threshold} transitions")
    print(f"  max_updates_per_iter : {max_updates_per_iter} (ramped 1->2->max)")
    print("=" * 70 + "\n")

    # Global iteration tracker for seamless logging (-19 to 2000)
    global_it = 1 - n_warmup
    prev_traj_state_np = np.zeros((2 * traj_state_len,), dtype=np.float32)
    prev_weights_np = np.zeros((2 * n_bfs,), dtype=np.float32)

    # ==========================================
    # PHASE 1: WARMUP LOOP
    # ==========================================
    print(f"[PHASE 1] Expert Warmup  (iters -{n_warmup - 1} to 0)")

    while global_it <= 0:
        controller.hard_reset_from_home(redraw=True)


        state_np = build_full_state(prev_traj_state_np, prev_weights_np)
        state_t = torch.tensor(state_np, dtype=torch.float32, device=device).unsqueeze(0)

        warmup_idx = (global_it + n_warmup - 1) % 4
        traj_ref = generate_warmup_trajectory(warmup_idx,config)
        dmp.imitate_path(traj_ref.T, plot=False)
        env_action = dmp.w.flatten().astype(np.float32)

        # 2. SHRINK THEM to [-1, 1] so the neural network can actually learn them
        action_np = np.clip(env_action / ACTION_SCALE, -0.999, 0.999)
        tag = "warmup_expert"

        target_a_t = torch.tensor(action_np, dtype=torch.float32, device=device).unsqueeze(0)
        for _ in range(5):
            a_opt.zero_grad()
            _, _, mu = actor(state_t)
            imitation_loss = nn.MSELoss()(mu, target_a_t)
            imitation_loss.backward()
            a_opt.step()

        current_weights_np = action_np.copy()
        delta_w_np = current_weights_np - prev_weights_np

        dmp.reset_state()
        joint_traj = []
        last_err = 0.0
        last_target_3d = np.zeros(3, dtype=float)

        for step_i in range(int(dmp.timesteps)):
            f = avoid_obstacles(dmp.y, dmp.dy, dmp.goal, rect_d0_x=config["obstacle_params"]["rect_d0"],
                                rect_eta=config["obstacle_params"]["rect_eta"],
                                max_force=config["obstacle_params"]["max_force"])
            y, _, _ = dmp.step(tau=2.0, external_force=f)
            last_target_3d = np.array([y[0], y[1], config["robot"]["mop_z_height"]], dtype=float)

            ok, err = enhanced_ik_solver(controller.model, controller.data, controller.site_id, last_target_3d,
                                         controller.joint_names)
            last_err = err
            if ok:
                joint_traj.append(get_joint_positions(controller.model, controller.data, controller.joint_names).copy())
            elif step_i % 100 == 0:
                save_ik_error(global_it, step_i, last_target_3d, last_err, config["logs"]["ik_error_csv"])

        if joint_traj:
            controller.execute_joint_trajectory(joint_traj)




        grid = controller.count_balls_in_grid()
        balls_remaining = int(np.sum(grid))
        balls_cleared = n_balls - balls_remaining

        # PURE REWARD LOGIC (No scaling)
        reward = float(balls_cleared) / float(n_balls)


        current_traj_state_np = build_traj_state(controller, traj_state_len=traj_state_len, ws_center=ws_center,
                                                 ws_width=ws_width, ws_length=ws_length)
        next_state_np = build_full_state(current_traj_state_np, delta_w_np)

        buffer.push(state_np, action_np, reward, next_state_np, done=1.0)

        log_iteration_data(global_it, grid, balls_remaining, len(joint_traj), config["logs"]["iter_log_csv"])
        save_trajectory_data(global_it, controller.ee_trajectory, config["logs"]["ee_trajectory_csv"])

        # append_weight_history(config["logs"]["weight_history_csv"], global_it, tag, dmp.w, n_bfs)
        append_weight_history(config["logs"]["weight_history_csv"], global_it, tag, env_action.reshape(2, n_bfs), n_bfs)
        save_ik_error(global_it, int(dmp.timesteps), last_target_3d, last_err, config["logs"]["ik_error_csv"])
        # At end of warmup loop body, add:
        prev_weights_np = current_weights_np.copy()
        prev_traj_state_np = current_traj_state_np.copy()

        print(
            f"Iter {global_it:4d} | Warmup | Remaining: {balls_remaining:4d} | Cleared: {balls_cleared:4d} | Reward: {reward:.4f} | Buf: {buffer.size}/{update_start_threshold}")

        global_it += 1

    # ==========================================
    # PHASE 2: SAC NESTED TRAINING LOOP
    # ==========================================
    print(f"\n[PHASE 2] SAC Training  (iters 1 to {total_train_episodes})")
    log_folder = os.path.dirname(config["logs"]["weight_history_csv"])
    checkpoint_dir = os.path.join(log_folder, "checkpoints")
    os.makedirs(checkpoint_dir, exist_ok=True)

    reward_plot_path = os.path.join(log_folder, "reward_curve.png")
    episode_rewards = []
    best_reward_seen = -np.inf
    best_action_np = None
    for iteration in range(1, num_iterations + 1):
        for ep in range(episodes_per_iteration):

            # Start of a fresh DMP rollout
            controller.hard_reset_from_home(redraw=True)


            state_np = build_full_state(prev_traj_state_np, prev_weights_np)
            state_t = torch.tensor(state_np, dtype=torch.float32, device=device).unsqueeze(0)

            # SAC exploration
            with torch.no_grad():
                action_t, _, _ = actor(state_t)
                action_np = action_t.squeeze(0).cpu().numpy().astype(np.float32)


            env_action = action_np * ACTION_SCALE
            dmp.w = env_action.reshape(2, n_bfs)
            tag = f"sac_iter{iteration}_ep{ep + 1}"

            current_weights_np = action_np.copy()
            delta_w_np = current_weights_np - prev_weights_np

            # Simulate trajectory
            dmp.reset_state()
            joint_traj = []
            last_err = 0.0
            last_target_3d = np.zeros(3, dtype=float)

            for step_i in range(int(dmp.timesteps)):
                f = avoid_obstacles(dmp.y, dmp.dy, dmp.goal, rect_d0_x=config["obstacle_params"]["rect_d0"],
                                    rect_eta=config["obstacle_params"]["rect_eta"],
                                    max_force=config["obstacle_params"]["max_force"])
                y, _, _ = dmp.step(tau=2.0, external_force=f)
                last_target_3d = np.array([y[0], y[1], config["robot"]["mop_z_height"]], dtype=float)

                ok, err = enhanced_ik_solver(controller.model, controller.data, controller.site_id, last_target_3d,
                                             controller.joint_names)
                last_err = err
                if ok:
                    joint_traj.append(
                        get_joint_positions(controller.model, controller.data, controller.joint_names).copy())
                elif step_i % 100 == 0:
                    save_ik_error(global_it, step_i, last_target_3d, last_err, config["logs"]["ik_error_csv"])

            if joint_traj:
                controller.execute_joint_trajectory(joint_traj)

            # Reward mapping
            grid = controller.count_balls_in_grid()
            balls_remaining = int(np.sum(grid))
            balls_cleared = n_balls - balls_remaining

            # PURE REWARD LOGIC (No scaling)
            reward = float(balls_cleared) / float(n_balls)
            episode_rewards.append(reward)
            # Track the best pure learned action
            if reward > best_reward_seen:
                best_reward_seen = reward
                with torch.no_grad():
                    _, _, mu = actor(state_t)
                    best_action_np = mu.squeeze(0).cpu().numpy().copy()
                print(f"   >>> NEW BEST reward={reward:.3f} at iter {global_it}")

                # --- FIX: Actually save the best action to a file! ---
                best_action_file = os.path.join(log_folder, "best_action.npy")
                np.save(best_action_file, best_action_np * ACTION_SCALE)
                # -----------------------------------------------------

            current_traj_state_np = build_traj_state(controller, traj_state_len=traj_state_len, ws_center=ws_center,
                                                     ws_width=ws_width, ws_length=ws_length)
            next_state_np = build_full_state(current_traj_state_np, delta_w_np)

            # Push transition
            buffer.push(state_np, action_np, reward, next_state_np, done=1.0)
            prev_traj_state_np = current_traj_state_np.copy()
            prev_weights_np = current_weights_np.copy()


            # Logging
            log_iteration_data(global_it, grid, balls_remaining, len(joint_traj), config["logs"]["iter_log_csv"])
            save_trajectory_data(global_it, controller.ee_trajectory, config["logs"]["ee_trajectory_csv"])
            append_weight_history(config["logs"]["weight_history_csv"], global_it, tag, env_action.reshape(2, n_bfs), n_bfs)
            save_ik_error(global_it, int(dmp.timesteps), last_target_3d, last_err, config["logs"]["ik_error_csv"])

            alpha_val = log_alpha.exp().item()
            buf_pct = 100.0 * buffer.size / update_start_threshold

            print(
                f"Iter {global_it:4d} | Block {iteration:3d}/{num_iterations} Ep {ep + 1}/{episodes_per_iteration} | Remaining: {balls_remaining:4d} | Cleared: {balls_cleared:4d} | Reward: {reward:.4f} | alpha: {alpha_val:.4f} | Buf: {buffer.size}/{update_start_threshold} ({buf_pct:.0f}%)")

            global_it += 1

        # ==========================================
        # PHASE 3: SAC GRADIENT UPDATES
        # (Executes after `episodes_per_iteration` finishes)
        # ==========================================
        updates_ran = 0
        if buffer.size >= update_start_threshold:

            # Ramped update logic based on buffer fullness

            updates_this_iter = max_updates_per_iter

            updates_ran = updates_this_iter


            for _ in range(updates_this_iter):
                s, a, r, s2, d = buffer.sample(batch_size)
                s = s.to(device)
                a = a.to(device)
                r = r.to(device)
                s2 = s2.to(device)
                d = d.to(device)

                # Fix 1: Clamp alpha

                curr_alpha = log_alpha.exp()

                # Critic target
                with torch.no_grad():
                    next_a, next_lp, _ = actor(s2)
                    q1_t, q2_t = critic_target(s2, next_a)
                    min_q_t = torch.min(q1_t, q2_t)
                    target_q = r + config["sac_params"]["gamma"] * (1.0 - d) * (min_q_t - curr_alpha * next_lp)

                # Critic update
                q1, q2 = critic(s, a)
                c_loss = nn.MSELoss()(q1, target_q) + nn.MSELoss()(q2, target_q)
                c_opt.zero_grad()
                c_loss.backward()

                torch.nn.utils.clip_grad_norm_(critic.parameters(), max_norm=1.0)
                c_opt.step()

                # Actor update
                new_a, lp, _ = actor(s)
                q1_new, q2_new = critic(s, new_a)
                min_q_new = torch.min(q1_new, q2_new)
                a_loss = (curr_alpha.detach() * lp - min_q_new).mean()
                a_opt.zero_grad()
                a_loss.backward()

                torch.nn.utils.clip_grad_norm_(actor.parameters(), max_norm=1.0)
                a_opt.step()

                # Alpha update
                alpha_loss = -(log_alpha * (lp + target_entropy).detach()).mean()
                alpha_opt.zero_grad()
                alpha_loss.backward()
                alpha_opt.step()

                # Soft update target critic
                tau = float(config["sac_params"]["tau"])
                for p, tp in zip(critic.parameters(), critic_target.parameters()):
                    tp.data.copy_(tau * p.data + (1.0 - tau) * tp.data)
        # --- ADD THIS BLOCK TO SAVE THE PLOT EVERY ITERATION ---
        plt.figure(figsize=(10, 5))
        plt.plot(episode_rewards, color='blue', linewidth=2, label='SAC Episode Reward')
        plt.title(f'Live Training Reward (Iteration {iteration}/{num_iterations})')
        plt.xlabel('Total Episodes')
        plt.ylabel('Normalized Reward (0.0 to 1.0)')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.tight_layout()
        plt.savefig(reward_plot_path)
        plt.close()
        # -------------------------------------------------------

        if updates_ran > 0:
            print(f"   >>> Executed {updates_ran} SAC gradient updates after episode block.")
        if iteration % 50 == 0:
            torch.save(actor.state_dict(), os.path.join(checkpoint_dir, f"actor_iter_{iteration}.pth"))
            torch.save(critic.state_dict(), os.path.join(checkpoint_dir, f"critic_iter_{iteration}.pth"))
            print(f"   >>> Saved network checkpoints at iteration {iteration}")

    if hasattr(controller, "viewer") and controller.viewer is not None:
        controller.viewer.close()

    print("\n[SAC] Training complete.")
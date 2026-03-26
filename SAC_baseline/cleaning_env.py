"""
SAC_baseline/cleaning_env.py
============================
Gymnasium environment wrapping the UR5e ball-clearing MuJoCo task.

RL formulation
--------------
  Action      : flattened DMP weights  w ∈ R^(2 · n_bfs)
                (bounded by ±w_bound, default 4 · random_scale ≈ 40)

  Observation : [ ee_site_pos_normalised  (3,)          ]
                [ prev_weight_normalised  (2 · n_bfs,)  ]
                All values mapped to [-1, 1].

  Reward      : fraction of balls swept off the workspace:
                    r = (initial_count − remaining_count) / initial_count
                with an optional additive grid-uniformity bonus.

  Episode     : exactly 1 MDP step
                  reset() → agent proposes weights → step() → terminated=True
                This mirrors one "iteration" of the LLM-runner in
                runner/llm_main_runner.py and makes both approaches
                directly comparable on the same metric.

Observation normalisation
-------------------------
  ee_site position  → each axis independently scaled to [-1, 1] using
                       workspace bounds (+20 % margin for the z-axis).
  prev weights      → divided by w_bound.
"""

from __future__ import annotations

import os
import sys
import time
import numpy as np

import gymnasium as gym
from gymnasium import spaces

# ── add project root to sys.path so existing modules are importable ────────
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import mujoco
from runner.main_runner import EnhancedDMPController
from env.robot_logic import (
    get_joint_positions,
    set_joint_positions,
    enhanced_ik_solver,
)
from env.llm_robot_logic import get_dmp_step_with_obstacles
from agent.pydmps.dmp_rhythmic import DMPs_rhythmic
from utils.draw_shapes import circle_trajectory


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _count_balls_silent(
    model, data,
    x_min, x_max, y_min, y_max,
    num_x, num_y, num_balls,
):
    """Count balls inside the workspace grid without any console output."""
    x_edges = np.linspace(x_min, x_max, num_x + 1)
    y_edges = np.linspace(y_min, y_max, num_y + 1)
    grid = np.zeros((num_x, num_y), dtype=np.int32)
    total = 0
    for i in range(1, num_balls + 1):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"ball_{i}")
        if bid == -1:
            continue
        px, py = data.xpos[bid, 0], data.xpos[bid, 1]
        if x_min <= px <= x_max and y_min <= py <= y_max:
            ix = int(np.clip(np.searchsorted(x_edges, px, side="right") - 1, 0, num_x - 1))
            jy = int(np.clip(np.searchsorted(y_edges, py, side="right") - 1, 0, num_y - 1))
            grid[ix, jy] += 1
            total += 1
    grid = grid[:, ::-1]   # mirror original visual layout
    return grid, total


# ---------------------------------------------------------------------------
# Gymnasium environment
# ---------------------------------------------------------------------------

class BallClearingEnv(gym.Env):
    """
    Parameters
    ----------
    config : dict
        Loaded YAML config (same schema as ``config/sac_config.yaml``).
    w_bound : float | None
        Symmetric bound for the weight action space.
        Defaults to ``4 * random_scale`` from config (≈ 40).
    use_grid_reward : bool
        Add a small shaped bonus for distributing cleared balls across
        all grid cells rather than only maximising total count.
    fast_physics : bool
        Skip ``time.sleep()`` during trajectory replay (strongly
        recommended during training; set to ``False`` only when you want
        to watch the simulation in real-time).
    verbose : int
        0 – silent, 1 – print per-episode summary line.
    """

    metadata = {"render_modes": ["human"]}

    # ------------------------------------------------------------------
    def __init__(
        self,
        config: dict,
        w_bound: float | None = None,
        use_grid_reward: bool = False,
        fast_physics: bool = True,
        verbose: int = 0,
    ):
        super().__init__()

        self.config = config
        self.use_grid_reward = use_grid_reward
        self.fast_physics = fast_physics
        self.verbose = verbose

        # ── extract config values ──────────────────────────────────────
        dmp_cfg   = config["dmp_params"]
        sim_cfg   = config["simulation"]
        robot_cfg = config["robot"]

        self.n_bfs: int         = int(dmp_cfg["n_bfs"])
        self.action_dim: int    = 2 * self.n_bfs
        self.dt: float          = float(sim_cfg["dt"])
        self.mop_z: float       = float(robot_cfg["mop_z_height"])
        self.num_balls: int     = int(dmp_cfg["num_balls"])
        self.num_x: int         = int(dmp_cfg["num_x_segments"])
        self.num_y: int         = int(dmp_cfg["num_y_segments"])
        self.joint_names: list  = robot_cfg["joint_names"]

        ws_c = sim_cfg["ws_center"]
        ws_w = float(sim_cfg["ws_width"])
        ws_l = float(sim_cfg["ws_length"])
        self.ws_cx  = float(ws_c[0])
        self.ws_cy  = float(ws_c[1])
        self.x_min  = self.ws_cx - ws_w / 2.0
        self.x_max  = self.ws_cx + ws_w / 2.0
        self.y_min  = self.ws_cy - ws_l / 2.0
        self.y_max  = self.ws_cy + ws_l / 2.0

        random_scale      = float(dmp_cfg.get("random_scale", 10.0))
        self.w_bound      = float(w_bound) if w_bound is not None else 4.0 * random_scale
        self._keep_every  = max(1, int(dmp_cfg.get("deci_build", 2)))

        # ── ee_site normalisation bounds (workspace + 20 % z margin) ──
        # x ∈ [x_min, x_max],  y ∈ [y_min, y_max],  z ≈ mop_z ± 0.2
        z_margin = 0.2
        self._pos_lo = np.array([self.x_min, self.y_min, self.mop_z - z_margin], dtype=np.float32)
        self._pos_hi = np.array([self.x_max, self.y_max, self.mop_z + z_margin], dtype=np.float32)
        self._pos_mid   = (self._pos_lo + self._pos_hi) / 2.0
        self._pos_scale = (self._pos_hi - self._pos_lo) / 2.0   # half-range

        # ── spaces ────────────────────────────────────────────────────
        obs_dim = 3 + self.action_dim   # ee_pos(3) + prev_weights(2*n_bfs)
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low =-self.w_bound,
            high= self.w_bound,
            shape=(self.action_dim,),
            dtype=np.float32,
        )

        # ── MuJoCo via the existing controller ────────────────────────
        # EnhancedDMPController owns the MjModel, MjData, IK solver,
        # hard_reset_from_home(), and ball-counting logic.
        self._ctrl    = EnhancedDMPController(config)
        self._model   = self._ctrl.model
        self._data    = self._ctrl.data
        self._site_id = self._ctrl.site_id

        # ── rhythmic DMP (2-DOF, same as the LLM-runner) ──────────────
        self._dmp = DMPs_rhythmic(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)
        # imitate a small circle once to initialise y0 / goal / basis
        # functions; afterwards we only swap the weights each episode
        _cx, _cy = circle_trajectory(
            center=(self.ws_cx, self.ws_cy),
            radius=0.3 * ws_w,
            num_points=200, plot=False,
        )
        _demo = np.vstack((_cx, _cy))
        _demo = np.hstack((np.array([[self.ws_cx], [self.ws_cy]]), _demo))
        self._dmp.imitate_path(_demo, plot=False)

        # ── episode bookkeeping ────────────────────────────────────────
        self._prev_weights        = np.zeros(self.action_dim, dtype=np.float32)
        self._initial_ball_count  = 0
        self._episode             = 0

        # record initial ball count from the pristine XML state
        mujoco.mj_forward(self._model, self._data)
        _, self._xml_ball_count = _count_balls_silent(
            self._model, self._data,
            self.x_min, self.x_max, self.y_min, self.y_max,
            self.num_x, self.num_y, self.num_balls,
        )

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------
    def _get_ee_pos(self) -> np.ndarray:
        """Return the current ee_site 3-D position in world frame."""
        return self._data.site_xpos[self._site_id].copy().astype(np.float32)

    def _normalise_pos(self, pos: np.ndarray) -> np.ndarray:
        """Map ee_pos to approximately [-1, 1] per axis."""
        return np.clip((pos - self._pos_mid) / (self._pos_scale + 1e-8), -1.0, 1.0).astype(np.float32)

    def _normalise_weights(self, w: np.ndarray) -> np.ndarray:
        return np.clip(w / (self.w_bound + 1e-8), -1.0, 1.0).astype(np.float32)

    def _build_obs(self, ee_pos: np.ndarray, prev_weights: np.ndarray) -> np.ndarray:
        return np.concatenate(
            [self._normalise_pos(ee_pos), self._normalise_weights(prev_weights)],
            dtype=np.float32,
        )

    # ------------------------------------------------------------------
    # Trajectory execution  (no viewer sync, no real-time sleep)
    # ------------------------------------------------------------------
    def _run_trajectory(self, weights: np.ndarray):
        """
        Roll out one DMP trajectory in the MuJoCo sim.

        Parameters
        ----------
        weights : ndarray, shape (2 * n_bfs,)

        Returns
        -------
        joint_traj : list of joint-position snapshots
        final_ee   : ndarray (3,)  – ee position after last waypoint
        """
        # configure DMP: swap weights, keep y0 / goal from circle init
        self._dmp.w = weights.reshape(2, self.n_bfs).copy()
        self._dmp.reset_state()

        start_joints = get_joint_positions(self._model, self._data, self.joint_names)
        joint_traj = []

        for i in range(int(self._dmp.timesteps)):
            y = get_dmp_step_with_obstacles(self._dmp)
            target_3d = np.array([y[0], y[1], self.mop_z], dtype=float)

            ok, _ = enhanced_ik_solver(
                self._model, self._data, self._site_id, target_3d,
                self.joint_names,
                max_iters_per_wp=50,
                print_every=1_000_000,  # silence IK output during training
            )
            if not ok:
                continue
            if i % self._keep_every == 0:
                joint_traj.append(
                    get_joint_positions(self._model, self._data, self.joint_names).copy()
                )

        # replay joints through the physics engine to move balls
        if joint_traj:
            set_joint_positions(self._model, self._data, self.joint_names, start_joints)
            mujoco.mj_forward(self._model, self._data)
            for joints in joint_traj:
                self._data.ctrl[:] = joints
                mujoco.mj_step(self._model, self._data)
                if not self.fast_physics:
                    time.sleep(self.dt * 2)

        # settle physics one extra step
        mujoco.mj_step(self._model, self._data)
        mujoco.mj_forward(self._model, self._data)

        final_ee = self._get_ee_pos()
        return joint_traj, final_ee

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)

        # restore full MuJoCo snapshot: balls to initial positions + robot home
        self._ctrl.hard_reset_from_home(redraw=False)
        mujoco.mj_forward(self._model, self._data)

        _, self._initial_ball_count = _count_balls_silent(
            self._model, self._data,
            self.x_min, self.x_max, self.y_min, self.y_max,
            self.num_x, self.num_y, self.num_balls,
        )

        self._prev_weights = np.zeros(self.action_dim, dtype=np.float32)
        self._episode += 1

        ee_pos = self._get_ee_pos()
        obs    = self._build_obs(ee_pos, self._prev_weights)
        info   = {
            "initial_ball_count": self._initial_ball_count,
            "episode":            self._episode,
            "ee_pos":             ee_pos.tolist(),
        }
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict]:
        """
        One MDP step = one complete DMP-weight-driven trajectory.

        Parameters
        ----------
        action : ndarray, shape (2 * n_bfs,)
            DMP weights proposed by the SAC policy.

        Returns
        -------
        obs        : ndarray  (ee_pos_norm (3,) + prev_weights_norm (2·n_bfs,))
        reward     : float    fraction of balls swept ∈ [0, 1]
        terminated : True     (single-step episode)
        truncated  : False
        info       : dict
        """
        weights = np.asarray(action, dtype=float)

        # ── run simulation ─────────────────────────────────────────────
        joint_traj, final_ee = self._run_trajectory(weights)

        # ── score ──────────────────────────────────────────────────────
        grid_after, balls_remaining = _count_balls_silent(
            self._model, self._data,
            self.x_min, self.x_max, self.y_min, self.y_max,
            self.num_x, self.num_y, self.num_balls,
        )

        balls_swept = max(0, self._initial_ball_count - balls_remaining)
        reward = balls_swept / max(self._initial_ball_count, 1)  # ∈ [0, 1]

        # optional: small bonus for spatially uniform coverage
        if self.use_grid_reward and self._initial_ball_count > 0:
            max_per_cell = self._initial_ball_count / (self.num_x * self.num_y)
            uniformity_penalty = float(
                np.std(grid_after.astype(float)) / max(max_per_cell, 1e-6)
            )
            reward = reward - 0.05 * uniformity_penalty

        # ── build next observation ─────────────────────────────────────
        self._prev_weights = weights.astype(np.float32)
        obs = self._build_obs(final_ee, self._prev_weights)

        if self.verbose >= 1:
            print(
                f"[BallClearingEnv ep={self._episode:4d}]  "
                f"swept={balls_swept}/{self._initial_ball_count}  "
                f"reward={reward:.4f}  waypoints={len(joint_traj)}  "
                f"ee_pos=[{final_ee[0]:.3f}, {final_ee[1]:.3f}, {final_ee[2]:.3f}]"
            )

        info = {
            "balls_swept":     balls_swept,
            "balls_remaining": balls_remaining,
            "initial_count":   self._initial_ball_count,
            "waypoints":       len(joint_traj),
            "ee_pos":          final_ee.tolist(),
        }

        # single-step episode: always terminated, never truncated
        return obs, float(reward), True, False, info

    def render(self):
        if self._ctrl.viewer is not None:
            self._ctrl.viewer.draw()

    def close(self):
        try:
            self._ctrl.viewer.close()
        except Exception:
            pass

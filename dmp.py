
import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import List

try:
    from pydmps.dmp_discrete import DMPs_discrete
    PYDMP_AVAILABLE = True
    print("PyDMP loaded from pydmps")
except Exception as e:
    PYDMP_AVAILABLE = False
    print("PyDMP import failed:", e)

class EnhancedDMPController:
    """
    Point-to-point discrete DMP controller with resolved-rate IK.
    Z coordinate is fixed to fixed_z for all waypoints.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, fixed_z: float = 0.54):
        self.model = model
        self.data = data
        self.fixed_z = float(fixed_z)

        # joint names + ee site (adjust to your model if needed)
        self.joint_names = [
            "shoulder_pan", "shoulder_lift", "elbow",
            "wrist_1", "wrist_2", "wrist_3"
        ]
        self.site_name = "ee_site"
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if self.site_id == -1:
            raise RuntimeError(f"Site '{self.site_name}' not found")

        # actuator names used to send position commands (adjust to your model)
        actuator_names = [
            "shoulder_pan_act", "shoulder_lift_act", "elbow_act",
            "wrist_1_act", "wrist_2_act", "wrist_3_act"
        ]
        self.actuator_ids = []
        for name in actuator_names:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid == -1:
                raise RuntimeError(f"Actuator '{name}' not found")
            self.actuator_ids.append(aid)

        # DOF indices for selecting columns from Jacobian
        self.dof_indices = []
        for jn in self.joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid == -1:
                raise RuntimeError(f"Joint '{jn}' not found")
            self.dof_indices.append(model.jnt_dofadr[jid])
        self.dof_indices = np.array(self.dof_indices, dtype=int)

        # Controller params (tweak for speed / smoothness)
        self.dt = 0.005            
        self.pos_k = 1.5       
        self.max_dq = 0.08        
        self.speed_factor = 0.6   
        self.pos_tol = 0.02       
        self.dmp_complete_threshold = 0.01

        # internal command state
        self.q_cmd = self._get_current_joint_positions()

        # placeholders
        self.current_dmp = None
        self.raw_waypoints = None

        print(f"Controller initialized: {len(self.joint_names)} joints, dt={self.dt}, fixed_z={self.fixed_z}")

    def _get_current_joint_positions(self) -> np.ndarray:
        q = np.zeros(len(self.joint_names))
        for i, jn in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            qpos_adr = self.model.jnt_qposadr[jid]
            q[i] = float(self.data.qpos[qpos_adr])
        return q

    def _set_actuator_positions(self, q_values: np.ndarray):
        for i, aid in enumerate(self.actuator_ids):
            if i < len(q_values):
                self.data.ctrl[aid] = float(q_values[i])

    def _compute_ik_step(self, target: np.ndarray) -> float:
        ee_pos = self.data.site_xpos[self.site_id].copy()
        # enforce fixed z for comparisons / smoothness
        ee_pos[2] = self.fixed_z
        err = target - ee_pos
        norm_err = np.linalg.norm(err)

        if norm_err < self.pos_tol:
            return norm_err

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
        J = jacp[:, self.dof_indices]  # shape (3, n_joints)

        xdot = self.pos_k * err
        try:
            J_pinv = np.linalg.pinv(J, rcond=1e-4)
            dq = J_pinv @ (xdot * self.dt)
        except Exception:
            dq = np.zeros(len(self.dof_indices))

        dq = np.clip(dq, -self.max_dq, self.max_dq) * self.speed_factor
        self.q_cmd += dq
        self._set_actuator_positions(self.q_cmd)
        return norm_err

    def run_segment_until_reached(
        self,
        start_pos: np.ndarray,
        goal_pos: np.ndarray,
        n_bfs: int,
        viewer,
        max_time: float = 10.0
    ) -> bool:
        """
        Run a DMP that goes from start_pos -> goal_pos until the EE is within pos_tol.
        start_pos and goal_pos are 3-vectors (z already set to fixed_z).
        """
        if not PYDMP_AVAILABLE:
            print("PyDMP unavailable")
            return False

        dmp = DMPs_discrete(n_dmps=3, n_bfs=n_bfs, dt=self.dt)
        demo_pts = np.linspace(start_pos, goal_pos, 60)  # 60 demo points
        dmp.imitate_path(demo_pts.T)
        dmp.goal = goal_pos.copy()
        dmp.y = start_pos.copy()
        dmp.reset_state()

        start_time = time.time()
        steps = 0
        max_steps = int(max_time / self.dt)

        while viewer.is_running() and (steps < max_steps):
            y, yd, ydd = dmp.step()
            target = np.array(y, dtype=float)
            # force target z to fixed_z (safety)
            target[2] = self.fixed_z

            err = self._compute_ik_step(target)

            mujoco.mj_step(self.model, self.data)
            time.sleep(self.dt)
            viewer.sync()

            ee_pos = self.data.site_xpos[self.site_id].copy()
            ee_pos[2] = self.fixed_z  # enforce fixed z for distance calc
            dist_to_goal = np.linalg.norm(goal_pos - ee_pos)
            if (dist_to_goal <= self.pos_tol) or (dmp.cs.x < self.dmp_complete_threshold):
                print(f"Reached segment goal (dist={dist_to_goal:.4f}) after {time.time()-start_time:.2f}s")
                return True

            steps += 1
            if steps % 200 == 0:
                print(f" segment time {time.time()-start_time:.2f}s, dist_to_goal={dist_to_goal:.4f}, phase={dmp.cs.x:.3f}")

        print("Segment timed out or viewer closed")
        return False

    def execute_point_to_point(
        self,
        waypoints_xy: List[List[float]],
        n_bfs: int = 50,
        viewer=None
    ) -> bool:
        """
        Execute waypoints in strict sequence: finish current point before moving to next.
        waypoints_xy: list of [x, y] pairs. z is set to fixed_z automatically.
        """
        if not PYDMP_AVAILABLE:
            print("PyDMP unavailable")
            return False
        if viewer is None:
            print("Viewer required")
            return False

        wp_xy = np.array(waypoints_xy, dtype=float)
        if wp_xy.ndim != 2 or wp_xy.shape[1] != 2 or wp_xy.shape[0] < 1:
            print("Invalid waypoint list; need Nx2 (x y) pairs")
            return False

        # store raw waypoints for reference
        self.raw_waypoints = np.hstack((wp_xy, np.full((wp_xy.shape[0],1), self.fixed_z)))

        # starting point: current EE pos but force z to fixed_z
        ee_start = self.data.site_xpos[self.site_id].copy()
        ee_start[2] = self.fixed_z
        start_pos = ee_start

        for idx in range(wp_xy.shape[0]):
            goal = np.array([wp_xy[idx,0], wp_xy[idx,1], self.fixed_z], dtype=float)
            print(f"\n Segment {idx+1}/{wp_xy.shape[0]}: start={start_pos} -> goal={goal} ---")
            success = self.run_segment_until_reached(start_pos, goal, n_bfs, viewer, max_time=30.0)
            if not success:
                print(f"Failed to reach waypoint {idx+1}")
                return False
            start_pos = self.data.site_xpos[self.site_id].copy()
            start_pos[2] = self.fixed_z

        print("All waypoints reached")
        return True


def main_user_input():
    """Interactive runner for point-to-point DMPs (enter only X Y; Z is constant for now)."""
    if not PYDMP_AVAILABLE:
        print("Cannot run: PyDMP unavailable")
        return

    joint_names = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
    desired_pose = [0.188, -2.2, -0.87, 0.0, np.pi/2, np.pi/2]

    try:
        model = mujoco.MjModel.from_xml_path("ballmove.xml")
        data = mujoco.MjData(model)
        print("Model loaded")
    except Exception as e:
        print("Load error:", e)
        return

    # set initial qpos
    for name, angle in zip(joint_names, desired_pose):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid == -1:
            print(f"Warning: joint {name} not found in model")
            continue
        adr = model.jnt_qposadr[jid]
        data.qpos[adr] = angle
    mujoco.mj_forward(model, data)

    fixed_z_value = 0.54  # change z here(might need as some friction)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            ctrl = EnhancedDMPController(model, data, fixed_z=fixed_z_value)
        except Exception as e:
            print("Controller init failed:", e)
            return

        ctrl.q_cmd = np.array(desired_pose, dtype=float)

        while True:
            try:
                n = int(input("\nNumber of waypoints (0 to exit): "))
            except Exception:
                print("Enter an integer.")
                continue
            if n == 0:
                break

            pts = []
            for i in range(n):
                while True:
                    vals = input(f" Waypoint {i+1} x y: ").split()
                    if len(vals) == 2:
                        try:
                            x_val = float(vals[0]); y_val = float(vals[1])
                            pts.append([x_val, y_val])
                            break
                        except ValueError:
                            print("Invalid floats, try again.")
                    else:
                        print("Enter 2 values: x y (z will be fixed at {:.2f})".format(fixed_z_value))

            try:
                bfs = int(input("Basis functions [50]: ") or "50")
            except ValueError:
                bfs = 50

            print(f"Executing point-to-point sequence with fixed z={fixed_z_value} (will wait at each point)...")
            success = ctrl.execute_point_to_point(pts, n_bfs=bfs, viewer=viewer)
            print("Success" if success else "❌ Failed")


if __name__ == "__main__":
    main_user_input()

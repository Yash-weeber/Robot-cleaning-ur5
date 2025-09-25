import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import List

try:
    from dmps.dmp_discrete import DMPs_discrete
    PYDMP_AVAILABLE = True
    print(" PyDMP loaded from dmps")
except ImportError:
    PYDMP_AVAILABLE = True


class EnhancedDMPController:
    """
    Discrete DMP Controller integrating DMPs_discrete with resolved-rate IK control.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data

        # UR5e joint names and EE site
        self.joint_names = [
            "shoulder_pan", "shoulder_lift", "elbow",
            "wrist_1", "wrist_2", "wrist_3"
        ]
        self.site_name = "ee_site"
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, self.site_name
        )
        if self.site_id == -1:
            raise RuntimeError(f"Site '{self.site_name}' not found")

        # Actuator IDs for position control
        actuator_names = [
            "shoulder_pan_act", "shoulder_lift_act", "elbow_act",
            "wrist_1_act", "wrist_2_act", "wrist_3_act"
        ]
        self.actuator_ids = []
        for name in actuator_names:
            aid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            )
            if aid == -1:
                raise RuntimeError(f"Actuator '{name}' not found")
            self.actuator_ids.append(aid)

        # DOF indices for Jacobian extraction
        self.dof_indices = []
        for jn in self.joint_names:
            jid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, jn
            )
            if jid == -1:
                raise RuntimeError(f"Joint '{jn}' not found")
            self.dof_indices.append(model.jnt_dofadr[jid])
        self.dof_indices = np.array(self.dof_indices)

        # Control gains and limits
        self.dt = 0.01
        self.pos_k = 5.0
        self.max_dq = 0.3
        self.pos_tol = 0.015

        # Initialize joint commands to current positions
        self.q_cmd = self._get_current_joint_positions()

        # Placeholder for DMP instance
        self.current_dmp = None
        self.dmp_complete_threshold = 0.01

        print(f"Controller initialized: {len(self.joint_names)} joints")

    def _get_current_joint_positions(self) -> np.ndarray:
        """Read current qpos for each joint."""
        q = np.zeros(len(self.joint_names))
        for i, jn in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, jn
            )
            qpos_adr = self.model.jnt_qposadr[jid]
            q[i] = self.data.qpos[qpos_adr]
        return q

    def setup_discrete_dmp(self, waypoints: np.ndarray, n_bfs: int = 50) -> bool:
        """
        Train a discrete DMP on given waypoints.
        waypoints: shape (n_points, 3)
        """
        if not PYDMP_AVAILABLE:
            print(" PyDMP unavailable")
            return False
        if waypoints.shape[0] < 2:
            print("we need >=2 waypoints required")
            return False

        print(f" Training DMP: {waypoints.shape[0]} points, {n_bfs} basis funcs")
        try:
            self.current_dmp = DMPs_discrete(
                n_dmps=3, n_bfs=n_bfs, dt=self.dt
            )
            self.current_dmp.imitate_path(waypoints.T)
            self.current_dmp.reset_state()
            print(" DMP training complete")
            return True
        except Exception as e:
            print(f" DMP setup error: {e}")
            return False

    def execute_dmp_trajectory(
        self,
        viewer,
        max_duration: float = 15.0
    ) -> bool:
        """
        Execute the trained DMP, controlling joints via resolved-rate IK.
        """
        if self.current_dmp is None:
            print(" No DMP trained")
            return False

        print(" Starting trajectory...")
        start = time.time()
        steps = 0
        max_steps = int(max_duration / self.dt)
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        while (
            viewer.is_running() and
            steps < max_steps
        ):
            # DMP step => target EE pos
            target, _, _ = self.current_dmp.step()
            ee_pos = self.data.site_xpos[self.site_id].copy()
            err = target - ee_pos
            norm_err = np.linalg.norm(err)

            # IK if outside tolerance
            if norm_err > self.pos_tol:
                mujoco.mj_jacSite(
                    self.model, self.data, jacp, jacr, self.site_id
                )
                J = jacp[:, self.dof_indices]
                xdot = self.pos_k * err
                dq = np.linalg.pinv(J, rcond=1e-4) @ xdot * self.dt
                dq = np.clip(dq, -self.max_dq, self.max_dq)
                self.q_cmd += dq
                for i, aid in enumerate(self.actuator_ids):
                    self.data.ctrl[aid] = self.q_cmd[i]
            else:
                for i, aid in enumerate(self.actuator_ids):
                    self.data.ctrl[aid] = self.q_cmd[i]

            # Check canonical phase
            phase = self.current_dmp.cs.x
            if phase < self.dmp_complete_threshold:
                print("🎯 Trajectory complete")
                return True

            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            steps += 1
            time.sleep(0.005)

            # Progress log every 200 steps
            if steps % 200 == 0:
                print(f" {time.time()-start:.1f}s, err={norm_err:.3f}, phase={phase:.3f}")

        print("️ Trajectory timed out or viewer closed")
        return False

    def execute_multi_waypoint(
        self,
        waypoints: List[List[float]],
        n_bfs: int = 50,
        viewer=None
    ) -> bool:
        """
        Run a DMP on a sequence of waypoints.
        """
        wp = np.array(waypoints)
        if wp.shape[0] < 2 or wp.shape[1] != 3:
            print("Invalid waypoint list")
            return False
        if viewer is None:
            print("Viewer required")
            return False

        if not self.setup_discrete_dmp(wp, n_bfs):
            return False
        return self.execute_dmp_trajectory(viewer)

def main_user_input():
    """Interactive DMP waypoint runner."""
    if not PYDMP_AVAILABLE:
        print(" Cannot run: PyDMP unavailable")
        return

    # Desired initial joint configuration
    joint_names   = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
    desired_pose  = [0.188, -2.2, -0.87, 0.0, np.pi/2, np.pi/2]

    try:
        model = mujoco.MjModel.from_xml_path("ballmove.xml")
        data  = mujoco.MjData(model)
        print(" Model loaded")
    except Exception as e:
        print(f" Load error: {e}")
        return

    # Manually set the initial joint positions in data.qpos
    for name, angle in zip(joint_names, desired_pose):
        jid    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        adr    = model.jnt_qposadr[jid]
        data.qpos[adr] = angle
    # Propagate qpos -> qvel, qacc, geom, sites, etc.
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            ctrl = EnhancedDMPController(model, data)
        except Exception as e:
            print(f" Controller init failed: {e}")
            return

        # Also initialize the controller’s internal q_cmd to the same pose
        ctrl.q_cmd = np.array(desired_pose)

        while True:
            try:
                n = int(input("Number of waypoints (0 to exit): "))
            except ValueError:
                continue
            if n == 0:
                break

            pts = []
            for i in range(n):
                while True:
                    vals = input(f" Waypoint {i+1} x y z: ").split()
                    if len(vals)==3:
                        try:
                            pts.append([float(v) for v in vals])
                            break
                        except:
                            pass

            try:
                bfs = int(input("Basis functions [25]: ") or "25")
            except:
                bfs = 25

            print("️ Executing")
            success = ctrl.execute_multi_waypoint(pts, bfs, viewer)
            print(" Success" if success else "❌ Failed")


if __name__ == "__main__":
    main_user_input()

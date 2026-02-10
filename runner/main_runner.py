import time
import numpy as np
import mujoco
try:
    import tkinter as tk
    from tkinter import messagebox, simpledialog
    GUI_AVAILABLE = True
except ImportError:
    tk = None
    messagebox = None
    simpledialog = None
    GUI_AVAILABLE = False

import threading

# Internal imports from the factorized codebase
from env.adapter import ViewerAdapter
from env.robot_logic import (
    set_joint_positions, get_joint_positions, _clamp_limits,
    enhanced_ik_solver, animate_robot_movement
)
from env.world import count_balls_in_grid
from agent.dmp_logic import DMPs_discrete, DMPs_rhythmic
from agent.interfaces import DrawingInterface, RealTimeMouseControl
from utils.draw_shapes import infinity_trajectory
from utils.obstacle_avoidance import avoid_obstacles


class EnhancedDMPController:
    def __init__(self, config):
        # Configuration setup
        self.config = config
        self.xml_path = config['simulation']['xml_path']
        self.site_name = config['simulation']['site_name']
        self.dt = config['simulation']['dt']
        self.joint_names = config['robot']['joint_names']
        self.home_positions = np.array(config['robot']['home_joint_positions'])
        self.mop_z_height = config['robot']['mop_z_height']

        self.n_bfs = config['dmp_params']['n_bfs']
        self.num_balls = config['dmp_params']['num_balls']
        self.num_x_segments = config['dmp_params']['num_x_segments']
        self.num_y_segments = config['dmp_params']['num_y_segments']

        self.ik_params = config['ik_params']

        # Load model and data
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if self.site_id == -1:
            raise RuntimeError(f"Site '{self.site_name}' not found in model")

        # Initialize viewer
        self.viewer = ViewerAdapter(self.model, self.data)
        self._qpos0 = self.data.qpos.copy()
        self._qvel0 = self.data.qvel.copy()
        self._act0 = self.data.act.copy() if hasattr(self.data, "act") else None

        # State and logging
        self.running = True
        self.ws_center = config["simulation"]["ws_center"]
        ws_width = config["simulation"]["ws_width"]
        ws_length = config["simulation"]["ws_length"]
        self.x_min = self.ws_center[0] - ws_width / 2.0
        self.x_max = self.ws_center[0] + ws_width / 2.0
        self.y_min = self.ws_center[1] - ws_length / 2.0
        self.y_max = self.ws_center[1] + ws_length / 2.0
        self.grid_count = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)
        self.kp = [300, 300, 150, 80, 50, 50]
        self.kd = [150, 150, 80, 40, 20, 20]
        # self.set_joint_pid_gains(self.kp, self.kd)
        self.reset_robot_to_home()

    def reset_robot_to_home(self):

        set_joint_positions(self.model, self.data, self.joint_names, self.home_positions)
        _clamp_limits(self.model, self.data.qpos, self.joint_names)
        mujoco.mj_forward(self.model, self.data)
        current_pos = self.data.site_xpos[self.site_id]
        print(f" Robot reset to home position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        self.viewer.draw()
        return True

    def set_joint_pid_gains(self, kp_values, kd_values):
        """
        Set kp and kd for each joint actuator in MuJoCo.
        kp_values and kd_values should be lists/arrays of same length as joint_names.
        """
        for i, jn in enumerate(self.joint_names):
            # Get joint ID and DOF ID
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            dof_id = self.model.jnt_dofadr[joint_id]

            # Find actuator ID by checking which joint it controls
            actuator_id = -1
            for aid in range(self.model.nu):
                # trnid[aid, 0] stores the joint index for the actuator
                if self.model.actuator_trnid[aid, 0] == joint_id:
                    actuator_id = aid
                    break
            
            if actuator_id != -1:
                # Set kp (proportional gain)
                self.model.actuator_gainprm[actuator_id, 0] = kp_values[i]
                # Set bias for position servo (standard MuJoCo position actuator formula)
                self.model.actuator_biasprm[actuator_id, 1] = -kp_values[i] 
            else:
                print(f"Warning: No actuator found for joint '{jn}'")
            self.model.dof_damping[dof_id] = kd_values[i]
    
    def hard_reset_from_home(self, redraw=True):

        # 1) Restore full snapshot from the copy created in __init__
        np.copyto(self.data.qpos, self._qpos0)
        np.copyto(self.data.qvel, self._qvel0)
        if hasattr(self.data, "act") and self._act0 is not None:
            np.copyto(self.data.act, self._act0)

        # 2) Overwrite robot joints to HOME and zero their velocities
        for i, jn in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            qadr = self.model.jnt_qposadr[jid]
            dadr = self.model.jnt_dofadr[jid]
            self.data.qpos[qadr] = self.home_positions[i]
            self.data.qvel[dadr] = 0.0

        # 3) Rebuild physics + clear counters
        mujoco.mj_forward(self.model, self.data)

        # 4) Optional redraw
        if redraw and self.viewer:
            self.viewer.draw()
    def move_to_3d_position(self, target_xy, animate=True):
        # Exact copy of 3D move logic
        target_3d = np.array([target_xy[0], target_xy[1], self.mop_z_height])
        print(f"Moving to 3D target: [{target_3d[0]:.3f}, {target_3d[1]:.3f}, {target_3d[2]:.3f}]")

        start_joints = get_joint_positions(self.model, self.data, self.joint_names)

        success, error = enhanced_ik_solver(
            self.model, self.data, self.site_id, target_3d, self.joint_names,
            step_clip=0.2, max_wp_step=0.03, max_iters_per_wp=300,
            lam_init=self.ik_params['init_lambda'],
            tol=self.ik_params['tol'],
            print_every=self.ik_params['print_every']
        )

        if success:
            print(f"IK Success! Position error: {error:.6f} m")
            if animate:
                target_joints = get_joint_positions(self.model, self.data, self.joint_names)
                set_joint_positions(self.model, self.data, self.joint_names, start_joints)
                # Call to animation function preserved exactly
                animate_robot_movement(self.model, self.data, self.viewer, self.joint_names,
                                       start_joints, target_joints, duration=2.0, fps=60)
            return True
        else:
            print(f"IK Failed! Final error: {error:.6f} m")
            return False

    def apply_dmp(self, pattern="discrete", draw_waypoints=False, shape="infinity"):
        # Original training logic
        if pattern == "discrete":
            print("\nDISCRETE DMP MODE (3D)")
            self.dmp = DMPs_discrete(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)
        elif pattern == "rhythmic":
            print("\nRHYTHMIC DMP MODE (3D)")
            self.dmp = DMPs_rhythmic(n_dmps=2, n_bfs=self.n_bfs, dt=self.dt)

        if draw_waypoints:
            drawing_interface = DrawingInterface(title="Draw DMP 2D Trajectory")
            trajectory = drawing_interface.get_trajectory()
            if trajectory is None: return None
            current_pos = self.data.site_xpos[self.site_id]
            trajectory = np.vstack(([current_pos[:2]], trajectory))
        else:
            if shape == "infinity":
                x_traj, y_traj = infinity_trajectory(center=(self.ws_center[0], self.ws_center[1]), size=(1.0, 2.5), num_points=400, plot=False)
                trajectory = np.vstack((x_traj, y_traj)).T
                start_target_3d = np.array([x_traj[0], y_traj[0], self.mop_z_height])
                # Training IK preserved
                enhanced_ik_solver(self.model, self.data, self.site_id, start_target_3d, self.joint_names,
                                   max_iters_per_wp=50, print_every=1000)
            else:
                trajectory = self.get_discrete_waypoints()

        if trajectory is None: return None
        self.dmp.imitate_path(trajectory.T)
        self.dmp.reset_state()

        task_traj = []
        for i in range(int(self.dmp.timesteps)):
           
            ext_f = avoid_obstacles(
                self.dmp.y, self.dmp.dy, self.dmp.goal,
                rect_d0_x=0.14, rect_d0_y=0.06, rect_eta=0.2, 
                obs_d0=0.1, obs_eta=25.0, max_force=220.0
            )
            y, _, _ = self.dmp.step(tau=2.0, external_force=ext_f)
            task_traj.append(np.array([y[0], y[1], self.mop_z_height]))

        joint_traj = []
        for target_3d in task_traj:
            success, _ = enhanced_ik_solver(self.model, self.data, self.site_id, target_3d, self.joint_names,
                                            max_iters_per_wp=50, print_every=1000)
            if success:
                joint_traj.append(get_joint_positions(self.model, self.data, self.joint_names).copy())
        return joint_traj

    def execute_joint_trajectory(self, joint_traj, dt=None):

        if dt is None:
            dt = self.dt

        print(f"Executing joint trajectory with {len(joint_traj)} waypoints...")
        self.ee_trajectory = []  # Essential for trajectory analysis feedback

        if len(joint_traj) > 0:
            set_joint_positions(self.model, self.data, self.joint_names, joint_traj[0])
            mujoco.mj_forward(self.model, self.data)
            time.sleep(self.dt)

        for joints in joint_traj:
            self.data.ctrl[:] = joints
            mujoco.mj_step(self.model, self.data)

            # Log current ee position
            cl_pos = self.data.site_xpos[self.site_id].copy()
            self.ee_trajectory.append(cl_pos)

            if self.viewer and self.viewer.is_running():
                self.viewer.draw()
            time.sleep(dt)
        print("Trajectory execution complete.")

    def execute_realtime_mode(self):

        print(f"Z-coordinate fixed at: { self.mop_z_height:.4f} m")
        if not GUI_AVAILABLE:
            print("⚠️ Real-time mode requires a GUI. Skipping.")
            return

        # Create mouse control interface
        mouse_control = RealTimeMouseControl()


        dt = 0.01
        control_thread = threading.Thread(target=self.manual_move_prompt(),
                                          args=(mouse_control, dt))
        control_thread.daemon = True
        control_thread.start()

        # Keep interface running
        try:
            while mouse_control.running and self.viewer.is_running():
                mouse_control.root.update()
                time.sleep(0.01)
        except tk.TclError:
            pass

        print(" Real-time control mode ended")
    def count_balls_in_grid(self):
        """
        Counts balls in a 2x3 grid and applies the original visual layout transformation.
        """
        x_edges = np.linspace(self.x_min, self.x_max, self.num_x_segments + 1)
        y_edges = np.linspace(self.y_min, self.y_max, self.num_y_segments + 1)
        grid_counts = np.zeros((self.num_x_segments, self.num_y_segments), dtype=int)

        # Iterate through balls and check coordinates against grid edges
        ball_names = [f"ball_{i + 1}" for i in range(self.num_balls)]
        for name in ball_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if body_id != -1:
                pos = self.data.xpos[body_id][:2]
                if (self.x_min <= pos[0] <= self.x_max) and (self.y_min <= pos[1] <= self.y_max):
                    i = np.searchsorted(x_edges, pos[0], side='right') - 1
                    j = np.searchsorted(y_edges, pos[1], side='right') - 1
                    i = min(max(i, 0), self.num_x_segments - 1)
                    j = min(max(j, 0), self.num_y_segments - 1)
                    grid_counts[i, j] += 1

        # Match exact visual layout: reverse columns
        grid_counts = grid_counts[:, ::-1]
        self.grid_count = grid_counts.copy()
        return grid_counts

    def run(self):
        print("\nEnhanced DMP Controller Started!")
        while self.running and self.viewer.is_running():
            print("\nMAIN MENU:")
            print("1. Discrete DMP Mode (waypoint navigation)")
            print("2. Discrete DMP Mode (Draw waypoints)")
            print("3. Rhythmic DMP Mode (mouse-drawn patterns)")
            print("4. Rhythmic DMP Mode (Predefined Patterns)")
            print("5. Real-time Mouse Control")
            print("6. Reset Robot to Home Position")
            print("7. Move to Custom Position (X, Y)")
            print("8. Quit")
            choice = input("\nSelect mode (1-8): ").strip()
            if choice == '1':
                traj = self.apply_dmp(pattern="discrete", draw_waypoints=False)
                if traj: self.execute_joint_trajectory(traj)
            elif choice == '2':
                traj = self.apply_dmp(pattern="discrete", draw_waypoints=True)
                if traj: self.execute_joint_trajectory(traj)
            elif choice == '3':
                traj = self.apply_dmp(pattern="rhythmic", draw_waypoints=True)
                if traj: self.execute_joint_trajectory(traj)
            elif choice == '4':
                traj = self.apply_dmp(pattern="rhythmic", draw_waypoints=False)
                if traj: self.execute_joint_trajectory(traj)
            elif choice == '5':
                self.execute_realtime_mode()
            elif choice == '6':
                self.reset_robot_to_home()
            elif choice == '7':
                self.manual_move_prompt()
            elif choice == '8':
                self.running = False
            time.sleep(0.5)

 
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        count_balls_in_grid(self.model, self.data, self.x_min, self.x_max, self.y_min, self.y_max, self.num_x_segments,
                            self.num_y_segments, self.num_balls)
        self.viewer.close()

    def manual_move_prompt(self):
      
        coord_str = input(f"Enter target (x, y) [Z={self.mop_z_height:.4f}]: ").strip()
        if not coord_str: return
        try:
            x, y = map(float, coord_str.replace(',', ' ').split())
            if self.move_to_3d_position(np.array([x, y])): print("Movement completed.")
        except ValueError:
            print("Invalid coordinates.")

    def get_discrete_waypoints(self):
        
        if not GUI_AVAILABLE:
            print("⚠️ Waypoint dialog requires a GUI.")
            return None
        root = tk.Tk()
        root.withdraw()
        num_points = simpledialog.askinteger("Discrete DMP", "How many waypoints? (2-10)", minvalue=2, maxvalue=10)
        if num_points is None: return None
        points = []
        current_pos = self.data.site_xpos[self.site_id]
        for i in range(num_points):
            coord_str = simpledialog.askstring("Waypoint", f"Enter point {i + 1} (x, y):")
            if coord_str is None: return None
            x, y = map(float, coord_str.split(','))
            points.append([x, y])
        root.destroy()
        return np.array(points)
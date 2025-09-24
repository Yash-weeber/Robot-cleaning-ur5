import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

import mujoco
import mujoco.viewer
import numpy as np
from typing import Dict
import threading
import time
from shared_control import OSC, MujocoApp
from transforms3d.euler import euler2quat, quat2mat, mat2euler
from transforms3d.quaternions import qmult
from transforms3d.affines import compose
from shared_control.utils import Target
from shared_control.input_devices.space_mouse import SpaceMouse


class Welding(MujocoApp):
    """A task for controlling a UR5 robot with a space mouse, allowing the user to grab, move, and weld hexagon plates."""

    def __init__(self, robot_config_file: str = None, scene_file: str = None):
        super().__init__(robot_config_file, scene_file, use_sim=True)

        self.robot = self.get_robot(robot_name="UR5")

        osc_device_configs = [
            ("base", self.get_controller_config("osc2")),
            ("ur5", self.get_controller_config("osc2")),
        ]

        # Controller
        nullspace_config = self.get_controller_config("nullspace")
        self.controller = OSC(
            self.robot, self.model, self.data, osc_device_configs, nullspace_config
        )

        # Gripper
        self.gripper_motor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_motor"
        )
        self.gripper_closed_val = 0.08
        self.gripper_open_val = -0.08

        self.last_target_pos = None
        self.last_target_quat = None

        self.lock_initial_sm_pos = None
        self.lock_initial_sm_rot = None

        self.sm = SpaceMouse(origin=[0.0]*6)
        self.viewer = None

    def _generate_control_signals(self, target_world_pos, target_world_quat, sm_pos_offset=None, sm_rot_offset=None):

        # manual control
        if sm_pos_offset is not None:
            target_world_pos += sm_pos_offset * 0.5
        if sm_rot_offset is not None:
            target_world_quat = qmult(target_world_quat, sm_rot_offset)

        # target for end-effector
        ee_target = Target()
        ee_target.set_xyz(target_world_pos)
        ee_target.set_quat(target_world_quat)

        # Update mocap target visualization
        target_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "target"
        )
        if target_body_id != -1:
            mocap_id = self.model.body_mocapid[target_body_id]
            if mocap_id != -1 and 0 <= mocap_id < self.model.nmocap:
                self.data.mocap_pos[mocap_id] = np.array(target_world_pos)
                self.data.mocap_quat[mocap_id] = np.array(target_world_quat)

        self.last_target_pos = np.array(target_world_pos)
        self.last_target_quat = np.array(target_world_quat)

        # Generate control signals
        ctrl = self.controller.generate({"ur5": ee_target})
        return ctrl

    def key_callback(self, key):
        print(f"Key pressed: {key}")

        # Unlock end-effector state from controller
        if key == ord("1"):
            # self.lock_ee_state = False
            self.lock = None
            if self.last_target_pos is not None and self.last_target_quat is not None:
                self.sm.state.x = self.last_target_pos[0]
                self.sm.state.y = self.last_target_pos[1]
                self.sm.state.z = self.last_target_pos[2]
                orientation = mat2euler(quat2mat(self.last_target_quat), axes="rxyz")

                self.sm.state.roll = orientation[1]
                self.sm.state.pitch = orientation[0]
                self.sm.state.yaw = orientation[2]

            print("EE state: Unlocked to current position and orientation")
            return True

        # strong gripper control for plate movement
        elif key == ord("2"):
            self.gripper_closed_val = 2.0
            return True
            
        return False

    def run_task(self, demo_duration: int):
        self.timer_running = True
        time_thread = threading.Thread(target=self.sleep_for, args=(demo_duration,))
        time_thread.daemon = True
        time_thread.start()

        targets: Dict[str, Target] = {"ur5": Target(), "base": Target()}

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=self.key_callback) as viewer:
            self.viewer = viewer
            while self.timer_running and viewer.is_running():
                # Set the target values for the robot's devices
                x, y, z, roll, pitch, yaw, button1_pressed, button2_pressed = (
                    self.sm.update_state()
                )

                angle = euler2quat(
                    pitch, roll, yaw, axes="rxyz"
                )  # pitch and roll are flipped from SM API
                angle_mat = quat2mat(angle)

                tfmat_r = compose([x, y, z], angle_mat, [1, 1, 1])
                r_xyz = tfmat_r[0:3, -1].flatten()
                r_ang = np.array(mat2euler(tfmat_r[:3, :3]))

                targets["ur5"].set_xyz(r_xyz)
                targets["ur5"].set_abg(r_ang)
                targets["base"].set_abg([0, 0, np.arctan2(y, x) - np.pi / 2])

                ctrlr_output = self.controller.generate(targets)
                
                for force_idx, force in zip(*ctrlr_output):
                    self.data.ctrl[force_idx] = force

                if self.gripper_motor_id != -1:
                    if button1_pressed:
                        print("Gripper button pressed, toggling gripper state")
                        self.data.ctrl[self.gripper_motor_id] = self.gripper_open_val
                    else:
                        self.data.ctrl[self.gripper_motor_id] = self.gripper_closed_val

                if (
                    mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
                    != -1
                ):
                    body_id = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_BODY, "target"
                    )

                    mocap_id = self.model.body_mocapid[body_id]
                    if mocap_id != -1 and 0 <= mocap_id < self.model.nmocap:
                        if not self.lock:
                            self.data.mocap_pos[mocap_id] = r_xyz
                            self.data.mocap_quat[mocap_id] = euler2quat(
                                r_ang[0], r_ang[1], r_ang[2]
                            )

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.001)

        time_thread.join()

    def sleep_for(self, duration: int):
        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.1)
        self.timer_running = False

if __name__ == "__main__":
    debug_mode = False

    if debug_mode:
        scene_file = "welding_scene_ur5_debug.xml"
    else:
        scene_file = "welding_scene_ur5.xml"

    sim = Welding(
        robot_config_file="ur5_xyz_abg.yaml", scene_file=scene_file
    )
    sim.run_task(400)

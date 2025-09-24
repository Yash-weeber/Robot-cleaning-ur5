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
from shared_control.perception import PerceptionSystem


class GrabAndMove(MujocoApp):
    """A task for controlling a UR5 robot with a space mouse, allowing the user to grab and move hexagon plate."""

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

        # Lock ee state
        self.lock_ee_state = False
        self.prev_button2_pressed = False

        self.last_target_pos = None
        self.last_target_quat = None

        self.lock_initial_sm_pos = None
        self.lock_initial_sm_rot = None

        # Initialize perception system
        self.perception = PerceptionSystem(
            model=self.model,
            data=self.data,
            marker_size=0.10,  # 10cm marker (MuJoCo box size 0.05 is half-extent)
            image_width=self.image_width,
            image_height=self.image_height,
        )
        self.aruco_detection_interval = 10
        self.aruco_ids = [3,0]  # Updated to match hexagon plate
        self.current_aruco_idx = 0

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

    def _apply_sequential_rotations(
        self, marker_quat: np.ndarray, rotations: np.ndarray
    ) -> np.ndarray:
        for rot in rotations:
            marker_quat = qmult(marker_quat, euler2quat(*rot, axes="sxyz"))
        return marker_quat

    def lock_ee_to_target(
        self,
        marker_id,
        marker_pos,
        marker_quat,
        sm_pos_offset=None,
        sm_rot_offset=None,
    ):
        # default fallback position and orientation
        if marker_pos is None or marker_quat is None:
            self.lock_ee_state = False
            fallback_quat = euler2quat(
                self.lock_initial_sm_rot[1],  # pitch
                self.lock_initial_sm_rot[0],  # roll
                self.lock_initial_sm_rot[2],  # yaw
                axes="rxyz"
            )
            return self._generate_control_signals(
                self.lock_initial_sm_pos, fallback_quat, sm_pos_offset, sm_rot_offset
            )
        else:
            if marker_id % 2 == 0:  # Even marker IDs: side approach
                marker_base_offset = np.array([0.0, -0.25, 0.0])
                rotation_sequence = [
                    np.array([np.pi/2, np.pi/4, 0]),
                    np.array([0, 0, np.pi/2]),
                ]
            else:
                marker_base_offset = np.array([-0.45, 0.25, 0.4])
                rotation_sequence = [
                    # np.array([-np.pi/2, 0, 0]),
                    np.array([-np.pi, 0, 0]),
                    np.array([0, -np.pi/4, 0]),
                ]

            marker_pos_offset = marker_pos + marker_base_offset
            marker_quat_offset = self._apply_sequential_rotations(
                    marker_quat,
                    rotation_sequence
                )

            target_world_pos, target_world_quat = self.perception.camera_to_world_transform(
                marker_pos_offset, marker_quat_offset, "main_arm"
            )

        return self._generate_control_signals(
            target_world_pos, target_world_quat, sm_pos_offset, sm_rot_offset
        )

    def key_callback(self, key):
        print(f"Key pressed: {key}")

        # Unlock end-effector state from controller
        if key == ord("1"):
            self.lock_ee_state = False
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
            self.gripper_closed_val = 1.0
            return True        
        return False

    def run_task(self, demo_duration: int):
        self.timer_running = True
        time_thread = threading.Thread(target=self.sleep_for, args=(demo_duration,))
        time_thread.daemon = True
        time_thread.start()

        targets: Dict[str, Target] = {"ur5": Target(), "base": Target()}
        frame_counter = 0

        with mujoco.viewer.launch_passive(self.model, self.data, key_callback=self.key_callback) as viewer:
            self.viewer = viewer
            while self.timer_running and viewer.is_running():
                # Set the target values for the robot's devices
                x, y, z, roll, pitch, yaw, button1_pressed, button2_pressed = (
                    self.sm.update_state()
                )

                if button2_pressed and not self.prev_button2_pressed:
                    # First cycle to the next marker
                    self.current_aruco_idx = (self.current_aruco_idx + 1) % len(self.aruco_ids)
                    current_marker_id = self.aruco_ids[self.current_aruco_idx]

                    # Then lock to the new marker
                    self.lock_ee_state = True
                    self.lock_initial_sm_pos = np.array([x, y, z])
                    self.lock_initial_sm_rot = np.array([roll, pitch, yaw])
                    approach_type = "side" if current_marker_id % 2 == 0 else "top"
                    print(f"Cycled to marker {current_marker_id} and locked with {approach_type} approach")

                self.prev_button2_pressed = button2_pressed

                if frame_counter % self.aruco_detection_interval == 0:
                    camera_image = self.perception.get_camera_image("main_arm")
                    marker_pos, marker_quat = self.perception.detect_aruco_marker(
                        camera_image, marker_id=self.aruco_ids[self.current_aruco_idx]
                    )

                frame_counter += 1

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

                if self.lock_ee_state:
                    # Calculate space mouse offsets from initial locked position
                    sm_pos_offset = np.array([x, y, z]) - self.lock_initial_sm_pos
                    sm_rot_offset = (
                        np.array([roll, pitch, yaw]) - self.lock_initial_sm_rot
                    )

                    # Convert rotation offset to quaternion
                    sm_rot_quat = euler2quat(
                        sm_rot_offset[1],
                        sm_rot_offset[0],
                        sm_rot_offset[2],
                        axes="rxyz",
                    )

                    ctrlr_output = self.lock_ee_to_target(
                        self.aruco_ids[self.current_aruco_idx],
                        marker_pos,
                        marker_quat,
                        sm_pos_offset,
                        sm_rot_quat,
                    )

                else:
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
                        if not self.lock_ee_state:
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
    sim = GrabAndMove(
        robot_config_file="ur5_xyz_abg.yaml", scene_file="space_mouse_scene_ur5.xml"
    )
    sim.run_task(400)

import numpy as np
from threading import Lock
from typing import Dict, Any, Callable
from enum import Enum
import mujoco
import copy


class DeviceState(Enum):
    Q = "Q"
    Q_ACTUATED = "Q_ACTUATED"
    DQ = "DQ"
    DQ_ACTUATED = "DQ_ACTUATED"
    DDQ = "DDQ"
    EE_XYZ = "EE_XYZ"
    EE_XYZ_VEL = "EE_XYZ_VEL"
    EE_QUAT = "EE_QUAT"
    FORCE = "FORCE"
    TORQUE = "TORQUE"
    J = "JACOBIAN"


class Device:
    """
    The Device class encapsulates the device parameters specified in the yaml file
    that is passed to MujocoApp. It collects data from the simulator, obtaining the
    desired device states.
    """

    def __init__(
        self,
        device_yml: Dict,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        use_sim: bool,
    ):
        self.model = model
        self.data = data
        self.__use_sim = use_sim

        # Assign all of the yaml parameters
        self.name = device_yml["name"]
        self.max_vel = device_yml.get("max_vel")
        self.EE = device_yml["EE"]
        self.EE_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, self.EE
        )
        if self.EE_body_id == -1:
            raise ValueError(f"End-effector body '{self.EE}' not found in the model.")

        self.ctrlr_dof_xyz = device_yml["ctrlr_dof_xyz"]
        self.ctrlr_dof_abg = device_yml["ctrlr_dof_abg"]
        self.ctrlr_dof = np.hstack([self.ctrlr_dof_xyz, self.ctrlr_dof_abg])

        self.start_angles = np.array(device_yml["start_angles"])
        self.num_gripper_joints = device_yml["num_gripper_joints"]

        # Check if the user specifies a start body for the while loop to terminte at
        try:
            start_body_name = device_yml["start_body"]
            self.start_body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, start_body_name
            )
            if self.start_body_id == -1:
                raise ValueError(
                    f"Start body '{start_body_name}' not found in the model."
                )
        except (KeyError, ValueError):
            self.start_body_id = 0

        self._initialize_joint_chain()

        if self.name in ["ur5right", "ur5left", "base"] and len(self.joint_ids) > 0:
            if len(self.start_angles) != len(self.joint_ids):
                raise ValueError(
                    f"Start angles length {len(self.start_angles)} does not match joint IDs length {len(self.joint_ids)} for device '{self.name}'."
                )
            else:
                self.data.qpos[self.joint_ids] = np.copy(self.start_angles)

        # Check that the controller DoF specified is not larger than the number of joints
        if sum(self.ctrlr_dof) > len(self.joint_ids):
            raise ValueError(
                f"Controller DoF {self.ctrlr_dof} exceeds number of joints {len(self.joint_ids)} for device '{self.name}'."
            )

        self._initialize_state_tracking()

    def _initialize_joint_chain(self) -> None:
        """
        Initialize the joint chain of the device, starting from the end-effector
        and going backwards to the base of the arm.
        """
        current_body_id = self.EE_body_id
        joint_ids = []
        joint_names = []
        while (
            self.model.body_parentid[current_body_id] != 0
            and self.model.body_parentid[current_body_id] != self.start_body_id
            and current_body_id != self.start_body_id
        ):
            if self.model.body_jntnum[current_body_id] > 0:
                jntadrs_start = self.model.body_jntadr[current_body_id]
                tmp_ids = []
                tmp_names = []

                for i in range(self.model.body_jntnum[current_body_id]):
                    joint_idx = jntadrs_start + i
                    tmp_ids.append(joint_idx)
                    tmp_names.append(
                        mujoco.mj_id2name(
                            self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_idx
                        )
                    )
                joint_ids.extend(tmp_ids[::-1])
                joint_names.extend(tmp_names[::-1])

            current_body_id = self.model.body_parentid[current_body_id]

        # flip the list for base to ee ordering
        self.joint_names = joint_names[::-1]
        self.joint_ids = np.array(joint_ids[::-1], dtype=np.int32)
        print(self.joint_ids)
        print(self.joint_names)

        self._initialize_gripper_joints()
        self._map_actuators_to_joints()

    def _initialize_gripper_joints(self) -> None:
        """Initialize gripper joint IDs"""
        if len(self.joint_ids) > 0 and self.num_gripper_joints > 0:
            # Find the first gripper joint ID based on the last arm joint
            last_arm_joint_id = self.joint_ids[-1]
            # Check if gripper joints exist in sequence after the arm joints
            self.gripper_ids = []

            for i in range(self.num_gripper_joints):
                next_joint_id = last_arm_joint_id + 1 + i
                # Verify this is actually a joint in the model
                if next_joint_id < self.model.njnt:
                    self.gripper_ids.append(next_joint_id)
                else:
                    print(
                        f"Warning: Expected gripper joint {i} (ID {next_joint_id}) not found in model"
                    )

            self.gripper_ids = np.array(self.gripper_ids, dtype=np.int32)
        else:
            self.gripper_ids = np.array([], dtype=np.int32)

        # Combine arm and gripper joint IDs
        self.joint_ids_all = np.hstack([self.joint_ids, self.gripper_ids])

    def _map_actuators_to_joints(self) -> None:
        """Map actuators to the joints they control"""
        # Extract transmission target IDs for all actuators
        actuator_trnids = self.model.actuator_trnid[:, 0]

        # Find which actuators control our joints
        indices = []
        self.actuator_trnids = []

        for i, trnid in enumerate(actuator_trnids):
            if trnid in self.joint_ids_all:
                indices.append(i)
                self.actuator_trnids.append(trnid)

        self.ctrl_idxs = np.array(indices, dtype=np.int32)
        self.actuator_trnids = np.array(self.actuator_trnids, dtype=np.int32)

    def _initialize_state_tracking(self) -> None:
        """Initialize state tracking variables and functions"""
        # Create mapping from state enum to accessor function
        self.__state_var_map: Dict[DeviceState, Callable[[], np.ndarray]] = {
            DeviceState.Q: lambda: self.data.qpos[self.joint_ids_all],
            DeviceState.Q_ACTUATED: lambda: self.data.qpos[self.joint_ids],
            DeviceState.DQ: lambda: self.data.qvel[self.joint_ids_all],
            DeviceState.DQ_ACTUATED: lambda: self.data.qvel[self.joint_ids],
            DeviceState.DDQ: lambda: self.data.qacc[self.joint_ids_all],
            DeviceState.EE_XYZ: lambda: np.copy(self.data.xpos[self.EE_body_id]),
            DeviceState.EE_XYZ_VEL: lambda: self._get_body_velocity(),
            DeviceState.EE_QUAT: lambda: self._get_body_quaternion(),
            DeviceState.FORCE: lambda: self._get_force(),
            DeviceState.TORQUE: lambda: self._get_torque(),
            DeviceState.J: lambda: self._get_jacobian(),
        }

        # Initialize state storage and locks
        self.__state: Dict[DeviceState, np.ndarray] = {}
        self.__state_locks: Dict[DeviceState, Lock] = {
            key: Lock() for key in DeviceState
        }

        # Commonly accessed state variables for get_all_states()
        self.concise_state_vars = [
            DeviceState.Q_ACTUATED,
            DeviceState.DQ_ACTUATED,
            DeviceState.EE_XYZ,
            DeviceState.EE_XYZ_VEL,
            DeviceState.EE_QUAT,
            DeviceState.FORCE,
            DeviceState.TORQUE,
        ]

    def _get_body_velocity(self) -> np.ndarray:
        return np.copy(self.data.cvel[self.EE_body_id][:3])

    def _get_body_quaternion(self) -> np.ndarray:
        return np.copy(self.data.xquat[self.EE_body_id])

    def _get_jacobian(self, full=False):
        """
        NOTE: Returns either:
        1) The full jacobian (of the Device, using its EE), if full==True
        2) The full jacobian evaluated at the controlled DoF, if full==False
        """
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.EE_body_id)
        
        # Stack linear and angular Jacobians (vertical stack for 6×nv)
        J_full = np.vstack([jacp, jacr])
        
        if not full:
            # Create proper boolean mask with correct dimensions
            bool_indices = np.array(self.ctrlr_dof, dtype=bool)
            if len(bool_indices) != J_full.shape[0]:
                print(f"Warning: Jacobian has {J_full.shape[0]} rows but control DoF mask has {len(bool_indices)} elements")
                # Use min length to avoid index error
                min_len = min(len(bool_indices), J_full.shape[0])
                bool_indices = bool_indices[:min_len]
            J = J_full[bool_indices]
        else:
            J = J_full
            
        return J

    def _get_site_rotation(self, site_name: str) -> np.ndarray:
        """
        Get the rotation matrix of a site in the device's frame.
        """
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id == -1:
            raise ValueError(f"Site '{site_name}' not found in the model.")
        return self.data.site_xmat[site_id].reshape(3, 3)

    def _get_force(self):
        """
        Get the external forces acting upon the gripper sensors.
        Used for admittance control.
        """
        if self.name == "ur5right":
            ft_frame = "ft_frame"
            sensor_range = slice(0, 3)
        elif self.name == "ur5left":
            ft_frame = "ft_frame_ur5left"
            sensor_range = slice(6, 9)
        else:
            return np.zeros(3)

        try:
            # Get the rotation matrix for the force-torque sensor frame
            R = self._get_site_rotation(ft_frame)
            # Transform force from sensor frame to world frame
            force = R @ self.data.sensordata[sensor_range]
            return force
        except Exception as e:
            print(f"Error getting force for {self.name}: {e}")
            return np.zeros(3)

    def _get_torque(self):
        """
        Get the external torques acting upon the gripper sensors.
        Used for admittance control.
        """
        if self.name == "ur5right":
            ft_frame = "ft_frame"
            sensor_range = slice(3, 6)
        elif self.name == "ur5left":
            ft_frame = "ft_frame_ur5left"
            sensor_range = slice(9, 12)
        else:
            return np.zeros(3)

        try:
            # Get the rotation matrix for the force-torque sensor frame
            R = self._get_site_rotation(ft_frame)
            # Transform torque from sensor frame to world frame
            torque = R @ self.data.sensordata[sensor_range]
            return torque
        except Exception as e:
            print(f"Error getting torque for {self.name}: {e}")
            return np.zeros(3)

    def __set_state(self, state_var: DeviceState):
        """
        Set the state of the device corresponding to the key value (if exists)
        """
        if self.__use_sim:
            raise RuntimeError("__set_state should not be called when use_sim=True")

        with self.__state_locks[state_var]:
            var_func = self.__state_var_map[state_var]
            var_value = var_func()
            self.__state[state_var] = copy.copy(var_value)

    def get_state(self, state_var: DeviceState):
        """
        Get the state of the device corresponding to the key value.

        Args:
            state_var: The state variable to retrieve

        Returns:
            The state as a numpy array
        """
        if self.__use_sim:
            # Get state directly from the simulator
            func = self.__state_var_map[state_var]
            return func()
        else:
            # Get state from cached values
            with self.__state_locks[state_var]:
                if state_var not in self.__state:
                    # Initialize the state if it doesn't exist
                    self.__set_state(state_var)
                return copy.copy(self.__state[state_var])

    def get_all_states(self):
        return dict([(key, self.get_state(key)) for key in self.concise_state_vars])

    def update_state(self):
        """
        Update all device states. This should be called from a separate thread.
        """
        if self.__use_sim:
            raise RuntimeError("update_state should not be called when use_sim=True")

        for var in DeviceState:
            self.__set_state(var)

    def get_all_joint_ids(self):
        return self.joint_ids_all

    def get_actuator_joint_ids(self):
        return self.joint_ids

    def get_gripper_joint_ids(self):
        return self.gripper_ids

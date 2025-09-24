import numpy as np
import mujoco
import time
from shared_control.device import Device, DeviceState
from enum import Enum
from threading import Lock
from typing import Dict, Any, List, Callable
import copy

class RobotState(Enum):
    M = 'INERTIA'
    DQ = 'DQ'
    J = 'JACOBIAN'

class Robot():
    def __init__(self, sub_devices: List[Device], robot_name, model: mujoco.MjModel, data: mujoco.MjData, use_sim, collect_hz=1000):
        self.model = model
        self.data = data
        self.__use_sim = use_sim
        self.sub_devices = sub_devices
        self.sub_devices_dict: Dict[str, Device] = dict()
        for dev in self.sub_devices:
            self.sub_devices_dict[dev.name] = dev

        self.name = robot_name
        self.num_scene_joints = self.model.nv
        self.M_vec = np.zeros(self.num_scene_joints**2)

        self.joint_ids_all = np.array([], dtype=np.int32)
        for dev in self.sub_devices:
            self.joint_ids_all = np.hstack([self.joint_ids_all, dev.joint_ids_all])
        self.joint_ids_all = np.sort(np.unique(self.joint_ids_all))
        self.num_joints_total = len(self.joint_ids_all)

        self.running = False
        self.__state_locks: Dict[RobotState, Lock] = dict([(key, Lock()) for key in RobotState])
        self.__state_var_map: Dict[RobotState, Callable[[], Any]] = {
            RobotState.M : lambda : self.__get_M(),
            RobotState.DQ : lambda : self.__get_dq(),
            RobotState.J : lambda : self.__get_jacobian()
        }
        self.__state: Dict[RobotState, Any] = dict()
        self.data_collect_hz = collect_hz

    
    def __get_jacobian(self):
        """
            Return the Jacobians for all of the devices,
            so that OSC can stack them according to provided the target entries
        """
        Js = dict()
        J_idxs = dict()
        start_idx = 0
        for name, device in self.sub_devices_dict.items():
            J_sub = device.get_state(DeviceState.J)
            J_idxs[name] = np.arange(start_idx, start_idx + J_sub.shape[0])
            start_idx += J_sub.shape[0]
            J_sub = J_sub[:, self.joint_ids_all]
            Js[name] = J_sub
        return Js, J_idxs
    
    def __get_dq(self):
        dq = np.zeros(self.joint_ids_all.shape)
        for dev in self.sub_devices:
            for i, jnt_id in enumerate(dev.get_all_joint_ids()):
                idx = np.where(self.joint_ids_all == jnt_id)[0]
                if len(idx) > 0:
                    dq[idx[0]]  = dev.get_state(DeviceState.DQ)[i]
        return dq

    def __get_M(self):
        M_full = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, M_full, self.data.qM)

        M = M_full[np.ix_(self.joint_ids_all, self.joint_ids_all)]
        return M

    def get_state(self, state_var: RobotState):
        if self.__use_sim:
            func = self.__state_var_map[state_var]
            state = copy.copy(func())
        else:
            with self.__state_locks[state_var]:
                state = copy.copy(self.__state[state_var])
        return state

    def __set_state(self, state_var: RobotState):

        if self.__use_sim:
            raise RuntimeError("Cannot set state when using simulation mode")

        with self.__state_locks[state_var]:
            func = self.__state_var_map[state_var]
            value = func()
            self.__state[state_var] = copy.copy(value) # Make sure to copy (or else reference will stick to Dict value)

    def is_running(self):
        return self.running
    
    def is_using_sim(self):
        return self.__use_sim

    def __update_state(self):

        if self.__use_sim:
            raise RuntimeError("Cannot set state when using simulation mode")
        for var in RobotState:
            self.__set_state(var)

    def start(self):

        if self.running or self.__use_sim:
            raise RuntimeError("Cannot start the robot when it is already running or in simulation mode")

        self.running = True
        interval = 1.0 / self.data_collect_hz
        prev_time = time.time()

        while self.running:
            for dev in self.sub_devices:
                dev.update_state()

            self.__update_state()

            curr_time = time.time()
            diff = curr_time - prev_time
            delay = max(interval - diff, 0)
            time.sleep(delay)
            prev_time = curr_time

    def stop(self):
        if not self.running or self.__use_sim:
            raise RuntimeError("Cannot stop the robot when it is not running or in simulation mode")
        self.running = False

    def get_device(self, device_name: str) -> Device:
        if device_name not in self.sub_devices_dict:
            raise ValueError(f"Device '{device_name}' not found in robot '{self.name}'")
        return self.sub_devices_dict[device_name]

    def get_all_states(self):
        """
        Get's the state of all the devices connected plus the robot states
        """
        state = {}
        for device_name, device in self.sub_devices_dict.items():
            state[device_name] = device.get_all_states()

        for key in RobotState:
            state[key.value] = self.get_state(key)

        return state

    def get_device_states(self):
        """
        Get's the state of all the devices connected
        """
        state = {}
        for device_name, device in self.sub_devices_dict.items():
            state[device_name] = device.get_all_states()
        return state

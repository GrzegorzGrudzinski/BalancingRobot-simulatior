import pybullet as p
import numpy as np
from sim_components.sensors.base_sensor import BaseEncoder


class IdealEncoder(BaseEncoder):
    def read_all(self, robot_id: int, joint_indices: list[int]) -> float:
        left_wheel = p.getJointState(robot_id, joint_indices[0])
        right_wheel = p.getJointState(robot_id, joint_indices[1])
        
        return left_wheel[0], right_wheel[0], left_wheel[1], right_wheel[1]
    
    def reset(self, initial_angle: float) -> None:
        pass


class NoisyEncoder(BaseEncoder):
    pass
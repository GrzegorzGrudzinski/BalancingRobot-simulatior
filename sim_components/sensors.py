import pybullet as p
import numpy as np
from abc import ABC, abstractmethod

class BaseSensor(ABC):
    @abstractmethod
    def read(self, robot_id: int, y_axis_num: int = 0) -> float:
        pass
    
    def reset(self, initial_angle: float) -> None:
        pass

class IdealSensor(BaseSensor):
    def __init__(self):
        pass
    def read(self, robot_id: int, y_axis_num: int = 0) -> float:
        _, orientation = p.getBasePositionAndOrientation(robot_id)
        return p.getEulerFromQuaternion(orientation)[y_axis_num]

    def reset(self, initial_angle: float) -> None:
        pass

class NoisySensor(BaseSensor):
    def __init__(self, alpha: float = 0.85):
        self.rng = np.random.default_rng()
        self.alpha = alpha  # filter value
        self._sensor_bias = 0.0
        self._y_angle = 0.0

    def reset(self, initial_angle: float) -> None:
        self._y_angle = initial_angle
        self._sensor_bias = 0.0

    def read(self, robot_id: int, y_axis_num: int = 0) -> float:
        _, orn = p.getBasePositionAndOrientation(robot_id)
        sim_angle = p.getEulerFromQuaternion(orn)[y_axis_num]

        self._sensor_bias += self.rng.normal(0.0, 0.0001)   # Temperature drift
        noise = self.rng.normal(0.0, np.deg2rad(0.5))       # Random noise
        real_angle = sim_angle + noise + self._sensor_bias  # 
        
        self._y_angle = self.alpha * self._y_angle + (1 - self.alpha) * real_angle # low-pass filter
        return self._y_angle

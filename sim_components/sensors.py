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
        _, orientation = p.getBasgetBasePositionAndOrientation(robot_id)
        return p.getEulerFromQuaternion(orientation)[y_axis_num]

    def reset(self, initial_angle: float) -> None:
        pass

class NoisySensor(BaseSensor):
    def __init__(self, alpha: float = 0.85):
        self.rng = np.random.default_rng()
        self.alpha = alpha
        self._sensor_bias = 0.0
        self._y_angle = 0.0

    def reset(self, initial_angle: float) -> None:
        self._y_angle = initial_angle
        self._sensor_bias = 0.0

    def read(self, robot_id: int, y_axis_num: int = 0) -> float:
        _, orn = p.getBasePositionAndOrientation(robot_id)
        sim_angle = p.getEulerFromQuaternion(orn)[y_axis_num]

        self._sensor_bias += self.rng.normal(0.0, 0.0001)
        noise = self.rng.normal(0.0, np.deg2rad(0.5))
        real_angle = sim_angle + noise + self._sensor_bias
        
        self._y_angle = self.alpha * self._y_angle + (1 - self.alpha) * real_angle
        return self._y_angle

'''

self._position, self._orientation = p.getBasePositionAndOrientation(self._id)
        sim_angle =  p.getEulerFromQuaternion(self._orientation)[self._y_axis_num] # obrów w osi Y

        self._sensor_bias += rng.normal(0.0, 0.0001)
        noise = rng.normal(0.0, np.deg2rad(0.5))
        real_angle = sim_angle + noise + self._sensor_bias
        
        alpha = 0.85
        self._y_angle = alpha * self._y_angle + (1-alpha) * real_angle


'''
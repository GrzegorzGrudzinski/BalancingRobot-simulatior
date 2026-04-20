import numpy as np
from abc import ABC, abstractmethod

class BaseIMU(ABC):
    @abstractmethod
    def read(self, robot_id: int, y_axis_num: int = 0) -> float:
        pass
    
    def reset(self, initial_angle: float) -> None:
        pass

class BaseEncoder(ABC):
    @abstractmethod
    def read_all(self, robot_id: int, joint_nr: int) -> tuple[float, float]:
        pass
    
    def reset(self) -> None:
        pass
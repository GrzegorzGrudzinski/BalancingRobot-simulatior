import numpy as np
from abc import ABC, abstractmethod

class BaseController(ABC):
    @abstractmethod
    def compute(self, setpoint: float, current_value: float, dt: float) -> float:
        pass
    
    @abstractmethod
    def reset(self) -> None:
        pass
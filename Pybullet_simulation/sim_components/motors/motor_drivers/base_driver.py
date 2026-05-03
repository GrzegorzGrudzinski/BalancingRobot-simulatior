from abc import ABC, abstractmethod

class BaseDriver(ABC):
    def __int__(self):
        pass
    
    @abstractmethod
    def _compute(self, setpoint: float, current_value: float, dt: float) -> float:
        pass
    
    @abstractmethod
    def apply(self) -> None:
        pass
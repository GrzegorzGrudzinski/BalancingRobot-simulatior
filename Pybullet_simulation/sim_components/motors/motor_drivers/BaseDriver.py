from abc import ABC, abstractmethod

class BaseDriver(ABC):
    def __init__(self):
        pass
    
    @abstractmethod
    def _compute(self) -> None:
        pass
    
    @abstractmethod
    def apply(self) -> None:
        pass
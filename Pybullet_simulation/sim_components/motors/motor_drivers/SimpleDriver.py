import numpy as np
from sim_components.motors.motor_drivers.BaseDriver import BaseDriver

class SimpleDriver(BaseDriver):
    def __init__(self):
        pass
    
    def _compute(self) -> None:
        # pass
        print("compute")
    
    def apply(self) -> None:
        self._compute()
        print("apply")
import numpy as np
from sim_components.motors.motor_drivers import base_driver

class simple_driver(base_driver):
    def __int__(self):
        pass
    
    def _compute(self, setpoint: float, current_value: float, dt: float) -> float:
        pass
    
    def apply(self) -> None:
        pass
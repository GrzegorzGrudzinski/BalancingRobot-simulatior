'''
    pid.py
'''

import numpy as np
from sim_components.controllers.base_controller import BaseController

class PID(BaseController):
    def __init__(self, max_output: float, Kp: float = 1.0, Ki: float = 0.0, Kd: float = 0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.output_min = -max_output
        self.output_max = max_output
        
        self.integral = 0.0
        self.last_input = 0.0 

    def compute(self, setpoint: float, current_value: float, dt: float) -> float:
        sp = np.deg2rad(setpoint)
        error = sp - current_value
        
        P = self.Kp * error
        
        if self.Ki != 0:
            self.integral += error * dt
            # Anti-Windup
            self.integral = max(min(self.integral, self.output_max / self.Ki), 
                                self.output_min / self.Ki)
        I = self.Ki * self.integral
        
        # Derivative on Measurement
        D = -self.Kd * (current_value - self.last_input) / dt
        
        output = P + I + D
        output = max(min(output, self.output_max), self.output_min)
        
        self.last_input = current_value
        return output

    def reset(self):
        self.integral = 0.0
        self.last_input = 0.0
'''
    cascade_pid.py
'''

import numpy as np
from sim_components.controllers.base_controller import BaseController
from sim_components.controllers.pid import PID  # Importujemy Twój zwykły PID!

class CascadePID(BaseController):
    def __init__(self, 
                 max_output: float, 
                 max_pitch_angle: float = 15.0, # Zabezpieczenie: max kąt pochylenia w stopniach
                 pos_Kp: float = 0.0, pos_Ki: float = 0.0, pos_Kd: float = 0.0,
                 ang_Kp: float = 1.0, ang_Ki: float = 0.0, ang_Kd: float = 0.0):
        
        # 1. Pętla zewnętrzna (POZYCJA -> DOCELOWY KĄT)
        # Jej maksymalnym wyjściem nie jest prąd, a kąt! 
        # Zabezpiecza to robota przed próbą "położenia się na płasko", by dogonić pozycję.
        self.position_pid = PID(max_output=max_pitch_angle, 
                                Kp=pos_Kp, Ki=pos_Ki, Kd=pos_Kd)
        
        # 2. Pętla wewnętrzna (KĄT -> MOMENT / PRĄD)
        # Dokładnie to samo, co Twój dotychczasowy zwykły PID
        self.angle_pid = PID(max_output=max_output, 
                             Kp=ang_Kp, Ki=ang_Ki, Kd=ang_Kd)

    def compute(self, 
                target_position: float, current_position: float, 
                current_pitch: float, dt: float) -> float:
        """
        Zwraca wyliczony moment obrotowy (torque).
        """
        # Krok 1: Wyliczamy jak mocno robot musi się pochylić, by zniwelować błąd pozycji
        # (Zauważ, że tu docelowym setpointem jest pozycja)
        target_pitch = self.position_pid.compute(setpoint=target_position, 
                                                 current_value=current_position, 
                                                 dt=dt)
        
        # UWAGA NA ZNAKI!
        # W zależności od Twojego układu współrzędnych, może być konieczne 
        # odwrócenie znaku: target_pitch = -target_pitch 
        # (Jeśli robot uciekł do przodu, musi pochylić się w tył).

        # Krok 2: Podajemy wyliczony kąt do wewnętrznego PID-a balansującego
        torque = self.angle_pid.compute(setpoint=target_pitch, 
                                        current_value=current_pitch, 
                                        dt=dt)
        
        return torque

    def reset(self):
        """Resetuje pamięć całkowania obu kontrolerów."""
        self.position_pid.reset()
        self.angle_pid.reset()
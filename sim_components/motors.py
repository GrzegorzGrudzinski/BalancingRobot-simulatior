import pybullet as p
import numpy as np
from collections import deque
from abc import ABC, abstractmethod

class BaseMotor(ABC):
    @abstractmethod
    def move(self, target_torque: float) -> float:
        pass
    def reset(self) -> None:
        pass
        
class IdealMotor(BaseMotor):
    def __init__(self, max_torque: float):
        self.max_torque = max_torque

    def move(self, val: float) -> float:
        return max(min(val, self.max_torque), -self.max_torque)



class RealMotor(BaseMotor):
    def __init__(self, max_torque: float, deadband_ratio: float,
                 noise: float = 0.0, delay: int = 0, 
                 asymmetry: float = 0.0 ):
        self.rng = np.random.default_rng()
        self.deadband_ratio = deadband_ratio
        self.noise = noise
        self.asymmetry = asymmetry
        self.base_max_torque = max_torque
        
        self.delay = delay
        self.buffer = deque( [0.0]*max(1,delay), maxlen=max(1,delay) )
        self.reset()

    def move(self, val: float) -> float:
        self.buffer.append(val)
        delayed_torque = self.buffer[0] if self.delay > 0 else val

        torque = max(min(delayed_torque, self.max_torque), -self.max_torque)
        
        if abs(torque) < self.deadband:
                return 0.0
        
        if self.noise > 0:
            torque += self.rng.normal(0, self.noise)
        
        return torque
    
    def reset(self) -> None:
        self.buffer.extend( [0.0] * max(1, self.delay) )

        bias = self.rng.uniform(-self.asymmetry, self.asymmetry)
        self.max_torque = self.base_max_torque + (1.0 + bias)
        self.deadband = self.deadband_ratio * self.max_torque




'''    TODO      '''
class FOCMotor(BaseMotor):
    def __init__(self, torque_constant: float, kv_rating: float, supply_voltage: float, 
                 bandwidth_hz: float, dt: float = 1/240.0, 
                 noise: float = 0.001, delay_steps: int = 1, asymmetry_variance: float = 0.0):
        
        self.torque_constant = torque_constant
        self.kv_rating = kv_rating
        self.supply_voltage = supply_voltage
        self.noise_std = noise
        self.delay_steps = delay_steps
        self.asymmetry_variance = asymmetry_variance
        
        self.max_rad_s = (self.kv_rating * self.supply_voltage) * (2.0 * np.pi / 60.0)
        
        rc = 1.0 / (2.0 * np.pi * bandwidth_hz)
        self.alpha = dt / (rc + dt)
        
        self.rng = np.random.default_rng()
        self.command_buffer = deque([0.0] * max(1, delay_steps), maxlen=max(1, delay_steps))
        
        self.reset()

    def reset(self) -> None:
        self.command_buffer.extend([0.0] * max(1, self.delay_steps))
        self.current_actual_torque = 0.0
        
        # Domain Randomization asymetrii
        bias = self.rng.uniform(-self.asymmetry_variance, self.asymmetry_variance)
        # Indywidualna stała momentu dla tego koła (np. jedno ma 0.023, drugie 0.0225)
        self.actual_kt = self.torque_constant * (1.0 + bias)

    # UWAGA: target_val to teraz wartość z PID (np. 1.0 Amper), a nie gotowe Nm!
    def move(self, target_val: float, current_speed: float = 0.0) -> float:
        # 1. Opóźnienie komunikacji
        self.command_buffer.append(target_val)
        delayed_cmd_amps = self.command_buffer[0] if self.delay_steps > 0 else target_val

        # 2. KONWERSJA: Ampery ze sterownika na Niutonometry dla symulacji
        target_torque_nm = delayed_cmd_amps * self.actual_kt

        # 3. Limit Back-EMF
        speed_ratio = abs(current_speed) / self.max_rad_s
        # Z fizycznego punktu widzenia Back-EMF zmniejsza dostępny max_torque.
        # W uproszczeniu: max teoretyczny moment to prąd_zwarcia * Kt. 
        # Ponieważ ograniczasz prąd do 1A programowo, Back-EMF wpłynie na Ciebie dopiero, 
        # gdy wygenerowane napięcie przeciwelektromotoryczne przekroczy (Napięcie Zasilania - Napięcie dla 1A).
        # Zostawmy to bezpieczne ograniczenie liniowe:
        max_available_nm = (self.supply_voltage * self.actual_kt) * max(0.0, 1.0 - speed_ratio)

        # 4. Nasycenie
        saturated_torque = max(min(target_torque_nm, max_available_nm), -max_available_nm)

        # 5. Opóźnienie pętli prądowej
        self.current_actual_torque += self.alpha * (saturated_torque - self.current_actual_torque)
        
        # 6. Szum mechaniczny
        output_torque = self.current_actual_torque
        if self.noise_std > 0:
            output_torque += self.rng.normal(0, self.noise_std)
            
        return output_torque
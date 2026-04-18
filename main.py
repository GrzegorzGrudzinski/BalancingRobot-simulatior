'''
    pybullet_simulation.py
'''

from sim_components.config import AppConfig, RunMode, ControllerType, SensorType
from sim_components.runner import AppRunner


'''
TODO:
    - add a messaging system to interact with physical robot
    (start, stop, reset, setPID etc)

updatePos:
    - sim / IMU

resetPos:
    - add a STOP signal for robot
setPID:
    - sim / MCU    
'''


if __name__ == "__main__":
    config = AppConfig(
        mode= RunMode.SIMULATION,
        controller= ControllerType.PID,
        sensor= SensorType.IMU_NOISY
    )

    config.ctrl_params.pid_kp = 10
    config.ctrl_params.pid_ki = 0
    config.ctrl_params.pid_kd = 0

    app = AppRunner(config)
    app.run()
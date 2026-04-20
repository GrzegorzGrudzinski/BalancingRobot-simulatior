'''
    pybullet_simulation.py
'''

from sim_components.config import AppConfig, RunMode, ControllerType, ImuType, EncoderType, MotorType
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
        sensor_imu= ImuType.IMU_NOISY,
        sensor_encoder= EncoderType.IDEAL,
        motors= MotorType.REAL
    )
    config.sim_params.disturb_force = 40
    config.sim_params.disturb_interval = 600

    # config.motor_params.MAX_TORQUE = 0.0
    # config.motor_params.DELAY = 0
    # config.motor_params.ASYMMETRY = 0
    # config.motor_params.NOISE  = 0
    # config.motor_params.DEADBAND_RATIO  = 0

    config.ctrl_params.pid_kp = 40
    # config.ctrl_params.pid_ki = 0.5
    # config.ctrl_params.pid_kd = 0.5  

    app = AppRunner(config)
    app.run()
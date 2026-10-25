'''
    main.py

    pybullet simulation
'''

from sim_components import *
from sim_components.runner import AppRunner

if __name__ == "__main__":
    config = AppConfig(
        mode= RunMode.SIMULATION,
        controller= ControllerType.PID,
        sensor_imu= ImuType.IMU_NOISY,
        sensor_encoder= EncoderType.IDEAL,
        driver= DriverType.OPEN_LOOP,
        motors= MotorType.REAL,
        motor_profile="bldc_5010_360kV"
    )

    sp = config.sim_params
    sp.seed = 0                         # None -> different run every time
    sp.duration_s = 30.0
    sp.gui, sp.realtime = True, True    # watch the robot
    # sp.gui, sp.realtime = False, False  # batch run: no window, as fast as possible
    sp.camera_tracking = True if sp.gui == True else False

    config.sim_params.start_pitch_deg = None # 5.0
    config.sim_params.log_csv = "logs/run.csv"   # None -> no log

    
    sp.disturb_force = 0
    sp.disturb_interval_s = 0.83 
    sp.disturb_time_s = 0.083

    # config.motor_params.VISCOUS_FRICTION = 0.5
    # config.motor_params.COULOMB_FRICTION = 0.5

    config.motor_params.DELAY_STEPS = 1
    # config.motor_params.ASYMMETRY = 0.0
    # config.motor_params.NOISE  = 0.0
    # config.motor_params.DEADBAND_RATIO  = 0.0

    # firmware pitch loop (config.h), deg -> A
    config.ctrl_params.pid_kp = 0.8
    config.ctrl_params.pid_ki = 0.0
    config.ctrl_params.pid_kd = 0.04 

    app = AppRunner(config)
    app.run()



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

    #dodać czas trwania zakłócenia
'''

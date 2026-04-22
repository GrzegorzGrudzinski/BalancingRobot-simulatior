'''
    runner.py
'''

from sim_components.config import AppConfig, RunMode, RobotConfig
from sim_components.builder import RobotBuilder

class AppRunner:
    def __init__(self, config: AppConfig):
        self.config = config

        self.controller = RobotBuilder.create_controller(config)
        self.sensor_imu = RobotBuilder.create_imu_sensor(config)
        self.sensor_encoder = RobotBuilder.create_encoder_sensor(config)
        self.motors = RobotBuilder.create_motor(config)

        self.robot_cfg = RobotConfig(
            controller= self.controller,
            sensor_imu= self.sensor_imu,
            sensor_encoder= self.sensor_encoder,
            motors = self.motors
        )

    def run(self):
        if self.config.mode == RunMode.SIMULATION:
            self._run_simulation()
        elif self.config.mode == RunMode.PHYSICAL:
            self._run_physical()

    def _run_simulation(self):
        from sim_components.sim import Simulation
        from sim_components.robot import Robot

        sim = Simulation()
        sim.load_plane("plane.urdf")

        sim.enable_wireframe(self.config.sim_params.show_wireframe)
        sim.enable_camera_tracking(self.config.sim_params.camera_tracking)

        # BASE_PATH = 
        URDF_PATH = "./robot.urdf"

        robot = Robot(URDF_PATH, config=self.robot_cfg, scale=1)
        robot.enable_com_display(self.config.sim_params.show_com)
        
        
        sim.attach_robot(robot)
        sim.set_disturbances(force=self.config.sim_params.disturb_force, 
                             interval_steps=self.config.sim_params.disturb_interval)

        sim.start()
        sim.run(max_steps=10000, freq=200)
        sim.disconnect()

    def _run_physical(self):
        pass
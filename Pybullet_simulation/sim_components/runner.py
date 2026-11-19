'''
    runner.py

    run the app
'''

from sim_components.config import AppConfig, RunMode, RobotConfig
from sim_components.builder import RobotBuilder

import numpy as np # for rng
class AppRunner:
    def __init__(self, config: AppConfig):
        self.config = config
        # one seed -> independent streams for every random component
        rng_imu, rng_motor, self._rng_robot, self._rng_sim = \
            np.random.default_rng(config.sim_params.seed).spawn(4)
        
        self.controller = RobotBuilder.create_controller(config)
        self.sensor_imu = RobotBuilder.create_imu_sensor(config, rng_imu)
        self.sensor_encoder = RobotBuilder.create_encoder_sensor(config)
        self.driver = RobotBuilder.create_driver(config)
        self.motors = RobotBuilder.create_motor(config, rng_motor)

        self.robot_cfg = RobotConfig(
            controller= self.controller,
            sensor_imu= self.sensor_imu,
            sensor_encoder= self.sensor_encoder,
            driver= self.driver,
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

        sp = self.config.sim_params
        sim = Simulation(gui=sp.gui, realtime=sp.realtime, rng=self._rng_sim)
        sim.load_plane("plane.urdf")    

        sim.enable_wireframe(self.config.sim_params.show_wireframe)
        sim.enable_camera_tracking(self.config.sim_params.camera_tracking)

        URDF_PATH = "./robot.urdf"

        robot = Robot(URDF_PATH, config=self.robot_cfg, scale=1, rng=self._rng_robot, sim_cfg=sp)
        robot.enable_com_display(self.config.sim_params.show_com)
        
        
        sim.attach_robot(robot)
        sim.set_disturbances(force=self.config.sim_params.disturb_force, 
                             interval_s=self.config.sim_params.disturb_interval_s,
                             time_s=self.config.sim_params.disturb_time_s
                            )

        from sim_components.logger import CsvLogger
        logger = CsvLogger(sp.log_csv) if sp.log_csv else None

        controller = None
        if sp.sil_dir and sp.sil_full_core:
            from sim_components.controllers.full_core import FullCoreController
            controller = FullCoreController(sp.sil_dir)
        elif sp.sil_dir:

            from sim_components.controllers.c_core import CCoreController
            controller = CCoreController(sp.sil_dir, friction=sp.sil_friction_comp)

        sim.start()
        try:
            sim.run(sp.duration_s, logger=logger, controller=controller)
        finally:
            if logger:
                logger.close()
            sim.disconnect()

    def _run_physical(self):
        pass
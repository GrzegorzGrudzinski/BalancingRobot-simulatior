'''
    sim.py
'''


import pybullet as p
import pybullet_data
import time
import numpy as np

from sim_components.robot import Robot

class Simulation:
    def __init__(self, timestep:float = 1./240. ) -> None:
        #
        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)

        self._camera_tracking = False
        self._show_wireframe = False 
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0) # domyślnie wyłączone
        
        self._timestep = timestep
        p.setTimeStep(timestep)

        # 
        self._robot: Robot | None = None
        self._is_running = False

        self._disturb_force = 0.0
        self._disturb_interval = 200
        self._disturb_time = 0

        self._temp_force: list[float] = [0.0, 0.0, 0.0]        
        self._temp_disturb_time: int = 0
        self._disturbance_line_id: int = -1 

    def enable_camera_tracking(self, enable: bool) -> None:
        self._camera_tracking = enable

    def enable_wireframe(self, enable: bool) -> None:
        self._show_wireframe = enable
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1 if enable else 0)

    def load_plane(self, urdf_path: str) -> int:
        self._id = p.loadURDF(urdf_path)
        p.changeDynamics(self._id, -1, lateralFriction=1.2)

    def attach_robot(self, robot: Robot) -> None:
        self._robot = robot

    def set_disturbances(self, force: float = 0, interval_steps: int = 200, time_steps: int = 0) -> None:
        """  """
        self._disturb_force = force
        self._disturb_interval = interval_steps
        self._disturb_time = time_steps

    def reset_disturbances(self) -> None:
        """  """
        self._temp_disturb_time = 0
        self._temp_force = 0
        p.removeUserDebugItem(self._disturbance_line_id)
        self._disturbance_line_id = -1

    def start(self) -> None:
        self._is_running = True

    def stop(self) -> None:
        self._is_running = False

    def _start_disturbance(self):
        if self._robot and self._disturb_force > 0:
            self._temp_disturb_time = int(np.random.uniform(0,
                                                            max(0,self._disturb_time)))
            self._temp_force = np.random.uniform(-self._disturb_force,
                                                 self._disturb_force, 
                                                 size=3).tolist()


    def _apply_disturbance(self) -> None:
            if self._robot and self._temp_disturb_time > 0:
                self._robot.apply_disturbance(self._temp_force)
                self._temp_disturb_time -= 1
                
                # Draw the force vector
                pos = self._robot.position
                scale = 0.05 
                end_pos = [ pos[0] + self._temp_force[0] * scale, 
                            pos[1] + self._temp_force[1] * scale, 
                            pos[2] + self._temp_force[2] * scale ]
            
                if self._disturbance_line_id < 0:
                    self._disturbance_line_id = p.addUserDebugLine(pos, end_pos, lineColorRGB=[1, 0.5, 0], lineWidth=4)
                else:
                    self._disturbance_line_id = p.addUserDebugLine(pos, end_pos, lineColorRGB=[1, 0.5, 0], lineWidth=4, replaceItemUniqueId=self._disturbance_line_id)

            # remove the line
            elif self._disturbance_line_id >= 0:
                p.removeUserDebugItem(self._disturbance_line_id)
                self._disturbance_line_id = -1

    def update_camera_tracking(self):
        """Aktualizuje pozycję kamery, aby śledziła robota."""
        if self._robot is None:
            return
        base_pos, _ = p.getBasePositionAndOrientation(self._robot._id)
        
        # --- Camera config ---
        distance = 1.0   # Distance (in m)
        yaw = 50         # Horizontal rotation (0 = in line with X axsis, 90 = with Y axis)
        pitch = -35      # Vertical rotation (-90 = view from top, 0 = horizontally)

        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=base_pos
        )

    def run(self, max_steps: int = 10000, freq: float = 100.0) -> None:
        if not self._robot:
            print("Error: robot not connected")
            return

        dt = 1.0 / freq
        self._robot.set_dt(dt)
        time_accumulator = 0.0

        for step in range(max_steps):
            time_accumulator += self._timestep

            if self._is_running and time_accumulator >= dt:
                self._robot.update()
                time_accumulator -= dt
            
            if self._camera_tracking:
                self.update_camera_tracking()
    
            if step % 10 == 0:
                self._robot.draw_debug_data()

            # Disturbances
            if self._robot.reset_flag:
                self.reset_disturbances()
            else:
                if self._disturb_force > 0 and step % self._disturb_interval == 0:
                    self._start_disturbance()
                self._apply_disturbance()

            p.stepSimulation()
            time.sleep(self._timestep)

    def disconnect(self):
        p.disconnect()

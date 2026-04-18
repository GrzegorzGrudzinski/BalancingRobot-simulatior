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
        self.physicsClient = p.connect(p.GUI)
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

    def set_disturbances(self, force: float = 0, interval_steps: int = 200) -> None:
        """  """
        self._disturb_force = force
        self._disturb_interval = interval_steps

    def start(self) -> None:
        self._is_running = True

    def stop(self) -> None:
        self._is_running = False

    def _trigger_disturbance(self) -> None:
        if self._robot and self._disturb_force > 0:
            force = np.random.uniform(-self._disturb_force, self._disturb_force, size=3).tolist()
            self._robot.apply_disturbance(force)
                
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
            if self._disturb_force > 0 and step % self._disturb_interval == 0:
                self._trigger_disturbance()

            p.stepSimulation()
            time.sleep(self._timestep)

    def disconnect(self):
        p.disconnect()

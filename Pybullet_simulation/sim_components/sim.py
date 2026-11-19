'''
    sim.py
'''


import pybullet as p
import pybullet_data
import time
import numpy as np
import math # for .isclose

from sim_components.robot import Robot

class Simulation:
    def __init__(self, timestep: float = 1./1000., ctrl_freq: float = 200.0, gui: bool = True, realtime: bool = True,
                 rng=None) -> None:
        self._gui = gui
        self._realtime = realtime
        self._rng = rng if rng is not None else np.random.default_rng()
        self.physicsClient = p.connect(p.GUI if gui else p.DIRECT)
        if gui:
            # graphical look only (these calls dont change anything in physocs)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # hide side panels
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1) # shadows on
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0) # wireframe off by default
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)

        self._camera_tracking = False
        self._show_wireframe = False 
        
        self._timestep = timestep
        p.setTimeStep(timestep)
        # controller period must be a whole number of physics steps (5 ms = 5 x 1 ms)
        self._ctrl_every = round(1.0 / (ctrl_freq * timestep))
        if not math.isclose(self._ctrl_every * timestep * ctrl_freq, 1.0):
            raise ValueError(f"1/ctrl_freq is not a multiple of the physics step {timestep}")
        self._physics_step = 0 # physics steps counter
        self._t_start: float | None = None  # wall-clock time of physics step 0 (realtime pacing)

        # 
        self._robot: Robot | None = None
        self._is_running = False

        self._disturb_force = 0.0
        self._disturb_interval = 200 # physics steps
        self._disturb_time = 0 # physics steps

        self._temp_force: list[float] = [0.0, 0.0, 0.0]        
        self._temp_disturb_time: int = 0
        self._disturbance_line_id: int = -1 

    @property
    def timestep(self) -> float: # external access to the timestep
        return self._timestep
    
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
        self._robot.set_dt(self._ctrl_every * self._timestep)

    @property
    def time(self) -> float:
        return self._physics_step * self._timestep

    def set_disturbances(self, force: float = 0, interval_s: int = 0.83, time_s: int = 0) -> None:
        """Times in seconds, stored in physics steps."""
        self._disturb_force = force
        self._disturb_interval = max(1, round(interval_s / self._timestep))
        self._disturb_time = round(time_s / self._timestep)

    def reset_disturbances(self) -> None:
        """  """
        self._temp_disturb_time = 0
        self._temp_force = [0.0, 0.0, 0.0]
        p.removeUserDebugItem(self._disturbance_line_id)
        self._disturbance_line_id = -1

    def start(self) -> None:
        self._is_running = True

    def stop(self) -> None:
        self._is_running = False

    def _start_disturbance(self):
        if self._robot and self._disturb_force > 0: 
            self._temp_disturb_time = int(self._rng.uniform(0,
                                                            max(0,self._disturb_time)))
            self._temp_force = self._rng.uniform(-self._disturb_force,
                                                 self._disturb_force, 
                                                 size=3).tolist()

    def _apply_disturbance(self, step: int) -> None:
        if self._robot and self._temp_disturb_time > 0:
            self._robot.apply_disturbance(self._temp_force)
            self._temp_disturb_time -= 1
            # redraw every 20 ms (debug drawing is slow in GUI mode)
            if self._gui and (self._disturbance_line_id < 0 or step % 20 == 0):
                self._draw_disturbance()
        elif self._disturbance_line_id >= 0:
            p.removeUserDebugItem(self._disturbance_line_id)
            self._disturbance_line_id = -1

    def _draw_disturbance(self) -> None:
        pos = self._robot.position
        scale = 0.05
        end_pos = [pos[i] + self._temp_force[i] * scale for i in range(3)]
        if self._disturbance_line_id < 0:
            self._disturbance_line_id = p.addUserDebugLine(
                pos, end_pos, lineColorRGB=[1, 0.5, 0], lineWidth=4)
        else:
            self._disturbance_line_id = p.addUserDebugLine(
                pos, end_pos, lineColorRGB=[1, 0.5, 0], lineWidth=4,
                replaceItemUniqueId=self._disturbance_line_id)

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

    def step(self, command: tuple[float, float] | None = None) -> dict:
        """One controller period: controller update + ctrl_every physics steps.
        command = (out_l, out_r) [A] from outside, None = robot's own controller.
        Returns the robot state after the period."""
        self._robot.update(command)
        if self._robot.reset_flag:
            self.reset_disturbances()

        if self._t_start is None:  # first call: wall-clock reference for realtime pacing
            self._t_start = time.perf_counter() - self.time

        for _ in range(self._ctrl_every):
            if self._disturb_force > 0 and self._physics_step % self._disturb_interval == 0:
                self._start_disturbance()
            self._apply_disturbance(self._physics_step)

            self._robot.apply_motors()  # every physics step: zero-order hold + command delay
            p.stepSimulation()
            self._physics_step += 1
            if self._realtime:
                # sleep until the wall-clock time of the next step (overhead does not accumulate)
                delay = self._t_start + self.time - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.1:
                    # far behind (breakpoint, window dragged): resync instead of fast-forwarding
                    self._t_start -= delay

        if self._gui:
            # debug drawing is slow in GUI mode: camera every 20 ms, CoM points every 10 ms
            if self._camera_tracking and self._physics_step % 20 == 0:
                self.update_camera_tracking()
            if self._physics_step % 10 == 0:
                self._robot.draw_debug_data()

        state = self._robot.read_state()
        state["time"] = self.time
        state["fallen"] = self._robot.reset_flag
        state["measured_pitch"] = np.rad2deg(self._robot.measured_pitch)
        state["out_l"], state["out_r"] = self._robot.last_out
        state["disturb_fy"] = self._temp_force[1] if self._temp_disturb_time > 0 else 0.0
        return state

    def run(self, duration_s: float, logger=None, controller=None) -> None:
        """controller: object with step(state, dt) -> (out_l, out_r), reset() and debug, e.g. CCoreController.
        None = the robot's own Python controller."""

        if not self._robot:
            print("Error: robot not connected")
            return

        dt = self._ctrl_every * self._timestep
        n_periods = round(duration_s / (self._ctrl_every * self._timestep))
        state = self._robot.read_state()  # what the controller sees before the first period
        for _ in range(n_periods):
            if self._gui and hasattr(controller, "command_pos"):
                keys = p.getKeyboardEvents()
                fwd = bool(keys.get(p.B3G_UP_ARROW, 0) & p.KEY_IS_DOWN)
                back = bool(keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_IS_DOWN)
                controller.command_pos(0.5 * (fwd - back))   # [m]

                left  = bool(keys.get(p.B3G_LEFT_ARROW, 0) & p.KEY_IS_DOWN)
                right = bool(keys.get(p.B3G_RIGHT_ARROW, 0) & p.KEY_IS_DOWN)
                if hasattr(controller, "command_yaw"):
                    controller.command_yaw(0.2 * (left - right))   # [rad], CCW (left arrow) > 0, as test_core.c:161
            if not self._is_running:
                break
            command = controller.step(state, dt) if controller else None
            state = self.step(command)
            if controller:
                state.update(controller.debug)
                if state["fallen"]:
                    controller.reset()  # next step() re-arms like CMD_START on the robot
            if logger:
                logger.write(state)


    def disconnect(self):
        p.disconnect()

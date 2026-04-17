'''
    pybullet_simulation.py
'''

from sim_components.sim import Simulation
from sim_components.robot import Robot


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
    sim = Simulation()
    sim.load_plane("plane.urdf")

    robot = Robot("robot.urdf", scale=1)
    
    robot.configure_pid(kp=1500, kd=1)

    robot.enable_com_display(True)
    
    sim.attach_robot(robot)
    sim.set_disturbances(force=0.0, interval_steps=200)
    sim.start()

    sim.enable_camera_tracking(True)
    sim.enable_wireframe(False)

    sim.run(max_steps=10000, pid_freq=200)

    sim.disconnect()

"""Calibrate the SO101 follower WITHOUT ever enabling torque.

`lerobot-calibrate` calls robot.connect(), which runs configure() and re-enables
torque on every motor. Our servos had Goal_Position = 0, so that could make the
arm jump. Here we open the bus directly and run only robot.calibrate(), which
keeps torque off. Goal positions are also synced to present positions first.
"""
import sys

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

PORT = "/dev/cu.usbmodem5AE60826921"
ROBOT_ID = sys.argv[1] if len(sys.argv) > 1 else "my_follower"

robot = SOFollower(SOFollowerRobotConfig(port=PORT, id=ROBOT_ID))
bus = robot.bus
bus.connect()  # opens port + pings motors; does not touch torque
try:
    bus.disable_torque()
    # Belt and braces: make goal == present so any later torque-enable holds still.
    for motor in bus.motors:
        present = bus.read("Present_Position", motor, normalize=False)
        bus.write("Goal_Position", motor, present, normalize=False)
    print("Torque is OFF on all motors. You can move the arm freely by hand.\n")
    robot.calibrate()
finally:
    bus.disconnect(disable_torque=True)
    print("\nDone. Torque left OFF.")

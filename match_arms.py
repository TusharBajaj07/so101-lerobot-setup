"""Slowly move single joints (powered) so the follower and leader poses match before teleop.

  python match_arms.py leader:wrist_roll=21 follower:wrist_flex=-103 follower:gripper=5

Targets are in LeRobot units (degrees; gripper 0-100 %). One joint at a time, slow, 40 % torque,
stops on stall. Only use for joints that don't hold the arm up against gravity: torque goes OFF
right after each move (verified by readback).
"""
import sys
import time

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
from lerobot.teleoperators.so_leader.so_leader import SOLeader

ARMS = {
    "leader": lambda: SOLeader(SOLeaderTeleopConfig(port="/dev/cu.usbmodem5AB01585991", id="my_leader")),
    "follower": lambda: SOFollower(SOFollowerRobotConfig(port="/dev/cu.usbmodem5AE60826921", id="my_follower")),
}
GRAVITY_JOINTS = {"shoulder_lift", "elbow_flex"}  # releasing these mid-air would drop the arm
SPEED, ACCEL, TORQUE = 250, 10, 400  # ~22 deg/s, gentle ramp, 40 %
TOL, STALL_S, TIMEOUT_S = 3.0, 2.0, 25


def torque_off(bus, motor):
    for _ in range(5):
        try:
            bus.write("Torque_Enable", motor, 0, num_retry=2)
            if bus.read("Torque_Enable", motor, num_retry=2) == 0:
                return 0
        except Exception:
            pass
        time.sleep(0.05)
    return bus.read("Torque_Enable", motor, num_retry=5)


def move(bus, motor, target):
    start = bus.read("Present_Position", motor, num_retry=5)
    print(f"  {motor}: {start:.1f} -> {target}", flush=True)
    # After power-on a servo can ignore position goals until its mode is rewritten (seen on the
    # follower gripper and wrist). LeRobot's configure() does this on connect; do the same here.
    bus.disable_torque(motor)  # also unlocks EEPROM
    bus.write("Operating_Mode", motor, 0)
    bus.write("Acceleration", motor, ACCEL)
    bus.write("Goal_Velocity", motor, SPEED)
    bus.write("Torque_Limit", motor, TORQUE)
    bus.write("Goal_Position", motor, target)  # turns torque ON (Feetech), so limits went first
    bus.write("Torque_Enable", motor, 1)
    try:
        best, last_progress, t0 = abs(start - target), time.time(), time.time()
        while time.time() - t0 < TIMEOUT_S:
            now = bus.read("Present_Position", motor, num_retry=5)
            err = abs(now - target)
            if err <= TOL:
                print(f"    reached {now:.1f}", flush=True)
                return
            if err < best - 0.5:
                best, last_progress = err, time.time()
            elif time.time() - last_progress > STALL_S:
                print(f"    stopped at {now:.1f} (stalled), leaving it there", flush=True)
                return
            time.sleep(0.05)
        print(f"    timed out at {now:.1f}", flush=True)
    finally:
        print(f"    torque readback (0 = off): {torque_off(bus, motor)}", flush=True)


if __name__ == "__main__":
    jobs = []
    for arg in sys.argv[1:]:
        arm, rest = arg.split(":")
        motor, value = rest.split("=")
        if motor in GRAVITY_JOINTS:
            sys.exit(f"Refusing {motor}: it holds the arm up, and this script releases torque after the move.")
        jobs.append((arm, motor, float(value)))
    for s in range(10, 0, -1):
        print(f"Moving in {s}s...", flush=True)
        time.sleep(1)
    for arm in dict.fromkeys(a for a, _, _ in jobs):
        dev = ARMS[arm]()
        dev.bus.connect()  # opens port + pings; no torque change
        try:
            print(arm, flush=True)
            for a, motor, target in jobs:
                if a == arm:
                    move(dev.bus, motor, target)
        finally:
            dev.bus.port_handler.closePort()

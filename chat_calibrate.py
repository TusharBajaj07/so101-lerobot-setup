"""Two-step SO101 calibration that can be driven from a chat (no ENTER key needed).

Mirrors LeRobot's own calibrate() but splits it in two runs. Torque is never enabled.

  python chat_calibrate.py middle leader|follower   # arm held in middle pose -> writes homing offsets
  python chat_calibrate.py sweep  leader|follower   # records min/max until STOP file appears, then saves

Stop a sweep with:  touch .stop_<arm>
"""
import json
import sys
import time
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
from lerobot.teleoperators.so_leader.so_leader import SOLeader

HERE = Path(__file__).parent
ARMS = {
    "leader": ("/dev/cu.usbmodem5AB01585991", "my_leader"),
    "follower": ("/dev/cu.usbmodem5AE60826921", "my_follower"),
}
FULL_TURN_MOTOR = "wrist_roll"
SWEEP_TIMEOUT_S = 600


def make_device(arm):
    port, dev_id = ARMS[arm]
    if arm == "leader":
        return SOLeader(SOLeaderTeleopConfig(port=port, id=dev_id))
    return SOFollower(SOFollowerRobotConfig(port=port, id=dev_id))


def sync_goal_to_present(bus):
    # With goal == present, a later torque-enable holds the arm still instead of jumping.
    for motor in bus.motors:
        present = bus.read("Present_Position", motor, normalize=False)
        bus.write("Goal_Position", motor, present, normalize=False)


def middle(dev, state_file, delay_s=0):
    bus = dev.bus
    if delay_s:
        print(f"Capturing middle pose in {delay_s}s...", flush=True)
        time.sleep(delay_s)
    bus.disable_torque()
    for motor in bus.motors:
        bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    homing_offsets = bus.set_half_turn_homings()
    sync_goal_to_present(bus)
    state_file.write_text(json.dumps({m: int(v) for m, v in homing_offsets.items()}))
    pos = bus.sync_read("Present_Position", normalize=False)
    print("Homing offsets written:", homing_offsets)
    print("Positions now (should all be ~2047):", pos)


def sweep(dev, state_file, stop_file):
    if not state_file.exists():
        sys.exit("Run the 'middle' step first.")
    homing_offsets = json.loads(state_file.read_text())
    bus = dev.bus
    bus.disable_torque()
    motors = [m for m in bus.motors if m != FULL_TURN_MOTOR]

    pos = bus.sync_read("Present_Position", motors, normalize=False, num_retry=5)
    mins, maxes = dict(pos), dict(pos)
    stop_file.unlink(missing_ok=True)
    start = last_print = time.time()
    print("Recording. Move each joint (except wrist_roll) to both end stops.", flush=True)
    while not stop_file.exists() and time.time() - start < SWEEP_TIMEOUT_S:
        pos = bus.sync_read("Present_Position", motors, normalize=False, num_retry=5)
        for m in motors:
            mins[m] = min(mins[m], pos[m])
            maxes[m] = max(maxes[m], pos[m])
        if time.time() - last_print > 2:
            last_print = time.time()
            print(f"[{int(last_print - start):>3}s] " + "  ".join(
                f"{m}:{mins[m]}..{maxes[m]}(span {maxes[m] - mins[m]})" for m in motors), flush=True)
        time.sleep(0.02)
    if not stop_file.exists():
        sys.exit("Timed out before you said done. Nothing saved; rerun the sweep.")
    stop_file.unlink(missing_ok=True)

    same = [m for m in motors if mins[m] == maxes[m]]
    if same:
        sys.exit(f"Not saved: these joints never moved: {same}")
    mins[FULL_TURN_MOTOR], maxes[FULL_TURN_MOTOR] = 0, 4095

    dev.calibration = {
        m: MotorCalibration(id=mot.id, drive_mode=0, homing_offset=homing_offsets[m],
                            range_min=mins[m], range_max=maxes[m])
        for m, mot in bus.motors.items()
    }
    bus.write_calibration(dev.calibration)
    sync_goal_to_present(bus)
    dev._save_calibration()
    state_file.unlink(missing_ok=True)
    print("FINAL:", {m: (mins[m], maxes[m]) for m in bus.motors})
    print("Calibration saved to", dev.calibration_fpath)


if __name__ == "__main__":
    step, arm = sys.argv[1], sys.argv[2]
    dev = make_device(arm)
    dev.bus.connect()  # opens port + pings; never touches torque
    try:
        if step == "middle":
            middle(dev, HERE / f".homing_{arm}.json", int(sys.argv[3]) if len(sys.argv) > 3 else 0)
        elif step == "sweep":
            sweep(dev, HERE / f".homing_{arm}.json", HERE / f".stop_{arm}")
        else:
            sys.exit(f"unknown step {step}")
    finally:
        dev.bus.disconnect(disable_torque=True)

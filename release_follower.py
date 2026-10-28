"""Turn follower torque off ONLY if the arm is resting folded; otherwise refuse (it would drop).

Use after teleop runs with --robot.disable_torque_on_disconnect=false, which leaves the follower holding.
"""
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = "/dev/cu.usbmodem5AE60826921"  # follower (black arm)
LIFT, ELBOW = 2, 3
TORQUE_EN, PRESENT_POS = 40, 56

port, ph = PortHandler(PORT), PacketHandler(0)
port.openPort()
port.setBaudRate(1_000_000)
time.sleep(0.1)
port.clearPort()
try:
    lift, elbow = (ph.read2ByteTxRx(port, m, PRESENT_POS)[0] for m in (LIFT, ELBOW))
    if not (lift < 1150 and elbow > 2900):
        raise SystemExit(f"NOT releasing: arm isn't resting folded (shoulder {lift}, elbow {elbow}). "
                         "Bring it to rest first (via the leader), then run this again.")
    off = {}
    for m in range(1, 7):  # retry each joint until a clean readback says off
        for _ in range(5):
            ph.write1ByteTxRx(port, m, TORQUE_EN, 0)
            time.sleep(0.02)
            port.clearPort()
            v, r, _ = ph.read1ByteTxRx(port, m, TORQUE_EN)
            if r == COMM_SUCCESS and v == 0:
                break
        off[m] = v if r == COMM_SUCCESS else "ERR"
    print(f"At rest (shoulder {lift}, elbow {elbow}). Torque readback (0 = off): {off}")
finally:
    port.closePort()

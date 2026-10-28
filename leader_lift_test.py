"""Powered test: lift the leader arm to its calibrated middle pose ("L"), hold, lower back, release.

Motors 2 (shoulder) + 3 (elbow) move to 2047; motor 4 (wrist) holds still so the handle doesn't flop.
The arm is lowered back to where it started before torque goes off, so it doesn't drop.
Emergency stop = unplug the 12 V barrel jack (the arm will then fall to rest).
"""
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = "/dev/cu.usbmodem5AB01585991"  # leader
SHOULDER, ELBOW, WRIST = 2, 3, 4
LIFT_TARGET = {SHOULDER: 2047, ELBOW: 2047}  # calibrated middle pose
TORQUE_LIMIT = {SHOULDER: 500, ELBOW: 400, WRIST: 300}  # 50% / 40% / 30%
SPEED, ACCEL = 150, 10   # ~13 deg/s
STALL_S = 1.5            # no progress for this long -> abort
SAFE_RANGE = {SHOULDER: (767, 3311), ELBOW: (950, 3156)}

ACC, GOAL_POS, GOAL_SPEED, TORQUE_LIM, TORQUE_EN, PRESENT_POS = 41, 42, 46, 48, 40, 56

port, ph = PortHandler(PORT), PacketHandler(0)
port.openPort()
port.setBaudRate(1_000_000)


def w(mid, addr, v, size=2):
    fn = ph.write2ByteTxRx if size == 2 else ph.write1ByteTxRx
    res, _ = fn(port, mid, addr, v)
    assert res == COMM_SUCCESS, f"motor {mid}: {ph.getTxRxResult(res)}"


def pos(mid):
    v, res, _ = ph.read2ByteTxRx(port, mid, PRESENT_POS)
    assert res == COMM_SUCCESS, f"motor {mid}: {ph.getTxRxResult(res)}"
    return v


def move(targets, label):
    print(f"{label}: {targets}", flush=True)
    for mid, t in targets.items():
        lo, hi = SAFE_RANGE[mid]
        assert lo <= t <= hi, f"motor {mid} target {t} outside {lo}..{hi}"
        w(mid, GOAL_POS, t)
    last = {m: pos(m) for m in targets}
    last_progress = time.time()
    t0 = time.time()
    while time.time() - t0 < 10:
        now = {m: pos(m) for m in targets}
        if all(abs(now[m] - t) <= 20 for m, t in targets.items()):
            print(f"  reached {now}", flush=True)
            return
        if any(abs(now[m] - last[m]) > 5 for m in targets):
            last, last_progress = now, time.time()
        elif time.time() - last_progress > STALL_S:
            raise RuntimeError(f"Stalled at {now} (target {targets}): too heavy or blocked")
        time.sleep(0.05)
    raise RuntimeError(f"Timed out at {now}")


motors = (SHOULDER, ELBOW, WRIST)
start = {}
torque_on = False
try:
    for s in range(10, 0, -1):
        print(f"Lifting in {s}s... (emergency stop = unplug 12 V power)", flush=True)
        time.sleep(1)
    start = {m: pos(m) for m in motors}
    print(f"Start: {start}", flush=True)
    for m in motors:
        w(m, GOAL_POS, start[m])     # hold still when torque comes on
        w(m, ACC, ACCEL, size=1)
        w(m, GOAL_SPEED, SPEED)
        w(m, TORQUE_LIM, TORQUE_LIMIT[m])
    for m in motors:
        w(m, TORQUE_EN, 1, size=1)
    torque_on = True
    move(LIFT_TARGET, "Lifting to L pose")
    print("Holding 3 s...", flush=True)
    time.sleep(3)
    move({SHOULDER: start[SHOULDER], ELBOW: start[ELBOW]}, "Lowering back")
    print("Test complete.", flush=True)
except Exception as e:
    print(f"ABORT: {e}", flush=True)
    if torque_on and start:
        try:
            move({SHOULDER: start[SHOULDER], ELBOW: start[ELBOW]}, "Lowering back after abort")
        except Exception as e2:
            print(f"Could not lower cleanly: {e2}", flush=True)
finally:
    for m in motors:
        ph.write1ByteTxRx(port, m, TORQUE_EN, 0)
    print("Torque OFF on motors 2, 3, 4.", flush=True)
    port.closePort()

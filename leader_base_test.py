"""Small powered test: rotate leader motor 1 (base) ~20 deg each way, slowly, low torque.

Only motor 1 gets torque. Torque is always turned off at the end (finally block).
Emergency stop = unplug the 12 V barrel jack (USB unplug does NOT make it limp).
"""
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = "/dev/cu.usbmodem5AB01585991"  # leader
MOTOR_ID = 1
STEP = 228            # counts, ~20 deg (4096 counts = 360 deg)
SPEED = 200           # counts/s, ~18 deg/s
ACCEL = 10
TORQUE_LIMIT = 300    # 30% of max
MIN_POS, MAX_POS = 773, 3401  # calibrated range stored in the motor
LOAD_ABORT = 500      # abort if load exceeds ~50%

# STS3215 registers
ACC, GOAL_POS, GOAL_SPEED, TORQUE_LIM, TORQUE_EN = 41, 42, 46, 48, 40
PRESENT_POS, PRESENT_LOAD = 56, 60

port, ph = PortHandler(PORT), PacketHandler(0)
port.openPort()
port.setBaudRate(1_000_000)


def w1(addr, v):
    res, err = ph.write1ByteTxRx(port, MOTOR_ID, addr, v)
    assert res == COMM_SUCCESS, ph.getTxRxResult(res)


def w2(addr, v):
    res, err = ph.write2ByteTxRx(port, MOTOR_ID, addr, v)
    assert res == COMM_SUCCESS, ph.getTxRxResult(res)


def r2(addr):
    v, res, _ = ph.read2ByteTxRx(port, MOTOR_ID, addr)
    assert res == COMM_SUCCESS, ph.getTxRxResult(res)
    return v


def load():
    raw = r2(PRESENT_LOAD)
    return raw & 0x3FF  # bit 10 = direction


def move_to(target):
    print(f"  -> moving to {target}", flush=True)
    w2(GOAL_POS, target)
    t0 = time.time()
    while time.time() - t0 < 5:
        pos, ld = r2(PRESENT_POS), load()
        if ld > LOAD_ABORT:
            raise RuntimeError(f"Load {ld} too high at pos {pos}: something is blocking the arm")
        if abs(pos - target) <= 10:
            print(f"     reached {pos} (load {ld})", flush=True)
            time.sleep(0.5)
            return
        time.sleep(0.05)
    raise RuntimeError(f"Did not reach {target} in 5 s (at {r2(PRESENT_POS)})")


try:
    for s in range(10, 0, -1):
        print(f"Moving in {s}s... (emergency stop = unplug 12 V power)", flush=True)
        time.sleep(1)
    start = r2(PRESENT_POS)
    lo, hi = start - STEP, start + STEP
    if lo < MIN_POS or hi > MAX_POS:
        raise RuntimeError(f"Start {start}: +/-{STEP} would leave the safe range {MIN_POS}..{MAX_POS}")
    print(f"Start position {start}. Torque ON for motor {MOTOR_ID} only.", flush=True)
    w2(GOAL_POS, start)          # hold still when torque comes on
    w1(ACC, ACCEL)
    w2(GOAL_SPEED, SPEED)
    w2(TORQUE_LIM, TORQUE_LIMIT)
    w1(TORQUE_EN, 1)
    for target in (hi, start, lo, start):
        move_to(target)
    print("Test complete.", flush=True)
finally:
    try:
        ph.write1ByteTxRx(port, MOTOR_ID, TORQUE_EN, 0)
        print("Torque OFF.", flush=True)
    finally:
        port.closePort()

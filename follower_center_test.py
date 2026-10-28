"""Powered test: move the follower to the centre of every calibrated range (LeRobot's 0 deg), hold, return, release.

Stage 1 moves the light joints (base, wrist roll, gripper); stage 2 lifts shoulder + elbow + wrist flex together.
All six motors hold their start position when torque comes on, so nothing flops or jumps.
Start with the arm resting folded. Torque goes off ONLY after the arm is back at that rest pose;
on any error it is lowered under power, and if that fails it HOLDS (torque on) -- it is never dropped.
Emergency stop = unplug the 12 V barrel jack (this WILL drop the arm, so support it if you can).
"""
import json
import time
from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = "/dev/cu.usbmodem5AE60826921"  # follower (black arm)
CAL = json.loads((Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json").read_text())
IDS = {name: c["id"] for name, c in CAL.items()}
SAFE_RANGE = {c["id"]: (c["range_min"], c["range_max"]) for c in CAL.values()}
CENTER = {mid: (lo + hi) // 2 for mid, (lo, hi) in SAFE_RANGE.items()}  # = 0 deg (gripper 50%) in LeRobot
TORQUE_LIMIT = {1: 400, 2: 500, 3: 400, 4: 300, 5: 300, 6: 250}
STAGES = [("Stage 1: base, wrist roll, gripper", (1, 5, 6)),
          ("Stage 2: shoulder, elbow, wrist flex", (2, 3, 4))]
SPEED, ACCEL = 150, 10   # ~13 deg/s
TOLERANCE = 40           # counts; gravity left the leader ~2.5 deg short at 50% torque
STALL_S = 1.5            # no progress for this long -> abort
LIFT_JOINTS = (2, 3, 4)  # shoulder, elbow, wrist flex: the ones gravity pulls down
REST_TOLERANCE = 60      # counts from the start (rest) pose before torque may go off

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
        if not lo <= t <= hi:
            # A resting joint can sit a few counts past its swept stop; clamp instead of aborting,
            # because aborting here cuts torque with the arm lifted and drops it.
            print(f"  motor {mid}: target {t} outside {lo}..{hi}, using {min(max(t, lo), hi)}", flush=True)
            targets[mid] = t = min(max(t, lo), hi)
        w(mid, GOAL_POS, t)
    last = {m: pos(m) for m in targets}
    last_progress = time.time()
    t0 = time.time()
    while time.time() - t0 < 15:
        now = {m: pos(m) for m in targets}
        if all(abs(now[m] - t) <= TOLERANCE for m, t in targets.items()):
            print(f"  reached {now}  (error {[now[m] - t for m, t in targets.items()]})", flush=True)
            return
        if any(abs(now[m] - last[m]) > 5 for m in targets):
            last, last_progress = now, time.time()
        elif time.time() - last_progress > STALL_S:
            raise RuntimeError(f"Stalled at {now} (target {targets}): too heavy or blocked")
        time.sleep(0.05)
    raise RuntimeError(f"Timed out at {now}")


def at_rest(start):
    now = {m: pos(m) for m in LIFT_JOINTS}
    ok = all(abs(now[m] - min(max(start[m], SAFE_RANGE[m][0]), SAFE_RANGE[m][1])) <= REST_TOLERANCE
             for m in LIFT_JOINTS)
    print(f"Rest check: now {now}, start {[start[m] for m in LIFT_JOINTS]} -> {'at rest' if ok else 'NOT at rest'}",
          flush=True)
    return ok


def lower_patiently(targets):
    # No stall abort here: aborting again would leave the arm up. Keep pushing toward rest for up to 30 s.
    targets = {m: min(max(t, SAFE_RANGE[m][0]), SAFE_RANGE[m][1]) for m, t in targets.items()}
    print(f"Lowering back after abort: {targets}", flush=True)
    for m, t in targets.items():
        w(m, GOAL_POS, t)
    t0 = time.time()
    while time.time() - t0 < 30:
        if all(abs(pos(m) - t) <= REST_TOLERANCE for m, t in targets.items()):
            return
        time.sleep(0.1)


motors = tuple(SAFE_RANGE)
start = {}
torque_on = False
try:
    for s in range(15, 0, -1):
        print(f"Moving in {s}s... (emergency stop = unplug 12 V power)", flush=True)
        time.sleep(1)
    start = {m: pos(m) for m in motors}
    print(f"Start: {start}", flush=True)
    print(f"Centre targets: {CENTER}", flush=True)
    for m in motors:
        w(m, ACC, ACCEL, size=1)
        w(m, GOAL_SPEED, SPEED)
        w(m, TORQUE_LIM, TORQUE_LIMIT[m])
        w(m, GOAL_POS, start[m])     # writing a goal turns torque ON (Feetech), so limits go first
    for m in motors:
        w(m, TORQUE_EN, 1, size=1)
    torque_on = True
    for label, ids in STAGES:
        move({m: CENTER[m] for m in ids}, label)
    print("At centre. Holding 5 s -- check the pose now.", flush=True)
    time.sleep(5)
    for label, ids in reversed(STAGES):
        move({m: start[m] for m in ids}, "Returning " + label.split(": ")[1])
    print("Test complete.", flush=True)
except Exception as e:
    print(f"ABORT: {e}", flush=True)
    if torque_on and start:
        try:
            lower_patiently({m: start[m] for m in LIFT_JOINTS})
        except Exception as e2:
            print(f"Could not lower: {e2}", flush=True)
finally:
    # NEVER cut torque with the arm raised: it drops. Release only once it is back at rest.
    try:
        resting = not torque_on or at_rest(start)
    except Exception as e:
        print(f"Could not check rest pose: {e}", flush=True)
        resting = False
    if resting:
        for m in motors:
            ph.write1ByteTxRx(port, m, TORQUE_EN, 0)
        print("Arm at rest. Torque readback (0 = off):",
              {m: ph.read1ByteTxRx(port, m, TORQUE_EN)[0] for m in motors}, flush=True)
    else:
        for m in motors:
            v, r, _ = ph.read2ByteTxRx(port, m, PRESENT_POS)
            if r == COMM_SUCCESS:
                ph.write2ByteTxRx(port, m, GOAL_POS, v)  # hold right here (keeps torque on)
        print("HOLDING with torque ON: arm is not back at rest. Support the arm by hand before releasing it.",
              flush=True)
    port.closePort()

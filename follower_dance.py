"""Fun routine for the follower: wake up, stand tall like a giraffe, look around, chomp, wave, bow, sleep.

Safety rules (same as follower_center_test.py):
- Must start resting folded; the start pose is where it goes back to before release.
- Writing a goal turns torque ON (Feetech), so speed/limits are written before any goal.
- Torque goes off ONLY once the arm is back at rest. On any error it is lowered under power,
  and if that fails it HOLDS with torque on -- it is never dropped.
Emergency stop = unplug the 12 V barrel jack (this WILL drop the arm, so support it if you can).
"""
import json
import time
from pathlib import Path

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PORT = "/dev/cu.usbmodem5AE60826921"  # follower (black arm)
CAL = json.loads((Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json").read_text())
SAFE_RANGE = {c["id"]: (c["range_min"], c["range_max"]) for c in CAL.values()}
C = {mid: (lo + hi) // 2 for mid, (lo, hi) in SAFE_RANGE.items()}  # centre of each range (LeRobot 0 deg)
PAN, LIFT, ELBOW, WFLEX, WROLL, GRIP = 1, 2, 3, 4, 5, 6
DEG = 4096 / 360  # counts per degree
TORQUE_LIMIT = {PAN: 400, LIFT: 500, ELBOW: 400, WFLEX: 300, WROLL: 300, GRIP: 250}
SLOW, MED, FAST = 150, 300, 500  # counts/s: ~13, ~26, ~44 deg/s
ACCEL = 20
TOLERANCE = 40
STALL_S = 1.5
LIFT_JOINTS = (LIFT, ELBOW, WFLEX)
REST_TOLERANCE = 60

ACC, GOAL_POS, GOAL_SPEED, TORQUE_LIM, TORQUE_EN, PRESENT_POS = 41, 42, 46, 48, 40, 56

port, ph = PortHandler(PORT), PacketHandler(0)
port.openPort()
port.setBaudRate(1_000_000)


def w(mid, addr, v, size=2):
    fn = ph.write2ByteTxRx if size == 2 else ph.write1ByteTxRx
    for _ in range(3):
        res, _ = fn(port, mid, addr, v)
        if res == COMM_SUCCESS:
            return
        time.sleep(0.02)
    raise RuntimeError(f"motor {mid}: write {addr}={v} failed: {ph.getTxRxResult(res)}")


def pos(mid):
    for _ in range(5):
        try:
            v, res, _ = ph.read2ByteTxRx(port, mid, PRESENT_POS)
        except IndexError:  # SDK crashes on an empty reply instead of returning an error
            res = None
        if res == COMM_SUCCESS:
            return v
        time.sleep(0.02)
    raise RuntimeError(f"motor {mid}: no position reply")


def clamp(mid, t):
    lo, hi = SAFE_RANGE[mid]
    return min(max(int(t), lo), hi)


def move(targets, label="", speed=MED):
    targets = {m: clamp(m, t) for m, t in targets.items()}
    if label:
        print(label, flush=True)
    for m in targets:
        w(m, GOAL_SPEED, speed)
    for m, t in targets.items():
        w(m, GOAL_POS, t)
    last = {m: pos(m) for m in targets}
    last_progress = t0 = time.time()
    while time.time() - t0 < 15:
        now = {m: pos(m) for m in targets}
        if all(abs(now[m] - t) <= TOLERANCE for m, t in targets.items()):
            return
        if any(abs(now[m] - last[m]) > 5 for m in targets):
            last, last_progress = now, time.time()
        elif time.time() - last_progress > STALL_S:
            raise RuntimeError(f"Stalled at {now} (target {targets}): too heavy or blocked")
        time.sleep(0.03)
    raise RuntimeError(f"Timed out at {now}")


def at_rest(start):
    now = {m: pos(m) for m in LIFT_JOINTS}
    ok = all(abs(now[m] - clamp(m, start[m])) <= REST_TOLERANCE for m in LIFT_JOINTS)
    print(f"Rest check: now {now}, start {[start[m] for m in LIFT_JOINTS]} -> {'at rest' if ok else 'NOT at rest'}",
          flush=True)
    return ok


def lower_patiently(start):
    # No stall abort here: aborting again would leave the arm up. Fold the wrist/elbow, then lower, up to 30 s.
    print("Lowering back to rest after abort...", flush=True)
    for m in (WFLEX, ELBOW, LIFT):
        w(m, GOAL_SPEED, SLOW)
    for m in (WFLEX, ELBOW, LIFT):
        w(m, GOAL_POS, clamp(m, start[m]))
    t0 = time.time()
    while time.time() - t0 < 30 and not all(
            abs(pos(m) - clamp(m, start[m])) <= REST_TOLERANCE for m in LIFT_JOINTS):
        time.sleep(0.1)


def routine(start):
    tall = {LIFT: C[LIFT], ELBOW: C[ELBOW] - 85 * DEG, WFLEX: C[WFLEX]}  # forearm up in line: giraffe

    move({PAN: C[PAN], WROLL: C[WROLL], GRIP: C[GRIP]}, "1. Waking up...", SLOW)
    move({LIFT: C[LIFT], ELBOW: C[ELBOW], WFLEX: C[WFLEX]}, "   ...stretching into the L pose", SLOW)
    time.sleep(0.8)

    move(tall, "2. Standing tall like a giraffe", SLOW)
    time.sleep(1.2)

    print("3. Looking around, curious", flush=True)
    for pan, roll in ((-30, 20), (30, -20), (0, 0)):
        move({PAN: C[PAN] + pan * DEG, WROLL: C[WROLL] + roll * DEG}, speed=MED)
        time.sleep(0.6)

    print("4. Nodding hello", flush=True)
    for _ in range(2):
        move({WFLEX: C[WFLEX] + 20 * DEG}, speed=FAST)
        move({WFLEX: C[WFLEX] - 20 * DEG}, speed=FAST)
    move({WFLEX: C[WFLEX]}, speed=FAST)

    move({LIFT: C[LIFT], ELBOW: C[ELBOW], WFLEX: C[WFLEX]}, "5. Back to L pose for some bird chomps", SLOW)
    for _ in range(3):
        move({GRIP: 2700}, speed=FAST)
        move({GRIP: 1650}, speed=FAST)
    move({GRIP: C[GRIP]}, speed=FAST)

    print("6. Waving", flush=True)
    move({PAN: C[PAN] + 20 * DEG}, speed=MED)
    for _ in range(3):
        move({WROLL: C[WROLL] + 35 * DEG}, speed=FAST)
        move({WROLL: C[WROLL] - 35 * DEG}, speed=FAST)
    move({WROLL: C[WROLL], PAN: C[PAN]}, speed=MED)

    print("7. Taking a bow", flush=True)
    move({LIFT: C[LIFT] + 15 * DEG, WFLEX: C[WFLEX] + 35 * DEG}, speed=SLOW)
    time.sleep(1.0)
    move({LIFT: C[LIFT], WFLEX: C[WFLEX]}, speed=SLOW)

    print("8. Going back to sleep", flush=True)
    move({GRIP: start[GRIP], WROLL: start[WROLL], PAN: start[PAN]}, speed=SLOW)
    move({WFLEX: start[WFLEX]}, speed=SLOW)
    move({ELBOW: start[ELBOW], LIFT: start[LIFT]}, speed=SLOW)


motors = tuple(SAFE_RANGE)
start = {m: pos(m) for m in motors}
if not (start[LIFT] < 1150 and start[ELBOW] > 2900):
    port.closePort()
    raise SystemExit(f"Not starting: arm isn't resting folded (shoulder {start[LIFT]}, elbow {start[ELBOW]}). "
                     "Put it on its rest first. Torque untouched.")

torque_on = False
try:
    for s in range(10, 0, -1):
        print(f"Starting in {s}s... (emergency stop = unplug 12 V power)", flush=True)
        time.sleep(1)
    print(f"Start (rest) pose: {start}", flush=True)
    for m in motors:
        w(m, ACC, ACCEL, size=1)
        w(m, GOAL_SPEED, SLOW)
        w(m, TORQUE_LIM, TORQUE_LIMIT[m])
        w(m, GOAL_POS, start[m])  # writing a goal turns torque ON, so limits go first
    for m in motors:
        w(m, TORQUE_EN, 1, size=1)
    torque_on = True
    routine(start)
    print("Routine complete.", flush=True)
except Exception as e:
    print(f"ABORT: {e}", flush=True)
    if torque_on:
        try:
            lower_patiently(start)
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

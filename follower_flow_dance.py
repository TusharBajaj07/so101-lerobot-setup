"""Flowing dance for the follower: several joints move together on smooth curves, streamed at 50 Hz.

Each section eases (minimum-jerk) from the previous pose to its own base pose, and layers sine
motions on top that fade in and out, so every joint is continuous and nothing jumps.

  python follower_flow_dance.py --dry-run   # check ranges + speeds only, never touches the motors
  python follower_flow_dance.py             # run it

Safety (same rules as follower_center_test.py):
- Must start resting folded; that pose is where it goes back to before release.
- Every streamed goal is clamped to the calibrated range and speed-limited per step.
- If a joint falls behind, the choreography pauses until it catches up. If it stops making
  progress (blocked), it aborts, lowers under power, and only releases at rest; otherwise it
  HOLDS with torque on. It is never dropped.
Emergency stop = unplug the 12 V barrel jack (this WILL drop the arm, so support it if you can).
"""
import json
import math
import sys
import time
from pathlib import Path

CAL = json.loads((Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json").read_text())
SAFE_RANGE = {c["id"]: (c["range_min"], c["range_max"]) for c in CAL.values()}
C = {mid: (lo + hi) // 2 for mid, (lo, hi) in SAFE_RANGE.items()}
PAN, LIFT, ELBOW, WFLEX, WROLL, GRIP = 1, 2, 3, 4, 5, 6
MOTORS = (PAN, LIFT, ELBOW, WFLEX, WROLL, GRIP)
DEG = 4096 / 360
HZ = 50
MAX_STEP = 90 * DEG / HZ  # never command faster than 90 deg/s
TORQUE_LIMIT = {m: 800 for m in (PAN, LIFT, ELBOW, WFLEX, WROLL, GRIP)}  # 80% (user's choice)
# Note: the gripper's EEPROM Max_Torque_Limit is 500 (LeRobot's anti-burnout cap) and likely still caps it at 50%.
MAX_TEMP_C = 60  # stop and lower the arm if any servo gets this hot
DANCE_P = 32  # Feetech default; base gain test: lag 7.0 deg at P=8, 4.4 at 16, 2.7 at 32
PAUSE_ERR, RESUME_ERR = 150, 60  # counts: pause the choreography above ~13 deg lag, resume below ~5
STUCK_S = 2.0  # paused, a joint is > DROOP_OK behind and makes no progress for this long -> abort
DROOP_OK, SETTLE_S = 250, 1.0  # P-only servos sag short under gravity: steady error < ~22 deg is fine
LIFT_JOINTS = (LIFT, ELBOW, WFLEX)
REST_TOLERANCE = 60
TAU = 2 * math.pi


def S(t, period, phase=0.0):
    return math.sin(TAU * t / period + phase)


def peck(t, period):
    return max(0.0, S(t, period)) ** 2  # quick dip, then a pause up top


# Each section: (seconds, base pose offsets in degrees from centre, layered motion f(t) -> degrees)
# Gripper is in absolute counts (closed ~1481, open ~3031) instead of degrees.
SECTIONS = [
    # Rise with the elbow + wrist still folded (compact arm = small load on the shoulder), then unfold.
    ("Rising up, still curled", 6.0, {ELBOW: "rest", WFLEX: "curl"}, lambda t: {PAN: 12 * S(t, 6.0)}),
    ("Unfolding into the L pose", 3.5, {}, None),
    ("Cobra sway", 9.0, {ELBOW: -30}, lambda t: {
        PAN: 25 * S(t, 4.5), WROLL: -20 * S(t, 4.5), LIFT: 5 * S(t, 4.5),
        WFLEX: 15 * S(t, 2.25), ELBOW: 8 * S(t, 2.25, math.pi / 2)}),
    ("Giraffe rises and looks around", 7.0, {ELBOW: -80}, lambda t: {
        PAN: 35 * S(t, 7.0), WFLEX: 25 * S(t, 3.5), WROLL: 15 * S(t, 3.5, math.pi / 2)}),
    ("Figure eight", 8.0, {ELBOW: -45}, lambda t: {
        PAN: 25 * S(t, 4.0), WFLEX: 20 * S(t, 2.0), ELBOW: 10 * S(t, 2.0), WROLL: 10 * S(t, 4.0)}),
    ("Bird pecking", 7.0, {LIFT: 5, ELBOW: -20, WFLEX: 10}, lambda t: {
        LIFT: 10 * peck(t, 1.75), ELBOW: 15 * peck(t, 1.75), WFLEX: 25 * peck(t, 1.75),
        PAN: 20 * S(t, 7.0), GRIP: 450 - 1000 * peck(t, 1.75)}),
    ("Waving hello", 7.5, {PAN: 25, ELBOW: -70}, lambda t: {
        WROLL: 30 * S(t, 2.5), WFLEX: 20 * S(t, 2.5, -math.pi / 2), ELBOW: 8 * S(t, 2.5)}),
    ("Taking a bow", 4.0, {LIFT: 15, ELBOW: 10, WFLEX: 40}, None),
    ("Standing back up", 3.0, {}, None),
    ("Curling up", 3.5, {ELBOW: "rest", WFLEX: "curl"}, None),
    ("Going back to sleep", 5.0, "REST", None),
]
FADE_S = 1.5


def min_jerk(x):
    x = min(max(x, 0.0), 1.0)
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def envelope(t, dur):
    return min_jerk(t / FADE_S) * min_jerk((dur - t) / FADE_S)


def pose_counts(offsets_deg, rest):
    if offsets_deg == "REST":
        return dict(rest)
    named = {"rest": rest, "curl": {WFLEX: SAFE_RANGE[WFLEX][0] + 60}}  # curl = wrist fully folded
    p = {m: named[offsets_deg[m]][m] if isinstance(offsets_deg.get(m), str) else C[m] + offsets_deg.get(m, 0) * DEG
         for m in MOTORS if m != GRIP}
    p[GRIP] = C[GRIP]
    return p


def build(rest):
    """Return a list of (label, [goal dict per 1/HZ tick]) with ranges clamped and speeds limited."""
    out, prev, last = [], dict(rest), dict(rest)
    for label, dur, offsets, motion in SECTIONS:
        base = pose_counts(offsets, rest)
        # Big shoulder moves fight gravity: spread them over the whole section.
        ease = dur if motion is None or abs(base[LIFT] - prev[LIFT]) > 300 else min(dur, 3.0)
        ticks = []
        for i in range(int(dur * HZ)):
            t = i / HZ
            a = min_jerk(t / ease)  # ease into this section's base pose
            goal = {m: prev[m] + (base[m] - prev[m]) * a for m in MOTORS}
            if motion:
                env = envelope(t, dur)
                for m, v in motion(t).items():
                    goal[m] += env * (v if m == GRIP else v * DEG)
            for m in MOTORS:
                lo, hi = SAFE_RANGE[m]
                g = min(max(goal[m], lo), hi)
                g = min(max(g, last[m] - MAX_STEP), last[m] + MAX_STEP)
                last[m] = g
            ticks.append({m: int(round(last[m])) for m in MOTORS})
        out.append((label, ticks))
        prev = base
    return out


def report(plan):
    peak = {m: 0.0 for m in MOTORS}
    lo_seen = {m: 99999 for m in MOTORS}
    hi_seen = {m: -1 for m in MOTORS}
    prev = None
    for _, ticks in plan:
        for g in ticks:
            for m in MOTORS:
                lo_seen[m], hi_seen[m] = min(lo_seen[m], g[m]), max(hi_seen[m], g[m])
                if prev:
                    peak[m] = max(peak[m], abs(g[m] - prev[m]) * HZ / DEG)
            prev = g
    names = {PAN: "pan", LIFT: "lift", ELBOW: "elbow", WFLEX: "wrist_flex", WROLL: "wrist_roll", GRIP: "gripper"}
    for m in MOTORS:
        print(f"  {names[m]:<10} goals {lo_seen[m]}..{hi_seen[m]} (range {SAFE_RANGE[m][0]}..{SAFE_RANGE[m][1]})"
              f"  peak {peak[m]:.0f} deg/s", flush=True)
    print(f"  total {sum(len(t) for _, t in plan) / HZ:.0f} s", flush=True)


def run():
    from scservo_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler

    P_GAIN, ACC, GOAL_POS, GOAL_SPEED, TORQUE_LIM, TORQUE_EN, PRESENT_POS = 21, 41, 42, 46, 48, 40, 56
    port, ph = PortHandler("/dev/cu.usbmodem5AE60826921"), PacketHandler(0)
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

    writer = GroupSyncWrite(port, ph, GOAL_POS, 2)
    reader = GroupSyncRead(port, ph, PRESENT_POS, 2)
    temp_reader = GroupSyncRead(port, ph, 63, 1)  # Present_Temperature
    for m in MOTORS:
        reader.addParam(m)
        temp_reader.addParam(m)

    def send(goal):
        writer.clearParam()
        for m, g in goal.items():
            writer.addParam(m, [g & 0xFF, (g >> 8) & 0xFF])
        writer.txPacket()

    def read_all():
        if reader.txRxPacket() != COMM_SUCCESS:
            return None
        if not all(reader.isAvailable(m, PRESENT_POS, 2) for m in MOTORS):
            return None
        return {m: reader.getData(m, PRESENT_POS, 2) for m in MOTORS}

    def at_rest():
        now = {m: pos(m) for m in LIFT_JOINTS}
        ok = all(abs(now[m] - min(max(rest[m], SAFE_RANGE[m][0]), SAFE_RANGE[m][1])) <= REST_TOLERANCE
                 for m in LIFT_JOINTS)
        print(f"Rest check: now {now}, start {[rest[m] for m in LIFT_JOINTS]} -> "
              f"{'at rest' if ok else 'NOT at rest'}", flush=True)
        return ok

    def lower_patiently():
        # No abort here: aborting again would leave the arm up. Push toward rest for up to 30 s.
        print("Lowering back to rest after abort...", flush=True)
        for m in MOTORS:
            w(m, GOAL_SPEED, 150)
        for m in (WFLEX, ELBOW, LIFT, PAN, WROLL, GRIP):
            w(m, GOAL_POS, min(max(rest[m], SAFE_RANGE[m][0]), SAFE_RANGE[m][1]))
        t0 = time.time()
        while time.time() - t0 < 30 and not all(
                abs(pos(m) - min(max(rest[m], SAFE_RANGE[m][0]), SAFE_RANGE[m][1])) <= REST_TOLERANCE
                for m in LIFT_JOINTS):
            time.sleep(0.1)

    rest = {m: pos(m) for m in MOTORS}
    p_orig = {m: ph.read1ByteTxRx(port, m, P_GAIN)[0] for m in MOTORS}
    if not (rest[LIFT] < 1150 and rest[ELBOW] > 2900):
        port.closePort()
        raise SystemExit(f"Not starting: arm isn't resting folded (shoulder {rest[LIFT]}, elbow {rest[ELBOW]}). "
                         "Put it on its rest first. Torque untouched.")
    plan = build(rest)

    torque_on = False
    try:
        for s in range(10, 0, -1):
            print(f"Starting in {s}s... (emergency stop = unplug 12 V power)", flush=True)
            time.sleep(1)
        for m in MOTORS:
            w(m, P_GAIN, DANCE_P, size=1)  # stiffer than LeRobot's 16 so it follows the curve; restored after
            w(m, ACC, 254, size=1)       # like LeRobot teleop: the streamed curve itself is smooth
            w(m, GOAL_SPEED, 0)          # no cap; the curve itself limits speed (<= 90 deg/s)
            w(m, TORQUE_LIM, TORQUE_LIMIT[m])
            w(m, GOAL_POS, rest[m])      # writing a goal turns torque ON, so limits go first
        for m in MOTORS:
            w(m, TORQUE_EN, 1, size=1)
        torque_on = True

        # Time-scaling: if a joint falls behind, the choreography clock pauses until it catches up,
        # so the arm is never asked to go faster than it can at these torque limits.
        flat = [(label, g) for label, ticks in plan for g in ticks]
        worst = {m: 0 for m in MOTORS}
        i, label_shown, waiting, pauses, peak_temp = 0, None, False, 0, 0
        best_err = last_progress = None
        next_t = time.perf_counter()
        while i < len(flat):
            label, goal = flat[i]
            if label != label_shown:
                print(label, flush=True)
                label_shown = label
            send(goal)
            if waiting or i % 5 == 0:  # check tracking 10x per second, every tick while paused
                now = read_all()
                if now:
                    errs = {m: abs(now[m] - goal[m]) for m in MOTORS}
                    for m in MOTORS:
                        worst[m] = max(worst[m], errs[m])
                    err = max(errs.values())
                    if not waiting and err > PAUSE_ERR:
                        waiting, pauses = True, pauses + 1
                        best_err, last_progress = err, time.time()
                    elif waiting and err < RESUME_ERR:
                        waiting = False
                    elif waiting:
                        if err < best_err - 10:
                            best_err, last_progress = err, time.time()
                        elif err < DROOP_OK and time.time() - last_progress > SETTLE_S:
                            waiting = False  # steady gravity sag, not blocked: carry on
                        elif time.time() - last_progress > STUCK_S:
                            lag = max(errs, key=errs.get)
                            raise RuntimeError(f"motor {lag} stuck: at {now[lag]}, goal {goal[lag]}")
            if i % HZ == 0 and temp_reader.txRxPacket() == COMM_SUCCESS:  # once per second
                temps = {m: temp_reader.getData(m, 63, 1) for m in MOTORS if temp_reader.isAvailable(m, 63, 1)}
                # Sync reads occasionally return garbage (e.g. 150 C): ignore implausible values and
                # confirm any hot reading with a direct read before stopping.
                hot = {m: t for m, t in temps.items() if MAX_TEMP_C <= t < 100}
                if hot:
                    port.clearPort()
                    hot = {m: t for m in hot if (t := ph.read1ByteTxRx(port, m, 63)[0]) >= MAX_TEMP_C}
                temps = {m: t for m, t in temps.items() if t < MAX_TEMP_C}  # unconfirmed hot readings were garbage
                if hot:
                    raise RuntimeError(f"servo too hot (deg C): {hot}")
                peak_temp = max(peak_temp, max(temps.values(), default=0))
            if not waiting:
                i += 1
            next_t += 1 / HZ
            time.sleep(max(0.0, next_t - time.perf_counter()))
        time.sleep(1.0)  # let the last goal settle
        print(f"Dance complete. Paused {pauses}x to let the arm catch up. Worst lag (counts): {worst}. "
              f"Hottest servo: {peak_temp} C", flush=True)
    except Exception as e:
        print(f"ABORT: {e}", flush=True)
        if torque_on:
            try:
                lower_patiently()
            except Exception as e2:
                print(f"Could not lower: {e2}", flush=True)
    finally:
        # NEVER cut torque with the arm raised: it drops. Release only once it is back at rest.
        for m in MOTORS:  # a gain write never turns torque on, so restoring first is safe
            ph.write1ByteTxRx(port, m, P_GAIN, p_orig[m])
        try:
            resting = not torque_on or at_rest()
        except Exception as e:
            print(f"Could not check rest pose: {e}", flush=True)
            resting = False
        if resting:
            off = {}
            for m in MOTORS:  # retry each joint until a clean readback says off
                for _ in range(5):
                    ph.write1ByteTxRx(port, m, TORQUE_EN, 0)
                    time.sleep(0.02)
                    port.clearPort()  # stale replies in the buffer garble readbacks
                    v, r, _ = ph.read1ByteTxRx(port, m, TORQUE_EN)
                    if r == COMM_SUCCESS and v == 0:
                        break
                off[m] = v if r == COMM_SUCCESS else "ERR"
            print("Arm at rest. Torque readback (0 = off):", off, flush=True)
            if any(v != 0 for v in off.values()):
                print("WARNING: some joint did not confirm torque OFF.", flush=True)
        else:
            for m in MOTORS:
                v, r, _ = ph.read2ByteTxRx(port, m, PRESENT_POS)
                if r == COMM_SUCCESS:
                    ph.write2ByteTxRx(port, m, GOAL_POS, v)  # hold right here (keeps torque on)
            print("HOLDING with torque ON: arm is not back at rest. Support the arm by hand before releasing it.",
                  flush=True)
        port.clearPort()
        print("P gains restored:", {m: ph.read1ByteTxRx(port, m, P_GAIN)[0] for m in MOTORS}, flush=True)
        port.closePort()


if __name__ == "__main__":
    if "--dry-run" in sys.argv:
        fake_rest = {PAN: 1992, LIFT: 873, ELBOW: 3076, WFLEX: 1279, WROLL: 2044, GRIP: 2257}
        report(build(fake_rest))
    else:
        run()

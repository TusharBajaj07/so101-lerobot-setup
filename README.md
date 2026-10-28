# SO-101 arm setup with LeRobot

Scripts and notes from setting up a pair of SO-101 arms (leader + follower) with
[LeRobot](https://github.com/huggingface/lerobot) 0.6.1 on macOS, working toward imitation learning.

## Hardware

- Two SO-101 arms, each on a Waveshare Bus Servo Adapter (A), jumpers on B (USB-SERVO), 12 V supply.
- **Both arms use 12 V STS3215 servos** (Max_Voltage_Limit 14.0 V, firmware 3.9), including the leader.
- Emergency stop is the 12 V barrel jack. Unplugging USB does **not** make the servos go limp.
  Pulling 12 V drops a raised arm, so support it first if you can.

## Scripts

| Script | What it does | Moves the arm? |
|---|---|---|
| `scan.py` | Read-only scan of servos on a port (ID, voltage, temperature, torque state) | No |
| `chat_calibrate.py` | Two-step calibration (`middle`, then `sweep` ended by a stop file) that can be driven without a keyboard. Torque stays off | No (by hand) |
| `safe_calibrate.py` | LeRobot `calibrate()` without the torque-enable that `connect()` does | No (by hand) |
| `leader_base_test.py`, `leader_lift_test.py` | First powered tests on the leader | Yes |
| `follower_center_test.py` | Moves the follower to the centre of every calibrated range, holds, returns | Yes |
| `follower_dance.py` | Step-by-step routine (one move at a time) | Yes |
| `follower_flow_dance.py` | Flowing multi-joint dance streamed at 50 Hz. `--dry-run` checks ranges and speeds without moving anything | Yes |
| `match_arms.py` | Slowly moves single joints so leader and follower poses match before teleop | Yes |
| `release_follower.py` | Turns follower torque off **only** if the arm is resting folded | No |

`calibration/` holds this pair's calibration files (`my_leader.json`, `my_follower.json`). LeRobot reads them from
`~/.cache/huggingface/lerobot/calibration/`.

## Teleoperation

```bash
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=<follower port> --robot.id=my_follower \
  --robot.disable_torque_on_disconnect=false --robot.max_relative_target=20 \
  --teleop.type=so101_leader --teleop.port=<leader port> --teleop.id=my_leader
```

- `disable_torque_on_disconnect=false`: LeRobot normally cuts torque on exit wherever the arm is, which drops a
  raised arm. With this the follower holds instead. Fold the leader to rest, stop teleop, then run `release_follower.py`.
- `max_relative_target`: 1.5 was far too tight. It is measured from the *current* position, and gravity droop is
  larger than that, so the follower trailed the leader by up to 60-130 deg. 20 works.
- Start with both arms in matching poses (compare them first), otherwise the follower jumps to the leader's pose.

## Things learned (STS3215 / SO-101)

- **Never cut torque with the arm raised.** Lower it under power to rest, check it's resting, then release.
  On any error, hold position instead of releasing.
- **Writing `Goal_Position` turns torque ON**, even right after `Torque_Enable = 0`. Set speed, acceleration and
  torque limit before writing a goal, make torque-off the last write, and read it back to confirm.
- **A servo can ignore position goals** (torque on, goal far away, load 0, no motion), seen after power-on and when
  the goal was written while torque was off. What worked reliably: enable torque holding the current position
  first, then send the new goal with torque already on. LeRobot rewrites `Operating_Mode` on connect.
- **The PID runs inside each servo.** P / I / D live at registers 21 / 23 / 22. LeRobot sets P=16, I=0, D=32 on
  connect (Feetech default P=32) to reduce shaking. With I=0, loaded joints settle a few degrees short under gravity.
  Streaming lag on an unloaded base: 7.0 deg at P=8, 4.4 at P=16, 2.7 at P=32.
- **`Torque_Limit` acts like a PWM cap**, so under gravity it also caps speed. At 60 % the shoulder could not stream
  past mid-lift at about 30 deg/s, while slow moves (about 13 deg/s) passed.
- Group sync reads occasionally return garbage (for example a 150 C temperature). Sanity-check values and confirm
  with a direct read before acting on them.
- Resting joints can sit a few counts past their swept range. Clamp targets instead of rejecting them.
- In PWM mode (`Operating_Mode` 2), `Present_Position` is raw (no homing offset applied).

## Known issues

- Teleop has twice stopped after a few minutes when the **leader** stopped answering ("There is no status packet").
  A 45 s read-only stress test showed no dropouts, so it may be bus load at 60 Hz or a cable that moves during use.
  Next test: `--fps=30`.
- `leader_lift_test.py` still turns torque off after a failed lowering. Don't reuse it without applying the
  rest-check pattern from `follower_flow_dance.py`.

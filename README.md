# dobot_magician_arm

A single-file, pure-Python Linux driver for the **Dobot Magician** 4-DOF robotic
arm, plus an **agent skill** that teaches an AI coding agent (e.g. Claude Code)
to drive the arm safely over its USB serial link.

No vendor DLL, no ROS, no daemon — `driver/dobot.py` speaks the Dobot binary
protocol (V1.1.5) directly over `pyserial`. Every command ID in the driver was
validated against real hardware, including several fixes for errors you will
hit if you follow the official documentation naively (see
[Hardware-validated gotchas](#hardware-validated-gotchas)).

```
driver/dobot.py                  the driver (one file, pyserial only)
skills/dobot-magician/SKILL.md   agent skill: how an AI drives the arm safely
.claude-plugin/plugin.json       installable as a Claude Code plugin
```

## What the driver does

- **Reads:** pose (cartesian + all four joints), alarm bitmap with a full
  decoded name table (`decode_alarms()` → `["plan_inv_limit"]`, not hex soup)
- **Motion:** homing, PTP moves (cartesian `move_to` and joint `move_joint`),
  **JUMP mode** (`jump_to` — auto lift → travel → descend, for pick and place),
  and `run_sequence()` — queue a whole path so the firmware flows through it
  continuously instead of stop-and-go
- **End effectors:** pneumatic gripper (`grip`/`release`/`pump_off`), suction
  cup (`suction`), servo PWM on GP3
- **Accessories:** color sensor (GP2), IR photoelectric sensor (GP4),
  conveyor/stepper (EMotor)
- **Teach & playback:** hand-hold teaching via the forearm unlock button
  (`teach()` captures a pose on each button release), saved as JSON routines,
  replayed with `playback()`
- **Guard rails:** planar-reach and joint soft-limit checks before sending,
  alarm decode + auto-clear after every motion

## Quick start

```bash
pip install pyserial
python3 driver/dobot.py        # read-only: prints port, pose, alarms
```

```python
from dobot import Dobot

d = Dobot()                    # /dev/dobot, falls back to /dev/ttyUSB0
print(d.get_pose())
d.home()                       # REQUIRED once after every power-up
d.move_to(200, 0, 50)          # x, y, z in mm (r = wrist deg, default 0)
d.jump_to(250, 80, -20)        # lift, travel over, descend
d.grip(); d.release()
d.run_sequence([               # queued: flows continuously
    ("jump", 250, 80, -20, 0),
    ("grip",),
    ("jump", 150, -80, -20, 0),
    ("release",),
])
d.close()
```

### Serial port setup (Linux)

The arm's onboard CP2102N shows up as `/dev/ttyUSB0` (vid:pid `10c4:ea60`).
For a stable name and non-root access:

```bash
# /etc/udev/rules.d/99-dobot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="dobot", GROUP="plugdev", MODE="0660"
```

then `sudo udevadm control --reload && sudo udevadm trigger`, and make sure
your user is in `plugdev` (or `dialout` if you skip the rule). Opening the
port does **not** reset the arm — reads are always safe.

## ⚠️ Safety

Reads (pose, alarms) are harmless. **Homing and PTP moves swing the arm
through its full 320 mm radius.** Clear the workspace and check the attached
end effector before any motion, and home once after every power-up. The driver
enforces the *real* firmware soft limits (tighter than the marketing specs):

| Joint | Limit |
|-------|-------|
| J1 (base) | ±90° |
| J2 (rear arm) | 0° … 85° |
| J3 (forearm) | −10° … 90° |
| J4 (wrist) | ±90° |

Planar reach is guarded to 50–320 mm. The z envelope is pose-dependent: high z
is only reachable at short planar reach (z ≈ 90 mm tops out near x ≈ 200), and
the arm reaches *below* z = 0 at short reach (3D-printer-bed depth).

## Hardware-validated gotchas

These cost real debugging time on real hardware; the driver already handles
them, but they matter if you write your own:

- **`ClearAllAlarms` is protocol ID 20 with the write bit set** — the same ID
  as `GetAlarmsState` (read). It is *not* command 21. If you send 21, every
  alarm clear is a silent no-op.
- **A `plan_inv_limit` alarm latches**: after one unreachable target, the
  firmware refuses *all* further PTP moves — even reachable ones — until the
  alarm is cleared. An out-of-envelope move can therefore "fail" silently with
  the arm just sitting still.
- **The gripper is pneumatic, not a servo.** Gripper command 63:
  `[1,1]` = close, `[1,0]` = open, `[0,0]` = pump off. The suction-cup command
  (62) will close the claw but can never open it.
- **Suction seals on flat, rigid surfaces only** — thin crinkly foil will not
  hold vacuum. Press to contact, suction on, dwell ~3 s to seal, then lift to
  a *reachable* z.
- **A pose read immediately after a move can return the pre-settle value** —
  trust a settled re-read (or a camera), not the first poll.
- Queued IO commands (ctrl `0x03`) never execute unless the queue is started —
  immediate (`0x01`) is correct for one-off IO.

## The agent skill

[`skills/dobot-magician/SKILL.md`](skills/dobot-magician/SKILL.md) packages the
operating knowledge — connection, safety rules, motion primitives, pick-and-
place recipes, alarm recovery — so an AI agent can drive the arm through this
driver. Install it standalone by copying the folder into `.claude/skills/`, or
as a plugin via the [ai-toolbox](https://github.com/magicsmokepress/ai-toolbox)
marketplace.

## Prior art

Existing Dobot Magician software, and how this repo differs:

- [Dobot official repositories](https://github.com/dobot-arm) and the
  [Magician product page](https://www.dobot-robots.com/products/education/magician.html) —
  the vendor SDK is built around a compiled DLL and Windows-centric demo apps.
  This repo is pure Python over the documented
  [Communication Protocol V1.1.5](https://download.dobot.cc/product-manual/dobot-magician/pdf/en/Dobot-Communication-Protocol-V1.1.5.pdf).
- [luismesas/pydobot](https://github.com/luismesas/pydobot)
  ([PyPI](https://pypi.org/project/pydobot/)) — the classic pip-installable
  driver: pose, `move_to`, suction, grip. No JUMP mode, no queued sequences,
  no alarm decode, no teach/playback.
- [AlexGustafsson/dobot-python](https://github.com/AlexGustafsson/dobot-python) —
  a broad low-level command wrapper. This repo instead curates the
  hardware-validated subset and adds envelope guards and alarm recovery.
- [sammydick22/pydobotplus](https://github.com/sammydick22/pydobotplus) —
  extends pydobot with the color/IR sensors and conveyor. Those accessory
  command IDs are folded in here (validated on hardware), on top of the
  queue/JUMP/teach features pydobotplus lacks.
- [jkaniuka/magician_ros2](https://github.com/jkaniuka/magician_ros2) — a full
  ROS 2 control stack. Right choice if you're in the ROS ecosystem; this repo
  is for when you want one file and a serial port.

What none of them provide is the **agent skill**: a documented, safety-first
operating procedure for an *AI agent* (rather than a human programmer) to run
the arm.

## License

[MIT](LICENSE)

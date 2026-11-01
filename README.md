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

## Getting started (step by step)

You need: a Dobot Magician, its USB cable and 12 V power supply, and a Linux
computer with Python 3.8+. No prior robotics or serial-port experience
assumed.

**1. Get the code and the one dependency**

```bash
git clone https://github.com/magicsmokepress/dobot_magician_arm.git
cd dobot_magician_arm
pip install pyserial
```

(If `pip` isn't found, try `pip3` or `sudo apt install python3-pip` first.)

**2. Plug in the arm**

Connect the power supply, flip the power switch on the arm's base (you'll
hear a beep), then connect the USB cable. Check Linux saw it:

```bash
ls /dev/ttyUSB*
```

You should see `/dev/ttyUSB0` (the arm's built-in USB-serial chip). If
nothing appears, try another cable/port — some USB cables are charge-only.

**3. Give yourself permission to use the port**

Out of the box the port is root-only. Easiest fix:

```bash
sudo usermod -aG dialout $USER
```

then **log out and back in** (required — group changes don't apply to open
sessions). Optional but nicer: install the udev rule below to get a stable
`/dev/dobot` name that survives replugging:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="dobot", GROUP="plugdev", MODE="0660"' | sudo tee /etc/udev/rules.d/99-dobot.rules
sudo udevadm control --reload && sudo udevadm trigger
```

**4. First contact (safe — nothing moves)**

```bash
python3 driver/dobot.py
```

Expected output, something like:

```
port: /dev/ttyUSB0 pose: {'x': 259.1, 'y': 0.0, 'z': -8.5, ...} alarms: none
```

This only *reads* from the arm. Opening the port never resets or moves it.
If you got a pose, everything works — the rest is just Python.

**5. First motion**

⚠️ Clear everything within 35 cm of the base first — homing swings the arm
through its full range. Then:

```python
import sys; sys.path.insert(0, "driver")
from dobot import Dobot

d = Dobot()
d.home()                 # REQUIRED once after every power-up (~20 s sweep)
d.move_to(200, 0, 50)    # x=200mm forward, y=0 centered, z=50mm up
d.move_to(200, 100, 50)  # slide left
d.grip(); d.release()    # if the pneumatic gripper is attached
d.close()
```

Every motion call returns the resulting pose and any alarms — an empty
alarm list means the move really happened.

**6. Going further**

```python
d.jump_to(250, 80, -20)        # pick-and-place arc: lift, travel over, descend
d.run_sequence([               # queue a whole path; the arm flows through it
    ("jump", 250, 80, -20, 0),
    ("grip",),
    ("jump", 150, -80, -20, 0),
    ("release",),
])
```

See the [agent skill](skills/dobot-magician/SKILL.md) for the full operating
guide (coordinate frame, reachable envelope, pick/place recipes, alarm
recovery) — it's written for AI agents but works just as well for humans.

### Troubleshooting

| Symptom | Fix |
|---------|-----|
| `No module named serial` | `pip install pyserial` (note: the package is **pyserial**, not "serial") |
| `cannot open dobot port` / permission denied | Step 3 not done, or you didn't log out/in after it |
| `/dev/ttyUSB0` missing | Charge-only USB cable, arm not powered on, or another program grabbed the port |
| Arm connects but won't move | Check `d.decode_alarms()` — a latched alarm blocks all motion; `d.clear_alarms()` then retry. Also: did you `home()` after power-up? |
| Move "succeeds" but arm doesn't go there | Target out of reach — see the envelope notes below; the returned `alarms` list will say `plan_inv_limit` |

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
driver. To install it in Claude Code, run these two commands inside Claude
Code itself:

```
/plugin marketplace add magicsmokepress/ai-toolbox
/plugin install dobot-magician@ai-toolbox
```

Then just ask the agent to move the arm. (Alternative without the
marketplace: copy the `skills/dobot-magician/` folder into your project's
`.claude/skills/` directory.)

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

[MIT](LICENSE). This is AI-assisted work.

Third-party work this repo depends on or references. Nothing from any of it
is vendored here.

| What | Role here | Licence |
|---|---|---|
| [pyserial](https://github.com/pyserial/pyserial) | the only runtime dependency | BSD-3-Clause |
| [Dobot Magician Communication Protocol V1.1.5](https://download.dobot.cc/product-manual/dobot-magician/pdf/en/Dobot-Communication-Protocol-V1.1.5.pdf) | the command IDs, frame format, checksum and alarm codes the driver implements | Dobot's documentation; no text reproduced |
| [luismesas/pydobot](https://github.com/luismesas/pydobot), [sammydick22/pydobotplus](https://github.com/sammydick22/pydobotplus), [AlexGustafsson/dobot-python](https://github.com/AlexGustafsson/dobot-python), [jkaniuka/magician_ros2](https://github.com/jkaniuka/magician_ros2) | prior art, compared above; no code taken from any of them | MIT, MIT, MIT, MIT |

Dobot and Dobot Magician are trademarks of Shenzhen Yuejiang Technology Co.,
Ltd. This project is not affiliated with or endorsed by Dobot.

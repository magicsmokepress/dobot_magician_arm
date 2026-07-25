---
name: dobot-magician
description: Drive a Dobot Magician 4-DOF robotic arm from an AI agent over USB serial using this repo's pure-Python driver. Use when the agent needs to read the arm's pose, home it, move it, pick and place with the gripper or suction cup, run queued motion sequences, or recover from arm alarms. Triggers on "move the arm", "home the arm", "pick up", "grab", "suction", "dobot", "robot arm".
---

# Driving the Dobot Magician arm

You (the agent) control a real robotic arm through `driver/dobot.py` — a
pure-Python serial driver, no vendor SDK. The arm is physical: it can knock
things over, crush fingers, and damage itself. Follow the safety rules; they
are not optional.

## Safety rules (before anything else)

1. **Reads are always safe** — `get_pose()`, `decode_alarms()` never move
   the arm. Start every session with them.
2. **Never send the first motion command until a human confirms the workspace
   is clear** and tells you which end effector is attached. Homing and moves
   swing the arm through its full 320 mm radius.
3. **Home once after every power-up** (`d.home()`) — pose readings are
   meaningless before homing.
4. **Use conservative speed** (the driver defaults, `vel=30, acc=30`) until
   the human has watched a motion and okayed faster. Low `acc` (≈8) relative
   to `vel` gives smooth, natural motion.
5. **After every motion, check the returned `alarms` list.** An empty list
   means the move really happened; a `plan_inv_limit` means your target was
   unreachable and the arm did NOT go there.

## Setup (once per session)

```python
import sys; sys.path.insert(0, "driver")   # from the repo root
from dobot import Dobot

d = Dobot()          # /dev/dobot, falls back to /dev/ttyUSB0
print(d.get_pose())  # {'x':…, 'y':…, 'z':…, 'r':…, 'j1':…, …}
print(d.decode_alarms() or "no alarms")
```

If the port won't open: another process holds it, or you lack permission
(user must be in `plugdev`/`dialout` — see repo README for the udev rule).
Opening the port does not reset the arm.

## Coordinate frame and envelope

- `x` forward from the base, `y` left(+)/right(−), `z` up, all mm;
  `r` = wrist rotation (J4) in degrees. Home is about (259, 0, −8.5).
- Planar reach 50–320 mm. **High z only works at short reach** (z ≈ 90 mm max
  near x ≈ 200); the arm reaches below z = 0 at short reach — work surfaces
  can legitimately be at negative z. Don't guess floor height: hover above
  the target and ask the human to measure the gap, then
  `target_z = hover_z − measured_gap_mm`.
- Soft limits (enforced by driver AND firmware): J1 ±90°, J2 0–85°,
  J3 −10–90°, J4 ±90°.

## Motion primitives

```python
d.home()                            # full homing sweep, clears alarms after
d.move_to(x, y, z, r=0)             # PTP cartesian; returns {"pose":…, "alarms":[…]}
d.move_joint(j1, j2, j3, j4)        # PTP joint-space
d.jump_to(x, y, z, r=0, height=40)  # lift → travel over → descend (pick/place)
d.grip(); d.release(); d.pump_off() # pneumatic claw (63): close / open / pump off
d.suction(True); d.suction(False)   # suction cup (62)
```

For multi-step paths use one queued sequence — the firmware flows through it
continuously instead of stop-and-go between separate calls:

```python
d.run_sequence([
    ("jump", 250,  80, -20, 0),   # over the object, descend
    ("grip",),
    ("wait", 500),                # ms
    ("jump", 150, -80, -20, 0),   # carry to the drop zone
    ("release",),
], vel=60, acc=20, jump_height=40)
```

## Pick and place recipes

**Gripper (pneumatic 2-finger claw):** works on objects with graspable bodies.
It cannot pick up flat items lying on a surface — prop them up or use suction.

**Suction cup:** flat rigid surfaces only (foil/crinkly films won't seal).
Recipe: descend to firm contact → `suction(True)` → dwell ~3 s to seal →
lift to a *reachable* z. Verify the object actually rose (camera or human)
before reporting success — the pose read alone proves the arm moved, not
that the object came with it.

## Alarm recovery

```python
d.decode_alarms()     # e.g. ["plan_inv_limit"]
d.clear_alarms()      # actually works (protocol ID 20 + write bit)
```

A `plan_inv_limit` (unreachable target) **latches**: until cleared, the
firmware silently refuses every further move. `move_to`/`jump_to` auto-clear
after reporting, but if the arm "won't move," decode alarms first — don't
retry blindly. `limit_j1_neg` right after homing is benign (the homing sweep
touches the limit); it clears normally.

## Teach and playback

With a human at the arm: `d.teach("routine_name")` — they hold the forearm
unlock button, position the arm by hand, release to capture each waypoint.
`d.playback("routine_name")` replays it as a queued sequence.
`d.list_routines()` shows what's saved.

## Gotchas that will bite you

- A pose read immediately after a move can return the pre-settle value —
  re-read after the arm is still, or trust the returned `pose` (taken after
  the stability wait).
- A "lift" must target a reachable pose: raising z at short planar reach hits
  `plan_inv_limit` and silently does nothing. Lift at larger x, or use
  `jump_to`.
- If a camera watches the workspace, remember a single oblique view has no
  depth — 2D overlap of tool and target does not mean they're aligned. Use a
  top-down view, two views, or a human measurement.
- Always `d.close()` when done; leave the arm parked away from people with
  pump off.

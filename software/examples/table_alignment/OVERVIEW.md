# Table alignment on XLeRobot: how it works

This explains what the table alignment does, how the parts fit together, and what is and isn't verified yet. For the
bring-up steps and commands, see [README.md](README.md).

## What it does

It drives the 3-omniwheel base to a fixed, taught pose in front of a table: square to the edge, at a set distance,
with the work region centred. The input is one AprilTag on the table, seen by the head camera. A manipulation
policy can then start from the same pose every time.

It does not move the arms, the lift or the head. The head is held at a saved pose throughout.

## How it works

```
head camera ──► AprilTag detect + PnP ──► camera→base (extrinsics) ──► table pose in the base frame
                                                                              │
                         taught target ◄── teach (once) ◄─────────────────────┤
                                                                              ▼
                                               errors: distance, lateral, heading
                                                                              │
                                                                              ▼
           AlignToTable controller ──► short velocity pulse ──► verified stop ──► settle ──► measure again
```

- **Perception** (`perception/`)
  - Finds a 36h11 AprilTag and solves its pose with OpenCV (IPPE square PnP).
  - Using the camera's extrinsics and where the tag sits on the table (`TagPlacement`), it turns that into the
    table's pose in the base frame.
  - A frame is rejected if:
    - it is older than 0.3 s;
    - the tag's median reprojection is over 2 px;
    - none, or more than one, matching tag is seen.
- **Target**
  - `teach` records the measured distance, lateral offset and heading with the robot parked by hand.
  - Alignment drives back to those same readings. Constant errors in the extrinsics or the tag placement cancel
    out, because the same ones were in the readings when you taught.
- **Controller** (`behaviors/align_to_table.py`)
  - Works in pulses: it never drives continuously.
  - Each pulse is about 0.3 s. After it, the wheels are set to zero and read back to confirm they stopped, the
    controller waits 0.4 s to settle, then measures again from several fresh frames.
- **Base** (`base/`)
  - Inverse kinematics for the three wheels, with the same signs as `XLerobot._body_to_wheel_raw`.
  - Raw `Goal_Velocity` writes to the wheel motors, which are sts3215 IDs 7/8/9 on the wheel bus.
  - `stop_base()` writes zero to each wheel separately, reads it back and retries. A failed stop is a fault.
- **Head** (`head.py`)
  - `head_pose.py capture` saves the head motors' positions (IDs 7/8 on the head bus) and holds them there with
    torque on.
  - During alignment, every pulse first checks the head is still within 6 ticks (about 0.5°) of the saved pose. If
    it isn't, the run faults, because the camera calibration only holds at that pose.

### Controller stages

```
[SEARCH] → [RADIAL / ORBIT] → ESTIMATE → ALIGN_HEADING → ALIGN_LATERAL → ALIGN_DISTANCE → VERIFY → BASE_READY
                                    any problem → FAULT (base stopped)
```

- **SEARCH:** if no tag is visible at the start, the base rotates in 15° steps, at most one full turn, until it
  sees the tag. `--search-direction` sets which way it turns first.
- **RADIAL / ORBIT:**
  - Used only when the robot is displaced around the table, more than 10° off the taught approach line, rather
    than just turned.
  - With `--table-radius` (a round table), the base first moves to a fixed circle around the table centre, then
    orbits it facing the table.
  - The keep-out radius is the table radius plus 0.40 m. If the arms reach further than about 0.30 m from the base
    centre, set `--keepout-radius`.
- **ALIGN_\*:**
  - Proportional correction, one axis at a time, each down to half its tolerance.
  - The tolerances are 2° for heading and 2 cm each for lateral and distance.
  - `--axes` limits a run to some of the axes, so you can bring them up one at a time.
- **VERIFY:** three readings in a row must be within tolerance. The run then ends in BASE_READY, and the head is
  checked one last time.
- **Keep the tag in view:** at 640 px wide, with the margins scaling with image width:
  - A pulse is planned so the tag's predicted corners stay at least 30 px inside the image.
  - The tag is tracked during the pulse, and the pulse ends early if the tag comes within 10 px of the edge.
  - If the tag is lost anyway, the last pulse is reversed.

### Safety limits

These don't depend on the gains:

| Limit | Value |
| --- | --- |
| Net travel in the ALIGN stages | 0.60 m and 60° |
| Overshoot past the taught distance | at most 3 cm |
| Steps per run | 50 |
| Run time | 120 s |

The run faults, and the base stops, when:
- the camera stops giving fresh frames;
- an error grows after a correction (a wrong wheel sign looks like this);
- the head moves;
- a stop can't be confirmed;
- Ctrl-C is pressed.

**This is not an emergency stop.** Keep a hand on the base power.

## Frames and signs

- **Base:** origin at the base centre on the floor, x forward, y left, z up. On the keyboard, `W` = +x,
  `A` = +y and `Q` = +yaw (counter-clockwise from above). Step 1 of the README checks these.
- **Table:** origin on the front edge, at the centre of the work region. x points into the table, y along the edge
  to the robot's left, z up.
- **Errors:** a positive error is corrected by a positive command on the same axis. The commands are forward for
  distance, left for lateral and counter-clockwise for heading.
- **Tag mount:**
  - `flat`: lying on the tabletop, default yaw −90°, with its printed top pointing away from the robot.
  - `vertical`: standing upright facing the robot, default yaw 0°, with its printed top up. `--tag-height` is
    the tag centre's height above the tabletop.

  Use the same tag options for `watch`, `teach` and `align_to_table.py`. If they differ, you get a warning and the
  errors are offset.

## Code map

| Path | What it is |
| --- | --- |
| `software/src/table_alignment/` | The library, importable as `xlerobot_table_alignment` (via the symlink in `software/plugins/xlerobot_table_alignment`) |
| `…/base/omni_kinematics.py`, `base_controller.py` | Wheel kinematics, pulses, verified stop, bus check by motor ID |
| `…/perception/` | AprilTag detector and estimator, table pose and errors, camera calibration, geometry |
| `…/behaviors/align_to_table.py` | The controller (`AlignToTable`, `AlignConfig`) |
| `…/head.py` | Head pose capture, hold and check; `HeadGuardedBase` |
| `software/examples/table_alignment/wheels.py` | Keyboard pulses and the pulse test (step 1) |
| `…/head_pose.py` | `capture` / `check` / `goto` |
| `…/table_perception.py` | `snapshot`, intrinsics (ChArUco, RealSense or Orbbec), `calibrate-extrinsics`, `watch`, `teach` |
| `…/align_to_table.py` | The alignment run; writes a JSON log per run |
| `…/align_report.py` | Table of the logged runs |
| `software/tests/table_alignment/` | Simulated tests, no hardware needed: `python -m pytest software/tests/table_alignment` |

Files written under `outputs/table_alignment/` in the folder you run from (`software/examples/table_alignment`):

| File | Written by |
| --- | --- |
| `head_pose.json` | `head_pose.py capture` |
| `forward_intrinsics.json` | `realsense-intrinsics`, `orbbec-intrinsics` or `calibrate-intrinsics` |
| `forward_extrinsics.json` | `calibrate-extrinsics` |
| `table_target.json` | `teach` |
| `logs/` | `align_to_table.py` |

## Hardware and ports on the host PC (DGX)

- **USB:** everything plugs into the DGX directly: the wheel bus, the head bus and the head camera. There is no
  separate robot computer.
- **Serial ports:**
  - The defaults are `/dev/ttyACM1` for the wheels and `/dev/ttyACM0` for the head. The numbers depend on plug
    order, so check them with `ls -l /dev/serial/by-id/`. Those paths don't change between boots, so they're
    better than `ttyACM*` for `--port` and `--head-port`.
  - `open_wheel_bus` and `open_head_bus` check the motor IDs and models before writing anything. If you give the
    wrong bus, they say so; you never move the head by mistake.
  - For "Permission denied", run `sudo usermod -aG dialout $USER`, then log in again.
- **Camera:**
  - The default is `--camera realsense` (pyrealsense2).
  - Any V4L2 camera also works: `--camera /dev/videoN --size WxH --fourcc MJPG`.
  - The intrinsics only hold at the size they were made at. Later steps take that size from the intrinsics file.
- **lerobot version:**
  - The DGX venv has a newer lerobot, which uses the `rustypot` bus backend.
  - `lerobot.motors.feetech.tables` no longer exists there; the code now reads `bus.model_number_table` instead.
  - The other calls used are `ping`, `read`/`write` with `normalize=False`, `enable_torque`/`disable_torque`, and
    `connect(handshake=False)`. None of them have been tried against that version yet.
  - If one fails, the fix belongs in `base_controller.py` or `head.py`. Nothing else talks to the motors.
- **Before running:** stop teleoperation and anything else holding the ports or the camera, and stow the arms.

## Status

- Ported from AlohaMini, where it ran on the robot. On XLeRobot only the simulated tests have run; nothing has run
  on the hardware yet.
- Not yet checked on XLeRobot:
  - the wheel signs (README step 1);
  - whether short pulses move the cart at all: raise `--min-linear-speed` and `--min-heading-speed` if they don't;
  - the head tolerance.
- Known open issue, from AlohaMini: one trial start ("spot 6") stalled in the RADIAL move. It hasn't been diagnosed.
- **Copies:** the code is copied from `lerobot_alohamini`, not shared with it. A fix made in one repo has to be
  made in the other by hand. Synced up to AlohaMini commit `3b738b77`.

## Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| `Could not connect on port …` / `No such file or directory` | Wrong `--port`; see `/dev/serial/by-id/` |
| `… is not the wheel bus … looks like the head bus` | The ports are swapped; use the other one |
| `No wheel answered` | Base motor power is off |
| `INVALID: stale frame` | Something else has the camera, or the frames are slow; check `snapshot` |
| `INVALID: … reprojection` | Wrong `--tag-size` or `--tag-mount`, or the tag isn't flat |
| Intermittent `saw []` | Tag too small or blurred, or no white border around it |
| `calibrate-extrinsics`: `Nothing saved: median reprojection …` | Wrong `--tag-mount` (an upright tag treated as flat, or the reverse) |
| `HeadMovedError` | The head was bumped or sagged. Run `head_pose.py goto`; if it was re-aimed, recapture and redo extrinsics and teach |
| Fault: error grows after a correction | A wheel sign is wrong (README step 1), or the wrong tag yaw or mount |
| Corrections do nothing, then a fault | Pulses too weak for the cart; raise `--min-linear-speed` / `--min-heading-speed` |
| Teach warns the tag is near the image edge | Re-aim the head (then recapture and redo the steps after it) or move the tag |

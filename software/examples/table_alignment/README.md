# Table alignment for the 3-omniwheel XLeRobot

Drives the base to a taught pose in front of a table using an AprilTag and the head RealSense, in short pulses that
each end in a verified stop. Ported from AlohaMini. The code is in `software/src/table_alignment`.

**Not yet run on an XLeRobot.** The steps below are also the bring-up: do them in order, and don't skip the sign check.

Software stopping is not an emergency stop. Keep a hand on the base power whenever it is on.

## Install

```bash
pip install -e software/plugins/xlerobot_table_alignment
cd software/examples/table_alignment
```

Stop teleoperation first: these scripts need the motor ports and the camera to themselves. Calibration and logs go
to `outputs/table_alignment/` under the current directory, so always run from the same folder.

Defaults are `--port /dev/ttyACM1` for the wheels and `--head-port /dev/ttyACM0` for the head. The numbers can swap
between boots. Both scripts check by motor ID before writing anything, and tell you if the port is the wrong one.

Stow the arms first. Nothing here commands them, and the base moves whether or not they hold torque.

## 1. Wheel directions

On clear floor, away from the table:

```bash
python wheels.py --pulse-test
python wheels.py
```

Each key is one 0.3 s pulse. `W` must go forward, `A` left and `Q` counter-clockwise from above. If a direction is
wrong, **stop**: the alignment depends on these signs.

While here, check that each key moves the base at all and in a straight line. The cart's casters must swivel before
it tracks, and short pulses may stall (see Tuning).

## 2. Head pose

Stand the robot about where it will stop at the table. Point the head so the tag is well inside the image, then:

```bash
python table_perception.py snapshot   # saves outputs/table_alignment/snapshot.png with detected tags drawn
python head_pose.py capture
```

The head now holds this pose, and it stays held after the script exits. Everything after this is valid only at this
pose. If you capture again, redo steps 3 and 5. `head_pose.py goto` slowly returns to it after a power cycle, and
`head_pose.py check` shows how far off it is.

## 3. Camera calibration

```bash
python table_perception.py realsense-intrinsics
```

Lay the tag flat on the floor where the head camera sees it, with its printed top pointing away from the robot.
Measure from the base centre (the middle of the three omniwheels) to the tag centre:

```bash
python table_perception.py calibrate-extrinsics --tag-x 0.60 --tag-y 0.0
```

Check the printed camera position against a tape measure.

## 4. Tag on the table

```bash
python table_perception.py watch --tag-edge-offset 0.3 --tag-yaw-deg -90
```

`--tag-edge-offset` is the tag centre's distance in from the table's front edge. Move the robot by hand: distance,
lateral and heading errors must change in the expected direction.

## 5. Teach the target

Park the robot by hand exactly where it should end up, then:

```bash
python table_perception.py teach --tag-edge-offset 0.3 --tag-yaw-deg -90
```

## 6. Align

One axis at a time from a small offset, then all together:

```bash
python align_to_table.py --tag-edge-offset 0.3 --tag-yaw-deg -90 --axes heading --dry-run
python align_to_table.py --tag-edge-offset 0.3 --tag-yaw-deg -90 --axes heading
python align_to_table.py --tag-edge-offset 0.3 --tag-yaw-deg -90 --axes lateral
python align_to_table.py --tag-edge-offset 0.3 --tag-yaw-deg -90 --axes distance
python align_to_table.py --tag-edge-offset 0.3 --tag-yaw-deg -90 --axes heading,lateral,distance --note spot1
```

For a round table, add `--table-radius 0.3`. Measure the robot before orbiting: the keep-out radius defaults to table
radius + 0.40 m, which assumes nothing reaches further than 0.30 m from the base centre. That is the bare cart, so if
the arms stick out further, set `--keepout-radius`.

`align_report.py` tabulates the trial logs.

## Tuning

Set on AlohaMini and not yet checked here:

- `--min-linear-speed` (0.01 m/s) and `--min-heading-speed` (3 deg/s). Raise them if small corrections do nothing.
  The base has weaker motors than AlohaMini's and drags a cart. If alignment faults with an error that does not
  shrink, try this first.
- Head tolerance: 6 ticks (about 0.5 deg), `DEFAULT_TOLERANCE_TICKS` in `head.py`. Raise it only if the head
  holds still but the check still faults.

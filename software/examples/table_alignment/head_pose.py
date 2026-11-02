#!/usr/bin/env python3
"""
Save and restore the head pose the head camera is calibrated at. The wheels never move.

The camera calibration and the taught target are only valid at this pose, so choose it once and keep it: point
the head at where the tag will be when the robot stands at the table, with the tag well inside the image.

  python head_pose.py capture            # hold the head where it is and save that as the pose
  python head_pose.py check              # how far is the head from the saved pose? (read-only)
  python head_pose.py goto               # ramp slowly back to the saved pose

Position the head first (teleop, or by hand with the torque off), then capture. Capturing again invalidates the
extrinsics and target: redo table_perception.py calibrate-extrinsics and teach afterwards.
"""

import argparse
import logging
import sys
from pathlib import Path

from table_perception import DEFAULTS

from xlerobot_table_alignment.head import TICKS_PER_DEG, HeadHold, HeadMovedError, HeadPose, open_head_bus


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=("capture", "check", "goto"))
    p.add_argument("--head-port", default=DEFAULTS["head_port"], help="Serial port of the bus with the head motors")
    p.add_argument("--head-pose", default=str(DEFAULTS["head_pose"]))
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.command != "capture" and not Path(args.head_pose).exists():
        raise SystemExit(f"No saved head pose at {args.head_pose}; run capture first")

    head = HeadHold(open_head_bus(args.head_port))
    try:
        if args.command == "capture":
            pose = head.capture()
            pose.save(args.head_pose)
            print(f"Holding the head. Saved {pose.present} to {args.head_pose}")
            print("If this replaces an earlier pose, redo calibrate-extrinsics and teach.")
            return 0
        pose = HeadPose.load(args.head_pose)
        if args.command == "goto":
            head.goto(pose)
        deviation = head.deviation(pose)
        print({name: f"{ticks:+d} ticks ({ticks / TICKS_PER_DEG:+.1f} deg)" for name, ticks in deviation.items()})
        head.check(pose)
        print("Head is at the saved pose.")
        return 0
    except HeadMovedError as e:
        print(e)
        return 1
    finally:
        head.close()


if __name__ == "__main__":
    sys.exit(main())

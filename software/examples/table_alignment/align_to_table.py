#!/usr/bin/env python3
"""
Closed-loop base alignment to the taught table pose.

Bring the axes up one at a time:
  --axes heading
  --axes lateral
  --axes distance
  --axes heading,lateral,distance     full alignment -> BASE_READY

Prerequisites: a saved head pose (head_pose.py), camera intrinsics/extrinsics and a taught target
(table_perception.py), nothing else holding the camera or motor ports, base power on, and someone ready to cut
base power. The head is held at the saved pose and checked before every pulse; if it moved, alignment faults. Every pulse is <= 0.3 s and ends
in a verified stop; any lost/stale tag, timeout, or error that grows after a correction stops the base.

  python align_to_table.py --tag-edge-offset 0.15 --axes heading --dry-run
  python align_to_table.py --tag-edge-offset 0.15 --axes heading

If the tag is not usable at the start, the robot first rotates in place to find it (--search-direction), and
if it stands off to the side of the table's approach line it circles the approach point at constant distance
until it is roughly in front (--no-search / --no-orbit disable these).

Once the tag has been seen, every pulse is planned so the tag stays inside the image (shortened, turned to keep
looking at the tag, or another axis corrected first), the tag is tracked during the pulse and the pulse ends
early if it nears the image edge, and if it is lost anyway the pulse is reversed and, if needed, the base turns
+-20 deg to find it again (--recover-scan-deg; --no-keep-view disables all of this).

A JSONL log of every measurement, pulse, stop and state transition is written per trial, with the reading
before the start ("initial"), the start pose ("start_pose": measured, or estimated after a blind search) and a
"summary" at the end. align_report.py tabulates many trials.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

from table_perception import (
    OUT,
    LatestFrame,
    add_camera_args,
    head_args,
    load_target,
    make_estimator,
    table_args,
    target_placement_mismatch,
)

from xlerobot_table_alignment.base import (
    BASE_RADIUS_M,
    WHEEL_RADIUS_M,
    BaseController,
    BaseLimits,
    open_wheel_bus,
)
from xlerobot_table_alignment.behaviors import AXES, ERROR_FIELD, AlignConfig, AlignToTable
from xlerobot_table_alignment.head import HeadGuardedBase, HeadHold, HeadMovedError, HeadPose, open_head_bus
from xlerobot_table_alignment.perception import CameraIntrinsics, table_pose_error

DEFAULT_PORT = "/dev/ttyACM1"

logger = logging.getLogger("align_to_table")


def describe_error(e) -> str:
    if e is None:
        return "unknown"
    return (
        f"distance {e.distance_error_m * 100:+.1f} cm, lateral {e.lateral_error_m * 100:+.1f} cm, "
        f"heading {e.heading_error_deg:+.1f} deg"
    )


def _raise_interrupt(signum, _frame):
    raise KeyboardInterrupt(f"signal {signum}")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_camera_args(p)
    table_args(p)
    head_args(p)
    p.add_argument("--port", default=DEFAULT_PORT, help="Wheel serial port")
    p.add_argument("--axes", default="heading", help=f"Comma-separated subset of {','.join(AXES)}")
    p.add_argument(
        "--dry-run", action="store_true", help="Measure and print the first planned pulse; no motion"
    )
    p.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    p.add_argument("--log-dir", default=str(OUT / "logs"))
    p.add_argument(
        "--note", default="", help="Free text saved with the trial (e.g. the start spot), for reports"
    )
    p.add_argument("--timeout-s", type=float, default=AlignConfig.timeout_s, help="Overall alignment timeout")
    p.add_argument(
        "--max-travel",
        type=float,
        default=AlignConfig.max_total_translation_m,
        help="Net translation (m) the heading/lateral/distance stages may command in one run "
        "(search and orbit not counted)",
    )
    p.add_argument(
        "--min-linear-speed",
        type=float,
        default=AlignConfig.distance.min_speed,
        help="Slowest sideways/forward pulse (m/s); raise it if short pulses stall against the cart's casters",
    )
    p.add_argument(
        "--min-heading-speed",
        type=float,
        default=AlignConfig.heading.min_speed,
        help="Slowest turning pulse (deg/s); raise it if short pulses stall against the cart's casters",
    )
    p.add_argument("--no-search", action="store_true", help="Fault instead of rotating to find the tag")
    p.add_argument(
        "--no-orbit",
        action="store_true",
        help="Do not circle the table to correct a robot standing off to the side of the approach line",
    )
    p.add_argument(
        "--table-radius",
        type=float,
        default=None,
        help="Round table radius (m): orbit around the table centre, outside --keepout-radius "
        "(default: straight edge, orbit around the approach point)",
    )
    p.add_argument(
        "--keepout-radius",
        type=float,
        default=None,
        help="With --table-radius: minimum distance (m) from the table centre to the base centre while orbiting "
        f"(default: table radius + {AlignConfig.orbit_footprint_clearance_m})",
    )
    p.add_argument(
        "--orbit-radius",
        type=float,
        default=None,
        help="With --table-radius: before orbiting, move to this distance (m) from the table centre and orbit "
        "there (default: keep-out radius + 0.03)",
    )
    p.add_argument(
        "--no-keep-view",
        action="store_true",
        help="Do not plan/watch pulses to keep the tag in the image, or reverse a pulse that lost it",
    )
    p.add_argument(
        "--recover-scan-deg",
        type=float,
        default=AlignConfig.recover_scan_deg,
        help="If the tag is still lost after reversing a pulse, turn up to +- this much to find it (0 = off)",
    )
    p.add_argument(
        "--search-direction",
        choices=("ccw", "cw"),
        default="ccw",
        help="Rotation direction while the tag is not in view (viewed from above)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    signal.signal(signal.SIGTERM, _raise_interrupt)
    signal.signal(signal.SIGHUP, _raise_interrupt)

    axes = tuple(a.strip() for a in args.axes.split(",") if a.strip())
    config = AlignConfig(
        axes=axes,
        heading=dataclasses.replace(AlignConfig.heading, min_speed=args.min_heading_speed),
        lateral=dataclasses.replace(AlignConfig.lateral, min_speed=args.min_linear_speed),
        distance=dataclasses.replace(AlignConfig.distance, min_speed=args.min_linear_speed),
        timeout_s=args.timeout_s,
        max_total_translation_m=args.max_travel,
        search=not args.no_search,
        orbit=not args.no_orbit,
        keep_tag_in_view=not args.no_keep_view,
        recover_scan_deg=args.recover_scan_deg,
        table_radius_m=args.table_radius,
        orbit_keepout_radius_m=args.keepout_radius,
        orbit_radius_m=args.orbit_radius,
        search_direction=1 if args.search_direction == "ccw" else -1,
    )

    if not Path(args.target).exists():
        raise SystemExit(f"No taught target at {args.target}; run table_perception.py teach first")
    if (mismatch := target_placement_mismatch(args)) is not None:
        raise SystemExit(f"Refusing to align: {mismatch}. Use the same tag options or re-teach.")
    target = load_target(args)
    if not Path(args.head_pose).exists():
        raise SystemExit(f"No saved head pose at {args.head_pose}; run head_pose.py capture first")
    head_pose = HeadPose.load(args.head_pose)

    head = HeadHold(open_head_bus(args.head_port))
    try:
        # A dry run only reads the head; a real run holds it too.
        (head.check if args.dry_run else head.engage)(head_pose)
    except HeadMovedError as e:
        head.close()
        raise SystemExit(f"Refusing to align: {e}") from e
    except BaseException:
        head.close()
        raise

    intr = CameraIntrinsics.load(args.intrinsics)
    cam = LatestFrame(args.camera, *intr.image_size, fourcc=args.fourcc)
    estimator = make_estimator(args, cam, target)
    try:
        cam.wait_first()
        time.sleep(0.5)
        err = estimator.get_table_pose_error()
        initial = {
            "event": "initial",
            "valid": err.valid,
            "reason": err.reason,
            "error": asdict(err),
            "tag_bearing_deg": estimator.last_tag_bearing_deg,
            "tag_edge_margin_px": estimator.last_tag_margin_px,
        }
        if not err.valid and estimator.last_raw_T_base_table is not None:
            # Tag fitted but rejected (e.g. heading beyond the limit): record the pose anyway.
            initial["raw_error"] = asdict(
                table_pose_error(estimator.last_raw_T_base_table, target, err.timestamp_s)
            )
        if err.valid:
            print(
                f"Current error: distance {err.distance_error_m * 100:+.1f} cm, "
                f"lateral {err.lateral_error_m * 100:+.1f} cm, heading {err.heading_error_deg:+.1f} deg. "
                f"Correcting: {', '.join(axes)}"
            )
        elif config.search:
            if "raw_error" in initial:
                raw = initial["raw_error"]
                print(
                    f"Tag seen but rejected: distance {raw['distance_error_m'] * 100:+.1f} cm, "
                    f"lateral {raw['lateral_error_m'] * 100:+.1f} cm, heading {raw['heading_error_deg']:+.1f} deg"
                )
            print(
                f"Tag not usable yet ({err.reason}). Will rotate {args.search_direction.upper()} in "
                f"{config.search_step_deg:g} deg steps at {config.search_speed_degps:g} deg/s to find it "
                f"(up to {config.max_search_rotation_deg:g} deg), then align: {', '.join(axes)}"
            )
        else:
            raise SystemExit(f"Table pose invalid before starting: {err.reason}")

        if args.dry_run:
            if err.valid:
                for axis in axes:
                    ax, e = config.axis(axis), getattr(err, ERROR_FIELD[axis])
                    if abs(e) <= ax.tolerance:
                        plan = "within tolerance"
                    else:
                        plan = f"first pulse speed {max(-ax.max_speed, min(ax.max_speed, ax.gain * e)):+.3f}"
                    print(f"  {axis}: error {e:+.3f}, tolerance {ax.tolerance}, {plan}")
            print("Dry run: no motion.")
            return 0

        if not args.yes:
            input(
                "Base will move in short pulses. Hand on the power cutoff? Press Enter to start (Ctrl+C aborts). "
            )

        bus = open_wheel_bus(args.port)
        base = BaseController(
            bus,
            wheel_radius=WHEEL_RADIUS_M,
            base_radius=BASE_RADIUS_M,
            limits=BaseLimits(
                max_vx_mps=config.distance.max_speed,
                max_vy_mps=config.lateral.max_speed,
                max_omega_degps=config.heading.max_speed,
                max_pulse_s=config.max_pulse_s,
            ),
        )

        log_dir = Path(args.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"align_{time.strftime('%Y%m%d_%H%M%S')}_{'-'.join(axes)}.jsonl"
        exit_code = 1
        with log_path.open("w") as log_file:

            def log(event: dict) -> None:
                log_file.write(json.dumps(event, default=str) + "\n")
                log_file.flush()

            log(
                {
                    "event": "trial",
                    "axes": axes,
                    "target": asdict(target),
                    "tag": vars(args),
                    "note": args.note,
                }
            )
            log(initial)
            try:
                result = AlignToTable(HeadGuardedBase(base, head, head_pose), estimator, config, log=log).run()
                print(
                    f"\n{result.state.value} after {result.steps} pulses in {result.duration_s:.0f} s. {result.reason}"
                )
                print(f"Start ({result.start_source}): {describe_error(result.start_error)}")
                print(f"Final error: {describe_error(result.final_error)}")
                summary = result.events[-1]
                print(
                    f"Searched {summary['search_rotation_deg']:+.0f} deg, moved {summary['radial_m'] * 100:+.0f} cm "
                    f"towards the table centre, orbited {summary['orbit_deg']:+.0f} deg, "
                    f"recoveries {summary['recoveries']}"
                )
                exit_code = 0 if result.ready else 1
                if result.ready:
                    # The head is checked before each pulse; this catches a move after the last one.
                    try:
                        head.check(head_pose)
                    except HeadMovedError as e:
                        log({"event": "fault", "reason": str(e), "steps": result.steps})
                        print(f"NOT READY: {e}")
                        exit_code = 1
            except KeyboardInterrupt:
                logger.info("Interrupted")
            finally:
                stop = base.stop_base()
                if not stop.confirmed:
                    logger.critical("Final stop not confirmed - CUT BASE POWER")
                    exit_code = 2
                try:
                    bus.disconnect(disable_torque=True)
                except Exception:
                    logger.exception("Disconnect / torque disable failed")
        print(f"Log: {log_path}")
        return exit_code
    finally:
        cam.close()
        head.close()


if __name__ == "__main__":
    sys.exit(main())

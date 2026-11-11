"""Keeping the head camera where it was calibrated.

The camera extrinsics and the taught target are only valid at one head pose: half a degree of tilt moves the
apparent tag position by about a centimetre at a metre's range. So the pose is saved next to the calibration,
held with torque on during alignment, and checked before every base pulse.

The head shares IDs 7 and 8 with two wheels on the other bus, and ``/dev/ttyACM*`` numbers can swap. Whether ID 9
answers tells the buses apart, and it is checked before anything is written.
"""

import json
import logging
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .base import BaseInterlockError

logger = logging.getLogger(__name__)

HEAD_MOTOR_MODEL = "sts3215"
HEAD_IDS = {"head_motor_1": 7, "head_motor_2": 8}
WHEEL_BUS_ONLY_ID = 9
TICKS_PER_DEG = 4096.0 / 360.0
# About 0.5 deg.
DEFAULT_TOLERANCE_TICKS = 6


class HeadBusMismatchError(RuntimeError):
    """The serial device does not look like the head bus."""


class HeadMovedError(BaseInterlockError):
    """The head is not at the calibrated pose, or its position could not be read."""


@dataclass(frozen=True)
class HeadPose:
    """Raw motor positions: ``goal`` is what to command, ``present`` what the motors then read (they sag)."""

    goal: dict[str, int]
    present: dict[str, int]

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "HeadPose":
        record = json.loads(Path(path).read_text())
        return cls(record["goal"], record["present"])


def open_head_bus(port: str) -> Any:
    """Connect to ``port`` and verify it is the head bus. Nothing is written to any motor."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    motors = {
        name: Motor(id=id_, model=HEAD_MOTOR_MODEL, norm_mode=MotorNormMode.RANGE_M100_100)
        for name, id_ in HEAD_IDS.items()
    }
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect(handshake=False)
    try:
        expected = bus.model_number_table[HEAD_MOTOR_MODEL]
        found = {name: bus.ping(name, num_retry=2) for name in motors}
        if any(model != expected for model in found.values()):
            raise HeadBusMismatchError(f"{port} is not the head bus: expected {expected} on {HEAD_IDS}, got {found}")
        if bus.ping(WHEEL_BUS_ONLY_ID, num_retry=2) is not None:
            raise HeadBusMismatchError(
                f"{port} is the wheel bus (ID {WHEEL_BUS_ONLY_ID} answers), not the head bus; try the other port"
            )
    except BaseException:
        bus.disconnect(disable_torque=False)
        raise
    return bus


class HeadHold:
    """Holds the head on a verified bus. Closing leaves the torque on, so the head does not drop."""

    def __init__(
        self,
        bus: Any,
        *,
        tolerance_ticks: int = DEFAULT_TOLERANCE_TICKS,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.bus = bus
        self.tolerance_ticks = tolerance_ticks
        self._sleep = sleep

    def read(self) -> dict[str, int]:
        return {name: int(self.bus.read("Present_Position", name, normalize=False)) for name in HEAD_IDS}

    def _hold_here(self) -> dict[str, int]:
        """Torque on at the current position, without a jump: the goal is set first. Position mode is required."""
        for name in HEAD_IDS:
            mode = int(self.bus.read("Operating_Mode", name, normalize=False))
            if mode != 0:
                raise HeadBusMismatchError(f"{name} is in operating mode {mode}, expected position mode (0)")
        present = self.read()
        for name, value in present.items():
            self.bus.write("Goal_Position", name, value, normalize=False)
        self.bus.enable_torque(list(HEAD_IDS))
        return present

    def capture(self, settle_s: float = 1.0) -> HeadPose:
        """Hold the head where it is and return that as the calibrated pose."""
        goal = self._hold_here()
        self._sleep(settle_s)
        return HeadPose(goal, self.read())

    def deviation(self, pose: HeadPose) -> dict[str, int]:
        present = self.read()
        return {name: present[name] - pose.present[name] for name in HEAD_IDS}

    def check(self, pose: HeadPose) -> None:
        """Raise HeadMovedError unless the head is within tolerance of ``pose``. Read-only."""
        try:
            deviation = self.deviation(pose)
        except Exception as e:
            raise HeadMovedError(f"cannot read the head position: {e!r}") from e
        if any(abs(ticks) > self.tolerance_ticks for ticks in deviation.values()):
            degrees = {name: round(ticks / TICKS_PER_DEG, 1) for name, ticks in deviation.items()}
            raise HeadMovedError(f"head moved from the calibrated pose by {degrees} deg; run head_pose.py goto")

    def engage(self, pose: HeadPose) -> None:
        """Hold the head and verify it is at ``pose``. Never moves it towards the pose: see :meth:`goto`."""
        self._hold_here()
        self._sleep(0.3)
        self.check(pose)

    def goto(
        self,
        pose: HeadPose,
        *,
        speed_ticks_s: float = 100.0,
        max_travel_ticks: int = 500,
        rate_hz: float = 20.0,
        settle_s: float = 1.0,
    ) -> None:
        """Ramp slowly to ``pose``. Refuses a long move, which would mean the wrong file or a wrong head."""
        start = self.read()
        far = {name: pose.goal[name] - start[name] for name in HEAD_IDS}
        if any(abs(ticks) > max_travel_ticks for ticks in far.values()):
            raise HeadMovedError(f"head is {far} ticks from the saved pose (limit {max_travel_ticks}); move it closer")
        self._hold_here()
        steps = max(1, int(max(abs(t) for t in far.values()) / speed_ticks_s * rate_hz))
        for step in range(1, steps + 1):
            for name in HEAD_IDS:
                value = round(start[name] + far[name] * step / steps)
                self.bus.write("Goal_Position", name, value, normalize=False)
            self._sleep(1.0 / rate_hz)
        self._sleep(settle_s)
        self.check(pose)

    def close(self) -> None:
        self.bus.disconnect(disable_torque=False)


class HeadGuardedBase:
    """A base controller that checks the head pose before commanding each pulse. Stops are never blocked."""

    def __init__(self, base: Any, head: HeadHold, pose: HeadPose) -> None:
        self.base = base
        self.head = head
        self.pose = pose

    def move_base_for(self, *args: Any, **kwargs: Any) -> Any:
        self.head.check(self.pose)
        return self.base.move_base_for(*args, **kwargs)

    def stop_base(self) -> Any:
        return self.base.stop_base()

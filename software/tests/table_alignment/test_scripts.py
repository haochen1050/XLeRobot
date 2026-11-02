"""The align_to_table.py wiring, with only the hardware faked."""

import json
import sys
import time
from pathlib import Path

import pytest
from xlerobot_table_alignment.head import HeadPose
from xlerobot_table_alignment.perception import TablePoseError, TableTarget

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "table_alignment"))
import align_to_table as script  # noqa: E402

GOAL = {"head_motor_1": 2000, "head_motor_2": 1500}
WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")


class FakeBus:
    """Head and wheels both: reads the head position, remembers writes."""

    def __init__(self, drift=0):
        self.present = dict(GOAL)
        self.drift = drift
        self.goal = dict.fromkeys(WHEELS, 0)
        self.torque = False
        self.disconnects = []

    def read(self, data, name, normalize=True):
        if data == "Present_Position":
            return self.present[name] + self.drift
        return 0 if data == "Operating_Mode" else self.goal[name]

    def write(self, data, name, value, normalize=True):
        if data == "Goal_Velocity":
            self.goal[name] = value

    def enable_torque(self, motors):
        self.torque = True

    def disconnect(self, disable_torque):
        self.disconnects.append(disable_torque)


class FakeCamera:
    def __init__(self, *args):
        self.closed = False

    def wait_first(self):
        pass

    def close(self):
        self.closed = True


class ReadyEstimator:
    """Already at the target."""

    last_tag_bearing_deg = 0.0
    last_tag_margin_px = 100.0
    last_raw_T_base_table = None

    def get_table_pose_error(self):
        return TablePoseError(0.0, 0.0, 0.0, time.monotonic(), True)


@pytest.fixture
def run(monkeypatch, tmp_path):
    def run(head_bus, *extra):
        target, pose = tmp_path / "target.json", tmp_path / "head.json"
        target.write_text("{}")
        HeadPose(GOAL, GOAL).save(pose)
        wheel_bus = FakeBus()
        monkeypatch.setattr(script, "open_head_bus", lambda port: head_bus)
        monkeypatch.setattr(script, "open_wheel_bus", lambda port: wheel_bus)
        monkeypatch.setattr(script, "LatestFrame", FakeCamera)
        monkeypatch.setattr(script, "make_estimator", lambda *a: ReadyEstimator())
        monkeypatch.setattr(script, "load_target", lambda args: TableTarget(distance_m=0.3))
        monkeypatch.setattr(script, "target_placement_mismatch", lambda args: None)
        monkeypatch.setattr(
            script.CameraIntrinsics, "load", lambda path: type("I", (), {"image_size": (640, 480)})
        )
        monkeypatch.setattr(
            sys,
            "argv",
            [
                "align_to_table.py",
                "--tag-edge-offset=0.3",
                f"--target={target}",
                f"--head-pose={pose}",
                f"--log-dir={tmp_path}",
                "--axes=heading,lateral,distance",
                "--min-linear-speed=0.02",
                "--min-heading-speed=5",
                "--yes",
                *extra,
            ],
        )
        return script.main(), wheel_bus, sorted(tmp_path.glob("align_*.jsonl"))

    return run


def test_ready_run_holds_the_head_and_writes_a_log(run):
    head = FakeBus()
    code, wheels, logs = run(head)
    assert code == 0
    assert head.torque and head.disconnects == [False]
    assert wheels.disconnects == [True]
    events = [json.loads(line) for line in logs[0].read_text().splitlines()]
    assert events[-1]["event"] == "summary" and events[-1]["state"] == "BASE_READY"
    start = next(e for e in events if e["event"] == "start")
    assert start["config"]["lateral"]["min_speed"] == 0.02
    assert start["config"]["heading"]["min_speed"] == 5.0


def test_dry_run_never_touches_the_torque_or_the_wheels(run, monkeypatch):
    head = FakeBus()
    code, wheels, logs = run(head, "--dry-run")
    assert code == 0 and not head.torque and head.disconnects == [False]
    assert not wheels.disconnects and not logs


def test_refuses_to_start_with_the_head_away_from_the_pose(run):
    head = FakeBus(drift=50)
    with pytest.raises(SystemExit, match="Refusing to align"):
        run(head)
    assert head.disconnects == [False]


def test_head_moving_after_the_last_pulse_is_not_ready(run, monkeypatch):
    head = FakeBus()
    real_run = script.AlignToTable.run

    def run_then_move(self):
        result = real_run(self)
        assert result.ready  # alignment itself succeeded; only the check after it may fail
        head.drift = 50
        return result

    monkeypatch.setattr(script.AlignToTable, "run", run_then_move)
    code, _, logs = run(head)
    assert code == 1
    assert '"fault"' in logs[0].read_text()

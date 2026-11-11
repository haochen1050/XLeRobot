import pytest
from xlerobot_table_alignment.base import BaseInterlockError, StopResult
from xlerobot_table_alignment.behaviors import AlignConfig, AlignState, AlignToTable
from xlerobot_table_alignment.head import (
    HeadBusMismatchError,
    HeadGuardedBase,
    HeadHold,
    HeadMovedError,
    HeadPose,
    open_head_bus,
)
from xlerobot_table_alignment.perception import TablePoseError

NAMES = ("head_motor_1", "head_motor_2")
POSE = HeadPose({"head_motor_1": 2000, "head_motor_2": 1500}, {"head_motor_1": 1998, "head_motor_2": 1497})


class FakeHeadBus:
    def __init__(self, present=None, mode=0):
        self.present = dict(present or POSE.present)
        self.mode = mode
        self.goal = dict.fromkeys(NAMES)
        self.goals: list[dict] = []
        self.torque = False
        self.torque_when_goal_set = None
        self.disconnects = []

    def read(self, data, name, normalize=True):
        assert not normalize
        return {"Present_Position": self.present[name], "Operating_Mode": self.mode}[data]

    def write(self, data, name, value, normalize=True):
        assert data == "Goal_Position" and not normalize
        if self.torque_when_goal_set is None:
            self.torque_when_goal_set = self.torque
        self.goal[name] = value
        self.goals.append(dict(self.goal))

    def enable_torque(self, motors):
        assert list(motors) == list(NAMES)
        self.torque = True

    def disconnect(self, disable_torque):
        self.disconnects.append(disable_torque)


def hold(bus):
    return HeadHold(bus, sleep=lambda _s: None)


def test_pose_round_trips(tmp_path):
    POSE.save(tmp_path / "x" / "pose.json")
    assert HeadPose.load(tmp_path / "x" / "pose.json") == POSE


def test_capture_sets_the_goal_before_torque_so_the_head_does_not_jump():
    bus = FakeHeadBus({"head_motor_1": 100, "head_motor_2": 200})
    pose = hold(bus).capture()
    assert bus.torque and bus.torque_when_goal_set is False
    assert pose.goal == {"head_motor_1": 100, "head_motor_2": 200} == bus.goal


def test_check_accepts_the_tolerance_and_rejects_beyond_it():
    bus = FakeHeadBus({"head_motor_1": 1998 + 6, "head_motor_2": 1497 - 6})
    hold(bus).check(POSE)
    bus.present["head_motor_2"] = 1497 - 7
    with pytest.raises(HeadMovedError, match="head moved"):
        hold(bus).check(POSE)
    assert not bus.goals and not bus.torque  # read-only


def test_a_failed_read_counts_as_moved():
    bus = FakeHeadBus()
    bus.read = lambda *a, **k: (_ for _ in ()).throw(OSError("no answer"))
    with pytest.raises(HeadMovedError, match="cannot read"):
        hold(bus).check(POSE)


def test_engage_holds_but_never_drives_towards_the_pose():
    bus = FakeHeadBus({"head_motor_1": 2100, "head_motor_2": 1497})
    with pytest.raises(HeadMovedError):
        hold(bus).engage(POSE)
    assert bus.goal == {"head_motor_1": 2100, "head_motor_2": 1497}


def test_engage_refuses_a_motor_not_in_position_mode():
    bus = FakeHeadBus(mode=1)
    with pytest.raises(HeadBusMismatchError, match="position mode"):
        hold(bus).engage(POSE)
    assert not bus.goals and not bus.torque


def test_goto_ramps_in_small_steps_and_ends_on_the_goal():
    bus = FakeHeadBus({"head_motor_1": 2100, "head_motor_2": 1500})

    def write(data, name, value, normalize=True):
        FakeHeadBus.write(bus, data, name, value, normalize)
        bus.present[name] = value - (POSE.goal[name] - POSE.present[name])  # sags like the captured pose

    bus.write = write
    hold(bus).goto(POSE)
    assert bus.goal == POSE.goal
    first = [g["head_motor_1"] for g in bus.goals if g["head_motor_1"] is not None]
    assert max(abs(a - b) for a, b in zip(first, first[1:], strict=False)) <= 100
    hold(bus).check(POSE)


def test_goto_refuses_a_long_move():
    bus = FakeHeadBus({"head_motor_1": 3000, "head_motor_2": 1500})
    with pytest.raises(HeadMovedError, match="move it closer"):
        hold(bus).goto(POSE)
    assert not bus.goals


def test_close_keeps_the_torque_on():
    bus = FakeHeadBus()
    hold(bus).close()
    assert bus.disconnects == [False]


@pytest.mark.parametrize(
    ("answering", "match"),
    [
        ({7: 777, 8: 777, 9: 777}, "wheel bus"),  # the wheels share IDs 7 and 8 with the head
        ({7: 777}, "not the head bus"),
    ],
)
def test_open_head_bus_rejects_the_wrong_bus_without_writing(monkeypatch, answering, match):
    disconnected = []

    from lerobot.motors.feetech import FeetechMotorsBus

    class FakeFeetech:
        model_number_table = FeetechMotorsBus.model_number_table

        def __init__(self, port, motors):
            self.ids = {name: motor.id for name, motor in motors.items()}

        def connect(self, handshake):
            pass

        def ping(self, motor, num_retry=0):
            return answering.get(self.ids.get(motor, motor))

        def disconnect(self, disable_torque):
            disconnected.append(disable_torque)

    monkeypatch.setattr("lerobot.motors.feetech.FeetechMotorsBus", FakeFeetech)
    with pytest.raises(HeadBusMismatchError, match=match):
        open_head_bus("/dev/null")
    assert disconnected == [False]


class CountingBase:
    def __init__(self):
        self.pulses = 0
        self.stops = 0

    def move_base_for(self, *args, **kwargs):
        self.pulses += 1
        return StopResult(True, 1, elapsed_s=0.3)

    def stop_base(self):
        self.stops += 1
        return StopResult(True, 1)


def test_guard_blocks_pulses_but_never_stops():
    bus, inner = FakeHeadBus(), CountingBase()
    base = HeadGuardedBase(inner, hold(bus), POSE)
    base.move_base_for(0.01, 0, 0, 0.3)
    assert inner.pulses == 1

    bus.present["head_motor_1"] += 50
    with pytest.raises(BaseInterlockError):
        base.move_base_for(0.01, 0, 0, 0.3)
    assert inner.pulses == 1
    assert base.stop_base().confirmed


class FixedEstimator:
    def __init__(self):
        self.t = 100.0

    def get_table_pose_error(self):
        self.t += 1.0
        return TablePoseError(0.10, 0.0, 0.0, self.t, True)


def test_alignment_faults_when_the_head_moves():
    bus, inner = FakeHeadBus(), CountingBase()
    bus.present["head_motor_1"] += 50
    now = [0.0]

    def sleep(s):
        now[0] += s

    def clock():
        now[0] += 0.01
        return now[0]

    aligner = AlignToTable(
        HeadGuardedBase(inner, hold(bus), POSE),
        FixedEstimator(),
        AlignConfig(search=False, orbit=False, keep_tag_in_view=False),
        clock=clock,
        sleep=sleep,
    )
    result = aligner.run()
    assert result.state is AlignState.FAULT
    assert "head moved" in result.reason
    assert inner.pulses == 0 and inner.stops >= 1

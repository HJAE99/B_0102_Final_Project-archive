from scv_msgs.msg import Candidate

from scv_manager.inspection_manager import InspectionManager
from scv_manager.manager_core import ManagerCore, RunState, TaskState


class FakeLogger:
    def info(self, message: str) -> None:
        del message


class FakeInspectionManager:
    """ROS 노드를 띄우지 않고 후보 처리 콜백만 시험한다."""

    def __init__(self, core: ManagerCore):
        self.core = core
        self.survey_robot_id = "robot4"
        self.seen_candidate_ids = set()
        self.alerts = []

    def publish_alert(self, **kwargs) -> None:
        self.alerts.append(kwargs)

    def get_logger(self) -> FakeLogger:
        return FakeLogger()


def make_candidate(
    run_id: int,
    candidate_id: str = "candidate-1",
) -> Candidate:
    candidate = Candidate()
    candidate.run_id = run_id
    candidate.candidate_id = candidate_id
    candidate.source = Candidate.SOURCE_NETWORK
    candidate.ref = "cell-Z1-01"
    candidate.x = 1.0
    candidate.y = 2.0
    candidate.yaw = 0.0
    return candidate


def test_duplicate_candidate_creates_only_one_task():
    core = ManagerCore(
        robot_ids=["robot3", "robot4"],
        offline_timeout_sec=29.0,
    )
    run_id = core.start_run(timestamp_sec=1000)
    manager = FakeInspectionManager(core)
    candidate = make_candidate(run_id)

    InspectionManager.on_candidate(manager, candidate)
    InspectionManager.on_candidate(manager, candidate)

    assert len(core.tasks) == 1
    assert manager.seen_candidate_ids == {"candidate-1"}
    assert len(manager.alerts) == 1

    task = next(iter(core.tasks.values()))
    assert task.state == TaskState.PENDING
    assert task.preferred_robot_id == "robot4"
    assert task.candidate_id == "candidate-1"


def test_candidate_with_different_run_id_is_ignored():
    core = ManagerCore(
        robot_ids=["robot3", "robot4"],
        offline_timeout_sec=29.0,
    )
    run_id = core.start_run(timestamp_sec=1000)
    manager = FakeInspectionManager(core)

    candidate = make_candidate(run_id + 1)
    InspectionManager.on_candidate(manager, candidate)

    assert core.tasks == {}
    assert manager.seen_candidate_ids == set()
    assert manager.alerts == []


def test_candidate_reopens_completed_run():
    core = ManagerCore(
        robot_ids=["robot3", "robot4"],
        offline_timeout_sec=29.0,
    )
    run_id = core.start_run(timestamp_sec=1000)
    core.complete_run()
    manager = FakeInspectionManager(core)

    assert core.run_state == RunState.COMPLETED

    candidate = make_candidate(run_id)
    InspectionManager.on_candidate(manager, candidate)

    assert core.run_state == RunState.RUNNING
    assert len(core.tasks) == 1

    task = next(iter(core.tasks.values()))
    assert task.state == TaskState.PENDING
    assert task.preferred_robot_id == "robot4"
    assert task.allowed_robot_ids == ("robot4",)

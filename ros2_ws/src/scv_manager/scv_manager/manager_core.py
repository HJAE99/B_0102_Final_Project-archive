from dataclasses import dataclass
from enum import Enum
from typing import Optional


class RunState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    ESTOPPED = "ESTOPPED"
    COMPLETED = "COMPLETED"


class TaskState(str, Enum):
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    BLOCKED = "BLOCKED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RobotStatusSnapshot:
    """RobotStatus 메시지에서 받은 최신 상태."""

    state: int
    battery_percent: float
    accepting_tasks: bool
    task_id: str
    run_id: int
    task_type: int
    progress: float
    warning: str


@dataclass
class RobotRecord:
    """관제에서 관리하는 로봇의 최신 상태와 수신 시각."""

    robot_id: str
    online: bool = False
    last_seen: Optional[float] = None
    status: Optional[RobotStatusSnapshot] = None


@dataclass
class TaskRecord:
    task_id: str
    run_id: int
    task_type: int
    priority: int
    target_ref: str
    preferred_robot_id: str = ""
    allowed_robot_ids: tuple[str, ...] = ()
    waypoints: tuple[Waypoint, ...] = ()
    candidate_id: str = ""
    state: TaskState = TaskState.PENDING
    robot_id: Optional[str] = None
    progress: float = 0.0
    reason: str = ""


TERMINAL_TASK_STATES = {
    TaskState.SUCCEEDED,
    TaskState.BLOCKED,
    TaskState.FAILED,
}


class ManagerCore:
    """ROS에 의존하지 않는 로봇·순찰·작업 상태 관리 로직."""

    def __init__(
        self,
        robot_ids: list[str],
        offline_timeout_sec: float = 30.0,
    ):
        self.offline_timeout_sec = offline_timeout_sec
        self.robots = {
            robot_id: RobotRecord(robot_id=robot_id)
            for robot_id in robot_ids
        }
        self.run_id = 0
        self.run_state = RunState.IDLE
        self.tasks: dict[str, TaskRecord] = {}
        self.task_order: list[str] = []

    def update_robot(
        self,
        robot_id: str,
        now: float,
        status: RobotStatusSnapshot,
    ) -> None:
        robot = self.robots[robot_id]
        robot.online = True
        robot.last_seen = now
        robot.status = status

    def check_offline(self, now: float) -> list[str]:
        newly_offline = []

        for robot_id, robot in self.robots.items():
            if not robot.online or robot.last_seen is None:
                continue

            if now - robot.last_seen >= self.offline_timeout_sec:
                robot.online = False
                newly_offline.append(robot_id)

        return newly_offline

    def start_run(self) -> int:
        self.run_id += 1
        self.run_state = RunState.RUNNING
        self.tasks.clear()
        self.task_order.clear()
        return self.run_id

    def create_task(
        self,
        *,
        task_id: str,
        run_id: int,
        task_type: int,
        priority: int,
        target_ref: str,
        preferred_robot_id: str = "",
        allowed_robot_ids: tuple[str, ...] = (),
        waypoints: tuple[Waypoint, ...] = (),
        candidate_id: str = "",
    ) -> None:
        self.tasks[task_id] = TaskRecord(
            task_id=task_id,
            run_id=run_id,
            task_type=task_type,
            priority=priority,
            target_ref=target_ref,
            preferred_robot_id=preferred_robot_id,
            allowed_robot_ids=allowed_robot_ids,
            waypoints=waypoints,
            candidate_id=candidate_id,
        )
        self.task_order.append(task_id)

    def next_pending_task(self) -> Optional[TaskRecord]:
        if self.run_state != RunState.RUNNING:
            return None

        if any(
            task.state in {TaskState.ASSIGNED, TaskState.RUNNING}
            for task in self.tasks.values()
        ):
            return None

        for task_id in self.task_order:
            task = self.tasks[task_id]
            if task.state == TaskState.PENDING:
                return task

        return None

    def assign_task(self, task_id: str, robot_id: str) -> None:
        task = self.tasks[task_id]
        task.robot_id = robot_id
        task.state = TaskState.ASSIGNED
        task.reason = ""

    def start_task(self, task_id: str) -> None:
        self.tasks[task_id].state = TaskState.RUNNING

    def update_task_progress(self, task_id: str, progress: float) -> None:
        self.tasks[task_id].progress = progress

    def finish_task(self, task_id: str, *, ok: bool, reason: str) -> None:
        task = self.tasks[task_id]

        if ok:
            task.state = TaskState.SUCCEEDED
            task.progress = 1.0
            task.reason = ""
            return

        task.reason = reason
        task.state = (
            TaskState.BLOCKED
            if reason.upper() == "BLOCKED"
            else TaskState.FAILED
        )

    def requeue_task(self, task_id: str, reason: str = "") -> None:
        task = self.tasks[task_id]
        task.state = TaskState.PENDING
        task.robot_id = None
        task.progress = 0.0
        task.reason = reason

    def estop(self) -> None:
        self.run_state = RunState.ESTOPPED

        for task in self.tasks.values():
            if task.state in {TaskState.ASSIGNED, TaskState.RUNNING}:
                task.state = TaskState.CANCELED
                task.reason = "ESTOP"

    def resume(self) -> None:
        self.run_state = RunState.RUNNING

        for task in self.tasks.values():
            if task.state == TaskState.CANCELED:
                self.requeue_task(task.task_id)

    def all_tasks_terminal(self) -> bool:
        return bool(self.tasks) and all(
            task.state in TERMINAL_TASK_STATES
            for task in self.tasks.values()
        )

    def complete_run(self) -> None:
        self.run_state = RunState.COMPLETED

    def reopen_run(self) -> None:
        if self.run_state == RunState.COMPLETED:
            self.run_state = RunState.RUNNING

    def reassign_pending_tasks(
        self,
        from_robot_id: str,
        to_robot_id: str,
    ) -> int:
        moved_tasks = 0

        for task in self.tasks.values():
            if task.state != TaskState.PENDING:
                continue
            if task.preferred_robot_id != from_robot_id:
                continue
            if (
                task.allowed_robot_ids
                and to_robot_id not in task.allowed_robot_ids
            ):
                continue

            task.preferred_robot_id = to_robot_id
            moved_tasks += 1

        return moved_tasks

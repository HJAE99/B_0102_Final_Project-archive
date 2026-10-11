from dataclasses import dataclass
from typing import Optional

from enum import Enum

@dataclass(frozen=True)
class RobotStatusSnapshot:
    # RobotStatus 메시지에서 받은 최신 상태
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
    #관제에서 관리하는 로봇 상태.
    robot_id: str
    online: bool = False
    last_seen: Optional[float] = None
    status: Optional[RobotStatusSnapshot] = None

class TaskState(str, Enum):
    PENDING = 'PENDING'
    ASSIGNED = 'ASSIGNED'
    RUNNING = 'RUNNING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'

@dataclass
class TaskRecord:
    task_id: str
    run_id: int
    task_type: int
    priority: int
    target_ref: str

    state: TaskState = TaskState.PENDING
    robot_id: Optional[str] = None
    progress: float = 0.0
    reason: str = ""

class ManagerCore:
    def __init__(self, robot_ids: list[str], offline_timeout_sec: float = 30.0):
        self.offline_timeout_sec = offline_timeout_sec
        self.robots = {
            robot_id: RobotRecord(robot_id=robot_id)
            for robot_id in robot_ids
        }

        self.tasks: dict[str, TaskRecord] = {}

    def update_robot(self, robot_id: str, now: float, status: RobotStatusSnapshot) -> None:
        #AMR 최신 상태를 저장한다.
        if robot_id not in self.robots:
            raise KeyError(f"Unknown robot_id: {robot_id}")

        robot = self.robots[robot_id]

        robot.online = True
        robot.last_seen = now
        robot.status = status

    def check_offline(self, now:float) -> list[str]:
        newly_offline = []

        for robot_id, robot in self.robots.items():
            if not robot.online:
                continue
            if robot.last_seen is None:
                continue

            elapsed = now - robot.last_seen

            if elapsed >= self.offline_timeout_sec:
                robot.online = False
                newly_offline.append(robot_id)

        return newly_offline

    def create_task(
        self,
        *,
        task_id: str,
        run_id: int,
        task_type: int,
        priority: int,
        target_ref: str,
    ) -> None:
        self.tasks[task_id] = TaskRecord(
            task_id=task_id,
            run_id=run_id,
            task_type=task_type,
            priority=priority,
            target_ref=target_ref,
        )

    def assign_task(self, task_id: str, robot_id: str) -> None:
        task = self.tasks[task_id]
        task.robot_id = robot_id
        task.state = TaskState.ASSIGNED

    def start_task(self, task_id: str) -> None:
        task = self.tasks[task_id]
        task.state = TaskState.RUNNING

    def update_task_progress(self, task_id: str, progress: float) -> None:
        task = self.tasks[task_id]
        task.progress = progress

    def finish_task(self, task_id: str, *, ok: bool, reason: str) -> None:
        task = self.tasks[task_id]

        if ok:
            task.state = TaskState.SUCCEEDED
            task.progress = 1.0
            task.reason = ""
        else:
            task.state = TaskState.FAILED
            task.reason = reason

from dataclasses import dataclass
from typing import Optional

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


class ManagerCore:
    def __init__(self, robot_ids: list[str], offline_timeout_sec: float = 30.0):
        self.offline_timeout_sec = offline_timeout_sec
        self.robots = {
            robot_id: RobotRecord(robot_id=robot_id)
            for robot_id in robot_ids
        }

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

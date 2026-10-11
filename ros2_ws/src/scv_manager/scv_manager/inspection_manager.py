import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scv_msgs.action import ExecuteTask
from scv_msgs.msg import Alert, Candidate, InspectionResult, RobotStatus
from scv_msgs.srv import Reassign
from std_srvs.srv import Trigger

from .manager_core import (
    ManagerCore,
    RobotStatusSnapshot,
    RunState,
    TaskState,
    Waypoint,
)
from .route_builder import RouteBuilder


STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

CANDIDATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

INSPECTION_RESULT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InspectionManager(Node):
    def __init__(self):
        super().__init__("inspection_manager")

        self.declare_parameter("robot_ids", ["robot4", "robot3"])
        self.declare_parameter("survey_robot_id", "robot4")
        self.declare_parameter("inspection_robot_id", "robot3")
        self.declare_parameter("offline_timeout_sec", 30.0)
        self.declare_parameter("require_robot_ready", True)

        bringup_share = Path(get_package_share_directory("scv_bringup"))
        self.declare_parameter(
            "zones_file",
            str(bringup_share / "config" / "zones.yaml"),
        )
        self.declare_parameter(
            "facilities_file",
            str(bringup_share / "config" / "facilities.yaml"),
        )

        robot_ids = list(self.get_parameter("robot_ids").value)
        self.survey_robot_id = str(
            self.get_parameter("survey_robot_id").value
        )
        self.inspection_robot_id = str(
            self.get_parameter("inspection_robot_id").value
        )
        self.offline_timeout_sec = float(
            self.get_parameter("offline_timeout_sec").value
        )
        self.require_robot_ready = bool(
            self.get_parameter("require_robot_ready").value
        )

        role_robot_ids = {
            self.survey_robot_id,
            self.inspection_robot_id,
        }
        if not role_robot_ids.issubset(robot_ids):
            raise ValueError(
                "survey_robot_id와 inspection_robot_id는 "
                "robot_ids에 포함되어야 합니다."
            )

        self.core = ManagerCore(
            robot_ids=robot_ids,
            offline_timeout_sec=self.offline_timeout_sec,
        )
        self.route_builder = RouteBuilder(
            zones_file=str(self.get_parameter("zones_file").value),
            facilities_file=str(
                self.get_parameter("facilities_file").value
            ),
        )

        self.alert_publisher = self.create_publisher(
            Alert,
            "/inspection/alerts",
            10,
        )
        self.execute_task_clients = {
            robot_id: ActionClient(
                self,
                ExecuteTask,
                f"/{robot_id}/execute_task",
            )
            for robot_id in robot_ids
        }

        self.active_goals = {}
        self.canceling_task_ids = set()
        self.seen_candidate_ids = set()
        self.seen_inspection_results = set()
        self.unavailable_action_servers = set()

        self.status_subscriptions = [
            self.create_subscription(
                RobotStatus,
                f"/{robot_id}/status",
                lambda msg, robot_id=robot_id: self.on_robot_status(
                    robot_id,
                    msg,
                ),
                STATUS_QOS,
            )
            for robot_id in robot_ids
        ]
        self.candidate_subscription = self.create_subscription(
            Candidate,
            "/inspection/candidates",
            self.on_candidate,
            CANDIDATE_QOS,
        )
        self.inspection_result_subscription = self.create_subscription(
            InspectionResult,
            f"/{self.inspection_robot_id}/inspect/result",
            self.on_inspection_result,
            INSPECTION_RESULT_QOS,
        )

        self.start_service = self.create_service(
            Trigger,
            "/inspection/start",
            self.on_start,
        )
        self.estop_service = self.create_service(
            Trigger,
            "/inspection/estop",
            self.on_estop,
        )
        self.resume_service = self.create_service(
            Trigger,
            "/inspection/resume",
            self.on_resume,
        )
        self.reassign_service = self.create_service(
            Reassign,
            "/inspection/reassign",
            self.on_reassign,
        )

        self.watchdog_timer = self.create_timer(
            1.0,
            self.check_robot_timeouts,
        )
        self.dispatch_timer = self.create_timer(
            0.5,
            self.dispatch_next_task,
        )

        self.get_logger().info(
            "inspection_manager 시작: "
            f"RSSI={self.survey_robot_id}, "
            f"설비={self.inspection_robot_id}"
        )

    def on_robot_status(self, robot_id: str, msg: RobotStatus) -> None:
        status = RobotStatusSnapshot(
            state=msg.state,
            battery_percent=msg.battery_percent,
            accepting_tasks=msg.accepting_tasks,
            task_id=msg.task_id,
            run_id=msg.run_id,
            task_type=msg.task_type,
            progress=msg.progress,
            warning=msg.warning,
        )
        self.core.update_robot(
            robot_id=robot_id,
            now=time.monotonic(),
            status=status,
        )

    def check_robot_timeouts(self) -> None:
        for robot_id in self.core.check_offline(now=time.monotonic()):
            self.get_logger().error(
                f"{robot_id}: RobotStatus 수신 중단"
            )
            self.publish_alert(
                level=Alert.LEVEL_ERROR,
                code="E3",
                robot_id=robot_id,
                message=(
                    f"{self.offline_timeout_sec:g}초 동안 "
                    "RobotStatus가 수신되지 않았습니다."
                ),
            )

    def on_start(self, request, response):
        del request

        if self.core.run_state in {RunState.RUNNING, RunState.ESTOPPED}:
            response.success = False
            response.message = "이미 진행 중인 순찰이 있습니다."
            return response

        try:
            survey_routes = self.route_builder.build_survey_routes()
            facility_routes = self.route_builder.build_facility_routes()
        except (OSError, KeyError, TypeError, ValueError) as exc:
            response.success = False
            response.message = f"경로 설정을 읽을 수 없습니다: {exc}"
            self.get_logger().error(response.message)
            return response

        if not survey_routes or not facility_routes:
            response.success = False
            response.message = "zones.yaml과 facilities.yaml에 경로가 필요합니다."
            return response

        run_id = self.core.start_run()
        self.seen_candidate_ids.clear()
        self.seen_inspection_results.clear()

        for index, route in enumerate(survey_routes, start=1):
            self.core.create_task(
                task_id=f"run-{run_id}-survey-{index:03d}",
                run_id=run_id,
                task_type=ExecuteTask.Goal.SURVEY_SEGMENT,
                priority=1,
                target_ref=route.zone_id,
                preferred_robot_id=self.survey_robot_id,
                allowed_robot_ids=(self.survey_robot_id,),
                waypoints=route.waypoints,
            )

        for index, route in enumerate(facility_routes, start=1):
            self.core.create_task(
                task_id=f"run-{run_id}-facility-{index:03d}",
                run_id=run_id,
                task_type=ExecuteTask.Goal.FACILITY_CHECK,
                priority=1,
                target_ref=route.facility_id,
                preferred_robot_id=self.inspection_robot_id,
                allowed_robot_ids=(self.inspection_robot_id,),
                waypoints=route.waypoints,
            )

        task_count = len(self.core.tasks)
        response.success = True
        response.message = f"run_id={run_id}, 작업 {task_count}건 생성"
        self.publish_alert(
            level=Alert.LEVEL_INFO,
            code="RUN_STARTED",
            robot_id="",
            ref=str(run_id),
            message=response.message,
        )
        self.get_logger().info(response.message)
        return response

    def dispatch_next_task(self) -> None:
        if self.core.run_state != RunState.RUNNING:
            return
        if self.active_goals:
            return

        task = self.core.next_pending_task()
        if task is None:
            self.complete_run_if_ready()
            return

        robot_id = task.preferred_robot_id
        if not self.robot_is_ready(robot_id):
            return

        client = self.execute_task_clients[robot_id]
        if not client.server_is_ready():
            if robot_id not in self.unavailable_action_servers:
                self.get_logger().warning(
                    f"{robot_id}: ExecuteTask 액션 서버 대기 중"
                )
                self.unavailable_action_servers.add(robot_id)
            return

        self.unavailable_action_servers.discard(robot_id)
        self.send_task(task.task_id, robot_id)

    def robot_is_ready(self, robot_id: str) -> bool:
        if not self.require_robot_ready:
            return True

        robot = self.core.robots[robot_id]
        if not robot.online or robot.status is None:
            return False

        return (
            robot.status.accepting_tasks
            and robot.status.state
            in {RobotStatus.STATE_IDLE, RobotStatus.STATE_DOCKED}
        )

    def send_task(self, task_id: str, robot_id: str) -> None:
        task = self.core.tasks[task_id]
        goal = ExecuteTask.Goal()
        goal.task_id = task.task_id
        goal.run_id = task.run_id
        goal.type = task.task_type
        goal.priority = task.priority
        goal.waypoints = [
            self.to_pose_stamped(waypoint)
            for waypoint in task.waypoints
        ]
        goal.target_ref = task.target_ref

        self.core.assign_task(task_id, robot_id)
        self.get_logger().info(
            f"{robot_id}에 작업 전송: {task_id} ({task.target_ref})"
        )

        send_future = self.execute_task_clients[robot_id].send_goal_async(
            goal,
            feedback_callback=lambda feedback, rid=robot_id, tid=task_id: (
                self.on_task_feedback(rid, tid, feedback)
            ),
        )
        send_future.add_done_callback(
            lambda future, rid=robot_id, tid=task_id: (
                self.on_goal_response(rid, tid, future)
            )
        )

    def on_goal_response(self, robot_id: str, task_id: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if task_id in self.canceling_task_ids:
                self.canceling_task_ids.discard(task_id)
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="SEND_FAILED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                f"작업 전송 실패: {exc}",
            )
            return

        if not goal_handle.accepted:
            if task_id in self.canceling_task_ids:
                self.canceling_task_ids.discard(task_id)
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="GOAL_REJECTED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                "로봇이 작업을 거절했습니다.",
            )
            return

        self.active_goals[task_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, rid=robot_id, tid=task_id: (
                self.on_task_result(rid, tid, result)
            )
        )

        task = self.core.tasks[task_id]
        if (
            self.core.run_state == RunState.ESTOPPED
            or task.state == TaskState.CANCELED
        ):
            goal_handle.cancel_goal_async()
            return

        self.core.start_task(task_id)
        self.get_logger().info(f"{robot_id}: 작업 승인됨: {task_id}")

    def on_task_feedback(
        self,
        robot_id: str,
        task_id: str,
        feedback_message,
    ) -> None:
        task = self.core.tasks.get(task_id)
        if task is None or task.state not in {
            TaskState.ASSIGNED,
            TaskState.RUNNING,
        }:
            return

        feedback = feedback_message.feedback
        self.core.update_task_progress(task_id, feedback.progress)
        self.get_logger().info(
            f"{robot_id}: {task_id} 진행률 "
            f"{feedback.progress * 100:.0f}% "
            f"(waypoint={feedback.waypoint_index})"
        )

    def on_task_result(self, robot_id: str, task_id: str, future) -> None:
        self.active_goals.pop(task_id, None)
        task = self.core.tasks.get(task_id)
        if task is None:
            return

        if task_id in self.canceling_task_ids:
            self.canceling_task_ids.discard(task_id)
            self.get_logger().warning(
                f"{robot_id}: 비상정지로 작업 취소됨: {task_id}"
            )
            return

        try:
            result = future.result().result
        except Exception as exc:
            if task.state == TaskState.CANCELED:
                return
            self.core.finish_task(
                task_id,
                ok=False,
                reason="RESULT_FAILED",
            )
            self.publish_task_failure(
                robot_id,
                task_id,
                f"작업 결과 수신 실패: {exc}",
            )
            return

        self.core.finish_task(
            task_id,
            ok=result.ok,
            reason=result.reason,
        )

        if result.ok:
            self.get_logger().info(f"{robot_id}: 작업 완료: {task_id}")
            if task.task_type == ExecuteTask.Goal.REINSPECTION:
                self.publish_alert(
                    level=Alert.LEVEL_INFO,
                    code="REINSPECTION_DONE",
                    robot_id=robot_id,
                    ref=task.candidate_id,
                    message="네트워크 후보 재측정 작업을 완료했습니다.",
                )
        else:
            message = f"작업 실패: {result.reason or '사유 없음'}"
            self.publish_task_failure(robot_id, task_id, message)

        self.complete_run_if_ready()

    def complete_run_if_ready(self) -> None:
        if (
            self.core.run_state == RunState.RUNNING
            and self.core.all_tasks_terminal()
        ):
            run_id = self.core.run_id
            self.core.complete_run()
            self.publish_alert(
                level=Alert.LEVEL_INFO,
                code="RUN_COMPLETED",
                robot_id="",
                ref=str(run_id),
                message=f"run_id={run_id}의 모든 작업이 종료되었습니다.",
            )
            self.get_logger().info(f"run_id={run_id} 순찰 종료")

    def on_candidate(self, msg: Candidate) -> None:
        if msg.source != Candidate.SOURCE_NETWORK:
            return
        if msg.run_id != self.core.run_id:
            return
        if self.core.run_state == RunState.IDLE:
            return
        if msg.candidate_id in self.seen_candidate_ids:
            return

        self.seen_candidate_ids.add(msg.candidate_id)
        self.core.reopen_run()
        task_id = f"run-{msg.run_id}-reinspect-{msg.candidate_id}"
        self.core.create_task(
            task_id=task_id,
            run_id=msg.run_id,
            task_type=ExecuteTask.Goal.REINSPECTION,
            priority=10,
            target_ref=msg.ref,
            preferred_robot_id=self.survey_robot_id,
            allowed_robot_ids=(self.survey_robot_id,),
            waypoints=(Waypoint(msg.x, msg.y, msg.yaw),),
            candidate_id=msg.candidate_id,
        )
        self.publish_alert(
            level=Alert.LEVEL_WARN,
            code="CANDIDATE",
            robot_id=msg.robot_id,
            ref=msg.candidate_id,
            message=(
                f"네트워크 후보 {msg.candidate_id}: "
                f"{self.survey_robot_id} 재측정 작업 생성"
            ),
        )
        self.get_logger().info(
            f"후보 재측정 생성: {task_id} -> {self.survey_robot_id}"
        )

    def on_inspection_result(self, msg: InspectionResult) -> None:
        if msg.run_id != self.core.run_id:
            return

        result_key = (
            msg.robot_id,
            msg.run_id,
            msg.task_id,
            msg.facility_id,
            msg.verdict,
            msg.attempts,
            msg.stamp.sec,
            msg.stamp.nanosec,
        )
        if result_key in self.seen_inspection_results:
            return
        self.seen_inspection_results.add(result_key)

        if msg.verdict == InspectionResult.NORMAL:
            self.publish_alert(
                level=Alert.LEVEL_INFO,
                code="INSPECTION_NORMAL",
                robot_id=msg.robot_id,
                ref=msg.facility_id,
                message=f"{msg.facility_id}: 정상",
            )
            return

        if msg.verdict == InspectionResult.UNCONFIRMED:
            self.publish_alert(
                level=Alert.LEVEL_WARN,
                code="E6",
                robot_id=msg.robot_id,
                ref=msg.facility_id,
                message=(
                    msg.detail
                    or f"{msg.facility_id}: {msg.attempts}회 관측 후 미확인"
                ),
            )
            return

        self.publish_alert(
            level=Alert.LEVEL_ERROR,
            code="CONFIRMED",
            robot_id=msg.robot_id,
            ref=msg.facility_id,
            message=msg.detail or f"{msg.facility_id}: 이상 판정",
        )

    def on_estop(self, request, response):
        del request

        if self.core.run_state == RunState.ESTOPPED:
            response.success = True
            response.message = "이미 비상정지 상태입니다."
            return response
        if self.core.run_state != RunState.RUNNING:
            response.success = False
            response.message = "진행 중인 순찰이 없습니다."
            return response

        self.core.estop()
        self.canceling_task_ids.update(
            task.task_id
            for task in self.core.tasks.values()
            if task.state == TaskState.CANCELED
        )
        for goal_handle in self.active_goals.values():
            goal_handle.cancel_goal_async()

        self.publish_alert(
            level=Alert.LEVEL_ERROR,
            code="E7",
            robot_id="",
            message="비상정지: 실행 중인 로봇 작업 취소를 요청했습니다.",
        )
        response.success = True
        response.message = "비상정지 요청 완료"
        return response

    def on_resume(self, request, response):
        del request

        if self.core.run_state != RunState.ESTOPPED:
            response.success = False
            response.message = "비상정지 상태가 아닙니다."
            return response

        self.core.resume()
        response.success = True
        response.message = "취소된 작업을 대기열에 넣고 순찰을 재개합니다."
        self.publish_alert(
            level=Alert.LEVEL_INFO,
            code="RESUMED",
            robot_id="",
            message=response.message,
        )
        return response

    def on_reassign(self, request, response):
        from_robot_id = request.from_robot_id
        to_robot_id = request.to_robot_id

        if not to_robot_id:
            to_robot_id = next(
                (
                    robot_id
                    for robot_id in self.core.robots
                    if robot_id != from_robot_id
                ),
                "",
            )

        if (
            from_robot_id not in self.core.robots
            or to_robot_id not in self.core.robots
            or from_robot_id == to_robot_id
        ):
            response.ok = False
            response.message = "from/to 로봇 ID를 확인하세요."
            response.moved_tasks = 0
            return response

        moved_tasks = self.core.reassign_pending_tasks(
            from_robot_id,
            to_robot_id,
        )
        response.moved_tasks = moved_tasks
        response.ok = moved_tasks > 0

        if moved_tasks:
            response.message = (
                f"{from_robot_id} -> {to_robot_id}: "
                f"대기 작업 {moved_tasks}건 재배정"
            )
        else:
            response.message = (
                "재배정 가능한 대기 작업이 없습니다. "
                "현재 시나리오는 RSSI와 설비 로봇 역할이 고정되어 있습니다."
            )
        return response

    def publish_task_failure(
        self,
        robot_id: str,
        task_id: str,
        message: str,
    ) -> None:
        task = self.core.tasks[task_id]
        blocked = task.state == TaskState.BLOCKED
        self.publish_alert(
            level=Alert.LEVEL_WARN if blocked else Alert.LEVEL_ERROR,
            code="E1" if blocked else "TASK_FAILED",
            robot_id=robot_id,
            ref=task_id,
            message=message,
        )
        self.get_logger().error(f"{robot_id}: {task_id}: {message}")

    def publish_alert(
        self,
        *,
        level: int,
        code: str,
        robot_id: str,
        message: str,
        ref: str = "",
    ) -> None:
        alert = Alert()
        alert.stamp = self.get_clock().now().to_msg()
        alert.level = level
        alert.code = code
        alert.robot_id = robot_id
        alert.ref = ref
        alert.message = message
        alert.delayed = False
        self.alert_publisher.publish(alert)

    def to_pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

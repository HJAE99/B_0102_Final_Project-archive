import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from scv_msgs.action import ExecuteTask
from scv_msgs.msg import Alert, RobotStatus
from .manager_core import ManagerCore, RobotStatusSnapshot

STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

class InspectionManager(Node):
    def __init__(self):
        super().__init__("inspection_manager")

        self.declare_parameter('robot_ids', ['robot4', 'robot3'])
        self.declare_parameter('offline_timeout_sec', 30.0)
        self.declare_parameter('send_test_task', False)

        robot_ids = list(self.get_parameter('robot_ids').value)
        offline_timeout_sec = float(self.get_parameter('offline_timeout_sec').value)

        self.core = ManagerCore(robot_ids=robot_ids, offline_timeout_sec=offline_timeout_sec)

        self.alert_publisher = self.create_publisher(Alert, '/inspection/alerts', 10)

        self.execute_task_clients = {
            robot_id: ActionClient(self, ExecuteTask, f'/{robot_id}/execute_task') for robot_id in robot_ids
        }

        self.status_subscriptions = []

        for robot_id in robot_ids:
            topic_name = f"/{robot_id}/status"

            subscription = self.create_subscription(
                RobotStatus,
                topic_name,
                lambda msg, robot_id=robot_id: (self.on_robot_status(robot_id, msg)),
                STATUS_QOS,
            )

            self.status_subscriptions.append(subscription)

            self.get_logger().info(f"RobotStatus 구독: {topic_name}")

        self.watching_timer = self.create_timer(1.0, self.check_robot_timeouts)

        if self.get_parameter('send_test_task').value:
            self.test_task_timer = self.create_timer(2.0, self.send_test_task_once)

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

        self.core.update_robot(robot_id=robot_id, now=time.monotonic(), status=status)

    def check_robot_timeouts(self) -> None:
        newly_offline = self.core.check_offline(now=time.monotonic())

        for robot_id in newly_offline:
            self.get_logger().error(f'{robot_id}: RobotStatus 수신 중단')
            self.publish_alert(
                level=Alert.LEVEL_ERROR, 
                code='E3', 
                robot_id=robot_id,
                message="30초 동안 RobotStatus가 수신되지 않았습니다.",
            )

    def send_test_task_once(self) -> None:
        self.test_task_timer.cancel()

        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = "map"

        # mock 서버 시험용 좌표
        waypoint.pose.position.x = 0.0
        waypoint.pose.position.y = 0.0
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.w = 1.0

        self.send_task(
            robot_id="robot4",
            task_id="test-survey-001",
            run_id=1,
            task_type=ExecuteTask.Goal.SURVEY_SEGMENT,
            priority=1,
            waypoints=[waypoint],
            target_ref="Z1",
        )

    def send_task(
        self,
        *,
        robot_id: str,
        task_id: str,
        run_id: int,
        task_type: int,
        priority: int,
        waypoints: list[PoseStamped],
        target_ref: str,
    ) -> None:
        client = self.execute_task_clients[robot_id]

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"{robot_id}: ExecuteTask 액션 서버를 찾을 수 없습니다."
            )
            return

        goal = ExecuteTask.Goal()
        goal.task_id = task_id
        goal.run_id = run_id
        goal.type = task_type
        goal.priority = priority
        goal.waypoints = waypoints
        goal.target_ref = target_ref

        self.get_logger().info(f"{robot_id}에 작업 전송: {task_id}")

        send_future = client.send_goal_async(
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
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning(f"{robot_id}: 작업 거절됨: {task_id}")
            return

        self.get_logger().info(f"{robot_id}: 작업 승인됨: {task_id}")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda future, rid=robot_id, tid=task_id: (
                self.on_task_result(rid, tid, future)
            )
        )

    def on_task_feedback(self, robot_id: str, task_id: str, feedback_message) -> None:
        feedback = feedback_message.feedback

        self.get_logger().info(
            f"{robot_id}: {task_id} 진행률 "
            f"{feedback.progress * 100:.0f}% "
            f"(waypoint={feedback.waypoint_index})"
        )

    def on_task_result(self, robot_id: str, task_id: str, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result

        if result.ok:
            self.get_logger().info(f"{robot_id}: 작업 완료: {task_id}")
        else:
            self.get_logger().error(
                f"{robot_id}: 작업 실패: {task_id}, "
                f"reason={result.reason}"
            )

    def publish_alert(self, *, level: int, code: str, robot_id: str, message: str) -> None:
        alert = Alert()
        alert.stamp = self.get_clock().now().to_msg()
        alert.level = level
        alert.code = code
        alert.robot_id = robot_id
        alert.ref = ""
        alert.message = message
        alert.delayed = False

        self.alert_publisher.publish(alert)

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
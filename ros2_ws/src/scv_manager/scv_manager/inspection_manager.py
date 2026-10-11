import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

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

        robot_ids = list(self.get_parameter('robot_ids').value)
        offline_timeout_sec = float(self.get_parameter('offline_timeout_sec').value)

        self.core = ManagerCore(robot_ids=robot_ids, offline_timeout_sec=offline_timeout_sec)

        self.alert_publisher = self.create_publisher(Alert, '/inspection/alerts', 10)

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
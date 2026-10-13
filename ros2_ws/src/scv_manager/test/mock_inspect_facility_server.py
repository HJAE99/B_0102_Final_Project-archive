"""묶음 시험용 가짜 비전: {ns}/inspect_facility 를 받아 2초 뒤 NORMAL 을 돌려주고 {ns}/inspect/result 를 낸다.

    python3 mock_inspect_facility_server.py --ros-args -r __ns:=/robot4

자세 보정은 요청하지 않는다 (로봇이 벽 가까이에서 더 움직이지 않게).
"""

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from scv_msgs.action import InspectFacility
from scv_msgs.msg import InspectionResult


class MockInspectFacilityServer(Node):
    def __init__(self):
        super().__init__("mock_inspect_facility_server")
        self.result_publisher = self.create_publisher(
            InspectionResult,
            "inspect/result",
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.action_server = ActionServer(
            self,
            InspectFacility,
            "inspect_facility",
            self.execute_callback,
        )

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(f"설비 점검 요청: {request.facility_id}")
        goal_handle.publish_feedback(InspectFacility.Feedback(attempt=1))
        time.sleep(2.0)

        message = InspectionResult()
        message.stamp = self.get_clock().now().to_msg()
        message.robot_id = self.get_namespace().strip("/")
        message.run_id = request.run_id
        message.task_id = request.task_id
        message.facility_id = request.facility_id
        message.kind = request.kind
        message.verdict = InspectionResult.NORMAL
        message.detail = "mock"
        message.attempts = 1
        self.result_publisher.publish(message)

        goal_handle.succeed()
        return InspectFacility.Result(
            verdict=InspectionResult.NORMAL,
            detail="mock",
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockInspectFacilityServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

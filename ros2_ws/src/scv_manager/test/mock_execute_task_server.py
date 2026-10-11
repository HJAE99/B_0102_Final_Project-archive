import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from scv_msgs.action import ExecuteTask


class MockExecuteTaskServer(Node):
    def __init__(self):
        super().__init__("mock_execute_task_server")

        self.action_server = ActionServer(
            self,
            ExecuteTask,
            "/robot4/execute_task",
            self.execute_callback,
        )

        self.get_logger().info(
            "mock 서버 시작: /robot4/execute_task"
        )

    def execute_callback(self, goal_handle):
        request = goal_handle.request

        self.get_logger().info(
            f"작업 수신: {request.task_id}"
        )

        waypoint_count = max(
            len(request.waypoints),
            1,
        )

        for index in range(waypoint_count):
            feedback = ExecuteTask.Feedback()
            feedback.waypoint_index = index
            feedback.progress = (
                float(index + 1) / waypoint_count
            )

            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        goal_handle.succeed()

        result = ExecuteTask.Result()
        result.ok = True
        result.reason = ""

        return result


def main(args=None):
    rclpy.init(args=args)

    node = MockExecuteTaskServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
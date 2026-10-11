import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scv_msgs.action import ExecuteTask


class MockExecuteTaskServer(Node):
    def __init__(self):
        super().__init__("mock_execute_task_server")
        self.callback_group = ReentrantCallbackGroup()
        self.action_servers = [
            ActionServer(
                self,
                ExecuteTask,
                f"/{robot_id}/execute_task",
                self.execute_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group,
            )
            for robot_id in ("robot4", "robot3")
        ]
        self.get_logger().info(
            "mock 서버 시작: /robot4/execute_task, /robot3/execute_task"
        )

    @staticmethod
    def cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(f"작업 수신: {request.task_id}")
        waypoint_count = max(len(request.waypoints), 1)

        for index in range(waypoint_count):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteTask.Result()
                result.ok = False
                result.reason = "ESTOP"
                return result

            feedback = ExecuteTask.Feedback()
            feedback.waypoint_index = index
            feedback.progress = float(index + 1) / waypoint_count
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        goal_handle.succeed()
        result = ExecuteTask.Result()
        result.ok = True
        result.reason = ""
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockExecuteTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

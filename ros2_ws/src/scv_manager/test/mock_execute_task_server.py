import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from scv_msgs.action import ExecuteTask
from scv_msgs.msg import RobotStatus


class MockExecuteTaskServer(Node):
    def __init__(self):
        super().__init__("mock_execute_task_server")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("step_delay_sec", 0.2)
        self.step_delay_sec = float(
            self.get_parameter("step_delay_sec").value
        )
        self.declare_parameter("status_duration_sec", 0.0)
        self.status_duration_sec = float(
            self.get_parameter("status_duration_sec").value
        )
        self.started_at = time.monotonic()
        self.robot_ids = ("robot4", "robot3")
        self.status_publishers = {
            robot_id: self.create_publisher(
                RobotStatus,
                f"/{robot_id}/status",
                10,
            )
            for robot_id in self.robot_ids
        }
        self.robot_status = {
            robot_id: {
                "state": RobotStatus.STATE_DOCKED,
                "task_id": "",
                "run_id": 0,
                "task_type": 0,
                "progress": 0.0,
            }
            for robot_id in self.robot_ids
        }
        self.action_servers = [
            ActionServer(
                self,
                ExecuteTask,
                f"/{robot_id}/execute_task",
                lambda goal_handle, rid=robot_id: self.execute_callback(
                    rid,
                    goal_handle,
                ),
                cancel_callback=self.cancel_callback,
                callback_group=self.callback_group,
            )
            for robot_id in self.robot_ids
        ]
        self.status_timer = self.create_timer(1.0, self.publish_statuses)
        self.get_logger().info(
            "mock 서버 시작: /robot4/execute_task, /robot3/execute_task"
        )

    @staticmethod
    def cancel_callback(goal_handle):
        del goal_handle
        return CancelResponse.ACCEPT

    def publish_statuses(self):
        if (
            self.status_duration_sec > 0.0
            and time.monotonic() - self.started_at
            >= self.status_duration_sec
        ):
            return

        for robot_id, values in self.robot_status.items():
            message = RobotStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.robot_id = robot_id
            message.state = values["state"]
            message.battery_percent = 100.0
            message.accepting_tasks = True
            message.task_id = values["task_id"]
            message.run_id = values["run_id"]
            message.task_type = values["task_type"]
            message.progress = values["progress"]
            message.pose_ok = True
            message.warning = ""
            self.status_publishers[robot_id].publish(message)

    def execute_callback(self, robot_id, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f"{robot_id} 작업 수신: {request.task_id} type={request.type}"
        )
        values = self.robot_status[robot_id]
        values["state"] = RobotStatus.STATE_EXECUTING
        values["task_id"] = request.task_id
        values["run_id"] = request.run_id
        values["task_type"] = request.type
        values["progress"] = 0.0
        waypoint_count = max(len(request.waypoints), 1)

        for index in range(waypoint_count):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteTask.Result()
                result.ok = False
                result.reason = "ESTOP"
                values["state"] = RobotStatus.STATE_IDLE
                values["task_id"] = ""
                values["progress"] = 0.0
                return result

            feedback = ExecuteTask.Feedback()
            feedback.waypoint_index = index
            feedback.progress = float(index + 1) / waypoint_count
            goal_handle.publish_feedback(feedback)
            values["progress"] = feedback.progress
            time.sleep(self.step_delay_sec)

        goal_handle.succeed()
        values["state"] = (
            RobotStatus.STATE_DOCKED
            if request.type == ExecuteTask.Goal.DOCK
            else RobotStatus.STATE_IDLE
        )
        values["task_id"] = ""
        values["progress"] = 1.0
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

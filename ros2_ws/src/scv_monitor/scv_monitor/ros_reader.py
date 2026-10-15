"""Own ROS context and subscriptions only; never creates motion clients/publishers."""
import threading
from functools import partial

from .telemetry import STATES


class RosReader:
    def __init__(self, cache, robot_ids, media_dir):
        self.cache, self.robot_ids, self.media_dir = cache, robot_ids, media_dir
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name='scv-web-ros', daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stopping.set()
        self.thread.join(timeout=3)

    def run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import Path
            from sensor_msgs.msg import BatteryState
            from scv_msgs.msg import Alert, InspectionResult, RobotStatus
            context = Context()
            rclpy.init(context=context)
            node = Node('web_telemetry_reader', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            def status(rid, m):
                if m.robot_id != rid:
                    return
                self.cache.on_status(rid, dict(x=float(m.x), y=float(m.y), yaw=float(m.yaw),
                    pose_ok=bool(m.pose_ok), state=int(m.state), nav_state=STATES[m.state] if m.state < len(STATES) else '알 수 없음',
                    is_docked=m.state == 4, estop=m.state == 5, run_id=m.run_id, task_id=m.task_id,
                    task_type=m.task_type, progress=float(m.progress), warning=m.warning,
                    source_stamp={'sec': m.stamp.sec, 'nanosec': m.stamp.nanosec}))
            def alert(m):
                self.cache.on_alert(dict(level=('info', 'warn', 'error')[min(m.level, 2)], code=m.code,
                    robot_id=m.robot_id, ref=m.ref, message=m.message, delayed=m.delayed))
            def inspection(m):
                # Live receipt is distinct from durable DB inspections. Images stay in a bounded memory cache.
                import base64
                photo = None
                if any(fmt in m.image.format.lower() for fmt in ('jpeg', 'jpg')) and 0 < len(m.image.data) <= 256 * 1024:
                    photo = 'data:image/jpeg;base64,' + base64.b64encode(bytes(m.image.data)).decode('ascii')
                self.cache.on_inspection(dict(run_id=m.run_id, robot_id=m.robot_id, facility_id=m.facility_id,
                    verdict=m.verdict, detail=m.detail, attempts=m.attempts, photo=photo, source='ROS live / not persisted'))
            for rid in self.robot_ids:
                node.create_subscription(RobotStatus, f'/{rid}/status', partial(status, rid), 10)
                node.create_subscription(BatteryState, f'/{rid}/battery_state',
                    lambda m, r=rid: self.cache.on_battery(r, float(m.percentage)), qos_profile_sensor_data)
                node.create_subscription(PoseWithCovarianceStamped, f'/{rid}/amcl_pose',
                    lambda m, r=rid: self.cache.on_pose(r), qos_profile_sensor_data)
                node.create_subscription(Path, f'/{rid}/plan',
                    lambda m, r=rid: self.cache.on_path(r, m.header.frame_id,
                        [dict(x=float(p.pose.position.x), y=float(p.pose.position.y)) for p in m.poses]), 10)
                node.create_subscription(InspectionResult, f'/{rid}/inspect/result', inspection, 10)
            node.create_subscription(Alert, '/inspection/alerts', alert, 10)
            self.cache.ready = True
            while not self.stopping.is_set():
                executor.spin_once(timeout_sec=0.2)
        except Exception as exc:
            self.cache.error = f'{type(exc).__name__}: {exc}'
        finally:
            self.cache.ready = False
            if executor:
                executor.shutdown(timeout_sec=1)
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()

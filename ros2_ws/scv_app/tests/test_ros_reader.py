"""Optional local ROS message test. Must use a dedicated localhost-only DDS domain."""
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/scv_monitor'))


def test_readonly_ros_subscriptions_receive_status_battery_and_alert(tmp_path):
    if os.environ.get('SCV_TEST_LOCAL_ROS') != '1':
        pytest.skip('Enable only with a dedicated local test DDS domain')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID')=='217'
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from scv_msgs.msg import Alert,RobotStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import BatteryState
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from scv_monitor.telemetry import Telemetry
    from scv_monitor.ros_reader import RosReader
    cache=Telemetry(); reader=RosReader(cache,['preview_robot'],tmp_path)
    context=Context();rclpy.init(context=context);node=Node('scv_preview_only_publisher',context=context)
    status=node.create_publisher(RobotStatus,'/preview_robot/status',10)
    battery=node.create_publisher(BatteryState,'/preview_robot/battery_state',qos_profile_sensor_data)
    pose=node.create_publisher(PoseWithCovarianceStamped,'/preview_robot/amcl_pose',qos_profile_sensor_data)
    alerts=node.create_publisher(Alert,'/inspection/alerts',10)
    plan=node.create_publisher(Path,'/preview_robot/plan',10)
    reader.start()
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            pose.publish(PoseWithCovarianceStamped())
            battery.publish(BatteryState(percentage=.73))
            status.publish(RobotStatus(robot_id='preview_robot',pose_ok=True,x=0.,y=0.,yaw=0.,run_id=7,task_id='fake',state=1))
            path=Path();path.header.frame_id='map';path.poses=[PoseStamped(),PoseStamped()]
            path.poses[-1].pose.position.x=1.
            plan.publish(path)
            alerts.publish(Alert(level=1,code='PREVIEW',message='local fake message'))
            time.sleep(.1)
            snap=cache.snapshot(['preview_robot']);s=snap['robots'][0]['state']
            if s and s['pose_ok'] and s['battery_pct'] and snap['alerts'] and snap['paths']:
                break
        assert not cache.error,cache.error
        assert s and s['pose_ok'] and s['x']==0 and s['run_id']==7
        assert s['battery_pct']==pytest.approx(73,abs=.01)
        assert snap['alerts'][-1]['code']=='PREVIEW'
        assert snap['paths'][-1]['source']=='Nav2 global plan'
    finally:
        reader.stop();node.destroy_node();context.shutdown()

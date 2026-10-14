"""시험 전용 ROS domain에서 가짜 TF·상태·RSSI만 사용한다. motion 노드는 실행하지 않는다."""

import csv
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('scv_msgs.msg')

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import TransformBroadcaster

from scv_msgs.msg import NetRaw, RobotStatus
from scv_msgs.srv import SyncSamples
from scv_survey.heatmap import Sample, SurveyGrid
from scv_survey.manual_tagger import AMCL_QOS, SAMPLE_QOS
from scv_survey.rssi_scanner import RAW_QOS
from scv_survey.survey_buffer import SurveyBuffer


@pytest.fixture
def ros_buffer(tmp_path):
    path = tmp_path / 'buffer.csv'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/buffer_test_robot',
                    '-p', f'buffer_csv_path:={path}'], domain_id=219)
    node = SurveyBuffer()
    peer = rclpy.create_node('buffer_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    try:
        yield node, peer, executor, path
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()


def test_ros_status_tf_restart_and_sync(ros_buffer):
    node, peer, executor, path = ros_buffer
    received = []
    from scv_msgs.msg import NetSample
    peer.create_subscription(NetSample, 'survey/sample', received.append, SAMPLE_QOS)
    raw_pub = peer.create_publisher(NetRaw, 'survey/raw', RAW_QOS)
    status_pub = peer.create_publisher(RobotStatus, 'status', 10)
    amcl_pub = peer.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', AMCL_QOS)
    tf = TransformBroadcaster(peer)
    client = peer.create_client(SyncSamples, 'survey/sync')

    def support():
        transform = TransformStamped()
        transform.header.stamp = peer.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 2.0
        transform.transform.rotation.w = 1.0
        tf.sendTransform(transform)
        amcl = PoseWithCovarianceStamped()
        amcl.pose.covariance[0] = amcl.pose.covariance[7] = 0.01
        amcl_pub.publish(amcl)

    def wait(predicate, timeout=6):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            support()
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), 'ROS 시험 메시지 수신 시간 초과'

    def set_run(run):
        status_pub.publish(RobotStatus(robot_id='buffer_test_robot', run_id=run))
        wait(lambda: node.run_id == run)

    def measure(detected=True):
        raw = NetRaw(robot_id='buffer_test_robot', detected=detected,
                     rssi_dbm=-55 if detected else 0)
        raw.stamp = peer.get_clock().now().to_msg()
        raw_pub.publish(raw)

    wait(lambda: raw_pub.get_subscription_count() == 1 and client.service_is_ready()
         and node.cov_xy is not None)
    set_run(7)
    measure()
    wait(lambda: len(received) == 1)
    assert received[0].run_id == 7 and received[0].seq == 0
    assert received[0].pose_ok and received[0].x == 1.0 and received[0].y == 2.0
    assert len(list(csv.DictReader(path.open()))) == 1  # 수신 시점에는 이미 디스크에 저장됨

    set_run(8)
    measure(detected=False)
    wait(lambda: len(received) == 2)
    assert received[1].seq == 1 and not received[1].detected

    future = client.call_async(SyncSamples.Request(run_id=7, from_seq=0))
    wait(lambda: future.done() and len(received) == 3)
    assert future.result().ok and future.result().count == 1 and future.result().last_seq == 0
    assert received[2].resent and received[2].run_id == 7 and received[2].seq == 0
    assert received[2].stamp == received[0].stamp
    assert len(list(csv.DictReader(path.open()))) == 2

    executor.remove_node(node)
    node.destroy_node()
    node = SurveyBuffer()
    executor.add_node(node)
    try:
        assert node.seq == 2
        set_run(8)
        # 재시작 시 raw_pub의 transient_local 캐시가 다시 와도 저장하지 않는다.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            support()
            executor.spin_once(timeout_sec=0.01)
        assert node.seq == 2 and len(node.buffer.samples) == 2
        measure()
        wait(lambda: len(received) == 4)
        assert received[3].seq == 2 and received[3].run_id == 8
        future = client.call_async(SyncSamples.Request(run_id=8, from_seq=1))
        wait(lambda: future.done() and len(received) == 6)
        assert future.result().count == 2 and future.result().last_seq == 2
        assert [m.seq for m in received[4:]] == [1, 2]
        assert all(m.resent for m in received[4:])
        # 수신 단절 구간의 원본은 놓치고 재전송만 받았어도 히트맵에는 한 번씩만 반영된다.
        grid = SurveyGrid()
        for m in [received[3], received[4], received[5], received[4]]:
            grid.add(Sample(m.robot_id, m.seq, m.x, m.y, m.pose_ok, m.detected, m.rssi_dbm))
        assert {s.seq for s in grid.samples} == {1, 2}
        future = client.call_async(SyncSamples.Request(run_id=99, from_seq=0))
        wait(future.done)
        assert not future.result().ok and future.result().count == 0
    finally:
        executor.remove_node(node)
        node.destroy_node()


def test_no_publication_when_storage_fails(ros_buffer, monkeypatch):
    node, _peer, _executor, _path = ros_buffer
    sent = []
    monkeypatch.setattr(node, 'pub', SimpleNamespace(publish=sent.append))
    raw = NetRaw(robot_id='buffer_test_robot', detected=True, rssi_dbm=-55)
    raw.stamp.sec = 100
    node.sample_runs[id(raw)] = 7
    node.pending_keys.add((raw.robot_id, raw.stamp.sec, raw.stamp.nanosec))

    def fail(_sample):
        raise OSError('disk full')

    monkeypatch.setattr(node.buffer, 'append', fail)
    with pytest.raises(OSError):
        node.publish(raw, True)
    assert sent == [] and node.seq == 0


def test_pending_tf_keeps_run_at_raw_receipt(ros_buffer, monkeypatch):
    node, _peer, _executor, _path = ros_buffer
    sent = []
    monkeypatch.setattr(node, 'pub', SimpleNamespace(publish=sent.append))
    monkeypatch.setattr(node.tf_buffer, 'can_transform', lambda *_args: False)
    node.on_status(RobotStatus(robot_id='buffer_test_robot', run_id=7))
    raw = NetRaw(robot_id='buffer_test_robot', detected=True, rssi_dbm=-55)
    raw.stamp = node.get_clock().now().to_msg()
    node.on_raw(raw)
    node.on_raw(raw)    # TF를 기다리는 같은 원본도 한 번만 보관
    assert len(node.pending) == 1 and sent == []
    node.on_status(RobotStatus(robot_id='buffer_test_robot', run_id=8))
    monkeypatch.setattr(node.tf_buffer, 'can_transform', lambda *_args: True)
    monkeypatch.setattr(node, 'tf_pose_ok', lambda *_args: False)
    node.flush_pending()
    assert len(sent) == 1 and sent[0].run_id == 7 and not sent[0].pose_ok
    assert node.buffer.samples[0].run_id == 7


def test_launch_uses_buffer_only_in_tf_mode():
    import importlib.util
    from pathlib import Path

    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node

    path = Path(__file__).parents[1] / 'launch' / 'laptop_test.launch.py'
    spec = importlib.util.spec_from_file_location('survey_test_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(rviz='false', keepout_costmap='false')
    for mode, expected in [('manual', 'manual_tagger'), ('tf', 'survey_buffer')]:
        context.launch_configurations['pose_source'] = mode
        names = [action.node_executable if isinstance(action.node_executable, str)
                 else perform_substitutions(context, action.node_executable)
                 for action in module._nodes(context) if isinstance(action, Node)]
        assert [name for name in names if name in ('manual_tagger', 'survey_buffer')] == [expected]

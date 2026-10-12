"""로봇 PC 한 대: Nav2·AMCL(keepout_layer), mission_executor, survey_buffer, detection_alert.

두 로봇이 같은 구성이다 (10/12). 로봇 A(robot4)는 섹터 1, 로봇 B(robot3)는 섹터 2 — zones.yaml 의 robot:.
기준 지도는 scv_bringup/maps/testbed_v1.yaml.

    ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    nodes = [
        # 노드가 준비되면 여기에 추가한다:
        # 주행: Nav2 bringup(map, params), Node(package='scv_nav', executable='mission_executor')
        # 비전 (두 로봇): Node(package='scv_detection', executable='detection_alert')
        # RSSI (두 로봇): Node(package='scv_survey', executable='survey_buffer')
    ]
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='로봇 이름공간 (robot4, robot3)'),
        # 한 launch 에 노드가 많을 때 SUPER_CLIENT 면 Nav2 lifecycle 응답을 놓친 적이 있다 (미니 프로젝트, 4번 로봇)
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        GroupAction([PushRosNamespace(LaunchConfiguration('robot_ns')), *nodes]),
    ])

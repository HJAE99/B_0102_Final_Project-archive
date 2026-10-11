"""로봇 PC 한 대: Nav2·AMCL, mission_executor + 역할에 따라 survey_buffer 또는 detection_alert.

로봇 A(robot4)는 RSSI 측정만 → survey_buffer. 로봇 B(robot3)는 물리 인프라 검출·판정만 → detection_alert, keepout 필터.
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
        # 비전 (robot3 에만): Node(package='scv_detection', executable='detection_alert')
        # RSSI (robot4 에만): Node(package='scv_survey', executable='survey_buffer')
    ]
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='로봇 이름공간 (robot4, robot3)'),
        # 한 launch 에 노드가 많을 때 SUPER_CLIENT 면 Nav2 lifecycle 응답을 놓친 적이 있다 (미니 프로젝트, 4번 로봇)
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        GroupAction([PushRosNamespace(LaunchConfiguration('robot_ns')), *nodes]),
    ])

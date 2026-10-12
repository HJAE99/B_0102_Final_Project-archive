"""사용자가 고른 로봇 이름공간에서 화면 없는 설비 검출 진단 노드를 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """필수 모델·로봇 인자를 받고, 상대 영상 주소 앞에 로봇 이름공간을 붙인다."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', description='설비 점검 로봇 이름공간'),
        DeclareLaunchArgument('weights', default_value='', description='YOLO 사용 시 필수인 pt 모델 경로'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('rack_enabled', default_value='false'),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('device', default_value='auto'),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_ns')),
            Node(package='scv_detection', executable='scv_detection_manager', output='screen',
                 parameters=[{'weights': LaunchConfiguration('weights'),
                              'enable_yolo': ParameterValue(LaunchConfiguration('enable_yolo'), value_type=bool),
                              'rack_enabled': ParameterValue(LaunchConfiguration('rack_enabled'), value_type=bool),
                              'device': LaunchConfiguration('device'),
                              'show': ParameterValue(LaunchConfiguration('show'), value_type=bool)}]),
        ]),
    ])

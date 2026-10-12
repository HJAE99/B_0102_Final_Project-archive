"""로봇 PC 한 대 (로봇 위에 얹음): Nav2·AMCL(keepout_layer), mission_executor, rssi_scanner, survey_buffer, detection_alert.

두 로봇이 같은 구성이다 (10/12). 로봇 A(robot4)는 섹터 1, 로봇 B(robot3)는 섹터 2 — zones.yaml 의 robot:.
기준 지도는 scv_bringup/maps/testbed_v1.yaml.

    ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """로봇 이름공간을 붙이고, 요청한 경우에만 설비 검출 진단 노드를 함께 실행한다."""
    nodes = [
        # 액션 서버 완성 전의 진단 실행기. 기본으로 꺼 두며 이름공간은 아래 GroupAction을 따른다.
        Node(package='scv_detection', executable='scv_detection_manager', output='screen',
             condition=IfCondition(LaunchConfiguration('start_detection_diagnostics')),
             parameters=[{'weights': LaunchConfiguration('detection_weights'),
                          'enable_yolo': ParameterValue(LaunchConfiguration('detection_enable_yolo'), value_type=bool),
                          'rack_enabled': ParameterValue(LaunchConfiguration('detection_rack_enabled'), value_type=bool),
                          'device': LaunchConfiguration('detection_device'),
                          'show': ParameterValue(LaunchConfiguration('detection_show'), value_type=bool)}]),
        # 노드가 준비되면 여기에 추가한다:
        # 주행: Nav2 bringup(map, params), Node(package='scv_nav', executable='mission_executor')
        # 비전 (두 로봇): Node(package='scv_detection', executable='detection_alert')
        # RSSI (두 로봇, 10/12 저녁부터 로봇 PC 에서): Node(package='scv_survey', executable='rssi_scanner',
        #     parameters=[{'robot_id': LaunchConfiguration('robot_ns'), 'iface': '<로봇 PC 무선랜 이름>'}]),
        # RSSI (두 로봇): Node(package='scv_survey', executable='survey_buffer')
    ]
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='로봇 이름공간 (robot4, robot3)'),
        DeclareLaunchArgument('start_detection_diagnostics', default_value='false', description='상시 구독 진단용 실행 여부; 액션 서버가 아님'),
        DeclareLaunchArgument('detection_weights', default_value='', description='진단용 모델 경로; 실행 시 필수'),
        DeclareLaunchArgument('detection_enable_yolo', default_value='true'),
        DeclareLaunchArgument('detection_rack_enabled', default_value='false'),
        DeclareLaunchArgument('detection_show', default_value='false'),
        DeclareLaunchArgument('detection_device', default_value='auto'),
        # 한 launch 에 노드가 많을 때 SUPER_CLIENT 면 Nav2 lifecycle 응답을 놓친 적이 있다 (미니 프로젝트, 4번 로봇)
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        GroupAction([PushRosNamespace(LaunchConfiguration('robot_ns')), *nodes]),
    ])

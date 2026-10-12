"""로봇 PC 한 대 (로봇 위에 얹음): localization · Nav2(keepout_layer) · mission_executor · RSSI 측정 · 정밀 측정 서버 · (가짜) 비전.

두 로봇이 같은 구성이다 (10/12). 로봇 A(robot4)는 섹터 1, 로봇 B(robot3)는 섹터 2 — zones.yaml 의 robot:.
**저장소 뿌리에서** 실행한다 (주행 설정·지도·스크립트를 ros2_ws/src/scv_nav 에서 읽는다). 관제 PC 를 먼저 띄운다.

    ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot4 iface:=<무선랜> ssid:=<핫스팟>

띄우는 순서는 주행 팀이 확인한 대로 localization → mission_executor → Nav2 다 (10/12 주행 일지).
Dock 에 있는 채로 Nav2 를 먼저 띄우면 AMCL 초기 pose 가 늦어 global costmap 이 map TF 를 못 받고 bringup 이 중단된다.
mission_executor 는 켜지면 스스로 undock 하고 초기 pose 를 준 뒤 Nav2 가 켜지기를 기다린다 (로봇이 움직인다).
비전 노드(detection_alert)의 점검 요청 받기가 아직 없어 fake_vision 으로 대신한다.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    ns = arg('robot_ns')
    nav = os.path.join(arg('repo_root'), 'ros2_ws', 'src', 'scv_nav')
    manager_test = os.path.join(arg('repo_root'), 'ros2_ws', 'src', 'scv_manager', 'test')
    if not os.path.isfile(os.path.join(nav, 'config', arg('nav2_params'))):
        raise RuntimeError(f'{nav}/config/{arg("nav2_params")} 가 없습니다 — 저장소 뿌리에서 실행하거나 repo_root:= 를 주세요')

    def include(package, launch_file, **args):
        path = os.path.join(FindPackageShare(package).perform(context), 'launch', launch_file)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items())

    def script(path):
        return ExecuteProcess(cmd=['python3', path, '--ros-args', '-r', f'__ns:=/{ns}'], output='screen')

    nodes = [
        # 주행 팀 방식 그대로 (nav2.yaml 머리 주석). params 안의 $(var namespace) 는 이 launch 가 바꿔 준다
        include('turtlebot4_navigation', 'localization.launch.py', namespace=f'/{ns}',
                map=os.path.join(nav, 'maps', 'testbed_v1.yaml'),
                params=os.path.join(nav, 'config', 'localization.yaml')),
        TimerAction(period=float(arg('executor_delay')),
                    actions=[script(os.path.join(nav, 'scripts', 'mission_executor.py'))]),
        TimerAction(period=float(arg('nav2_delay')), actions=[
            include('turtlebot4_navigation', 'nav2.launch.py', namespace=f'/{ns}',
                    params_file=os.path.join(nav, 'config', arg('nav2_params')))]),
        # RSSI: rssi_scanner · manual_tagger(pose 는 TF) · 미리보기 히트맵
        include('scv_survey', 'laptop_test.launch.py', pose_source='tf', robot_ns=ns, rviz='false',
                iface=arg('iface'), ssid=arg('ssid')),
        Node(package='scv_survey', executable='precise_server', namespace=ns, output='screen'),
    ]
    if arg('fake_vision') == 'true':
        nodes.append(script(os.path.join(manager_test, 'mock_inspect_facility_server.py')))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='robot4', description='로봇 이름공간 (robot4, robot3)'),
        DeclareLaunchArgument('iface', description='RSSI 를 재는 로봇 PC 의 무선랜 이름 (iw dev)'),
        DeclareLaunchArgument('ssid', default_value='', description='측정 대상 핫스팟 SSID (비우면 자동 탐색)'),
        DeclareLaunchArgument('repo_root', default_value=os.getcwd(), description='저장소 뿌리 (기본: 지금 폴더)'),
        DeclareLaunchArgument('nav2_params', default_value='nav2.yaml', description='nav2.yaml(MPPI) · nav2_rpp.yaml(RPP)'),
        DeclareLaunchArgument('executor_delay', default_value='10.0',
                              description='localization 뒤 mission_executor(undock · 초기 pose)를 띄우는 시간 [s]'),
        DeclareLaunchArgument('nav2_delay', default_value='30.0',
                              description='Nav2 를 띄우는 시간 [s]. mission_executor 가 undock·초기 pose 를 끝낸 뒤여야 한다'),
        DeclareLaunchArgument('fake_vision', default_value='true', description='가짜 inspect_facility 서버 (2초 뒤 NORMAL)'),
        # 한 launch 에 노드가 많을 때 SUPER_CLIENT 면 Nav2 lifecycle 응답을 놓친 적이 있다 (미니 프로젝트, 4번 로봇)
        SetEnvironmentVariable('ROS_SUPER_CLIENT', 'False'),
        OpaqueFunction(function=_nodes),
    ])

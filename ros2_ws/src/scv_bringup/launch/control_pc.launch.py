"""관제 PC: cell_map_writer(칸 맵 저장) + inspection_manager(작업 배정 · 재측정 · 회피 마스크 · 설비 점검).

로봇 PC 의 Nav2 보다 먼저 띄운다 (inspection_manager 가 빈 회피 마스크를 먼저 내야 주행의 costmap 이 준비된다).

    ros2 launch scv_bringup control_pc.launch.py                                   # 두 대, 칸 맵은 파일
    ros2 launch scv_bringup control_pc.launch.py robots:=robot4                    # robot4 한 대
    ros2 launch scv_bringup control_pc.launch.py backend:=db ap_ssid:=<핫스팟>     # 칸 맵을 DB 에 (SCV_DB_DSN 필요)
    … backend:=db ap_ssid:=<핫스팟> close_stale_run:=true    # 관제를 끄면서 남은 running 회차를 닫고 새로 시작

아직 없는 것: ros_bridge · 관제 웹 (관제 B), network_map_engine 은 로봇 PC 쪽 미리보기로 뜬다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ZONE_OF = {'robot4': 'Z1', 'robot3': 'Z2'}      # zones.yaml 의 robot: 과 같게


def _nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    robots = list(ZONE_OF) if arg('robots') == 'both' else [arg('robots')]
    store = {'ap_ssid': arg('ap_ssid')}
    return [
        Node(package='scv_survey', executable='cell_map_writer', output='screen',
             parameters=[{'sample_topics': [f'/{r}/survey/sample' for r in robots],
                          'robot_ids': robots, 'zone_ids': [ZONE_OF[r] for r in robots],
                          'backend': arg('backend'), **store}]),
        Node(package='scv_manager', executable='inspection_manager', output='screen',
             parameters=[{'robot_ids': robots, 'zone_ids': [ZONE_OF[r] for r in robots],
                          'cellmap_backend': arg('backend'),
                          'close_stale_run': arg('close_stale_run') == 'true', **store}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='both', description='both · robot4 · robot3'),
        DeclareLaunchArgument('backend', default_value='file', description='칸 맵 저장: file(~/scv_cellmap) · db'),
        DeclareLaunchArgument('ap_ssid', default_value='', description='측정 대상 핫스팟 SSID (db 면 필수, rssi_scanner 와 같은 값)'),
        DeclareLaunchArgument('close_stale_run', default_value='false',
                              description='db: 닫히지 않은 running 회차가 남아 있으면 aborted 로 닫고 시작 (true · false)'),
        OpaqueFunction(function=_nodes),
    ])

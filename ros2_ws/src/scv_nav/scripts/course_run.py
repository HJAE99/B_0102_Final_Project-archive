#!/usr/bin/env python3
# PLAN 3-2·3-3: 같은 코스를 한 번 주행 (컨트롤러마다 5회 반복 실행)
#   python3 course_run.py --ros-args -r __ns:=/robot3
# 순서: 시작점으로 이동 → Enter 대기(이때 track_check 를 켬) → COURSE 웨이포인트를 차례로 통과
# COURSE 좌표는 3-1 통로 실측 후 testbed_v1 지도 기준으로 채움
import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

# (x m, y m, yaw 도). 첫 점이 시작점. 아래는 자리 표시용 값
COURSE = [
    (0.5, 0.5, 90.0),
    (0.5, 3.0, 90.0),
    (0.5, 5.5, 0.0),
    (2.5, 5.5, -90.0),
]


def main():
    rclpy.init()
    nav = TurtleBot4Navigator()
    nav.waitUntilNav2Active()
    poses = [nav.getPoseStamped([x, y], yaw) for x, y, yaw in COURSE]

    nav.info('시작점으로 이동')
    nav.startToPose(poses[0])
    input('시작점 도착. track_check 를 켠 뒤 Enter')

    nav.startThroughPoses(poses[1:])
    result = nav.getResult()
    nav.info(f'코스 결과: {result}')
    if result != TaskResult.SUCCEEDED:
        nav.error('코스 실패 — 이 회차는 비교에서 제외')

    rclpy.shutdown()


if __name__ == '__main__':
    main()

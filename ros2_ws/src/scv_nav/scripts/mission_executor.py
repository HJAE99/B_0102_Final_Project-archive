#!/usr/bin/env python3
# PLAN 5-1·5-2·5-3·3-4·4-2: IF-11 {ns}/execute_task 액션 서버 + IF-12 {ns}/status 1 Hz. 작업 한 건씩 받아 Nav2 로 실행
#   python3 mission_executor.py --ros-args -r __ns:=/robot3
#   (undock 상태로 다른 자리에서 다시 켤 때만 -p x:=… -p y:=… -p yaw:=… 로 그 자리 좌표를 줌)
# 시작: Dock 에 있으면 undock → 초기 pose 설정 → Nav2 활성화 대기 (dock_init.py 를 합침)
# HOME 의 x, y(m), yaw(도)는 undock 직후 로봇의 지도(testbed_v1) 좌표. Dock 에서는 lidar 가 꺼져 있어 이 위치를 기준으로 함
# 이 위치는 Dock 바로 앞이므로 배터리 부족·DOCK 작업 때 여기로 돌아와 dock 한다
# 속도 상한 0.2 m/s 는 Nav2 controller·velocity_smoother 설정에서 건다 (이 파일에서 속도를 내지 않음)
# 작업 종류
#   SURVEY_SEGMENT: waypoints 를 navigate_through_poses 로 통과
#   FACILITY_CHECK: 마지막 waypoint 에 정밀 도착 → {ns}/inspect_facility (자세 보정 피드백은 회전·전진으로 따름)
#   REINSPECTION:   마지막 waypoint 에 정밀 도착 → {ns}/survey/precise 30초
#   DOCK:           waypoints(없으면 undock 위치)로 가서 dock
# 막힘(E1): 5초 동안 5 cm 도 못 가면 BLOCKED, 10초면 그 waypoint 를 건너뜀
# 배터리(E2): 30 % 이하 새 작업 거절(DOCK 제외), 15 % 이하 작업 취소 후 dock
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_simple_commander.robot_navigator import TaskResult
from rcl_interfaces.srv import SetParameters
from scv_msgs.action import ExecuteTask, InspectFacility, PreciseMeasure
from scv_msgs.msg import InspectionResult, RobotStatus
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

T = ExecuteTask.Goal
S = RobotStatus
NORMAL_TOL = (0.10, 0.17)    # nav2.yaml general_goal_checker 와 같은 값 (m, rad)
PRECISE_TOL = (0.05, 0.087)  # PLAN 0-1 설비 점검 지점 5 cm, 5°
# shortcut: facilities.yaml id 앞 세 글자로 종류를 정함, 관제가 kind 를 보내 주면 그 값으로 바꿈
KIND = {'EXT': InspectionResult.KIND_EXTINGUISHER, 'SHT': InspectionResult.KIND_SHUTTER,
        'EXL': InspectionResult.KIND_EXIT_LIGHT}
# undock 직후 위치 (x m, y m, yaw 도). 2-1 pose_check 3회 평균 (docs/driving/data/2026-10-11_pose_check.csv)
HOME = {'/robot3': (2.855, 4.656, 176.7), '/robot4': (2.863, 0.948, 175.8)}


class MissionExecutor(Node):
    def __init__(self, nav, home):
        super().__init__('mission_executor')
        self.nav = nav   # nav 는 자기 스스로 spin 하므로 이 노드의 executor 에 넣지 않는다
        self.home = home
        self.busy = False
        self.state = S.STATE_IDLE
        self.task = None   # 수행 중인 ExecuteTask.Goal
        self.progress = 0.0
        self.warning = ''
        self.battery = 100.0
        self.pose = None   # (x, y, yaw rad, 위치 표준편차 m)
        self.robot_id = self.get_namespace().strip('/')
        self.pose_std_max = self.declare_parameter('pose_std_max', 0.15).value   # PLAN 2-3: 정상 9–11 cm, 틀림 52 cm (docs/driving/data/2026-10-11_robot4_pose_ok.csv)

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(RobotStatus, 'status', 10)
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.param_cli = self.create_client(SetParameters, 'controller_server/set_parameters', callback_group=cb)
        self.inspect_cli = ActionClient(self, InspectFacility, 'inspect_facility', callback_group=cb)
        self.precise_cli = ActionClient(self, PreciseMeasure, 'survey/precise', callback_group=cb)
        ActionServer(self, ExecuteTask, 'execute_task', self.execute,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=cb)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('mission_executor ready')

    # ---------- 상태 (IF-12) ----------
    def on_battery(self, msg):
        self.battery = msg.percentage * 100.0   # Create 3 는 0–1

    def on_pose(self, msg):
        p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw, math.sqrt(max(c[0], c[7], 0.0)))

    def publish_status(self):
        m = RobotStatus(robot_id=self.robot_id, state=self.state, battery_percent=self.battery,
                        accepting_tasks=self.battery > 30.0, progress=self.progress, warning=self.warning)
        m.stamp = self.get_clock().now().to_msg()
        if self.task:
            m.task_id, m.run_id, m.task_type = self.task.task_id, self.task.run_id, self.task.type
        if self.pose:
            m.x, m.y, m.yaw = self.pose[:3]
            m.pose_ok = self.pose[3] <= self.pose_std_max
        self.status_pub.publish(m)

    # ---------- 작업 (IF-11) ----------
    def on_goal(self, goal):
        # 한 번에 한 작업만 (관제가 이전 작업을 취소한 뒤 새로 보냄). 배터리 30 % 이하는 DOCK 만 받음
        if self.busy or (self.battery <= 30.0 and goal.type != T.DOCK):
            return GoalResponse.REJECT
        self.busy = True   # 수락과 실행 사이에 다른 goal 이 들어오지 않게 여기서 잡음
        return GoalResponse.ACCEPT

    def execute(self, gh):
        g = gh.request
        self.task, self.progress, self.warning = g, 0.0, ''
        if self.state == S.STATE_DOCKED and g.type != T.DOCK:
            self.nav.undock()
            self.nav.setInitialPose(self.home)   # 시작할 때와 같은 이유 (Dock 에서는 lidar 꺼짐)
        self.state = S.STATE_EXECUTING
        self.get_logger().info(f'task {g.task_id} type={g.type} waypoints={len(g.waypoints)}')
        try:
            if g.type == T.SURVEY_SEGMENT:
                ok, reason = self.follow(gh, g.waypoints)
            elif g.type == T.FACILITY_CHECK:
                ok, reason = self.follow(gh, g.waypoints, precise=True)
                if ok:
                    ok, reason = self.inspect(gh, g)
            elif g.type == T.REINSPECTION:
                ok, reason = self.follow(gh, g.waypoints, precise=True)
                if ok:
                    ok, reason = self.measure(gh)
            elif g.type == T.DOCK:
                ok, reason = self.go_dock(gh, g.waypoints)
            else:
                ok, reason = False, 'UNKNOWN_TYPE'
            if reason == 'LOW_BATTERY':
                self.go_dock(gh, [])
        finally:
            self.task, self.busy = None, False
            if self.state != S.STATE_DOCKED:
                self.state = S.STATE_IDLE
        self.get_logger().info(f'task {g.task_id} ok={ok} reason={reason}')
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return ExecuteTask.Result(ok=ok, reason=reason)

    def follow(self, gh, waypoints, precise=False):
        if not waypoints:
            return False, 'NO_WAYPOINTS'
        if precise:
            self.set_tolerance(*PRECISE_TOL)
        try:
            return self.follow_loop(gh, list(waypoints))
        finally:
            if precise:
                self.set_tolerance(*NORMAL_TOL)

    def follow_loop(self, gh, waypoints):
        n, done, skipped = len(waypoints), 0, 0
        while waypoints:
            self.nav.goThroughPoses(waypoints)
            last_xy, last_t = None, time.time()
            while not self.nav.isTaskComplete():
                if gh.is_cancel_requested or self.battery <= 15.0:
                    self.nav.cancelTask()
                    self.stop()
                    return False, 'CANCELED' if gh.is_cancel_requested else 'LOW_BATTERY'
                fb = self.nav.getFeedback()
                if fb:
                    i = done + len(waypoints) - fb.number_of_poses_remaining
                    self.progress = i / n
                    gh.publish_feedback(ExecuteTask.Feedback(waypoint_index=i, progress=self.progress))
                    xy = (fb.current_pose.pose.position.x, fb.current_pose.pose.position.y)
                    if last_xy is None or math.dist(xy, last_xy) > 0.05:
                        last_xy, last_t = xy, time.time()
                        if self.state == S.STATE_BLOCKED:
                            self.state, self.warning = S.STATE_EXECUTING, ''
                    elif time.time() - last_t > 10.0:
                        break   # 막힘 10초: 지금 향하는 waypoint 를 건너뜀
                    elif time.time() - last_t > 5.0 and self.state != S.STATE_BLOCKED:
                        self.state, self.warning = S.STATE_BLOCKED, 'BLOCKED'   # 5초: Nav2 가 재계획하는 중
                time.sleep(0.2)
            else:
                res = self.nav.getResult()
                if res == TaskResult.SUCCEEDED:
                    return (True, '') if not skipped else (False, f'BLOCKED skipped={skipped}')
                return False, str(res)
            # 막힘으로 빠져나옴
            self.nav.cancelTask()
            k = len(waypoints) - fb.number_of_poses_remaining   # 지금 향하던 waypoint 위치
            self.get_logger().warn(f'BLOCKED 10 s → waypoint {done + k} 건너뜀')
            done, skipped = done + k + 1, skipped + 1
            waypoints = waypoints[k + 1:]
            self.state = S.STATE_EXECUTING
        return False, f'BLOCKED skipped={skipped}'   # 마지막 waypoint 가 막힘

    def inspect(self, gh, g):
        adjust = []   # 피드백 콜백은 executor 스레드라서 nav 호출은 이 스레드에서 한다
        kind = KIND.get(g.target_ref[:3], 0)
        goal = InspectFacility.Goal(task_id=g.task_id, facility_id=g.target_ref, kind=kind)
        res, reason = self.call_action(self.inspect_cli, goal, gh, lambda m: adjust.append(m.feedback),
                                       lambda: self.do_adjust(adjust))
        if res is None:
            return False, reason
        return res.verdict != InspectionResult.UNCONFIRMED, f'verdict={res.verdict} {res.detail}'

    def do_adjust(self, adjust):
        while adjust:
            f = adjust.pop(0)
            self.get_logger().info(f'자세 보정 {f.attempt}: yaw={f.adjust_yaw:.2f} rad forward={f.adjust_forward_m:.2f} m')
            if f.adjust_yaw:
                self.nav.spin(f.adjust_yaw)
                self.wait_nav()
            if f.adjust_forward_m > 0:   # 후진은 하지 않음 (뒤에 센서 없음)
                self.nav.driveOnHeading(f.adjust_forward_m)
                self.wait_nav()

    def measure(self, gh):
        res, reason = self.call_action(self.precise_cli, PreciseMeasure.Goal(duration_s=30), gh)
        if res is None:
            return False, reason
        return True, f'median={res.rssi_median:.1f} iqr={res.rssi_iqr:.1f} n={res.n_samples}'

    def call_action(self, client, goal, gh, on_fb=None, each=None):
        # 다른 노드의 액션을 보내고 결과를 기다림. 작업이 취소되면 그 액션도 취소
        if not client.wait_for_server(timeout_sec=5.0):
            return None, 'NO_SERVER'
        f = client.send_goal_async(goal, feedback_callback=on_fb)
        while not f.done():
            time.sleep(0.05)
        h = f.result()
        if not h.accepted:
            return None, 'REJECTED'
        r = h.get_result_async()
        while not r.done():
            if gh.is_cancel_requested:
                h.cancel_goal_async()
                return None, 'CANCELED'
            if each:
                each()
            time.sleep(0.1)
        return r.result().result, ''

    def go_dock(self, gh, waypoints):
        if self.nav.getDockedStatus():
            self.state = S.STATE_DOCKED
            return True, ''
        ok, reason = self.follow(gh, waypoints) if waypoints else self.go_home()
        if not ok:
            return ok, reason
        self.state = S.STATE_DOCKING
        self.nav.dock()
        self.state = S.STATE_DOCKED
        return True, ''

    def go_home(self):
        self.nav.goToPose(self.home)
        self.wait_nav()
        res = self.nav.getResult()
        return res == TaskResult.SUCCEEDED, '' if res == TaskResult.SUCCEEDED else str(res)

    def set_tolerance(self, xy, yaw):
        # SimpleGoalChecker 는 실행 중 파라미터 변경을 받는다 (ros2 param set 과 같음)
        req = SetParameters.Request(parameters=[
            Parameter('general_goal_checker.xy_goal_tolerance', value=xy).to_parameter_msg(),
            Parameter('general_goal_checker.yaw_goal_tolerance', value=yaw).to_parameter_msg()])
        if not self.param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('controller_server 파라미터 서비스 없음, 허용 오차 못 바꿈')
            return
        f = self.param_cli.call_async(req)
        while not f.done():
            time.sleep(0.05)
        self.get_logger().info(f'goal 허용 오차 xy={xy} yaw={yaw}')

    def wait_nav(self):
        while not self.nav.isTaskComplete():
            time.sleep(0.1)

    def stop(self):
        for _ in range(5):
            self.cmd_pub.publish(TwistStamped())
            time.sleep(0.05)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    nav = TurtleBot4Navigator()
    xyyaw = [nav.declare_parameter(k, math.nan).value for k in ('x', 'y', 'yaw')]
    if any(map(math.isnan, xyyaw)):
        if nav.get_namespace() not in HOME:
            raise SystemExit(f'HOME 에 {nav.get_namespace()} 없음 — -p x y yaw 를 주세요')
        xyyaw = HOME[nav.get_namespace()]
    home = nav.getPoseStamped(xyyaw[:2], xyyaw[2])
    if nav.getDockedStatus():
        nav.undock()
    # undock 상태로 시작해도 home 을 줌. 안 주면 waitUntilNav2Active 가 (0,0) 을 초기 pose 로 보냄
    nav.setInitialPose(home)
    nav.waitUntilNav2Active()
    node = MissionExecutor(nav, home)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    try:
        while rclpy.ok():
            # 놀고 있을 때 배터리 15 % 이하면 스스로 dock (작업 중이면 follow 가 처리)
            if not node.busy and node.battery <= 15.0 and node.state != S.STATE_DOCKED:
                node.busy = True
                node.go_dock(None, [])
                node.busy = False
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node.busy:
            nav.cancelTask()
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

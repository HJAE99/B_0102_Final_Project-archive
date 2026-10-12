#!/usr/bin/env python3
"""
SCV 가상 로봇 robot3 (3-2차) — TurtleBot4 robot3 + 제어 PC의 AMCL 대신 ROS 2 토픽을 내보내는 시험용 노드

  구현방안 3장의 「기존」 로봇 통신과, 실제로는 제어 PC에서 도는 표준 노드(AMCL · Nav2)의 이름을 흉내 냅니다.
  지금 client page가 쓰는 것만 구현합니다(필요할 때마다 누적).
    토픽 (이름공간 /robot3)
      battery_state                    sensor_msgs/BatteryState                1 Hz   배터리 (도킹 중이면 충전)
      dock_status                      irobot_create_msgs/DockStatus           1 Hz   도킹 상태 (패키지가 없으면 생략)
      odom                             nav_msgs/Odometry                       10 Hz  오도메트리 (시작점 = odom 원점)
      amcl_pose                        geometry_msgs/PoseWithCovarianceStamped 2 Hz   지도(map) 좌표 위치 — 실제로는 제어 PC AMCL
      ip                               std_msgs/String                         1 Hz   로봇 IP
      oakd/rgb/image_raw/compressed    sensor_msgs/CompressedImage             10 Hz  카메라 JPEG (합성 영상)
    구독 (3-2차)
      initialpose                      geometry_msgs/PoseWithCovarianceStamped         시작 위치 — client가 관제 PC의 도크 접근 자세로 보냄
    서비스 · 액션: 아직 없음 (client가 요청하는 차수에서 추가 — 3-3차 Nav2 · dock · undock)

  위치: 켜지면 도크에 서 있지만(충전 중) 지도 위 위치는 모름 → amcl_pose를 내보내지 않음.
        initialpose를 받으면 그 자리를 도크 · odom 원점으로 삼고 amcl_pose를 내보냄 (실제 AMCL 초기 위치 지정과 같음)
  가상 세계: virtual_world/world.yaml — 로봇이 측정 · 판정으로 알아내야 하는 실제 세계의 값
             (AP 신호 원점 · 세기 모델, 음영 구역, 설비 실제 상태). 3-4차 RSSI부터 사용, DB에는 넣지 않음
  3-2차까지는 이동하지 않음(이동은 3-3차)

  실행 (ROS 2 Jazzy 환경):
    source /opt/ros/jazzy/setup.bash
    python3 robot3_virtual.py                 # /robot3/...
    python3 robot3_virtual.py --camera-hz 5   # 카메라 주기 변경 (0이면 카메라 끔)
    python3 robot3_virtual.py --world 다른/world.yaml
"""
from __future__ import annotations

import argparse
import array
import io
import math
import socket
import time

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String

try:                                   # TurtleBot4(Create 3) 메시지 — 없으면 dock_status만 생략
    from irobot_create_msgs.msg import DockStatus
except ImportError:                    # pragma: no cover
    DockStatus = None

try:                                   # JPEG 인코딩 — OpenCV가 있으면 사용, 없으면 Pillow
    import cv2
except ImportError:                    # pragma: no cover
    cv2 = None
    from PIL import Image, ImageDraw

ROBOT_ID = "robot3"
DEFAULT_WORLD = Path(__file__).resolve().parent / "virtual_world" / "world.yaml"
CAM_W, CAM_H = 640, 360


def local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def load_world(path: Path) -> dict:
    """world.yaml — 실제 세계의 정답 (ap_models · holes · facilities). 없으면 빈 세계"""
    p = Path(path)
    if not p.exists():
        return {}
    return yaml.safe_load(p.read_text(encoding="utf-8")) or {}


class VirtualRobot(Node):
    def __init__(self, robot_id: str, camera_hz: float, world: dict):
        super().__init__(f"{robot_id}_virtual", namespace=f"/{robot_id}")
        self.robot_id = robot_id
        self.world = world
        self.t0 = time.monotonic()
        self.battery = 0.87                                # 0~1 (Create 3와 같은 비율)
        self.ip = local_ip()
        self.frame_no = 0
        # 도크에 서서 시작(도킹 상태). 지도 위 위치는 initialpose를 받기 전까지 모름
        self.dock: tuple[float, float, float] | None = None   # odom 원점 = initialpose로 받은 자세
        self.pose: list[float] | None = None                  # map 좌표 x, y, yaw
        self.docked = True

        self.pub_battery = self.create_publisher(BatteryState, "battery_state", 10)
        self.pub_dock = self.create_publisher(DockStatus, "dock_status", 10) if DockStatus else None
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 10)
        self.pub_ip = self.create_publisher(String, "ip", 10)
        self.pub_cam = (self.create_publisher(CompressedImage, "oakd/rgb/image_raw/compressed", qos_profile_sensor_data)
                        if camera_hz > 0 else None)

        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initialpose, 10)
        self.create_timer(1.0, self.tick_1hz)
        self.create_timer(0.1, self.tick_odom)
        self.create_timer(0.5, self.tick_amcl)
        if self.pub_cam:
            self.create_timer(1.0 / camera_hz, self.tick_camera)
        if not DockStatus:
            self.get_logger().warn("irobot_create_msgs가 없어 dock_status는 내보내지 않습니다")
        self.get_logger().info(f"가상 로봇 {robot_id} 시작 — 도킹 중, 위치는 /{robot_id}/initialpose를 기다림 · "
                               f"/{robot_id}/battery_state · dock_status · odom · amcl_pose · ip"
                               + (" · oakd/rgb/image_raw/compressed" if self.pub_cam else "")
                               + f" · 세계 정답 {', '.join(k for k in ('ap_models', 'holes', 'facilities') if world.get(k)) or '없음'}")

    # ------------------------------------------------------------------ initialpose — 시작 위치 지정
    def on_initialpose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = [float(p.x), float(p.y), yaw]
        if self.dock is None or self.docked:               # 도크에 있으면 이 자리가 도크 · odom 원점
            self.dock = (float(p.x), float(p.y), yaw)
        self.get_logger().info(f"initialpose 받음 — ({p.x:.2f}, {p.y:.2f}, {yaw:.3f}) · amcl_pose 발행 시작")

    # ------------------------------------------------------------------ 1 Hz
    def tick_1hz(self):
        now = self.get_clock().now().to_msg()
        if self.docked:                                    # 도킹 중이면 충전, 아니면 방전
            self.battery = min(1.0, self.battery + 0.0005)
        else:
            self.battery = max(0.05, self.battery - 0.0002)
        b = BatteryState()
        b.header.stamp = now
        b.header.frame_id = "base_link"
        b.percentage = float(self.battery)
        b.voltage = float(14.4 + 2.0 * self.battery)
        b.current = 1.5 if self.docked else -1.2
        b.temperature = 31.5
        b.charge = float(2.1 * self.battery)
        b.capacity = 2.1
        b.design_capacity = 2.1
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if self.docked and self.battery >= 1.0
                                 else BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.docked
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        b.present = True
        self.pub_battery.publish(b)

        if self.pub_dock:
            d = DockStatus()
            d.header.stamp = now
            d.dock_visible = self.docked
            d.is_docked = self.docked
            self.pub_dock.publish(d)

        s = String()
        s.data = self.ip
        self.pub_ip.publish(s)

    # ------------------------------------------------------------------ 10 Hz — odom (시작 자세 기준)
    def tick_odom(self):
        if self.pose is None:                              # 위치를 모름 — odom은 원점(0, 0)
            self.pub_odom.publish(self._odom_zero())
            return
        dx, dy = self.pose[0] - self.dock[0], self.pose[1] - self.dock[1]
        c, s_ = math.cos(-self.dock[2]), math.sin(-self.dock[2])
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = c * dx - s_ * dy
        o.pose.pose.position.y = s_ * dx + c * dy
        yaw = self.pose[2] - self.dock[2]
        o.pose.pose.orientation.z = math.sin(yaw / 2)
        o.pose.pose.orientation.w = math.cos(yaw / 2)
        self.pub_odom.publish(o)                           # 3-1차는 정지 — 속도 0

    def _odom_zero(self) -> Odometry:
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        o.pose.pose.orientation.w = 1.0
        return o

    # ------------------------------------------------------------------ 2 Hz — amcl_pose (map 좌표)
    def tick_amcl(self):
        if self.pose is None:                              # initialpose 전 — AMCL처럼 위치를 내보내지 않음
            return
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose.position.x = float(self.pose[0])
        m.pose.pose.position.y = float(self.pose[1])
        m.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        m.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.0025                           # 위치 분산 (σ 5 cm)
        cov[35] = 0.003                                    # yaw 분산
        m.pose.covariance = cov
        self.pub_amcl.publish(m)

    # ------------------------------------------------------------------ 카메라 — 합성 영상 JPEG
    def tick_camera(self):
        self.frame_no += 1
        jpeg = self.render_frame()
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "oakd_rgb_camera_optical_frame"
        m.format = "jpeg"
        m.data = array.array("B", jpeg)
        self.pub_cam.publish(m)

    def render_frame(self) -> bytes:
        t = time.monotonic() - self.t0
        label = f"{self.robot_id} virtual  frame {self.frame_no}  {time.strftime('%H:%M:%S')}"
        x = int((CAM_W - 120) * (0.5 + 0.5 * math.sin(t * 0.8)))
        if cv2 is not None:
            img = np.zeros((CAM_H, CAM_W, 3), np.uint8)
            img[:] = (60, 52, 40)
            cv2.rectangle(img, (x, 120), (x + 120, 240), (40, 170, 220), -1)
            cv2.line(img, (0, 300), (CAM_W, 300), (120, 120, 120), 2)
            cv2.putText(img, label, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes()
        img = Image.new("RGB", (CAM_W, CAM_H), (40, 52, 60))
        d = ImageDraw.Draw(img)
        d.rectangle([x, 120, x + 120, 240], fill=(220, 170, 40))
        d.line([0, 300, CAM_W, 300], fill=(120, 120, 120), width=2)
        d.text((16, 24), label, fill=(255, 255, 255))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description="SCV 가상 로봇")
    ap.add_argument("--robot-id", default=ROBOT_ID)
    ap.add_argument("--camera-hz", type=float, default=10.0)
    ap.add_argument("--world", default=str(DEFAULT_WORLD), help="world.yaml (실제 세계 정답)")
    a, _ros_args = ap.parse_known_args()       # --ros-args … 는 rclpy가 sys.argv에서 직접 읽음
    world = load_world(Path(a.world))
    rclpy.init()
    node = VirtualRobot(a.robot_id, a.camera_hz, world)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

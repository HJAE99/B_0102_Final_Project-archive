#!/usr/bin/env python3
"""
SCV client page — 제어 PC (3-3차)

  로봇과는 ROS 2, 관제 PC(server page)와는 웹(HTTP · WebSocket)으로 통신합니다.
  한 프로세스 안에서 Uvicorn(화면 · API)과 rclpy(별도 스레드)를 함께 돌립니다 (구현방안 10.4절).

  화면: /client/login → /client/robot (로봇 정보) · /client/camera (카메라)
  흐름
    1) 로그인 — 아이디·비밀번호를 관제 PC POST /api/client/login으로 확인 (DB에 등록된 계정)
    2) 로그인하면 ROS 토픽 목록을 확인해 /{robot_id}/… 토픽을 내보내는 노드가 있는지 봄
       없으면(비활성) ACTIVE_CHECK_S마다 다시 확인, 있으면(활성) 확인을 멈춤
    3) 활성이 되면 관제 PC WebSocket /ws/agent에 접속해 hello { robot_id, … }를 한 번 보냄
       → 관제 PC가 robots.is_active = true. 연결이 끊기면 관제 PC가 false로 바꾸고, 이쪽은 다시 접속해 hello
    4) 로봇 정보 탭: battery_state · dock_status · odom · ip 구독 값 표시
       활성인 동안 이 값이 STALE_S(5초) 넘게 하나도 안 오면 로봇이 꺼진 것으로 보고 비활성으로 바꿈
       → 관제 PC 연결을 닫아 관제 PC도 비활성, 2)의 토픽 목록 확인을 다시 시작
         (다시 활성이 되려면 토픽 목록의 발행자 + 꺼진 뒤 새로 받은 값이 함께 있어야 함 — DDS 목록에 남은 옛 발행자 제외)
    5) 카메라 탭: 카메라 토픽(CompressedImage)을 보는 동안만 구독해 MJPEG으로 화면에 전달
    6) 3-1차 — mission_executor가 1초마다 status(RobotStatus 필드)를 만들고, scv_agent가 관제 PC로 보냄
    7) 3-2차 — 관제 PC config(현재 지도 · 도크 접근 자세)를 받아, 로봇이 도킹 중이거나 위치를 모르면
               /{ns}/initialpose로 시작 위치를 지정 (가상 로봇은 이 값을 받아야 위치를 내보냄)
       위치는 {ns}/amcl_pose(지도 좌표), 배터리 · 도킹은 로봇 토픽
    8) 3-3차 — 관제 PC의 run_config · task_assign을 받아 task를 하나씩 실행 (로봇마다 ACTIVE task 하나)
       SURVEY_SEGMENT  undock → Nav2 navigate_through_poses로 경로점 주행, 그동안 {ns}/rssi 표본을 자기 섹터 칸만 모음
                       → 끝나면 칸 중앙값으로 네트워크 후보(CANDIDATE 이벤트)
       REINSPECTION    후보 지점으로 이동 → 정지 정밀측정(precise_duration_s) → CONFIRMED / CLEARED
       FACILITY_CHECK  관측 자세로 이동 → 설비 판정 1회(다시 판정하지 않음) + 사진. 정면 확보 실패가 max_pose_retry를 넘으면 미확인(E6)
       DOCK            도크 접근 자세로 이동 → {ns}/dock
       기록(표본 · 이벤트 · 정밀측정 · 판정 · 사진 · task 결과)은 아웃박스 폴더에 쌓았다가 순서대로 관제 PC REST로 보냄
       (관제 PC가 끊겨도 받은 task는 끝까지 수행하고, 다시 연결되면 이어서 보냄)
       카메라 탭(3-3차): 순찰 중에만 실시간 영상 · 지도(제어 PC의 지도 파일 robot_map_path) 위 이 로봇 위치 ·
                        순찰 시작 때 관제 PC가 보낸 섹터 칸 · 도크 · 설비 · 측정 AP(순찰이 끝날 때까지) · 이번 순찰의 후보 · 판정

  구성 (구현방안 A-2 결정 — 제어 PC 안 역할은 이 프로그램의 모듈, 서로는 함수 호출, 로봇과는 ROS)
    RosBridge        로봇과의 ROS 통신 (구독 · 활성 확인)
    MissionExecutor  task 실행 담당 — 상태(status) 만들기 · task 실행(Nav2 · undock · dock) · 사건 이벤트
    SurveyBuffer     RSSI + AMCL 위치 + seq · 자기 섹터 칸만 · 칸 중앙값으로 후보 · 정밀측정 (survey_buffer)
    DetectionAlert   설비 판정 1회 + 사진 (detection_alert) — 3차는 가상 판정(virtual/detect_*), 실제 YOLO는 이후 차수
    Outbox           관제 PC로 보낼 기록을 폴더에 쌓아 순서대로 REST 전송 (scv_agent의 아웃박스)
    ScvAgent         관제 PC와의 유일한 창구 — WebSocket hello · status · 명령 받기, REST 아웃박스 (ROS 노드 아님)

  실행 (제어 PC, ROS 2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 client_page.py --server http://<관제PC>:8000            # http://<제어PC>:8080 → /client/login
  옵션 · 환경 변수
    --robot-id      SCV_ROBOT_ID       기본 robot3
    --server        SCV_SERVER_URL     기본 http://127.0.0.1:8000
    --camera-topic  SCV_CAMERA_TOPIC   기본 /{robot_id}/oakd/rgb/image_raw/compressed (짝인 로봇의 카메라)
    --host / --port SCV_CLIENT_HOST / SCV_CLIENT_PORT  기본 0.0.0.0 / 8080
    --check-s       SCV_ACTIVE_CHECK_S 비활성일 때 토픽 목록 확인 주기(초), 기본 2
    --outbox        SCV_OUTBOX_DIR     아웃박스 폴더, 기본 ./outbox_{robot_id}
"""
from __future__ import annotations

import argparse
import asyncio
import io
import json
import math
import os
import queue
import secrets
import statistics
import threading
import time
import traceback
import urllib.error
import urllib.request
import uuid
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response, StreamingResponse
from PIL import Image
from fastapi.staticfiles import StaticFiles
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from starlette.concurrency import run_in_threadpool
from std_msgs.msg import String
from websockets.asyncio.client import connect as ws_connect

try:                                   # TurtleBot4(Create 3) 메시지 — 없으면 도킹 정보 · dock · undock 생략
    from irobot_create_msgs.msg import DockStatus
    from irobot_create_msgs.action import Dock, Undock
except ImportError:                    # pragma: no cover
    DockStatus = Dock = Undock = None
try:                                   # Nav2 — 없으면 주행 task는 실패(NO_NAV2)
    from nav2_msgs.action import NavigateThroughPoses
except ImportError:                    # pragma: no cover
    NavigateThroughPoses = None
try:
    from action_msgs.msg import GoalStatus
    GOAL_OK, GOAL_CANCELED = GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED
except ImportError:                    # pragma: no cover
    GOAL_OK, GOAL_CANCELED = 4, 5

# ---------------------------------------------------------------------------
# 설정 (환경 변수 → main()의 명령행 옵션이 덮어씀)
# ---------------------------------------------------------------------------
BASE = Path(__file__).resolve().parent
STATIC_DIR = BASE / "static"
CFG = {
    "robot_id": os.environ.get("SCV_ROBOT_ID", "robot3"),
    "server": os.environ.get("SCV_SERVER_URL", "http://127.0.0.1:8000").rstrip("/"),
    "camera_topic": os.environ.get("SCV_CAMERA_TOPIC", ""),      # 비우면 /{robot_id}/oakd/rgb/image_raw/compressed
    "host": os.environ.get("SCV_CLIENT_HOST", "0.0.0.0"),
    "port": int(os.environ.get("SCV_CLIENT_PORT", "8080")),
    "check_s": float(os.environ.get("SCV_ACTIVE_CHECK_S", "2")),
    "outbox": os.environ.get("SCV_OUTBOX_DIR", ""),             # 비우면 ./outbox_{robot_id}
}
AGENT_VERSION = "3-3"
SESSION_COOKIE = "scv_client_session"
SESSION_TTL_S = 8 * 3600
PAGES = {"login", "robot", "camera"}
STALE_S = 5.0                          # 이 시간 동안 값이 안 오면 화면에 「끊김」, 활성 로봇은 비활성으로

POWER_STATUS = {0: "알 수 없음", 1: "충전 중", 2: "방전 중", 3: "충전 안 함", 4: "완충"}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def guarded(fn):
    """ROS 콜백 보호 — 예외가 나면 spin 전체가 멈추므로, 내용을 남기고 계속 돌게 함."""
    def wrap(*args):
        try:
            return fn(*args)
        except Exception:
            print(f"[ros] {fn.__name__} 오류 — 계속 동작함\n{traceback.format_exc()}", flush=True)
    wrap.__name__ = fn.__name__
    return wrap


# ---------------------------------------------------------------------------
# ROS 쪽 — rclpy 노드를 별도 스레드에서 spin. 구독·해제는 모두 ROS 스레드의 타이머에서 처리
# ---------------------------------------------------------------------------
class RosBridge:
    def __init__(self, robot_id: str, camera_topic: str, check_s: float):
        self.robot_id = robot_id
        self.camera_topic = camera_topic
        self.check_s = check_s
        self.lock = threading.Lock()

        # 활성 확인
        self.checking = False          # 로그인 뒤 비활성인 동안 True
        self.check_now = False
        self.active = False
        self.active_at: float | None = None
        self.checked_at: float | None = None
        self.next_check = 0.0
        self.found_topics: list[str] = []
        self.off_at: float | None = None   # 값이 끊겨 비활성으로 바꾼 시각
        self.last_data = 0.0               # 로봇 정보 토픽 값을 마지막으로 받은 시각

        # 로봇 정보 구독
        self.want_info = False
        self.info_subs: list[Any] = []
        self.info: dict[str, dict[str, Any]] = {}

        # 카메라 — 보는 사람이 있을 때만 구독
        self.cam_viewers = 0
        self.cam_sub = None
        self.cam: dict[str, Any] = {"data": None, "ctype": "image/jpeg", "seq": 0, "t": None, "fps": 0.0,
                                    "format": "", "size": 0}

        try:                                           # Ctrl+C는 Uvicorn이 받도록 rclpy 신호 처리기는 끔
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self.node = Node("scv_client_page")
        self.node.create_timer(0.2, guarded(self._tick))
        # 로봇 시작 위치 지정 (3-2차) — 관제 PC가 보낸 도크 접근 자세를 AMCL 초기 위치로 (실제 구성의 rviz 2D Pose Estimate와 같은 토픽)
        self.pub_initialpose = self.node.create_publisher(PoseWithCovarianceStamped, f"/{robot_id}/initialpose", 10)
        self.initialpose: dict[str, Any] | None = None     # 마지막으로 보낸 값 {x, y, yaw, at, reason}
        # 3-3차 — 주행 · 도킹 액션 (실제는 제어 PC Nav2 · 로봇 Create 3, 시험 때는 가상 로봇)
        ns = f"/{robot_id}"
        self.nav = ActionClient(self.node, NavigateThroughPoses, f"{ns}/navigate_through_poses") if NavigateThroughPoses else None
        self.undock_ac = ActionClient(self.node, Undock, f"{ns}/undock") if Undock else None
        self.dock_ac = ActionClient(self.node, Dock, f"{ns}/dock") if Dock else None
        # 가상 판정 (시험 전용 — 실제 구성에서는 detection_alert가 카메라 영상으로 YOLO 판정)
        self.pub_detect = self.node.create_publisher(String, f"{ns}/virtual/detect_request", 10)
        self.node.create_subscription(String, f"{ns}/virtual/detect_result", guarded(self._on_detect), 10)
        self.detect_results: dict[str, dict[str, Any]] = {}
        self.on_rssi = None                            # survey_buffer가 넣는 콜백 (rssi JSON dict)
        self.first_seen: set[str] = set()     # 토픽마다 첫 값 받았을 때 한 번 로그
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name="ros-spin", daemon=True)
        self.thread.start()
        self.log(f"client page ROS 노드 시작 — 로봇 {robot_id}, 카메라 토픽 {camera_topic}")

    def log(self, text: str):
        self.node.get_logger().info(text)

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as e:                         # 종료 중 예외는 무시
            if rclpy.ok():
                print(f"[ros] spin 중단: {e}", flush=True)

    def shutdown(self):
        try:
            self.executor.shutdown()
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    # ---- 화면 쪽에서 부르는 함수 (스레드 안전: 표시만 바꾸고 실제 처리는 _tick)
    def on_login(self):
        with self.lock:
            self.want_info = True
            if not self.active:
                self.checking = True
                self.check_now = True

    def add_viewer(self):
        with self.lock:
            self.cam_viewers += 1

    def remove_viewer(self):
        with self.lock:
            self.cam_viewers = max(0, self.cam_viewers - 1)

    # ---- ROS 스레드 (0.2 s)
    def _tick(self):
        now = time.monotonic()
        went_off = False
        with self.lock:                                # 활성인데 로봇 정보 값이 STALE_S 넘게 없으면 → 비활성
            wall = time.time()
            if self.active and wall - max(self.last_data, self.active_at or 0) > STALE_S:
                self.active = False
                self.off_at = wall
                self.checking = True
                self.next_check = now + self.check_s
                went_off = True
        if went_off:
            self.log(f"{self.robot_id} 비활성 — 로봇 정보 값이 {STALE_S:g}초 넘게 오지 않음. 토픽 목록 확인을 다시 시작")
        with self.lock:
            do_check = self.checking and (self.check_now or now >= self.next_check)
            want_info, viewers = self.want_info, self.cam_viewers
        if do_check:
            self._check_active()
            with self.lock:
                self.check_now = False
                self.next_check = now + self.check_s
        if want_info and not self.info_subs:
            self._subscribe_info()
        if viewers > 0 and self.cam_sub is None:
            self.cam_sub = self.node.create_subscription(CompressedImage, self.camera_topic, guarded(self._on_camera),
                                                         qos_profile_sensor_data)
            self.log(f"카메라 구독 시작 {self.camera_topic}")
        elif viewers == 0 and self.cam_sub is not None:
            self.node.destroy_subscription(self.cam_sub)
            self.cam_sub = None
            self.log("카메라 구독 해제 (보는 화면 없음)")

    def _check_active(self):
        """토픽 목록에서 /{robot_id}/… 토픽 중 발행자가 있는 것이 하나라도 있으면 활성."""
        prefix = f"/{self.robot_id}/"
        listed = [name for name, _types in self.node.get_topic_names_and_types() if name.startswith(prefix)]
        found = sorted(name for name in listed if self.node.count_publishers(name) > 0)
        became = False
        with self.lock:
            self.checked_at = time.time()
            self.found_topics = found
            # 꺼졌다 다시 켜질 때는 꺼진 뒤 새 값도 받아야 함 (DDS 목록에 잠시 남는 옛 발행자로 다시 활성이 되지 않게)
            fresh = self.off_at is None or self.last_data > self.off_at
            if found and fresh and not self.active:
                self.active = True
                self.active_at = time.time()
                self.checking = False                  # 활성이면 확인을 멈춤
                became = True
        if became:
            self.log(f"{self.robot_id} 활성 — 발행 중인 토픽 {len(found)}개")
        else:
            self.log(f"토픽 목록 확인 — {prefix}… 토픽 {len(listed)}개, 발행자 있는 토픽 {len(found)}개"
                     + ("" if fresh or not found else " (꺼진 뒤 새 값이 아직 없어 비활성 유지)"))

    def _subscribe_info(self):
        ns = f"/{self.robot_id}"
        q = qos_profile_sensor_data                    # best effort — 로봇 쪽이 reliable이든 best effort든 받음
        self.info_subs.append(self.node.create_subscription(BatteryState, f"{ns}/battery_state", guarded(self._on_battery), q))
        self.info_subs.append(self.node.create_subscription(Odometry, f"{ns}/odom", guarded(self._on_odom), q))
        self.info_subs.append(self.node.create_subscription(PoseWithCovarianceStamped, f"{ns}/amcl_pose",
                                                            guarded(self._on_amcl), q))
        self.info_subs.append(self.node.create_subscription(String, f"{ns}/ip", guarded(self._on_ip), q))
        self.info_subs.append(self.node.create_subscription(String, f"{ns}/rssi", guarded(self._on_rssi), q))
        if DockStatus is not None:
            self.info_subs.append(self.node.create_subscription(DockStatus, f"{ns}/dock_status", guarded(self._on_dock), q))
        self.log("로봇 정보 토픽 구독 — battery_state · odom · amcl_pose · ip · rssi" + (" · dock_status" if DockStatus else ""))

    # ---- 구독 콜백 (ROS 스레드)
    def _put(self, key: str, topic: str, data: dict[str, Any]):
        with self.lock:
            prev = self.info.get(key)
            n = (prev["count"] + 1) if prev else 1
            self.info[key] = {"topic": topic, "data": data, "t": time.time(), "count": n}
            self.last_data = self.info[key]["t"]
            first = topic not in self.first_seen
            self.first_seen.add(topic)
        if first:
            self.log(f"첫 값 받음 {topic}")

    def _on_battery(self, m):
        self._put("battery", f"/{self.robot_id}/battery_state", {
            "percentage": round(float(m.percentage) * 100, 1) if m.percentage == m.percentage else None,
            "voltage": round(float(m.voltage), 2), "current": round(float(m.current), 2),
            "temperature": round(float(m.temperature), 1) if m.temperature == m.temperature else None,
            "status": POWER_STATUS.get(int(m.power_supply_status), str(m.power_supply_status)),
        })

    def set_initial_pose(self, pose: dict[str, float], reason: str):
        """{ns}/initialpose 발행 — 지도(map) 좌표, 위치 σ 5 cm · 방향 σ 약 3°"""
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose.position.x = float(pose["x"])
        m.pose.pose.position.y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.0025
        cov[35] = 0.003
        m.pose.covariance = cov
        self.pub_initialpose.publish(m)
        self.initialpose = {"x": float(pose["x"]), "y": float(pose["y"]), "yaw": yaw, "at": time.time(), "reason": reason}
        self.log(f"initialpose 발행 /{self.robot_id}/initialpose ({pose['x']}, {pose['y']}, {yaw:.3f}) — {reason}")

    def _on_dock(self, m):
        self._put("dock", f"/{self.robot_id}/dock_status", {"is_docked": bool(m.is_docked),
                                                             "dock_visible": bool(m.dock_visible)})

    def _on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._put("odom", f"/{self.robot_id}/odom", {
            "x": round(p.x, 3), "y": round(p.y, 3), "yaw_deg": round(math.degrees(yaw), 1),
            "linear": round(m.twist.twist.linear.x, 3), "angular": round(m.twist.twist.angular.z, 3),
        })

    def _on_amcl(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cov = list(m.pose.covariance)
        self._put("pose", f"/{self.robot_id}/amcl_pose", {
            "x": round(p.x, 3), "y": round(p.y, 3), "yaw": round(yaw, 4), "yaw_deg": round(math.degrees(yaw), 1),
            "frame": m.header.frame_id, "sigma_xy": round(math.sqrt(max(cov[0], cov[7])), 3) if len(cov) >= 8 else None,
        })

    def _on_ip(self, m):
        self._put("ip", f"/{self.robot_id}/ip", {"ip": m.data})

    def _on_rssi(self, m):
        d = json.loads(m.data)
        self._put("rssi", f"/{self.robot_id}/rssi", {"scans": d.get("scans", []), "stamp": d.get("stamp")})
        if self.on_rssi:
            self.on_rssi(d)

    def _on_detect(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.detect_results[str(d.get("request_id"))] = d

    # ---- 3-3차 — 액션 · 가상 판정 · 사진 (MissionExecutor 스레드에서 부름. 결과는 ROS 스레드가 채움)
    @staticmethod
    def _wait(future, stop=None, timeout: float = 1e9) -> bool:
        end = time.monotonic() + timeout
        while not future.done():
            if (stop and stop()) or time.monotonic() > end:
                return False
            time.sleep(0.05)
        return True

    def _pose_stamped(self, p: dict[str, Any]) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = float(p["x"])
        ps.pose.position.y = float(p["y"])
        yaw = float(p.get("yaw") or 0.0)
        ps.pose.orientation.z = math.sin(yaw / 2)
        ps.pose.orientation.w = math.cos(yaw / 2)
        return ps

    def _run_action(self, client, goal, stop, feedback=None, timeout: float = 1e9) -> str:
        """'ok' · 'failed' · 'stopped' · 'no_server'"""
        if client is None or not client.wait_for_server(timeout_sec=3.0):
            return "no_server"
        f = client.send_goal_async(goal, feedback_callback=feedback)
        if not self._wait(f, stop, 10.0):
            return "stopped" if stop and stop() else "failed"
        gh = f.result()
        if not gh.accepted:
            return "failed"
        rf = gh.get_result_async()
        if not self._wait(rf, stop, timeout):
            gh.cancel_goal_async()                     # 취소 요청 → 결과(취소됨)를 잠시 기다림
            self._wait(rf, None, 5.0)
            return "stopped"
        st = rf.result().status
        return "ok" if st == GOAL_OK else ("stopped" if st == GOAL_CANCELED else "failed")

    def navigate(self, waypoints: list[dict[str, Any]], stop, on_remaining=None) -> str:
        """Nav2 navigate_through_poses — 경로점을 차례로. on_remaining(남은 점 수)"""
        if NavigateThroughPoses is None:
            return "no_server"
        goal = NavigateThroughPoses.Goal()
        poses = [self._pose_stamped(p) for p in waypoints]
        if hasattr(goal, "poses"):                     # Jazzy
            goal.poses = poses
        else:                                          # 이후 판 — nav_msgs/Goals
            goal.goals.goals = poses
        fb = (lambda m: on_remaining(int(m.feedback.number_of_poses_remaining))) if on_remaining else None
        return self._run_action(self.nav, goal, stop, fb)

    def undock(self, stop) -> str:
        return self._run_action(self.undock_ac, Undock.Goal(), stop, timeout=60) if Undock else "no_server"

    def dock(self, stop) -> str:
        return self._run_action(self.dock_ac, Dock.Goal(), stop, timeout=120) if Dock else "no_server"

    def virtual_detect(self, req: dict[str, Any], timeout: float = 3.0) -> dict[str, Any] | None:
        rid = req["request_id"]
        self.pub_detect.publish(String(data=json.dumps(req)))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                r = self.detect_results.pop(rid, None)
            if r:
                return r
            time.sleep(0.05)
        return None

    def capture_jpeg(self, timeout: float = 3.0) -> bytes | None:
        """사진 1장 — 카메라를 잠시 구독해 새 프레임을 받음 (판정에 쓴 사진)"""
        self.add_viewer()
        try:
            seq0, _d, _c = self.frame()
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                seq, data, _ctype = self.frame()
                if data is not None and seq > seq0:
                    return data
                time.sleep(0.05)
            return None
        finally:
            self.remove_viewer()

    def _on_camera(self, m):
        data = bytes(m.data)
        ctype = "image/png" if data[:4] == b"\x89PNG" else "image/jpeg"
        if self.camera_topic not in self.first_seen:          # (카메라)
            self.first_seen.add(self.camera_topic)
            self.log(f"첫 값 받음 {self.camera_topic} ({m.format}, {len(data)} B)")
        now = time.time()
        with self.lock:
            c = self.cam
            if c["t"]:
                dt = now - c["t"]
                if dt > 0:
                    c["fps"] = 1 / dt if not c["fps"] else c["fps"] * 0.8 + (1 / dt) * 0.2
            c.update(data=data, ctype=ctype, seq=c["seq"] + 1, t=now, format=m.format, size=len(data))

    # ---- 화면용 읽기
    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self.lock:
            info = {k: {**v, "age_s": round(now - v["t"], 1), "stale": now - v["t"] > STALE_S}
                    for k, v in self.info.items()}
            return {
                "robot_id": self.robot_id,
                "ros": {"checking": self.checking, "active": self.active, "active_at": self.active_at,
                        "checked_at": self.checked_at, "found_topics": self.found_topics,
                        "check_s": self.check_s, "off_at": self.off_at, "stale_s": STALE_S},
                "info": info,
                "dock_supported": DockStatus is not None,
            }

    def camera_status(self) -> dict[str, Any]:
        now = time.time()
        with self.lock:
            c = self.cam
            return {"topic": self.camera_topic, "subscribed": self.cam_sub is not None, "viewers": self.cam_viewers,
                    "frames": c["seq"], "fps": round(c["fps"], 1) if c["t"] and now - c["t"] < STALE_S else 0,
                    "age_s": round(now - c["t"], 1) if c["t"] else None, "format": c["format"], "size": c["size"]}

    def frame(self) -> tuple[int, bytes | None, str]:
        with self.lock:
            return self.cam["seq"], self.cam["data"], self.cam["ctype"]

    def latest(self, key: str) -> dict[str, Any] | None:
        """로봇 정보 최신값 (STALE_S 안에 받은 것만)."""
        with self.lock:
            v = self.info.get(key)
            return dict(v["data"]) if v and time.time() - v["t"] <= STALE_S else None


# ---------------------------------------------------------------------------
# 아웃박스 — 관제 PC로 보낼 기록을 폴더에 쌓아 순서대로 REST로 보냄 (scv_agent의 일부)
#   파일 하나 = 요청 하나 (번호 순). 관제 PC가 받으면(2xx) 지움, 연결 실패 · 5xx · 409는 잠시 뒤 다시,
#   그 밖의 4xx는 다시 보내도 같으므로 failed/ 폴더로 옮기고 다음으로 넘어감
#   관제 PC는 seq · event_uid · inspection_uid · (task_id, ap_id) · object_key로 중복을 무시하므로 다시 보내도 됨
# ---------------------------------------------------------------------------
class Outbox:
    def __init__(self, folder: Path):
        self.dir = folder
        (folder / "failed").mkdir(parents=True, exist_ok=True)
        (folder / "files").mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        names = sorted(p.name for p in folder.glob("[0-9]*.json"))
        self.seq = int(names[-1].split("_")[0]) + 1 if names else 1
        self.sent = 0
        self.last_error: str | None = None
        self.state_path = folder / "state.dat"        # run마다 표본 seq · 이벤트 번호 (client를 다시 켜도 이어서)
        try:
            self.state = json.loads(self.state_path.read_text())
        except (OSError, ValueError):
            self.state = {}
        threading.Thread(target=self._loop, name="outbox", daemon=True).start()

    def counter(self, key: str, start: int = 0, step: int = 1) -> int:
        """run별 번호를 step만큼 올리고 마지막 값을 돌려줌 (파일에 남김 — client를 다시 켜도 이어서)"""
        with self.lock:
            v = max(int(self.state.get(key, 0)), start) + step
            self.state[key] = v
            tmp = self.state_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.state))
            os.replace(tmp, self.state_path)
            return v

    def put(self, kind: str, path: str, body: dict[str, Any], file: bytes | None = None):
        with self.lock:
            n = self.seq
            self.seq += 1
            item = {"kind": kind, "path": path, "body": body}
            if file is not None:
                fp = self.dir / "files" / f"{n:010d}.bin"
                fp.write_bytes(file)
                item["file"] = fp.name
            tmp = self.dir / f"{n:010d}_{kind}.tmp"
            tmp.write_text(json.dumps(item, ensure_ascii=False, default=str))
            os.replace(tmp, self.dir / f"{n:010d}_{kind}.json")

    def pending(self) -> int:
        return len(list(self.dir.glob("[0-9]*.json")))

    def status(self) -> dict[str, Any]:
        return {"folder": str(self.dir), "pending": self.pending(), "sent": self.sent, "last_error": self.last_error}

    def _send(self, item: dict[str, Any]) -> int:
        url = CFG["server"] + item["path"]
        if item.get("file"):                           # 사진 — multipart (meta JSON + 파일)
            boundary = uuid.uuid4().hex
            data = (self.dir / "files" / item["file"]).read_bytes()
            parts = [f"--{boundary}\r\nContent-Disposition: form-data; name=\"meta\"\r\n\r\n".encode()
                     + json.dumps(item["body"]).encode() + b"\r\n",
                     f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"photo.jpg\"\r\n"
                     "Content-Type: image/jpeg\r\n\r\n".encode() + data + b"\r\n", f"--{boundary}--\r\n".encode()]
            req = urllib.request.Request(url, data=b"".join(parts), method="POST",
                                         headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
        else:
            req = urllib.request.Request(url, data=json.dumps(item["body"], default=str).encode(), method="POST",
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status
        except urllib.error.HTTPError as e:
            self.last_error = f"{e.code} {e.read()[:200].decode(errors='replace')}"
            return e.code

    def _loop(self):
        backoff = 1.0
        while True:
            files = sorted(self.dir.glob("[0-9]*.json"))
            if not files:
                time.sleep(0.3)
                continue
            f = files[0]
            try:
                item = json.loads(f.read_text())
                item["path"], item["body"]
            except (OSError, ValueError, KeyError) as e:   # 깨진 파일 — 옮기고 다음으로
                print(f"[outbox] {f.name} 읽기 실패 — failed/로 옮김: {e}", flush=True)
                os.replace(f, self.dir / "failed" / f.name)
                continue
            try:
                code = self._send(item)
            except (OSError, urllib.error.URLError) as e:   # 관제 PC 연결 안 됨 — 기다렸다 다시
                self.last_error = f"{type(e).__name__} {e}"
                time.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
                continue
            if 200 <= code < 300:
                f.unlink(missing_ok=True)
                if item.get("file"):
                    (self.dir / "files" / item["file"]).unlink(missing_ok=True)
                self.sent += 1
                backoff = 1.0
            elif code == 409 or code >= 500:
                time.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
            else:
                print(f"[outbox] {f.name} 거부 {code} — failed/로 옮김: {self.last_error}", flush=True)
                os.replace(f, self.dir / "failed" / f.name)


# ---------------------------------------------------------------------------
# survey_buffer — RSSI + 같은 시각 AMCL 위치 + seq. 자기 섹터 칸 표본만 (다른 섹터를 지나며 잰 것은 버림)
#   후보 규칙(임시 — 함수 하나, 4차에 교체): 같은 칸 표본 min_samples개 이상이고 「가장 센 AP의 중앙값」 < rssi_suspect_dbm
# ---------------------------------------------------------------------------
POSE_SIGMA_OK_M = 0.25                 # AMCL 위치 σ가 이보다 크면 pose_ok = false (E5)
POSE_MAX_AGE_S = 1.5


def candidate_rule(medians: dict[str, float], n_min: int, counts: dict[str, int], thr: float) -> float | None:
    """칸 하나 — 후보면 점수(가장 센 AP 중앙값), 아니면 None"""
    ok = {s: m for s, m in medians.items() if counts.get(s, 0) >= n_min}
    if not ok:
        return None
    best = max(ok.values())
    return best if best < thr else None


def iqr(vals: list[float]) -> float:
    if len(vals) < 2:
        return 0.0
    q = statistics.quantiles(vals, n=4, method="inclusive")
    return float(q[2] - q[0])


class SurveyBuffer:
    def __init__(self, ros: RosBridge, outbox: Outbox):
        self.ros, self.outbox = ros, outbox
        self.lock = threading.Lock()
        self.cfg: dict[str, Any] | None = None
        self.cells: dict[tuple[int, int], int] = {}    # 자기 섹터 칸 (ix, iy) → cell_id
        self.ssids: set[str] = set()
        self.collect_task: str | None = None           # 표본을 모으는 SURVEY task
        self.precise: dict[str, list[float]] | None = None
        self.batch: list[dict[str, Any]] = []
        self.vals: dict[int, dict[str, list[float]]] = {}   # cell_id → ssid → RSSI (이 run)
        self.where: dict[int, tuple[float, float]] = {}     # cell_id → 그 칸에서 마지막으로 잰 위치 (재방문 지점)
        self.reported: set[int] = set()
        self.seen: dict[str, float] = {}               # ssid → 마지막으로 받은 시각 (status의 ssids → 관제 PC AP 활성)
        self.stats = {"kept": 0, "other_sector": 0}
        self.seq = 0                                   # 이 run의 마지막 표본 번호 (flush 때 아웃박스 상태 파일에 남김)
        ros.on_rssi = self.on_rssi

    def configure(self, cfg: dict[str, Any]):
        with self.lock:
            new_run = not self.cfg or self.cfg.get("run_id") != cfg.get("run_id")
            self.cfg = cfg
            self.cells = {(int(ix), int(iy)): int(cid) for cid, ix, iy in cfg.get("sector_cells") or []}
            self.ssids = set(cfg.get("ssids") or [])
            if new_run:
                self.vals, self.where, self.reported = {}, {}, set()
                self.stats = {"kept": 0, "other_sector": 0}
        if new_run:                                    # 이어서 보낼 seq — 관제 PC에 저장된 마지막 번호와 이 PC 기록 중 큰 값
            try:
                with urllib.request.urlopen(f"{CFG['server']}/api/agent/last-seq?robot_id={self.ros.robot_id}"
                                            f"&run_id={cfg['run_id']}", timeout=5) as r:
                    last = int(json.loads(r.read())["last_seq"])
            except (OSError, ValueError, KeyError):
                last = 0
            self.seq = self.outbox.counter(f"seq_r{cfg['run_id']}", last, 0)

    def recent_ssids(self) -> list[str]:
        now = time.time()
        with self.lock:
            return sorted(s for s, t in self.seen.items() if now - t < 5)

    def on_rssi(self, d: dict[str, Any]):              # ROS 스레드
        now = time.time()
        scans = d.get("scans") or []
        with self.lock:
            for sc in scans:
                self.seen[sc["ssid"]] = now
            cfg, task, prec = self.cfg, self.collect_task, self.precise
        if prec is not None:                           # 정밀측정 중 — 멈춰 선 자리의 값만
            with self.lock:
                for sc in scans:
                    if sc["ssid"] in self.ssids:
                        prec.setdefault(sc["ssid"], []).append(float(sc["rssi_dbm"]))
            return
        if not cfg or not task:
            return
        with self.ros.lock:
            info = self.ros.info.get("pose")
            pose = dict(info["data"]) if info else None
            age = now - info["t"] if info else 1e9
        if not pose or age > STALE_S:
            return
        pose_ok = (pose.get("sigma_xy") or 0) <= POSE_SIGMA_OK_M and age <= POSE_MAX_AGE_S
        g = cfg["grid"]
        key = (math.floor((pose["x"] - g["origin_x"]) / g["size"]), math.floor((pose["y"] - g["origin_y"]) / g["size"]))
        cid = self.cells.get(key)
        if cid is None:                                # 자기 섹터 밖(다른 섹터 · 격자 밖) — 버림
            with self.lock:
                self.stats["other_sector"] += 1
            return
        rows = []
        for sc in scans:
            if sc["ssid"] not in self.ssids:
                continue
            with self.lock:
                self.seq += 1
                seq = self.seq
            rows.append({"seq": seq, "stamp": d.get("stamp") or utc_now_iso(), "x": pose["x"], "y": pose["y"],
                         "yaw": pose["yaw"], "pose_ok": bool(pose_ok), "ssid": sc["ssid"], "rssi_dbm": int(sc["rssi_dbm"]),
                         "task_id": task})                 # 표본을 모은 측정 task (이력 「자세히」)
        with self.lock:
            self.batch += rows
            self.stats["kept"] += len(rows)
            if pose_ok:
                for r_ in rows:
                    self.vals.setdefault(cid, {}).setdefault(r_["ssid"], []).append(float(r_["rssi_dbm"]))
                self.where[cid] = (pose["x"], pose["y"])

    def flush(self):
        with self.lock:
            batch, self.batch = self.batch, []
            cfg, seq = self.cfg, self.seq
        if batch and cfg:
            self.outbox.counter(f"seq_r{cfg['run_id']}", seq, 0)
            self.outbox.put("samples", "/api/agent/samples",
                            {"robot_id": self.ros.robot_id, "run_id": cfg["run_id"], "samples": batch})

    def find_candidates(self) -> list[dict[str, Any]]:
        """새로 후보가 된 칸 → CANDIDATE 이벤트 detail 목록"""
        st = self.cfg["settings"]
        thr, n_min = float(st.get("rssi_suspect_dbm") or -72), int(float(st.get("min_samples") or 3))
        out = []
        with self.lock:
            for cid, by in sorted(self.vals.items()):
                if cid in self.reported:
                    continue
                med = {s: statistics.median(v) for s, v in by.items() if v}
                score = candidate_rule(med, n_min, {s: len(v) for s, v in by.items()}, thr)
                if score is None:
                    continue
                self.reported.add(cid)
                out.append({"cell_id": cid, "score": score, "medians": med, "target": self.where.get(cid)})
        return out

    def measure(self, seconds: float, stop) -> dict[str, list[float]]:
        """정지 정밀측정 — seconds 동안 받은 RSSI (ssid → 값들)"""
        with self.lock:
            self.precise = {}
        end = time.monotonic() + seconds
        while time.monotonic() < end and not stop():
            time.sleep(0.1)
        with self.lock:
            got, self.precise = self.precise, None
        return got


# ---------------------------------------------------------------------------
# detection_alert — 설비 판정 1회 + 사진 (3-3차 결정: 다시 판정 · 설비 후보 없음, 1회 판정이 끝)
#   3차는 가상 판정: 가상 로봇에 virtual/detect_request → virtual/detect_result (world.yaml 정답)
#   실제 구성에서는 같은 자리에서 카메라 영상 YOLO 결과를 쓰면 됨 (이후 차수)
#   판정 규칙(임시 — 함수 하나, 4차에 교체): 가림 · 정면 아님 → 자세 보정 재관측(max_pose_retry까지) → 넘으면 OCCLUDED · 미확인
#     안 보임 → NOT_DETECTED · 위치가 POSITION_TOL_M 넘게 다름 → POSITION_ANOMALY · 상태 이상 → ABNORMAL · 그 밖 NORMAL
# ---------------------------------------------------------------------------
POSITION_TOL_M = 0.5
MODEL_VERSION = "virtual-truth-1"


def judge(fac: dict[str, Any], res: dict[str, Any] | None) -> dict[str, Any]:
    if res is None:
        return {"detail": "NOT_DETECTED", "final": "unconfirmed", "frontal": False, "det": None}
    if res.get("occluded") or not res.get("frontal"):
        return {"detail": "OCCLUDED", "final": "unconfirmed", "frontal": False, "det": None}
    dets = [d for d in res.get("detections") or [] if d.get("class") == fac["facility_type"]]
    if not dets:
        return {"detail": "NOT_DETECTED", "final": "abnormal", "frontal": True, "det": None}
    d = min(dets, key=lambda d: math.hypot(d["x"] - fac["expected_x"], d["y"] - fac["expected_y"]))
    if math.hypot(d["x"] - fac["expected_x"], d["y"] - fac["expected_y"]) > POSITION_TOL_M:
        return {"detail": "POSITION_ANOMALY", "final": "abnormal", "frontal": True, "det": d}
    if d.get("state") == "abnormal" or d.get("gauge") == "abnormal":
        return {"detail": "ABNORMAL", "final": "abnormal", "frontal": True, "det": d}
    return {"detail": "NORMAL", "final": "normal", "frontal": True, "det": d}


class DetectionAlert:
    def __init__(self, ros: RosBridge):
        self.ros = ros

    def inspect(self, fac: dict[str, Any], max_retry: int, stop) -> tuple[dict[str, Any], bytes | None, int]:
        retry = 0
        while True:
            res = self.ros.virtual_detect({"request_id": uuid.uuid4().hex[:12], **{k: fac[k] for k in (
                "facility_id", "facility_type", "has_gauge", "expected_x", "expected_y")}})
            j = judge(fac, res)
            if j["detail"] != "OCCLUDED" or retry >= max_retry or stop():
                break
            retry += 1                                 # 자세 보정 후 다시 관측 (가상: 같은 자리에서 다시)
            time.sleep(1.0)
        return j, self.ros.capture_jpeg(), retry


# ---------------------------------------------------------------------------
# mission_executor — task 실행 담당. 1초마다 상태(status), 관제 PC가 보낸 task를 하나씩 실행
#   관제 PC 연결이 끊겨도 받은 task는 끝까지 하고, 결과는 아웃박스로 (다시 연결되면 보냄)
# ---------------------------------------------------------------------------
class MissionExecutor:
    def __init__(self, ros: RosBridge, survey: SurveyBuffer, detect: DetectionAlert, outbox: Outbox):
        self.ros, self.survey, self.detect, self.outbox = ros, survey, detect, outbox
        self.run_cfg: dict[str, Any] | None = None
        self.run_id: int | None = None
        self.task: dict[str, Any] | None = None        # 실행 중 task (task_assign 본문)
        self.progress: float | None = None
        self.phase = "IDLE"                            # NAVIGATING · MEASURING · INSPECTING · UNDOCKING · DOCKING · RETURNING
        self.estop = False
        self.q: queue.Queue = queue.Queue()
        self.cancel = threading.Event()
        self.go_dock = False
        self.send_ws = None                            # ScvAgent가 넣는 함수 (type, body) — 진행률 · 받음 · 거부
        self.history: list[dict[str, Any]] = []        # 최근 task 결과 (화면 표시)
        self.log: dict[str, Any] = {"run_id": None, "candidates": [], "inspections": []}   # 이번 순찰 (카메라 탭)
        self.photos: dict[str, bytes] = {}             # 판정 uid → 사진 (최근 몇 장, 카메라 탭)
        threading.Thread(target=self._loop, name="mission", daemon=True).start()

    @property
    def patrolling(self) -> bool:
        """순찰 중 = 관제 PC의 run을 받아 둔 동안 (측정 · 정밀측정 · 탐지 · 복귀 task 모두)"""
        return self.run_id is not None or bool(self.task and self.task.get("task_id"))

    # ---- 관제 PC 명령 (ScvAgent가 부름)
    def on_run_config(self, cfg: dict[str, Any]):
        self.run_cfg, self.run_id = cfg, cfg.get("run_id")
        if self.log["run_id"] != self.run_id:          # 새 순찰 — 카메라 탭 기록을 새로
            self.log = {"run_id": self.run_id, "candidates": [], "inspections": []}
            self.photos = {}
        self.survey.configure(cfg)
        print(f"[mission] run_config run {self.run_id} — 섹터 {cfg.get('sectors')} 칸 {len(cfg.get('sector_cells') or [])}개 "
              f"· SSID {cfg.get('ssids')}", flush=True)

    def on_task_assign(self, t: dict[str, Any]):
        cur = self.task
        if cur and cur["task_id"] == t["task_id"]:
            return                                     # 이미 하는 중 (재접속 때 다시 온 것)
        if any(x["task_id"] == t["task_id"] for x in list(self.q.queue)):
            return
        self.q.put(t)

    def on_task_cancel(self, task_id: str, go_dock: bool):
        with self.q.mutex:
            keep = [x for x in self.q.queue if x.get("task_id") != task_id]
            self.q.queue.clear()
            self.q.queue.extend(keep)
        if self.task and self.task["task_id"] == task_id:
            self.go_dock = go_dock
            self.cancel.set()
        elif go_dock and not self.task:
            self.q.put({"type": "_RETURN", "task_id": None})

    def on_run_end(self, run_id: int, status: str):
        """관제 PC가 순찰을 끝냄(scv_close_run) — 순찰 표시를 끝냄 (기록은 다음 순찰까지 남김)"""
        if self.run_id == run_id:
            self.run_id = None
            self.run_cfg = None
        self.log["status"] = status

    def on_run_cancel(self, run_id: int, go_dock: bool):
        with self.q.mutex:
            self.q.queue.clear()
        self.log["status"] = "aborted"
        dock = (self.run_cfg or {}).get("dock_pose")
        self.run_cfg = None if self.run_id == run_id else self.run_cfg
        self.home_pose = dock
        if self.task:
            self.go_dock = go_dock
            self.cancel.set()
        elif go_dock:
            self.q.put({"type": "_RETURN", "task_id": None})
        self.run_id = None

    # ---- 상태 (1 Hz)
    def status(self) -> dict[str, Any]:
        pose = self.ros.latest("pose")
        bat = self.ros.latest("battery")
        dock = self.ros.latest("dock")
        docked = bool(dock and dock["is_docked"])
        nav = "STOPPED" if self.estop else (self.phase if self.phase != "IDLE" else ("DOCKED" if docked else "IDLE"))
        t = self.task if self.task and self.task.get("task_id") else None
        return {
            "stamp": utc_now_iso(), "robot_id": self.ros.robot_id,
            "run_id": t and t.get("run_id"), "task_id": t and t["task_id"], "nav_state": nav,
            "progress": round(self.progress, 3) if t and self.progress is not None else None,
            "x": pose["x"] if pose else None, "y": pose["y"] if pose else None, "yaw": pose["yaw"] if pose else None,
            "battery_pct": bat["percentage"] if bat else None, "is_docked": docked, "estop": self.estop,
            "ssids": self.survey.recent_ssids(),
        }

    def snapshot(self) -> dict[str, Any]:
        t = self.task
        return {"run_id": self.run_id, "phase": self.phase, "progress": self.progress,
                "task": t and {k: t.get(k) for k in ("task_id", "type", "target_ref")},
                "queued": [x.get("task_id") for x in list(self.q.queue)], "history": self.history[-8:],
                "survey": dict(self.survey.stats)}

    # ---- 실행
    def _loop(self):
        while True:
            t = self.q.get()
            self.cancel.clear()
            self.go_dock = False
            self.task, self.progress = t, 0.0
            try:
                if t["type"] == "_RETURN":
                    self._return_home()
                    continue
                ok, reason = self._execute(t)
                if self.cancel.is_set():               # 관제 PC가 취소 — 결과는 보내지 않음
                    self._note(t, "취소됨")
                    if self.go_dock:
                        self._return_home()
                else:
                    self._result(t, ok, reason)
            except Exception as e:                     # 예상 못 한 오류 — 실패로 보고
                traceback.print_exc()
                self._result(t, False, f"ERROR:{type(e).__name__}")
            finally:
                self.task, self.progress, self.phase = None, None, "IDLE"

    def _note(self, t, text):
        self.history.append({"task_id": t.get("task_id"), "type": t.get("type"), "result": text, "at": utc_now_iso()})
        print(f"[mission] {t.get('task_id')} {t.get('type')} — {text}", flush=True)

    def _result(self, t, ok, reason):
        self.survey.flush()
        self.outbox.put("result", f"/api/agent/tasks/{t['task_id']}/result",
                        {"robot_id": self.ros.robot_id, "ok": ok, "reason": reason, "ended_at": utc_now_iso()})
        self._note(t, "완료" if ok else f"실패 {reason}")

    def _event(self, code: str, level: str, detail: dict[str, Any] | None = None, task_id: str | None = None):
        run = (self.task or {}).get("run_id") or self.run_id
        n = self.outbox.counter(f"event_r{run}")
        ev = {"event_uid": f"{self.ros.robot_id}-r{run}-e{n:06d}", "run_id": run,
              "task_id": task_id if task_id is not None else (self.task or {}).get("task_id"),
              "code": code, "source": "robot", "level": level, "detail": detail, "ts": utc_now_iso()}
        self.outbox.put("event", "/api/agent/events", {"robot_id": self.ros.robot_id, "events": [ev]})

    def _ws(self, kind, body):
        if self.send_ws:
            self.send_ws(kind, body)

    def _setting(self, key, default):
        try:
            return float(((self.run_cfg or {}).get("settings") or {}).get(key))
        except (TypeError, ValueError):
            return default

    def _battery_low(self) -> bool:
        b = self.ros.latest("battery")
        return bool(b and b["percentage"] is not None and b["percentage"] <= self._setting("battery_return_pct", 15))

    def _drive(self, t, waypoints, p0=0.0, p1=1.0, check_battery=True) -> tuple[bool, str | None]:
        """undock(도킹 중이면) → navigate_through_poses. 진행률 p0~p1. 반환 (ok, 실패 사유)"""
        d = self.ros.latest("dock")
        if d and d.get("is_docked"):
            self.phase = "UNDOCKING"
            r = self.ros.undock(self.cancel.is_set)
            if r != "ok":
                return False, "UNDOCK_FAILED" if r != "stopped" else "CANCELED"
            self._event("UNDOCK", "info", {"x": (self.ros.latest("pose") or {}).get("x"), "y": (self.ros.latest("pose") or {}).get("y")})
        self.phase = "NAVIGATING"
        total = max(1, len(waypoints))
        low = {"hit": False}
        last = {"t": 0.0}

        def stop():
            if check_battery and self._battery_low():
                low["hit"] = True
            now = time.monotonic()
            if now - last["t"] > 1.0:                  # 1초마다 표본 묶음 보내기 · 진행률
                last["t"] = now
                self.survey.flush()
                self._ws("task_progress", {"task_id": t["task_id"], "progress": round(self.progress or 0, 3)})
            return self.cancel.is_set() or low["hit"]

        def remaining(n):
            self.progress = p0 + (p1 - p0) * (1 - n / total)

        r = self.ros.navigate(waypoints, stop, remaining)
        if low["hit"]:
            self._event("E2", "warn", {"battery_pct": (self.ros.latest("battery") or {}).get("percentage")})
            return False, "BATTERY_LOW"
        if r == "ok":
            self.progress = p1
            return True, None
        if r == "stopped":
            return False, "CANCELED"
        if r == "no_server":
            return False, "NO_NAV2"
        p = self.ros.latest("pose") or {}
        self._event("E1", "warn", {"x": p.get("x"), "y": p.get("y")})
        return False, "BLOCKED"

    def _execute(self, t) -> tuple[bool, str | None]:
        if t.get("map_path") and not os.path.exists(t["map_path"]):   # 작업의 지도 파일이 이 제어 PC에 없음
            self._ws("task_rejected", {"task_id": t["task_id"], "reason": "MAP_MISMATCH"})
            self._event("MAP_MISMATCH", "error", {"map_path": t["map_path"]})
            self.cancel.set()                          # 거부는 WS로 알렸으므로 결과는 보내지 않음
            return False, "MAP_MISMATCH"
        self._ws("task_accepted", {"task_id": t["task_id"]})
        kind = t["type"]
        self._note(t, f"시작 ({len(t.get('waypoints') or [])}점)")
        if kind == "SURVEY_SEGMENT":
            self.survey.collect_task = t["task_id"]
            try:
                ok, why = self._drive(t, t["waypoints"])
            finally:
                self.survey.collect_task = None
            self.survey.flush()
            for c in self.survey.find_candidates():    # 이번 구간까지 쌓인 칸 중 새 후보 (같은 칸은 한 번만)
                n = self.outbox.counter(f"cand_r{t['run_id']}")
                cid = f"r{t['run_id']}-{self.ros.robot_id}-c{n:02d}"
                self._event("CANDIDATE", "warn", {"candidate_id": cid, "source": "NETWORK", "kind": "NET_COVERAGE",
                                                  "ref": str(c["cell_id"]), "target": list(c["target"]), "score": c["score"],
                                                  "medians": c["medians"]})
                self.log["candidates"].append({"candidate_id": cid, "cell_id": c["cell_id"], "score": c["score"],
                                               "target": list(c["target"]), "outcome": None, "precise": None, "at": utc_now_iso()})
            return ok, why
        if kind == "REINSPECTION":
            ok, why = self._drive(t, t["waypoints"], 0.0, 0.3)
            if not ok:
                return ok, why
            self.phase = "MEASURING"
            sec = self._setting("precise_duration_s", 30)
            t0 = time.monotonic()
            got = {}
            th = threading.Thread(target=lambda: got.update(self.survey.measure(sec, self.cancel.is_set)))
            th.start()
            while th.is_alive():
                self.progress = 0.3 + 0.7 * min(1.0, (time.monotonic() - t0) / max(1, sec))
                self._ws("task_progress", {"task_id": t["task_id"], "progress": round(self.progress, 3)})
                th.join(1.0)
            if self.cancel.is_set():
                return False, "CANCELED"
            cand = t.get("candidate") or {}
            meas = [{"ssid": s, "duration_s": int(round(time.monotonic() - t0)), "n_samples": len(v),
                     "rssi_median": statistics.median(v), "rssi_iqr": iqr(v), "measured_at": utc_now_iso()}
                    for s, v in sorted(got.items()) if v]
            if meas:
                self.outbox.put("precise", "/api/agent/precise-measures",
                                {"robot_id": self.ros.robot_id, "task_id": t["task_id"], "cell_id": int(cand.get("ref")),
                                 "measures": meas})
            best = max((m["rssi_median"] for m in meas), default=None)
            thr = self._setting("rssi_suspect_dbm", -72)
            code = "UNCONFIRMED" if best is None else ("CONFIRMED" if best < thr else "CLEARED")
            self._event(code, "error" if code == "CONFIRMED" else "info",
                        {"candidate_id": t["target_ref"], "best_median": best,
                         "medians": {m["ssid"]: m["rssi_median"] for m in meas}})
            for c in self.log["candidates"]:
                if c["candidate_id"] == t["target_ref"]:
                    c.update(outcome=code, precise=best)
            return True, None
        if kind == "FACILITY_CHECK":
            ok, why = self._drive(t, t["waypoints"], 0.0, 0.6)
            if not ok:
                return ok, why
            self.phase = "INSPECTING"
            fac = t.get("facility") or {}
            j, photo, retry = self.detect.inspect(fac, int(self._setting("max_pose_retry", 3)), self.cancel.is_set)
            if self.cancel.is_set():
                return False, "CANCELED"
            uid = str(uuid.uuid4())
            d = j["det"] or {}
            self.outbox.put("inspection", "/api/agent/inspections", {
                "robot_id": self.ros.robot_id, "inspection_uid": uid, "task_id": t["task_id"], "facility_id": fac["facility_id"],
                "detail": j["detail"], "final": j["final"], "frontal": j["frontal"], "pose_retry": retry,
                "gauge_state": d.get("gauge") if fac.get("has_gauge") and j["frontal"] and d else None,
                "confidence": d.get("conf"), "bbox": d.get("bbox"), "model_version": MODEL_VERSION,
                "inspected_at": utc_now_iso()})
            if photo:
                self.outbox.put("media", "/api/agent/media", {"robot_id": self.ros.robot_id, "inspection_uid": uid,
                                                              "kind": "rgb", "index": 1, "captured_at": utc_now_iso()}, photo)
                self.photos[uid] = photo
                for old in list(self.photos)[:-12]:        # 화면용 사진은 최근 12장만
                    self.photos.pop(old, None)
            self.log["inspections"].append({"facility_id": fac["facility_id"], "facility_type": fac.get("facility_type"),
                                            "detail": j["detail"], "final": j["final"], "pose_retry": retry,
                                            "uid": uid if photo else None, "at": utc_now_iso()})
            if j["detail"] == "OCCLUDED":              # 자세 보정을 max_pose_retry번 해도 정면을 못 봄 → 미확인(E6)
                self._event("E6", "warn", {"facility_id": fac["facility_id"], "pose_retry": retry})
            self.progress = 1.0
            return True, None
        if kind == "DOCK":
            d = self.ros.latest("dock")
            if d and d.get("is_docked"):
                return True, None
            ok, why = self._drive(t, t["waypoints"], 0.0, 0.8, check_battery=False)
            if not ok:
                return ok, why
            self.phase = "DOCKING"
            r = self.ros.dock(self.cancel.is_set)
            if r != "ok":
                return False, "DOCK_FAILED" if r != "stopped" else "CANCELED"
            self._event("DOCK", "info", {})
            return True, None
        return False, f"UNKNOWN_TYPE:{kind}"

    def _return_home(self):
        """순찰 중단 · 관제 PC에 없는 task — task 없이 도크로 복귀"""
        dock = ((self.run_cfg or {}).get("dock_pose") or getattr(self, "home_pose", None)
                or ((link.config or {}).get("dock_pose") if link else None))
        d = self.ros.latest("dock")
        if not dock or (d and d.get("is_docked")):
            return
        self.cancel.clear()
        self.phase = "RETURNING"
        print("[mission] 도크로 복귀", flush=True)
        if self.ros.navigate([dock], lambda: False) == "ok":
            self.phase = "DOCKING"
            self.ros.dock(lambda: False)


# ---------------------------------------------------------------------------
# scv_agent — 관제 PC와의 유일한 창구 (ROS 노드 아님). 로봇이 활성이 되면 WebSocket /ws/agent에 접속해
#   hello 한 번 → 이후 1초마다 status, 관제 PC 명령(run_config · task_assign · task_cancel · run_cancel)을 받음.
#   기록은 Outbox가 REST로. 로봇이 꺼지면(값 끊김) 연결을 닫아 관제 PC도 비활성
# ---------------------------------------------------------------------------
class ScvAgent:
    STATUS_PERIOD_S = 1.0

    def __init__(self, ros: RosBridge, executor: MissionExecutor, outbox: Outbox):
        self.ros = ros
        self.executor = executor
        self.outbox = outbox
        self.status_sent = 0
        self.status_at: float | None = None
        self.state = "waiting"         # waiting(로봇 비활성) · connecting · connected · retry
        self.detail = "로봇 활성 확인 전"
        self.hello_at: float | None = None
        self.welcome: dict[str, Any] | None = None
        self.config: dict[str, Any] | None = None      # 관제 PC config — 현재 지도 · 이 로봇의 도크 접근 자세 (3-2차)
        self.config_at: float | None = None
        self.connects = 0
        self.loop: asyncio.AbstractEventLoop | None = None
        self.outq: asyncio.Queue | None = None         # 다른 스레드(MissionExecutor)가 보내는 WS 메시지
        self.sent_n = 0
        executor.send_ws = self.send_ws

    def ws_url(self) -> str:
        s = CFG["server"]
        return ("wss://" + s[8:] if s.startswith("https://") else "ws://" + s.split("://", 1)[-1]) + "/ws/agent"

    def send_ws(self, kind: str, body: dict[str, Any]):
        """스레드 안전 — 연결 중일 때만 보냄 (진행률 등은 끊겨 있으면 버림)"""
        if self.loop and self.outq is not None and self.state == "connected":
            self.loop.call_soon_threadsafe(self.outq.put_nowait, (kind, body))

    async def run(self):
        self.loop = asyncio.get_running_loop()
        backoff = 1.0
        while True:
            if not self.ros.active:                    # 로봇이 활성이 될 때까지 기다림 (꺼진 뒤에도 같음)
                if self.state != "waiting":
                    self.state = "waiting"
                    self.detail = ("로봇 활성 확인 전" if self.ros.off_at is None
                                   else "로봇 값이 끊겨 연결을 닫음 — 관제 PC에서 비활성")
                await asyncio.sleep(0.3)
                continue
            self.state, self.detail = "connecting", f"{self.ws_url()} 접속 중"
            try:
                async with ws_connect(self.ws_url(), open_timeout=5) as ws:
                    t = self.executor.task
                    hello = {"type": "hello", "msg_id": f"c-{self.connects + 1:06d}", "ts": utc_now_iso(), "body": {
                        "robot_id": self.ros.robot_id, "agent_version": AGENT_VERSION,
                        "run_id": self.executor.run_id, "active_task_id": t and t.get("task_id"),
                        "outbox_pending": self.outbox.pending()}}
                    await ws.send(json.dumps(hello))
                    self.hello_at = time.time()
                    reply = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    if reply.get("type") != "welcome":
                        raise RuntimeError(f"관제 PC 응답: {reply.get('type')} {reply.get('body')}")
                    self.welcome = reply
                    self.connects += 1
                    self.outq = asyncio.Queue()
                    self.state, self.detail = "connected", "hello 보냄 — 관제 PC에서 활성"
                    backoff = 1.0
                    print(f"[agent] 관제 PC 연결 · hello 보냄 ({self.ros.robot_id})", flush=True)
                    next_status = 0.0
                    while self.ros.active:
                        now = time.monotonic()
                        if now >= next_status:
                            next_status = now + self.STATUS_PERIOD_S
                            body = self.executor.status()
                            await ws.send(json.dumps({"type": "status", "msg_id": f"c-s{self.status_sent + 1:08d}",
                                                      "ts": body["stamp"], "body": body}))
                            self.status_sent += 1
                            self.status_at = time.time()
                            self.detail = f"hello 보냄 · status {self.status_sent}건 보냄 (1초마다)"
                        while not self.outq.empty():   # 진행률 · task 받음 · 거부
                            kind, b = self.outq.get_nowait()
                            self.sent_n += 1
                            await ws.send(json.dumps({"type": kind, "msg_id": f"c-m{self.sent_n:08d}", "ts": utc_now_iso(),
                                                      "body": b}))
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=max(0.05, min(0.2, next_status - time.monotonic())))
                        except asyncio.TimeoutError:
                            continue
                        try:
                            self.on_message(json.loads(raw))
                        except Exception as e:             # 메시지 하나가 틀려도 연결은 유지
                            print(f"[agent] 받은 메시지 처리 실패: {type(e).__name__} {e}", flush=True)
                    # 로봇이 꺼짐 → 연결을 닫으면 관제 PC가 robots.is_active = false
                    await ws.close(code=1000, reason="robot inactive")
                    print(f"[agent] {self.ros.robot_id} 비활성 — 관제 PC 연결 닫음", flush=True)
                    continue
            except Exception as e:
                self.detail = f"연결 실패: {type(e).__name__} {e}".strip()
            self.state = "retry"
            self.outq = None
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 15.0)

    def on_message(self, msg: dict[str, Any]):
        kind = msg.get("type")
        b = msg.get("body") or {}
        if kind == "config":
            self.config, self.config_at = b, time.time()
            dock = self.config.get("dock_pose")
            mp = self.config.get("map") or {}
            print(f"[agent] config 받음 — 지도 {mp.get('name', '없음')} · 도크 접근 자세 {dock}", flush=True)
            if not dock:
                return
            # 로봇이 도크에 있거나(dock_status) 아직 위치를 모를 때만 시작 위치를 지정 — 순찰 중 위치를 덮어쓰지 않음
            d = self.ros.latest("dock")                 # 최근 STALE_S 안의 값만 (없으면 None)
            p = self.ros.latest("pose")
            docked = bool(d and d.get("is_docked"))
            if (docked and not self.executor.task) or p is None:
                self.ros.set_initial_pose(dock, "도킹 중 · 관제 PC 도크 접근 자세" if docked else "위치 모름 · 관제 PC 도크 접근 자세")
        elif kind == "run_config":
            self.executor.on_run_config(b)
        elif kind == "task_assign":
            print(f"[agent] task_assign {b.get('task_id')} {b.get('type')} ({len(b.get('waypoints') or [])}점)", flush=True)
            self.executor.on_task_assign(b)
        elif kind == "task_cancel":
            print(f"[agent] task_cancel {b.get('task_id')} — {b.get('reason')}", flush=True)
            self.executor.on_task_cancel(b.get("task_id"), bool(b.get("go_dock")))
        elif kind == "run_end":
            print(f"[agent] run_end run {b.get('run_id')} — {b.get('status')}", flush=True)
            self.executor.on_run_end(b.get("run_id"), b.get("status"))
        elif kind == "run_cancel":
            print(f"[agent] run_cancel run {b.get('run_id')} — {b.get('reason')} (도크 복귀 {b.get('go_dock')})", flush=True)
            self.executor.on_run_cancel(b.get("run_id"), bool(b.get("go_dock")))
        else:
            print(f"[agent] ← {kind} (처리하지 않는 메시지)", flush=True)

    def status(self) -> dict[str, Any]:
        return {"server": CFG["server"], "state": self.state, "detail": self.detail,
                "config": self.config, "config_at": self.config_at,
                "hello_at": self.hello_at, "connects": self.connects,
                "status_sent": self.status_sent, "status_at": self.status_at}


# ---------------------------------------------------------------------------
# 웹 (FastAPI)
# ---------------------------------------------------------------------------
ros: RosBridge | None = None
executor: MissionExecutor | None = None
link: ScvAgent | None = None
outbox: Outbox | None = None
SESSIONS: dict[str, dict[str, Any]] = {}


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global ros, executor, link, outbox
    camera_topic = CFG["camera_topic"] or f"/{CFG['robot_id']}/oakd/rgb/image_raw/compressed"
    ros = RosBridge(CFG["robot_id"], camera_topic, CFG["check_s"])
    outbox = Outbox(Path(CFG["outbox"] or BASE / f"outbox_{CFG['robot_id']}").resolve())
    survey = SurveyBuffer(ros, outbox)
    executor = MissionExecutor(ros, survey, DetectionAlert(ros), outbox)
    link = ScvAgent(ros, executor, outbox)
    print(f"[outbox] {outbox.dir} — 보낼 기록 {outbox.pending()}건", flush=True)
    task = asyncio.create_task(link.run())
    yield
    task.cancel()
    ros.shutdown()


app = FastAPI(title="SCV client page", version=AGENT_VERSION, lifespan=lifespan)


def current_user(request: Request) -> dict[str, Any]:
    token = request.cookies.get(SESSION_COOKIE)
    s = SESSIONS.get(token or "")
    if not s or s["expires"] < time.time():
        SESSIONS.pop(token or "", None)
        raise HTTPException(401, "로그인이 필요합니다")
    s["expires"] = time.time() + SESSION_TTL_S
    return s


@app.get("/", include_in_schema=False)
def root():
    return RedirectResponse("/client/login")


@app.get("/client", include_in_schema=False)
def client_root():
    return RedirectResponse("/client/robot")


@app.get("/client/{page}", include_in_schema=False)
def client_page(page: str):
    if page not in PAGES:
        raise HTTPException(404, "없는 화면입니다")
    return FileResponse(STATIC_DIR / "client" / "index.html")


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def server_login(username: str, password: str) -> dict[str, Any]:
    """관제 PC POST /api/client/login — 제어 PC는 DB에 직접 접속하지 않음."""
    req = urllib.request.Request(CFG["server"] + "/api/client/login", method="POST",
                                 data=json.dumps({"username": username, "password": password}).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())["user"]
    except urllib.error.HTTPError as e:
        try:
            detail = json.loads(e.read()).get("detail")
        except Exception:
            detail = None
        raise HTTPException(e.code if e.code in (400, 401) else 502, detail or f"관제 PC 오류 ({e.code})")
    except (urllib.error.URLError, TimeoutError, OSError):
        raise HTTPException(503, f"관제 PC(server page)에 접속하지 못했습니다 — {CFG['server']}")


@app.post("/api/login")
async def login(request: Request):
    body = await request.json()
    username = str(body.get("username", "")).strip()
    password = str(body.get("password", ""))
    if not username or not password:
        raise HTTPException(400, "아이디와 비밀번호를 입력하세요")
    user = await run_in_threadpool(server_login, username, password)
    token = secrets.token_urlsafe(32)
    SESSIONS[token] = {**user, "expires": time.time() + SESSION_TTL_S}
    ros.on_login()                                     # 로그인하면 로봇 활성 확인 시작
    resp = JSONResponse({"username": user["username"], "role": user["role"]})
    resp.set_cookie(SESSION_COOKIE, token, httponly=True, samesite="lax", max_age=SESSION_TTL_S, path="/")
    return resp


@app.post("/api/logout")
def logout(request: Request):
    SESSIONS.pop(request.cookies.get(SESSION_COOKIE, ""), None)
    resp = JSONResponse({"ok": True})
    resp.delete_cookie(SESSION_COOKIE, path="/")
    return resp


@app.get("/api/me")
def me(request: Request):
    try:
        u = current_user(request)
    except HTTPException:
        return {"user": None, "robot_id": CFG["robot_id"]}
    return {"user": {"username": u["username"], "role": u["role"]}, "robot_id": CFG["robot_id"]}


@app.get("/api/robot")
def robot(request: Request):
    current_user(request)
    return {**ros.snapshot(), "link": link.status(), "status": executor.status(), "initialpose": ros.initialpose,
            "mission": executor.snapshot(), "outbox": outbox.status()}


@app.get("/api/camera")
def camera(request: Request):
    current_user(request)
    return {**ros.camera_status(), "patrol": executor.patrolling}


@app.get("/api/patrol")
def patrol(request: Request):
    """카메라 탭 — 순찰 중 여부 · 이 로봇 위치 · 수행 중 task 경로 · 이번 순찰의 후보 · 판정"""
    current_user(request)
    t = executor.task if executor.task and executor.task.get("task_id") else None
    cfg = executor.run_cfg or {}
    mp = (cfg.get("map") or (link.config or {}).get("map") or {}).get("robot_map_path")
    return {"patrol": executor.patrolling, "run_id": executor.run_id, "phase": executor.phase,
            "progress": executor.progress, "pose": ros.latest("pose"),
            "task": t and {k: t.get(k) for k in ("task_id", "type", "target_ref", "waypoints")},
            "log": executor.log, "map_key": f"{mp}|{executor.run_id}"}


@app.get("/api/patrol/photo/{uid}")
def patrol_photo(uid: str, request: Request):
    current_user(request)
    data = executor.photos.get(uid)
    if not data:
        raise HTTPException(404, "사진이 없습니다 (최근 12장만 보관)")
    return Response(data, media_type="image/jpeg")


MAP_CACHE: dict[str, Any] = {}


def local_map(path: str) -> dict[str, Any]:
    """제어 PC의 지도 파일(yaml + pgm) → 표시용 정보 · PNG (파일이 바뀌면 다시 읽음)"""
    yp = Path(path)
    key = f"{yp}|{yp.stat().st_mtime_ns}"
    if MAP_CACHE.get("key") != key:
        meta = yaml.safe_load(yp.read_text(encoding="utf-8"))
        img = Path(str(meta["image"]))
        img = img if img.is_absolute() else yp.parent / img
        with Image.open(img) as im:
            g = im.convert("L")
            buf = io.BytesIO()
            g.save(buf, format="PNG")
            w, h = g.size
        MAP_CACHE.update(key=key, png=buf.getvalue(), meta={
            "path": str(yp), "name": yp.stem, "resolution_m": float(meta["resolution"]),
            "origin_x": float(meta["origin"][0]), "origin_y": float(meta["origin"][1]), "width": w, "height": h,
            "image_url": f"/api/map/image?v={yp.stat().st_mtime_ns}"})
    return MAP_CACHE


@app.get("/api/map")
def map_info(request: Request):
    """카메라 탭 지도 — 제어 PC 지도 파일 + 순찰 중이면 관제 PC가 순찰 시작 때 보낸 섹터 칸 · 도크 · 설비 · 측정 AP"""
    current_user(request)
    cfg = executor.run_cfg or {}
    mp = (cfg.get("map") or (link.config or {}).get("map") or {}).get("robot_map_path")
    if not mp:
        return {"map": None, "reason": "관제 PC config의 지도가 없습니다"}
    try:
        info = local_map(mp)["meta"]
    except (OSError, KeyError, ValueError, TypeError) as e:
        return {"map": None, "reason": f"제어 PC에 지도 파일이 없거나 읽지 못했습니다 — {mp} ({type(e).__name__})"}
    ov = None
    if cfg:
        ov = {k: cfg.get(k) for k in ("grid", "sectors", "sector_cells", "dock_pose", "facilities", "target_ap")}
    return {"map": info, "overlay": ov, "dock_pose": (link.config or {}).get("dock_pose")}


@app.get("/api/map/image")
def map_image(request: Request):
    current_user(request)
    if not MAP_CACHE.get("png"):
        raise HTTPException(404, "지도 이미지가 없습니다")
    return Response(MAP_CACHE["png"], media_type="image/png", headers={"Cache-Control": "max-age=3600"})


@app.get("/api/camera/stream")
async def camera_stream(request: Request):
    """MJPEG(multipart/x-mixed-replace). 화면이 이 주소를 여는 동안만 카메라 토픽을 구독."""
    current_user(request)

    async def frames():
        ros.add_viewer()
        try:
            last = -1
            while not await request.is_disconnected():
                seq, data, ctype = ros.frame()
                if data is not None and seq != last:
                    last = seq
                    yield (b"--frame\r\nContent-Type: " + ctype.encode() +
                           b"\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                await asyncio.sleep(0.03)
        finally:
            ros.remove_viewer()

    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame",
                             headers={"Cache-Control": "no-store"})


def main():
    ap = argparse.ArgumentParser(description="SCV client page (제어 PC)")
    ap.add_argument("--robot-id", default=CFG["robot_id"])
    ap.add_argument("--server", default=CFG["server"])
    ap.add_argument("--camera-topic", default=CFG["camera_topic"])
    ap.add_argument("--host", default=CFG["host"])
    ap.add_argument("--port", type=int, default=CFG["port"])
    ap.add_argument("--check-s", type=float, default=CFG["check_s"])
    ap.add_argument("--outbox", default=CFG["outbox"], help="아웃박스 폴더 (기본 ./outbox_{robot_id})")
    a, _ros_args = ap.parse_known_args()
    CFG.update(robot_id=a.robot_id, server=a.server.rstrip("/"), camera_topic=a.camera_topic,
               host=a.host, port=a.port, check_s=a.check_s, outbox=a.outbox)
    print(f"client page — 로봇 {CFG['robot_id']} · 관제 PC {CFG['server']} · http://{CFG['host']}:{CFG['port']}/client/login",
          flush=True)
    uvicorn.run(app, host=CFG["host"], port=CFG["port"], workers=1)


if __name__ == "__main__":
    main()

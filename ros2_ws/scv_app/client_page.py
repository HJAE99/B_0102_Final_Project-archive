#!/usr/bin/env python3
"""
SCV client page — 제어 PC (3-2차)

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

  구성 (구현방안 A-2 결정 — 제어 PC 안 역할은 이 프로그램의 모듈, 서로는 함수 호출, 로봇과는 ROS)
    RosBridge        로봇과의 ROS 통신 (구독 · 활성 확인)
    MissionExecutor  task 실행 담당 — 3-1차: 상태(status) 만들기
    ScvAgent         관제 PC와의 유일한 창구 — WebSocket hello · status (ROS 노드 아님)
    (survey_buffer · detection_alert는 해당 차수에서 추가)

  실행 (제어 PC, ROS 2 Jazzy):
    source /opt/ros/jazzy/setup.bash
    python3 client_page.py --server http://<관제PC>:8000            # http://<제어PC>:8080 → /client/login
  옵션 · 환경 변수
    --robot-id      SCV_ROBOT_ID       기본 robot3
    --server        SCV_SERVER_URL     기본 http://127.0.0.1:8000
    --camera-topic  SCV_CAMERA_TOPIC   기본 /{robot_id}/oakd/rgb/image_raw/compressed (짝인 로봇의 카메라)
    --host / --port SCV_CLIENT_HOST / SCV_CLIENT_PORT  기본 0.0.0.0 / 8080
    --check-s       SCV_ACTIVE_CHECK_S 비활성일 때 토픽 목록 확인 주기(초), 기본 2
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import secrets
import threading
import time
import traceback
import urllib.error
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from starlette.concurrency import run_in_threadpool
from std_msgs.msg import String
from websockets.asyncio.client import connect as ws_connect

try:                                   # TurtleBot4(Create 3) 메시지 — 없으면 도킹 정보만 생략
    from irobot_create_msgs.msg import DockStatus
except ImportError:                    # pragma: no cover
    DockStatus = None

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
}
AGENT_VERSION = "3-2"
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
        if DockStatus is not None:
            self.info_subs.append(self.node.create_subscription(DockStatus, f"{ns}/dock_status", guarded(self._on_dock), q))
        self.log("로봇 정보 토픽 구독 — battery_state · odom · amcl_pose · ip" + (" · dock_status" if DockStatus else ""))

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
# mission_executor — task 실행 담당. 3-1차: 1초마다 상태(RobotStatus 필드) 만들기
#   run · task는 3-3차부터. 지금은 run_id · task_id · progress가 비어 있음
# ---------------------------------------------------------------------------
class MissionExecutor:
    def __init__(self, ros: RosBridge):
        self.ros = ros
        self.run_id: int | None = None
        self.task_id: str | None = None
        self.progress: float | None = None
        self.estop = False

    def status(self) -> dict[str, Any]:
        pose = self.ros.latest("pose")
        bat = self.ros.latest("battery")
        dock = self.ros.latest("dock")
        docked = bool(dock and dock["is_docked"])
        nav = "STOPPED" if self.estop else ("DOCKED" if docked else "IDLE")
        return {
            "stamp": utc_now_iso(), "robot_id": self.ros.robot_id,
            "run_id": self.run_id, "task_id": self.task_id, "nav_state": nav, "progress": self.progress,
            "x": pose["x"] if pose else None, "y": pose["y"] if pose else None, "yaw": pose["yaw"] if pose else None,
            "battery_pct": bat["percentage"] if bat else None, "is_docked": docked, "estop": self.estop,
        }


# ---------------------------------------------------------------------------
# scv_agent — 관제 PC와의 유일한 창구 (ROS 노드 아님). 로봇이 활성이 되면 WebSocket /ws/agent에 접속해
#   hello 한 번 → 이후 1초마다 status. 로봇이 꺼지면(값 끊김) 연결을 닫아 관제 PC도 비활성
# ---------------------------------------------------------------------------
class ScvAgent:
    STATUS_PERIOD_S = 1.0

    def __init__(self, ros: RosBridge, executor: MissionExecutor):
        self.ros = ros
        self.executor = executor
        self.status_sent = 0
        self.status_at: float | None = None
        self.state = "waiting"         # waiting(로봇 비활성) · connecting · connected · retry
        self.detail = "로봇 활성 확인 전"
        self.hello_at: float | None = None
        self.welcome: dict[str, Any] | None = None
        self.config: dict[str, Any] | None = None      # 관제 PC config — 현재 지도 · 이 로봇의 도크 접근 자세 (3-2차)
        self.config_at: float | None = None
        self.connects = 0

    def ws_url(self) -> str:
        s = CFG["server"]
        return ("wss://" + s[8:] if s.startswith("https://") else "ws://" + s.split("://", 1)[-1]) + "/ws/agent"

    async def run(self):
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
                    hello = {"type": "hello", "msg_id": f"c-{self.connects + 1:06d}", "ts": utc_now_iso(), "body": {
                        "robot_id": self.ros.robot_id, "agent_version": AGENT_VERSION,
                        "run_id": None, "active_task_id": None, "outbox_pending": 0}}
                    await ws.send(json.dumps(hello))
                    self.hello_at = time.time()
                    reply = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
                    if reply.get("type") != "welcome":
                        raise RuntimeError(f"관제 PC 응답: {reply.get('type')} {reply.get('body')}")
                    self.welcome = reply
                    self.connects += 1
                    self.state, self.detail = "connected", "hello 보냄 — 관제 PC에서 활성"
                    backoff = 1.0
                    print(f"[agent] 관제 PC 연결 · hello 보냄 ({self.ros.robot_id})", flush=True)
                    next_status = 0.0
                    while self.ros.active:             # 명령 처리는 다음 차수 — 지금은 받기만 + status 1 Hz
                        now = time.monotonic()
                        if now >= next_status:
                            next_status = now + self.STATUS_PERIOD_S
                            body = self.executor.status()
                            await ws.send(json.dumps({"type": "status", "msg_id": f"c-s{self.status_sent + 1:08d}",
                                                      "ts": body["stamp"], "body": body}))
                            self.status_sent += 1
                            self.status_at = time.time()
                            self.detail = f"hello 보냄 · status {self.status_sent}건 보냄 (1초마다)"
                        try:
                            raw = await asyncio.wait_for(ws.recv(), timeout=max(0.05, next_status - time.monotonic()))
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
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 15.0)

    def on_message(self, msg: dict[str, Any]):
        kind = msg.get("type")
        if kind == "config":
            self.config, self.config_at = msg.get("body") or {}, time.time()
            dock = self.config.get("dock_pose")
            mp = self.config.get("map") or {}
            print(f"[agent] config 받음 — 지도 {mp.get('name', '없음')} · 도크 접근 자세 {dock}", flush=True)
            if not dock:
                return
            # 로봇이 도크에 있거나(dock_status) 아직 위치를 모를 때만 시작 위치를 지정 — 순찰 중 위치를 덮어쓰지 않음
            d = self.ros.latest("dock")                 # 최근 STALE_S 안의 값만 (없으면 None)
            p = self.ros.latest("pose")
            docked = bool(d and d.get("is_docked"))
            if docked or p is None:
                self.ros.set_initial_pose(dock, "도킹 중 · 관제 PC 도크 접근 자세" if docked else "위치 모름 · 관제 PC 도크 접근 자세")
        else:
            print(f"[agent] ← {kind} (아직 처리하지 않는 메시지)", flush=True)

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
SESSIONS: dict[str, dict[str, Any]] = {}


@asynccontextmanager
async def lifespan(_app: FastAPI):
    global ros, executor, link
    camera_topic = CFG["camera_topic"] or f"/{CFG['robot_id']}/oakd/rgb/image_raw/compressed"
    ros = RosBridge(CFG["robot_id"], camera_topic, CFG["check_s"])
    executor = MissionExecutor(ros)
    link = ScvAgent(ros, executor)
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
    return {**ros.snapshot(), "link": link.status(), "status": executor.status(), "initialpose": ros.initialpose}


@app.get("/api/camera")
def camera(request: Request):
    current_user(request)
    return ros.camera_status()


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
    a, _ros_args = ap.parse_known_args()
    CFG.update(robot_id=a.robot_id, server=a.server.rstrip("/"), camera_topic=a.camera_topic,
               host=a.host, port=a.port, check_s=a.check_s)
    print(f"client page — 로봇 {CFG['robot_id']} · 관제 PC {CFG['server']} · http://{CFG['host']}:{CFG['port']}/client/login",
          flush=True)
    uvicorn.run(app, host=CFG["host"], port=CFG["port"], workers=1)


if __name__ == "__main__":
    main()

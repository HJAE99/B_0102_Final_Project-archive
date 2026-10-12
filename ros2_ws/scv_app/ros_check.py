#!/usr/bin/env python3
"""
ROS 통신 점검 — client_page.py와 같은 방식(토픽 목록 + best effort 구독)으로 로봇 토픽이 보이고 값이 오는지 확인

  client_page.py를 실행하는 것과 같은 터미널(같은 환경 변수)에서 실행:
    source /opt/ros/jazzy/setup.bash
    python3 ros_check.py               # 기본 robot3, 5초
    python3 ros_check.py --robot-id robot3 --seconds 8
"""
from __future__ import annotations

import argparse
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String

try:
    from irobot_create_msgs.msg import DockStatus
except ImportError:
    DockStatus = None

ENV_KEYS = ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT", "FASTRTPS_DEFAULT_PROFILES_FILE",
            "FASTDDS_BUILTIN_TRANSPORTS", "CYCLONEDDS_URI", "VIRTUAL_ENV")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", default="robot3")
    ap.add_argument("--seconds", type=float, default=5.0)
    a, _ = ap.parse_known_args()
    rid = a.robot_id

    print("== 환경 변수 (로봇을 실행한 터미널과 같아야 함)")
    for k in ENV_KEYS:
        print(f"  {k:<32}{os.environ.get(k, '(없음)')}")
    import rclpy as _r
    print(f"  rclpy 위치                        {os.path.dirname(_r.__file__)}")

    rclpy.init()
    node = Node("scv_ros_check")
    counts: dict[str, int] = {}
    topics = {f"/{rid}/battery_state": BatteryState, f"/{rid}/odom": Odometry, f"/{rid}/ip": String,
              f"/{rid}/oakd/rgb/image_raw/compressed": CompressedImage}
    if DockStatus is not None:
        topics[f"/{rid}/dock_status"] = DockStatus
    for t, typ in topics.items():
        counts[t] = 0
        node.create_subscription(typ, t, lambda _m, t=t: counts.__setitem__(t, counts[t] + 1), qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    time.sleep(2.0)                                     # 발견(discovery) 대기
    print(f"\n== 토픽 목록 — /{rid}/… (발행자 수)")
    listed = [(n, ty) for n, ty in node.get_topic_names_and_types() if n.startswith(f"/{rid}/")]
    for n, ty in listed:
        print(f"  {node.count_publishers(n)}  {n}  {','.join(ty)}")
    if not listed:
        print("  (없음)")
    print(f"\n== {a.seconds:g}초 동안 받은 값 (best effort 구독)")
    time.sleep(a.seconds)
    for t, n in counts.items():
        print(f"  {n:>5}개  {t}")

    pubs = sum(1 for n, _ in listed if node.count_publishers(n) > 0)
    got = sum(1 for n in counts.values() if n > 0)
    print("\n== 판단")
    if pubs == 0:
        print("  로봇 토픽의 발행자가 보이지 않습니다 → 로봇과 이 터미널의 ROS 환경이 다릅니다"
              " (ROS_DOMAIN_ID · ROS_LOCALHOST_ONLY/ROS_AUTOMATIC_DISCOVERY_RANGE · ROS_DISCOVERY_SERVER · RMW를 비교)")
    elif got == 0:
        print("  토픽은 보이지만 값이 하나도 오지 않습니다 → 발견은 되고 데이터 전달이 막힘"
              " (같은 PC면 공유 메모리 문제: 양쪽에 export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 후 다시 실행)")
    else:
        print("  토픽도 보이고 값도 옵니다 → ROS 통신은 정상. client_page 터미널 로그를 보내 주세요")
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Bounded, read-only ROS telemetry cache. No database or motion calls."""
import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from urllib.parse import urlencode

STATES = ('대기', '작업 중', '통로 막힘', '복귀·도킹 중', '도킹 완료', '비상정지', '오류')


class Telemetry:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.status = {}
        self.batteries = {}
        self.poses = {}
        self.last_valid = {}
        self.paths = {}
        self.alerts = deque(maxlen=100)
        self.inspections = {}
        self.error = None
        self.ready = False

    def on_pose(self, robot_id):
        with self.lock:
            self.poses[robot_id] = self.clock()

    def on_battery(self, robot_id, percentage):
        if not math.isfinite(percentage) or not 0 <= percentage <= 1:
            return
        with self.lock:
            self.batteries[robot_id] = (percentage * 100, self.clock())

    def on_status(self, robot_id, data):
        now = self.clock()
        data = dict(data)
        if data.get('progress') is not None and not math.isfinite(data['progress']):
            data['progress'] = None
        data.update(received_at=datetime.now(timezone.utc).isoformat())
        with self.lock:
            self.status[robot_id] = (data, now)
            # RobotStatus stamps are publication times; AMCL arrival tracks actual pose freshness.
            if (data['pose_ok'] and now - self.poses.get(robot_id, -math.inf) < 5
                    and all(math.isfinite(data[k]) for k in ('x', 'y', 'yaw'))):
                self.last_valid[robot_id] = {k: data[k] for k in ('x', 'y', 'yaw', 'received_at')}

    def on_alert(self, data):
        with self.lock:
            self.alerts.append(dict(data, received_at=datetime.now(timezone.utc).isoformat()))

    def on_path(self, robot_id, frame_id, points):
        if frame_id not in ("map", robot_id + "/map"):
            return
        finite = [p for p in points if all(math.isfinite(p[k]) for k in ("x", "y"))]
        if len(finite) < 2:
            return
        # Bound browser payload; this is a drawing simplification, never a navigation command.
        stride = max(1, math.ceil(len(finite) / 500))
        drawn = finite[::stride]
        if drawn[-1] != finite[-1]:
            drawn.append(finite[-1])
        with self.lock:
            status = self.status.get(robot_id)
            self.paths[robot_id] = dict(robot_id=robot_id, run_id=status[0].get("run_id") if status else None,
                frame_id=frame_id, points=drawn, rx=self.clock(), source="Nav2 global plan")

    def on_inspection(self, data):
        with self.lock:
            key = (data['run_id'], data['robot_id'], data['facility_id'])
            self.inspections[key] = dict(data, version=str(time.monotonic_ns()))
            if len(self.inspections) > 20:
                del self.inspections[next(iter(self.inspections))]

    def inspection_photo(self, run_id, robot_id, facility_id):
        with self.lock:
            return self.inspections.get((run_id, robot_id, facility_id), {}).get("photo")

    def snapshot(self, robot_ids, timeout=5):
        now = self.clock()
        out = []
        with self.lock:
            for rid in robot_ids:
                st, rx = self.status.get(rid, (None, -math.inf))
                fresh = now - rx < timeout
                data = dict(st) if st else None
                if data:
                    pose_ok = (fresh and data['pose_ok'] and now - self.poses.get(rid, -math.inf) < timeout
                               and all(math.isfinite(data[k]) for k in ('x', 'y', 'yaw')))
                    battery, brx = self.batteries.get(rid, (None, -math.inf))
                    data.update(pose_ok=pose_ok, battery_pct=battery if now - brx < timeout else None,
                                battery_source='BatteryState', age_s=round(now - rx, 1), ts=data['received_at'])
                    if not pose_ok:
                        data.update(x=None, y=None, yaw=None)
                    data['last_valid_pose'] = self.last_valid.get(rid)
                out.append({'robot_id': rid, 'connected': fresh, 'is_active': fresh, 'state': data})
            return {'robots': out, 'alerts': list(self.alerts), 'inspections': [dict({k: v for k, v in i.items() if k != 'photo'},
                        photo_url='/api/telemetry/inspection-image?' + urlencode({k:i[k] for k in ('run_id','robot_id','facility_id','version')}) if i.get('photo') else None)
                        for i in self.inspections.values()],
                    'paths': [{k: v for k, v in p.items() if k != 'rx'} for p in self.paths.values() if now - p['rx'] < timeout],
                    'bridge': {'ready': self.ready, 'error': self.error, 'source': 'ROS subscriptions'}}

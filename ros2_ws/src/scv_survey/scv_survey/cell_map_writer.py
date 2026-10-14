"""cell_map_writer (관제 PC, 묶음 시험용 임시 · T3-6): {ns}/survey/sample 을 칸으로 모아 칸 맵 저장소에 쓴다.

- network_map_engine 과 같은 SurveyGrid·cell_status 를 써서 판정 기준이 같다 (히트맵·RViz 는 network_map_engine 이 맡는다)
- ~/commit_<zone> (Trigger): 그 zone polygon 안의 칸을 저장소에 쓴다 (관제가 그 섹터 1차 측정이 끝났을 때 부른다).
  확정한 zone 의 이후 샘플은 버린다 — 재측정으로 서 있는 동안의 샘플이 관제가 쓴 재측정 값을 덮지 않게
- ~/reset (Trigger): 샘플과 확정 표시를 비운다 (관제가 회차를 시작할 때 부른다)
"""

from pathlib import Path
from functools import partial
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_srvs.srv import Trigger

from scv_msgs.msg import Alert, NetSample, RobotStatus
from scv_msgs.srv import SyncSamples
from scv_survey.cellmap_store import SampleRunMismatch, in_polygon, open_store, read_map_yaml, rows_from_stats
from scv_survey.heatmap import MISS, SHADOW, SHADOW_DBM, Sample, SurveyGrid, cell_center
from scv_survey.manual_tagger import SAMPLE_QOS
from scv_survey.sample_sync import SampleSync


class CellMapWriter(Node):
    def __init__(self):
        super().__init__('cell_map_writer')
        bringup = Path(get_package_share_directory('scv_bringup'))
        topics = self.declare_parameter('sample_topics', ['/robot4/survey/sample', '/robot3/survey/sample']).value
        cell = self.declare_parameter('cell_size', 0.5).value
        self.shadow_dbm = float(self.declare_parameter('shadow_dbm', float(SHADOW_DBM)).value)
        self.miss_thresh = self.declare_parameter('miss_ratio_thresh', 0.5).value
        self.ap_ssid = self.declare_parameter('ap_ssid', '').value
        self.backend = self.declare_parameter('backend', 'file').value
        self.map_yaml = self.declare_parameter('map_yaml', str(bringup / 'maps' / 'testbed_v1.yaml')).value
        self.store_args = (self.backend, self.map_yaml, cell,
                           self.declare_parameter('out_dir', '~/scv_cellmap').value,
                           self.declare_parameter('db_dsn_env', 'SCV_DB_DSN').value, self.ap_ssid)
        self.alert_pub = self.create_publisher(Alert, '/inspection/alerts', 10)
        self.store = None
        self.db_error = False
        self.last_db_attempt = 0.0
        self.active_run = 0
        zones_file = self.declare_parameter('zones_file', str(bringup / 'config' / 'zones.yaml')).value
        with open(zones_file, encoding='utf-8') as f:
            self.zones = {str(z['id']): z['polygon'] for z in yaml.safe_load(f)['zones']}

        self.grid = SurveyGrid(cell_size=cell, origin=read_map_yaml(self.map_yaml).origin)
        self.committed = set()
        self.sample_queue = {}
        self.rejected_runs = set()
        self.sync_states = {}
        self.commit_waiters = []
        self.sync_interval = float(self.declare_parameter('sync_interval_sec', 3.0).value)
        self.commit_wait = float(self.declare_parameter('commit_wait_sec', 8.0).value)
        self.max_queue = int(self.declare_parameter('max_sample_queue', 100000).value)
        if self.sync_interval <= 0 or self.commit_wait <= 0 or self.max_queue <= 0:
            raise ValueError('sync_interval_sec·commit_wait_sec·max_sample_queue는 양수여야 합니다')
        robots = self.declare_parameter('robot_ids', ['robot4', 'robot3']).value
        zone_ids = self.declare_parameter('zone_ids', ['Z1', 'Z2']).value
        if len(robots) != len(zone_ids):
            raise ValueError('robot_ids와 zone_ids 수가 다릅니다')
        self.robot_zones = dict(zip(robots, zone_ids))
        self.sync_clients = {robot: self.create_client(SyncSamples, f'/{robot}/survey/sync')
                             for robot in robots} if self.backend == 'db' else {}
        self.commit_group = ReentrantCallbackGroup()
        for zone in self.zones:
            self.create_service(Trigger, f'~/commit_{zone}', partial(self.on_commit, zone),
                                callback_group=self.commit_group)
        self.create_service(Trigger, '~/reset', self.on_reset)
        for t in topics:
            self.create_subscription(NetSample, t, self.on_sample, SAMPLE_QOS)
        for robot in self.sync_clients:
            self.create_subscription(RobotStatus, f'/{robot}/status', self.on_status, 10)
        self._connect()
        self.create_timer(1.0, self.flush_samples)
        self.create_timer(0.2, self.sync_tick)
        self.get_logger().info(f'구독 {list(topics)}, 칸 {cell} m, zone {list(self.zones)}, '
                               f'저장소 {type(self.store).__name__}')

    def alert(self, code, message, level=Alert.LEVEL_ERROR):
        msg = Alert(code=code, message=message, level=level)
        msg.stamp = self.get_clock().now().to_msg()
        self.alert_pub.publish(msg)
        self.get_logger().error(message) if level == Alert.LEVEL_ERROR else self.get_logger().info(message)

    def _connect(self):
        self.last_db_attempt = time.monotonic()
        try:
            self.store = open_store(*self.store_args)
            if self.backend == 'db':
                self.set_run(self.store.running_run())
            return True
        except (RuntimeError, ImportError) as exc:
            if self.backend != 'db':
                raise
            self.db_error = True
            self.store = None
            self.alert('DB_UNAVAILABLE', f'샘플 저장소 준비 실패: {exc}')
            return False

    def set_run(self, run_id, reset=False):
        if run_id == self.active_run and not reset:
            return
        # DB에 칸 통계가 있으면 이미 확정한 구역이다. 재시작해도 재측정 값을 보존한다.
        stored = self.store.read_cells(run_id) if run_id else {}
        self.committed = {self.zone_of(*cell_center(i, j, *self.grid.origin, self.grid.cell_size))
                          for i, j in stored} - {''}
        self.active_run = run_id
        self.grid.clear()
        for sample in self.sample_queue.values():
            if sample['run_id'] == run_id and self.zone_of(sample['x'], sample['y']) not in self.committed:
                self.grid.add(Sample(sample['robot_id'], sample['seq'], sample['x'], sample['y'],
                                     sample['pose_ok'], sample['detected'], sample['rssi_dbm']))

    def state(self, robot_id, run_id):
        return self.sync_states.setdefault((robot_id, run_id), SampleSync())

    def on_status(self, msg):
        if msg.robot_id in self.sync_clients and msg.run_id and msg.run_id not in self.rejected_runs:
            self.state(msg.robot_id, msg.run_id)

    def zone_of(self, x: float, y: float) -> str:
        return next((z for z, poly in self.zones.items() if in_polygon(x, y, poly)), '')

    def on_sample(self, m: NetSample):
        if self.backend == 'db':
            if not m.run_id or m.run_id in self.rejected_runs or m.robot_id not in self.sync_clients:
                return      # 준비 중 run 0을 임의의 DB 회차로 바꾸지 않는다.
            state = self.state(m.robot_id, m.run_id)
            key = m.robot_id, m.run_id, m.seq
            if key not in self.sample_queue and len(self.sample_queue) >= self.max_queue:
                state.snapshot_count = None
                self.alert('DB_UNAVAILABLE', '미저장 샘플 큐 한도 — 로봇 버퍼에서 전체 재복구가 필요합니다')
                return
            state.observe(m.seq, time.monotonic())
            self.sample_queue.setdefault(key, {
                'robot_id': m.robot_id, 'run_id': m.run_id, 'seq': m.seq,
                'stamp_sec': m.stamp.sec, 'stamp_nanosec': m.stamp.nanosec,
                'x': m.x, 'y': m.y, 'yaw': m.yaw, 'pose_ok': m.pose_ok,
                'detected': m.detected, 'resent': m.resent, 'rssi_dbm': m.rssi_dbm})
            if m.run_id != self.active_run:
                return      # 과거 회차의 원본 복구는 저장만 하고 현재 칸 맵에 섞지 않는다.
        zone = self.zone_of(m.x, m.y)
        if zone in self.committed:
            self.get_logger().info(f'{zone} 은 확정됨 — 이후 샘플은 버림', once=True)
            return
        self.grid.add(Sample(m.robot_id, m.seq, m.x, m.y, m.pose_ok, m.detected, m.rssi_dbm))

    async def on_commit(self, zone: str, req, res):
        if self.backend == 'db':
            if self.store is None:
                res.success, res.message = False, 'DB_UNAVAILABLE: 샘플 저장소 준비 대기'
                return res
            try:
                run_id = self.store.running_run()
            except RuntimeError as exc:
                res.success, res.message = False, str(exc)
                self.alert('DB_UNAVAILABLE', res.message)
                return res
            if not run_id:
                res.success, res.message = False, '확정할 진행 중 회차가 없습니다'
                return res
            now = time.monotonic()
            targets = [(r, run_id) for r, z in self.robot_zones.items() if z == zone]
            if not targets:
                res.success, res.message = False, '이 구역에 배정된 수신 로봇이 없습니다'
                return res
            for robot, run in targets:
                state = self.state(robot, run)
                pending, state.future = state.future, None
                if pending is not None:
                    pending.cancel()
                state.snapshot_count, state.last_request = None, None
            future = Future()
            self.commit_waiters.append((zone, run_id, targets, now, res, future))
            return await future     # 재진입 그룹: 다른 콜백이 sync·샘플 저장을 진행한다.
        return self.write_zone(zone, res)

    def write_zone(self, zone, res, expected_run=None):
        try:
            run_id = self.store.running_run()
            if expected_run is not None and run_id != expected_run:
                raise RuntimeError('확정 대기 중 회차가 바뀌었습니다 — 다른 회차에 쓰지 않습니다')
            if self.backend == 'db' and zone in self.committed:
                res.success, res.message = True, f'{zone}: 기존 확정·재측정 값 유지, run_id={run_id}'
                return res
            rows = [r for r in rows_from_stats(self.grid.cells(), self.grid.origin, self.grid.cell_size,
                                               self.shadow_dbm, self.miss_thresh)
                    if self.zone_of(r['cx'], r['cy']) == zone]
            n = self.store.write_cells(run_id, zone, rows, self.ap_ssid)
        except RuntimeError as e:
            res.success, res.message = False, str(e)
            self.alert('DB_UNAVAILABLE', res.message)
            return res
        self.committed.add(zone)
        count = lambda s: sum(1 for r in rows if r['state'] == s)
        res.success = True
        res.message = (f'{zone}: {n}칸 (음영 {count(SHADOW)} · 미검출 {count(MISS)} · '
                       f'low {sum(1 for r in rows if r["n"] < 2)}), run_id={run_id}')
        self.get_logger().info(res.message)
        return res

    def flush_samples(self):
        if self.backend != 'db':
            return
        if self.store is None:
            if time.monotonic() - self.last_db_attempt < 3.0 or not self._connect():
                return
        keys = list(self.sample_queue)[:1000]
        try:
            current = self.store.running_run()
            self.set_run(current)
            self.store.write_samples([self.sample_queue[k] for k in keys])
        except SampleRunMismatch as exc:
            # 다른 DB/지도의 과거 원본은 로봇 CSV에 남는다. 정상 회차의 저장을 막지 않는다.
            self.rejected_runs.update(exc.run_ids)
            for key in list(self.sample_queue):
                if key[1] in exc.run_ids:
                    self.sample_queue.pop(key)
            for key in list(self.sync_states):
                if key[1] in exc.run_ids:
                    state = self.sync_states.pop(key)
                    if state.future is not None:
                        pending, state.future = state.future, None
                        pending.cancel()
            self.alert('SAMPLE_RUN_MISMATCH', f'{exc}; 로봇 CSV 보존, DB 환경 확인 필요', Alert.LEVEL_WARN)
            return
        except RuntimeError as exc:
            if not self.db_error:
                self.alert('DB_UNAVAILABLE', f'원본 저장 실패 — {len(self.sample_queue)}개 큐 유지: {exc}')
            self.db_error = True
            return
        for key in keys:
            self.sample_queue.pop(key)
        if self.db_error:
            self.alert('DB_RECOVERED', 'DB 저장 연결이 복구됐습니다', Alert.LEVEL_INFO)
        self.db_error = False

    def sync_tick(self):
        if self.backend != 'db':
            return
        now = time.monotonic()
        for (robot, run_id), state in list(self.sync_states.items()):
            if state.future is not None:
                if state.last_request is not None and now - state.last_request > 10:
                    pending, state.future = state.future, None
                    pending.cancel()
                    state.snapshot_count = None
                else:
                    continue
            if state.last_request is not None and now - state.last_request < self.sync_interval:
                continue
            client = self.sync_clients[robot]
            if not client.service_is_ready():
                continue
            start = state.from_seq()
            prefix = state.snapshot_count if start else 0
            future = client.call_async(SyncSamples.Request(run_id=run_id, from_seq=start))
            state.future, state.last_request = future, now
            future.add_done_callback(partial(self.sync_result, state, prefix))
        for waiter in list(self.commit_waiters):
            zone, run, targets, started, response, future = waiter
            ready = all(self.state(*key).complete() for key in targets)
            queued = any(k[1] == run and self.robot_zones.get(k[0]) == zone for k in self.sample_queue)
            if ready and not queued:
                self.write_zone(zone, response, expected_run=run)
            elif now - started >= self.commit_wait:
                response.success, response.message = False, 'DB_UNAVAILABLE: 원본 저장·sync 복구가 끝나지 않아 확정 보류'
                self.alert('DB_UNAVAILABLE', response.message)
            else:
                continue
            self.commit_waiters.remove(waiter)
            future.set_result(response)

    def sync_result(self, state, prefix, future):
        if state.future is not future:
            return
        state.future = None
        try:
            result = future.result()
            if result is not None and result.ok:
                state.snapshot(result.last_seq, (prefix or 0) + result.count)
            else:
                state.snapshot_count = None
        except Exception as exc:
            state.snapshot_count = None
            self.get_logger().warning(f'sync 응답 실패: {type(exc).__name__}')

    def on_reset(self, req, res):
        self.grid.clear()
        self.committed.clear()
        if self.backend == 'db' and self.store is not None:
            try:
                self.set_run(self.store.running_run(), reset=True)
            except RuntimeError:
                self.active_run = 0
        res.success, res.message = True, '샘플과 확정 표시를 비움'
        self.get_logger().warn(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = CellMapWriter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

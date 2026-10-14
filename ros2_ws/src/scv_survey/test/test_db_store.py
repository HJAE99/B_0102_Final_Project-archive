"""임시 PostgreSQL의 독립 스키마에서만 실행. SCV_TEST_DB_DSN이 없으면 건너뛴다."""

import os
from pathlib import Path
import uuid

import pytest

from scv_survey.cellmap_store import DbStore, FileStore, free_cells, read_map_yaml, rows_from_stats
from scv_survey.heatmap import CellStats
from scv_survey.sample_sync import SampleSync


def sequences(store, robot_id, run_id):
    return {r[0] for r in store._q('SELECT seq FROM net_samples WHERE robot_id=%s AND run_id=%s',
                                  (robot_id, run_id))}


def test_sync_restores_leading_and_middle_gaps_with_global_seq():
    sync = SampleSync()
    sync.observe(103, 1)
    sync.snapshot(103, 4)
    assert not sync.complete() and sync.from_seq() == 0
    for seq in (100, 102, 102):
        sync.observe(seq, 2)
    assert not sync.complete()
    sync.observe(101, 2)
    assert sync.complete() and sync.from_seq() == 104
    sync.observe(105, 3)
    assert sync.from_seq() == 0
    sync.snapshot(105, 6)
    sync.observe(104, 3)
    assert sync.complete()
    sync.observe(106, 7)
    assert sync.from_seq() == 0


def test_commit_cancels_unanswered_sync_request():
    from types import SimpleNamespace
    from rclpy.task import Future
    from scv_survey.cell_map_writer import CellMapWriter
    state = SampleSync()
    pending = Future()
    state.future, state.last_request = pending, 10.0
    writer = SimpleNamespace(backend='db', store=SimpleNamespace(running_run=lambda: 1),
                             robot_zones={'robot3': 'Z1'}, state=lambda *args: state,
                             commit_waiters=[])
    response = SimpleNamespace(success=False, message='')
    call = CellMapWriter.on_commit(writer, 'Z1', None, response)
    try:
        call.send(None)
        assert pending.cancelled(), '기존 응답 대기 요청이 남으면 새 sync가 막힘'
        assert state.future is None
    finally:
        call.close()


@pytest.fixture
def db_case(tmp_path, monkeypatch):
    dsn = os.environ.get('SCV_TEST_DB_DSN')
    if not dsn:
        pytest.skip('SCV_TEST_DB_DSN: 시험 DB에서만 실행')
    driver = pytest.importorskip('psycopg')
    from psycopg.conninfo import conninfo_to_dict, make_conninfo
    from psycopg import sql
    if not conninfo_to_dict(dsn).get('dbname', '').endswith('_test'):
        pytest.fail('운영 DB 보호: DB 이름은 _test로 끝나야 합니다')
    schema = 'scv_test_' + uuid.uuid4().hex
    with driver.connect(dsn) as conn:
        conn.execute(sql.SQL('CREATE SCHEMA {}').format(sql.Identifier(schema)))
    isolated = make_conninfo(dsn, options=f'-c search_path={schema},public')
    root = Path(__file__).resolve().parents[3]
    with driver.connect(isolated) as conn:
        conn.execute((root / 'scv_db/sql/01_schema.sql').read_text())
        conn.execute("INSERT INTO robots (robot_id) VALUES ('robot3'), ('robot4')")
    monkeypatch.setenv('SCV_DB_DSN', isolated)
    (tmp_path / 'm.pgm').write_bytes(b'P5\n4 4\n255\n' + bytes([255] * 16))
    map_yaml = tmp_path / 'm.yaml'
    map_yaml.write_text('image: m.pgm\nresolution: 0.25\norigin: [1.0, 2.0, 0]\n')

    def register(path):
        info = read_map_yaml(path)
        with driver.connect(isolated) as conn:
            conn.execute('UPDATE maps SET is_active = false WHERE is_active')
            map_id = conn.execute(
                'INSERT INTO maps (name, robot_map_path, image_key, resolution_m, origin_x, origin_y, grid_size_m, is_active) '
                "VALUES ('db-test', %s, 'test.png', %s, %s, %s, 0.5, true) RETURNING map_id",
                (str(path), info.resolution, *info.origin)).fetchone()[0]
            with conn.cursor() as cur:
                cur.executemany('INSERT INTO grid_cells (map_id, ix, iy, center_x, center_y) VALUES (%s,%s,%s,%s,%s)',
                                [(map_id, *cell) for cell in free_cells(info, 0.5)])
        return map_id

    register(map_yaml)
    yield DbStore(map_yaml, ap_ssid='db-test-ap'), map_yaml, isolated, register
    # 시험 스키마를 남겨 실패 시 SQL 상태를 확인할 수 있게 한다. 운영 표는 손대지 않는다.


def raw_sample(run_id, seq, **changes):
    sample = dict(robot_id='robot3', run_id=run_id, seq=seq, stamp_sec=100 + seq,
                stamp_nanosec=123456789, x=1.25, y=2.25, yaw=0.4, pose_ok=True,
                detected=True, resent=False, rssi_dbm=-60)
    sample.update(changes)
    return sample


def test_real_db_runs_cells_and_raw_idempotency(db_case):
    store, map_yaml, _dsn, _register = db_case
    run = store.open_run({'1': 'robot3'})
    with pytest.raises(RuntimeError, match='running'):
        store.open_run({'1': 'robot3'})
    file = FileStore(map_yaml, out_dir=map_yaml.parent / 'file')
    file_run = file.open_run({'1': 'robot3'})
    rows = rows_from_stats({(0, 0): CellStats([-50, -52]), (1, 0): CellStats([-80, -79]),
                            (0, 1): CellStats([], 2)}, store.map.origin, 0.5)
    assert store.write_cells(run, 'Z1', rows) == file.write_cells(file_run, 'Z1', rows) == 3
    assert store.read_cells(run) == file.read_cells(file_run)
    assert store.update_cell(run, 1, 0, 'normal', -60, 30)
    assert store.read_cells(run)[(1, 0)].median == -60
    assert store.last_seq('robot3', run) is None
    originals = [raw_sample(run, 100), raw_sample(run, 102)]
    originals[1].update(pose_ok=False, detected=False, rssi_dbm=0)
    assert store.write_samples(originals) == 2
    assert store.write_samples(originals) == 0
    assert store.last_seq('robot3', run) == 102
    assert sequences(store, 'robot3', run) == {100, 102}
    record = store._q('SELECT detected, cell_id, task_id, stamp_sec, stamp_nanosec '
                      'FROM net_samples WHERE seq = 102')[0]
    assert record == (False, None, None, 202, 123456789)
    store.write_samples([raw_sample(run, 101)])
    assert sequences(store, 'robot3', run) == {100, 101, 102}
    store.close_run(run, 'done')
    assert store.running_run() == 0
    run2 = store.open_run({'1': 'robot3'})
    store.close_run(run2, 'partial')
    assert store._q('SELECT status FROM runs ORDER BY run_id') == [('done',), ('partial',)]
    with pytest.raises(RuntimeError, match='지도'):
        store.write_samples([raw_sample(0, 104)])


def test_real_db_rejects_grid_mismatch_and_offline(db_case, monkeypatch):
    store, map_yaml, _dsn, _register = db_case
    store._q('DELETE FROM grid_cells WHERE ix = 0 AND iy = 0')  # 이 시험의 빈 격자만
    with pytest.raises(RuntimeError, match='격자'):
        DbStore(map_yaml, ap_ssid='db-test-ap')
    monkeypatch.setenv('SCV_DB_DSN', 'host=127.0.0.1 port=15433 dbname=unused_test user=unused')
    with pytest.raises(RuntimeError, match='접속'):
        DbStore(map_yaml, ap_ssid='db-test-ap')


def test_old_unknown_run_does_not_block_current_batch(db_case):
    from types import SimpleNamespace
    from scv_survey.cell_map_writer import CellMapWriter
    store, _map, _dsn, _register = db_case
    run = store.open_run({'1': 'robot3'})
    alerts = []
    writer = SimpleNamespace(backend='db', store=store, db_error=False,
                             set_run=lambda current: None, rejected_runs=set(), sync_states={},
                             alert=lambda *args: alerts.append(args), sample_queue={
                                 ('robot3', 999999, 0): raw_sample(999999, 0),
                                 ('robot3', run, 1): raw_sample(run, 1)})
    CellMapWriter.flush_samples(writer)
    CellMapWriter.flush_samples(writer)
    assert store.last_seq('robot3', run) == 1
    assert alerts[0][0] == 'SAMPLE_RUN_MISMATCH'


def test_ros_auto_sync_db_retry_commit_and_writer_restart(db_case, tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    from scv_msgs.msg import RobotStatus
    from scv_survey.cell_map_writer import CellMapWriter
    from scv_survey.sample_buffer import BufferedSample
    from scv_survey.survey_buffer import SurveyBuffer, sample_message

    store, map_yaml, _dsn, _register = db_case
    run = store.open_run({'1': 'robot3'})
    zones = tmp_path / 'zones.yaml'
    zones.write_text('zones:\n  - id: Z1\n    polygon: [[1, 2], [2, 2], [2, 3], [1, 3]]\n')
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robot3',
                    '-p', f'buffer_csv_path:={tmp_path / "buffer.csv"}',
                    '-p', 'backend:=db', '-p', 'ap_ssid:=db-test-ap',
                    '-p', f'map_yaml:={map_yaml}', '-p', f'zones_file:={zones}',
                    '-p', 'robot_ids:=[robot3]', '-p', 'zone_ids:=[Z1]',
                    '-p', 'sample_topics:=[/robot3/survey/sample]',
                    '-p', 'sync_interval_sec:=0.3'], domain_id=220)
    buffer = SurveyBuffer()
    writer = CellMapWriter()
    peer = rclpy.create_node('db_test_peer')
    executor = SingleThreadedExecutor()
    for node in (buffer, writer, peer):
        executor.add_node(node)

    def wait(predicate, timeout=10):
        deadline = __import__('time').monotonic() + timeout
        while not predicate() and __import__('time').monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS/DB 시험 시간 초과'

    try:
        # run 0 때문에 작업 회차의 첫 번호가 1. 선두·중간을 모두 놓친 상태에서 시작.
        samples = [BufferedSample(seq, 100 + seq, 123456789, 'robot3', 0 if seq == 0 else run,
                                  1.25, 2.25, 0.4, True, True, -60) for seq in range(4)]
        for sample in samples:
            buffer.buffer.append(sample)
        buffer.seq = 4
        writer.on_sample(sample_message(samples[-1]))
        writer.on_status(RobotStatus(robot_id='robot3', run_id=run))
        wait(lambda: sequences(store, 'robot3', run) == {1, 2, 3})
        assert len(writer.grid.samples) == 3
        assert store._q('SELECT count(*) FROM net_samples')[0][0] == 3

        # DB 쓰기 실패 시 큐·원본을 유지. commit은 sync 응답만 받고 성공하면 안 됨.
        extra = BufferedSample(4, 104, 123456789, 'robot3', run, 1.25, 2.25, 0.4, True, False, 0)
        buffer.buffer.append(extra)
        writer.on_sample(sample_message(extra))
        original_dsn = writer.store.dsn
        # 단순 예외 mock 대신 psycopg가 실제로 접속 실패하도록 시험 포트를 바꾼다.
        monkeypatch.setattr(writer.store, 'dsn',
                            'host=127.0.0.1 port=15433 dbname=unused_test user=unused')
        writer.flush_samples()
        assert ('robot3', run, 4) in writer.sample_queue and writer.db_error
        client = peer.create_client(Trigger, '/robot3/cell_map_writer/commit_Z1')
        wait(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert not future.result().success
        monkeypatch.setattr(writer.store, 'dsn', original_dsn)
        future = client.call_async(Trigger.Request())
        wait(lambda: bool(writer.commit_waiters))
        assert not future.done()
        wait(future.done)
        assert future.result().success, future.result().message
        assert sequences(store, 'robot3', run) == {1, 2, 3, 4}
        assert store.read_cells(run)[(0, 0)].n == 4

        store.update_cell(run, 0, 0, 'normal', -50, 30)  # 확정 후 재측정으로 바뀐 값
        for timer in writer.timers:
            timer.cancel()
        wait(lambda: all(state.future is None for state in writer.sync_states.values()))
        executor.remove_node(writer)
        writer.destroy_node()
        writer = CellMapWriter()
        executor.add_node(writer)
        writer.on_status(RobotStatus(robot_id='robot3', run_id=run))
        wait(lambda: writer.state('robot3', run).complete() and not writer.sample_queue)
        assert writer.committed == {'Z1'}
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success
        assert store.read_cells(run)[(0, 0)].median == -50
        assert store.read_cells(run)[(0, 0)].n == 30
        assert store._q('SELECT count(*) FROM net_samples')[0][0] == 4
        assert len(buffer.buffer.samples) == 5  # run 0 한 줄은 DB에 넣지 않음
    finally:
        for timer in writer.timers:
            timer.cancel()
        wait(lambda: all(state.future is None for state in writer.sync_states.values()))
        executor.shutdown()
        writer.destroy_node()
        buffer.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()


def test_web_ros_owner_grid_generation_and_fresh_heatmap(db_case, tmp_path, monkeypatch):
    import importlib.util
    from PIL import Image
    pytest.importorskip('fastapi')
    from psycopg.rows import dict_row
    from fastapi import HTTPException
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from starlette.websockets import WebSocketDisconnect
    store, _map_yaml, dsn, register = db_case
    root = Path(__file__).resolve().parents[3]
    actual_map = root / 'src/scv_bringup/maps/testbed_v1.yaml'
    map_id = register(actual_map)
    monkeypatch.setenv('SCV_RUN_OWNER', 'ros')
    monkeypatch.setenv('SCV_MEDIA_DIR', str(tmp_path))
    spec = importlib.util.spec_from_file_location('db_test_web_server', root / 'scv_app/server_page.py')
    web = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(web)
    image = Image.open(actual_map.with_suffix('.pgm')).convert('L')
    image.save(tmp_path / 'test.png')
    with store.driver.connect(dsn, row_factory=dict_row) as conn:
        conn.execute('DELETE FROM grid_cells WHERE map_id = %s', (map_id,))
        m = conn.execute('SELECT * FROM maps WHERE map_id = %s', (map_id,)).fetchone()
        assert web.generate_cells(conn, m) == 98
    actual_store = DbStore(actual_map, ap_ssid='db-test-ap')
    assert len(actual_store.grid_cells()) == 98
    with pytest.raises(HTTPException, match='ROS'):
        web.start_run({}, {})
    with pytest.raises(HTTPException, match='ROS'):
        web.abort_run(1, 'test')
    client = TestClient(web.app)   # lifespan/운영 서비스 실행 없이 입력 차단만 확인
    assert client.post('/api/agent/samples', json={}).status_code == 409
    with pytest.raises(WebSocketDisconnect) as closed:
        with client.websocket_connect('/ws/agent') as ws:
            ws.receive_json()
    assert closed.value.code == 4409
    run = actual_store.open_run({'1': 'robot3'})
    ix, iy, x, y = actual_store.grid_cells()[0]
    with store.driver.connect(dsn, row_factory=dict_row) as conn:
        assert web.running_run(conn)['run_id'] == run
        assert web.CURRENT_RUN['run_id'] == run
        assert web.live_cells(conn, run) == {}
    actual_store.write_samples([raw_sample(run, 0, x=x, y=y)])
    actual_store.write_samples([raw_sample(run, 1, x=x, y=y, detected=False, rssi_dbm=0)])
    with store.driver.connect(dsn, row_factory=dict_row) as conn:
        live = web.live_cells(conn, run)
        assert next(iter(live.values()))[actual_store.ap_id][:2] == [1, -60]
    actual_store.write_samples([raw_sample(run, 2, x=x, y=y, rssi_dbm=-80)])
    with store.driver.connect(dsn, row_factory=dict_row) as conn:
        assert next(iter(web.live_cells(conn, run).values()))[actual_store.ap_id][:2] == [2, -140]
    actual_store.close_run(run, 'done')
    with store.driver.connect(dsn, row_factory=dict_row) as conn:
        assert web.running_run(conn) is None and web.CURRENT_RUN['run_id'] is None

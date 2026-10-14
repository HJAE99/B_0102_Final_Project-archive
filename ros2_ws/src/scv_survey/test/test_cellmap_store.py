from scv_survey.cellmap_store import FileStore, in_polygon, rows_from_stats
from scv_survey.heatmap import CellStats
from scv_survey.sample_buffer import BufferedSample, ReplayQueue, SampleBuffer

from dataclasses import replace

import pytest


def make_map(tmp_path):
    # 4 x 4 px, 0.25 m/px → 0.5 m 칸 2 x 2. 오른쪽 위 한 칸만 벽(검정)
    rows = [[255, 255, 0, 0], [255, 255, 0, 0], [255, 255, 255, 255], [255, 255, 255, 255]]
    (tmp_path / 'm.pgm').write_bytes(b'P5\n4 4\n255\n' + bytes(v for r in rows for v in r))
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.25\norigin: [1.0, 2.0, 0]\n')
    return tmp_path / 'm.yaml'


def test_file_store_round_trip(tmp_path):
    store = FileStore(make_map(tmp_path), 0.5, tmp_path / 'out')
    assert [(i, j) for i, j, _, _ in store.grid_cells()] == [(0, 0), (0, 1), (1, 0)]

    run_id = store.open_run({'1': 'robot4'})
    rows = rows_from_stats({(0, 0): CellStats([-50, -52]), (1, 0): CellStats([-80, -78, -79]),
                            (0, 1): CellStats([], 2)}, store.map.origin, 0.5)
    assert [r['state'] for r in rows] == ['normal', 'miss', 'shadow']
    assert store.write_cells(run_id, 'Z1', rows, 'ap') == 3

    cells = store.read_cells(run_id)
    assert cells[(1, 0)].state == 'shadow' and cells[(1, 0)].median == -79
    assert cells[(0, 1)].state == 'miss' and cells[(0, 1)].confidence == 'ok'

    assert store.update_cell(run_id, 1, 0, 'normal', -60.0, 30)
    assert store.read_cells(run_id)[(1, 0)].state == 'normal'
    assert store.open_run({'1': 'robot4'}) == run_id + 1
    assert store.read_cells(run_id + 1) == {}


def test_in_polygon():
    square = [[0, 0], [2, 0], [2, 2], [0, 2]]
    assert in_polygon(1, 1, square) and not in_polygon(3, 1, square)


def test_db_backend_needs_ap_ssid(tmp_path):
    import pytest
    from scv_survey.cellmap_store import open_store
    with pytest.raises(RuntimeError, match='ap_ssid'):
        open_store('db', make_map(tmp_path), 0.5)


def buffer_sample(seq=0, run_id=1, **kwargs):
    return replace(BufferedSample(seq, 100 + seq, 123456789, 'robot3', run_id,
                                  1.2, -0.3, 0.4, True, True, -60), **kwargs)


def test_buffer_restores_samples_and_global_seq_across_runs(tmp_path):
    path = tmp_path / 'robot3_buffer.csv'
    store = SampleBuffer(path, 'robot3')
    originals = [buffer_sample(), buffer_sample(1, 2, pose_ok=False, detected=False, rssi_dbm=0)]
    for sample in originals:
        store.append(sample)
    store.close()
    restored = SampleBuffer(path, 'robot3')
    assert restored.samples == originals
    assert restored.next_seq == 2
    assert originals[0].raw_key in restored.raw_keys
    restored.append(buffer_sample(2, 3))
    assert restored.next_seq == 3
    restored.close()


def test_buffer_sync_is_inclusive_and_scoped_to_run(tmp_path):
    store = SampleBuffer(tmp_path / 'buffer.csv', 'robot3')
    for seq, run in enumerate((0, 7, 7, 8)):
        store.append(buffer_sample(seq, run))
    assert [s.seq for s in store.select(7, 2)] == [2]
    assert [s.seq for s in store.select(7, 1)] == [1, 2]
    assert store.select(7, 3) == []
    assert store.select(99, 0) == []
    assert store.last_seq(7) == 2 and store.last_seq(99) is None
    store.close()


def test_buffer_rejects_two_writers_and_wrong_robot(tmp_path):
    path = tmp_path / 'buffer.csv'
    store = SampleBuffer(path, 'robot3')
    with pytest.raises(BlockingIOError):
        SampleBuffer(path, 'robot3')
    store.append(buffer_sample())
    store.close()
    with pytest.raises(ValueError, match='다른 로봇'):
        SampleBuffer(path, 'robot4')


def test_buffer_rejects_reused_seq_and_raw_stamp(tmp_path):
    store = SampleBuffer(tmp_path / 'buffer.csv', 'robot3')
    original = buffer_sample()
    store.append(original)
    with pytest.raises(ValueError, match='seq'):
        store.append(original)
    with pytest.raises(ValueError, match='이미 저장'):
        store.append(replace(original, seq=1))
    assert store.next_seq == 1 and len(store.samples) == 1
    store.close()


@pytest.mark.parametrize('corruption', ['header', 'partial', 'duplicate'])
def test_buffer_never_resets_seq_on_corrupt_csv(tmp_path, corruption):
    path = tmp_path / 'buffer.csv'
    store = SampleBuffer(path, 'robot3')
    store.append(buffer_sample())
    store.close()
    content = path.read_text()
    if corruption == 'header':
        content = content.replace('stamp_sec', 'stamp')
    elif corruption == 'partial':
        content += '1,101,123'
    else:
        content += content.splitlines(keepends=True)[1]
    path.write_text(content)
    with pytest.raises(ValueError):
        SampleBuffer(path, 'robot3')
    assert path.read_text() == content


def test_buffer_write_failure_does_not_advance_in_memory_seq(tmp_path, monkeypatch):
    store = SampleBuffer(tmp_path / 'buffer.csv', 'robot3')

    def fail(_fd):
        raise OSError('disk full')

    monkeypatch.setattr('scv_survey.sample_buffer.os.fsync', fail)
    with pytest.raises(OSError, match='disk full'):
        store.append(buffer_sample())
    assert store.next_seq == 0 and store.samples == []
    with pytest.raises(OSError, match='이전 CSV 쓰기'):
        store.append(buffer_sample())
    store.close()


def test_replay_overlap_and_rate_limit():
    replay = ReplayQueue(100)     # 요청 값이 커도 20 Hz 이하
    samples = [buffer_sample(i) for i in range(3)]
    assert replay.schedule(samples) == 3
    assert replay.schedule(samples[1:]) == 0
    assert replay.pop_ready(0.0) == samples[0]
    assert replay.pop_ready(0.049) is None
    assert replay.pop_ready(0.051) == samples[1]
    assert replay.pop_ready(0.099) is None
    assert replay.pop_ready(0.102) == samples[2]
    assert replay.pop_ready(1.0) is None
    # 응답을 못 받은 관제는 완료된 범위를 다시 요청할 수 있다.
    assert replay.schedule(samples) == 3


@pytest.mark.parametrize('rate', [0, -1, float('nan'), float('inf')])
def test_replay_rejects_invalid_rate(rate):
    with pytest.raises(ValueError):
        ReplayQueue(rate)

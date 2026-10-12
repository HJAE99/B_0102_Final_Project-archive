from scv_survey.cellmap_store import FileStore, in_polygon, rows_from_stats
from scv_survey.heatmap import CellStats


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

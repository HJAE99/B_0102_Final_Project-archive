"""묶음 시험에서 더한 순수 로직: 벽 여유 경로, 설비 섹터 배정, 재측정 칸 선택, 회피 칸."""

from pathlib import Path
from types import SimpleNamespace

from scv_survey.cellmap_store import CellRow

from scv_manager.manager_core import mask_cells, remeasure_targets
from scv_manager.route_builder import RouteBuilder, wall_points

ZONES = """
zones:
  - id: Z1
    dock: {x: 3.184, y: 0.995}
    polygon: [[0.08, -0.05], [3.18, -0.05], [3.18, 3.29], [0.08, 3.29]]
    spacing: 0.5
  - id: Z2
    dock: {x: 3.176, y: 4.597}
    polygon: [[0.08, 3.29], [3.18, 3.29], [3.18, 6.48], [0.08, 6.48]]
    spacing: 0.5
"""
FACILITIES = """
facilities:
  - id: SHT-01
    view: {x: 2.70, y: 3.00, yaw: 3.142}
  - id: EXT-01
    view: {x: 0.0, y: 0.0, yaw: 0.0}
"""
ORIGIN = (-0.15, -0.25)


def builder(tmp_path: Path) -> RouteBuilder:
    (tmp_path / "zones.yaml").write_text(ZONES, encoding="utf-8")
    (tmp_path / "facilities.yaml").write_text(FACILITIES, encoding="utf-8")
    return RouteBuilder(
        str(tmp_path / "zones.yaml"),
        str(tmp_path / "facilities.yaml"),
        wall_bounds=(0.08, -0.05, 3.18, 6.48),
        wall_clearance=0.45,
        origin=ORIGIN,
    )


def test_wall_points_reads_occupied_pixels_in_map_coordinates(tmp_path):
    # 3x2, 8비트. 윗줄 가운데와 아랫줄 왼쪽이 검은(점유) 픽셀
    (tmp_path / "m.pgm").write_bytes(b"P5\n3 2\n255\n" + bytes([254, 0, 254, 0, 205, 254]))
    info = SimpleNamespace(pgm=tmp_path / "m.pgm", origin=(1.0, 2.0), resolution=0.5)

    assert wall_points(info) == [(1.75, 2.75), (1.25, 2.25)]


def test_route_ends_and_cells_keep_clear_of_inner_wall(tmp_path):
    # Z2 첫 줄(y 3.5) 오른쪽 위의 벽 조각 (실제 지도의 y 3.62, x 2.0–2.7)
    walls = [(2.0 + 0.05 * k, 3.62) for k in range(15)]
    rb = builder(tmp_path)
    rb.walls, rb.obstacle_clearance = walls, 0.30
    z1, z2 = rb.build_survey_routes()

    assert len(z1.waypoints) == 12 and len(z2.waypoints) == 12
    assert all(rb.clear_of_walls(p.x, p.y) for p in z2.waypoints)
    assert (round(z2.waypoints[0].x, 2), z2.waypoints[0].y) == (1.68, 3.5)
    assert {round(p.x, 2) for p in z1.waypoints} == {0.53, 2.73}

    grid = [
        (i, j, ORIGIN[0] + (i + 0.5) * 0.5, ORIGIN[1] + (j + 0.5) * 0.5)
        for i in range(7)
        for j in range(14)
    ]
    # 칸 중심 (2.1, 3.5), (2.6, 3.5) 는 벽 조각에서 0.12 m
    assert rb.reachable_cells("Z2", grid) == {
        c for c in builder(tmp_path).reachable_cells("Z2", grid)
        if c not in {(4, 7), (5, 7)}
    }


def test_survey_route_keeps_wall_clearance_and_starts_at_dock_side(tmp_path):
    z1, z2 = builder(tmp_path).build_survey_routes()

    assert len(z1.waypoints) == 12
    assert {round(p.x, 2) for p in z1.waypoints} == {0.53, 2.73}
    assert sorted({round(p.y, 2) for p in z1.waypoints}) == [
        0.5, 1.0, 1.5, 2.0, 2.5, 3.0,
    ]
    assert (round(z1.waypoints[0].x, 2), z1.waypoints[0].y) == (2.73, 0.5)
    assert sorted({round(p.y, 2) for p in z2.waypoints}) == [
        3.5, 4.0, 4.5, 5.0, 5.5, 6.0,
    ]


def test_facility_gets_zone_and_zero_view_is_skipped(tmp_path):
    rb = builder(tmp_path)
    routes = rb.build_facility_routes()

    assert [(r.facility_id, r.zone_id) for r in routes] == [("SHT-01", "Z1")]
    assert rb.skipped_facilities == ["EXT-01"]


def test_reachable_cells_are_inside_zone_and_clearance(tmp_path):
    grid = [
        (i, j, ORIGIN[0] + (i + 0.5) * 0.5, ORIGIN[1] + (j + 0.5) * 0.5)
        for i in range(7)
        for j in range(14)
    ]
    rb = builder(tmp_path)

    assert len(rb.reachable_cells("Z1", grid)) == 30
    assert len(rb.reachable_cells("Z2", grid)) == 30
    assert not rb.reachable_cells("Z1", grid) & rb.reachable_cells("Z2", grid)


def test_remeasure_targets_order_limit_and_reachable():
    cells = {
        (1, 1): CellRow("normal", -50.0, 3, "ok"),
        (2, 1): CellRow("shadow", -75.0, 3, "ok"),
        (3, 1): CellRow("shadow", -80.0, 3, "ok"),
        (4, 1): CellRow("miss", None, 3, "ok"),
        (9, 9): CellRow("miss", None, 3, "ok"),
    }
    reachable = {(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)}

    assert remeasure_targets(cells, reachable, 10) == [
        (4, 1), (3, 1), (2, 1), (5, 1),
    ]
    assert remeasure_targets(cells, reachable, 2) == [(4, 1), (3, 1)]
    assert (5, 1) not in remeasure_targets(cells, reachable, 10, False)


def test_mask_cells_skip_normal_low_confidence_and_dock():
    cells = {
        (1, 1): CellRow("normal", -50.0, 3, "ok"),
        (2, 1): CellRow("shadow", -75.0, 3, "ok"),
        (3, 1): CellRow("miss", None, 1, "low"),
        (6, 2): CellRow("shadow", -75.0, 3, "ok"),
    }

    assert mask_cells(cells, 0.5, ORIGIN, [(3.184, 0.995)], 0.6) == {(2, 1)}

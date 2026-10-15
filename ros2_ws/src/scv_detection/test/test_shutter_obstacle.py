"""방화셔터 구역의 기본 장애물 높이(7cm)가 10cm 물체는 잡고 판지 바닥 오차는 무시하는지 확인하는 시험."""

import cv2
import numpy as np

from scv_detection import shutter_obstacle


def _scene(box_height_m=None, noise_m=0.012, seed=0):
    """카메라가 바닥을 비스듬히 보는 가상 장면을 만든다.

    반환값은 (BGR 영상, 미터 Depth, 픽셀 방향, 카메라 기준 위쪽 방향)이다.
    box_height_m을 주면 구역 가운데에 그 높이의 상자 윗면을 Depth로 넣는다.
    """
    height, width, focal = 480, 640, 518.0
    matrix = (focal, 0.0, 320.0, 0.0, focal, 240.0, 0.0, 0.0, 1.0)
    calibration = {'width': width, 'height': height, 'k': matrix, 'p': matrix[:3] + (0.0,) + matrix[3:6] + (0.0,)
                   + matrix[6:] + (0.0,), 'r': (1, 0, 0, 0, 1, 0, 0, 0, 1), 'd': (0, 0, 0, 0, 0),
                   'distortion_model': 'plumb_bob', 'frame_id': 'camera', 'cropped_or_binned': False}
    rays = shutter_obstacle.camera_rays((height, width), calibration)
    tilt = np.deg2rad(20)
    up = np.array([0.0, -np.cos(tilt), -np.sin(tilt)])
    camera_height = 0.25
    along_down = rays @ -up
    depth = np.where(along_down > 1e-3, camera_height / np.maximum(along_down, 1e-6), 0.0)
    frame = np.full((height, width, 3), (150, 170, 180), np.uint8)
    corners = np.array([[150, 250], [490, 250], [600, 460], [40, 460]])
    cv2.polylines(frame, [corners], True, (30, 30, 200), 8)
    if box_height_m is not None:
        box = np.zeros((height, width), bool)
        # 윗면의 바닥 투영이 가까운 변의 안쪽 띠(obstacle_boundary_inset_m)에 걸리지 않게 조금 멀리 둔다.
        box[280:330, 280:360] = True
        top = camera_height - box_height_m
        depth[box] = top / along_down[box]
    depth = depth + np.random.default_rng(seed).normal(0, noise_m, depth.shape)
    return frame, depth, rays, up


def test_default_height_detects_ten_centimeter_box():
    """요구사항 크기인 10cm 상자는 기본 설정에서 장애물로 판정한다."""
    report = shutter_obstacle.inspect(*_scene(box_height_m=0.10))
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    assert max(item['height_p90_m'] for item in report['obstacles']) > 0.08


def test_default_height_ignores_floor_depth_error():
    """판지 바닥 녹화에서 본 12mm 수준 Depth 오차만 있는 바닥은 장애물로 판정하지 않는다."""
    report = shutter_obstacle.inspect(*_scene())
    assert report['obstacles'] == [] and report['status'] == 'NORMAL'
    assert report['floor_pixel_ratio'] > 0.9


def test_thick_near_tape_is_kept_as_zone_boundary():
    """카메라 가까이에서 약 25px 폭으로 굵게 보이는 테두리 테이프도 넓은 빨간 면으로 지우지 않는다."""
    frame = np.full((480, 640, 3), (150, 170, 180), np.uint8)
    corners = np.array([[230, 300], [600, 300], [630, 470], [60, 470]])
    cv2.polylines(frame, [corners[:2]], False, (30, 30, 200), 6)
    cv2.polylines(frame, [corners[1:3]], False, (30, 30, 200), 10)
    cv2.polylines(frame, [corners[2:]], False, (30, 30, 200), 18)
    cv2.line(frame, tuple(corners[3]), tuple(corners[0]), (30, 30, 200), 25)
    target = cv2.contourArea(corners.astype(np.float32))
    zone = shutter_obstacle._find_zone(frame, shutter_obstacle.validate_options())
    old_zone = shutter_obstacle._find_zone(frame, shutter_obstacle.validate_options(max_tape_width_ratio=0.035))
    # 기본값에서는 왼쪽 굵은 선까지 구역에 들어가고, 예전 17px 기준에서는 그 선이 지워져 구역이 작아진다.
    assert zone is not None and cv2.contourArea(zone[0].astype(np.float32)) > 0.8 * target
    assert old_zone is None or cv2.contourArea(old_zone[0].astype(np.float32)) < 0.6 * target


def test_unpaired_reading_keeps_confirmation_history():
    """짝 없는 판독이 사이에 끼어도 같은 구역의 ABNORMAL 확인 횟수가 이어져 확정되는지 확인한다."""
    polygon = [[0, 0], [100, 0], [100, 100], [0, 100]]
    state = {}

    def abnormal():
        """구역 가운데 장애물 하나가 있는 ABNORMAL 판독을 만든다."""
        return dict(shutter_obstacle.unconfirmed('x'), status='ABNORMAL', reason='obstacle_above_floor_inside_zone',
                    zone_polygon=polygon, obstacles=[{'bbox': [40, 40, 60, 60]}], obstacles_evaluated=True)

    shutter_obstacle.stabilize_report(abnormal(), state, 0.0, 1)
    skipped = shutter_obstacle.stabilize_report(shutter_obstacle.unconfirmed('rgb_depth_time_mismatch'), state, 0.1, 0)
    assert skipped['status'] == 'UNCONFIRMED' and state['count'] == 1
    geometry_fail = dict(shutter_obstacle.unconfirmed('zone_plane_intersection_invalid'), zone_polygon=polygon)
    shutter_obstacle.stabilize_report(geometry_fail, state, 0.15, 0)
    assert state['count'] == 1
    shutter_obstacle.stabilize_report(abnormal(), state, 0.3, 2)
    final = shutter_obstacle.stabilize_report(abnormal(), state, 0.6, 3)
    assert final['status'] == 'ABNORMAL' and final['confirm_frames'] == 3


def test_slightly_changed_zone_keeps_history():
    """잘린 구역의 모양이 조금 바뀌어(겹침 약 40%) 도 기본 기준 30%에서는 이력이 유지되는지 확인한다."""
    first = np.array([[0, 0], [100, 0], [100, 100], [0, 100]])
    second = np.array([[0, 0], [100, 0], [100, 40], [0, 40]])
    assert shutter_obstacle._same_zone(first, second, 0.30)
    assert not shutter_obstacle._same_zone(first, second, 0.50)


def test_obstacle_candidates_exclude_boundary_band():
    """경계에서 안쪽 띠 안에 있는 점은 음수 허용값으로 구역 밖 취급되는지 확인한다."""
    polygon = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    points = np.array([[0.5, 0.98], [0.5, 0.5]])
    assert shutter_obstacle._inside_polygon(points, polygon, 0.0).tolist() == [True, True]
    assert shutter_obstacle._inside_polygon(points, polygon, -0.05).tolist() == [False, True]


def test_scale_depth_calibration_moves_focal_and_center():
    """화면 배율·이동이 K와 P의 초점 거리·주점에 반영되고, 1·0 이면 원본을 그대로 돌려주는지 확인한다."""
    calibration = {'k': (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
                   'p': (500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0), 'frame_id': 'cam'}
    assert shutter_obstacle.scale_depth_calibration(calibration) is calibration
    scaled = shutter_obstacle.scale_depth_calibration(calibration, 0.5, 100.0, 2.0, -10.0)
    assert scaled['k'][0] == 250.0 and scaled['k'][2] == 260.0 and scaled['k'][4] == 1000.0 and scaled['k'][5] == 470.0
    assert scaled['p'][0] == 250.0 and scaled['p'][2] == 260.0 and scaled['p'][5] == 1000.0 and scaled['p'][6] == 470.0
    assert scaled['frame_id'] == 'cam' and calibration['k'][0] == 500.0


def test_fixed_plane_skips_ransac_but_still_detects_box():
    """미리 잰 바닥 평면을 주면 RANSAC 없이도 10cm 상자를 잡고 빈 바닥은 정상으로 판정하며, 틀린 평면은 거부한다."""
    frame, depth, rays, up = _scene(box_height_m=0.10)
    true_plane = [*up, 0.25]
    report = shutter_obstacle.inspect(frame, depth, rays, up, fixed_plane=true_plane)
    assert report['status'] == 'ABNORMAL' and report['floor_diagnostics']['plane_source'] == 'fixed_plane'
    empty = shutter_obstacle.inspect(*_scene(), fixed_plane=true_plane)
    assert empty['status'] == 'NORMAL' and empty['obstacles'] == []
    wrong = shutter_obstacle.inspect(*_scene(), fixed_plane=[*up, 0.50])
    assert wrong['status'] == 'UNCONFIRMED' and wrong['reason'] == 'floor_not_confirmed'
    assert wrong['floor_diagnostics']['plane_source'] == 'fixed_plane_rejected'


def test_display_bbox_covers_object_outside_zone():
    """구역 밖으로 이어진 상자 윗부분까지 표시용 사각형에 포함되고, 판정용 bbox 는 구역 안에 머무는지 확인한다."""
    frame, depth, rays, up = _scene(box_height_m=0.10)
    # 같은 높이·거리의 상자 면을 구역 위쪽(빨간 테두리 밖, 행 200~280)까지 이어 붙인다.
    box_rows = slice(200, 330)
    along_down = rays @ -up
    extended = np.zeros(depth.shape, bool)
    extended[box_rows, 280:360] = True
    depth[extended] = (0.25 - 0.10) / along_down[extended]
    report = shutter_obstacle.inspect(frame, depth, rays, up)
    assert report['status'] == 'ABNORMAL' and report['obstacles']
    obstacle = report['obstacles'][0]
    assert obstacle['bbox'][1] >= 250 and obstacle['display_bbox'][1] < obstacle['bbox'][1]
    assert obstacle['display_bbox'][2] >= obstacle['bbox'][2] and obstacle['display_bbox'][3] >= obstacle['bbox'][3]


def test_vertical_consistency_drops_thin_band_but_keeps_surface():
    """6줄짜리 가까운 깊이 띠는 걸러지고, 20줄짜리 실제 면과 평탄한 바닥은 통과하는지 확인한다."""
    depth = np.full((120, 40), 2.0, dtype=np.float32)
    depth[50:56, :] = 1.3   # 상자 밑단 가장자리 잡음 같은 얇은 띠
    depth[80:100, :] = 1.5  # 실제 물체 면
    depth[10:14, 5] = np.nan  # 없는 깊이는 판단에서 제외
    mask = shutter_obstacle._vertically_consistent(depth, 0.10)
    assert not mask[50:56, 20].any()
    assert mask[82:98, 20].all() and mask[20:45, 20].all()
    assert mask[10:14, 5].all()


def _reference_vertical_check(depth, tolerance, rows=21):
    """속도 개선 전 방식(np.nanmedian)으로 세로 중앙값 검사를 그대로 계산한 기준 답을 반환한다."""
    import warnings
    values = np.where(np.isfinite(depth) & (depth > 0), depth, np.nan).astype(np.float32)
    half = rows // 2
    padded = np.pad(values, ((half, half), (0, 0)), constant_values=np.nan)
    stack = np.stack([padded[offset:offset + values.shape[0]] for offset in range(rows)])
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        median = np.nanmedian(stack, axis=0)
    enough = np.count_nonzero(np.isfinite(stack), axis=0) >= (rows + 1) // 2
    return ~np.isfinite(values) | ~enough | ~np.isfinite(median) | (np.abs(values - median) <= tolerance)


def test_fast_vertical_check_matches_nanmedian_with_holes():
    """빠르게 바꾼 세로 중앙값 검사가 빈 Depth가 섞여도 예전 nanmedian 방식과 픽셀마다 같은지 확인한다.

    where로 일부 픽셀만 검사할 때는 그 픽셀의 답이 같고, 나머지는 통과(참)여야 한다.
    """
    rng = np.random.default_rng(3)
    depth = 1.0 + rng.normal(0, 0.05, (120, 90))
    depth[rng.random(depth.shape) < 0.25] = 0.0  # 빈 값을 많이 섞어 짝수 개 중앙값 경우도 만든다.
    depth[30:36, 10:30] = 0.6  # 가까운 얇은 띠
    depth[:, 50] = np.nan
    expected = _reference_vertical_check(depth, 0.10)
    assert np.array_equal(shutter_obstacle._vertically_consistent(depth, 0.10), expected)
    where = rng.random(depth.shape) < 0.3
    partial = shutter_obstacle._vertically_consistent(depth, 0.10, where=where)
    assert np.array_equal(partial[where], expected[where]) and partial[~where].all()


def test_focal_override_changes_only_focal_length():
    """초점 거리 덮어쓰기는 K·P의 fx·fy만 바꾸고 주점·왜곡은 그대로 두며, 0이면 원래 보정값을 그대로 돌려준다."""
    calibration = {'k': (569.95, 0, 353.4, 0, 569.95, 354.1, 0, 0, 1),
                   'p': (569.95, 0, 353.4, 0, 0, 569.95, 354.1, 0, 0, 0, 1, 0),
                   'd': (0.1, 0.2, 0, 0, 0), 'width': 704, 'height': 704}
    assert shutter_obstacle.override_focal_length(calibration, 0) is calibration
    changed = shutter_obstacle.override_focal_length(calibration, 1036.0)
    k = np.reshape(changed['k'], (3, 3)); p = np.reshape(changed['p'], (3, 4))
    assert k[0, 0] == k[1, 1] == p[0, 0] == p[1, 1] == 1036.0
    assert (k[0, 2], k[1, 2], p[0, 2], p[1, 2]) == (353.4, 354.1, 353.4, 354.1)
    assert changed['d'] == calibration['d'] and calibration['k'][0] == 569.95


def test_camera_height_check_suggests_focal_from_tf_height():
    """바닥 평면 높이가 TF 높이의 1.77배면 비율 1.77과 그만큼 큰 제안 초점 거리를 돌려준다."""
    check = shutter_obstacle.camera_height_check([0.0, -1.0, 0.0, 0.432], 0.244, 569.95)
    assert abs(check['camera_height_ratio'] - 0.432 / 0.244) < 1e-9
    assert abs(check['suggested_focal_px'] - 569.95 * 0.432 / 0.244) < 1e-6
    empty = shutter_obstacle.camera_height_check(None, 0.244, 569.95)
    assert empty['camera_height_ratio'] is None and empty['suggested_focal_px'] is None

"""빨간 방화셔터 구역의 RGB·Depth를 이용해 RANSAC으로 바닥과 장애물을 구분하는 함수 모음.

카메라 구독·압축 해제·CameraInfo·TF·ROS 실행은 매니저가 담당한다.
높이와 면적은 미터 단위다. 기본 감도는 보정 전 초기값이며 실제 센서에 맞춰 조정한다.
구역이 잘렸거나 깊이·바닥 근거가 부족한 경우에는 장애물이 없다고 확정하지 않는다.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


DEFAULT_OPTIONS = {
    'red_h_low_end': 12, 'red_h_high_start': 165, 'red_min_s': 70, 'red_min_v': 45,
    'red_support_min_s': 35, 'min_red_fragment_pixel_ratio': 0.0001,
    'red_join_ratio': 0.015, 'min_boundary_support_ratio': 0.70,
    'max_tape_width_ratio': 0.035, 'geometry_max_gap_seconds': 1.0,
    'tracking_min_points': 12, 'tracking_min_inlier_ratio': 0.65,
    'tracking_red_support_ratio': 0.50, 'plane_update_ratio': 0.20,
    'zone_fragment_overlap_ratio': 0.20,
    'min_zone_area_ratio': 0.03, 'max_red_fill_ratio': 0.45, 'zone_candidate_margin': 0.25,
    'floor_band_ratio': 0.02, 'depth_min_m': 0.2, 'depth_max_m': 5.0,
    'plane_distance_m': 0.008, 'ransac_iterations': 250, 'max_plane_points': 6000,
    'floor_noise_multiplier': 3.0, 'floor_max_height_ratio': 0.75,
    'min_plane_points': 100, 'min_plane_inlier_ratio': 0.55, 'max_floor_tilt_deg': 15.0,
    # 요구사항(BR-T14)은 10cm 이상 물체다. 2026-10-14 녹화에서 판지 바닥의 Depth 오차가 최대 57mm였으므로
    # 그보다 높고 10cm보다 충분히 낮은 7cm를 기본 높이로 둔다.
    'min_floor_span_m': 0.15, 'min_obstacle_height_m': 0.07, 'cluster_voxel_m': 0.03,
    'min_cluster_points': 20, 'min_obstacle_area_m2': 0.0004, 'boundary_tolerance_m': 0.005,
    'coverage_cell_m': 0.05, 'max_coverage_cells': 10000, 'min_coverage_ratio': 0.98,
    'min_valid_depth_ratio': 0.80, 'confirm_frames': 3, 'confirm_seconds': 0.5,
    'confirm_max_gap_seconds': 1.0,
}

STATUS_MESSAGES = {
    'ABNORMAL': '경고)장애물을 치워주십시오',
    'NORMAL': '정상)장애물이 없습니다',
    'UNCONFIRMED': '미확인)장애물 상태를 확인할 수 없습니다',
}


def validate_options(**overrides):
    """색·선 폭·영상 추적·바닥 오차와 시간 설정을 검사해 계산에 쓸 설정 사전을 반환한다."""
    if set(overrides) - set(DEFAULT_OPTIONS):
        raise ValueError('unknown shutter obstacle option')
    settings = dict(DEFAULT_OPTIONS, **overrides)
    for name, value in settings.items():
        if not np.isfinite(value):
            raise ValueError(f'shutter_{name} must be finite')
    integers = ('red_h_low_end', 'red_h_high_start', 'red_min_s', 'red_min_v', 'red_support_min_s',
                'ransac_iterations', 'max_plane_points', 'min_plane_points',
                'min_cluster_points', 'max_coverage_cells', 'confirm_frames', 'tracking_min_points')
    for name in integers:
        if settings[name] < 1 or settings[name] != int(settings[name]):
            raise ValueError(f'shutter_{name} must be a positive integer')
        settings[name] = int(settings[name])
    if not 0 < settings['red_h_low_end'] < settings['red_h_high_start'] <= 179:
        raise ValueError('shutter red H intervals must be ordered within 0..179')
    if settings['red_min_s'] > 255 or settings['red_min_v'] > 255:
        raise ValueError('shutter red S/V lower bounds must not exceed 255')
    if settings['red_support_min_s'] > settings['red_min_s']:
        raise ValueError('shutter red support S must not exceed strong red S')
    for name in ('min_zone_area_ratio', 'max_red_fill_ratio', 'zone_candidate_margin',
                 'floor_band_ratio', 'red_join_ratio', 'min_boundary_support_ratio',
                 'min_red_fragment_pixel_ratio', 'zone_fragment_overlap_ratio', 'floor_max_height_ratio',
                 'min_plane_inlier_ratio', 'min_coverage_ratio', 'min_valid_depth_ratio',
                 'max_tape_width_ratio', 'tracking_min_inlier_ratio', 'tracking_red_support_ratio',
                 'plane_update_ratio'):
        if not 0 < settings[name] < 1:
            raise ValueError(f'shutter_{name} must be within 0..1')
    positive = ('depth_min_m', 'depth_max_m', 'plane_distance_m', 'min_floor_span_m',
                'min_obstacle_height_m', 'cluster_voxel_m', 'min_obstacle_area_m2',
                'coverage_cell_m', 'confirm_seconds', 'confirm_max_gap_seconds', 'floor_noise_multiplier',
                'geometry_max_gap_seconds')
    if any(settings[name] <= 0 for name in positive) or settings['boundary_tolerance_m'] < 0:
        raise ValueError('shutter distance/area/time must be positive and boundary tolerance nonnegative')
    if (settings['depth_min_m'] >= settings['depth_max_m']
            or settings['min_obstacle_height_m'] <= settings['plane_distance_m']
            or settings['min_plane_points'] < 3 or settings['max_plane_points'] < settings['min_plane_points']
            or settings['confirm_frames'] < 2 or settings['tracking_min_points'] < 4
            or not 0 < settings['max_floor_tilt_deg'] < 45):
        raise ValueError('invalid shutter depth limits, plane/height sensitivity or confirmation settings')
    return settings


def unconfirmed(reason, detail=''):
    """미확인 사유와 설명을 받아 구역·바닥 표시와 관측 비율·오차 허용값을 비운 판정을 반환한다."""
    return {'status': 'UNCONFIRMED', 'reason': reason, 'detail': detail,
            'zone_polygon': None, 'zone_complete': False, 'plane': None,
            '_zone_mask': None, 'zone_pixel_count': 0, 'floor_fit_point_count': 0,
            'floor_anchor_point_count': 0, 'floor_classify_distance_m': None, 'floor_pixel_ratio': None,
            '_floor_inlier_mask': None, 'floor_inlier_count': 0,
            '_red_tape_mask': None, 'red_pixel_count': None, 'red_component_count': None,
            'zone_candidate_count': None, 'zone_detection_reason': None,
            'zone_merged_fragment_count': 0,
            'floor_inlier_ratio': None, 'floor_noise_m': None, 'effective_height_m': None,
            'valid_depth_ratio': None, 'coverage_ratio': None, 'obstacles': []}


def report_details(report):
    """판정 사전을 받아 표시 전용 값을 제외한 JSON 발행용 사전을 반환한다.

    이름이 밑줄로 시작하는 값은 영상 표시용이다. 구역·바닥·빨간 선 마스크를 ROS 상태 설명에 넣지 않는다.
    """
    return {name: value for name, value in report.items() if not name.startswith('_')}


def stream_timing(rgb_samples, depth_samples, tolerance_s, pair_results=()):
    """RGB·Depth의 촬영 시각과 수신 시각으로 발행 주기와 시각 정렬 상태를 계산해 사전으로 반환한다.

    rgb_samples·depth_samples는 (촬영 시각 ns, 수신 단조 시각 초) 목록이다. tolerance_s는 짝 허용 시간이다.
    pair_results는 최근 짝 찾기 성공 여부(참·거짓) 목록이다.
    반환값의 rgb_hz·depth_hz는 촬영 시각 간격으로 구한 실제 발행 주기, phase_*_ms는 각 Depth와
    가장 가까운 RGB의 촬영 시각 차이, matched_ratio는 그 차이가 허용 시간 안인 Depth 비율이다.
    depth_extra_lag_s는 Depth가 RGB보다 PC에 늦게 도착하는 시간이다. 두 컴퓨터 시계 차이는 서로 빼져 사라진다.
    표본이 부족한 값은 None이다.
    """
    def period_hz(stamps):
        """촬영 시각 배열의 중앙 간격을 주파수(Hz)로 바꾼다. 간격이 없으면 None이다."""
        steps = np.diff(np.unique(stamps)) / 1e9
        return float(1.0 / np.median(steps)) if len(steps) and np.median(steps) > 0 else None

    rgb = np.asarray([item for item in rgb_samples if item[0] > 0], dtype=float).reshape(-1, 2)
    depth = np.asarray([item for item in depth_samples if item[0] > 0], dtype=float).reshape(-1, 2)
    result = {'rgb_hz': period_hz(rgb[:, 0]), 'depth_hz': period_hz(depth[:, 0]),
              'phase_median_ms': None, 'phase_p90_ms': None, 'matched_ratio': None,
              'depth_extra_lag_s': None, 'pair_success_ratio': None,
              'rgb_samples': len(rgb), 'depth_samples': len(depth)}
    if len(rgb) and len(depth):
        rgb_stamps = np.sort(rgb[:, 0])
        index = np.clip(np.searchsorted(rgb_stamps, depth[:, 0]), 1, len(rgb_stamps) - 1) if len(rgb_stamps) > 1 else None
        if index is None:
            nearest = np.abs(depth[:, 0] - rgb_stamps[0])
        else:
            nearest = np.minimum(np.abs(depth[:, 0] - rgb_stamps[index - 1]), np.abs(depth[:, 0] - rgb_stamps[index]))
        # RGB 표본 범위 밖의 Depth는 짝이 아직 안 왔거나 이미 지나간 것이므로 위상 계산에서 뺀다.
        inside = (depth[:, 0] >= rgb_stamps[0]) & (depth[:, 0] <= rgb_stamps[-1])
        if np.any(inside):
            phase = nearest[inside] / 1e6
            result.update(phase_median_ms=float(np.median(phase)), phase_p90_ms=float(np.percentile(phase, 90)),
                          matched_ratio=float(np.mean(phase <= tolerance_s * 1000)))
        result['depth_extra_lag_s'] = float(np.median(depth[:, 1] - depth[:, 0] / 1e9)
                                            - np.median(rgb[:, 1] - rgb[:, 0] / 1e9))
    if len(pair_results):
        result['pair_success_ratio'] = float(np.mean(pair_results))
    return result


def _depth_step(depth_values):
    """구역 안 Depth 값에서 중앙 거리 근처의 서로 다른 값 간격을 구해 Depth 단계(m)를 반환한다.

    스테레오 Depth가 계단처럼 끊어진 값만 내면 이 간격이 커진다. 표본이 부족하면 None이다.
    """
    if len(depth_values) < 100:
        return None
    center = float(np.median(depth_values))
    near = np.unique(np.round(depth_values[np.abs(depth_values - center) <= 0.1], 4))
    if len(near) < 3:
        return None
    return float(np.median(np.diff(near)))


def rotation_matrix(quaternion):
    """TF의 (x, y, z, w) 회전값을 받아 벡터를 회전시키는 3×3 행렬을 반환한다."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) <= 0:
        raise ValueError('invalid camera TF quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


@lru_cache(maxsize=4)
def _camera_rays(height, width, matrix_values, distortion, model, rectified):
    """영상 크기와 보정값을 받아 각 픽셀의 카메라 방향 벡터를 만들고 같은 보정값은 재사용한다.

    반환 배열은 (높이, 너비, 3)이며 Z=1인 방향 벡터다. 원본 왜곡 영상은 왜곡을 보정한다.
    """
    matrix = np.asarray(matrix_values, dtype=float).reshape(3, 3)
    if (not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], (0, 0, 1))):
        raise ValueError('camera calibration is missing or invalid')
    ys, xs = np.indices((height, width), dtype=np.float64)
    pixels = np.column_stack((xs.ravel(), ys.ravel())).reshape(-1, 1, 2)
    if rectified or not distortion:
        inverse = np.linalg.inv(matrix)
        rays = np.column_stack((xs.ravel(), ys.ravel(), np.ones(height * width))) @ inverse.T
        rays /= rays[:, 2:3]
    else:
        coefficients = np.asarray(distortion, dtype=float)
        if not np.isfinite(coefficients).all():
            raise ValueError('invalid camera distortion coefficients')
        if model == 'equidistant' and len(coefficients) == 4:
            normalized = cv2.fisheye.undistortPoints(pixels, matrix, coefficients)
        elif model in ('plumb_bob', 'rational_polynomial') and len(coefficients) in (4, 5, 8, 12, 14):
            normalized = cv2.undistortPoints(pixels, matrix, coefficients)
        else:
            raise ValueError('unsupported camera distortion model')
        rays = np.column_stack((normalized.reshape(-1, 2), np.ones(height * width)))
    if not np.isfinite(rays).all():
        raise ValueError('camera calibration produces invalid pixel rays')
    rays = rays.reshape(height, width, 3)
    rays.setflags(write=False)
    return rays


def camera_rays(image_shape, calibration, rectified=False):
    """원본 영상 크기와 CameraInfo에서 정리한 보정 사전을 받아 같은 픽셀의 방향 배열을 반환한다.

    임의 크기 조절이나 보정되지 않은 잘라내기는 허용하지 않는다. 오류가 있으면 ValueError다.
    """
    height, width = image_shape[:2]
    if (height, width) != (calibration['height'], calibration['width']):
        raise ValueError('RGB size differs from CameraInfo calibration size')
    if calibration.get('cropped_or_binned', False):
        raise ValueError('cropped/binned CameraInfo requires corrected calibration')
    if rectified:
        projection = np.asarray(calibration['p'], dtype=float).reshape(3, 4)
        rotation = np.asarray(calibration['r'], dtype=float).reshape(3, 3)
        if (not np.isfinite(projection).all() or not np.allclose(projection[:, 3], 0)
                or not np.allclose(rotation, np.eye(3))):
            raise ValueError('rectified RGB must use its own optical frame and monocular calibration')
        matrix = projection[:, :3].reshape(-1)
    else:
        matrix = calibration['k']
    return _camera_rays(height, width, tuple(matrix), tuple(calibration['d']),
                        calibration['distortion_model'], bool(rectified))


@lru_cache(maxsize=4)
def _depth_pixel_map(height, width, rgb_matrix, rgb_distortion, rgb_model, rgb_rectified,
                     depth_matrix, depth_distortion, depth_model):
    """두 카메라 보정값으로 RGB 픽셀마다 읽어야 할 Depth 위치를 계산하고 재사용한다.

    두 영상은 같은 광학 좌표계를 써야 한다. 반환값은 OpenCV remap용 X·Y 위치 배열이다.
    """
    rays = _camera_rays(height, width, rgb_matrix, rgb_distortion, rgb_model, rgb_rectified)
    matrix = np.asarray(depth_matrix, dtype=float).reshape(3, 3)
    # Depth의 보정값도 같은 검증 경로를 거쳐 잘못된 초점 거리나 왜곡 모델을 거른다.
    _camera_rays(height, width, depth_matrix, depth_distortion, depth_model, False)
    directions = rays.reshape(-1, 1, 3)
    coefficients = np.asarray(depth_distortion, dtype=float)
    if depth_model == 'equidistant' and len(coefficients) == 4:
        pixels, _ = cv2.fisheye.projectPoints(directions, np.zeros(3), np.zeros(3), matrix, coefficients)
    else:
        pixels, _ = cv2.projectPoints(directions, np.zeros(3), np.zeros(3), matrix,
                                     coefficients if coefficients.size else None)
    pixels = pixels.reshape(height, width, 2).astype(np.float32)
    if not np.isfinite(pixels).all():
        raise ValueError('invalid Depth to RGB pixel mapping')
    return pixels[:, :, 0].copy(), pixels[:, :, 1].copy()


def depth_for_rgb(depth_m, image_shape, rgb_calibration, depth_calibration, rgb_rectified=False):
    """RGB·Depth CameraInfo를 받아 Depth를 RGB 픽셀 위치로 옮기고 미터 배열을 반환한다.

    해상도와 광학 좌표계가 같은 입력만 받는다. 두 영상의 왜곡값이 달라도 같은 방향의 Depth를 읽는다.
    가장 가까운 실제 샘플을 사용하고 영상 밖은 0으로 둔다. 없는 Depth를 보간해 만들어 내지 않는다.
    """
    rays = camera_rays(image_shape, rgb_calibration, rgb_rectified)
    camera_rays(depth_m.shape, depth_calibration)
    if (depth_m.shape != image_shape[:2] or not rgb_calibration.get('frame_id')
            or rgb_calibration['frame_id'] != depth_calibration.get('frame_id')):
        raise ValueError('Depth mapping requires equal sizes and a shared optical frame')
    rotation = np.asarray(depth_calibration['r'], dtype=float).reshape(3, 3)
    projection = np.asarray(depth_calibration['p'], dtype=float).reshape(3, 4)
    if not np.allclose(rotation, np.eye(3)) or not np.allclose(projection[:, 3], 0):
        raise ValueError('Depth mapping requires monocular calibration in its optical frame')
    rgb_matrix = (np.asarray(rgb_calibration['p']).reshape(3, 4)[:, :3].ravel()
                  if rgb_rectified else rgb_calibration['k'])
    map_x, map_y = _depth_pixel_map(*depth_m.shape, tuple(rgb_matrix), tuple(rgb_calibration['d']),
                                   rgb_calibration['distortion_model'], bool(rgb_rectified),
                                   tuple(depth_calibration['k']), tuple(depth_calibration['d']),
                                   depth_calibration['distortion_model'])
    mapped = cv2.remap(depth_m, map_x, map_y, cv2.INTER_NEAREST,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return mapped, rays


def _boundary_supported(polygon, red, settings):
    """구역 다각형의 각 변에 실제 빨간 선이 충분히 보이는지 검사해 참 또는 거짓을 반환한다.

    red는 보조 색 조건까지 확인한 빨간 픽셀 마스크다. 선의 작은 틈만 허용하고 없는 경계를 있다고 보지 않는다.
    """
    size = max(3, int(round(min(red.shape) * settings['red_join_ratio'])) | 1)
    nearby = cv2.dilate(red, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))) > 0
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        count = max(2, int(np.ceil(np.linalg.norm(end - start))) + 1)
        samples = np.rint(np.linspace(start, end, count)).astype(np.int32)
        samples[:, 0] = np.clip(samples[:, 0], 0, red.shape[1] - 1)
        samples[:, 1] = np.clip(samples[:, 1], 0, red.shape[0] - 1)
        if np.mean(nearby[samples[:, 1], samples[:, 0]]) < settings['min_boundary_support_ratio']:
            return False
    return True


def _red_masks(frame, settings):
    """BGR 영상에서 (진한 빨간색 마스크, 옅은 빨간색까지 포함한 보조 마스크)를 반환한다.

    두 마스크 모두 같은 색상 구간을 쓴다. 보조 마스크는 채도만 낮춰 밝게 보이는 테이프를 보존한다.
    보조 색만 있는 묶음은 독립 검사 구역으로 확정하지 않고 진한 빨간 선과의 위치 관계도 확인한다.
    넓은 빨간 면을 제거해 박스의 색칠된 면이 바닥 테이프처럼 사용되는 것을 줄인다.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = []
    for saturation in (settings['red_min_s'], settings['red_support_min_s']):
        masks.append(cv2.bitwise_or(
            cv2.inRange(hsv, (0, saturation, settings['red_min_v']), (settings['red_h_low_end'], 255, 255)),
            cv2.inRange(hsv, (settings['red_h_high_start'], saturation, settings['red_min_v']), (179, 255, 255))))
    size = max(3, int(round(min(frame.shape[:2]) * settings['max_tape_width_ratio'])) | 1)
    broad = cv2.morphologyEx(masks[1], cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size)))
    broad = cv2.dilate(broad, np.ones((3, 3), np.uint8))
    narrow = cv2.bitwise_not(broad)
    return cv2.bitwise_and(masks[0], narrow), cv2.bitwise_and(masks[1], narrow)


def _zone_masks(polygon, red, settings):
    """구역 경계와 빨간 선에서 구역 마스크·바깥 경계 주변 근거·표시용 선을 반환한다.

    내부 사선이나 박스 색은 바닥 평면의 기준으로 삼지 않는다. 벽과 바닥이 만나는 경계 바로 옆을
    피하고, 바깥 경계에서 안쪽으로 한 띠 들어온 바닥만 근거로 쓴다. 근거를 구역 밖으로 넓히지 않는다.
    """
    mask = np.zeros(red.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
    zone_mask = mask > 0
    selected_red = (red > 0) & zone_mask
    radius = max(1, int(round(min(red.shape) * settings['floor_band_ratio'])))
    boundary = np.zeros_like(mask)
    cv2.polylines(boundary, [np.rint(polygon).astype(np.int32)], True, 255, 2 * radius + 1)
    boundary_red = selected_red & (boundary > 0)
    inner = cv2.erode(mask, np.ones((2 * radius + 1, 2 * radius + 1), np.uint8),
                       borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0
    seed = (cv2.dilate(boundary_red.astype(np.uint8),
                       np.ones((4 * radius + 1, 4 * radius + 1), np.uint8)) > 0) & inner
    return zone_mask, seed, selected_red


def _merge_tape_fragments(fragments, settings):
    """빨간 선 조각들의 화면 범위가 충분히 겹치면 같은 구역의 묶음으로 합쳐 목록을 반환한다.

    fragments의 각 항목에는 실제 빨간 점 좌표, 둘러싼 사각형, 라벨 번호와 진한 점 수가 있다.
    겹침은 두 사각형 중 큰 면적을 기준으로 계산하므로 작은 잡점이나 떨어진 구역을 쉽게 합치지 않는다.
    """
    if not fragments:
        return []
    boxes = np.asarray([fragment['bbox'] for fragment in fragments], dtype=float)
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    parent = list(range(len(fragments)))

    def root(index):
        """조각 번호를 받아 같은 묶음의 대표 번호를 찾고 이후 탐색 경로를 줄인다."""
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for index, box in enumerate(boxes):
        low = np.maximum(box[:2], boxes[:, :2])
        high = np.minimum(box[2:], boxes[:, 2:])
        overlap = np.prod(np.maximum(0, high - low), axis=1) / np.maximum(areas[index], areas)
        for other in np.flatnonzero(overlap >= settings['zone_fragment_overlap_ratio']):
            if other > index:
                parent[root(int(other))] = root(index)
    groups = {}
    for index, fragment in enumerate(fragments):
        groups.setdefault(root(index), []).append(fragment)
    return [{'points': np.concatenate([fragment['points'] for fragment in group]),
             'labels': [fragment['label'] for fragment in group],
             'strong_count': sum(fragment['strong_count'] for fragment in group)} for group in groups.values()]


def _find_zone(frame, settings, diagnostics=None):
    """빨간 선 조각을 합쳐 (구역 다각형, 구역 마스크, 바깥 경계 근거 마스크, 완전성)을 반환한다.

    진한 선을 기준으로 옅거나 끊어진 선 조각의 위치 관계를 확인해 같은 구역으로 묶는다.
    고정 위치나 사진을 쓰지 않으며, 분리된 비슷한 구역은 임의로 하나라고 정하지 않는다.
    diagnostics가 있으면 선택된 조각 수와 색 검출·실패 정보를 넣는다. 실패하면 None이다.
    """
    height, width = frame.shape[:2]
    strong, red = _red_masks(frame, settings)
    join_size = max(3, int(round(min(height, width) * settings['red_join_ratio'])) | 1)
    joined = cv2.dilate(red, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (join_size, join_size)))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(joined)
    minimum_pixels = max(3, int(np.ceil(width * height * settings['min_red_fragment_pixel_ratio'])))
    minimum_area = width * height * settings['min_zone_area_ratio']
    rejected = {'small_area': 0, 'dense_red': 0, 'degenerate_polygon': 0, 'weak_only': 0}
    diagnostic = {'_red_tape_mask': red > 0, 'red_pixel_count': int(np.count_nonzero(red)),
                  'red_search_pixel_count': int(np.count_nonzero(red)), 'red_component_count': int(count - 1),
                  'zone_candidate_count': 0, 'zone_merged_fragment_count': 0,
                  'zone_detection_reason': 'red_pixels_missing', 'zone_rejected_counts': rejected}
    fragments = []
    for label in range(1, count):
        x, y, w, h, _ = stats[label]
        selected = labels[y:y+h, x:x+w] == label
        ys, xs = np.nonzero(selected & (red[y:y+h, x:x+w] > 0))
        if len(xs) < minimum_pixels:
            rejected['small_area'] += 1
            continue
        points = np.column_stack((xs + x, ys + y)).astype(np.int32)
        area = float(cv2.contourArea(cv2.convexHull(points)))
        if area >= minimum_area and len(points) / area > settings['max_red_fill_ratio']:
            rejected['dense_red'] += 1
            continue
        low, high = points.min(axis=0), points.max(axis=0) + 1
        fragments.append({'points': points, 'bbox': [*low, *high], 'label': label,
                          'strong_count': int(np.count_nonzero(selected & (strong[y:y+h, x:x+w] > 0)))})
    candidates = []
    for group in _merge_tape_fragments(fragments, settings):
        if group['strong_count'] < minimum_pixels:
            rejected['weak_only'] += 1
            continue
        points = group['points']
        hull = cv2.convexHull(points)
        area = float(cv2.contourArea(hull))
        if area < minimum_area:
            rejected['small_area'] += 1
            continue
        if len(points) / area > settings['max_red_fill_ratio']:
            rejected['dense_red'] += 1
            continue
        polygon = cv2.approxPolyDP(hull, cv2.arcLength(hull, True) * 0.02, True)
        if len(polygon) < 3 or not cv2.isContourConvex(polygon) or cv2.contourArea(polygon) <= 0:
            rejected['degenerate_polygon'] += 1
            continue
        polygon = polygon.reshape(-1, 2)
        clipped = (points[:, 0].min() <= 1 or points[:, 1].min() <= 1
                   or points[:, 0].max() >= width - 2 or points[:, 1].max() >= height - 2)
        complete = len(polygon) == 4 and not clipped and _boundary_supported(polygon, red, settings)
        candidates.append((area, polygon, group['labels'], complete))
    candidates.sort(key=lambda item: item[0], reverse=True)
    diagnostic['zone_candidate_count'] = len(candidates)
    if not candidates or (len(candidates) > 1
                          and candidates[1][0] >= candidates[0][0] * (1 - settings['zone_candidate_margin'])):
        if candidates:
            diagnostic['zone_detection_reason'] = 'zone_candidates_ambiguous'
        elif diagnostic['red_pixel_count']:
            if rejected['weak_only']:
                diagnostic['zone_detection_reason'] = 'strong_red_anchor_missing'
            elif rejected['degenerate_polygon']:
                diagnostic['zone_detection_reason'] = 'degenerate_polygon'
            elif rejected['dense_red']:
                diagnostic['zone_detection_reason'] = 'red_area_too_dense'
            else:
                diagnostic['zone_detection_reason'] = 'red_components_too_small'
        if diagnostics is not None:
            diagnostics.update(diagnostic)
        return None
    _, polygon, selected_labels, complete = candidates[0]
    zone_mask, seed, selected_red = _zone_masks(polygon, red, settings)
    diagnostic.update(_red_tape_mask=selected_red, red_pixel_count=int(np.count_nonzero(selected_red)),
                      zone_merged_fragment_count=len(selected_labels),
                      zone_detection_reason='zone_found' if complete else 'zone_boundary_partial')
    if diagnostics is not None:
        diagnostics.update(diagnostic)
    return polygon, zone_mask, seed, complete


def _tracked_zone(frame, detected, diagnostics, state, observation_time, settings):
    """이전 바닥 특징점의 이동과 현재 빨간 선을 확인해 흔들림을 줄인 구역을 반환한다.

    state는 이번 검사에서만 쓰는 영상·구역 이력이다. 실제 특징점과 빨간 선의 근거가 부족하거나
    관측 공백이 길면 이전 위치를 쓰지 않는다. 로봇이 움직이면 측정한 영상 이동으로 경계를 옮긴다.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    previous_gray = state.get('gray')
    previous_polygon = state.get('polygon')
    previous_time = state.get('time')
    diagnostics['zone_tracking'] = 'new_detection'
    can_track = (previous_gray is not None and previous_gray.shape == gray.shape
                 and previous_polygon is not None and previous_time is not None
                 and 0 < observation_time - previous_time <= settings['geometry_max_gap_seconds'])
    tracked = None
    if can_track:
        mask = state.get('floor_mask')
        if mask is None:
            mask = state.get('zone_mask')
        if mask is not None and mask.shape == gray.shape:
            features = cv2.goodFeaturesToTrack(previous_gray, maxCorners=160, qualityLevel=0.01,
                                               minDistance=5, mask=mask.astype(np.uint8) * 255)
            if features is not None and len(features) >= settings['tracking_min_points']:
                moved, success, _ = cv2.calcOpticalFlowPyrLK(previous_gray, gray, features, None)
                if moved is not None and success is not None:
                    backwards, back_success, _ = cv2.calcOpticalFlowPyrLK(gray, previous_gray, moved, None)
                    if backwards is not None and back_success is not None:
                        valid = ((success.ravel() > 0) & (back_success.ravel() > 0)
                                 & (np.linalg.norm(backwards - features, axis=2).ravel() < 1.5))
                        if np.count_nonzero(valid) >= settings['tracking_min_points']:
                            matrix, inliers = cv2.findHomography(features[valid], moved[valid], cv2.RANSAC, 2.0)
                            if (matrix is not None and np.isfinite(matrix).all() and inliers is not None
                                    and np.count_nonzero(inliers) >= settings['tracking_min_points']
                                    and np.mean(inliers) >= settings['tracking_min_inlier_ratio']):
                                warped = cv2.perspectiveTransform(
                                    np.asarray(previous_polygon, np.float32).reshape(-1, 1, 2), matrix).reshape(-1, 2)
                                if (np.isfinite(warped).all() and cv2.isContourConvex(warped)
                                        and cv2.contourArea(warped) >= gray.size * settings['min_zone_area_ratio']):
                                    # 경계를 화면 밖으로 새로 추측하지 않고 현재 보이는 범위만 검사한다.
                                    bounds = np.array([[0, 0], [gray.shape[1]-1, 0],
                                                       [gray.shape[1]-1, gray.shape[0]-1],
                                                       [0, gray.shape[0]-1]], np.float32)
                                    area, clipped = cv2.intersectConvexConvex(warped, bounds)
                                    if clipped is not None and area >= gray.size * settings['min_zone_area_ratio']:
                                        polygon = np.rint(clipped.reshape(-1, 2)).astype(np.int32)
                                        _, red = _red_masks(frame, settings)
                                        old_red = state.get('red_mask')
                                        if old_red is not None:
                                            expected = cv2.warpPerspective(old_red.astype(np.uint8), matrix,
                                                                           (gray.shape[1], gray.shape[0]),
                                                                           flags=cv2.INTER_NEAREST) > 0
                                            nearby = cv2.dilate(red, np.ones((5, 5), np.uint8)) > 0
                                            support = np.mean(nearby[expected]) if np.any(expected) else 0.0
                                            diagnostics['tracking_red_support_ratio'] = float(support)
                                            if support >= settings['tracking_red_support_ratio']:
                                                zone_mask, seed, selected_red = _zone_masks(polygon, red, settings)
                                                touches_edge = np.any((polygon <= 1)
                                                                      | (polygon >= np.array(gray.shape[::-1]) - 2))
                                                complete = (len(polygon) == 4 and not touches_edge
                                                            and _boundary_supported(polygon, red, settings))
                                                tracked = polygon, zone_mask, seed, complete
    chosen = detected
    if tracked is not None:
        # 새로 보인 완전한 경계는 다시 잡고, 같은 구역의 색 검출 흔들림은 특징점 추적 결과를 쓴다.
        if detected is None or (_same_zone(detected[0], tracked[0])
                                and not (detected[3] and not tracked[3])):
            chosen = tracked
            diagnostics['zone_tracking'] = 'verified_optical_flow'
    if chosen is None:
        state.clear()
        return None
    polygon, zone_mask, seed, complete = chosen
    _, red = _red_masks(frame, settings)
    selected_red = (red > 0) & zone_mask
    if previous_polygon is not None and not _same_zone(polygon, previous_polygon):
        state.pop('plane', None)
    state.update(gray=gray, polygon=polygon.copy(), zone_mask=zone_mask,
                 red_mask=selected_red, time=observation_time)
    state.pop('floor_mask', None)
    diagnostics.update(_red_tape_mask=selected_red, red_pixel_count=int(np.count_nonzero(selected_red)),
                       zone_detection_reason='zone_found' if complete else 'zone_boundary_partial')
    return polygon, zone_mask, seed, complete


def _floor_fit_distance(settings):
    """평면을 찾는 허용 거리를 미터로 반환하며 장애물 최소 높이보다 작게 제한한다.

    바닥으로 칠하는 거리와 평면을 찾는 거리를 분리한다. 초기 설정(7cm×0.75)에서는 탐색 상한이 52.5mm다.
    """
    return max(settings['plane_distance_m'],
               settings['min_obstacle_height_m'] * settings['floor_max_height_ratio'])


def _plane_quality(plane, points, anchors, up, settings, diagnostics=None):
    """현재 점군으로 평면의 기울기·일치 비율·분포를 검증해 비율·잡음을 반환한다.

    실패하면 None이며 diagnostics에 실패 단계와 실제 수치를 넣는다. 이전 평면도 같은 검증을 받는다.
    """
    metrics = {'fit_reason': 'floor_points_insufficient', 'point_count': len(points), 'anchor_count': len(anchors),
               'fit_distance_m': _floor_fit_distance(settings), 'required_inlier_ratio': settings['min_plane_inlier_ratio'],
               'max_tilt_deg': settings['max_floor_tilt_deg'], 'required_span_m': settings['min_floor_span_m'] / 3}
    quality = None
    if len(points) >= settings['min_plane_points']:
        metrics['fit_reason'] = 'floor_anchors_insufficient'
        if len(anchors) >= settings['min_plane_points']:
            metrics['fit_reason'] = 'floor_plane_invalid'
            if plane is not None:
                plane = np.asarray(plane, dtype=float)
                if (plane.shape == (4,) and np.isfinite(plane).all()
                        and np.isclose(np.linalg.norm(plane[:3]), 1.0)):
                    tilt = float(np.rad2deg(np.arccos(np.clip(plane[:3] @ up, -1, 1))))
                    residual = points @ plane[:3] + plane[3]
                    anchor_residual = anchors @ plane[:3] + plane[3]
                    anchor_inliers = np.abs(anchor_residual) <= metrics['fit_distance_m']
                    fraction = float(np.mean(np.abs(residual) <= metrics['fit_distance_m']))
                    anchor_fraction = float(np.mean(anchor_inliers))
                    metrics.update(tilt_deg=tilt, inlier_ratio=fraction, anchor_inlier_ratio=anchor_fraction,
                                   anchor_residual_p90_m=float(np.percentile(np.abs(anchor_residual), 90)))
                    if tilt > settings['max_floor_tilt_deg']:
                        metrics['fit_reason'] = 'floor_plane_tilt_exceeded'
                    elif fraction < settings['min_plane_inlier_ratio']:
                        metrics['fit_reason'] = 'floor_plane_inliers_insufficient'
                    elif anchor_fraction < settings['min_plane_inlier_ratio']:
                        metrics['fit_reason'] = 'floor_anchor_inliers_insufficient'
                    else:
                        supported = anchors[anchor_inliers]
                        _, singular, _ = np.linalg.svd(supported - supported.mean(axis=0), full_matrices=False)
                        span = float(singular[1] / np.sqrt(len(supported))) if len(singular) > 1 else 0.0
                        metrics['span_m'] = span
                        metrics['fit_reason'] = 'floor_anchor_span_insufficient'
                        if span >= metrics['required_span_m']:
                            noise = float(1.4826 * np.median(np.abs(anchor_residual - np.median(anchor_residual))))
                            quality = fraction, noise
                            metrics.update(fit_reason='plane_fit_ok', noise_m=noise)
    if diagnostics is not None:
        diagnostics.update(metrics)
    return quality


def _stable_floor(points, anchors, up, state, settings, diagnostics=None):
    """현재 근거로 새 평면과 이전 평면을 검증해 안정된 평면·비율·잡음을 반환한다.

    diagnostics에는 최종 사용한 평면의 수치 또는 실패 사유를 보관한다. 없는 바닥을 이전 결과로 채우지 않는다.
    """
    current_diagnostics = {}
    current = _fit_floor(points, up, settings, anchor_points=anchors, diagnostics=current_diagnostics)
    previous = state.get('plane')
    previous_diagnostics = {}
    previous_quality = _plane_quality(previous, points, anchors, up, settings, previous_diagnostics)
    if current is None:
        if previous_quality is None:
            state.pop('plane', None)
            if diagnostics is not None:
                diagnostics.update(current_diagnostics)
            return None
        if diagnostics is not None:
            diagnostics.update(previous_diagnostics, plane_source='validated_previous_plane')
        return np.asarray(previous), *previous_quality
    plane, ratio, noise = current
    if previous_quality is not None:
        alpha = settings['plane_update_ratio']
        blended = np.asarray(previous) * (1 - alpha) + plane * alpha
        blended /= np.linalg.norm(blended[:3])
        blended_diagnostics = {}
        quality = _plane_quality(blended, points, anchors, up, settings, blended_diagnostics)
        if quality is not None:
            plane, (ratio, noise) = blended, quality
            current_diagnostics = dict(blended_diagnostics, plane_source='validated_blended_plane')
    state['plane'] = plane.copy()
    if diagnostics is not None:
        diagnostics.update(current_diagnostics)
    return plane, ratio, noise


def _fit_floor(points, up, settings, anchor_points=None, diagnostics=None):
    """경계 안쪽 바닥 근거에서 RANSAC 평면을 만들고 구역 전체로 검증해 평면·비율·잡음을 반환한다.

    points는 구역 전체 점, anchor_points는 경계에서 안쪽으로 들어온 점이다. 장애물 표면이
    평면을 끌어올리지 않도록 근거 점으로만 최종 평면을 다듬는다. 실패 단계와 수치는 diagnostics에 넣는다.
    """
    anchors = points if anchor_points is None else anchor_points
    metrics = {'fit_reason': 'floor_points_insufficient', 'point_count': len(points), 'anchor_count': len(anchors),
               'fit_distance_m': _floor_fit_distance(settings), 'required_inlier_ratio': settings['min_plane_inlier_ratio'],
               'max_tilt_deg': settings['max_floor_tilt_deg'], 'plane_source': 'ransac_inner_boundary'}
    if diagnostics is not None:
        diagnostics.update(metrics)
    if len(points) < settings['min_plane_points']:
        return None
    if len(anchors) < settings['min_plane_points']:
        if diagnostics is not None:
            diagnostics['fit_reason'] = 'floor_anchors_insufficient'
        return None
    # 같은 점군의 무작위 추출 순서를 일정하게 유지해 추출 운에 따른 깜빡임을 줄인다.
    rng = np.random.default_rng(0)
    samples = points if len(points) <= settings['max_plane_points'] else points[
        rng.choice(len(points), settings['max_plane_points'], replace=False)]
    anchor_samples = anchors if len(anchors) <= settings['max_plane_points'] else anchors[
        rng.choice(len(anchors), settings['max_plane_points'], replace=False)]
    cosine = np.cos(np.deg2rad(settings['max_floor_tilt_deg']))
    best, best_score, best_oriented, oriented_score, best_any, any_score = None, (-1, -1), None, (-1, -1), None, (-1, -1)
    for _ in range(settings['ransac_iterations']):
        a, b, c = anchor_samples[rng.choice(len(anchor_samples), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if normal @ up < 0:
            normal = -normal
        plane = np.r_[normal, -float(normal @ a)]
        point_count = int(np.count_nonzero(np.abs(samples @ normal + plane[3]) <= metrics['fit_distance_m']))
        anchor_count = int(np.count_nonzero(np.abs(anchor_samples @ normal + plane[3]) <= metrics['fit_distance_m']))
        score = point_count, anchor_count
        if score > any_score:
            best_any, any_score = plane, score
        if normal @ up < cosine:
            continue
        if score > oriented_score:
            best_oriented, oriented_score = plane, score
        if (point_count / len(samples) < settings['min_plane_inlier_ratio']
                or anchor_count / len(anchor_samples) < settings['min_plane_inlier_ratio']):
            continue
        if score > best_score:
            best, best_score = plane, score
    if best is None:
        candidate = best_oriented if best_oriented is not None else best_any
        if candidate is None:
            metrics['fit_reason'] = 'floor_samples_degenerate'
        else:
            _plane_quality(candidate, points, anchors, up, settings, metrics)
            if metrics['fit_reason'] == 'plane_fit_ok':
                metrics['fit_reason'] = 'floor_sample_support_insufficient'
        if diagnostics is not None:
            diagnostics.update(metrics)
        return None
    plane = best
    # 경계 안쪽의 일치하는 근거 점으로만 다시 맞추고 새 평면을 같은 조건으로 검증한다.
    for _ in range(3):
        selected = anchors[np.abs(anchors @ plane[:3] + plane[3]) <= metrics['fit_distance_m']]
        if len(selected) < settings['min_plane_points']:
            if diagnostics is not None:
                diagnostics.update(metrics, fit_reason='floor_anchors_insufficient')
            return None
        center = selected.mean(axis=0)
        _, _, vectors = np.linalg.svd(selected - center, full_matrices=False)
        normal = vectors[-1]
        if normal @ up < 0:
            normal = -normal
        plane = np.r_[normal, -float(normal @ center)]
    quality = _plane_quality(plane, points, anchors, up, settings, metrics)
    if diagnostics is not None:
        diagnostics.update(metrics)
    if quality is None:
        return None
    return plane, *quality


def _inside_polygon(points, polygon, tolerance=0.0):
    """바닥의 2D 점들과 볼록 구역 다각형을 받아 경계를 포함한 구역 내부 여부 배열을 반환한다."""
    area = np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1) - polygon[:, 1] * np.roll(polygon[:, 0], -1))
    inside = np.ones(len(points), dtype=bool)
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = end - start
        cross = edge[0] * (points[:, 1] - start[1]) - edge[1] * (points[:, 0] - start[0])
        if area < 0:
            cross = -cross
        inside &= cross >= -tolerance * np.linalg.norm(edge)
    return inside


def _cluster_points(points, pixels, floor_xy, settings):
    """바닥 위 후보를 미터 단위 3D 격자 이웃으로 묶어 크기가 충분한 장애물 목록을 반환한다.

    pixels는 후보의 원본 영상 xy 좌표이고 floor_xy는 바닥으로 투영한 2D 미터 좌표다.
    작은 고립 잡음은 제외하며 원본 해상도의 후보를 사용해 낮은 물체를 보존한다.
    """
    if not len(points):
        return []
    cells, inverse = np.unique(np.floor(points / settings['cluster_voxel_m']).astype(np.int64),
                               axis=0, return_inverse=True)
    by_cell = {tuple(cell): index for index, cell in enumerate(cells)}
    parent = np.arange(len(cells))

    def root(index):
        """격자 묶음 번호를 받아 최상위 묶음 번호를 찾고 다음 탐색을 빠르게 만든다."""
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return int(index)

    for index, cell in enumerate(cells):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    neighbor = by_cell.get((cell[0] + dx, cell[1] + dy, cell[2] + dz))
                    if neighbor is not None and neighbor < index:
                        parent[root(index)] = root(neighbor)
    roots = np.array([root(index) for index in range(len(cells))])[inverse]
    obstacles = []
    for group in np.unique(roots):
        selected = roots == group
        if np.count_nonzero(selected) < settings['min_cluster_points']:
            continue
        hull = cv2.convexHull(floor_xy[selected].astype(np.float32))
        area = float(cv2.contourArea(hull))
        if area < settings['min_obstacle_area_m2']:
            continue
        corner_min, corner_max = pixels[selected].min(axis=0), pixels[selected].max(axis=0)
        obstacles.append({'bbox': [int(corner_min[0]), int(corner_min[1]), int(corner_max[0])+1, int(corner_max[1])+1],
                          'point_count': int(np.count_nonzero(selected)), 'footprint_area_m2': area,
                          'indices': np.flatnonzero(selected)})
    return obstacles


def _coverage(floor_xy, polygon, settings):
    """바닥에서 관측한 2D 점과 검사 다각형을 받아 미터 단위 격자의 관측 비율을 반환한다.

    격자가 너무 많거나 유효 구역이 없으면 None을 반환해 관측 범위를 확정하지 않는다.
    """
    cell_size = settings['coverage_cell_m']
    low = np.floor(polygon.min(axis=0) / cell_size).astype(np.int64)
    high = np.ceil(polygon.max(axis=0) / cell_size).astype(np.int64)
    shape = high - low + 1
    if np.any(shape <= 0) or float(shape[0]) * float(shape[1]) > settings['max_coverage_cells']:
        return None
    ys, xs = np.indices((int(shape[1]), int(shape[0])))
    centers = (np.column_stack((xs.ravel(), ys.ravel())) + low + 0.5) * cell_size
    needed = _inside_polygon(centers, polygon)
    if not np.any(needed):
        return None
    visible = np.zeros((int(shape[1]), int(shape[0])), dtype=bool)
    if len(floor_xy):
        indices = np.floor(floor_xy / cell_size).astype(np.int64) - low
        valid = np.all((indices >= 0) & (indices < shape), axis=1)
        visible[indices[valid, 1], indices[valid, 0]] = True
    return float(np.mean(visible.ravel()[needed]))


def inspect(frame, depth_m, rays, up, *, geometry_state=None, observation_time=None, **options):
    """원본 RGB·미터 Depth·보정된 픽셀 방향·TF 위쪽 방향으로 구역 안의 장애물 판정 사전을 반환한다.

    RGB와 Depth의 정렬·시각 일치 검증은 매니저가 먼저 수행해야 한다.
    RANSAC 바닥 추정에는 줄인 점군을, 장애물 판독에는 원본 해상도의 점군을 사용한다.
    바닥 평면은 빨간 경계 안쪽의 유효 Depth로만 추정한다.
    바깥 경계에서 안쪽으로 들어온 근거로 평면을 구하고 구역 전체 점으로 검증한다.
    평면 탐색 거리와 장애물 최소 높이를 분리하며 실패 단계와 수치를 floor_diagnostics로 반환한다.
    geometry_state와 Depth 수신 시각 observation_time을 주면 실제 영상 근거로 구역·평면을 추적한다.
    바닥 분류 거리는 측정 오차를 반영하며 최소 장애물 높이보다 작게 제한한다.
    그 거리 안에 들어온 구역 안의 유효 픽셀만 빨간 바닥 표시 마스크에 보관한다.
    장애물은 화면의 검사 구역 안에 있고 바닥에 투영한 실제 위치도 구역 안일 때만 판정한다.
    장애물 최소 높이는 잡음에 따라 올리지 않는다. 잡음이 크면 정상 확정을 보류한다.
    구역을 못 찾은 경우에도 빨간 선의 인식 마스크와 실패 단계를 반환한다.
    구역 안 Depth 값의 계단 간격(depth_step_m)도 반환해 스테레오 Depth의 해상도 문제를 구분한다.
    """
    settings = validate_options(**options)
    report = unconfirmed('zone_not_found')
    if frame.ndim != 3 or depth_m.shape != frame.shape[:2] or rays.shape != (*depth_m.shape, 3):
        return unconfirmed('rgb_depth_size_mismatch')
    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) <= 0:
        return unconfirmed('invalid_ground_direction')
    up /= np.linalg.norm(up)
    geometry = {} if geometry_state is None else geometry_state
    if observation_time is not None:
        previous_time = geometry.get('time')
        signature = (rays.shape, tuple(rays[0, 0]), tuple(rays[-1, -1]))
        if (geometry.get('ray_signature') != signature or (previous_time is not None
                and not 0 < observation_time - previous_time <= settings['geometry_max_gap_seconds'])):
            geometry.clear()
        geometry['ray_signature'] = signature
    zone_diagnostics = {}
    zone = _find_zone(frame, settings, diagnostics=zone_diagnostics)
    if observation_time is not None:
        zone = _tracked_zone(frame, zone, zone_diagnostics, geometry, observation_time, settings)
    report.update(zone_diagnostics)
    if zone is None:
        return report
    polygon, zone_mask, seed_mask, complete = zone
    report.update(zone_polygon=polygon.tolist(), zone_complete=bool(complete),
                  _zone_mask=zone_mask, zone_pixel_count=int(np.count_nonzero(zone_mask)))
    valid = np.isfinite(depth_m) & (depth_m >= settings['depth_min_m']) & (depth_m <= settings['depth_max_m'])
    report['valid_depth_ratio'] = float(np.mean(valid[zone_mask]))
    report['depth_step_m'] = _depth_step(depth_m[valid & zone_mask])
    points = rays * np.where(valid, depth_m, np.nan)[:, :, None]
    floor_seed = valid & zone_mask & seed_mask
    floor_points = valid & zone_mask
    report['floor_fit_point_count'] = int(np.count_nonzero(floor_points))
    report['floor_anchor_point_count'] = int(np.count_nonzero(floor_seed))
    floor_diagnostics = {}
    floor = _stable_floor(points[floor_points], points[floor_seed], up, geometry, settings, floor_diagnostics)
    report.update(floor_diagnostics=floor_diagnostics, ground_up_camera=up.tolist())
    if floor is None:
        report['reason'] = 'floor_not_confirmed'
        return report
    plane, ratio, noise = floor
    normal, offset = plane[:3], plane[3]
    report.update(plane=plane.tolist(), floor_inlier_ratio=ratio, floor_noise_m=noise)
    # 허용값을 잡음에 맞춰 늘리되 목표 장애물 높이까지 바닥으로 흡수하지 않게 상한을 둔다.
    floor_distance_cap = max(settings['plane_distance_m'],
                             settings['min_obstacle_height_m'] * settings['floor_max_height_ratio'])
    floor_distance = min(settings['plane_distance_m'] + settings['floor_noise_multiplier'] * noise,
                         floor_distance_cap)
    report['floor_classify_distance_m'] = float(floor_distance)
    ys, xs = np.nonzero(valid)
    available = points[valid]
    heights = available @ normal + offset
    # 같은 평면에 있는 구역 밖의 바닥은 제외하고, 경계 안의 실제 바닥 픽셀만 표시한다.
    floor_inliers = zone_mask[ys, xs] & (np.abs(heights) <= floor_distance)
    floor_mask = np.zeros(depth_m.shape, dtype=bool)
    floor_mask[ys[floor_inliers], xs[floor_inliers]] = True
    floor_count = int(np.count_nonzero(floor_inliers))
    report.update(_floor_inlier_mask=floor_mask, floor_inlier_count=floor_count,
                  floor_pixel_ratio=float(floor_count / report['zone_pixel_count']))
    geometry['floor_mask'] = floor_mask
    corner_rays = rays[polygon[:, 1], polygon[:, 0]]
    denominators = corner_rays @ normal
    if np.any(np.abs(denominators) < 1e-6):
        report['reason'] = 'zone_plane_intersection_invalid'
        return report
    lengths = -offset / denominators
    if not np.isfinite(lengths).all() or np.any(lengths <= 0):
        report['reason'] = 'zone_plane_intersection_invalid'
        return report
    world_polygon = corner_rays * lengths[:, None]
    origin = world_polygon[0]
    axis_x = world_polygon[1] - origin
    if np.linalg.norm(axis_x) < 1e-6:
        report['reason'] = 'zone_plane_intersection_invalid'
        return report
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    basis = np.column_stack((axis_x, axis_y))
    floor_polygon = (world_polygon - origin) @ basis
    projections = available - heights[:, None] * normal
    floor_xy = (projections - origin) @ basis
    inside = _inside_polygon(floor_xy, floor_polygon, settings['boundary_tolerance_m'])
    effective_height = settings['min_obstacle_height_m']
    report['effective_height_m'] = effective_height
    candidates = zone_mask[ys, xs] & inside & (heights >= effective_height)
    obstacles = _cluster_points(available[candidates], np.column_stack((xs, ys))[candidates],
                                floor_xy[candidates], settings)
    for obstacle in obstacles:
        member_heights = heights[candidates][obstacle.pop('indices')]
        obstacle['height_p90_m'] = float(np.percentile(member_heights, 90))
    report['obstacles'] = obstacles
    observed_floor = inside & floor_inliers
    report['coverage_ratio'] = _coverage(floor_xy[observed_floor], floor_polygon, settings)
    if obstacles:
        report.update(status='ABNORMAL', reason='obstacle_above_floor_inside_zone')
    elif not complete:
        report['reason'] = 'zone_partial'
    elif report['valid_depth_ratio'] < settings['min_valid_depth_ratio']:
        report['reason'] = 'insufficient_depth'
    elif settings['plane_distance_m'] + settings['floor_noise_multiplier'] * noise >= effective_height:
        report['reason'] = 'floor_noise_exceeds_target_height'
    elif report['coverage_ratio'] is None or report['coverage_ratio'] < settings['min_coverage_ratio']:
        report['reason'] = 'insufficient_floor_coverage'
    else:
        report.update(status='NORMAL', reason='observed_zone_has_no_qualifying_obstacle')
    return report


def reset_stability_state(state, keep_geometry=False):
    """연속 판정 이력을 지운다. keep_geometry가 참이면 현재 영상으로 검증할 기하 이력만 남긴다.

    검사 시작·종료·영상 만료 시에는 기본값으로 호출해 구역과 평면 이력도 모두 지운다.
    """
    geometry = state.get('geometry') if keep_geometry else None
    state.clear()
    if geometry is not None:
        state['geometry'] = geometry


def _same_zone(first, second):
    """현재와 이전 구역 다각형의 겹침을 확인해 서로 다른 구역의 확인 이력을 섞지 않는다."""
    if first is None or second is None:
        return False
    a, b = np.asarray(first, np.float32), np.asarray(second, np.float32)
    area_a, area_b = cv2.contourArea(a), cv2.contourArea(b)
    intersection, _ = cv2.intersectConvexConvex(a, b)
    union = area_a + area_b - intersection
    return union > 0 and intersection / union >= 0.5


def stabilize_report(report, state, observation_time, observation_stamp, **options):
    """현재 판독과 이력을 받아 같은 구역의 서로 다른 Depth 사진으로 확인한 판정을 반환한다.

    observation_time은 Depth 수신 시각이고 observation_stamp는 촬영 시각이다.
    같은 Depth를 두 번 사용해도 확인 횟수를 늘리지 않는다. 품질 실패나 긴 관측 공백은 이전 확정을 지운다.
    경고 후보의 위치가 크게 바뀌어도 확인을 다시 시작하여 흩어진 잡점을 같은 장애물로 세지 않는다.
    """
    settings = validate_options(**options)
    raw = report['status']
    output = dict(report, raw_status=raw, raw_reason=report['reason'])
    previous_stamp = state.get('stamp')
    if (raw not in ('NORMAL', 'ABNORMAL') or not _same_zone(report['zone_polygon'], state.get('polygon'))
            or (previous_stamp is not None and observation_stamp < previous_stamp)
            or (state.get('last_time') is not None
                and observation_time - state['last_time'] > settings['confirm_max_gap_seconds'])):
        reset_stability_state(state, keep_geometry=True)
    if raw not in ('NORMAL', 'ABNORMAL'):
        return output
    if raw == 'ABNORMAL' and state.get('raw_status') == 'ABNORMAL':
        current_boxes = [item['bbox'] for item in report['obstacles']]
        previous_boxes = state.get('obstacle_boxes', [])
        if not _overlapping_obstacles(current_boxes, previous_boxes):
            reset_stability_state(state, keep_geometry=True)
    duplicate = state.get('stamp') == observation_stamp
    if raw != state.get('raw_status'):
        state.update(raw_status=raw, count=0, since=observation_time, confirmed=False)
    if not duplicate:
        state['count'] = state.get('count', 0) + 1
        state['last_time'] = observation_time
    elapsed = max(0.0, state.get('last_time', observation_time) - state.get('since', observation_time))
    state['confirmed'] = (state.get('count', 0) >= settings['confirm_frames'] and elapsed >= settings['confirm_seconds'])
    state.update(stamp=observation_stamp, polygon=report['zone_polygon'],
                 obstacle_boxes=[item['bbox'] for item in report['obstacles']])
    output.update(confirm_frames=state.get('count', 0), confirm_elapsed_s=elapsed,
                  required_confirm_frames=settings['confirm_frames'], required_confirm_seconds=settings['confirm_seconds'])
    if not state['confirmed']:
        output.update(status='UNCONFIRMED', reason='confirmation_pending')
    return output


def _overlapping_obstacles(current_boxes, previous_boxes):
    """현재와 이전 장애물 사각형 중 겹침이 충분한 쌍이 있는지 참·거짓으로 반환한다.

    사각형은 [왼쪽, 위, 오른쪽, 아래] 좌표다. 위치가 다른 잡점을 하나의 연속 후보로 세지 않는다.
    """
    for current in current_boxes:
        a = np.asarray(current, dtype=float)
        for previous in previous_boxes:
            b = np.asarray(previous, dtype=float)
            intersection = np.prod(np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])))
            union = np.prod(a[2:] - a[:2]) + np.prod(b[2:] - b[:2]) - intersection
            if union > 0 and intersection / union >= 0.20:
                return True
    return False


@lru_cache(maxsize=32)
def load_font(font_path='', size=24):
    """지정한 경로 또는 설치된 한글 글꼴에서 폰트를 읽어 반환한다. 없으면 OSError로 알린다."""
    paths = [font_path] if font_path else [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc']
    for name in paths:
        if name and Path(name).is_file():
            return ImageFont.truetype(name, size=int(size), index=1 if name.endswith('.ttc') else 0)
    raise OSError('Korean font not found; set shutter_font_path')


def draw_result(frame, report, *, font_path='', font_size=24):
    """표시용 영상의 좌측 상단에 요청한 한글 정상·경고 문구를 그리고 새 BGR 영상을 반환한다.

    빨간 경계 안에서 실제 평면에 가까운 바닥 픽셀만 반투명 빨강으로 칠하고, 검출 수와 사유를 표시한다.
    장애물은 확정 시 빨간 박스, 확인 중에는 주황 박스다. 후보 높이도 표시한다.
    구역은 판정에 맞는 색이며 바닥을 못 찾으면 바닥을 칠하지 않는다.
    RGB와 Depth의 촬영 시각이 맞지 않으면 실제 차이와 허용 시간을 함께 표시한다.
    stream_timing이 있으면 RGB·Depth 발행 주기, 위상차, Depth 추가 지연, 짝 성공률도 표시한다.
    구역을 찾으면 해당 구역의 빨간 선만 노란 윤곽으로 그리고, 구역 후보 수와 실패 단계도 표시한다.
    바닥으로 확인된 비율·유효 Depth 비율·바닥 분류 거리를 표시해 빈 부분의 근거를 확인할 수 있게 한다.
    평면을 못 찾으면 실제 실패 단계와 후보 기울기·구역/근거 일치 비율을 표시한다.
    카메라 정렬 자동 조회 대기와 실제 정렬 꺼짐·다른 카메라 지정은 서로 다른 안내로 표시한다.
    """
    status = report['status']
    color = {'ABNORMAL': (0, 0, 255), 'NORMAL': (0, 255, 0), 'UNCONFIRMED': (180, 180, 180)}[status]
    output = frame.copy()
    zone_mask = report.get('_zone_mask')
    floor_mask = report.get('_floor_inlier_mask')
    if floor_mask is not None:
        floor_mask = np.asarray(floor_mask, dtype=bool)
        if (floor_mask.shape == output.shape[:2] and zone_mask is not None
                and zone_mask.shape == output.shape[:2]):
            floor_mask = floor_mask & zone_mask
            # 경계 안의 바닥에만 빨강을 40% 섞고 구역 밖의 색은 보존한다.
            output[floor_mask] = np.rint(output[floor_mask].astype(np.float32) * 0.6
                                        + np.array((0, 0, 255), dtype=np.float32) * 0.4).astype(np.uint8)
    red_mask = report.get('_red_tape_mask')
    if red_mask is not None and red_mask.shape == output.shape[:2]:
        if zone_mask is not None and zone_mask.shape == output.shape[:2]:
            red_mask = red_mask & zone_mask
        contours, _ = cv2.findContours(np.asarray(red_mask, dtype=np.uint8) * 255,
                                      cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(output, contours, -1, (0, 255, 255), 1, cv2.LINE_AA)
    if report.get('zone_polygon') is not None:
        cv2.polylines(output, [np.asarray(report['zone_polygon'], np.int32)], True, color, 2, cv2.LINE_AA)
    for obstacle in report.get('obstacles', []):
        x1, y1, x2, y2 = obstacle['bbox']
        obstacle_color = (0, 0, 255) if status == 'ABNORMAL' else (0, 165, 255)
        cv2.rectangle(output, (x1, y1), (x2, y2), obstacle_color, 2)
        if obstacle.get('height_p90_m') is not None:
            cv2.putText(output, f"{obstacle['height_p90_m'] * 1000:.0f}mm", (x1, max(12, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, obstacle_color, 1, cv2.LINE_AA)
    label = STATUS_MESSAGES[status]
    if status == 'UNCONFIRMED':
        labels = {'zone_partial': '미확인)방화셔터 구역 전체를 보여주십시오',
                  'depth_missing': '미확인)Depth 영상을 기다리고 있습니다',
                  'camera_info_missing': '미확인)카메라 보정 정보를 기다리고 있습니다',
                  'depth_alignment_unconfirmed': '미확인)RGB와 Depth 정렬을 확인해 주십시오',
                  'depth_alignment_check_pending': '미확인)카메라 정렬 설정을 자동 확인 중입니다',
                  'depth_alignment_disabled': '미확인)카메라의 Depth 정렬 설정이 꺼져 있습니다',
                  'depth_alignment_target_mismatch': '미확인)Depth 정렬 대상이 RGB 카메라와 다릅니다',
                  'rgb_depth_time_mismatch': '미확인)RGB와 Depth의 촬영 시각이 맞지 않습니다',
                  'rgb_depth_pair_stale': '미확인)촬영 시각이 맞는 RGB와 Depth를 기다리고 있습니다',
                  'capture_pair_stale': '미확인)카메라 영상의 촬영 시각이 지연되고 있습니다',
                  'rgb_depth_pair_unavailable': '미확인)새 RGB와 Depth 쌍을 기다리고 있습니다',
                  'zone_not_found': '미확인)빨간 검사 구역을 찾지 못했습니다',
                  'floor_not_confirmed': '미확인)RANSAC 바닥 평면을 찾지 못했습니다',
                  'confirmation_pending': '미확인)장애물 상태를 확인 중입니다'}
        label = labels.get(report['reason'], label)
    floor_label = (f"검출 ({report.get('floor_inlier_count', 0)}픽셀)"
                   if report.get('plane') is not None else '미검출')
    if report['reason'] == 'rgb_depth_time_mismatch':
        floor_label = '시각 불일치로 대기'
    elif report['reason'] == 'zone_not_found':
        floor_label = '구역 검출 대기'
    diagnostic = f"RANSAC 바닥: {floor_label} | reason={report.get('reason', 'unknown')}"
    fit = report.get('floor_diagnostics', {})
    if report.get('reason') == 'floor_not_confirmed' and fit.get('fit_reason'):
        diagnostic = f"RANSAC 바닥: {floor_label} | reason={fit['fit_reason']}"
    canvas = Image.fromarray(cv2.cvtColor(output, cv2.COLOR_BGR2RGB))
    drawer = ImageDraw.Draw(canvas)
    lines = []
    banner_height = 8
    banner_lines = [(label, tuple(reversed(color))), (diagnostic, (220, 220, 220))]
    if report.get('rgb_depth_time_diff_s') is not None:
        timing = (f"RGB와 Depth 시각 차이: {report['rgb_depth_time_diff_s']:.3f}초 / "
                  f"허용: {report['rgb_depth_sync_tolerance_s']:.3f}초")
        banner_lines.append((timing, (220, 220, 220)))
    timing = report.get('stream_timing')
    if timing and timing.get('depth_hz') is not None:
        def number(value, scale=1.0, digits=1):
            """없는 값은 '-'로, 있는 값은 배율을 곱해 정해진 자릿수 문자열로 바꾼다."""
            return '-' if value is None else f'{value * scale:.{digits}f}'
        timing_text = (f"주기 RGB {number(timing.get('rgb_hz'))}Hz / Depth {number(timing['depth_hz'])}Hz | "
                       f"위상차 {number(timing.get('phase_median_ms'))}ms "
                       f"(허용 내 {number(timing.get('matched_ratio'), 100, 0)}%) | "
                       f"Depth 추가 지연 {number(timing.get('depth_extra_lag_s'), 1.0, 2)}초 | "
                       f"짝 성공 {number(timing.get('pair_success_ratio'), 100, 0)}%")
        banner_lines.append((timing_text, (220, 220, 220)))
    if report.get('red_pixel_count') is not None:
        zone_labels = {'red_pixels_missing': '빨간 픽셀 없음',
                       'strong_red_anchor_missing': '진한 빨간 선 근거 부족',
                       'red_components_too_small': '선 조각의 구역 면적 부족',
                       'red_area_too_dense': '빨간 면적이 너무 큼',
                       'degenerate_polygon': '구역 모양이 너무 얇음',
                       'zone_candidates_ambiguous': '여러 구역 후보가 비슷함',
                       'zone_boundary_partial': '일부 구역 검출',
                       'zone_found': '전체 구역 검출'}
        zone_reason = report.get('zone_detection_reason')
        tape_info = (f"빨간 선: {report['red_pixel_count']}픽셀 | "
                     f"구역 후보: {report.get('zone_candidate_count', 0)} | "
                     f"결합 조각: {report.get('zone_merged_fragment_count', 0)} | "
                     f"{zone_labels.get(zone_reason, zone_reason or '확인 중')}")
        banner_lines.append((tape_info, (255, 255, 0)))
    if report.get('valid_depth_ratio') is not None:
        floor_ratio = report.get('floor_pixel_ratio')
        floor_distance = report.get('floor_classify_distance_m')
        floor_value = '확인 전' if floor_ratio is None else f'{floor_ratio * 100:.1f}%'
        distance_value = '확인 전' if floor_distance is None else f'{floor_distance * 1000:.1f}mm'
        noise = report.get('floor_noise_m')
        step = report.get('depth_step_m')
        floor_info = (f"바닥 확인: {floor_value} / 유효 Depth: {report['valid_depth_ratio'] * 100:.1f}% / "
                      f"바닥 분류 거리: {distance_value} / 잡음: "
                      f"{'확인 전' if noise is None else f'{noise * 1000:.1f}mm'} / Depth 단계: "
                      f"{'확인 전' if step is None else f'{step * 1000:.1f}mm'}")
        banner_lines.append((floor_info, (220, 220, 220)))
    if fit:
        tilt = fit.get('tilt_deg')
        inliers = fit.get('inlier_ratio')
        anchor_inliers = fit.get('anchor_inlier_ratio')
        tilt_text = '후보 없음' if tilt is None else f"{tilt:.1f}/{fit['max_tilt_deg']:.1f}도"
        inlier_text = '확인 전' if inliers is None else f"{inliers * 100:.1f}%"
        anchor_text = '확인 전' if anchor_inliers is None else f"{anchor_inliers * 100:.1f}%"
        fit_info = (f"평면: 기울기 {tilt_text} | 구역 {inlier_text} / 근거 {anchor_text} "
                    f"(필요 {fit['required_inlier_ratio'] * 100:.0f}%) | "
                    f"근거 {fit['anchor_count']}점 / 탐색 {fit['fit_distance_m'] * 1000:.1f}mm")
        banner_lines.append((fit_info, (220, 220, 220)))
    if report.get('confirm_frames') is not None:
        progress = (f"확인: {report['confirm_frames']}/{report['required_confirm_frames']}프레임, "
                    f"{report['confirm_elapsed_s']:.2f}/{report['required_confirm_seconds']:.2f}초 | "
                    f"구역: {report.get('zone_tracking', 'new_detection')}")
        banner_lines.append((progress, (220, 220, 220)))
    for text, text_color in banner_lines:
        size = int(font_size)
        font = load_font(font_path, size)
        while size > 8 and drawer.textlength(text, font=font) > output.shape[1] - 24:
            size -= 1
            font = load_font(font_path, size)
        bounds = drawer.textbbox((0, 0), text, font=font)
        lines.append((text, text_color, font, banner_height - bounds[1]))
        banner_height += bounds[3] - bounds[1] + 8
    banner_height = min(output.shape[0], banner_height)
    drawer.rectangle((0, 0, output.shape[1], banner_height), fill=(15, 15, 15))
    for text, text_color, font, y in lines:
        drawer.text((12, y), text, font=font, fill=text_color)
    return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)

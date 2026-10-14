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
        box[330:380, 280:360] = True
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

"""소화기 압력계의 정면 여부와 정상·비정상을 판정하는 함수 모음."""

import cv2
import numpy as np
from scv_msgs.msg import InspectionResult

PRESSURE_CLASSES = {"normal", "abnormal_left", "abnormal_right", "pre_good", "good", "bad"}


def supports(class_name):
    """이 클래스 이름이 소화기 압력계에 해당하는지 알려준다."""
    return class_name.strip().lower() in PRESSURE_CLASSES


def check_pressure_frontality(frame, box, min_size=48, min_axis_ratio=0.90):
    """압력계 테두리가 원에 가까운지 검사한다. 작거나 테두리가 불분명하면 판단을 보류한다."""
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(value) for value in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0 or min(crop.shape[:2]) < min_size:
        return None, 'gauge too small or outside image'
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 120)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    candidates = []
    h, w = crop.shape[:2]
    for contour in contours:
        if len(contour) < 20:
            continue
        try:
            (cx, cy), axes, _ = cv2.fitEllipse(contour)
        except cv2.error:
            continue
        minor, major = sorted(axes)
        if not (0.5 * min(h, w) <= major <= 1.2 * max(h, w)):
            continue
        if abs(cx - w / 2) > 0.2 * w or abs(cy - h / 2) > 0.2 * h:
            continue
        area = np.pi * minor * major / 4
        if area <= 0 or cv2.contourArea(contour) / area < 0.80:
            continue
        candidates.append((area, minor / major))
    if not candidates:
        return None, 'gauge outline unclear or occluded'
    _, ratio = max(candidates)
    return ratio >= min_axis_ratio, f'outline axis ratio={ratio:.3f}'


def class_to_verdict(class_name, frontal=None):
    """정면 압력계만 정상·이상으로 바꾸고 그 외에는 판정을 보류한다."""
    name = class_name.strip().lower()
    if not supports(name) or frontal is not True:
        return InspectionResult.UNCONFIRMED
    return InspectionResult.NORMAL if name in {'normal', 'good', 'pre_good'} else InspectionResult.ABNORMAL


def assess(class_name, frame, box, min_size=48, min_axis_ratio=0.90):
    """압력계 사진과 박스를 받아 정면 여부·판정·이유를 함께 반환한다."""
    frontal, reason = check_pressure_frontality(frame, box, min_size, min_axis_ratio)
    return {'verdict': class_to_verdict(class_name, frontal), 'frontal': frontal, 'reason': reason}

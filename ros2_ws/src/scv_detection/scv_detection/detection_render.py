"""설비별 함수 모듈을 연결하고 판정 결과를 바운딩 박스로 그리는 공통 함수."""

import numpy as np
from scv_msgs.msg import InspectionResult
from ultralytics.utils.plotting import Annotator

from scv_detection import pressure_gauge, exit_sign

# 새 설비 모듈에 supports·assess 함수를 만들고 여기에 추가하면 manager는 그대로 사용한다.
DETECTORS = (pressure_gauge, exit_sign)
VERDICT_COLORS = {
    InspectionResult.NORMAL: (0, 255, 0),
    InspectionResult.ABNORMAL: (0, 0, 255),
    InspectionResult.UNCONFIRMED: (192, 192, 192),
}


def class_to_verdict(class_name, frontal=None):
    """클래스 이름에 맞는 설비 모듈에 정상·이상 변환을 맡긴다."""
    if pressure_gauge.supports(class_name):
        return pressure_gauge.class_to_verdict(class_name, frontal)
    return exit_sign.class_to_verdict(class_name)


def assess_detections(result, min_size=48, min_axis_ratio=0.90):
    """검출된 물체마다 담당 설비 모듈을 찾아 판정과 이유를 순서대로 모은다."""
    assessments = []
    if result.boxes is not None:
        for box in result.boxes:
            name = result.names[int(box.cls.item())]
            assessment = {'verdict': InspectionResult.UNCONFIRMED,
                          'frontal': None, 'reason': 'unsupported class'}
            for detector in DETECTORS:
                if detector.supports(name):
                    assessment = detector.assess(
                        name, result.orig_img, box.xyxy[0].cpu().tolist(), min_size, min_axis_ratio)
                    break
            assessments.append(assessment)
    return assessments


def annotate_detections(result, assessments=None) -> np.ndarray:
    """원본은 보존하고 정상은 초록, 이상은 빨강, 판정 보류는 회색으로 그린다."""
    if assessments is None:
        assessments = assess_detections(result)
    annotator = Annotator(result.orig_img.copy(), example=str(result.names))
    if result.boxes is not None:
        for box, assessment in reversed(list(zip(result.boxes, assessments))):
            class_name = result.names[int(box.cls.item())]
            color = VERDICT_COLORS.get(assessment['verdict'], (192, 192, 192))
            label = f"{class_name} {float(box.conf.item()):.2f}"
            if assessment['verdict'] == InspectionResult.UNCONFIRMED:
                label = 'UNCONFIRMED (pose/outline)'
            annotator.box_label(box.xyxy[0].cpu().tolist(), label, color=color)
    return annotator.result()

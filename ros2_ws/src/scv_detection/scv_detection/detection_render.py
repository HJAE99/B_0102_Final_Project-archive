"""YOLO가 검출한 모든 클래스의 이름·신뢰도·상태 색상을 표시하는 함수 모음."""

import numpy as np
from scv_msgs.msg import InspectionResult
from ultralytics.utils.plotting import Annotator

NORMAL_CLASSES = {'normal', 'good', 'pre_good', 'exit_sign_on'}
ABNORMAL_CLASSES = {'abnormal_left', 'abnormal_right', 'bad', 'exit_sign_off'}
VERDICT_COLORS = {
    InspectionResult.NORMAL: (0, 255, 0),
    InspectionResult.ABNORMAL: (0, 0, 255),
    InspectionResult.UNCONFIRMED: (192, 192, 192),
}


def class_to_verdict(class_name):
    """학습 라벨을 상태로 바꾼다. 전체샷처럼 상태를 뜻하지 않는 라벨은 정상으로 단정하지 않는다."""
    name = class_name.strip().lower()
    if name in NORMAL_CLASSES:
        return InspectionResult.NORMAL
    if name in ABNORMAL_CLASSES:
        return InspectionResult.ABNORMAL
    return InspectionResult.UNCONFIRMED


def assess_detections(result):
    """모델이 반환한 모든 박스의 상태를 같은 순서로 돌려준다. 새 클래스도 제외하지 않는다."""
    assessments = []
    if result.boxes is not None:
        for box in result.boxes:
            name = result.names[int(box.cls.item())]
            verdict = class_to_verdict(name)
            assessments.append({
                'verdict': verdict, 'frontal': None,
                'reason': ('class has no normal/abnormal mapping'
                           if verdict == InspectionResult.UNCONFIRMED
                           else 'YOLO class prediction'),
            })
    return assessments


def annotate_detections(result, assessments=None) -> np.ndarray:
    """모든 박스에 실제 이름과 신뢰도를 쓴다. 정상은 초록, 이상은 빨강, 기타는 회색이며 원본은 보존한다."""
    if assessments is None:
        assessments = assess_detections(result)
    # 상태 목록이 짧아도 일부 검출이 사라지지 않도록 다시 만든다.
    if result.boxes is not None and len(assessments) != len(result.boxes):
        assessments = assess_detections(result)
    annotator = Annotator(result.orig_img.copy(), example=str(result.names))
    if result.boxes is not None:
        for box, assessment in reversed(list(zip(result.boxes, assessments))):
            class_name = result.names[int(box.cls.item())]
            color = VERDICT_COLORS.get(assessment['verdict'], (192, 192, 192))
            label = f"{class_name} {float(box.conf.item()):.2f}"
            annotator.box_label(box.xyxy[0].cpu().tolist(), label, color=color)
    return annotator.result()

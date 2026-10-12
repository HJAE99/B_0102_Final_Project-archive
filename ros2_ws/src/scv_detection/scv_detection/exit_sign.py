"""비상구 표시등의 켜짐·꺼짐을 정상·비정상으로 판정하는 함수 모음."""

from scv_msgs.msg import InspectionResult

EXIT_CLASSES = {'exit_sign_on', 'exit_sign_off'}


def supports(class_name):
    """이 클래스 이름이 비상구 표시등에 해당하는지 알려준다."""
    return class_name.strip().lower() in EXIT_CLASSES


def class_to_verdict(class_name):
    """점등은 정상, 소등은 이상으로 바꾸며 모르는 이름은 판정을 보류한다."""
    name = class_name.strip().lower()
    if name == 'exit_sign_on':
        return InspectionResult.NORMAL
    if name == 'exit_sign_off':
        return InspectionResult.ABNORMAL
    return InspectionResult.UNCONFIRMED


def assess(class_name, frame, box, min_size=48, min_axis_ratio=0.90):
    """비상구 모델 결과를 판정으로 바꾼다. 공통 호출 규격의 사진·박스 인자는 사용하지 않는다."""
    return {'verdict': class_to_verdict(class_name), 'frontal': None,
            'reason': 'frontality check not required for this class'}

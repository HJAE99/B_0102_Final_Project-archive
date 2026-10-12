"""Configuration, headless operation, error handling and status-color tests."""

from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage
import torch
from ultralytics.engine.results import Results

from scv_detection import scv_detection_manager as module
from scv_detection import detection_render, pressure_gauge, exit_sign


@pytest.fixture
def ros():
    """각 시험을 시작할 때 ROS 통신을 준비하고 끝나면 종료한다."""
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def make_node(ros, tmp_path, monkeypatch):
    """시험용 모델을 사용해 검출 노드를 만들고 시험이 끝나면 정리한다."""
    weights = tmp_path / 'test.pt'
    weights.touch()
    monkeypatch.setattr(module, 'YOLO', Mock())
    monkeypatch.setattr(module.torch.cuda, 'is_available', lambda: False)
    nodes = []

    def create(namespace='robot3', **overrides):
        """기본 시험 설정에 원하는 값을 덮어쓴 뒤 검출 노드를 만든다."""
        params = dict(weights=str(weights), max_fps=0.0)
        params.update(overrides)
        node = module.ScvDetectionManager(namespace=namespace, parameter_overrides=[
            Parameter(name, value=value) for name, value in params.items()
        ])
        nodes.append(node)
        return node

    yield create
    for node in nodes:
        node.destroy_node()


def test_required_configuration(make_node):
    """모델이나 로봇 이름이 없으면 영상 처리를 시작하기 전에 오류가 나는지 확인한다."""
    with pytest.raises(ValueError, match='weights is required'):
        make_node(weights='')
    with pytest.raises(FileNotFoundError):
        make_node(weights='/no-such-model.pt')
    with pytest.raises(ValueError, match='Launch with robot_ns'):
        make_node(namespace='/')


@pytest.mark.parametrize('namespace', ['robot3', '/robot3', 'inspector'])
def test_topics_and_qos(make_node, namespace):
    """로봇 이름에 맞게 영상 주소가 만들어지고 통신 설정이 맞는지 확인한다."""
    node = make_node(namespace=namespace)
    prefix = '/' + namespace.strip('/')
    assert node.get_name() == 'detection_alert'
    assert node.topic == 'oakd/rgb/image_raw/compressed'
    assert node.output_topic == 'fire_extinguisher/detections/compressed'
    assert node.subscription.topic_name == prefix + '/' + node.topic
    assert node.publisher.topic_name == prefix + '/' + node.output_topic
    assert node.subscription.qos_profile.depth == 1
    assert node.subscription.qos_profile.reliability == module.ReliabilityPolicy.BEST_EFFORT


def test_headless_default(make_node, monkeypatch):
    """별도로 화면을 요청하지 않으면 영상 창을 만들지 않는지 확인한다."""
    window = Mock(side_effect=AssertionError('GUI must not be used'))
    monkeypatch.setattr(module.cv2, 'namedWindow', window)
    node = make_node()
    assert node.show is False
    window.assert_not_called()


def test_show_without_desktop(make_node, monkeypatch):
    """화면 표시를 요청해도 데스크톱이 없으면 창 기능만 꺼지는지 확인한다."""
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    window = Mock(side_effect=AssertionError('GUI must not be used'))
    monkeypatch.setattr(module.cv2, 'namedWindow', window)
    assert make_node(show=True).show is False
    window.assert_not_called()


def test_window_failure_falls_back(make_node, monkeypatch):
    """창 생성에 실패해도 검출 노드는 계속 사용할 수 있는지 확인한다."""
    monkeypatch.setenv('DISPLAY', ':test')
    monkeypatch.setattr(module.cv2, 'namedWindow', Mock(side_effect=cv2.error('no GUI')))
    assert make_node(show=True).show is False


def test_decode_and_inference_errors(make_node):
    """깨진 사진이나 모델 오류 뒤에도 다음 정상 사진을 처리할 수 있는지 확인한다."""
    node = make_node()
    node.publisher = Mock()
    for payload in (b'', b'not a JPEG'):
        node.image_callback(CompressedImage(data=payload))
    node.model.predict.assert_not_called()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    _, encoded = cv2.imencode('.jpg', frame)
    message = CompressedImage(format='jpeg', data=encoded.tobytes())
    node.model.predict.side_effect = RuntimeError('inference error')
    node.image_callback(message)
    node.publisher.publish.assert_not_called()
    node.model.predict.side_effect = None
    node.model.predict.return_value = [Results(frame, path='test.jpg',
                                             names={0: 'Normal'}, boxes=torch.empty((0, 6)))]
    node.image_callback(message)
    output = node.publisher.publish.call_args.args[0]
    assert output.format == 'jpeg'
    assert cv2.imdecode(np.frombuffer(output.data, dtype=np.uint8), cv2.IMREAD_COLOR).shape == frame.shape


@pytest.mark.parametrize('name,color', [
    ('Normal', (0, 255, 0)), ('Abnormal_left', (0, 0, 255)),
    ('Abnormal_right', (0, 0, 255)), ('Exit_sign_on', (0, 255, 0)),
    ('Exit_sign_off', (0, 0, 255)), ('other', (192, 192, 192)),
])
def test_detection_colors(name, color):
    """정상·점등은 초록, 이상·소등은 빨강으로 그리며 원본 사진은 보존하는지 확인한다."""
    frame = np.zeros((180, 280, 3), dtype=np.uint8)
    cv2.circle(frame, (130, 100), 30, (255, 255, 255), 3)
    original = frame.copy()
    result = Results(frame, path='test.jpg', names={0: name},
                     boxes=torch.tensor([[60, 60, 200, 145, 0.8, 0]]))
    annotated = module.annotate_detections(result)
    assert tuple(annotated[145, 130]) == color
    assert np.array_equal(frame, original)


@pytest.mark.parametrize('axes,expected', [((60, 60), True), ((60, 32), False)])
def test_frontality_geometry(axes, expected):
    """원을 정면, 찌그러진 타원을 비정면으로 구분하는 기본 동작을 확인한다."""
    frame = np.zeros((180, 180, 3), dtype=np.uint8)
    cv2.ellipse(frame, (90, 90), axes, 25, 0, 360, (255, 255, 255), 3)
    frontal, _ = pressure_gauge.check_pressure_frontality(frame, [10, 10, 170, 170])
    assert frontal is expected


@pytest.mark.parametrize('box', [[10, 10, 35, 35], [10, 10, 170, 170]])
def test_unclear_frontality_holds_verdict(box):
    """너무 작거나 테두리가 없는 압력계는 정상·이상 대신 판정 보류로 표시하는지 확인한다."""
    frame = np.zeros((180, 180, 3), dtype=np.uint8)
    result = Results(frame, path='test.jpg', names={0: 'Normal'},
                     boxes=torch.tensor([[*box, 0.9, 0]]))
    assessment = module.assess_detections(result)[0]
    assert assessment['frontal'] is None
    assert assessment['verdict'] == module.InspectionResult.UNCONFIRMED
    output = module.annotate_detections(result, [assessment])
    assert tuple(output[box[3], (box[0] + box[2]) // 2]) == (192, 192, 192)


@pytest.mark.parametrize('name', ['Normal', 'Abnormal_left', 'Abnormal_right'])
def test_nonfrontal_pressure_not_judged(name):
    """비정면 압력계에서는 모델이 상태를 예측해도 정상·이상 판정을 내보내지 않는다."""
    assert pressure_gauge.class_to_verdict(name, False) == module.InspectionResult.UNCONFIRMED
    assert pressure_gauge.class_to_verdict(name, None) == module.InspectionResult.UNCONFIRMED


@pytest.mark.parametrize('name,verdict', [('Normal', 'NORMAL'), ('Abnormal_left', 'ABNORMAL'),
                                        ('Abnormal_right', 'ABNORMAL'), ('Exit_sign_on', 'NORMAL'),
                                        ('Exit_sign_off', 'ABNORMAL')])
def test_interface_verdict_mapping(name, verdict):
    """모델 클래스가 공동 메시지의 정상·이상 상수에 올바르게 연결되는지 확인한다."""
    assert detection_render.class_to_verdict(name, True) == getattr(module.InspectionResult, verdict)


@pytest.mark.parametrize('name,expected', [('Exit_sign_on', 'NORMAL'),
                                         ('Exit_sign_off', 'ABNORMAL'), ('unknown', 'UNCONFIRMED')])
def test_exit_module_without_pressure_geometry(name, expected):
    """비상구 판정 함수는 압력계 테두리 검사 없이 점등·소등을 판단하는지 확인한다."""
    assessment = exit_sign.assess(name, np.zeros((1, 1, 3), np.uint8), [0, 0, 1, 1])
    assert assessment['verdict'] == getattr(module.InspectionResult, expected)
    assert assessment['frontal'] is None


def test_dispatch_to_facility_modules(monkeypatch):
    """압력계와 비상구가 함께 검출되면 각 담당 함수로 나누어 호출하는지 확인한다."""
    pressure = Mock(return_value={'verdict': module.InspectionResult.NORMAL,
                                 'frontal': True, 'reason': 'pressure test'})
    exit_light = Mock(return_value={'verdict': module.InspectionResult.ABNORMAL,
                                   'frontal': None, 'reason': 'exit test'})
    monkeypatch.setattr(pressure_gauge, 'assess', pressure)
    monkeypatch.setattr(exit_sign, 'assess', exit_light)
    frame = np.zeros((180, 280, 3), np.uint8)
    result = Results(frame, path='test.jpg', names={0: 'Normal', 1: 'Exit_sign_off'},
                     boxes=torch.tensor([[60, 60, 150, 145, 0.8, 0],
                                         [160, 60, 260, 145, 0.9, 1]]))
    assessments = module.assess_detections(result, 50, 0.92)
    assert [item['reason'] for item in assessments] == ['pressure test', 'exit test']
    pressure.assert_called_once()
    exit_light.assert_called_once()
    assert pressure.call_args.args[0] == 'Normal'
    assert pressure.call_args.args[3:] == (50, 0.92)
    assert exit_light.call_args.args[0] == 'Exit_sign_off'


def test_rack_only_without_weights(make_node):
    """랙만 검사할 때는 모델 파일 없이 실행하고 YOLO를 불러오지 않는지 확인한다."""
    node = make_node(enable_yolo=False, rack_enabled=True, weights='')
    assert node.model is None
    node.publisher = Mock()
    _, jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    node.image_callback(CompressedImage(format='jpeg', data=jpeg.tobytes()))
    node.publisher.publish.assert_called_once()


def test_rack_called_when_yolo_fails(make_node, monkeypatch):
    """YOLO에 오류가 나도 독립적인 랙 함수 결과를 화면 출력으로 발행하는지 확인한다."""
    node = make_node(rack_enabled=True)
    node.publisher = Mock()
    node.model.predict.side_effect = RuntimeError('test model failure')
    inspect = Mock(return_value={'verdict': module.InspectionResult.UNCONFIRMED,
                                'corners': None, 'boxes': [], 'protrusions': [], 'reason': 'test'})
    monkeypatch.setattr(module.rack_conditioner, 'inspect_rack', inspect)
    _, jpeg = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    node.image_callback(CompressedImage(format='jpeg', data=jpeg.tobytes()))
    inspect.assert_called_once()
    node.publisher.publish.assert_called_once()


@pytest.mark.parametrize('settings', [dict(rack_confirm_frames=0), dict(rack_tolerance=-1),
                                    dict(rack_hsv_lower=[180, 0, 0]),
                                    dict(enable_yolo=False, rack_enabled=False)])
def test_invalid_rack_settings(make_node, settings):
    """잘못된 테이프 색 범위나 확인 횟수 등으로 실행하지 못하게 하는지 확인한다."""
    with pytest.raises(ValueError):
        make_node(**settings)

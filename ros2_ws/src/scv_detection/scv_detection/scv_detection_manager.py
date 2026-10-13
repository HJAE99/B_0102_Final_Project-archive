#!/usr/bin/env python3
"""평소에는 카메라를 보여 주고 서비스 요청 때만 YOLO 또는 랙을 검사하는 노드."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scv_msgs.msg import InspectionResult
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from scv_detection import rack_conditioner


# 현재 best.pt의 클래스 ID별 BGR 색상과 로그 상태다. 모델의 ID 순서가 바뀌면 함께 수정한다.
YOLO_CLASS_STYLES = {
    0: ((0, 0, 255), 'ABNORMAL'),      # Abnormal_left
    1: ((0, 0, 255), 'ABNORMAL'),      # Abnormal_right
    2: ((192, 192, 192), 'UNCONFIRMED'),  # Fire Extinguisher: 전체 모습만으로 상태를 알 수 없다.
    3: ((0, 255, 0), 'NORMAL'),        # Normal
    4: ((0, 255, 0), 'NORMAL'),        # Exit_sign_on
    5: ((0, 0, 255), 'ABNORMAL'),      # Exit_sign_off
}


class ScvDetectionManager(Node):
    @staticmethod
    def _find_default_weights() -> str:
        """설치된 패키지의 models/best.pt 경로를 찾아 사용자별 절대 경로 없이 반환한다."""
        return str(Path(get_package_share_directory('scv_detection')) / 'models' / 'best.pt')

    @staticmethod
    def _find_default_reference():
        """패키지 소스 config 폴더에서 기준 파일 기본 경로를 찾는다."""
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = Path(get_package_prefix('scv_detection'))
            src_config = prefix.parent.parent / 'src' / 'scv_detection' / 'config'
            if src_config.is_dir():
                return str(src_config / 'rack_reference.npz')
        except Exception:
            pass
        return ''

    def __init__(self, **kwargs) -> None:
        """기본 robot4 카메라와 두 검사 서비스를 준비한다. 모델 추론은 요청 전에는 하지 않는다."""
        kwargs.setdefault('namespace', '/robot4')
        super().__init__("detection_alert", **kwargs)

        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("show", True)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('rack_enabled', True)
        self.declare_parameter('rack_confirm_frames', 3)
        self.declare_parameter('rack_diff_threshold', 35)
        self.declare_parameter('rack_min_area', 300)
        self.declare_parameter('rack_margin', 30)
        self.declare_parameter('rack_zone_width', 90)
        default_ref = self._find_default_reference()
        self.declare_parameter('rack_reference_path', default_ref)
        self.declare_parameter('camera_timeout', 2.0)
        self.declare_parameter('rack_roi_x1', 0.25)
        self.declare_parameter('rack_roi_y1', 0.30)
        self.declare_parameter('rack_roi_x2', 0.75)
        self.declare_parameter('rack_roi_y2', 0.83)
        self.enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.rack_enabled = bool(self.get_parameter('rack_enabled').value)
        self.rack_inspect_options = dict(
            margin=int(self.get_parameter('rack_margin').value),
            zone_width=int(self.get_parameter('rack_zone_width').value),
            diff_threshold=int(self.get_parameter('rack_diff_threshold').value),
            min_area=int(self.get_parameter('rack_min_area').value))
        # 1차 ROI: 랙을 찾을 고정 영역(화면 비율 0~1). 이 밖에서는 랙을 검출하지 않는다.
        self.rack_roi = (
            float(self.get_parameter('rack_roi_x1').value),
            float(self.get_parameter('rack_roi_y1').value),
            float(self.get_parameter('rack_roi_x2').value),
            float(self.get_parameter('rack_roi_y2').value))
        x1, y1, x2, y2 = self.rack_roi
        if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
            raise ValueError('rack_roi must satisfy 0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1')
        self.window_size = (960, 540)
        self.latest_frame = None
        self.last_frame_time = None
        ref_path = str(self.get_parameter('rack_reference_path').value).strip()
        self.rack_reference_path = ref_path if ref_path else None
        self.rack_reference = None
        if self.rack_reference_path:
            self.rack_reference = rack_conditioner.load_reference(self.rack_reference_path)
            if self.rack_reference is not None:
                r = self.rack_reference['rack']
                self.get_logger().info(
                    f'Loaded rack reference from {self.rack_reference_path} '
                    f'(L={r["left"]} R={r["right"]})')
            else:
                self.get_logger().info(
                    f'No rack reference at {self.rack_reference_path} yet; '
                    'press N to capture and save')
        self.rack_confirm_frames = int(self.get_parameter('rack_confirm_frames').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        if self.rack_confirm_frames < 1:
            raise ValueError('rack_confirm_frames must be positive')
        if self.camera_timeout <= 0:
            raise ValueError('camera_timeout must be positive')
        self.rack_state = {}
        # 시작·중지·기준 변경을 한 번에 하나씩 처리한다. 검사 모드는 다음 호출까지 유지한다.
        self._inspection_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._inspection_mode = None
        self._inspection_id = 0
        self._inspection_result = None
        self._stop_event = threading.Event()
        srv_group = ReentrantCallbackGroup()
        if self.rack_enabled:
            self.rack_service = self.create_service(
                Trigger, '~/inspect_rack', self._handle_inspect_rack,
                callback_group=srv_group)
        if self.enable_yolo:
            self.yolo_service = self.create_service(
                Trigger, '~/inspect_yolo', self._handle_inspect_yolo,
                callback_group=srv_group)

        self.model = None
        weights_value = str(self.get_parameter('weights').value).strip()
        self.weights = Path(weights_value or self._find_default_weights()).expanduser().resolve()
        self.device = 'cpu'
        self.topic = 'oakd/rgb/image_raw/compressed'
        self.output_topic = 'fire_extinguisher/detections/compressed'
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.show = bool(self.get_parameter("show").value)
        self.window_name = "YOLO and Rack Detection"
        if self.show and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warning("No desktop display available; disabling show")
            self.show = False
        if self.show:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, *self.window_size)
                waiting_frame = np.zeros((540, 960, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for camera frames...", (230, 280),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow(self.window_name, waiting_frame)
                cv2.waitKey(1)
            except cv2.error as error:
                self.get_logger().warning(f"Cannot open display; disabling show: {error}")
                self.show = False

        self._display_frame = None
        self.last_inference_time = 0.0
        self.last_log_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.output_topic, sensor_qos
        )
        self.subscription = self.create_subscription(
            CompressedImage, self.topic, self.image_callback, sensor_qos
        )

        self.get_logger().info(f'weights: {self.weights} (loaded on first YOLO request)')
        self.get_logger().info(f"input: {self.subscription.topic_name}")
        self.get_logger().info(f"output: {self.publisher.topic_name}")
        self.get_logger().info(f'YOLO enabled={self.enable_yolo}; rack enabled={self.rack_enabled}')
        self.get_logger().info('Camera preview ready. Call ~/inspect_yolo or ~/inspect_rack to toggle detection.')

    def crop_rack_roi(self, frame: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]]:
        """영상에서 1차 ROI(랙을 찾을 고정 영역)만 잘라낸다.

        rack_roi 파라미터는 0~1 비율이므로 실제 영상 크기에 맞춰 픽셀 좌표로 바꾼다.
        반환: (잘라낸 영상, 전체 영상 기준 (x1, y1, x2, y2) 픽셀 좌표)
        """
        height, width = frame.shape[:2]
        x1 = int(self.rack_roi[0] * width)
        y1 = int(self.rack_roi[1] * height)
        x2 = int(self.rack_roi[2] * width)
        y2 = int(self.rack_roi[3] * height)
        return frame[y1:y2, x1:x2].copy(), (x1, y1, x2, y2)

    def reference_for_roi(self, roi_box: tuple[int, int, int, int]):
        """저장된 기준 자료를 1차 ROI 좌표계에 맞춰 돌려준다.

        N키로 새로 저장한 기준은 이미 ROI 크기라 그대로 쓰고, 예전에 전체 화면으로
        저장한 기준이면 같은 ROI로 잘라내고 랙 좌표도 그만큼 옮긴다. 기준이 없으면 None.
        """
        reference = self.rack_reference
        if reference is None:
            return None
        x1, y1, x2, y2 = roi_box
        ref_frame = reference['frame']
        if ref_frame.shape[:2] == (y2 - y1, x2 - x1):
            return reference
        rack = reference['rack']
        return {'frame': ref_frame[y1:y2, x1:x2],
                'rack': {'left': rack['left'] - x1, 'right': rack['right'] - x1,
                         'top': rack['top'] - y1, 'bottom': rack['bottom'] - y1}}

    def capture_rack_reference(self) -> bool:
        """대기 중인 최신 ROI를 정상 기준으로 저장한다. 검사 중에는 기준을 바꾸지 않는다."""
        if not self._inspection_lock.acquire(blocking=False):
            self.get_logger().warning('Cannot change rack reference during inspection.')
            return False
        try:
            if self._inspection_mode is not None:
                self.get_logger().warning('Stop detection before changing the rack reference.')
                return False
            return self._capture_rack_reference()
        finally:
            self._inspection_lock.release()

    def _capture_rack_reference(self) -> bool:
        """요청 잠금을 확보한 뒤 최신 영상에서 랙을 찾아 기준 사진을 저장한다."""
        if self.latest_frame is None:
            self.get_logger().warning('No frame available yet.')
            return False
        roi_frame, _ = self.crop_rack_roi(self.latest_frame)
        rack = rack_conditioner.detect_rack(roi_frame)
        if rack is None:
            self.get_logger().warning('Cannot detect rack pipes in current frame.')
            return False
        reference = rack_conditioner.capture_reference(roi_frame, rack)
        if reference is None:
            self.get_logger().warning('Rack area has no visible content.')
            return False
        with self._state_lock:
            self.rack_reference = reference
            self.rack_state = {}
        if self.rack_reference_path:
            rack_conditioner.save_reference(reference, self.rack_reference_path)
            self.get_logger().info(
                f'Saved rack reference to {self.rack_reference_path} '
                f'(L={rack["left"]} R={rack["right"]})')
        else:
            self.get_logger().info(
                f'Rack reference set in memory (L={rack["left"]} R={rack["right"]}). '
                'Set rack_reference_path to persist across restarts.')
        return True

    def reset_rack_reference(self) -> bool:
        """검사 대기 중에 메모리의 랙 기준을 지운다. 저장한 NPZ 파일은 그대로 둔다."""
        if not self._inspection_lock.acquire(blocking=False):
            self.get_logger().warning('Cannot reset rack reference during inspection.')
            return False
        try:
            with self._state_lock:
                if self._inspection_mode is not None:
                    self.get_logger().warning('Stop detection before resetting the rack reference.')
                    return False
                self.rack_reference = None
                self.rack_state = {}
            return True
        finally:
            self._inspection_lock.release()

    def _load_yolo_model(self) -> None:
        """첫 YOLO 서비스 요청에서만 자동 선택한 best.pt를 로드하고 이후 요청에서 재사용한다."""
        if self.model is not None:
            return
        if not self.weights.is_file():
            raise FileNotFoundError(f'Weights file not found: {self.weights}')
        import torch
        from ultralytics import YOLO

        requested_device = str(self.get_parameter('device').value)
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if requested_device == 'auto' else requested_device
        self.model = YOLO(str(self.weights))
        self.get_logger().info(f'YOLO classes: {self.model.names}; device: {self.device}')

    def _handle_inspect_yolo(self, request, response):
        """빈 Trigger 요청으로 YOLO를 시작하거나 중지하고 변경 결과를 즉시 응답한다."""
        return self._handle_inspection('yolo', response)

    def _handle_inspect_rack(self, request, response):
        """빈 Trigger 요청으로 랙 검사를 시작하거나 중지하고 변경 결과를 즉시 응답한다."""
        return self._handle_inspection('rack', response)

    def _handle_inspection(self, mode, response):
        """같은 서비스를 부를 때마다 검사를 켜거나 끄고 Trigger 변경 결과를 반환한다.

        mode는 yolo 또는 rack이다. 시작 후에는 시간 제한 없이 검사한다.
        다른 모드가 켜져 있으면 거절한다. 중지는 영상이나 기준 사진 없이도 가능하다.
        응답은 시작·중지 성공 여부이며 실제 판정은 영상과 로그에 계속 표시한다.
        """
        response.success = False
        if not self._inspection_lock.acquire(blocking=False):
            response.message = 'busy: detection is starting/stopping or reference is updating'
            return response
        try:
            with self._state_lock:
                active_mode = self._inspection_mode
                if active_mode == mode:
                    self._inspection_mode = None
                    self._inspection_id += 1
                    self._inspection_result = None
                    self.rack_state = {}
                    if self.latest_frame is not None:
                        self._display_frame = self.latest_frame.copy()
                    response.success = True
                    response.message = f'{mode.upper()} detection stopped; camera preview resumed'
                    self.get_logger().info(response.message)
                    return response
                if active_mode is not None:
                    response.message = f'busy: {active_mode.upper()} detection is active; call its service again to stop'
                    return response
            if self._stop_event.is_set():
                response.message = 'node is stopping'
                return response
            if not (self.enable_yolo if mode == 'yolo' else self.rack_enabled):
                response.message = f'{mode} inspection is disabled'
                return response
            if self.last_frame_time is None or time.monotonic() - self.last_frame_time > self.camera_timeout:
                response.message = 'no recent camera frame available'
                return response
            if mode == 'rack' and self.rack_reference is None:
                response.message = 'no rack reference loaded; fit boxes inside and press N'
                return response
            if mode == 'yolo':
                self._load_yolo_model()
            # 첫 모델 로딩 동안 카메라가 끊긴 경우에는 검사를 시작하지 않는다.
            if self.last_frame_time is None or time.monotonic() - self.last_frame_time > self.camera_timeout:
                response.message = 'no recent camera frame available'
                return response
            if self._stop_event.is_set():
                response.message = 'node is stopping'
                return response
            with self._state_lock:
                self.rack_state = {}
                self._inspection_result = None
                self._inspection_id += 1
                self._inspection_mode = mode
            response.success = True
            response.message = f'{mode.upper()} detection started; call this service again to stop'
            self.get_logger().info(response.message)
        except Exception as error:
            response.message = f'{mode} inspection failed: {error}'
            self.get_logger().error(response.message)
        finally:
            self._inspection_lock.release()
        return response

    @staticmethod
    def annotate_yolo_result(result) -> tuple[np.ndarray, list[str]]:
        """YOLO 결과를 받아 클래스별 색상으로 모든 박스·이름·신뢰도를 그린다.

        원본 사진은 보존하고 (표시용 사진, 물체별 로그 문자열 목록)을 반환한다.
        소화기 전체와 등록하지 않은 ID도 회색으로 표시하며 검출에서 제외하지 않는다.
        """
        from ultralytics.utils.plotting import Annotator

        annotator = Annotator(result.orig_img.copy(), example=str(result.names))
        detections = []
        if result.boxes is not None:
            for box in reversed(result.boxes):
                class_id = int(box.cls.item())
                name = result.names[class_id]
                confidence = float(box.conf.item())
                color, state = YOLO_CLASS_STYLES.get(class_id, ((192, 192, 192), 'UNCONFIRMED'))
                annotator.box_label(box.xyxy[0].cpu().tolist(), f'{name} {confidence:.2f}', color=color)
                detections.append(f'{name}:{confidence:.2f} verdict={state}')
        detections.reverse()
        return annotator.result(), detections

    def image_callback(self, message: CompressedImage) -> None:
        """평소에는 원본 카메라를 발행하고, 서비스가 고른 검사 한 가지만 영상에 표시한다."""
        if self._stop_event.is_set():
            return
        now = time.monotonic()
        if self.max_fps > 0 and now - self.last_inference_time < 1.0 / self.max_fps:
            return
        self.last_inference_time = now

        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        except (cv2.error, ValueError) as error:
            self.get_logger().warning(f"Failed to decode compressed image: {error}")
            return
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            return
        with self._state_lock:
            self.latest_frame = frame.copy()
            self.last_frame_time = now
            mode = self._inspection_mode
            request_id = self._inspection_id
            rack_state = self.rack_state

        annotated = frame.copy()
        detections = []
        rack_report = None
        outcome = None
        try:
            if mode == 'yolo':
                result = self.model.predict(
                    source=frame, conf=self.conf, iou=self.iou, imgsz=self.imgsz,
                    device=self.device, verbose=False,
                )[0]
                annotated, detections = self.annotate_yolo_result(result)
                summary = ', '.join(detections) if detections else 'none'
                # 내부 결과의 success는 추론 성공이다. 검출 0개도 정상 상태를 뜻하지 않는다.
                outcome = {'success': True,
                           'message': f'YOLO detections={len(detections)} [{summary}]',
                           'time': time.monotonic()}
            elif mode == 'rack':
                roi_frame, roi_box = self.crop_rack_roi(frame)
                rack_report = rack_conditioner.stabilize_result(
                    rack_conditioner.inspect_rack(
                        roi_frame, reference=self.reference_for_roi(roi_box),
                        **self.rack_inspect_options),
                    rack_state, self.rack_confirm_frames)
                x1, y1, x2, y2 = roi_box
                annotated[y1:y2, x1:x2] = rack_conditioner.draw_result(roi_frame, rack_report)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 1)
                verdict = rack_report['verdict']
                if verdict != InspectionResult.UNCONFIRMED:
                    normal = verdict == InspectionResult.NORMAL
                    outcome = {'success': normal,
                               'message': f"{'NORMAL' if normal else 'ABNORMAL'}: {rack_report['reason']}",
                               'time': time.monotonic()}
        except Exception as error:
            self.get_logger().error(f'{mode} inspection failed: {error}')
            annotated = frame.copy()
            outcome = {'success': False, 'message': f'{mode} inspection failed: {error}',
                       'time': time.monotonic()}

        with self._state_lock:
            if mode is not None and mode == self._inspection_mode and request_id == self._inspection_id:
                # 마지막 프레임이 미확인이면 과거의 확정 결과를 지워 옛 판정이 남지 않게 한다.
                self._inspection_result = outcome
            elif mode is not None:
                # 끝난 요청의 계산 결과를 다음 검사나 대기 화면에 섞지 않는다.
                annotated = frame.copy()
                detections = []
                rack_report = None
                mode = None
            if self.show:
                self._display_frame = annotated

        try:
            ok, jpeg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                output = CompressedImage()
                output.header = message.header
                output.format = "jpeg"
                output.data = jpeg.tobytes()
                self.publisher.publish(output)
        except cv2.error as error:
            self.get_logger().error(f'Detection rendering failed: {error}')

        if mode is not None and now - self.last_log_time >= 1.0:
            if mode == 'yolo':
                summary = ', '.join(detections) if detections else 'none'
                self.get_logger().info(f'detections={len(detections)} [{summary}]')
            elif rack_report is not None:
                self.get_logger().info(f"rack verdict={rack_report['verdict']} reason={rack_report['reason']}")
            self.last_log_time = now

    def destroy_node(self) -> bool:
        """영상 처리를 중지하고 영상 창과 ROS 통신 자원을 정리한다."""
        self._stop_event.set()
        if self.show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main() -> None:
    """카메라 노드를 실행하고 메인 스레드에서 화면·기준 저장 키를 처리한 뒤 안전하게 종료한다."""
    rclpy.init()
    node = None
    executor = None
    spin_thread = None
    try:
        node = ScvDetectionManager()
        # 영상 처리와 서비스 시작·중지 요청을 별도 워커에서 처리한다.
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok():
            if node.show:
                with node._state_lock:
                    frame = node._display_frame
                    node._display_frame = None
                try:
                    if frame is not None:
                        display = cv2.resize(frame, node.window_size)
                        cv2.imshow(node.window_name, display)
                    key = cv2.waitKey(30) & 0xFF
                    if key in (ord('n'), ord('N')):
                        node.capture_rack_reference()
                    elif key in (ord('r'), ord('R')):
                        node.reset_rack_reference()
                except cv2.error as error:
                    node.get_logger().warning(f"Display failed; disabling show: {error}")
                    node.show = False
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node._stop_event.set()
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""영상 수신·추론을 관리하고 설비별 함수 모듈을 호출하는 단일 실행 노드."""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scv_msgs.msg import InspectionResult
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from scv_detection import rack_conditioner


class ScvDetectionManager(Node):
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
        """사용자가 지정한 모델과 로봇을 연결하고 영상 입출력과 선택 화면을 준비한다."""
        super().__init__("detection_alert", **kwargs)

        self.declare_parameter("weights", "")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("show", False)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('rack_enabled', False)
        self.declare_parameter('rack_confirm_frames', 3)
        self.declare_parameter('rack_diff_threshold', 35)
        self.declare_parameter('rack_min_area', 300)
        self.declare_parameter('rack_margin', 30)
        self.declare_parameter('rack_zone_width', 90)
        default_ref = self._find_default_reference()
        self.declare_parameter('rack_reference_path', default_ref)
        self.declare_parameter('rack_inspect_duration', 5.0)
        self.declare_parameter('rack_inspect_timeout', 10.0)
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
        self.rack_inspect_duration = float(self.get_parameter('rack_inspect_duration').value)
        self.rack_inspect_timeout = float(self.get_parameter('rack_inspect_timeout').value)
        if not self.enable_yolo and not self.rack_enabled:
            raise ValueError('Enable at least one inspection feature')
        if self.rack_confirm_frames < 1:
            raise ValueError('rack_confirm_frames must be positive')
        if self.rack_inspect_duration < 0 or self.rack_inspect_timeout <= 0:
            raise ValueError('rack_inspect_duration must be >= 0 and rack_inspect_timeout > 0')
        self.rack_state = {}
        self._rack_active = False
        self._rack_result = None
        self._rack_event = threading.Event()
        if self.rack_enabled:
            srv_group = MutuallyExclusiveCallbackGroup()
            self.rack_service = self.create_service(
                Trigger, '~/inspect_rack', self._handle_inspect_rack,
                callback_group=srv_group)

        self.model = None
        if self.enable_yolo:
            import torch
            from ultralytics import YOLO
            weights_value = str(self.get_parameter("weights").value).strip()
            if not weights_value:
                raise ValueError("weights is required: --ros-args -p weights:=/path/to/best.pt")
            weights = Path(weights_value).expanduser().resolve()
            if not weights.is_file():
                raise FileNotFoundError(f"Weights file not found: {weights}")
            requested_device = str(self.get_parameter("device").value)
            self.device = (
                "0" if torch.cuda.is_available() else "cpu"
            ) if requested_device == "auto" else requested_device
            self.model = YOLO(str(weights))
        else:
            self.device = "cpu"

        if self.get_namespace() == '/':
            raise ValueError('Launch with robot_ns, or pass --ros-args -r __ns:=/your_robot')
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

        self.get_logger().info(f"weights: {self.model.ckpt_path}" if self.model else 'weights: unused (YOLO disabled)')
        if self.model is not None:
            self.get_logger().info(f"YOLO classes: {self.model.names}")
        self.get_logger().info(f"device: {self.device}")
        self.get_logger().info(f"input: {self.subscription.topic_name}")
        self.get_logger().info(f"output: {self.publisher.topic_name}")
        self.get_logger().info(f'YOLO enabled={self.enable_yolo}; rack enabled={self.rack_enabled}')

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
        """현재 영상의 1차 ROI 안에서 랙을 감지하고 정상 기준으로 저장한다."""
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
        self.rack_reference = reference
        self.rack_state.clear()
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

    def _handle_inspect_rack(self, request, response):
        """서비스 요청이 오면 랙 검사를 켜고, 검사 화면을 일정 시간 유지한 뒤 마지막 확정 판정을 돌려준다.

        첫 확정 판정은 rack_inspect_timeout 초까지 기다리고, 판정이 나도 요청 시점부터
        rack_inspect_duration 초가 지날 때까지는 검사(노란 박스 표시)를 계속한다.
        그 사이 판정이 바뀌면 가장 마지막 확정 판정을 쓴다.
        """
        if self.latest_frame is None:
            response.success = False
            response.message = 'no camera frame available'
            return response
        if self.rack_reference is None:
            response.success = False
            response.message = 'no rack reference loaded'
            return response
        self.rack_state.clear()
        self._rack_result = None
        self._rack_event.clear()
        self._rack_active = True
        self.get_logger().info('Rack inspection started (service request)')
        hold_until = time.monotonic() + self.rack_inspect_duration
        self._rack_event.wait(timeout=self.rack_inspect_timeout)
        remaining = hold_until - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        if self._rack_result is not None:
            verdict = self._rack_result['verdict']
            reason = self._rack_result['reason']
            if verdict == InspectionResult.NORMAL:
                response.success = True
                response.message = f'NORMAL: {reason}'
            else:
                response.success = False
                response.message = f'ABNORMAL: {reason}'
        else:
            response.success = False
            response.message = 'inspection timed out'
        self._rack_active = False
        return response

    def image_callback(self, message: CompressedImage) -> None:
        """압축 영상을 풀어 선택한 YOLO·랙 기능을 호출하고 결과 사진을 발행한다."""
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
        self.latest_frame = frame.copy()

        rack_report = None
        roi_box = None
        if self.rack_enabled:
            roi_frame, roi_box = self.crop_rack_roi(frame)
        if self._rack_active:
            try:
                rack_report = rack_conditioner.stabilize_result(
                    rack_conditioner.inspect_rack(
                        roi_frame, reference=self.reference_for_roi(roi_box),
                        **self.rack_inspect_options),
                    self.rack_state, self.rack_confirm_frames)
                if rack_report['verdict'] != InspectionResult.UNCONFIRMED:
                    self._rack_result = rack_report
                    self._rack_event.set()
            except Exception as error:
                self.rack_state.clear()
                self.get_logger().error(f'Rack inspection failed: {error}')
                rack_report = {'verdict': InspectionResult.UNCONFIRMED, 'rack': None,
                               'protrusions': [], 'zones': [], 'reason': 'rack processing error'}
        result = None
        try:
            result = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )[0] if self.model is not None else None
        except Exception as error:
            self.get_logger().error(f"Inference failed: {error}")
            # YOLO 오류가 나도 독립적인 OpenCV 랙 검사는 계속 표시한다.
            if rack_report is None:
                return

        try:
            assessments = []
            if result is not None:
                from scv_detection.detection_render import assess_detections, annotate_detections
                assessments = assess_detections(result)
            annotated = annotate_detections(result, assessments) if result is not None else frame.copy()
            if roi_box is not None:
                x1, y1, x2, y2 = roi_box
                if rack_report is not None:
                    # 랙 결과 좌표는 1차 ROI 기준이므로 ROI 부분만 잘라 그린 뒤 원래 자리에 붙인다.
                    annotated[y1:y2, x1:x2] = rack_conditioner.draw_result(
                        annotated[y1:y2, x1:x2].copy(), rack_report)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 1)
                if self.rack_reference is None:
                    cv2.putText(annotated, 'Fit all boxes inside rack, then press N to save reference',
                                (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            ok, jpeg = cv2.imencode(
                ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
        except Exception as error:
            self.get_logger().error(f"Detection assessment failed: {error}")
            return
        if ok:
            output = CompressedImage()
            output.header = message.header
            output.format = "jpeg"
            output.data = jpeg.tobytes()
            self.publisher.publish(output)

        if now - self.last_log_time >= 1.0:
            detections = []
            if result is not None and result.boxes is not None:
                for class_id, confidence, assessment in zip(
                    result.boxes.cls.int().cpu().tolist(),
                    result.boxes.conf.cpu().tolist(),
                    assessments,
                ):
                    if assessment['verdict'] == InspectionResult.UNCONFIRMED:
                        detections.append(
                            f"{result.names[class_id]}:{confidence:.2f} "
                            f"verdict=UNCONFIRMED ({assessment['reason']})")
                    else:
                        state = 'NORMAL' if assessment['verdict'] == InspectionResult.NORMAL else 'ABNORMAL'
                        detections.append(f"{result.names[class_id]}:{confidence:.2f} verdict={state}")
            summary = ", ".join(detections) if detections else "none"
            self.get_logger().info(f"detections={len(detections)} [{summary}]")
            if rack_report is not None:
                self.get_logger().info(f"rack verdict={rack_report['verdict']} reason={rack_report['reason']}")
            self.last_log_time = now

        if self.show:
            self._display_frame = annotated

    def destroy_node(self) -> bool:
        """노드가 끝날 때 켜 둔 영상 창과 ROS 통신 자원을 정리한다."""
        if self.show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        return super().destroy_node()


def main() -> None:
    """검출 노드를 시작하고 메인 스레드에서 화면을 갱신하다가 종료 요청이 오면 정리한다."""
    rclpy.init()
    node = None
    executor = None
    try:
        node = ScvDetectionManager()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok():
            if node.show and node._display_frame is not None:
                frame = node._display_frame
                node._display_frame = None
                try:
                    display = cv2.resize(frame, node.window_size)
                    cv2.imshow(node.window_name, display)
                    key = cv2.waitKey(30) & 0xFF
                    if key == ord('n'):
                        node.capture_rack_reference()
                    elif key == ord('r'):
                        node.rack_state.clear()
                        node.rack_reference = None
                except cv2.error as error:
                    node.get_logger().warning(f"Display failed; disabling show: {error}")
                    node.show = False
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""YOLO inference node for a ROS 2 CompressedImage camera topic."""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


DEFAULT_DATASET_DIR = "/home/mu-09/Downloads/Fire-Extinguisher-Guage.v2i.yolov8"
DEFAULT_WEIGHTS = str(
    Path(get_package_share_directory("vision_pkg")) / "resource" / "best.pt"
)
DEFAULT_TOPIC = "/robot4/oakd/rgb/image_raw/compressed"
DEFAULT_OUTPUT_TOPIC = "/robot4/fire_extinguisher/detections/compressed"


def newest_best_weights(dataset_dir: Path) -> Path | None:
    candidates = list((dataset_dir / "runs" / "detect").glob("*/weights/best.pt"))
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


class YoloCompressedDetector(Node):
    def __init__(self) -> None:
        super().__init__("fire_extinguisher_yolo_detector")

        self.declare_parameter("dataset_dir", DEFAULT_DATASET_DIR)
        self.declare_parameter("weights", DEFAULT_WEIGHTS)
        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "auto")
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("show", True)

        dataset_dir = Path(self.get_parameter("dataset_dir").value).expanduser().resolve()
        weights_value = str(self.get_parameter("weights").value).strip()
        weights = Path(weights_value).expanduser().resolve() if weights_value else newest_best_weights(dataset_dir)
        if weights is None:
            raise FileNotFoundError(
                f"No best.pt found below {dataset_dir / 'runs/detect'}. "
                "Finish training or set --ros-args -p weights:=/path/to/best.pt"
            )
        if not weights.is_file():
            raise FileNotFoundError(f"Weights file not found: {weights}")

        self.topic = str(self.get_parameter("topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        requested_device = str(self.get_parameter("device").value)
        self.device = (
            "0" if torch.cuda.is_available() else "cpu"
        ) if requested_device == "auto" else requested_device
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.show = bool(self.get_parameter("show").value)
        self.window_name = "Fire Extinguisher Gauge Detection"

        if self.show:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 540)
            waiting_frame = np.zeros((540, 960, 3), dtype=np.uint8)
            cv2.putText(
                waiting_frame,
                "Waiting for camera frames...",
                (230, 280),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, waiting_frame)
            cv2.waitKey(1)

        self.model = YOLO(str(weights))
        self.last_inference_time = 0.0
        self.last_log_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.output_topic, sensor_qos
        )
        self.subscription = self.create_subscription(
            CompressedImage, self.topic, self.image_callback, sensor_qos
        )

        self.get_logger().info(f"weights: {weights}")
        self.get_logger().info(f"device: {self.device}")
        self.get_logger().info(f"input: {self.topic}")
        self.get_logger().info(f"output: {self.output_topic}")

    def image_callback(self, message: CompressedImage) -> None:
        now = time.monotonic()
        if self.max_fps > 0 and now - self.last_inference_time < 1.0 / self.max_fps:
            return
        self.last_inference_time = now

        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            return

        try:
            result = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as error:
            self.get_logger().error(f"Inference failed: {error}")
            return

        annotated = result.plot()
        ok, jpeg = cv2.imencode(
            ".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if ok:
            output = CompressedImage()
            output.header = message.header
            output.format = "jpeg"
            output.data = jpeg.tobytes()
            self.publisher.publish(output)

        if now - self.last_log_time >= 1.0:
            detections = []
            if result.boxes is not None:
                for class_id, confidence in zip(
                    result.boxes.cls.int().cpu().tolist(),
                    result.boxes.conf.cpu().tolist(),
                ):
                    detections.append(f"{result.names[class_id]}:{confidence:.2f}")
            summary = ", ".join(detections) if detections else "none"
            self.get_logger().info(f"detections={len(detections)} [{summary}]")
            self.last_log_time = now

        if self.show:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if self.show:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = YoloCompressedDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

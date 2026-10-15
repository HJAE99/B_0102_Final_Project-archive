# Project and official-model evidence

Read-only inspection on 2026-10-15 (Asia/Seoul); web documentation fetched live. GitHub main contains little implementation; inspect `feature/vision`, commit `2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8`. Read sources in temporary clone `/tmp/finalproj_evidence_repo`; no user repo edits.

## Project structure and processing logic grounded in code

Quantitative model evidence comes exclusively from the local runs `yolov8n`,
`yolo11n`, and `yolo26n-2`. GitHub is used for system structure and downstream
inspection logic, not checkpoint evaluation.

- `scv_detection_manager.py` full-frame `predict` L1611 and crop `predict` L1139 pass conf/iou/imgsz/device/verbose, no `nms=False` or `end2end` override. Default processing cap `max_fps=10.0`, image size 640, conf .25. A configured FPS limit is not a measured achieved FPS. [Full-frame inference](https://github.com/tnduaehowl12/B_0102_Final_Project/blob/2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8/ros2_ws/src/scv_detection/scv_detection/scv_detection_manager.py#L1611), [defaults](https://github.com/tnduaehowl12/B_0102_Final_Project/blob/2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8/ros2_ws/src/scv_detection/scv_detection/scv_detection_manager.py#L125).
- Architecture: TurtleBot4 x2, OAK-D RGB/depth; robot RPi runs drivers; each robot carries an MSI GPU laptop running detection and Nav2/AMCL; separate Victus control laptop. Ubuntu24.04/ROS2 Jazzy/FastDDS. No Jetson evidence, and the robot deployment GPU model is not specified here. Training RTX3060 cannot automatically be identified with deployment laptop. [System design, deployment section](https://github.com/tnduaehowl12/B_0102_Final_Project/blob/2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8/docs/deliverables/system_design.md#L21).

## Why detector quality is relevant to the actual system

The system is detector + explicit downstream inspection, not a five-state end-to-end detector:

1. Extinguisher detection locates the body; pressure_gauge detection locates the dial. If a gauge is missing, the same model runs again on extinguisher crop. HSV color regions check yellow needle pixels inside the green/cyan normal region; no pressure reading in bar/MPa. Normal needs >=0.5s and >=3 fresh frames; abnormal can be immediate. A smaller detector leaves compute capacity for crop retries and other modules, but actual total latency needs measurement.
2. exit_sign bbox is passed to brightness, green ratio, bright-pixel ratio, and surrounding-contrast logic. ON/OFF needs temporal confirmation. Detector mAP is **not** ON/OFF accuracy.
3. rack and box detections are associated by overlap. Box bbox crossing rack left/right/top/bottom boundaries drives abnormal candidates, confirmed over three new frames. Better localization is relevant; mAP50-95 helps evaluate stricter localization on average. However mAP cannot prove lower physical protrusion error. This code measures **2D image boundary excess**, not actual depth-direction protrusion distance.
4. Shutter obstacle inspection is RGB-D + RANSAC and separate from YOLO; its improvements must not be credited to YOLO26.

[Current detection README](https://github.com/tnduaehowl12/B_0102_Final_Project/blob/2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8/ros2_ws/src/scv_detection/README.md), [2026-10-14 current-code notes and class table](https://github.com/tnduaehowl12/B_0102_Final_Project/blob/2e7e0518cf38672dd8cf13f0eaa7c3713fecf5b8/docs/daily/OmverYo/2026-10-14.md).

## Implemented vs planned

README explicitly says operational InspectFacility action server is not implemented; current Trigger service toggles processing. Current published inspection result conveys confirmed abnormal events, not complete normal/unconfirmed status. Old system_design.md describes planned action flow and old OpenCV-only rack design; prefer current manager, modules, and 10/14 notes. Repository docs from 10/11–12 contain YOLOv8 state-class model (Normal/Abnormal_left/Abnormal_right/Exit_sign_on/off) and very high small-dataset validation scores. Those are **different label tasks/data** and cannot be merged into current five-object comparison. No current 3-model comparative benchmark was found in feature/vision sources; local new results must supply it.

## Official model background, kept separate from custom-data results

Sources:
- [YOLO26 official docs](https://docs.ultralytics.com/models/yolo26/)
- [YOLO11 official docs](https://docs.ultralytics.com/models/yolo11/)
- [YOLOv8 official docs](https://docs.ultralytics.com/models/yolov8/)

COCO 640 metrics listed at access:

| model | COCO mAP50-95 | params M | FLOPs B | CPU ONNX ms | GPU ms |
|---|---:|---:|---:|---:|---|
| YOLOv8n | 37.3 | 3.2 | 8.7 | 80.4 | .99 A100 TensorRT |
| YOLO11n | 39.5 | 2.6 | 6.5 | 56.1±.8 | 1.5 T4 TensorRT10 |
| YOLO26n | 40.9 (40.1 e2e) | 2.4 fused | 5.5 fused | 38.9±.7 | 1.7 T4 TensorRT10 |

YOLO26 uses a lighter DFL-free regression head; one-to-one NMS-free inference is optional (`nms=False`), default one-to-many uses NMS for accuracy. Docs describe Progressive Loss + small-target-aware label assignment and MuSGD recipe. Their existence is architecture/training-background support, not proof that this custom run used MuSGD or improved pressure-gauge AP. Docs say parameters/FLOPs are fused after unused branch removal; speed is measured with NMS-free head. Therefore do not use official 2.4M vs full training checkpoint counts without labeling, and do not claim deployed model is NMS-free. Official claims about CPU acceleration are specific ONNX/hardware/head benchmarks, **not** RTX3060 or robot deployment observations. v8 GPU hardware differs; even same-table 26n T4 latency is 1.7ms versus 11n 1.5ms, so "26n is fastest on every GPU" is unsupported.

## Connection to the selected comparison

The local comparison of `yolov8n`, `yolo11n`, and `yolo26n-2` supplies the
model-specific metrics and CPU-computed model complexity. GitHub and the shared
system diagram supply the context: a common five-object detector feeding
pressure-gauge, illumination, and rack/box inspection logic.

Use the measured detection performance and computational size to explain the
accuracy/compute trade-off. Actual latency, FPS, and complete inspection verdict
accuracy must be evaluated separately. No source here establishes that the
local `yolo26n-2` checkpoint has been loaded on a robot.

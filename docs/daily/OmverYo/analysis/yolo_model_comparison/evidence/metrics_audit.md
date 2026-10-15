# YOLO model evidence audit

Project root: `/home/hv-05/Downloads/finalproj.v5`.
No training or dataset evaluation was run. Checkpoint metadata and CPU-only
model-complexity analysis were used; original project files were not modified.

## Run mapping and comparable conditions

Use `runs/fire_extinguisher/yolov8n`, `yolo11n`, and **`yolo26n-2`**.
All YOLO26n figures and claims in this audit refer to `yolo26n-2/weights/best.pt`.
The latest three checkpoints report Ultralytics **8.4.177**, AdamW, and identical
saved `train_args` except the input model path and run name. Saved args use
100 max epochs, batch 8, imgsz 640, patience 30, seed 42, deterministic true,
AMP true, fraction 1.0, device 0; same resolved dataset and augmentations.
Thus these are comparable configured runs, but only one run per model is available
for this comparison and there is no repeated-seed uncertainty estimate.

Sources: `runs/fire_extinguisher/metrics_summary.json`; each run's `args.yaml`,
`results.csv`, and `weights/best.pt` checkpoint metadata.

## Best-checkpoint reevaluation (primary comparison table)

All values below are percentages, directly from `metrics_summary.json`.

| Split | Model | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---:|---:|---:|---:|
| Validation | YOLOv8n | 98.0656 | 96.6936 | 97.1977 | 88.4205 |
| Validation | YOLO11n | 97.2832 | 96.6540 | 97.2701 | 88.2822 |
| Validation | YOLO26n | 97.2288 | 97.4512 | 97.4661 | 89.2575 |
| Test | YOLOv8n | 98.2070 | 97.1419 | 98.1932 | 90.3176 |
| Test | YOLO11n | 97.1433 | 97.6870 | 97.0365 | 88.7482 |
| Test | YOLO26n | 97.0541 | 97.3187 | 98.0560 | 89.5040 |

YOLO26 has the highest validation mAP50-95 (+0.837 percentage points vs v8,
+0.975 vs 11), validation recall, and validation mAP50. On the test set,
YOLOv8n has the highest mAP50-95, exceeding YOLO26 by **0.814 percentage points**;
YOLO26 exceeds YOLO11 by 0.756 points. YOLO11 has the highest test recall.
These figures support retaining YOLO26 as a capable, comparatively compact
choice. They do not prove universal accuracy superiority.

The `results.csv` epoch metrics are a different recorded evaluation stage from
the post-training best.pt reevaluation. Do not substitute a training-curve peak
for a summary metric without clearly labeling it.

## Convergence and time from training CSVs

| Model | Completed epochs | Best epoch by validation mAP50-95 | Peak curve mAP50-95 | Cumulative time at peak | Final cumulative training time |
|---|---:|---:|---:|---:|---:|
| YOLOv8n | 100 | 90 | 88.391% | 510.972 s | 568.156 s |
| YOLO11n | 100 | 92 | 88.345% | 541.136 s | 587.572 s |
| YOLO26n | 86 | 56 | 89.433% | 399.861 s | 613.868 s |

The YOLO26 final epoch is exactly best epoch + patience (56 + 30 = 86), consistent
with configured early stopping. It reached its highest observed validation metric
in fewer epochs. Its total recorded training wall time was nevertheless longer.
Do not claim fastest total training or fastest per-epoch training.

First epoch reaching validation mAP50-95 >= 88%: v8 75 (425.882 s), 11 87
(511.680 s), 26 40 (286.185 s). This is descriptive for these particular runs;
the threshold should be labeled if used and should not replace the full curve.
Median per-epoch times after epoch 1: v8 5.668 s, 11 5.780 s, 26 7.137 s.

At epoch 20, curve mAP50-95 values were v8 83.303%, 11 82.259%, 26 85.262%;
at epoch 40: 86.869%, 85.783%, 88.253%. A full validation-mAP curve is clearer
than a cross-model raw-loss comparison. YOLOv8/11 expose DFL loss, while YOLO26
exposes L1 loss; head/loss definitions differ, so raw loss magnitude is not a
fair cross-model accuracy ranking.

Exact curve references: `yolov8n/results.csv:91` (epoch 90),
`yolo11n/results.csv:93` (epoch 92), `yolo26n-2/results.csv:57` (epoch 56),
under `runs/fire_extinguisher/`.

## CPU-computed inference-model complexity

Reproduction script: `evidence/measure_complexity.py`.
Output: `evidence/complexity_measurement.json`.
Figure input: `complexity_evidence.json`.

Environment: PyTorch 2.11.0+cu128, Ultralytics 8.4.177; CUDA disabled for this
analysis. For each best.pt: load YOLO model; FP32 CPU eval; call `model.fuse()`;
sum parameter `numel`; call installed `get_flops(model, imgsz=640)`.
This creates only an in-memory model copy and does not save altered weights.

| Model | Stored unfused parameters | Fused inference parameters | Estimated GFLOPs at 640 |
|---|---:|---:|---:|
| YOLOv8n | 3,138,975 | 3,133,679 | 8.6431872 |
| YOLO11n | 2,605,855 | 2,598,023 | 6.4417024 |
| YOLO26n | 2,535,830 | 2,390,707 | 5.3795072 |

YOLO26 fused parameters are 23.71% below v8 and 7.98% below 11; estimated
GFLOPs are 37.76% below v8 and 16.49% below 11. This supports lower inference
model size/arithmetic requirements. It does not establish measured FPS, latency,
peak memory, power, or end-to-end robot throughput.

Important implementation qualification: YAML says `end2end: true`, but the
actual saved latest YOLO26 head has `_end2end=False`. Both raw checkpoint and
`YOLO(...).model.end2end` resolve to False in this installed environment.
Thus the current measured/evaluated configuration uses one-to-many inference;
do **not** describe actual benchmark results as NMS-free inference.
Fusion removes the unused one-to-one branch (136,603 unfused parameters) as
well as batch normalization, explaining the larger YOLO26 parameter decrease.
The model still contains architecture/configuration support for a one-to-one
head before fusion, which is distinct from actually using that mode.

## Dataset scope and label counts

Source `runs/fire_extinguisher/data_resolved.yaml:5`: **five detection classes**
`box`, `exit_sign`, `fire_extinguisher`, `pressure_gauge`, `rack`.
There are no labels for exit-sign ON/OFF, gauge reading/status, or box
protrusion/normal state. Detection metrics alone cannot establish those downstream
decision accuracies; they only support detection/localization feeding those steps.

Counts from current image and label directories:

| Split | Images/label files | box instances | exit_sign | fire_extinguisher | pressure_gauge | rack | Total instances |
|---|---:|---:|---:|---:|---:|---:|---:|
| train | 1000 | 1251 | 306 | 289 | 260 | 635 | 2741 |
| valid | 71 | 78 | 20 | 27 | 27 | 42 | 194 |
| test | 33 | 41 | 9 | 11 | 11 | 18 | 90 |

Test images containing each class: box 16, exit_sign 5, fire_extinguisher 11,
pressure_gauge 11, rack 13. Validation: box 29, exit_sign 11,
fire_extinguisher 27, pressure_gauge 27, rack 31. Single images may contain
multiple classes. Test class counts are small; no statistical-significance claim
or real-warehouse deployment-generalization claim follows from these results.
The training directory contains augmented filename variants; source-scene/split
independence was not audited here and must not be assumed from image counts.

## Presentation framing for the selected runs

“이번에 동일 데이터와 학습 설정으로 학습한 YOLOv8n, YOLO11n,
YOLO26n을 비교했다. YOLO26n의 분석 대상은 yolo26n-2 실행이다.
YOLO26n은 검증셋에서 가장 높은 mAP50-95와 재현율을 기록했고,
테스트셋에서도 mAP50 98.06%, mAP50-95 89.50%를 확보했다.
테스트 mAP50-95는 YOLOv8n이 소폭 높았지만, YOLO26n의 추론용 모델은
파라미터와 추정 연산량이 더 작았다. 이 결과를 탐지 성능과 계산 자원의
균형을 고려한 선정 근거로 제시한다.”

Robot FPS and downstream state-decision accuracy require separate measurement.

# scv_detection

기본 실행은 카메라 미리보기다. 서비스 요청이 들어올 때만 YOLO 또는 랙 검사
한 가지를 계속 실행한다. 같은 서비스를 다시 호출하면 중지하고 원본 카메라로 돌아온다.
실행 진입점은 `scv_detection_manager.py:main`, 클래스는 `ScvDetectionManager`,
노드 이름은 `detection_alert`다. 운영용 `InspectFacility` 액션 서버는 아직 구현하지 않았다.

## 파일 역할

```text
scv_detection/
├── scv_detection_manager.py  # 카메라·두 서비스·YOLO 추론·클래스 색상·GUI
└── rack_conditioner.py       # 파이프 자동 검출·정상 기준 비교·랙 표시
```

매니저의 `annotate_yolo_result` 하나가 검출된 모든 클래스의 박스·이름·신뢰도를
표시하고 로그 문자열을 반환한다. 독립 YOLO 표시 모듈과 별도 판정 목록은 없다.
랙 알고리즘은 함수 모듈에 두며 영상 수신·모델 실행·ROS 통신은 매니저에서 관리한다.

## 기본 실행

```bash
ros2 run scv_detection scv_detection_manager
```

인자 없이 실행하면 `/robot4/oakd/rgb/image_raw/compressed`를 구독하고
카메라 창을 띄운다. 대기 중에는 YOLO 추론, 랙 파이프 검출, ROI 표시를 하지 않는다.
카메라가 아직 연결되지 않았으면 창에 영상을 기다린다는 안내를 표시한다.

설치된 `share/scv_detection/models/best.pt`를 자동으로 찾는다.
YOLO 모델은 첫 YOLO 서비스 요청에서 로딩하고 이후 재사용한다.
잘못된 모델 경로는 YOLO 요청을 실패시키며 카메라 실행은 계속된다.
화면 환경이 없으면 GUI만 끄고 영상 발행과 서비스는 유지한다.

다른 로봇은 이름공간을 바꾼다. 예:

```bash
ros2 run scv_detection scv_detection_manager --ros-args -r __ns:=/robot3
```

화면 없는 실행은 `-p show:=false`, 다른 모델은 `-p weights:=/path/to/model.pt`다.
`enable_yolo`, `rack_enabled`는 서비스 사용 허용 옵션이며 기본값은 둘 다 true다.
두 옵션을 모두 false로 설정한 카메라 전용 실행도 가능하다.

## 서비스 호출

노드 실행 터미널을 유지하고 다른 터미널에서 호출한다.

YOLO 검사:

```bash
ros2 service call /robot4/detection_alert/inspect_yolo std_srvs/srv/Trigger "{}"
```

랙 검사:

```bash
ros2 service call /robot4/detection_alert/inspect_rack std_srvs/srv/Trigger "{}"
```

YOLO 요청에서는 YOLO만, 랙 요청에서는 랙만 실행한다.
다른 로봇을 쓰면 서비스 주소의 robot4도 해당 이름공간으로 바꾼다.
같은 서비스를 처음 호출하면 시작, 다시 호출하면 중지한다. 자동 종료 시간은 없다.
다른 검사를 시작하려면 실행 중인 검사의 서비스를 다시 호출해 먼저 중지한다.
다른 모드가 실행 중이거나 모델 로딩·기준 변경 중이면 `success=false, message="busy: …"`로 응답한다.
기준 저장·초기화도 검사 중에는 거절한다.

| 서비스 | 대기 중 호출 | 같은 검사 실행 중 호출 |
|---|---|---|
| inspect_yolo | YOLO 시작 | YOLO 중지 |
| inspect_rack | 랙 검사 시작 | 랙 검사 중지 |

응답의 success는 시작·중지 성공 여부다. message의 `started`/`stopped`로 확인한다.
서비스는 최종 판정을 기다리지 않는다. 실제 검출·랙 판정은 영상과 로그로 계속 표시한다.
랙은 기본 3프레임 연속 판정 후 확정한다. 첫 YOLO 시작은 모델 로딩 시간이 걸릴 수 있다.
카메라가 없거나 최근 2초 동안 유효 영상이 없으면 시작 요청을 실패시킨다.
중지 요청은 카메라가 끊겼거나 기준 사진이 없어도 처리한다.
영상 유효 시간은 `camera_timeout`으로 바꾸며 예전 duration/inspect_timeout 파라미터는 제거했다.

## 랙 기준 준비

기존 정상 기준 NPZ가 있으면 시작 시 로드한다.
없거나 카메라 위치가 달라졌다면 대기 화면에서 다음처럼 준비한다.

1. 상자를 모두 랙 안에 넣고, 랙의 흰색 좌우 파이프가 ROI 안에 보이게 한다.
2. 영상 창에서 N을 눌러 정상 기준을 저장한다. 네 모서리 클릭은 필요 없다.
3. 상자를 옆으로 움직인 뒤 랙 서비스를 호출한다.
4. R은 메모리 기준만 초기화하며 저장 파일은 삭제하지 않는다.

기본 ROI 비율은 (x1=0.25, y1=0.30, x2=0.75, y2=0.83)이다.
필요하면 `rack_roi_x1/y1/x2/y2`를 조절한다.
ROI 테두리와 검사 범위는 랙 검사 중에만 표시된다.
`rack_reference_path`로 저장 파일을 명시할 수 있다.
기본 경로는 설치 위치에서 찾은 소스 `config/rack_reference.npz`이며,
소스 config 폴더가 없으면 기준을 메모리에 저장한다.

랙 비교는 BGR 최대 채널 차이를 사용한다.
기본 `rack_margin=30`, `rack_zone_width=90`은 경계 바깥 30~90픽셀 구간을 뜻한다.
`rack_diff_threshold=35`, `rack_min_area=300`, `rack_confirm_frames=3`이다.
사람·그림자·조명 변화도 검출될 수 있으며 실제 앞뒤 돌출 거리는 측정하지 않는다.

## 현재 모델과 색상

현재 설치 모델은 6개 클래스다. 색상과 로그 상태는 `YOLO_CLASS_STYLES`에서
ID로 직접 선택한다. 모델 ID 순서가 바뀌면 색상표도 갱신해야 한다.
등록되지 않은 새 ID도 회색으로 표시한다.

| ID | 클래스 | 색상 | 로그 상태 |
|---:|---|---|---|
| 0 | Abnormal_left | 빨강 | ABNORMAL |
| 1 | Abnormal_right | 빨강 | ABNORMAL |
| 2 | Fire Extinguisher | 회색 | UNCONFIRMED |
| 3 | Normal | 초록 | NORMAL |
| 4 | Exit_sign_on | 초록 | NORMAL |
| 5 | Exit_sign_off | 빨강 | ABNORMAL |

전체 소화기 검출만으로 상태를 정상이라고 판단하지 않는다.
압력계 정면성 검사는 하지 않으며 모델이 실제로 검출한 박스만 표시한다.

## 스레드와 입출력

메인 스레드는 GUI·N/R 키를 처리하고, 백그라운드 executor의 세 워커는
영상 처리·서비스 시작 및 중지·겹치는 요청 응답을 담당한다.
상태 잠금과 검사 잠금으로 요청을 분리하고 끝난 요청의 늦은 결과는 버린다.

입력: `oakd/rgb/image_raw/compressed`.
출력: `fire_extinguisher/detections/compressed`.
로봇 이름공간이 각 토픽 앞에 붙는다. 출력 header는 입력에서 복사한다.
현재 QoS는 RELIABLE / KEEP_LAST / depth 1이다.
`InspectionResult` 메시지는 발행하지 않고 랙 판정 상수만 사용한다.

## 설치

OpenCV·NumPy·Torch는 rosdep 키로 선언했다. Ultralytics는 기본 rosdep에 키가
없어 함께 제공한 `rosdep.yaml`을 등록한다. 저장소 루트에서 실행한다.

```bash
printf 'yaml file://%s/ros2_ws/src/scv_detection/rosdep.yaml\n' "$PWD" \
  | sudo tee /etc/ros/rosdep/sources.list.d/10-scv-detection.list
rosdep update
rosdep install --from-paths ros2_ws/src/scv_detection ros2_ws/src/scv_msgs --ignore-src -r -y
```

`setup.py`에도 `numpy`, `opencv-python`, `torch`, `ultralytics` 의존을 선언했다.
colcon의 Python과 실행 환경의 Python에 의존성이 설치되어 있어야 한다.
GPU용 PyTorch를 쓰는 PC는 기존 CUDA 환경을 유지한다.

## 빌드와 단독 launch

```bash
cd ~/final_ws/ros2_ws
colcon build --symlink-install --packages-select scv_detection
source install/setup.bash
ros2 run scv_detection scv_detection_manager
```

단독 launch도 같은 서비스 구조를 사용한다.

```bash
ros2 launch scv_detection detection_alert.launch.py
```

통합 bringup 변경은 별도 통합 작업에서 진행한다.

## 이전 5클래스 모델 학습 기록

아래 학습 수치와 해시는 이전 5클래스 모델의 기록이다. 현재 설치 모델의 6개 클래스와 혼동하지 않는다. 10/12에 기존 라벨 캐시를 백업하고
같은 277장 데이터셋으로 기존 모델을 추가 학습한
`fire-extinguisher.v1i.yolov8/runs/review_finetune_corrected_labels_20261012/yolov8n/weights/best.pt`
다. 예전 `Guage.v2i` 기본 데이터셋 경로는 제거했다.

SHA-256: `aff903afa7dbd8fea7ef36180141f9b0c6fd7c01513d4c1b8469d27d8956d35f`

이전 10/11 모델 SHA-256은
`0f7bcaa5f1cd098bf81c9b71b8446af037dc5d26747f5401eff772ea63795edb`다.
기존 학습 캐시에는 소등 라벨이 0건으로 남아 있었으므로, 라벨을 변경한
뒤에는 `train/labels.cache`·`valid/labels.cache`를 백업하고 재생성해야 한다.
검증 재현 도구는 저장소 루트의 `scripts/audit_detection_dataset.py`다.
재학습은 `scripts/train_detection_model.py`로 하면 모든 split의 라벨 캐시를
먼저 백업하고 재생성한다. `--data`, `--weights`, `--output-dir`를 명시한다.

| ID | 클래스 | 표시색 |
| --- | --- | --- |
| 0 | Abnormal_left | 빨강 |
| 1 | Abnormal_right | 빨강 |
| 2 | Normal | 초록 |
| 3 | Exit_sign_on | 초록 |
| 4 | Exit_sign_off | 빨강 |

비상구 두 클래스의 원본이 각각 1장인 한계는 남아 있다. 증강본을 분할한
검증 수치는 새로운 환경의 성능을 보장하지 않는다. 수집 조건별 장수와
실영상 신뢰도는 실제 촬영 기록으로 별도 확인한다.
최종 모델의 검증 precision은 0.9353으로 전체 95% 기준을 통과하지 못했다.
현재 모델은 진단용이며, 운영용 채택은 추가 실촬영 검증과 PM 판단이 필요하다.

## 다음 Step

- 운영용 InspectFacility 액션·취소·설비 결과 메시지 연동.
- 실제 카메라에서 조명·위치 변화에 대한 정확도 검증.
- 다중 랙과 셔터 위 물체 검사 설계.

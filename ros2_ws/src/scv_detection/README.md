# scv_detection

`vision_pkg/test`를 `scv_detection`으로 이관했다. 실행 파일은
`scv_detection_manager.py`, 클래스는 `ScvDetectionManager`다. 현재 실행기는
학습 모델 확인용 **상시 구독 진단 노드**다. `InspectFacility` 액션 요청 중에만
구독하고 3장을 판정하는 IF-05·14 기능은 다음 Step에서 구현해야 한다.
아직 운영용 액션 서버로 launch에 연결하지 않는다.

## 파일 역할과 기능 추가 방식

```text
scv_detection/
├── scv_detection_manager.py  # ROS 입출력·모델 추론·화면 표시 관리
├── pressure_gauge.py        # 압력계 정면 확인·정상/비정상 판정 함수
├── exit_sign.py             # 비상구 점등/소등 판정 함수
├── rack_conditioner.py      # 직접 선택한 랙 경계·상자 윤곽으로 옆방향 돌출 검사
└── detection_render.py      # 담당 함수 호출·공통 박스 색상/라벨 표시
```

manager 하나만 실행하면 두 설비를 함께 검사한다. 기능 모듈은 별도 ROS
노드나 클래스를 만들지 않고 함수만 제공한다. 각 모듈의 `supports(class_name)`은
담당 클래스인지 확인하고, `assess(class_name, frame, box, min_size, min_axis_ratio)`는
`verdict`, `frontal`, `reason`을 담은 사전을 반환한다. 사용하지 않는 인자는 무시한다.
새 설비 기능도 같은 방식으로 `.py` 파일에 작성하고 `detection_render.py`의
`DETECTORS`에 등록한다. 영상 수신·추론·창 코드를 새 모듈에 복사하지 않는다.
새 클래스 검출에는 해당 클래스를 학습한 모델도 필요하다.
단, `rack_conditioner`는 YOLO 클래스 등록 없이 manager에서 원본 프레임을
직접 전달받는다. OpenCV만 사용하는 독립 기능이다.

ROS 노드 이름 `detection_alert`는 기존 인터페이스와 맞게 유지한다.
예전 실행 명령 `detection_alert`도 같은 manager를 호출하는 호환 별칭이다.
10/12 구조 변경 후 로컬·MSI에서 각 30개 자동시험과 manager 단독 실행·통합
launch의 화면 없는 640×480 JPEG 입출력을 확인했다. 모델 파일은 변경하지 않았다.

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

## 빌드 및 실행

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select scv_detection
source install/setup.bash
ros2 launch scv_detection detection_alert.launch.py \
  robot_ns:=robot3 \
  weights:="$(ros2 pkg prefix --share scv_detection)/models/best.pt" \
  show:=false
```

설비 점검 로봇 B는 `robot3`이고, 진단 목적으로 다른 로봇을 사용할 때는
launch의 `robot_ns` 인자만 바꾼다. 번호를 검출 코드에 고정하지 않는다.
`PushRosNamespace`가 상대 토픽 앞에 이름공간을 붙인다.
직접 실행할 때는 `ros2 run scv_detection scv_detection_manager --ros-args
-r __ns:=/robot3 -p weights:=<모델 경로>`처럼 ROS 이름공간을 지정한다.
`weights`가 비어 있거나 노드 이름공간이 루트(`/`)이면 바로 오류로 종료한다.
단, `enable_yolo=false`, `rack_enabled=true`인 랙 전용 실행에는 weights가 필요 없다.

## OpenCV 랙 검사 준비와 실행

1. 상자를 모두 경계 안에 둔 정상 상태에서, 영상 창의 랙 영역 좌상·우상·우하·좌하를 차례로 클릭한다.
   노란 번호 점이 표시되고, 네 점이 유효하면 지정된 경계가 그려진다.
2. `n` 키를 눌러 지금 모습을 정상 기준 사진으로 저장한다. 이후 상자를 바깥으로 옮겨 시험한다.
3. 다시 지정하려면 오른쪽 클릭이나 `r` 키로 지운 뒤 정상 상태에서 네 점을 새로 선택하고 `n`을 누른다.
   `rack_enabled=true`를 켜야 하며 기본값은 false다.

랙만 검사하는 예시(학습·pt 파일 불필요):

```bash
ros2 run scv_detection scv_detection_manager --ros-args \
  -r __ns:=/robot3 -p enable_yolo:=false -p rack_enabled:=true -p show:=true
```

압력계·비상구와 동시에 사용할 때는 기존 weights 인자에 `-p rack_enabled:=true`만
추가한다. 전용 launch에도 `enable_yolo:=false rack_enabled:=true`를 지정할 수 있다.
통합 launch에서는 `detection_enable_yolo`, `detection_rack_enabled`를 사용한다.
출력은 기존 `fire_extinguisher/detections/compressed` 영상에 함께 표시한다.

검사 순서: 네 모서리 지정 → 정상 기준 사진 저장 → 정렬한 두 영상의 밝기 차이를
비교 → 작은 변화를 제거 → 경계 바깥의 큰 새 변화가 3프레임 이어지면 돌출 판정.
모서리나 정상 기준 사진이 없으면 회색 UNCONFIRMED다. 사람·그림자·다른 물체도
기준 사진에서 달라진 바깥 영역으로 잡힐 수 있어 조명과 카메라 위치를 고정해야 한다.

조절 파라미터:

- `rack_tolerance=8`: 정렬된 영상에서 허용하는 좌우 초과 픽셀 수.
- `rack_min_box_area=600`: 정렬 영상에서 상자 후보 최소 면적.
- `rack_confirm_frames=3`: 같은 판정의 연속 확인 횟수.

경계는 정렬 영상에서 폭 512px로 환산하므로 tolerance와 면적은 원본 해상도나
실제 cm가 아니다. 앞뒤 깊이 돌출, 완전히 가려진 상자, 모든 층의 검사는 현재
범위가 아니다. 조명이나 카메라가 달라지면 바깥 변화가 잘못 잡힐 수 있어 실제
랙 영상으로 확인해야 한다.
카메라 위치나 줌이 달라지면 네 점을 다시 지정해야 한다. 현재 첨부 화면처럼
랙 경계가 영상에 보이면 테이프 부착 없이 사용할 수 있다.
로컬·MSI 각 45개 자동시험 및 모델 없이 manager 단독 실행의 합성
640×480 JPEG 입출력을 확인했다. 실제 랙 영상의 정확도 시험은 아직 수행하지 않았다.

## launch 없이 두 로봇을 운용할 때

manager는 `ros2 run`으로 단독 실행한다. 두 로봇을 동시에 사용한다고 launch가
필수인 것은 아니다. 각 실행에서 `-r __ns:=/robot3` 또는 `-r __ns:=/robot4`를
지정하면 입력·출력 토픽과 노드 주소가 분리된다. 기본 역할은 robot3 설비 검사,
robot4 RSSI 측정이며, robot4에서도 비전 기능을 시험할 때만 manager를 실행한다.
같은 로봇 이름공간에서 manager를 두 번 실행하지 않는다.
각 PC의 ROS_DOMAIN_ID=4와 디스커버리 서버 설정을 맞춘다.
원격 PC의 실행은 해당 PC에서 해야 하며 launch가 자동으로 SSH 실행해 주지는 않는다.
기존 launch는 선택 실행 방법으로 남겨 두지만 새 동시운용 launch는 추가하지 않는다.

- 입력 상대 이름: `oakd/rgb/image_raw/compressed`
- 진단 이미지 출력 상대 이름: `fire_extinguisher/detections/compressed`
- 예: launch 이름공간이 `/robot3`이면 입력 주소는 `/robot3/oakd/rgb/image_raw/compressed`.
- QoS: best effort · keep last · depth 1
- `show` 기본값: `false`. 데스크톱에서 `-p show:=true`로 창을 켠다.
  디스플레이 환경이 없거나 OpenCV 창 생성/표시에 실패하면 화면을 끈다.
- 모델 클래스와 신뢰도에 따라 박스·라벨을 표시한다. 정상/점등은 초록색,
  비정상/소등은 빨간색이다. 미등록 클래스는 회색이다.

## 압력계의 정면 판별과 판정 보류

검출된 압력계 부분에서 테두리를 찾아 원/타원으로 맞추고 짧은 지름과 긴
지름의 비율을 확인한다. `frontal_min_axis_ratio=0.90` 이상이면 정면 후보,
미만이면 비정면으로 보류한다. `frontal_min_size=48` 픽셀보다 작거나,
테두리가 불분명하거나 가려지면 정면 여부를 알 수 없어 보류한다.
정면으로 확인된 압력계만 NORMAL/ABNORMAL로 표시하며, 나머지는 회색
UNCONFIRMED로 표시하고 정상/이상 판정을 로그에 내보내지 않는다.
이 값들은 노드 ROS 파라미터로 조정할 수 있다.

이 방법은 테두리 모양을 사용한 보수적 초기 구현이다. 실제 촬영의 원근,
반사·가림·인쇄 왜곡에 영향을 받는다. 원/타원 합성 그림 시험은 코드 분기를
검증할 뿐, 소품 시나리오의 정확도 ≥ 90%를 증명하지 않는다. 정면·비정면
실촬영과 사람이 확인한 정답으로 별도 시험해야 한다.

모델 이름은 `InspectionResult`의 NORMAL/ABNORMAL/UNCONFIRMED 상수로
매핑한다. 현재는 진단 로그와 표시에서만 사용한다. IF-19 설비 결과 발행과
InspectFacility 피드백·자세 보정 연동은 다음 Step이다.

로봇 PC 묶음 launch에서도 다음처럼 진단 실행을 명시할 수 있다(기본 비활성).

```bash
ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot3 \
  start_detection_diagnostics:=true \
  detection_weights:="$(ros2 pkg prefix --share scv_detection)/models/best.pt" \
  detection_show:=false
```

## 최종 모델

설치하는 모델은 `models/best.pt` 하나다. 10/12에 기존 라벨 캐시를 백업하고
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

- `InspectFacility` 요청 때만 구독 → 3장 → 판정 → 구독 해제.
- 취소·타임아웃·미검출·정면 판별 결과에 따른 자세 보정·IF-19 결과 연동.
- 추론 작업은 액션 콜백을 막지 않게 worker/콜백 그룹과
  `MultiThreadedExecutor`로 분리하고, 요청/취소 응답 시간을 측정.
- 소화기 유무·위치와 셔터 띠 위 물체는 현재 5클래스 모델이 판별하지 않음.

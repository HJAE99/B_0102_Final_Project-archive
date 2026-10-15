# 방화셔터 구역의 RANSAC 장애물 검사

구현일: 2026-10-14. 빌드·검출 노드 실행·알고리즘 테스트는 수행하지 않았다.
사용자의 요청으로 robot4의 카메라 파라미터·CameraInfo·영상 헤더만 읽었다. 영상 픽셀 판독이나 로봇 이동은 하지 않았다.

화면 좌측 상단에 다음 문구를 표시한다. 기존 영상 발행 토픽에도 같은 문구가 들어간다.

| 판정 | 문구 | 색 |
|---|---|---|
| 장애물 확정 | `경고)장애물을 치워주십시오` | 빨강 |
| 정상 확정 | `정상)장애물이 없습니다` | 초록 |
| 입력 부족·품질 실패·확인 중 | 미확인 안내 | 회색 |

화면 표시는 2026-10-15 에 바꿨다. RANSAC 평면 기준으로 바닥을 지우고 남은 픽셀, 즉 빨간 경계 안에서
바닥 분류 거리보다 높은 유효 Depth 픽셀과 장애물 전체 범위(구역 윗변 위로 넘어간 부분 포함, 같은 거리대로 이어진 픽셀)만
Depth 색(0.3~4.0m, 가까울수록 파랑·멀수록 빨강)으로 70% 덮어 그린다. 빨간 선 안에서 그 외의 곳(지운 바닥과 빈 Depth)은
바닥임을 알 수 있게 보라색(`FLOOR_FILL_BGR`, 35%)으로 고르게 채운다. 이전에는 바닥 픽셀마다 빨강을 40% 섞었는데
빈 Depth 구멍이 잡음처럼 드러나 보기 어려웠다. 이 표시는 그림 전용이며 판정에는 쓰지 않는다.
바닥보다 분류 거리 넘게 아래로 잰 점은 스테레오 Depth 오차로 보고 바닥과 함께 지운다(관측 범위 계산에는 넣지 않음).
좌측 상단 두 번째 줄에 `RANSAC 바닥: 검출 (N픽셀)` 또는 `미검출`과 `reason`을 표시한다.
정렬·TF·구역 검출 등 앞 단계에서 실패했거나 RANSAC 평면이 검증을 통과하지 못했으면 Depth 색 표시도 없다.
구역을 찾으면 해당 구역 안에서 선택된 빨간 선의 윤곽만 노란색으로 표시한다. 구역을 찾지 못한 경우에는
원인을 확인할 수 있도록 HSV 색 조건으로 인식한 선의 윤곽을 표시한다. 화면에 빨간 픽셀 수,
구역 후보 수, 구역 검출 실패 단계도 표시한다. 노란 윤곽은 색 인식 결과이며 바닥 평면 판정과는 별도다.
위치 관계로 결합한 선 조각 수와 `바닥 확인 % / 유효 Depth % / 바닥 분류 거리 mm`도 표시한다.
장애물 후보는 확인 중 주황 박스, 확정 후 빨간 박스로 표시하며 후보 높이를 mm로 적는다.
연속 확인 프레임 수·경과 시간과 `verified_optical_flow`(현재 영상으로 검증한 구역 추적) 여부도 표시한다.
바닥 확인 비율은 빨간 경계 안쪽 전체 픽셀 중 실제 바닥으로 분류된 비율이다.
Depth가 없거나 평면에서 많이 벗어난 곳은 채워 칠하지 않으므로 색이 비는 원인을 이 수치로 구분한다.
`zone_not_found`일 때는 RANSAC을 아직 실행하지 않았으므로 `RANSAC 바닥: 구역 검출 대기`로 표시한다.

## 실행과 robot3 / robot4 선택

```bash
cd /home/mu-09/final_ws/ros2_ws
colcon build --packages-select scv_detection
source install/setup.bash
ros2 run scv_detection scv_detection_manager
```

기본 이름공간은 `/robot4`다. 실행 인자 없이 미리보기와 카메라 정렬 설정 자동 조회가 시작된다.
다른 터미널에서 셔터 검사를 켠다.

```bash
source /home/mu-09/final_ws/ros2_ws/install/setup.bash
ros2 service call /robot4/detection_alert/inspect_shutter std_srvs/srv/Trigger "{}"
```

같은 서비스를 다시 호출하면 검사 종료다. 기존 YOLO 또는 랙 검사 중이면 먼저 해당 서비스를 다시 호출해 중지한다.
셔터 검사는 YOLO 모델을 사용하지 않으며 세 검사 모드는 동시에 활성화되지 않는다.
정렬 확인 중에 서비스를 켜면 자동 확인 안내가 나온다. 확인되면 같은 검사 요청을 그대로 이어간다.
robot3은 매니저 실행에 `--ros-args -r __ns:=/robot3`을 추가하고 서비스 주소도 `/robot3/`으로 바꾼다.

## 필요한 입력과 정렬 설정

2026-10-14 robot4에서 읽은 설정은 다음과 같다. robot3의 설정까지 확인한 것은 아니다.

| 읽은 항목 | 값 |
|---|---|
| `stereo.i_align_depth` | `true` |
| RGB 설정 FPS / 좌·우 카메라 설정 FPS | 30 / 10 |
| RGB / Depth 크기 | 둘 다 640×480 |
| RGB / Depth 광학 프레임 | 둘 다 `oakd_rgb_camera_optical_frame` |
| RGB / Depth 왜곡값 D | RGB는 비영 값, Depth는 모두 0 |
| RGB / Depth 초점 거리 | 둘 다 fx=fy=518.1376342773438 |
| RGB / Depth 주점 | 둘 다 cx=321.27423095703125, cy=241.92599487304688 |
| RGB 저대역폭 설정 | `i_low_bandwidth=true`, 프로파일 4, `libx264` |

추가로 45초 동안 영상 내용을 처리하지 않고 헤더를 읽었다. 압축 RGB 194개, 원본 RGB 213개,
Depth 360개를 받았으며 마지막 압축 RGB 촬영 시각은 마지막 Depth보다 **7.570422516초 이전**이었다.
원본 RGB도 마지막 Depth보다 약 7.14초 이전이었다. 원본 RGB에도 차이가 있으므로
PC에서 JPEG를 푸는 부분만 원인이라고 단정할 수 없다. 영상 인코딩·카메라 호스트 처리·전송·큐 누적 중
어느 단계인지까지는 확인하지 않았다. 촬영 시각이 겹치는 과거 메시지를 찾았다는 것과 현재 입력이
동시에 도착한다는 것은 다르므로, 오래된 Depth를 길게 보관해 현재처럼 검사하지 않는다.

같은 크기·프레임 이름이라도 왜곡 모델이 다를 수 있어 동일 픽셀 번호의 Depth를 그대로 사용하지 않는다.
RGB의 방향 벡터를 Depth CameraInfo의 모델로 투영해 대응하는 실제 Depth 샘플을 읽는다.
DepthAI ROS 소스에서는 [stereo 발행 설정](https://github.com/luxonis/depthai-ros/blob/jazzy/depthai_ros_driver/src/dai_nodes/sensors/stereo.cpp)에서
정류된 Depth로 설정하고, [ImagePublisher](https://github.com/luxonis/depthai-ros/blob/jazzy/depthai_ros_driver/src/dai_nodes/sensors/img_pub.cpp)는
정류 영상의 D를 0으로 발행한다. 실제 픽셀 정렬 오차까지 검증한 것은 아니며 카메라 설정은 변경하지 않았다.

모든 기본 토픽은 상대 이름이다. `__ns:=/robot3`이면 `/robot3/` 아래 주소를 구독한다.

Depth는 2026-10-14부터 압축 토픽 `oakd/stereo/image_raw/compressedDepth`(`16UC1; compressedDepth png`)를 기본으로 받는다.
robot4 실시간 15초 비교에서 같은 촬영 시각의 원본과 압축 Depth는 둘 다 값이 있는 픽셀에서 차이가 0mm였다.
압축 쪽은 약 10m를 넘는 값(최소 10.07m, 원본의 65.53m 같은 이상값 포함, 약 1%)을 0으로 바꿨는데,
셔터 검사의 유효 거리(`depth_min_m`~`depth_max_m`, 기본 0.2~5.0m) 밖이라 결과는 같다.
한 장 크기는 약 160KB로 원본(약 600KB)보다 작고, 15초 동안 원본 125장·압축 136장을 받았으며
촬영 간격 최대 공백은 원본 0.40초·압축 0.20초였다. 압축 해제는 한 장에 약 4ms다.
`oakd/rgb/image_raw/compressedDepth`는 컬러 영상이라 Depth 압축이 불가능해 메시지가 나오지 않는다.
이름이 `compressedDepth`로 끝나지 않는 토픽을 주면 기존처럼 원본 `Image`(16UC1·32FC1)로 구독한다.

| 입력 | 기본 상대 주소 / 설정 |
|---|---|
| RGB JPEG | `oakd/rgb/image_raw/compressed` |
| RGB에 정렬된 Depth | `shutter_depth_topic:=oakd/stereo/image_raw/compressedDepth` |
| RGB CameraInfo | `shutter_camera_info_topic:=oakd/rgb/camera_info` |
| Depth CameraInfo | `shutter_depth_camera_info_topic:=oakd/stereo/camera_info` |
| 움직이는 TF | `shutter_tf_topic:=tf` |
| 고정 TF | `shutter_tf_static_topic:=tf_static` |
| 바닥 위쪽 방향의 기준 프레임 | `shutter_ground_frame:=base_link` |
| 정렬 설정을 조회할 카메라 노드 | `shutter_camera_node:=oakd` |

기본 실행에서는 해당 이름공간의 `oakd/get_parameters`로 `stereo.i_align_depth`,
`stereo.i_board_socket_id`, `rgb.i_board_socket_id`를 읽는다. 정렬이 켜져 있고 정렬 대상 번호와
RGB 카메라 번호가 같을 때만 자동 확인한다. 카메라 설정을 변경하지 않는다.
조회 간격은 2초, 응답 대기는 최대 5초며, 마지막 확인이 10초 이상 오래되면 미확인으로 돌아간다.
조회 응답을 기다리며 미리보기나 검사 서비스를 막지 않는다. 계산 중 확인이 바뀐 결과도 재사용하지 않는다.
조회 대기·만료는 `depth_alignment_check_pending`, 정렬 꺼짐은 `depth_alignment_disabled`,
다른 카메라 지정은 `depth_alignment_target_mismatch`, 지원되지 않는 설정은 `depth_alignment_unconfirmed`다.
기존 `shutter_depth_aligned:=true`는 별도로 정렬을 확인한 외부 카메라의 수동 실행 호환용으로 남긴다.
이 값을 명시하면 자동 조회를 생략한다. 기본 실행에는 필요 없다.

Depth와 RGB는 해상도와 광학 좌표계가 같아야 한다. 각각 해당 영상의 CameraInfo가 필요하다.
이 조건만으로 정렬을 증명하지는 않으므로 카메라 정렬 설정 조회도 함께 확인한다.
Depth는 `16UC1` 밀리미터 또는 `32FC1` 미터이며 유효한 촬영 시각이 필요하다.
RGB와 Depth의 시각 차이는 기본 0.03초 이하, 짝을 고를 때 Depth 수신 경과 시간은 기본 0.5초 이하로 제한한다.
셔터 검사 중에는 RGB도 최근 90장(30Hz 기준 3초)까지 보관한다. 2026-10-14 21:29 녹화에서 Depth가 RGB보다 중앙값 0.84초 늦게 도착해 기존 40장(1.3초)으로는 짝이 될 RGB가 먼저 버려졌다. 아직 처리하지 않은 Depth 중 허용 시간 안에 RGB가 있는
**가장 최신 Depth**를 고르고, 그 Depth와 촬영 시각이 가장 가까운 RGB를 붙인다. 차이가 가장 작은 옛 쌍을
고르면 처리 결과가 점점 뒤처지므로 2026-10-14에 최신 짝 우선으로 바꿨다.
Depth는 RGB보다 늦게 도착하므로 오래된 쌍 판단은 최신 RGB가 아닌 최신 Depth 촬영 시각을 기준으로 한다.
선택한 RGB에 해당 Depth 결과를 표시하며
선택한 RGB의 원래 촬영 정보로 영상을 발행한다. 이미 선택한 RGB와 그보다 촬영 시각이 앞선 RGB는 목록에서 지운다.
보관은 처리 FPS 제한 전에 하므로 검사 속도를 줄여도 짝을 고를 후보가 즉시 사라지지 않는다.

Depth 신선도는 **짝을 고를 때만** 확인한다. 한 장의 계산이 0.5초를 넘었다고 결과를 `depth_stale`로 버리지 않는다.
이번 검사에서 짝을 한 번이라도 찾은 뒤에는, 다음 짝을 기다리는 RGB가 와도 화면을 다시 그리지 않고 마지막 결과
(바닥 빨간 표시 포함)를 유지한다. 같은 Depth로 확인 횟수를 늘리지는 않는다.
RGB 수신·Depth 수신·짝 선택 중 하나가 `camera_timeout`(기본 2초) 동안 끊기면 감시 타이머가
`rgb_stale`·`depth_stale`·`rgb_depth_pair_stale`로 결과를 지운다. 검사 시작·종료 때 대기 RGB 목록도 지운다.

2026-10-14 21:14~21:17 실행 로그(셔터 판정 113줄)를 분석한 결과, 이전 방식에서는 `rgb_depth_time_mismatch` 47회,
계산 후 `depth_stale` 17회로 절반 이상의 화면에 바닥 표시가 없었다. 짝이 없는 RGB마다 시각 불일치 화면을 그리고,
계산 시간 동안 0.5초 기준이 만료되어 결과를 지운 것이 바닥 표시 깜빡임의 원인이었다.
같은 로그에서 시각 불일치가 아닌 프레임의 RGB·Depth 시각 차이는 0.003~0.028초로 허용 0.03초 안이었다.

### 발행 주기와 시각 정렬 확인

셔터 검사 중 화면 네 번째 줄과 5초마다의 `shutter stream timing:` 로그에 다음 값을 표시한다.

| 항목 | 의미 |
|---|---|
| `rgb_hz` / `depth_hz` | 촬영 시각 간격의 중앙값으로 구한 실제 발행 주기 |
| `phase_median_ms` / `phase_p90_ms` | 각 Depth와 가장 가까운 RGB의 촬영 시각 차이 |
| `matched_ratio` | 위 차이가 `shutter_sync_tolerance` 안인 Depth 비율. 주기는 같아도 위상이 어긋나면 낮아진다 |
| `depth_extra_lag_s` | Depth가 RGB보다 PC에 늦게 도착하는 시간. 로봇·PC 시계 차이는 서로 빼져 사라진다 |
| `pair_success_ratio` | 새 Depth가 있을 때 짝 찾기에 성공한 비율 |

RGB 주기가 Depth 주기의 정수배이고 `matched_ratio`가 100%에 가까우면 두 영상의 촬영 시각이 정렬된 것이다.
`depth_extra_lag_s`가 0.5초에 가까우면 Depth가 짝 선택 기준을 넘겨 버려지므로 Wi-Fi 대역폭·Depth 크기를 확인한다.

화면에 `rgb_depth_time_mismatch`가 나오면 RANSAC 이전의 촬영 시각 검사에서 중단된 상태다.
화면에는 실제 시각 차이와 허용 시간이, 로그에는 `rgb_depth_delta_s`(Depth 시각 − RGB 시각)가 표시된다.
영상이 계속 들어오는데도 차이가 크게 유지되면 카메라 RGB/Depth 발행 시계와 촬영 시각 설정을 확인한다.
RGB 또는 Depth의 header 시각을 수신 시각으로 덮어쓰지는 않는다.
`shutter_sync_tolerance`는 필요할 때 실행 파라미터로 조정할 수 있지만, 큰 시각 오차를 숨기기 위해
무조건 늘리면 서로 다른 촬영 순간을 연결할 수 있으므로 기본값을 0.03초로 줄였다.
기존 실행 명령에 `-p shutter_sync_tolerance:=0.10`을 넣었다면 제거하거나 `0.03`으로 바꾼다.
45초 추가 헤더 읽기에서도 마지막 RGB와 Depth의 촬영 시각이 7.57초 차이였다.
이 차이를 임의 시각 보정으로 숨기지 않는다. 현 상태에서는 코드 보완 후에도 입력 검사에서 미확인이 나올 수 있다.

### 카메라 지연을 줄이기 위한 권장 변경

카메라 발행 설정 변경은 아직 적용하지 않았다. RGB 저대역폭 인코딩을 끄고 설정 FPS를 Depth와 같은
10으로 줄이며 RGB 큐를 30에서 4로 줄이는 것을 첫 조치로 권장한다. 현재 PC의 JPEG 구독 주소는 유지된다.
이 변경으로 실제 지연이 해결되는지는 사용자가 확인해야 한다. 동작 중인 카메라를 잠시 중지하는 명령이므로
카메라를 사용하는 다른 작업을 정리한 뒤 직접 실행한다.

```bash
ros2 service call /robot4/oakd/stop_camera std_srvs/srv/Trigger "{}"
ros2 param set /robot4/oakd rgb.i_low_bandwidth false
ros2 param set /robot4/oakd rgb.i_fps 10.0
ros2 param set /robot4/oakd rgb.i_max_q_size 4
ros2 service call /robot4/oakd/start_camera std_srvs/srv/Trigger "{}"
```

파라미터 변경 성공과 카메라 시작 성공을 각각 확인한다. 실제 robot4에서 시작·중지 서비스 존재와
`rgb.i_low_bandwidth`의 boolean 형식을 읽어 확인했다.
[카메라 드라이버 소스](https://github.com/luxonis/depthai-ros/blob/jazzy/depthai_ros_driver/src/camera.cpp)에서
중지 시 큐·파이프라인을 닫고 시작 시 구성을 다시 만드는 방식을 확인했다.
임시 변경을 계속 사용하려면 로봇의 원래 카메라 시작 설정에도 같은 값을 반영해야 한다.

원래 값으로 되돌릴 때도 카메라를 중지한 뒤 `rgb.i_low_bandwidth true`, `rgb.i_fps 30.0`,
`rgb.i_max_q_size 30`을 설정하고 다시 시작한다. 이름공간을 robot3으로 바꿀 경우 원래 값을 별도로 확인한다.

기본 RGB 토픽은 `image_raw`이므로 K·D를 사용한다. 실제로 정류된 RGB를 받는 구성에만
`shutter_rgb_rectified:=true`를 사용한다. 현재 정류 경로는 동일 광학 좌표계의 단안 P와 R=단위행렬을 요구한다.
해상도를 임의로 맞추거나 잘라낸 CameraInfo를 그대로 쓰지 않는다.

TF가 `/tf`, `/tf_static`에 전역으로 발행되는 구성에서는 실행 명령에 다음을 추가한다.

```text
-r tf:=/tf -r tf_static:=/tf_static
```

프레임 이름은 ROS namespace로 자동 변경되는 값이 아니다. 실제 TF 프레임이 `robot3/base_link`이면
`-p shutter_ground_frame:=robot3/base_link`를 지정한다. 촬영 시각의 카메라→기준 프레임 TF가 필요하다.
base_link의 Z 방향을 바닥 위쪽으로 사용할 수 없는 장착·주행 조건이면 실제 위쪽을 정의하는 프레임을 선택한다.

## 계산과 판정

1. 같은 빨간 HSV 구간에서 진한 선과 옅은 선을 찾고 넓은 빨간 면은 제거한다. 진한 선의 근거가 있고 화면 범위가 충분히 겹치는 조각은 함께 묶어 볼록 검사 다각형을 구한다. 고정 화면 ROI나 정상 사진을 쓰지 않는다.
2. 이전 바닥의 특징점을 현재 영상으로 추적한다. 왕복 추적 오차 1.5픽셀 이내인 점을 바탕으로 RANSAC 영상 이동을 구하고, 현재 빨간 선과도 충분히 겹칠 때만 구역을 이어 쓴다. RGB·Depth CameraInfo로 같은 방향의 실제 Depth를 읽어 원본 해상도의 3D 점을 구한다.
3. 바깥 경계에서 한 띠 안쪽으로 들어온 유효 Depth를 바닥 근거로 삼는다. 벽·바닥 경계 바로 옆과 내부 사선의 색은 근거에서 제외한다. 근거 점에서 RANSAC 평면을 만들고 구역 전체 점의 일치도도 확인한다. 최종 평면도 근거 점으로 다듬어 장애물 표면이 평면을 끌어올리는 것을 줄인다. 이전 평면은 현재 점·위쪽 방향·분포 검증을 통과할 때만 갱신한다.
4. 각 점의 바닥 높이를 계산하고 화면의 구역 안쪽과 바닥에 투영한 실제 위치가 모두 구역 안인 점을 후보로 삼는다. 테이프 경계도 포함한다.
5. 후보를 미터 단위 3D 격자 이웃으로 묶고 높이·점 수·바닥 투영 면적 기준을 만족하면 장애물로 판독한다.
6. 같은 구역의 서로 다른 Depth 사진이 기본 0.5초와 최소 3프레임 조건을 충족해야(사진 사이 공백 1초 이하) 정상 또는 경고를 확정한다. 경고는 장애물 후보 위치도 연속으로 겹쳐야 한다.

평면 추정에만 점 수를 줄인다. 장애물 판독은 원본 해상도의 점군을 사용한다.
바닥으로 지운 픽셀 수와 바닥 격자 관측 비율도 검사 구역 안에서만 계산한다.
`floor_fit_point_count`는 평면 후보 검증에 넣은 구역 전체의 유효 점 수, `floor_anchor_point_count`는
경계에서 안쪽으로 들어온 근거 점 수다. `zone_pixel_count`는 검사 구역 픽셀 수다.
RANSAC은 NumPy로 구현했으며 Open3D 설치를 요구하지 않는다.

부분 구역에서도 장애물이 있으면 경고를 확정할 수 있다. 정상은 빨간 구역이 화면 안에서 완전히 보이고,
유효 Depth 비율·바닥 격자 관측 비율·바닥 오차 조건까지 만족해야 한다.
화면에서 잘린 구역은 꼭짓점이 4개가 아닐 수 있으므로 3개 이상인 볼록 다각형도 부분 구역으로 검사한다.
가까운 선 조각을 묶을 때만 확장한 마스크를 사용하며, 구역 위치와 바닥 후보는 원래 빨간 픽셀로 구한다.
물리적으로 연결이 끊긴 조각도 둘러싼 사각형의 겹침이 큰 쪽 면적 대비 기본 20% 이상이면 같은 묶음으로 본다.
옅은 선만 있는 묶음은 독립 구역으로 선택하지 않는다. 확정한 경계 안의 작은 선 조각도 포함하되 구역을 더 넓히지 않는다.
완전한 구역은 화면 안의 네 변에 실제 빨간 선이 충분히 보이는 경우에만 인정한다.
첨부 사진처럼 구역이 화면 밖까지 이어지는 자세에서는 장애물이 안 보여도 정상 대신 미확인을 표시한다.
여러 시점의 관측을 지도에 합치거나 로봇을 움직이는 기능은 포함하지 않는다.
화면 구역 밖으로 솟은 부분만 보이는 물체는 후보에서 제외될 수 있다. 이 방식은 구역 안에 보이는
물체 표면의 유효 Depth가 필요하며, 관측되지 않은 영역까지 장애물 없음을 확정하지 않는다.

## 초기 감도와 조정

다음은 실촬영 보정 전 초기값이다. 낮은 물체를 검출하려면 감도와 실제 센서 오차가 양립해야 한다.
파라미터는 실행 시 지정하며 변경 후 노드를 재시작한다.

| 파라미터 | 초기값 | 의미 |
|---|---|---|
| `shutter_red_support_min_s` | 35 | 진한 선과 함께 확인할 옅은 빨간 선의 최소 채도 |
| `shutter_min_red_fragment_pixel_ratio` | 0.0001 | 선 조각으로 보존할 최소 픽셀 수의 영상 면적 대비 비율 |
| `shutter_zone_fragment_overlap_ratio` | 0.20 | 두 조각 중 큰 사각형 면적 대비 최소 겹침 |
| `shutter_zone_fragment_gap_ratio` | 0.02 | 큰 묶음끼리 합칠 때 허용하는 사각형 간격(짧은 변 비율). 이음매로 끊긴 좌우 판을 한 구역으로 합친다 |
| `shutter_zone_fragment_join_min_area_ratio` | 0.01 | 2단계 합치기에 참여하는 묶음의 최소 볼록 껍질 면적(화면 비율) |
| `shutter_obstacle_boundary_inset_m` | 0.05 | 장애물 후보에서 제외하는 구역 경계 안쪽 띠 폭(m). 벽 밑단 픽셀의 가짜 장애물을 막는다 |
| `shutter_confirm_zone_overlap_ratio` | 0.30 | 이전 구역과 같은 구역으로 보는 최소 겹침(IoU). 확인 이력 유지 기준 |
| `shutter_obstacle_boundary_inset_ratio` | 0.02 | 장애물 후보에서 제외하는 구역 테두리 안쪽 띠(화면 짧은 변 비율, 픽셀). 먼 경계의 벽 밑단 줄을 막는다 |
| `shutter_red_join_ratio` | 0.015 | 가까운 빨간 선 조각을 묶는 범위의 영상 짧은 변 대비 비율 |
| `shutter_min_boundary_support_ratio` | 0.70 | 완전한 구역으로 인정할 각 변의 빨간 선 확인 비율 |
| `shutter_max_tape_width_ratio` | 0.08 | 넓은 빨간 면을 제거하는 커널 크기의 영상 짧은 변 대비 비율 (480px에서 39px) |
| `shutter_geometry_max_gap_seconds` | 1.0 s | 구역·평면 추적을 이어 쓸 최대 관측 공백 |
| `shutter_tracking_min_points` | 12 | 영상 이동 추적에 필요한 최소 유효 특징점 수 |
| `shutter_tracking_min_inlier_ratio` | 0.65 | 영상 이동 RANSAC의 최소 일치 비율 |
| `shutter_tracking_red_support_ratio` | 0.50 | 옮긴 이전 빨간 선과 현재 빨간 선의 최소 겹침 비율 |
| `shutter_plane_update_ratio` | 0.20 | 검증된 이전 평면을 갱신할 때 새 평면의 비중 |
| `shutter_plane_distance_m` | 0.008 m | RANSAC 바닥 점 허용 거리 |
| `shutter_floor_noise_multiplier` | 3.0 | 바닥 분류 거리에 반영할 측정 잡음의 배수 |
| `shutter_floor_max_height_ratio` | 0.75 | 바닥 분류 거리 상한의 최소 장애물 높이 대비 비율 |
| `shutter_min_obstacle_height_m` | 0.07 m | 목표 최소 장애물 높이 (요구사항 10cm 이상, 2026-10-14 변경) |
| `shutter_min_obstacle_area_m2` | 0.0004 m² | 목표 최소 바닥 투영 면적 |
| `shutter_cluster_voxel_m` | 0.03 m | 장애물 이웃 묶음의 3D 격자 크기 |
| `shutter_min_cluster_points` | 20 | 잡음 묶음을 제외할 최소 점 수 |
| `shutter_min_valid_depth_ratio` | 0.80 | 구역 안의 유효 Depth 비율 |
| `shutter_min_coverage_ratio` | 0.98 | 바닥의 검사 격자 관측 비율 |
| `shutter_confirm_frames` | 3 | 최소 서로 다른 Depth 프레임 수 |
| `shutter_confirm_seconds` | 0.5 s | 최소 연속 관측 시간 |
| `shutter_confirm_max_gap_seconds` | 1.0 s | 확인 이력을 유지할 최대 관측 공백 |

장애물 높이 기준은 설정한 목표 높이(기본 70mm)를 유지한다. 바닥 잡음 때문에 자동으로 올리지 않는다.
바닥 분류 거리는 `평면 허용 거리 + floor_noise_multiplier×바닥 잡음`으로 조정하고,
`max(평면 허용 거리, 목표 최소 장애물 높이×floor_max_height_ratio)`를 넘지 않게 한다.
초기 설정에서는 바닥 분류 거리 상한이 52.5mm로, 목표 70mm 장애물을 바닥으로 칠하는 방향으로 늘리지 않는다.
평면 후보를 찾고 검증하는 거리도 `max(plane_distance_m, 목표 높이×floor_max_height_ratio)`다.
초기값에서는 52.5mm다. 근거 점으로만 평면을 다듬고, 바닥 표시 거리는 측정 잡음으로 별도 계산한다.
후보를 찾는 허용 거리가 커졌다는 이유로 구역 전체나 무효 Depth를 칠하지 않는다.
`평면 허용 거리 + floor_noise_multiplier×바닥 잡음`이 목표 높이 이상이면 장애물 후보 검출은 유지하되
장애물이 없다는 정상 판정은 보류한다. 무효 Depth를 임의 보간하지 않는다.
이 코드가 임의로 얇은 모든 물체를 검출한다고 보장하지 않는다.

### 2026-10-14 녹화 분석과 기본 높이 변경

`~/final_ws/bags/shutter_1014_2129`(robot4, 28.9초, Depth 187장)를 같은 함수로 재생했다. 장애물이 없는 판지 바닥이다.

| 확인 항목 | 결과 |
|---|---|
| 발행 주기 / 위상차 | RGB 30.0Hz, Depth 10.0Hz / 중앙 7.0ms, 허용 30ms 안 100% |
| Depth 전송 | RGB보다 중앙 0.84초 늦게 도착, 10Hz 중 초당 약 6.5장 수신 |
| Depth 단계 | 3mm (시차 해상도 부족은 아님) |
| 바닥 잡음 | 10~12mm. 5장 시간 평균·중앙값 필터 후에도 10.5mm로, 대부분 고정 오차 |
| 고정 오차 형태 | 가로 줄무늬, 가운데 테이프 이음매 기준 좌우 5~17mm 높이 차이 |
| 기존 20mm 기준 | 98% 프레임에 가짜 장애물(최대 48mm), 171장이 이상으로 확정 |
| 50mm 기준 | 9% 프레임에 가짜 장애물(최대 57mm) |
| 70mm 기준 | 가짜 장애물 0장, 바닥 표시 약 92% |

요구사항(BR-T14)이 10cm 이상 물체이므로 기본값을 70mm로 올렸다. 실제 10cm 물체의 검출률은 아직 녹화로 확인하지 않았다.
이 녹화에서는 구역 오른쪽·아래 테이프가 화면 밖이라 모든 프레임이 `zone_partial`(미확인)이다.
정상 판정을 받으려면 네 변의 빨간 테이프가 모두 보이는 위치에서 검사해야 한다.

### 가까운 테이프가 구역에서 빠지던 문제 (2026-10-14)

robot4 실시간 영상에서 왼쪽 긴 테두리 테이프는 색 조건(H 177·0, S 126~146)을 통과했지만,
카메라 가까이에서 화면 가로 폭이 12~26px로 굵어져 기존 넓은 면 제거 커널(0.035, 17px)에 지워졌다.
그 결과 구역이 오른쪽 사선 테이프 묶음만으로 만들어졌다. 기본값을 0.08(39px)로 올리자 왼쪽 테두리부터
오른쪽 화면 끝까지 하나의 구역으로 잡혔다. 녹화 185장에서 구역 면적 중앙값은 61,544px에서 82,589px로 늘었고
가짜 장애물은 0장으로 유지됐다. 39px보다 굵은 빨간 면은 계속 제거되며, 빨간 상자가 놓인 장면은 아직 시험하지 않았다.

로그에는 `reason`, `coverage`, `floor_noise_m`, `height_threshold_m`, `zone_tracking`,
`confirm_frames`, `confirm_elapsed_s`, `obstacle_heights_m`, `floor_diagnostics`, `ground_up_camera`가 나온다.
평면을 못 찾으면 화면의 `reason`은 `floor_diagnostics.fit_reason`으로 상세 단계를 표시한다.
실제 후보 기울기/허용 기울기, 구역·근거 일치 비율/필요 비율, 근거 점 수와 탐색 거리도 표시한다.

| 평면 실패 단계 | 의미 |
|---|---|
| `floor_points_insufficient` | 구역 안 유효 점이 최소 개수보다 적음 |
| `floor_anchors_insufficient` | 경계 안쪽 근거 점 또는 최종 일치 근거가 최소 개수보다 적음 |
| `floor_plane_tilt_exceeded` | 후보가 TF 위쪽 방향과 허용 기울기 이상 다름 |
| `floor_plane_inliers_insufficient` | 구역 점이 후보 평면과 충분히 일치하지 않음 |
| `floor_anchor_inliers_insufficient` | 경계 안쪽 근거가 후보 평면과 충분히 일치하지 않음 |
| `floor_anchor_span_insufficient` | 근거가 한 줄 또는 작은 영역에 몰림 |
| `floor_samples_degenerate` | 샘플 점으로 유효한 평면을 만들지 못함 |
| `floor_sample_support_insufficient` | 전체 점 검증과 샘플 검증 결과가 달라 후보를 보류함 |

일치 비율이 높아도 기울기·근거 분포 검증은 유지한다. 기울기 실패를 숨기려고 허용 각도를 늘리지 않는다.
`floor_diagnostics`의 `anchor_residual_p90_m`, `span_m`과 로그의 `ground_up_camera`로 근거 오차와 방향을 확인한다.
장애물 확정은 기존 `inspect/result`에 `KIND_SHUTTER`, `ABNORMAL`로 발행하며 정상·미확인은 화면과 로그에 표시한다.

한글은 Pillow와 한글 글꼴로 그린다. 현재 PC에서는 Noto Sans CJK 글꼴 경로를 자동 선택한다.
다른 PC에서는 `shutter_font_path`로 글꼴 경로를 지정할 수 있다. 글꼴이 없으면 셔터 서비스만 비활성화하고 이유를 로그로 알린다.


## RGB·Depth 화면 어긋남 보정 (2026-10-15 robot4 측정)

- 증상: 상자를 세워 놓아도 장애물이 전혀 잡히지 않았다. Depth 영상에는 상자가 또렷한데 RGB 보다 오른쪽 위로
  치우쳐 작게 보여, RGB 기준 구역 안의 Depth 픽셀이 모두 바닥 높이로 계산됐다.
- 원인: `stereo.i_align_depth=true` 이고 두 CameraInfo 의 K 도 같지만(518.1 / 321.3, 241.9), 실제 Depth 는
  RGB(1080P → ISP 2/3 → 640x480 중앙 잘라냄)보다 넓은 시야를 640x480 으로 줄인 영상이다. 카메라 설정은 건드리지
  않았으므로(읽기만 함) 소프트웨어에서 보정한다.
- 측정: 상자 두 개를 두고 `tools/measure_depth_alignment.py` 로 Depth 의 바닥 위 덩어리가 RGB 상자 사각형과 가장
  잘 겹치는 축별 배율·이동을 찾았다. 결과 `depth_x = 0.775*rgb_x + 79`, `depth_y = 0.670*rgb_y + 85`
  (정밀도 0.96).
- 적용: 매니저 파라미터 `shutter_depth_pixel_scale_x/offset_x/scale_y/offset_y` 를
  `shutter_obstacle.scale_depth_calibration()` 으로 Depth K·P 에 반영한 뒤 `depth_for_rgb()` 가 다시 샘플링한다.
  보정 없이 쓰려면 1,0,1,0 을 준다. 카메라 해상도·정렬 설정이 바뀌면 반드시 다시 측정한다.
- **카메라 설정 변경 후(2026-10-15 10:40 무렵, 사용자가 직접 변경)**: RGB·Depth 가 704x704, 둘 다 10Hz,
  K = 569.95 / 353.4, 354.1 로 바뀌었다. 녹화 `~/final_ws/bags/shutter_1015_1043`(28.4초, 앞 10초 빈 구역·뒤 상자)에서
  - 시각: 위상차 0.08ms, Depth 누락 0회, Depth 추가 도착 지연 0.07초.
  - 화면: RGB 색 경계와 Depth 거리 경계 겹침이 최대가 되는 배율이 약 0.98(이전 설정은 약 0.667)이었다.
    남은 약 2%는 RGB 렌즈 왜곡이며 `depth_for_rgb()` 의 왜곡 보정만으로 겹침이 40.8%→49.2%(최적 배율 맞춤 49.9%)로 올랐다.
  - 그래서 **기본값을 1, 0, 1, 0(보정 없음)으로 되돌렸다.** 이전 기본값 0.775/79/0.670/85 를 새 설정에 쓰면
    빈 구역·상자 26장 모두 `floor_not_confirmed` 로 상자를 하나도 잡지 못했다.
- 주의: 매니저의 "Shutter alignment=verified" 로그는 카메라 파라미터만 읽은 결과이며 실제 화면 정렬을 보장하지 않는다.


## 바닥 평면 고정값 (RANSAC 연산량 절감, 2026-10-15)

- 카메라가 로봇에 고정돼 바닥 평면이 거의 변하지 않으므로 매니저 파라미터 `shutter_floor_plane_mode:=fixed`
  에서는 `shutter_fixed_floor_plane`(카메라 광학 좌표계 `[nx, ny, nz, d]`, 높이 = p·n + d) 을
  `inspect(fixed_plane=...)` 로 넘겨 RANSAC(250회 반복)을 건너뛴다.
- **기본 모드는 `ransac`** 이다(2026-10-15 사용자 결정). 고정값 기능은 그대로 남아 있다.
- 고정값 기본은 새 카메라 설정 녹화 `shutter_1015_1043` 의 RANSAC 48장 중앙값 `[-0.0017, -0.9994, 0.0334, 0.4237]`
  (법선 흔들림 최대 0.63°)이다. 이 값으로 fixed 모드를 쓰면 빈 구역 가짜 장애물 0장, 상자 13장 모두 검출됐다.
  이전 설정에서 잰 `[0.0040, -0.9996, 0.0292, 0.4610]` 은 d 가 3.7cm 달라 빈 구역 13장 중 6장에서 가짜 장애물이 나왔다.
- 고정 평면은 매 프레임 `_plane_quality()` 로 기울기·일치 비율·근거 분포만 검증한다(가벼운 벡터 연산). 로봇이
  기울거나 바닥이 다르면 `floor_not_confirmed` + `plane_source=fixed_plane_rejected` 로 미확인이 된다.
- 다시 재려면 `-p shutter_floor_plane_mode:=ransac` 으로 실행하고 로그 `shutter verdict=... plane=[...]` 값을
  파라미터에 옮겨 적는다. 로봇·카메라 장착이 바뀌면 반드시 다시 잰다.


## 처리 속도 개선 (2026-10-15)

704x704 로 바뀐 뒤 이 PC 에서 한 장 검사(`depth_for_rgb` + `inspect` + `stabilize_report`)가 중앙 460ms 였다.
녹화 `shutter_1015_1043` 284장으로 측정했고, 아래 변경 뒤 **중앙 192ms(최대 413ms)** 이며 284장 모두 판정·사유·
구역·장애물 상자·높이·잡음·관측 비율이 변경 전과 같았다.

| 변경 | 변경 전 → 후 (한 장) |
|---|---|
| `_vertically_consistent`: `np.nanmedian` 대신 모두 유효한 창은 `partition`, 빈 값이 섞인 창만 정렬로 같은 중앙값 계산. 결과가 쓰이는 바닥보다 높은 픽셀(`where=tall`)만 검사 | 약 225ms → 수십 ms |
| `_red_masks`(넓은 면 제거 열기 연산) 를 `inspect` 에서 한 번만 만들어 `_find_zone`·`_tracked_zone` 에 전달 | 최대 3회 → 1회(1회 약 25ms) |
| `_fit_floor`: 무작위 세 점은 같은 순서로 뽑고, 250개 후보 평면의 일치 점 수를 행렬 한 번으로 계산 | 반복문 → 행렬 계산 |
| 바닥 좌표 변환·다각형 안 판정을 화면 전체 유효 픽셀 대신 구역 안 픽셀만 계산 | 약 47만 점 → 구역 안 점 |

남은 큰 비용은 넓은 면 제거 열기 연산(약 25ms)이며, 결과를 바꾸지 않고는 줄이기 어려워 그대로 두었다.


## 장애물 표시 범위 (2026-10-15)

- 판정용 `bbox` 는 구역(빨간 테두리) 안의 후보 픽셀만 감싸므로 세워 둔 상자는 윗부분이 잘려 보였다.
- `inspect()` 가 구역 안 장애물 픽셀을 씨앗으로, 기준 높이 이상이면서 씨앗 거리 중앙값 ±
  `shutter_obstacle_display_depth_tolerance_m`(기본 0.25 m) 안의 픽셀을 화면 전체에서 8-연결로 이어 붙여
  `display_bbox` 를 만든다. `draw_result()` 는 이것으로 그린다. 벽처럼 더 먼 면은 거리 차이로 제외된다.
- 판정·확인(겹침 비교)은 그대로 `bbox` 를 쓴다. 0 을 주면 표시도 구역 안 범위로 돌아간다.


## 상자 밑단의 가짜 장애물 (2026-10-15)

- 증상: 세운 상자 하나에 장애물이 두 개 잡혔다. 아래쪽 것은 상자 밑단 바로 아래 2~3줄(RGB 기준 9픽셀) 띠였다.
- 원인: 그 띠의 Depth 가 상자 면(1.84 m)·바닥(1.8 m)보다 0.5 m 가까운 1.3 m 로 나왔다. 깊이가 갑자기 바뀌는
  가장자리에서 스테레오 매칭이 만드는 잘못된 깊이(날아다니는 픽셀)다. 평면보다 가까우니 높이 15 cm 로 계산됐다.
- 대책: `_vertically_consistent()` 로 같은 열 위아래 21줄 중앙값과 `shutter_flying_pixel_depth_m`(기본 0.10 m)
  이상 다른 픽셀은 장애물 후보에서 뺀다. 실제 면은 여러 줄이 같은 깊이라 남는다. 0 이면 끈다.


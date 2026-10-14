# PLAN 12·13번 — 측정 버퍼·DB 연결 작업 요약과 PC별 시험 방법

작성: 2026-10-14 · 전희재(HJAE99) · 대상: `feature/control`의 ROS 측정 흐름

## 1. 한 일과 확인 범위

12번의 구현과 로봇 없는 검증은 완료했다. **실제 로봇 PC·관제 PC 사이 Wi-Fi 30초 차단 시험은 아직 하지 않았다.** 이 문서의 실기 절차는 알맞은 PC에 수정 코드를 반영한 뒤 수행한다.

- 기존 ManualTagger의 측정 시각 TF 위치 결합·공분산 검사를 재사용한 `survey_buffer` 추가.
- `RobotStatus.run_id` 연결. 상태 전에는 0, TF를 기다리는 동안 회차가 바뀌어도 원본을 받은 당시 회차 유지.
- 위치 불확실·미검출을 포함한 원본을 CSV에 flush·fsync **후** NetSample 발행.
- seq는 회차가 바뀌어도 계속 증가하고, 재시작 시 CSV 마지막 번호 다음부터 복원.
- 재시작 시 rssi_scanner의 과거 NetRaw 캐시를 새 번호로 중복 저장하지 않음.
- IF-09 `survey/sync`: 회차·from_seq 범위만, 원본 번호·시각·위치를 유지해 `resent=true`, 최대 20 Hz 재발행.
- tf launch만 버퍼로 교체. 노트북에서 위치를 찍는 manual 모드는 기존 태거 유지.
- CSV 손상·동일 파일의 두 작성자·쓰기 실패는 오류로 중단하며 파일 삭제나 번호 초기화로 숨기지 않음.

구현 직후 결과: 관제·RSSI 시험 **113 passed**, `scv_msgs`·`scv_survey`·`scv_bringup` 빌드 성공. 가짜 TF·RSSI·상태를 쓰는 별도 ROS domain에서 노드 재시작·sync·히트맵 중복 제거·launch 분기를 확인했다. 실제 motion/action은 호출하지 않았다.

## 2. 현재 연결

```text
robot_pc.launch.py
  └ laptop_test.launch.py (pose_source=tf)
       ├ rssi_scanner → survey/raw
       ├ survey_buffer → CSV 선저장 → survey/sample
       └ network_map_engine → RViz 히트맵·기존 CSV

관제 PC: survey/sample → cell_map_writer → DB 원본(1초 묶음) + 칸별 결과
                             └ survey/sync → 로봇 CSV 누락 복구 (db 모드)
```

`laptop_test.launch.py`를 직접 실행하지 않아도 통합 launch가 내부적으로 불러온다. 웹 `client_page.py`의 SurveyBuffer/Outbox는 별개이며 이번에 변경하지 않았다.

| 파일 | 변경 |
| --- | --- |
| `ros2_ws/src/scv_survey/scv_survey/survey_buffer.py` | 정식 TF 버퍼 노드·회차·sync |
| `ros2_ws/src/scv_survey/scv_survey/sample_buffer.py` | CSV 복원·잠금·번호·재발행 큐 |
| `manual_tagger.py` | 상속에 필요한 생성자 인자만 추가 |
| `setup.py`, `launch/laptop_test.launch.py` | 실행 항목 등록·tf 모드 교체 |
| `test_cellmap_store.py`, `test_survey_buffer.py` | 저장·복원·범위·실패·ROS 연동 시험 |
| `docs/interfaces.md`, PLAN, RSSI README·CODE_WALKTHROUGH | 규칙·진행·한계 기록 |

## 3. PC별 준비

- **로봇 PC:** 새 buffer 코드와 scv_msgs를 빌드. rssi_scanner·TF·AMCL·상태를 공급하는 기존 통합 환경 필요.
- **관제 PC:** 같은 scv_msgs와 작업공간 설정을 사용. 샘플 구독과 sync 요청을 수행.
- 로봇이 움직일 필요는 없다. 기존 통합이 실행 중이면 먼저 상태를 확인한다. `robot_pc.launch.py` 전체를 새로 실행하면 mission_executor가 자동 undock할 수 있으므로, 버퍼만 확인하려고 전체 통합을 재기동하지 않는다.
- 로봇마다 태거/버퍼는 하나만 실행한다. 같은 CSV에 두 버퍼를 띄우면 잠금 오류가 난다.

각 PC에서 수정 코드 반영 후 빌드:

```bash
cd /home/hj/B_0102_Final_Project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scv_msgs scv_survey scv_manager scv_bringup --symlink-install
source install/setup.bash
```

다른 PC에서는 실제 저장소 경로로 바꾼다. 새 터미널마다 `/opt/ros/jazzy/setup.bash`와 해당 작업공간의 `install/setup.bash`를 불러온다.

## 4. 로봇 없는 자동 시험

개발 PC에서:

```bash
cd /home/hj/B_0102_Final_Project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONPATH="$PWD/src/scv_survey:$PWD/src/scv_manager:$PYTHONPATH" \
/usr/bin/python3 -m pytest -q src/scv_survey/test src/scv_manager/test -p no:cacheprovider
```

12번 구현 직후에는 113 passed였다. 재검토 후 13번 PostgreSQL 시험까지 준비하면 122 passed이며, DB/웹 의존성이 없으면 해당 시험은 skipped이다. 테스트는 실제 로봇을 제어하지 않는다.

## 5. 버퍼 실행·저장 확인 (robot3 예시)

```bash
ros2 node list | rg 'survey_buffer|manual_tagger'
ros2 service list | rg '/robot3/survey/sync'
ros2 topic echo /robot3/status
```

tf 통합에서는 `/robot3/survey_buffer`가 보여야 한다. 기존 프로세스가 manual_tagger이면 새 코드로 RSSI 부분을 교체해야 한다.

기존 rssi_scanner·TF·AMCL·status는 살아 있고 태거만 교체하는 경우, 기존 태거를 중단한 뒤 버퍼 단독 실행 가능:

```bash
ros2 run scv_survey survey_buffer --ros-args \
  -r __ns:=/robot3 -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

이 명령은 주행 노드를 실행하지 않는다. 통합 launch가 버퍼를 이미 띄웠다면 실행하지 않는다.

로봇 PC에서:

```bash
ros2 topic echo /robot3/survey/sample
tail -n 5 ~/scv_survey_logs/robot3_buffer.csv
```

확인: status와 샘플의 run_id 일치, 새 측정마다 seq 증가, 일반 샘플 resent=false, 같은 번호가 CSV에 기록. TF 위치 품질이 부족하면 pose_ok=false여도 버퍼 기록은 남는다. status 이전 run_id 0 기록은 정식 작업 회차와 구분한다.

`robot3_buffer.csv`는 선저장·재전송 원본이다. `robot3_tf.csv`는 엔진이 히트맵에 사용한 기록이므로 두 파일의 전체 행 수가 반드시 같지는 않다.

## 6. 수동 sync 확인 (관제 PC)

샘플에서 실제 회차 N과 다시 받을 첫 번호 K를 확인한다. 아래 3과 10은 예시이므로 실제 값으로 바꾼다.

```bash
ros2 topic echo /robot3/survey/sample
```

다른 관제 터미널에서:

```bash
ros2 service call /robot3/survey/sync scv_msgs/srv/SyncSamples \
"{run_id: 3, from_seq: 10}"
```

성공 기준:

- 저장된 회차라면 ok=true, last_seq는 그 회차 마지막 저장 번호.
- count는 요청 범위에 해당하는 원본 수다. 이미 재발행 큐에 있는 대상도 포함하며 중복 예약은 막는다. 수신·DB 저장 ACK는 아니다.
- 회차 3의 seq 10 이상만 resent=true로 재발행. 원래 시각·위치·값 유지.
- 원본 CSV에 재전송 행이 추가되지 않고, 살아 있는 히트맵에는 같은 번호가 중복 집계되지 않음.
- 없는 회차는 ok=false. 마지막 번호 뒤를 요청하면 ok=true, count=0.

## 7. 실제 통신 차단·재시작 시험

1. 관제 PC에서 샘플 수신 기록을 시작하고 마지막 회차·seq를 남긴다.
2. **관제 PC의 ROS 연결만 30초 차단.** RSSI를 재는 로봇 PC의 Wi-Fi와 로컬 측정 환경은 유지한다.
3. 로봇 PC의 buffer CSV에 기록이 계속 쌓이는지 확인한다.
4. 관제 연결 복구 후, 같은 회차의 마지막 수신 번호 다음부터 수동 sync 요청.
5. 서비스 응답의 last_seq를 비교 종료 번호로 잡고, 그 번호까지의 CSV와 수신 고유 키 집합 `(robot_id, run_id, seq)` 비교. 측정이 계속 진행되므로 비교 종료 번호 뒤의 새 샘플은 제외한다.
6. 원본+재전송에서 메시지가 반복되는 것은 정상이다. **중복 제거 후** 누락 0·중복 0이어야 한다. 12번만으로 DB 저장까지 확인한 것은 아니며, 13번 db 모드에서는 자동 sync·DB 비교까지 아래 절차로 확인한다.
7. 별도로 버퍼만 재시작해 CSV 다음 번호부터 이어지는지 확인. 전체 통합/주행 launch를 종료·재실행하지 않고, 버퍼 실행 형태에 맞게 수행한다.

| 확인 항목 | 실제 기록 |
| --- | --- |
| 로봇/관제 PC·코드 버전 | |
| 회차·차단 전 마지막 seq·sync from_seq·응답 last_seq | |
| 비교 범위의 CSV 원본 수·수신 고유 키 수 | |
| 누락 키 수·중복 제거 후 중복 수 | |
| 재시작 전/후 seq | |
| RViz 유지·오류 로그 | |

## 8. 13번 구현 요약과 검증 범위

| 변경 | 동작 |
| --- | --- |
| `cellmap_store.py` | 기준 지도·DB 격자 일치 검사, write_samples·last_seq, 연결·질의 시간 제한 |
| `cell_map_writer.py`, `sample_sync.py` | 원본 1초 묶음 저장, 주기적 sync·빈틈·재연결 복구, DB 실패 큐 유지·경보, 저장/복구 전 commit 보류 |
| `inspection_manager.py` | commit 실패·응답 대기 시 다음 단계로 넘어가지 않고 재시도 |
| `control_pc.launch.py` | writer에도 사용 로봇·구역 배정 전달 |
| `01_schema.sql` | detected·resent·stamp_sec·stamp_nanosec 추가. 기존 웹 표본은 detected=true로 호환 |
| `server_page.py` | 선택형 ROS 회차 소유 모드, 외부 ROS 저장값으로 웹 히트맵 재집계, 미검출 0 dBm 집계 제외 |
| `test_db_store.py`, 관제 시험 | 별도 PostgreSQL·가짜 ROS 연동·실패 후 복구·웹 격자 검증 |

재검토 후 검증: 관제·RSSI 시험 **122 passed**, 4개 패키지 빌드 성공. 별도 PostgreSQL 16에서 회차 done/partial, 파일/DB 칸 통계 일치·재측정 갱신, 원본 중복 제거·미검출·위치 불확실·원래 nanosecond 시각을 확인했다. 가짜 ROS에서 선두/중간 누락 자동 복구·psycopg 실제 접속 실패 후 재시도·commit 실패/대기·writer 재시작 후 원본 중복 0을 확인했다. 웹 generate_cells와 ROS free_cells로 만든 실제 기준 지도 격자는 모두 **98칸**이었다.

**미검증:** 실 관제 PC의 DB·웹 지도 업로드, robot3 전체 순찰, 실제 Wi-Fi 30초 차단. 이 문서의 검증은 로봇 주행 성공을 뜻하지 않는다. 공유 대상 브랜치는 `feature/Integration_db`이며, PLAN 12·13번 작업을 한글 커밋으로 묶어 공유한다. 작업 기준 HEAD는 a4e6edd이며, 확인 시 원격의 새 커밋 3개는 로컬에 미반영 상태였다. 현장 반영 전에 동료 변경과 합칠 필요가 있다.

### 복구와 저장 규칙

- 회차 첫 seq는 0일 필요가 없다. 최초 수신/관제 재시작/누락은 from_seq=0 전체 범위를 요청하고, 응답 원본 수와 수신 고유 번호를 비교한다. MAX(seq)만 보면 빠진 선두/중간 번호를 놓치기 때문이다.
- 완전 수신이 확인되면 마지막 스냅샷 번호+1부터 요청한다. 기본 요청 주기는 3초, 재발행은 최대 20 Hz다.
- 원본 DB PK는 `(robot_id, run_id, seq)`. 재전송이 반복돼도 DB 원본은 한 건이다. task_id는 NULL로 두며 보고서용 작업 표 연결은 DB-7 이후다.
- run_id=0은 로봇 CSV에는 남고 작업 DB 원본에는 넣지 않는다. CSV/DB 비교는 동일 로봇·동일 **0이 아닌 회차**를 기준으로 한다.
- pose_ok=false도 DB 원본으로 보존하며 cell_id는 NULL. detected=false의 0 dBm은 유효 신호 평균에 넣지 않는다.
- 과거 회차 복구는 해당 회차 원본으로 저장하고 현재 칸 통계에는 섞지 않는다. 없는 회차/다른 지도 회차를 현재 회차로 바꾸지 않는다.
- DB 장애 시 최대 100,000건의 관제 메모리 큐를 유지한다. 한도를 넘기거나 관제가 재시작하면 로봇 CSV에서 다시 받는다. 관제 큐 자체는 디스크 저장소가 아니다.
- commit은 원본 sync와 DB 저장 완료 뒤 성공한다. 기본 8초 안에 끝나지 않으면 실패를 반환하고 관제가 재시도한다. 긴 replay 동안 로봇 버퍼가 계속 살아 있어야 한다.

## 9. 관제 PC DB 준비

운영 DB 대신 **시험 DB**로 먼저 확인한다. 기존 DB를 초기화하는 `--reset`은 쓰지 않는다.

```bash
cd /home/hj/B_0102_Final_Project/ros2_ws
bash scv_db/docker/setup_db.sh --test --verify
```

새 스키마가 적용돼야 한다. 이전 DB에는 원본 플래그/ROS 시각 열이 없으므로 코드만 바꾸면 연결 준비 오류가 난다. psycopg는 ROS를 실행하는 Python 환경에 설치돼 있어야 한다. SCV_DB_DSN은 해당 PC의 시험 DB 접속 정보로 설정한다. 실제 비밀번호는 이 문서나 Git에 기록하지 않는다.

```bash
python3 - <<'CHECK'
import os, psycopg
with psycopg.connect(os.environ['SCV_DB_DSN']) as conn:
    print(conn.execute('SELECT current_database()').fetchone()[0])
    print(conn.execute('SELECT name, grid_size_m, origin_x, origin_y FROM maps WHERE is_active').fetchall())
CHECK
```

확인한 DB가 `robotdb4_test`인지 확인한다. 지도/AP가 없는 새 시험 DB는 웹에서 아래 항목을 등록한다.

1. 기준 `scv_bringup/maps/testbed_v1.yaml`·PGM을 업로드하며 **grid_size_m=0.5** 지정.
2. 해당 지도를 활성화하고 격자 생성. 기준 지도에서는 98칸인지 확인.
3. 측정할 핫스팟 SSID 등록. writer의 ap_ssid와 rssi_scanner의 SSID가 같아야 한다.
4. 다른 지도/1.0m 격자면 writer/manager가 준비 실패를 알리는지 확인. 운영 지도 삭제나 기존 회차 변경으로 맞추지 않는다.

로봇을 움직이지 않고 DB 수신 노드만 확인하는 예:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run scv_survey cell_map_writer --ros-args \
  -p backend:=db -p ap_ssid:='시험 SSID' \
  -p 'robot_ids:=[robot3]' -p 'zone_ids:=[Z2]' \
  -p 'sample_topics:=[/robot3/survey/sample]'
```

이미 통합 launch에서 writer가 실행 중이면 두 번째 writer를 띄우지 않는다. 통합 설정은 `control_pc.launch.py backend:=db ap_ssid:=... robots:=robot3`이며, 전체 통합 실행과 순찰 시작은 현장 로봇 운용 절차를 따른다. 위 수신 노드만으로 새 run을 만들지는 않는다. run 0 표본은 DB 작업 원본에서 제외된다.

## 10. 선택형 웹 연결 (DB-6)

ROS 관제가 runs를 관리하는 시험에서 관제 PC 웹 서버를 실행한다면:

```bash
cd /home/hj/B_0102_Final_Project/ros2_ws/scv_app
SCV_RUN_OWNER=ros python3 server_page.py
```

웹에도 같은 시험 DB의 SCV_DB_DSN을 설정한다. 기본값 web은 기존 웹 순찰 흐름을 유지한다. ros 모드에서는 웹 순찰 시작·중단, `/ws/agent`, agent POST 입력을 차단한다. ROS 회차를 DB에서 읽어 갱신하고, ROS가 저장한 원본으로 히트맵을 재집계한다.

이는 회차 소유권 충돌을 막는 선택 모드다. **로봇 실시간 위치/상태·ROS 알림을 웹으로 보내는 ros_bridge나 tasks·inspections·보고서 저장을 완성한 것은 아니다.** 기존 RViz 히트맵은 기존 ROS 경로를 사용한다. D4 팀 배포 결정은 별도이며 기본 실행 모드를 일괄 변경하지 않았다.

## 11. 로봇 없는 PostgreSQL 자동 시험

시험 DB 접속 DSN을 SCV_TEST_DB_DSN에 설정한 뒤 실행한다. 테스트는 `_test`로 끝나는 DB 이름만 허용하고, 고유한 별도 스키마에 데이터와 표를 만든다. 운영 표를 사용하지 않는다. 실패 확인용 시험 스키마는 남긴다.

```bash
cd /home/hj/B_0102_Final_Project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export SCV_TEST_DB_DSN="$SCV_DB_DSN"
PYTHONPATH="$PWD/src/scv_survey:$PWD/src/scv_manager:$PYTHONPATH" \
/usr/bin/python3 -m pytest -q src/scv_survey/test src/scv_manager/test -p no:cacheprovider
```

DB 시험에는 psycopg, 웹 시험에는 scv_app requirements와 httpx가 같은 Python 환경에 필요하다. SCV_TEST_DB_DSN이 없으면 PostgreSQL 시험은 skipped이므로 이를 실기 성공으로 기록하지 않는다. 가짜 ROS만 domain 219·220에서 실행하며 실제 로봇 action/service는 부르지 않는다.

## 12. 실제 PC 간 자동 복구·DB 비교

1. 정상 통합 회차 N을 현장 절차로 시작. robot3의 status와 sample의 run_id=N인지 확인.
2. 관제 DB net_samples에 같은 회차 원본이 들어오는지 확인. 차단 전 seq 기록.
3. **관제 PC의 통신**을 30초 끊고 복구. 로봇 PC의 버퍼·TF·상태는 유지. 로봇 운용 담당자가 안전 상태를 관리한다.
4. 연결 복구 뒤 writer가 자동 sync를 호출하고 resent=true 표본을 수신하는지 확인. 수동 sync로 먼저 메워 자동 복구 검증을 대신하지 않는다.
5. 종료/안정 시점에 로봇 원본 CSV를 관제 PC에 복사한 뒤 아래 고유 번호 비교 수행. 재발행 큐가 비는 시간까지 기다린다.
6. 추가 시험: 시험 DB 서비스만 일시 정지·복구. DB_UNAVAILABLE·큐 유지·다음 단계 확정 보류·DB_RECOVERED 및 복구 후 원본 일치를 확인. 운영 DB 서비스는 끄지 않는다.
7. writer만 재시작해 원본 복구·DB 중복 0 확인. inspection_manager 자체의 순찰 중 재시작/하드웨어 재개는 이번 시험 범위 밖이다.

관제 PC의 비교 예시(파일·N·로봇명은 실제 값으로 수정):

```bash
python3 - <<'COMPARE'
import csv, os, psycopg
from pathlib import Path
robot, run = 'robot3', 3
path = Path('/복사한/robot3_buffer.csv')
with path.open() as f:
    rows = [r for r in csv.DictReader(f)
            if r['robot_id'] == robot and int(r['run_id']) == run]
expected = {int(r['seq']) for r in rows}
with psycopg.connect(os.environ['SCV_DB_DSN']) as conn:
    received = [r[0] for r in conn.execute(
        'SELECT seq FROM net_samples WHERE robot_id=%s AND run_id=%s', (robot, run))]
actual = set(received)
print('CSV 원본:', len(rows), 'DB 원본:', len(received))
print('누락:', sorted(expected - actual))
print('CSV에 없는 번호:', sorted(actual - expected))
print('DB 중복:', len(received) - len(actual))
COMPARE
```

성공 기준: 같은 안정 시점·같은 로봇·회차에서 **CSV 원본 수=DB 원본 수, 누락 0, 불필요한 번호 0, DB 중복 0**. seq가 연속이라는 가정만으로 판정하지 않고 실제 CSV와 비교한다. 계속 측정 중이면 복사 후 생긴 샘플로 개수가 달라질 수 있으므로 마지막 비교 번호를 함께 기록한다.

| 실제 시험 기록 | 결과 |
| --- | --- |
| 양쪽 PC 코드 버전·시험 DB·지도/AP | |
| 로봇·회차·차단 전후 seq | |
| 자동 sync from_seq·last_seq·count·복구 소요 시간 | |
| CSV 원본 수/DB 원본 수·누락·불필요한 번호·중복 | |
| DB 중단/복구 경보·commit 보류/재시도 | |
| 순찰 done/partial·칸 통계·재측정 결과 | |

13번의 구현·로봇 없는 검증과 위 실제 시험 완료는 분리해 기록한다.

## 13. 재검토 결과 (10/14)

이전 119개 시험만으로 예외 처리가 충분하다고 판단할 수 없었다. 추가 검토에서 아래를 확인하고 수정했다.

| 발견한 문제 | 수정·검증 |
| --- | --- |
| sync 무응답 중 commit이 들어오면 요청 시각이 초기화돼 재요청이 막힐 수 있음 | 기존 대기 요청 취소 후 새 요청. 해당 상황을 수정 전 실패로 재현하고 수정 후 통과 |
| writer 재시작 후 구역 확정 기록이 사라져 재측정 결과를 기존 측정값으로 덮을 수 있음 | DB 칸 통계에서 확정 구역 복원, 반복 commit은 기존 값을 유지. 중앙값 -50·표본 수 30 보존 검증 |
| 잘못된 과거 회차 한 건이 전체 원본 저장 묶음을 막음 | 회차 불일치 경보 후 해당 회차를 분리, 로봇 CSV는 보존. 정상 회차 저장을 수정 전 실패로 재현하고 수정 후 통과 |
| DB 종료 저장 실패에도 완료로 표시하던 기존 관제 흐름 | 종료 저장 성공까지 RUNNING 유지·경보·재시도. 실패/복구 전환 검증 |

중복된 회차 변경 처리는 set_run 한 곳으로 합쳤고, 시험에서만 사용한 번호 조회 도우미는 시험 파일로 옮겼다. 기능을 제거해 줄 수만 줄이는 방식은 사용하지 않았다.

주의: **운영 DB와 시험 DB마다 다른 buffer_csv_path를 사용한다.** 서로 다른 DB에서는 같은 숫자 run_id가 재사용될 수 있어 같은 로봇 CSV를 공유하면 과거 시험 표본을 같은 회차로 오인할 수 있다. robot_pc launch에도 해당 인자를 전달하도록 추가했다. 예: 시험은 `buffer_csv_path:=/기존/로그폴더/robot3_test_buffer.csv`, 운영은 기존 robot3_buffer.csv. 기존 로그를 삭제하지 않는다.

관제 manager가 처음 실행될 때 DB 준비에 실패했다면 DB를 먼저 준비하고 manager를 재시작해야 한다. 진행 중 회차의 DB 장애 복구 시험과 이 시작 순서는 구분한다. writer 단독 재시작 검증은 전체 manager/로봇의 순찰 재개 검증을 뜻하지 않는다.

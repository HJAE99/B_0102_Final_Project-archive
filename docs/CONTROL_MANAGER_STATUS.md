# 관제 A 구현·시험 결과 및 결정 요청

- 작성일: 2026-10-12
- 담당: 관제 A (`inspection_manager`)
- 브랜치: `feature/control`
- 구현 커밋: `54b104e` (`feat: 관제 작업 관리와 후보 재검증 로직 구현`)

## 1. 현재 상태

관제 A의 로컬 구현과 mock ROS 통합 시험을 완료했다. 실제 로봇 통합 전에
DOCK 작업의 생성 주체, IF-19 판정 의미 등 팀 간 결정이 필요한 항목은
아래 「결정 요청」에 정리했다.

## 2. 완료한 구현

- `/inspection/start` 요청으로 순찰 회차와 작업 생성
- 구역 polygon 기반 왕복 측정 경로 생성
- 설비별 관측 위치 기반 설비 점검 작업 생성
- 측정 작업은 `robot4`, 설비 점검 작업은 `robot3`에 고정 배정
- 네트워크 후보 수신 시 `robot4` 재측정 작업 생성
- 작업 상태 관리: `PENDING`, `ASSIGNED`, `RUNNING`, `SUCCEEDED`,
  `BLOCKED`, `FAILED`, `CANCELED`
- 비상정지, 재개 및 대기 작업 재배정
- `RobotStatus` 기반 ONLINE/OFFLINE 감시
- 기본 오프라인 감지 시간을 29초로 설정
- 작업을 받을 수 없는 로봇의 대기 사유를 로봇별로 로그 출력
- 노드 실행 시각 기반 `run_id` 생성
- 후보 및 설비 판정 결과 중복 처리 방지

## 3. 시험 결과

### 3.1 단위 테스트

- 결과: **14개 통과, 실패 0개**
- 대상:
  - 작업 상태 전이 및 완료
  - 비상정지와 재개
  - 고정 역할 작업의 잘못된 재배정 거절
  - 시작 시각 기반 `run_id`
  - 오프라인 경계와 상태 수신 후 온라인 복귀
  - 구역 왕복 경로 및 설비 관측 위치 생성
  - 중복 후보 무시
  - 다른 `run_id`의 후보 무시
  - 완료된 회차에서 후보 수신 시 회차 재개

재현 명령:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/scv_manager/test -q
```

확인된 결과:

```text
14 passed
```

### 3.2 mock ROS 액션 왕복

다음 흐름을 실제 ROS 2 서비스·토픽·액션 통신으로 확인했다.

1. `/inspection/start` 호출
2. `robot4`가 구역 측정 작업과 4개 웨이포인트 완료
3. `robot3`가 `EXT-01` 설비 점검 완료
4. 네트워크 후보 `net-candidate-001` 주입
5. 완료된 회차가 다시 `RUNNING`으로 전환
6. `robot4`가 후보 재측정 완료
7. 두 번째 `RUN_COMPLETED` 확인

확인한 알림 순서:

```text
RUN_STARTED
RUN_COMPLETED
CANDIDATE
REINSPECTION_DONE
RUN_COMPLETED
```

mock은 `require_robot_ready=false`로 실행했으며, 경로는
`scv_manager/test/data/zones.yaml`과 `facilities.yaml`을 사용했다.

### 3.3 노드 재시작 및 이전 후보 혼입 시험

- 첫 번째 회차: `run_id=1791770964`
- `inspection_manager` 종료 후 재시작
- 두 번째 회차: `run_id=1791770968`
- 이전 회차 후보를 TRANSIENT_LOCAL QoS로 계속 발행한 상태에서 재시작
- 두 번째 회차에 이전 후보의 재측정 작업이 생성되지 않음
- 두 번째 회차의 측정·설비 작업 및 `RUN_COMPLETED` 정상 확인

결과:

```text
run_id_changed=true
stale_candidate_task_created=false
```

## 4. PM·연동 팀 결정 요청

| 항목 | 현재 상황 | 관제 A 제안 | 확인 주체 |
| --- | --- | --- | --- |
| DOCK 작업 | `ExecuteTask`에 `DOCK` 타입이 있으나 생성 주체가 정해지지 않음 | `inspection_manager`가 DOCK 작업을 생성하고 주행팀 `mission_executor`가 실행 | PM·주행팀 |
| IF-19 이상 판정 | 현재 `ABNORMAL` 결과를 즉시 `CONFIRMED` 알림으로 변환 | 비전팀이 내부에서 3회 관측을 끝낸 최종 결과를 보내는 구조라면 현재 처리 유지 | PM·비전팀 |
| Alert code | 기본 E-code 외 관제 A 운영 알림이 추가됨 | 아래 code를 관제 B에서 표시·저장 | 관제 B |
| OFFLINE 중 RUNNING 작업 | 상태 수신 중단 시 E3는 발생하지만 실행 중 작업 처리 정책은 미정 | 역할 고정으로 교차 재배정할 수 없으므로 `FAILED(reason=OFFLINE)` 처리 | PM·주행팀 |
| 재배정 0건 | 현재 이동한 작업이 없으면 `ok=false` | 로봇 ID가 유효한 요청이면 `ok=true`, `moved_tasks=0` 및 안내 메시지 반환 | PM·관제 B |
| 실제 구역 좌표 | 운영 `zones.yaml`의 polygon이 모두 0이라 순찰 시작이 거절됨 | 통합 시험 전 실측 polygon 좌표 반영 | PM |

관제 B와 공유가 필요한 추가 Alert code:

```text
RUN_STARTED
RUN_COMPLETED
RESUMED
REINSPECTION_DONE
INSPECTION_NORMAL
TASK_FAILED
```

## 5. 결정 후 진행 순서

1. 위 결정 사항을 인터페이스 문서와 코드에 반영
2. 실제 `zones.yaml` 좌표 적용
3. 주행팀 액션 서버와 `require_robot_ready=true` 통합 시험
4. 실제 `RobotStatus` 수신 상태에서 A 측정 → B 설비 점검 → 후보 → A 재측정 확인
5. 통합 결과를 일지와 테스트 계획에 기록

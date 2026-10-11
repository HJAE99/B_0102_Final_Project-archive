# SCV 운영 DB (2차)

기준 문서: `SCV_DB구조도_2차.drawio` · `DB시스템제안설명_2차.docx` · `DB시스템구현방안_2차.docx` (통신 구조 안 C — 제어 PC ↔ 관제 PC는 웹 API)

```
scv_db/
├─ sql/
│  ├─ 01_schema.sql   테이블 20개 · 제약 · 인덱스 · 트리거 · 실패 처리 함수 · 컬럼 설명(COMMENT 201개) · 이전 판 업그레이드
│  ├─ 02_seed.sql     로봇 robot3·robot4 · 판정 기준값 13개 · 초기 계정 admin/viewer
│  └─ 03_verify.sql   규칙 검증 47항목 (한 트랜잭션, 끝에 ROLLBACK → 데이터 남지 않음)
└─ docker/
   ├─ setup_db.sh     관제 PC에서 실행: PostgreSQL·Redis 컨테이너 생성 + SQL 적용
   └─ redis.conf      Redis 설정 (ACL·AOF·키 만료 알림)
```

## 1. 실행 (관제 PC)

```bash
cd scv_db/docker
./setup_db.sh             # 컨테이너 + 01·02 적용
./setup_db.sh --verify    # 위 + 03 검증
```

스크립트가 하는 일:

| 순서 | 내용 |
|---|---|
| PostgreSQL | `docker pull postgres:16` → `docker volume create robotdata4_postgresql` → `docker run -d --name robotdb4_postgresql -e POSTGRES_DB=robotdb4_postgresql -e POSTGRES_USER=rokey -e POSTGRES_PASSWORD=rokey -p 5432:5432 -v robotdata4_postgresql:/var/lib/postgresql/data postgres:16` |
| SQL 적용 | `docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < ../sql/01_schema.sql` (02·03도 같은 방식) |
| Redis | `docker run -d --name robotdb4_redis -p 6379:6379 -v robotdata4_redis:/data -v $PWD/redis.conf:/usr/local/etc/redis/redis.conf:ro redis:8 redis-server /usr/local/etc/redis/redis.conf` |

여러 번 실행해도 됩니다. 컨테이너·볼륨이 있으면 그대로 쓰고, SQL은 `IF NOT EXISTS` · `CREATE OR REPLACE` · `ON CONFLICT DO NOTHING`이라 이미 있는 데이터와 admin이 바꾼 설정값을 덮어쓰지 않습니다.

SQL만 따로 적용할 때:

```bash
docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < sql/01_schema.sql
docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < sql/02_seed.sql
```

## 2. 접속 정보

| | 주소 | 계정 |
|---|---|---|
| PostgreSQL | `postgresql://rokey:rokey@<관제PC>:5432/robotdb4_postgresql` | rokey / rokey |
| Redis | `redis://rokey:rokey@<관제PC>:6379/0` | rokey / rokey (default 사용자는 꺼 둠) |
| server page 초기 계정 | — | admin / `admin1234!`, viewer / `viewer1234!` (bcrypt 해시 저장, 첫 로그인 뒤 변경) |

DB에 쓰는 것은 관제 PC의 api(inspection_manager 포함)뿐입니다. 제어 PC는 DB에 접속하지 않으므로, 컨테이너가 같은 PC에서만 쓰이면 포트를 `-p 127.0.0.1:5432:5432`, `-p 127.0.0.1:6379:6379`로 묶어 외부 접속을 막는 것을 권장합니다. DBeaver 등으로 다른 PC에서 볼 때만 열어 두세요.

## 3. DB가 직접 지키는 규칙

애플리케이션 코드에 버그가 있어도 아래 규칙은 DB가 거부합니다(03_verify.sql에서 확인).

| 규칙 | 구현 |
|---|---|
| 진행 중인 run은 하나 | 부분 유일 인덱스 `runs_one_running` |
| 로봇마다 ACTIVE task는 하나 | `tasks_one_active_per_robot` |
| 현재 지도 하나 · 칸·AP마다 현재 기준선 하나 | `maps_one_active` · `baselines_one_current` |
| 표본 중복 저장 0 (끊겼다 이어도) | `net_samples` PK `(robot_id, run_id, seq)` |
| 이벤트·판정·정지 측정·사진 재전송 중복 0 (제어 PC 아웃박스) | `robot_events.event_uid` UK · `inspections.inspection_uid` UK · `precise_measures (task_id, ap_id)` UK · `media.object_key` UK |
| 후보는 1차 task에서, 그 task의 로봇이 발견 | 트리거 `scv_check_candidate` |
| **다른 로봇이 재검증** (재검증 로봇 ≠ 발견 로봇), 재검증 task의 target_ref = candidate_id | 트리거 `scv_check_candidate` |
| **재검증 없이 확정 0건** (findings는 outcome = CONFIRMED + 그 후보의 재검증 task일 때만) | 트리거 `scv_check_finding` |
| 확정 이상이 있는 후보의 결론 변경 금지 | 트리거 `scv_lock_confirmed_candidate` |
| 판정·정지 측정·이벤트의 robot_id = task의 로봇 | 트리거 `scv_check_task_robot` |
| 후보·확정 이벤트에는 task_id 필수, 관제 PC 판단(source = control)은 E3·E8만 | CHECK |
| 중단된 run은 기준선 회차가 될 수 없음 | CHECK `runs_baseline_only_done` |

task 실패 처리(그 task FAILED → 남은 task CANCELED → run aborted)는 한 트랜잭션으로 묶은 함수로 제공합니다.

```sql
SELECT scv_fail_task('r12-t003', 'BLOCKED');   -- 반환값: 취소된 task 수
```

api는 제어 PC가 다시 보낸 데이터를 오류 없이 무시하도록 `ON CONFLICT … DO NOTHING`으로 저장합니다.

```sql
INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, detail, ts)
VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
ON CONFLICT (event_uid) DO NOTHING;            -- 다른 표: (robot_id, run_id, seq) · (inspection_uid) · (task_id, ap_id) · (object_key)
```

`event_uid`는 제어 PC scv_agent가 아웃박스에 쓸 때 붙이고(예: `robot3-r12-e000123`), 관제 PC가 판단하는 E3·E8은 api가 `control-<uuid>`로 붙입니다. `inspection_uid`는 detection_alert가 판정마다 만드는 UUID입니다.

## 4. 운영 메모

- **볼륨이 데이터입니다.** `docker rm`은 컨테이너만 지우고 데이터는 `robotdata4_postgresql`·`robotdata4_redis`에 남습니다. 처음부터 다시 만들 때만 `docker rm -f robotdb4_postgresql && docker volume rm robotdata4_postgresql`.
- **백업:** `docker exec robotdb4_postgresql pg_dump -U rokey -Fc robotdb4_postgresql > scv_$(date +%Y%m%d).dump`
- **이전 01_schema.sql로 이미 DB를 만들었다면** 새 01_schema.sql을 그대로 다시 실행하면 됩니다. 파일 끝의 업그레이드 구간이 `event_uid`·`inspection_uid` 컬럼과 세 유일키를 추가하고, 기존 행에는 `legacy-<event_id>`·임의 UUID를 채웁니다.
- **03_verify.sql은 운영 전에.** 진행 중인 run이 있으면 `runs_one_running` 때문에 실패합니다. 모두 ROLLBACK되어 데이터는 남지 않고 번호 시퀀스만 조금 올라갑니다.
- `/docker-entrypoint-initdb.d`에 SQL을 넣는 방식은 **볼륨이 비어 있는 첫 실행 때만** 동작하므로, 이미 볼륨을 만든 상태에서도 쓸 수 있게 `docker exec`로 적용합니다.
- 측정 대상 AP(`aps`)는 SSID를 정한 뒤 02_seed.sql 아래쪽 예시의 주석을 풀어 넣고, 지도·격자·설비는 SLAM 지도를 만든 뒤 server page 설정 탭(또는 등록 스크립트)에서 넣습니다.
- 설정값의 `source`(BR-T04, SR-06 …)는 문서의 「제안」 근거이며, 첫 측정 뒤 조정합니다.
- 비밀번호(rokey, 초기 계정)는 시연용입니다. 외부망에 연결하면 바꾸세요. Redis는 `redis.conf`의 `user rokey on >rokey` 줄을 고칩니다.

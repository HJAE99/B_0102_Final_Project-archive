# SCV 관제 웹 — 3-2차

통신 구조 안 C를 따르는 3차 구현의 두 번째 단계입니다. DB는 **scv_db_3차**(섹터 판)를 씁니다.

| 차수 | 내용 |
|---|---|
| 2_C-1 | 관제 PC server page — PostgreSQL 기준 정보 7개 표 |
| 2_C-2 | 제어 PC client page · 가상 로봇 robot3 — 로그인, 로봇 활성 알림(hello), 로봇 정보·카메라 |
| **3-1** | **가상 세계 묶음**(시험 지도 · world.yaml · 등록 스크립트), **Redis 도입**(로그인 세션 · 로봇 상태 · 응답 없음 E3), client **status 1 Hz** → robot_status_log, 모니터에 **로봇 위치·상태 실시간 표시**(`/ws/ui`) |
| **3-2** | **로봇 2대 동시 운행 · 섹터 방식** — 격자를 만들 때 **섹터 1 · 2**로 나눔(임시 규칙 `split_sectors`), 모니터에 섹터 색 표시 · 정보 탭에 섹터별 칸 수, **순찰 진행 중 격자 변경 잠금**, 순찰 중 현재 지도 바꾸기 잠금. **AP · 설비가 지도에 속함**(정보 탭에서 지도를 골라 추가 — 격자와 같은 방식, 설비 ID는 지도 안 유일), **도크 접근 자세**를 정보 › 지도 › 수정에서 입력 → 관제 PC가 제어 PC에 config로 보내고 client가 `initialpose`로 로봇 시작 위치 지정, 순찰 중인 지도의 AP · 설비 · 도크 잠금, 지도 삭제 때 그 지도의 격자 · AP · 설비 함께 삭제. 모니터에 **도크 · AP · 점검 설비** 표시, 활성 로봇의 **관제 PC 메모리 상태 전체** 표시. **Redis 제거** — 로그인 세션 · 로봇 최신 상태 · 응답 없음(E3) 판정을 관제 PC(server_page) 메모리로 옮김(9장). 가상 세계는 `world.yaml` 하나(측정·판정으로 알아낼 실제 세계 값만). DB: `grid_cells.sector` · `runs.sector_robots` · 로봇마다 ACTIVE task 1개 · 섹터별 실패(`partial`) · `maps.dock_poses` · aps `(map_id, ssid)` · facilities 키 `(map_id, facility_id)` |

```
scv_app/
├─ server_page.py          관제 PC — FastAPI(화면 + API) · Uvicorn. client page용 로그인 API · WebSocket /ws/agent
├─ client_page.py          제어 PC — FastAPI(화면 + API) + rclpy(별도 스레드) + 관제 PC WebSocket 클라이언트
├─ robot3_virtual.py       시험용 가상 로봇 — ROS 2 노드, /robot3/… 토픽 발행
├─ ros_check.py            ROS 통신 점검 — client와 같은 방식으로 로봇 토픽이 보이고 값이 오는지 확인
├─ virtual_world/          가상 로봇 시험 전용 세계 (3-1)
│  ├─ testbed.pgm · testbed.yaml   시험 지도 — 12 × 8 m 방(칸막이 · 작은 방 · 기둥 · 선반), 1 m 격자가 벽과 맞음
│  ├─ world.yaml                   실제 세계의 값 — AP 신호 원점 · 세기 모델, 음영 구역, 설비 실제 상태 (가상 로봇만 읽음, DB에 넣지 않음)
│  └─ load_world.py                testbed 지도를 server API로 등록 + 현재 지도 지정 (AP · 설비 · 도크는 화면에서 입력)
├─ requirements.txt        관제 PC (server_page)
├─ requirements-client.txt 제어 PC (client_page · robot3_virtual) — rclpy 등은 ROS 2 Jazzy에서
└─ static/
   ├─ server/              server page 화면 (index.html · app.js · app.css)
   ├─ client/              client page 화면 (index.html · app.js · app.css — server/app.css 위에 덧붙임)
   └─ vendor/              인터넷 없이 쓰도록 동봉한 라이브러리
                           react-htm.mjs (React 19 + htm, 빌드 없이 사용) · leaflet.js · leaflet.css · images/
```

```
 로봇(robot3_virtual) ──ROS 2──▶ 제어 PC client_page ──HTTP · WebSocket──▶ 관제 PC server_page ──▶ PostgreSQL
   /robot3/battery_state ·         로그인 확인        POST /api/client/login
   dock_status · odom · ip ·       활성 알림          WS /ws/agent  hello { robot_id }
   amcl_pose(3-1) ·                status 1 Hz(3-1)   WS /ws/agent  status { x, y, yaw, battery_pct, nav_state … }
   ◀ initialpose(3-2) ◀──────────── config(3-2) ◀──── WS /ws/agent  config { map, dock_pose } (연결 · 지도 변경 · 도크 수정 때)
   oakd/rgb/image_raw/compressed                                        │
                                                       관제 PC 메모리 (로봇별 최신 상태) · PostgreSQL robot_status_log
                                                       server page 모니터 ◀── WS /ws/ui (1초마다)
```

## 0. 3-2차 시험 순서 (가상 로봇)

```bash
# 관제 PC
cd scv_db/docker && ./setup_db.sh --test --reset              # 시험 DB robotdb4_test (PostgreSQL 컨테이너 1개)
cd scv_app
SCV_DB_DSN="host=127.0.0.1 port=5432 dbname=robotdb4_test user=rokey password=rokey" python3 server_page.py
python3 virtual_world/load_world.py --robot-map-path <제어 PC의 virtual_world/testbed.yaml 절대 경로>   # 지도 등록 + 현재 지도
# → server page 정보 탭에서 testbed 지도에 아래 표의 도크 · AP · 설비를 입력

# 제어 PC (ROS 2를 source한 터미널 — 8장 Discovery Server 주의)
python3 robot3_virtual.py
python3 client_page.py --server http://<관제PC>:8000            # http://<제어PC>:8080 → admin 또는 user 계정으로 로그인
```

**testbed 입력값** (정보 탭 — 목록 위 지도는 처음부터 testbed(현재 지도)로 골라져 있음)

| 어디에 | 값 |
|---|---|
| 지도 › 수정 › 도크 접근 자세 | robot3 (1.0, 0.6, 1.5708) · robot4 (7.0, 0.6, 1.5708) — 각자 섹터 안 |
| 측정 대상 AP › 추가 | SCV_VAP_A 「가상 AP A — 왼쪽 위」 (2.0, 6.5) · SCV_VAP_B 「가상 AP B — 가운데 오른쪽」 (9.0, 3.5) — SSID는 world.yaml `ap_models`와 같아야 함 |
| 점검 설비 › 추가 | ID · 종류 · 압력계 · 위치(x, y) · 관측 자세(x, y, yaw) |
|  | EXT-01 · 소화기 · 예 · (0.15, 4.5) · (0.9, 4.5, 3.1416) |
|  | EXT-02 · 소화기 · 예 · (11.85, 1.0) · (11.1, 1.0, 0) |
|  | EXT-03 · 소화기 · 아니요 · (5.85, 2.5) · (5.1, 2.5, 0) |
|  | SHT-01 · 셔터 · 아니요 · (6.05, 6.5) · (5.2, 6.5, 0) |
|  | EXL-01 · 유도등 · 아니요 · (10.0, 7.85) · (10.0, 7.1, 1.5708) |
|  | EXL-02 · 유도등 · 아니요 · (0.15, 2.0) · (0.9, 2.0, 3.1416) |

- **로봇 시작 위치** — 가상 로봇은 켜지면 도크에 서 있지만 지도 위 위치를 모릅니다(amcl_pose 없음). client가 관제 PC에 연결되면 config로 도크 접근 자세를 받고, 로봇이 도킹 중이면 `/robot3/initialpose`로 보내 시작 위치를 정합니다. 도크를 아직 넣지 않았다면 정보 › 지도 › 수정에서 넣는 순간 다시 보내집니다. client 로봇 정보 탭의 「도크 접근 자세 (관제 PC)」 칸에서 받은 값과 보낸 시각을 볼 수 있습니다.
- 모니터의 격자가 **섹터 1(파랑, x 0~6 m · 48칸) · 섹터 2(주황, x 6~12 m · 48칸)**로 칠해지고, 범례에 섹터별 칸 수와 기본 배정(섹터 1 = robot3, 섹터 2 = robot4)이 나옵니다. 정보 › 격자 칸 패널에도 섹터별 칸 수 · ix 범위가 나옵니다.
- 이미 3-1차로 만든 시험 DB라면 새 `01_schema.sql`을 다시 적용하면 기존 칸에 섹터가 채워집니다(`./setup_db.sh --test`). 정보 › 격자 칸 › 목록 › 「격자 다시 생성」으로 다시 만들어도 됩니다.
- 지도 위에 **도크**(robot3 (1.0, 0.6) · robot4 (7.0, 0.6), 로봇 색 네모 + 점선 = 도크 방향), **AP**(📶 SCV_VAP_A · B), **점검 설비 6개**(소 = 소화기 · 셔 = 셔터 · 유 = 유도등, 점선 끝 점 = 관측 자세)가 나옵니다. 음영 구역은 측정 결과로만 표시합니다(3-4차 히트맵).
- 「로봇 상태」에서 활성 로봇은 관제 PC 메모리에 있는 최신 상태의 모든 필드와 E3까지 남은 초를 그대로 보여 줍니다(1초마다 갱신).
- 시작 위치가 정해지면 server page 모니터의 지도 위 도크(1.0, 0.6)에 robot3가 보이고, 「로봇 상태」에 위치·배터리(충전 중)·DOCKED가 1초마다 갱신됩니다.
- client를 끄면 「연결 끊김 — n초 뒤 응답 없음」 → `status_timeout_s`(30초) 뒤 **E3(응답 없음)**가 robot_events에 남고 「비활성」으로 바뀝니다.
- 가상 로봇은 `virtual_world/world.yaml`(실제 세계의 값)을 3-4차부터 씁니다. 도크 위치는 파일에 두지 않고 관제 PC가 보낸 initialpose로만 압니다. 등록할 때의 `--robot-map-path`는 제어 PC에서 testbed.yaml이 있는 경로입니다(3-3차에 client가 지도를 불러올 때 씀).

## 1. 설치·실행 (관제 PC)

```bash
cd scv_app
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 server_page.py                 # http://<관제PC>:8000  →  /server/login
```

| 환경변수 | 기본값 | 설명 |
|---|---|---|
| `SCV_DB_DSN` | `host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey` | scv_db_3차로 만든 DB (시험은 robotdb4_test) |
| `SCV_MEDIA_DIR` | `./scv_media` | 등록한 지도 PNG 저장 위치 (`maps/`). 더미 데이터(`make_dummy.py --media`)도 같은 곳으로 |
| `SCV_HOST` / `SCV_PORT` | `0.0.0.0` / `8000` | |

- 로그인은 수퍼 계정 **admin / admin**(`02_seed.sql`)만 됩니다. 일반 계정(권한 user)은 client page용이라 server page 로그인이 거부됩니다.
- 워커는 1개(프로세스 1개)로만 실행합니다. 제어 PC WebSocket 연결(`/ws/agent`), 로그인 세션, 로봇 최신 상태를 이 프로세스 메모리가 쥐고 있기 때문입니다. 워커 1개는 단일 스레드가 아니라 이벤트 루프 + 요청 처리 스레드 풀이며, 메모리 상태는 잠금 하나로 보호합니다. server를 다시 시작하면 다시 로그인해야 합니다(9장).
- HTTPS·nginx는 쓰지 않습니다(포트폴리오 범위, 보안 고려 안 함).

## 2. 화면

| 주소 | 내용 |
|---|---|
| `/server/login` | admin 로그인. 로그인 전에는 다른 탭이 비활성 |
| `/server/monitor` | **현재 지도**를 가운데에 표시(Leaflet, 좌표 단위 m — 이미지 왼쪽 아래 = yaml origin). 그 위에 **격자 칸**(grid_cells)을 칸마다 사각형으로 그리고 **격자 번호**(큰 숫자 = cell_id, 작은 숫자 = ix,iy)를 표시. 칸이 화면에서 26 px보다 작으면 번호를 숨기고, 46 px 이상이면 ix,iy도 보임. 아래 세 칸은 추후 구현 자리(로봇 상태 · 순찰 진행 · 이벤트) |
| `/server/master` | **정보** 탭. 오른쪽 세로 탭에서 표를 누를 때마다 선택·해제(여러 개 가능), 선택한 표는 선택한 순서대로 왼쪽에 세로로 쌓여 표 이름 · 설명 · 행 수 · 현재 값을 보여 줌. 다른 탭에 다녀와도 선택 유지. 「목록」 → 목록 창에서 추가 · 삭제 · 값 수정 |

**행 출력 규칙** — 정보 탭 패널과 목록 창 모두 **최신 순 10행**씩 보이고, 10행을 넘으면 아래 쪽 번호(‹ 1 … 4 5 6 ›)로 다음 10행을 봅니다. 추가하면 1쪽으로 이동합니다.

| 표 | 최신 순 기준 |
|---|---|
| robots | registered_at ↓ |
| users | created_at ↓ |
| settings | updated_at ↓ (값이 같으면 key) |
| aps | ap_id ↓ (목록 창, 지도별) |
| maps | created_at ↓ |
| grid_cells | cell_id ↓ (목록 창, 지도별) |
| facilities | facility_id ↓ (목록 창, 지도별) — 등록 시각 컬럼이 없어 ID 내림차순 |

maps · grid_cells · facilities 패널은 행 나열이 아니라 요약(현재 지도 · 칸 수 · 종류별 개수)이라 쪽이 없습니다.

## 3. 표별 동작 규칙

| 표 | 추가 | 수정 | 삭제 | 비고 |
|---|---|---|---|---|
| robots | ○ | ○ (주소) | ○ | robot_id 형식은 DB CHECK(영문 소문자 시작, 하이픈 없음). **활성은 client(제어 PC)가 값을 보내 줄 때만** — server page에서 바꿀 수 없고, 추가하면 비활성 |
| users | ○ (아이디·비밀번호) | ○ | ○ | 추가하면 **권한 user**(client page 계정). 수정은 아이디·비밀번호(비밀번호는 입력했을 때만 바뀜). **수퍼 계정 admin**은 비밀번호만 수정, 삭제 버튼 없음 — DB CHECK·트리거도 같은 규칙. 이름 `admin`은 다른 계정이 쓸 수 없음. 비밀번호는 DB의 pgcrypto `crypt(…, gen_salt('bf'))`로 해시 |
| settings | ○ | ○ | ○ | **설명(description)** 표시·입력·수정. 바꾸면 updated_by(로그인 사용자)·updated_at이 자동 기록. 정보 탭 패널에도 key 아래 설명 표시 |
| aps | ○ | ○ (SSID·메모·설치 x, y) | ○ | **지도에 속함** — 목록 창 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. 지도마다 SSID는 하나(다른 지도에는 같은 SSID 가능). 설치 위치 x · y는 함께 넣거나 둘 다 비움. **활성은 client·로봇이 그 AP 값을 보내 줄 때만** — 추가하면 비활성. 순찰 중인 지도는 잠금(409) |
| maps | 업로드 | ○ (도크 접근 자세만) | ○ | 「지도 등록」: 이름 · 제어 PC 지도 경로 · 격자 크기 + **yaml · pgm** → resolution · origin 자동 추출(origin yaw = 0, negate = 0만 허용), pgm을 크기 그대로 PNG로 저장. **수정은 로봇별 도크 접근 자세**(도크 바로 앞에서 도크를 바라보는 x, y, yaw — 비우면 그 로봇 도크 없음)만, 바꾸면 현재 지도일 때 제어 PC에 바로 다시 보냄. 순찰(run) 진행 중에는 그 지도의 도크 수정 · 「현재 지도로」 다른 지도 고르기 잠금(409). **삭제하면 그 지도의 격자 칸 · AP · 설비도 함께 지움** — 순찰 · 측정 · 판정 기록이 있으면 「사용 중」으로 거부 |
| grid_cells | ✕ | ✕ | 지도 단위 | 지도를 **「현재 지도로」 선택할 때 그 지도만** 격자가 없으면 생성(섹터도 이때 정함). 목록 창에서 지도별 「격자 다시 생성」·「격자 삭제」 — **그 지도로 순찰(run)이 진행 중이면 409로 거부** |
| facilities | ○ | ○ | ○ | **지도에 속함** — AP와 같이 목록 위 지도에 추가. 설비 ID는 지도 안에서 유일(다른 지도에는 같은 ID 가능). 관측 자세는 x · y · yaw 세 칸 → JSON. 순찰 중인 지도는 잠금(409) |

- **삭제 규칙** — 다른 표가 참조하는 행은 DB 외래키 때문에 지워지지 않습니다. 이때 「사용 중 — 비활성(is_active = false)으로 바꾸세요」라고 안내합니다. 지도는 그 지도의 격자까지 함께 지우되, 설비·측정 기록이 참조하면 지우지 않습니다.
- **격자 생성 규칙** — 지도의 **흰 부분(빈 공간)이 조금이라도 걸치는 칸은 모두** 만듭니다. 픽셀 occupancy = (255 − 값)/255가 0.25 미만이면 흰 부분이고, 값 205(map_saver의 미탐색, 회색)는 뺍니다. 벽 여유(`settings.wall_margin_m`)로 칸을 빼지 않으므로 벽 가까이 좁게 남은 흰 띠에도 칸이 생깁니다. 칸 번호 기준점은 지도 원점입니다. 이전 규칙으로 만든 격자는 목록 창의 「격자 다시 생성」으로 바꿉니다.
- **섹터 규칙(3-2, 임시)** — 칸을 ix 순으로 세어 **칸 수가 절반에 가장 가까운 열**에서 나눕니다. 그 열까지(왼쪽)가 섹터 1, 오른쪽이 섹터 2입니다(같은 차이면 왼쪽 열, 열이 하나면 모두 섹터 1). `server_page.split_sectors()` 하나만 바꾸면 다른 분할 규칙으로 바꿔 끼울 수 있습니다. DB 업그레이드 구간과 더미 생성도 같은 규칙입니다. 설비의 섹터는 view_pose가 속한 칸의 섹터입니다(저장하지 않음).
- **섹터 ↔ 로봇** — 기본 배정은 섹터 1 = robot3, 섹터 2 = robot4(`DEFAULT_SECTOR_ROBOTS`)입니다. 순찰 시작 때 바꿀 수 있고, 활성인 로봇의 섹터만 그 회차에 넣어 `runs.sector_robots`에 저장합니다(3-3차).
- **활성 여부(robots · aps)** — client·로봇이 보내 준 값으로만 정합니다. 아직 값을 받는 기능이 없으므로(client page·scv_agent 구현 차수에서 추가) **server_page를 시작할 때 모두 비활성으로 맞춥니다.**

## 4. DB에 대해

- scv_db_3차 스키마를 씁니다(settings.description, users 권한 admin | user, grid_cells.sector, runs.sector_robots · status partial, maps.dock_poses, aps · facilities가 지도에 속함, inspections.map_id). 이전 판 DB는 새 `01_schema.sql` → `02_seed.sql`을 다시 실행하면 맞춰집니다.
- 트리거가 거부한 경우(예: admin 삭제 시도)는 DB 메시지를 그대로 400으로 보여 줍니다.


## 5. 제어 PC — client page (2_C-2차)

### 5.1 실행

```bash
source /opt/ros/jazzy/setup.bash
cd scv_app
python3 -m venv --system-site-packages .venv && source .venv/bin/activate   # rclpy를 쓰려면 --system-site-packages
pip install -r requirements-client.txt
python3 client_page.py --server http://<관제PC>:8000     # http://<제어PC>:8080 → /client/login
```

| 옵션 | 환경변수 | 기본값 | 설명 |
|---|---|---|---|
| `--robot-id` | `SCV_ROBOT_ID` | `robot3` | 이 제어 PC와 짝인 로봇 |
| `--server` | `SCV_SERVER_URL` | `http://127.0.0.1:8000` | 관제 PC server page 주소 |
| `--camera-topic` | `SCV_CAMERA_TOPIC` | `/{robot_id}/oakd/rgb/image_raw/compressed` | 카메라 탭에서 보여 줄 토픽 — 기본은 짝인 로봇(robot3)의 카메라 |
| `--host` / `--port` | `SCV_CLIENT_HOST` / `SCV_CLIENT_PORT` | `0.0.0.0` / `8080` | |
| `--check-s` | `SCV_ACTIVE_CHECK_S` | `2` | 로봇이 비활성일 때 토픽 목록 확인 주기(초) |


### 5.2 동작

| 순서 | 내용 |
|---|---|
| 로그인 | 첫 화면은 `/client/login`. 아이디·비밀번호를 관제 PC `POST /api/client/login`으로 확인합니다(제어 PC는 DB에 접속하지 않음). DB users의 **user**와 **admin** 모두 로그인할 수 있습니다. 로그인 전에는 탭이 비활성 |
| 로봇 활성 확인 | 로그인하면 바로 ROS 토픽 목록(`get_topic_names_and_types`)을 보고, `/robot3/…` 토픽 중 **발행자가 있는 토픽**이 하나라도 있으면 활성입니다(이 노드의 구독만 있는 토픽은 세지 않음). 비활성이면 `--check-s`마다 다시 확인하고, 활성이 되면 확인을 멈춥니다 |
| 관제 PC에 알림 | 활성이 되면 관제 PC `WebSocket /ws/agent`에 접속해 **hello**(`robot_id`, `agent_version`, `run_id`, `active_task_id`, `outbox_pending` — 구현방안 6.2절)를 **한 번** 보냅니다. 연결은 계속 유지하며, 끊기면 다시 접속해 hello를 보냅니다(구현방안 6.3절) |
| 로봇 정보 탭 | `/robot3/battery_state` · `dock_status` · `odom` · `ip`를 구독(best effort)해 1초마다 표시. 5초 넘게 값이 없으면 「끊김」 |
| 로봇이 꺼지면 | 활성인 동안 위 네 토픽의 값이 **5초 넘게 하나도 오지 않으면** 로봇이 꺼진 것으로 보고 비활성으로 바꿉니다. 관제 PC WebSocket 연결을 닫아 **관제 PC의 robots.is_active도 false**가 되고, 토픽 목록 확인을 다시 시작합니다. 다시 활성이 되려면 토픽 목록의 발행자와 **꺼진 뒤 새로 받은 값**이 함께 있어야 합니다(DDS 목록에 잠시 남는 옛 발행자로 다시 켜지지 않게). 활성이 되면 다시 접속해 hello를 보냅니다 |
| 카메라 탭 | 카메라 토픽(`sensor_msgs/CompressedImage`)을 **탭을 보는 동안만** 구독하고, JPEG를 그대로 MJPEG(`/api/camera/stream`)으로 화면에 보냅니다. 다른 탭으로 가면 구독 해제 |

### 5.3 관제 PC 쪽 처리 (server_page)

- `POST /api/client/login` — users에서 아이디·비밀번호 확인(admin · user 모두). 결과만 돌려주고 server page 세션은 만들지 않습니다.
- `WebSocket /ws/agent` — 첫 메시지가 hello가 아니면 닫습니다. hello의 `robot_id`가 robots에 있으면 **is_active = true**로 바꾸고 welcome을 보냅니다. 등록되지 않은 로봇이면 error 후 닫습니다. **연결이 끊기면 is_active = false**(값이 오지 않으면 비활성 규칙). 같은 로봇이 다시 접속하면 이전 연결을 닫습니다.
- server page를 다시 시작하면 시작할 때 모두 비활성이 되고, client가 다시 접속해 hello를 보내면 활성으로 돌아옵니다.

## 6. 가상 로봇 — robot3_virtual.py

```bash
source /opt/ros/jazzy/setup.bash
python3 robot3_virtual.py                # /robot3/…
python3 robot3_virtual.py --camera-hz 5  # 카메라 주기 (0이면 카메라 끔)
```

구현방안 3장의 기존 로봇 통신 중 **지금 client page가 쓰는 것만** 구현했고, 이후 차수에서 client가 구독·요청하는 것이 늘어날 때마다 추가합니다.

| 토픽 (`/robot3/…`) | 타입 | 주기 | 값 |
|---|---|---|---|
| `battery_state` | sensor_msgs/BatteryState | 1 Hz | 87%에서 천천히 줄어듦, 방전 중 |
| `dock_status` | irobot_create_msgs/DockStatus | 1 Hz | 도킹 아님 (irobot_create_msgs가 없으면 생략) |
| `odom` | nav_msgs/Odometry | 10 Hz | 반지름 1 m 원을 0.2 m/s로 |
| `ip` | std_msgs/String | 1 Hz | 이 PC의 IP |
| `oakd/rgb/image_raw/compressed` | sensor_msgs/CompressedImage | 10 Hz | 640 × 360 합성 JPEG (OpenCV, 없으면 Pillow) |

서비스·액션은 아직 없습니다.

## 7. client에 로봇 값이 안 뜰 때

client_page.py를 실행하는 **같은 터미널**에서 `python3 ros_check.py`를 실행하면 환경 변수, `/robot3/…` 토픽 목록(발행자 수), 5초 동안 받은 값 개수와 판단을 보여 줍니다.

| 결과 | 원인 | 조치 |
|---|---|---|
| 발행자가 안 보임 | 로봇과 client의 ROS 환경이 다름 | 두 터미널의 `ROS_DOMAIN_ID` · `ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE` · `ROS_DISCOVERY_SERVER` · `RMW_IMPLEMENTATION`을 맞춤 |
| 토픽은 보이는데 값이 0개 | 발견은 되지만 데이터 전달이 막힘(같은 PC의 공유 메모리 등) | 양쪽에 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 후 다시 실행 |
| 값이 옴 | ROS 통신 정상 | client 터미널 로그 확인 — 토픽 목록 확인 결과, 「첫 값 받음 …」, 콜백 오류(`[ros] … 오류`)가 찍힘 |

## 8. 주의사항 — Discovery Server와 가상 로봇 시험

TurtleBot4를 쓰는 PC는 `~/.bashrc`에 Fast DDS **Discovery Server** 설정이 들어 있습니다(예: `ROS_DISCOVERY_SERVER=;;;;;192.168.109.105:11811;`, `ROS_SUPER_CLIENT=True`). 이 설정이 켜져 있으면 노드는 같은 PC에 있어도 **서로 직접 찾지 않고 그 서버를 거쳐서만** 찾습니다. 서버는 실제 로봇의 Raspberry Pi에서 돌아가므로 그 로봇이 꺼져 있거나 주소에 닿지 않으면, robot3_virtual과 client_page가 정상으로 떠도 서로 보이지 않습니다. client 상태줄에는 「로봇 확인 중 — 아직 /robot3/… 발행자 없음」이 계속 보이고 로봇 정보·영상이 오지 않습니다.

### 8.1 가상 로봇으로 시험할 때 — Discovery Server 끄기

로봇(robot3_virtual) 터미널과 client(client_page) 터미널 **양쪽 모두**에서 실행한 뒤 각각 다시 실행합니다.

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
ros2 daemon stop        # ros2 topic list가 쓰는 데몬이 예전 설정을 기억하지 않게
```

`ROS_DOMAIN_ID`는 양쪽이 같으면 그대로 둡니다. `unset`은 **그 터미널에서만** 설정을 지우며 `~/.bashrc`는 바뀌지 않습니다.

Discovery Server를 그대로 쓰고 싶으면 이 PC에 서버를 직접 띄우는 방법도 있습니다.

```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811        # 별도 터미널
export ROS_DISCOVERY_SERVER=127.0.0.1:11811          # 로봇·client 터미널 양쪽
```

### 8.2 시험이 끝난 뒤 — Discovery Server 다시 켜기

셋 중 하나로 되돌린 뒤, 실행 중이던 로봇·client 프로그램도 다시 실행합니다.

| 방법 | 명령 | 비고 |
|---|---|---|
| A. 새 터미널 | `unset`한 터미널을 닫고 새 터미널을 연 뒤 `ros2 daemon stop` | 가장 간단. `~/.bashrc`가 다시 읽힘 |
| B. 같은 터미널에서 다시 읽기 | `source ~/.bashrc` → `ros2 daemon stop` | |
| C. 값을 직접 넣기 | `export ROS_DISCOVERY_SERVER=";;;;;192.168.109.105:11811;"` · `export ROS_SUPER_CLIENT=True` → `ros2 daemon stop` | 값은 이 PC의 `~/.bashrc`에 있던 것. `;` 때문에 따옴표 필요 |

되돌아왔는지 확인:

```bash
printenv | grep -E 'ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT'   # 원래 값이 나오면 복구
ros2 topic list                                               # 실제 로봇이 켜져 있으면 그 로봇 토픽이 보임
```

### 8.3 공통

- 로봇 · client · `ros2` 명령을 쓰는 **모든 터미널을 같은 상태**(모두 끔 또는 모두 켬)로 맞춥니다. 한쪽만 바꾸면 서로 보이지 않습니다.
- 설정을 바꾼 뒤에는 `ros2 daemon stop`으로 데몬을 재시작합니다(다음 `ros2` 명령 때 새 설정으로 다시 켜짐).
- 실제 로봇과 함께 쓰는 최종 구성에서는 `~/.bashrc`의 Discovery Server 설정을 그대로 씁니다.
- 원인이 분명하지 않으면 client 터미널 환경에서 `python3 ros_check.py`로 확인합니다(7장).

## 9. 관제 PC 메모리 상태 · 로봇 상태 (3-1 도입, 3-2에서 Redis → 메모리)

3-1차에는 Redis에 두었던 실시간 상태를 3-2차부터 server_page 프로세스 메모리에 둡니다. client가 로봇 토픽을 받아 1 Hz status로 줄여 보내므로 관제 PC가 처리할 양이 적고, 메모리에만 있는 기록은 만들지 않는다는 원칙(영구 기록은 모두 PostgreSQL)을 지키면 별도 저장소가 필요 없기 때문입니다.

| 항목 | 형식 | 쓰는 곳 | 내용 | server 재시작 때 |
|---|---|---|---|---|
| 로그인 세션 | token → 사전, 8 h | server 로그인 | user_id · username · role · created_at. 요청마다 8 h 연장, 로그아웃·계정 삭제 때 지움, 계정 이름 변경 때 함께 바꿈 | 다시 로그인 |
| 로봇 최신 상태 | robot_id → 사전 + 마지막 수신 시각 | `/ws/agent` status | ts · run_id · task_id · nav_state · progress · x · y · yaw · battery_pct · is_docked · estop · received_at | 다음 status(1초 안)로 다시 채움 |

- **status** — client의 mission_executor가 1초마다 만들고(위치 = `{ns}/amcl_pose`, 배터리·도킹 = 로봇 토픽) scv_agent가 `/ws/agent`로 보냅니다(구현방안 6.2절 status). 관제 PC는 메모리 최신값을 덮어쓰고 수신 시각을 갱신하며, 위치·배터리가 있으면 `robot_status_log`에도 남깁니다(같은 (robot_id, ts)는 무시).
- **E3 응답 없음** — 1초마다 도는 점검(watchdog)이 마지막 수신 뒤 `settings.status_timeout_s`(30 s)가 지난 로봇을 찾아 `robot_events`에 `E3 · source = control · level = warn`을 남기고 메모리 상태를 지웁니다. 같은 점검이 만료된 로그인 세션도 정리합니다.
- **이후 차수** — e-stop 상태(3-7)는 메모리 + robot_events에서 복원, 순찰 진행률(3-3)은 runs/tasks 조회, 실시간 칸 값(3-4)은 net_samples에서 다시 계산, 최근 이벤트(3-5)는 메모리 목록 + robot_events에서 다시 읽기로 같은 방식을 씁니다.
- **모니터 실시간** — 브라우저가 `/ws/ui`에 접속하면 1초마다 로봇별 상태를 받습니다(로그인 세션 쿠키로 확인). 지도 위 마커(방향 표시 포함)와 「로봇 상태」 칸이 갱신되며, 상태는 정상 / 연결 끊김(만료 대기) / 응답 없음 / 비활성으로 나눕니다.
- **가상 로봇 3-1** — 도크(= 시작 위치)에 서서 충전합니다(battery_state 충전 중, dock_status 도킹). `{ns}/amcl_pose`(map 좌표)를 2 Hz로 내보내 실제 구성의 제어 PC AMCL을 대신합니다. 이동(Nav2 · dock · undock)은 3-3차입니다. 3-2차부터 시작 위치는 관제 PC의 도크 접근 자세(→ client initialpose)로 정하고, 3-3차 DOCK task는 waypoints = [도크 접근 자세]로 그 자리까지 이동한 뒤 dock 액션을 부릅니다(robot4 가상 로봇 · 두 대 시험 환경은 이후 차수).

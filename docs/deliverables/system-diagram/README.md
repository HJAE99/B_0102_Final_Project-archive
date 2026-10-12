# 시스템 다이어그램 v2.0 (10/12)

코드 기준으로 다시 그린 시트 8장이다. 편집용 원본은 `../scv-system-diagram.drawio` (draw.io 에서 페이지 8개), 확대·이동해서 보는 판은 팀 아티팩트 「SCV System Diagram」.

- 실선 상자 = 지금 팀 브랜치 코드에 있는 것 · 점선 상자 = 10/12 결정으로 더할 것(아직 코드 없음)
- 보라 굵은 상자 = ROS·TurtleBot4 가 제공하는 노드(AMCL · Nav2 · Create 3 · map_server …) — 내부 흐름은 그리지 않고, 우리 코드의 어느 함수로 들어오고 무엇으로 나가는지만 그렸다
- 모든 화살표에 넘겨지는 값과 타입을 적었다 (노드 사이: 토픽·액션·서비스 이름과 메시지 타입, 노드 안: 변수와 파이썬 타입)

| 파일 | 내용 | 근거 코드 |
| --- | --- | --- |
| `0-system.png` | 전체 관계 — 우리 노드 ↔ ROS·TurtleBot4 제공 노드 ↔ DB·웹 | 전체 |
| `1-amr-mission_executor.png` | mission_executor 코드 흐름 (작업 수행 · 막힘 E1 · 배터리 E2 · 자세 보정 · Dock) | feature/driving `807b7e9` |
| `2-rssi.png` | rssi_scanner · survey_buffer · network_map_engine | feature/survey `9d3a4cc` |
| `3-detection.png` | detection_alert (설계) — 입력 → YOLO/depth/OpenCV → 판정 → 결과 | vision_pkg/test.py |
| `4-inspection_manager.png` | inspection_manager 코드 흐름 + 10/12 로 바꿀 곳 | feature/control `54b104e` |
| `5-monitor-event-db-web.png` | 이벤트 → ros_bridge·API → 테이블 CRUD → Redis → 화면 | scv_database 01_schema.sql |
| `6-web-pages.png` | 웹 페이지 구성도 · 로그인/로그아웃/메뉴 전환 · 버튼 → API → ROS | 설계 |
| `7-db-schema.png` | PostgreSQL 표 20개 · 열·타입 · 외래키 · 10/13 고칠 곳 | scv_database 01_schema.sql |

# Victus(관제 PC) 10/15 통합 작업 요약

## 갱신 이력 (worktree ~/scv_integ, origin/temp/integration-1014)
- 17:46 468afac: scv_app · scv_db 를 ros2_ws/src 로 이동 반영, 지도 PNG 수동 이동, scv_bringup 재빌드
- ~19:40 7259e85: 실측 설비 8곳 facilities.yaml. DB 업그레이드 1회차(setup_db.sh --test): net_samples.detected, grid_stats.n_detected/n_missed, settings precise_duration_s=15. 전체 재빌드 8패키지(scv_msgs 액션 결과 n_total 추가). venv 파이썬으로 빌드돼 shebang 이 venv 로 박힌 것 발견 → venv 제외 PATH 로 재빌드(/usr/bin/python3)
- ~19:50 a4b2af2: 로봇 카드 카메라 + server_page zones.yaml 경로 수정(7259e85 그대로면 ros 모드 웹 기동 실패). 재빌드
- ~20:30 e005689: 블랙박스(이력 탭 재생) · 설비 점검 팝업 · 소화기 ArUco 제조 정보. DB 업그레이드 2회차: inspections.aruco JSONB. 재빌드
- 설비 등록: facilities(map_id 1) 8행 upsert + SHT-02 is_active=false (도윤 직접 실행, 세션은 DB 쓰기 차단)
- AP 등록: iPhone(ap_id 38) is_target → 새 핫스팟 KDY-Zfold8 로 교체 SQL 전달 (실행은 도윤)
- robot4 는 이날부터 robot4-pc(mu-09)에서 기동, Victus 의 start_robot4.sh 미사용

## 겪은 문제 · 조치
- 오전~오후(run 1~9): 노드 켜면 인터넷 끊김 → ① robot4 rssi_scanner 가 Victus Wi-Fi 에서 매초 iw scan(채널 이탈) → rssi:=false ② robot4 Pi 가 죽은 Victus DDS 포트로 계속 송신(stale match) → Pi 재부팅. 교훈: 로봇 떠 있을 때 DS 재시작 금지, launch 는 SIGINT 로 종료
- 매회 robot4 도크 아래 칸 · (1,12) 등 BLOCKED 로 측정 구간 일부 실패(E1) — run 10~15 에도 지속
- /robot4/inspect/result QoS 비호환 경고: 비전 노드가 같은 토픽에 VOLATILE(주행 중 불량) · TRANSIENT_LOCAL(설비 판정) publisher 둘 → 설비 판정은 수신, 주행 중 불량 이벤트는 관제 미수신(코드 의도)
- run 15 종료 직후(21:19:43) robot4 RobotStatus 끊김(E3). Pi 는 ping 정상 → robot4-pc 측 발행 중단 추정
- 셔터 판정 detail: focal_length_px 1036 vs camera_info 570, suggested 953 → 초점거리 설정 재확인 권장

## run 10~15 회차표 (KST, 측정 AP 모두 iPhone)
| run | 시각 | 상태 | robot3 측정/점검/도킹 | robot4 측정/점검/도킹 | 표본(미검출) | 격자칸 | 이벤트(warn·err) |
|---|---|---|---|---|---|---|---|
| 10 | 19:58~20:08 | partial | 20 DONE·4 FAIL / 4 normal / 2 | 17·5 / SHT-01 ABN, RCK-01 OCC, EXT-01 ABN, EXL-01 ABN / 2 | 1109 (45) | 60 | 13 (E1 9, CONFIRMED 3, E6 1) |
| 11 | 20:35~20:37 | aborted | 6 DONE·18 CANCEL / – / 취소 | 5·2·15 CANCEL / – / 1 | 224 (224) | 0 | 2 (E1) |
| 12 | 20:41~20:41 | aborted | 24 CANCEL / – / 1 | 22 CANCEL / – / 1 | 28 (28) | 0 | 0 |
| 13 | 20:59~21:00 | aborted | 1 DONE·23 CANCEL / – / 1 | 22 CANCEL / – / 1 | 112 (2) | 0 | 0 |
| 14 | 21:02~21:07 | aborted (E7 비상정지 21:03) | 24 CANCEL / – / 1 | 1 FAIL·21 CANCEL / – / 1 | 592 (3) | 0 | 3 (E1 1, E7 2) |
| 15 | 21:09~21:19 | partial | 22 DONE·2 FAIL / 4 normal / 2 | 18·4 / RCK-01 OCC, EXT-01 ABN, EXL-01 ABN, SHT-01 BLOCKED / 2 | 1172 (125) | 59 | 10 (E1 7, CONFIRMED 2, E6 1) |

- 설비 점검 완주 회차: run 10(8/8 기록), run 15(7/8, SHT-01 접근 BLOCKED). robot3 담당 4곳은 두 회차 모두 normal. robot4 의 EXT-01(압력계 바늘) · EXL-01(유도등 OFF) ABNORMAL 은 두 회차 재현 → 실제 상태 확인 필요
- run 11~14 는 측정 표본이 거의 미검출(핫스팟 꺼짐 추정)이거나 비상정지로 중단
- inspections.aruco 는 전부 NULL (로봇이 e005689 이전 코드로 주행)

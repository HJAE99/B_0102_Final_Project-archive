# victus/1016/viewcheck_run20 — run 20 (10/16 10:25~10:35) 설비 점검 사진 (robot4 4장 + EXT-01 압력계 3배 확대)

- view 는 04b1cfc 기준. Δ = 실제 − view (cm, °; 반시계 +). 자세는 inspections.inspected_at 에 가장 가까운 robot_status_log 행.
- **robot3 사진 없음**: robot3 비전이 4곳 모두 `yolo: no recent camera frame available` 로 즉시 UNCONFIRMED → inspections 행은 있으나(OCCLUDED/unconfirmed) InspectionResult.image 가 비어 media 행·파일 없음. detail 은 'mock' 이 아님(실제 비전). 같은 회차 robot3 블랙박스 폴더(recordings/r20/robot3)도 아예 없음(run 19 는 514장) → **robot3 OAK-D 카메라 스트림이 이번 기동에서 안 나옴**(로봇 PC 쪽 문제, 관제는 정상 수신·저장 경로 그대로).

| 로봇 | 설비 | 판정 | detail | 이유(알림) | media | x, y (m) | yaw 실제 / view (°) | Δx, Δy (cm) | Δ거리 | Δyaw (°) | 파일 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| robot3 | EXT-02 | unconfirmed | OCCLUDED | yolo: no recent camera frame available | 없음 | 0.53, 4.25 | 141.4 / 140.0 | -1, +2 | 2 | +1.4 | - |
| robot4 | SHT-01 | unconfirmed | OCCLUDED | shutter: 판정 못 함 (결과 없음, 6s) | 있음 | 2.58, 2.90 | -178.8 / 173.3 | +0, -1 | 1 | +7.9 | r20_robot4_SHT-01_inspection.jpg |
| robot3 | EXT-03 | unconfirmed | OCCLUDED | yolo: no recent camera frame available | 없음 | 2.48, 5.75 | 54.0 / 53.0 | -9, -11 | 14 | +1.0 | - |
| robot3 | RCK-02 | unconfirmed | OCCLUDED | yolo: no recent camera frame available | 없음 | 1.83, 6.05 | 154.0 / 168.0 | +6, -3 | 7 | -14.0 | - |
| robot4 | RCK-01 | abnormal | ABNORMAL | {"mode": "yolo", "source": "yolo_rack_box_containment", "rea | 있음 | 1.84, 0.31 | -173.9 / -169.0 | +1, -3 | 3 | -4.9 | r20_robot4_RCK-01_inspection.jpg |
| robot3 | EXL-02 | unconfirmed | OCCLUDED | yolo: no recent camera frame available | 없음 | 2.11, 4.41 | -55.3 / -78.0 | -5, +6 | 8 | +22.8 | - |
| robot4 | EXT-01 | abnormal | ABNORMAL | {"text": "{\"mode\": \"yolo\", \"reason\": \"needle color ab | 있음 | 0.66, 1.37 | 145.5 / 143.0 | -1, +1 | 2 | +2.5 | r20_robot4_EXT-01_inspection.jpg<br>r20_robot4_EXT-01_gauge_x3.jpg |
| robot4 | EXL-01 | abnormal | ABNORMAL | {"mode": "yolo", "source": "exit_sign_brightness", "reason": | 있음 | 0.93, 0.93 | -178.7 / -175.3 | -1, +1 | 1 | -3.4 | r20_robot4_EXL-01_inspection.jpg |

robot3 의 RCK-02 Δyaw −14°, EXL-02 +22.8°, EXT-03 Δ거리 14 cm 는 비전이 카메라 프레임 없음으로 즉시 실패해 settle 이 끝나기 전 자세가 찍힌 것으로 보임 [추정].

## 사진 소견 (robot4)

- **EXT-01 압력계(3배 확대 r20_robot4_EXT-01_gauge_x3.jpg)**: 바늘은 **연노랑 화살표**, 약 11시 방향. 비전이 그린 녹색 정상 표시는 12시의 좁은 띠이고 바늘은 그 바로 왼쪽 바깥(약 15~20°). 로봇은 이번에 view 보다 +2.5°(반시계) 에 섰음 → 왼쪽에서 비스듬히 보는 좌우 시차로 바늘이 왼쪽으로 밀려 보이는 것과 일치(run 19 EXT-03 은 오른쪽). 판정 'needle color absent inside normal region'. 소화기 자체는 x 22~72 % 가운데.
- SHT-01: 자세 Δ1 cm, Δyaw **+7.9°**(run 17 +3.5 → 18 +5.0 → 19 +9.0 → 20 +7.9, robot4 settle 이 계속 반시계로 더 감). 띠가 오른쪽으로 치우쳐 오른쪽 선이 화면 밖, 왼쪽 맨바닥 쐐기(x 0~15 %)까지 마스크 → zone_partial, 6 s 판정 못 함.
- RCK-01: 랙 가운데(x 30~60 %), 'box 0.93 ABNORMAL Overflow RIGHT=52.9px' — 윗칸 상자가 랙 밖으로 나온 것을 검출(실제 배치가 그런지 확인).
- EXL-01: 유도등 가운데, 'exit_sign 0.95 OFF'.

## 공개 저장소 점검

5장 모두 사람 없음(SHT-01 은 오버레이가 상단을 가림), 얼굴 0. 모니터의 '(평가 7) FINAL PRESENTATION & DE…' 제목 외 판독 가능한 글자 없음. 비밀값 없음.

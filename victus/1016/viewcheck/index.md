# victus/1016/viewcheck — 설비 관측 자리에서 설비가 화면 가운데 보이는지 확인용 (10/15 run 10 · run 15)

- 점검 사진(`*_inspection.jpg`): 관제 DB media 에 저장된 비전 근거 사진(robot4, 640 px). robot3 은 가짜 비전이라 점검 사진 없음.
- 블랙박스 프레임(`*_bb1/bb2_<epoch ms>.jpg`): run 15 scv_media/recordings/r15 에서 점검 시각(inspections.inspected_at, BLOCKED 는 작업 종료 시각) 기준 ±4 s 안 가장 가까운 2장. **run 10 은 블랙박스 기능 전(e005689 배포 전)이라 프레임 없음.**
- 자세: 그 시각에 가장 가까운 robot_status_log 행(dt ≤ 0.5 s). Δ = 실제 − facilities.yaml view (x·y cm, yaw °, 반시계 +).

| run | 로봇 | 설비 | 판정 | nav | x, y (m) | yaw (°) | Δx, Δy (cm) | Δ거리 (cm) | Δyaw (°) | 파일 |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | robot3 | EXT-02 | normal | IDLE | 0.53, 4.25 | 125.7 | -2, +1 | 3 | -4.4 | (사진 없음) |
| 10 | robot4 | SHT-01 | abnormal | IDLE | 2.58, 2.92 | 179.8 | +0, +1 | 1 | -1.5 | r10_robot4_SHT-01_inspection.jpg |
| 10 | robot3 | EXT-03 | normal | IDLE | 2.57, 5.87 | 35.8 | +0, +1 | 1 | -1.2 | (사진 없음) |
| 10 | robot4 | RCK-01 | unconfirmed | EXECUTING | 1.83, 0.31 | 172.7 | -0, -3 | 3 | -3.0 | r10_robot4_RCK-01_inspection.jpg |
| 10 | robot3 | RCK-02 | normal | IDLE | 1.77, 6.08 | 179.0 | -0, -1 | 1 | +2.0 | (사진 없음) |
| 10 | robot4 | EXT-01 | abnormal | IDLE | 0.69, 1.35 | 157.9 | +1, -1 | 1 | +3.7 | r10_robot4_EXT-01_inspection.jpg |
| 10 | robot3 | EXL-02 | normal | IDLE | 2.14, 4.32 | -86.8 | -2, -2 | 3 | +1.8 | (사진 없음) |
| 10 | robot4 | EXL-01 | abnormal | EXECUTING | 0.94, 0.93 | 179.6 | +1, +0 | 1 | -5.1 | r10_robot4_EXL-01_inspection.jpg |
| 15 | robot4 | SHT-01 | BLOCKED(FAILED) | EXECUTING | 2.81, 0.94 | -175.8 | +23, -197 | 199 | +2.9 | r15_robot4_SHT-01_bb1_1792066636264.jpg<br>r15_robot4_SHT-01_bb2_1792066637464.jpg |
| 15 | robot4 | RCK-01 | unconfirmed | IDLE | 1.82, 0.32 | 173.5 | -2, -2 | 3 | -2.2 | r15_robot4_RCK-01_inspection.jpg<br>r15_robot4_RCK-01_bb1_1792066653864.jpg<br>r15_robot4_RCK-01_bb2_1792066655164.jpg |
| 15 | robot3 | EXT-02 | normal | EXECUTING | 0.52, 4.23 | 135.6 | -3, -0 | 3 | +5.5 | r15_robot3_EXT-02_bb1_1792066666617.jpg<br>r15_robot3_EXT-02_bb2_1792066667617.jpg |
| 15 | robot4 | EXT-01 | abnormal | EXECUTING | 0.68, 1.34 | 156.7 | -0, -2 | 2 | +2.4 | r15_robot4_EXT-01_inspection.jpg<br>r15_robot4_EXT-01_bb1_1792066670264.jpg<br>r15_robot4_EXT-01_bb2_1792066673264.jpg |
| 15 | robot4 | EXL-01 | abnormal | IDLE | 0.94, 0.94 | -179.2 | +1, +2 | 2 | -3.9 | r15_robot4_EXL-01_inspection.jpg<br>r15_robot4_EXL-01_bb1_1792066692964.jpg<br>r15_robot4_EXL-01_bb2_1792066693964.jpg |
| 15 | robot3 | EXT-03 | normal | EXECUTING | 2.57, 5.87 | 36.2 | +1, +0 | 1 | -0.7 | r15_robot3_EXT-03_bb1_1792066695217.jpg<br>r15_robot3_EXT-03_bb2_1792066696217.jpg |
| 15 | robot3 | RCK-02 | normal | IDLE | 1.77, 6.09 | -177.8 | -0, +0 | 0 | +5.3 | r15_robot3_RCK-02_bb1_1792066713117.jpg<br>r15_robot3_RCK-02_bb2_1792066714217.jpg |
| 15 | robot3 | EXL-02 | normal | EXECUTING | 2.16, 4.35 | -89.9 | -0, +0 | 0 | -1.4 | r15_robot3_EXL-02_bb1_1792066733917.jpg<br>r15_robot3_EXL-02_bb2_1792066734917.jpg |

비고: r15 robot4 SHT-01 은 관측 자리로 가지 못하고 BLOCKED 로 실패(10 s) — 자세는 도크 위(2.81, 0.94)라 Δ거리 199 cm, 프레임은 도크 주변 화면이며 셔터가 아니다.

## 공개 저장소 점검

- 23장 전부 눈으로 검토(서브에이전트 + 본 세션 확인). 사람 등장 9장, 얼굴 식별 가능 2장 — r15_robot3_EXL-02_bb1 · bb2 상단(칸막이 뒤 3명)을 모자이크 처리(빨간 테두리 영역, 유도등은 그대로). 나머지 7장은 몸통·뒷모습·머리 잘림으로 식별 불가. 비밀값 없음.

## 화면 가운데 여부 (검토 메모)

- 가운데: EXL-01(r10·r15 5장) · EXL-02 · EXT-02 · RCK-02(약간 우측).
- 치우침: **EXT-01(robot4, 4장) 소화기가 우측 끝에 걸쳐 잘림**, **RCK-01(robot4, 4장) 랙이 좌측 끝에 걸치거나 화면 밖**(UNCONFIRMED 박스가 빈 칸막이에 그려짐), EXT-03(robot3) 소화기가 좌측 절반.
- 자세 오차는 전부 ≤ 3 cm · ≤ 5.5°(BLOCKED 제외)라 로봇은 view 에 정확히 섰음 → 치우침은 facilities.yaml 의 view yaw(또는 x·y) 값 자체 문제. EXT-01 은 yaw 를 반시계(+)로, RCK-01 은 시계(−)로 돌리거나 자리를 옮겨야 함. SHT-01 r10 사진은 디버그 오버레이라 판단 불가, r15 는 BLOCKED.

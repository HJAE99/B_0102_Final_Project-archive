# victus/1016/viewcheck_r17 — a28b71c 각도 보정 뒤 첫 회차(run 17, 10/16 08:45~08:56) 설비 관측 사진

- 보정값: EXT-01 yaw +154.3°→+132°, RCK-01 +175.7°→−160°, EXT-03 +37°→+57° (facilities.yaml a28b71c). Δ 는 이 새 view 기준.
- `*_inspection.jpg` 비전 근거 사진(robot4), `*_bbN_<epoch ms>.jpg` 블랙박스(r17) 점검 시각 ±4 s(EXT-01 은 ±7 s, 3장). robot3 는 가짜 비전이라 블랙박스만.
- 자세: 점검 시각에 가장 가까운 robot_status_log 행(dt ≤ 0.2 s). Δ = 실제 − view (cm, °; yaw 반시계 +).

| 로봇 | 설비 | 판정 | nav | x, y (m) | yaw (°) | Δx, Δy (cm) | Δ거리 | Δyaw (°) | 파일 |
|---|---|---|---|---|---|---|---|---|---|
| robot3 | EXT-02 | normal | IDLE | 0.53, 4.24 | 128.8 | -2, +1 | 2 | -1.3 | r17_robot3_EXT-02_bb1_1792108419614.jpg<br>r17_robot3_EXT-02_bb2_1792108420614.jpg |
| robot3 | EXT-03 | normal | EXECUTING | 2.60, 5.89 | 60.5 | +3, +2 | 4 | +3.5 | r17_robot3_EXT-03_bb1_1792108447114.jpg<br>r17_robot3_EXT-03_bb2_1792108448114.jpg |
| robot4 | SHT-01 | abnormal | IDLE | 2.55, 2.93 | -175.2 | -2, +1 | 3 | +3.5 | r17_robot4_SHT-01_inspection.jpg<br>r17_robot4_SHT-01_bb1_1792108463069.jpg<br>r17_robot4_SHT-01_bb2_1792108464269.jpg |
| robot3 | RCK-02 | normal | IDLE | 1.77, 6.09 | -179.0 | -0, +1 | 1 | +4.0 | r17_robot3_RCK-02_bb1_1792108465214.jpg<br>r17_robot3_RCK-02_bb2_1792108466214.jpg |
| robot3 | EXL-02 | normal | IDLE | 2.17, 4.35 | -94.1 | +0, +1 | 1 | -5.5 | r17_robot3_EXL-02_bb1_1792108486314.jpg<br>r17_robot3_EXL-02_bb2_1792108487414.jpg |
| robot4 | RCK-01 | abnormal | EXECUTING | 1.84, 0.31 | -162.8 | +1, -3 | 3 | -2.8 | r17_robot4_RCK-01_inspection.jpg<br>r17_robot4_RCK-01_bb1_1792108488770.jpg<br>r17_robot4_RCK-01_bb2_1792108489970.jpg |
| robot4 | EXT-01 | unconfirmed(E6 yolo 결과 없음) | EXECUTING | 0.66, 1.37 | 124.7 | -2, +1 | 2 | -7.3 | r17_robot4_EXT-01_inspection.jpg<br>r17_robot4_EXT-01_bb1_1792108507570.jpg<br>r17_robot4_EXT-01_bb2_1792108508670.jpg<br>r17_robot4_EXT-01_bb3_1792108509870.jpg |
| robot4 | EXL-01 | abnormal | EXECUTING | 0.93, 0.94 | -179.1 | +0, +2 | 2 | -3.8 | r17_robot4_EXL-01_inspection.jpg<br>r17_robot4_EXL-01_bb1_1792108530670.jpg<br>r17_robot4_EXL-01_bb2_1792108531770.jpg |

## 화면 위치 검토

21장 전부 눈으로 검토(서브에이전트). 화면 가로 위치(x, 0 % = 왼쪽):
- **EXT-01(robot4, yaw +132°→실제 124.7°): 소화기가 왼쪽 끝 x 0~10 % 에 조각만 걸림** — 어제(+154.3°, 실제 157)는 오른쪽 끝 x 58~100 %. 비전은 "fire_extinguisher 0.91 UNCONFIRMED / Pressure gauge not detected" 를 띄우다 6 s 안에 판정 못 해 E6. **과보정**: 두 회차를 잇는 비율 ≈ 2.5 %/° (화면 가로 100 % ≈ 40°, 640×640 크롭) → 가운데(50 %)로 오려면 실제 yaw ≈ +143°. view 를 **+143°(2.50 rad)** 로 되돌리기 권장(현 settle 오차 ±7° 면 ±18 % 흔들리므로 가운데 겨냥이 중요).
- RCK-01(robot4, −160°→실제 −162.8°): 랙 전체가 화면 안, 오른쪽 치우침 x 58~88 %(중심 73 %). 어제는 왼쪽 끝 x 0~25 %. 판정 ABNORMAL("box 0.91 ABNORMAL Overflow RIGHT=58.5px", 박스가 랙 밖으로 돌출). 가운데로 하려면 시계 방향 약 9° → view **−169°(−2.95 rad)** 권장. 판정은 이미 가능.
- EXT-03(robot3, +57°→실제 60.5°): 소화기 오른쪽 x 65~100 %(오른쪽 끝 잘림). 어제(+37°)는 왼쪽 x 0~38 %. 가운데는 그 사이 → view **+45°(0.785 rad)** 권장.
- 그대로 좋은 곳: EXL-01 가운데(x 30~60 %, 판정 OFF), RCK-02 오른쪽 치우침이지만 전체 보임(x 62~95 %), EXL-02 왼쪽 치우침(x 6~40 %), EXT-02 왼쪽(x 0~28 %, 어제 run 15 는 가운데였음 — robot3 settle 편차).
- SHT-01: 관측 자리 정확(Δ3 cm), 띠 안 장애물 검출 ABNORMAL(바닥 분할 오버레이 사진).

## 공개 저장소 점검

- 사람 등장 4장(robot3 EXT-03 bb1·bb2 상단 뒷모습 1명, EXL-02 bb1·bb2 원경 1명) — 모두 식별 불가지만 보수적으로 모자이크(빨간 테두리). 식별 가능 얼굴 0장. 비밀값·판독 가능한 개인정보 없음(모니터 화면은 아이콘만).

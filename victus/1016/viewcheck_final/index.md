# victus/1016/viewcheck_final — run 19 (10/16 09:28~09:39) 설비 8곳 관측 사진 (두 로봇 모두 실제 비전)

- view 는 24175cf 기준(SHT-01 +173.3°, RCK-01 −169°, EXT-01 +143°, EXT-03 +45°, 나머지 원래 값). Δ = 실제 − view (cm, °; 반시계 +).
- `*_inspection.jpg` 비전 근거 사진(DB media), `*_bb_<epoch ms>.jpg` 점검 시각에 가장 가까운 블랙박스 원본 프레임(±1 s).

| 로봇 | 설비 | 판정 | detail | nav | x, y (m) | yaw 실제 / view (°) | Δx, Δy (cm) | Δ거리 | Δyaw (°) | 파일 |
|---|---|---|---|---|---|---|---|---|---|---|
| robot3 | EXT-02 | abnormal | ABNORMAL | EXECUTING | 0.51, 4.26 | 121.0 / 130.1 | -4, +2 | 5 | -9.1 | r19_robot3_EXT-02_inspection.jpg<br>r19_robot3_EXT-02_bb_1792111015519.jpg |
| robot3 | EXT-03 | abnormal | ABNORMAL | EXECUTING | 2.57, 5.88 | 41.8 / 45.0 | -0, +2 | 2 | -3.2 | r19_robot3_EXT-03_inspection.jpg<br>r19_robot3_EXT-03_bb_1792111040819.jpg |
| robot4 | SHT-01 | unconfirmed | OCCLUDED | EXECUTING | 2.56, 2.94 | -177.6 / 173.3 | -2, +3 | 3 | +9.0 | r19_robot4_SHT-01_inspection.jpg<br>r19_robot4_SHT-01_bb_1792111056879.jpg |
| robot3 | RCK-02 | normal | NORMAL | EXECUTING | 1.76, 6.07 | -175.8 / 176.9 | -1, -1 | 2 | +7.3 | r19_robot3_RCK-02_inspection.jpg<br>r19_robot3_RCK-02_bb_1792111062519.jpg |
| robot4 | RCK-01 | abnormal | ABNORMAL | EXECUTING | 1.86, 0.32 | -163.4 / -169.0 | +3, -2 | 3 | +5.6 | r19_robot4_RCK-01_inspection.jpg<br>r19_robot4_RCK-01_bb_1792111078379.jpg |
| robot3 | EXL-02 | unconfirmed | OCCLUDED | EXECUTING | 2.15, 4.37 | -92.3 / -88.6 | -1, +2 | 3 | -3.7 | r19_robot3_EXL-02_inspection.jpg<br>r19_robot3_EXL-02_bb_1792111085519.jpg |
| robot4 | EXT-01 | abnormal | ABNORMAL | EXECUTING | 0.73, 1.33 | 139.8 / 143.0 | +5, -4 | 6 | -3.2 | r19_robot4_EXT-01_inspection.jpg<br>r19_robot4_EXT-01_bb_1792111091779.jpg |
| robot4 | EXL-01 | abnormal | ABNORMAL | EXECUTING | 0.92, 0.92 | -176.4 / -175.3 | -1, -0 | 1 | -1.1 | r19_robot4_EXL-01_inspection.jpg<br>r19_robot4_EXL-01_bb_1792111113879.jpg |

## 화면 위치 · 권장 각도

화면 가로 위치(x, 0 % = 왼쪽; 가로 100 % ≈ 40°, 약 2.5~3 %/°, 반시계로 돌리면 장면이 오른쪽으로 이동):
| 설비 | 화면 위치 | 비전 표시 | 판단 | 권장 view (현재 → 권장) |
|---|---|---|---|---|
| SHT-01 (robot4) | 띠 가까운 끝 x 5~100 %(오른쪽 잘림), 먼 끝 x 45~100 %, 왼쪽 선만 보임 | zone_partial, 마스크가 띠 왼쪽 맨바닥 쐐기까지 포함(벽은 제외) | 띠가 여전히 오른쪽. 실제 yaw 가 view 보다 **+9.0° 반시계**(run 17 +3.5, 18 +5.0, 19 +9.0 — robot4 settle 이 매번 반시계로 더 감) | +173.3° → **+166°(2.90 rad)**; settle 편차 원인(spin/settle) 확인 권장 |
| RCK-01 (robot4) | 랙 x 31~60 % 가운데, 전체 보임 | rack 0.95 ABNORMAL, box Overflow RIGHT=49.8px | 구도 좋음 | 유지 |
| EXT-01 (robot4) | 소화기 x 15~62 % 가운데, 압력계 x 32~45 % | Pressure gauge 0.97, Pressure ABNORMAL | 구도 좋음 | 유지 |
| EXL-01 (robot4) | 유도등 x 27~58 % 가운데 | exit_sign 0.97 OFF | 구도 좋음 | 유지 |
| EXT-02 (robot3) | **소화기 x 0~21 % 왼쪽 끝 잘림(절반 밖), 압력계 x 0~8 % 잘림** | Pressure gauge 0.95(잘린 채), ABNORMAL | 최악. 실제 yaw 121.0° vs view 130.1°(Δ −9.1°) | +130.1° → **+140°(2.44 rad)** (가운데 오려면 실제 ≈ +137°) |
| EXT-03 (robot3) | 소화기 x 2~49 % 왼쪽 치우침(가로는 다 보임), 압력계 x 20~33 % | Pressure gauge 0.97, ArUco ID 2 VALID | 약간 왼쪽 | +45° → **+53°(0.925 rad)** |
| RCK-02 (robot3) | 랙 x 63~95 % 오른쪽 치우침, 전체 보임 | rack 0.93 NORMAL | 약간 오른쪽, 판정은 됨 | +176.9° → **+168°(2.93 rad)** |
| EXL-02 (robot3) | 유도등 x 3~37 % 왼쪽 치우침, 전체 보임 | exit_sign 0.96 UNCONFIRMED → E6 판정 못 함 | 보이는데도 6 s 안에 판정 못 함(밝기 판정 쪽 문제 가능) | −88.6° → **−78°(−1.36 rad)** |

### 소화기 압력계 (도윤님: 실물은 3곳 모두 정상)
- 세 곳 모두 "Pressure ABNORMAL", 이유 "needle color absent inside normal region"(source yellow_needle_presence) — 규칙이 **노란색 바늘**이 녹색 정상 구간 안에 있는지를 봄.
- EXT-01 사진: 압력계 지름 ≈ 90 px(640 px 기준), 다이얼 위쪽 녹색 호 + 양옆 빨강, 바늘은 가늘고 **어두운 금속색**(노란색 아님), 끝이 위쪽 녹색 쪽을 향하는 것으로 보이나 해상도상 확정 어려움.
- EXT-03 사진: 지름 ≈ 85 px, 같은 다이얼, 바늘 어두운 색, 위쪽(녹색) 근처.
- EXT-02 사진: 압력계가 왼쪽 끝에 잘려 절반만 보임 → 판정 불가.
- 결론: 세 곳이 실제 정상인데 ABNORMAL 이 나온 것은 구도가 아니라 **바늘 색 규칙(노란 바늘 전제)이 실물(어두운 바늘)과 안 맞기 때문** — 가운데 잡힌 EXT-01 도 같은 결과. 비전 쪽에서 바늘을 색이 아니라 다이얼 중심에서 뻗는 어두운 선의 각도로 찾거나, 압력계 영역을 확대해 판정하는 수정 필요.

## 공개 저장소 점검

- 16장 전부 검토. 식별 가능 얼굴 1장(SHT-01 bb, 칸막이 뒤 책상 남성 눈 부분) → 모자이크. EXT-03 bb·inspection 의 상단 2명(뒷모습·안경 조각)도 보수적으로 모자이크. SHT-01 inspection 은 오버레이가 사람을 가림. 그 외 몸통·팔 조각만. 모니터 글자는 판독 불가("SENTATION MATERIAL PLANNING" 벽 화면 제목 일부만). 비밀값 없음.

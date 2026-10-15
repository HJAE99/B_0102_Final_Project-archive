# victus/1015 — 관제 PC(Victus) 10/15 run 15 산출물

run 15: 21:09:03~21:19:34 KST, partial, 측정 AP iPhone, 섹터 Z1=robot4 · Z2=robot3. 통합본 a4b2af2 로 돌린 회차 (관제 Victus, robot4 는 robot4-pc, robot3 는 robot3 PC).

## inspections_r15/ — 설비 점검 근거 사진 (관제가 저장한 것 전부, 3장)

robot3 담당 4곳(EXT-02 · EXT-03 · RCK-02 · EXL-02)은 모두 normal 판정이라 관제에 사진이 저장되지 않았다 (이상·미확정만 저장).

| 파일 | 시각 | 로봇 | 설비 | 판정(final / detail) | 비고 |
|---|---|---|---|---|---|
| RCK-01_c8b590f2_1.jpg | 21:17:34 | robot4 | RCK-01 (rack) | unconfirmed / OCCLUDED | 6초 내 판정 결과 없음 (E6) |
| EXT-01_6a921459_1.jpg | 21:17:52 | robot4 | EXT-01 (extinguisher) | abnormal / ABNORMAL | 압력계 바늘이 정상 구간(녹색)에 없음 (yolo conf 0.95) |
| EXL-01_ff12ff73_1.jpg | 21:18:14 | robot4 | EXL-01 (exit_light) | abnormal / ABNORMAL | 유도등 OFF 밝기 근거 (yolo conf 0.95) |

## blackbox_r15/ — 블랙박스 프레임 (이 커밋에는 **올리지 않음**, 아래 점검 참조). 파일명 = 촬영 시각 epoch ms

| 로봇 | 장수 | 첫 프레임 | 마지막 프레임 | 간격 |
|---|---|---|---|---|
| robot3 | 507 (12.8 MB) | 1792066155402 (21:09:15) | 1792066751817 (21:19:11) | 중간값 1.10 s, 최대 55.2 s |
| robot4 | 503 (12.2 MB) | 1792066156056 (21:09:16) | 1792066743764 (21:19:03) | 중간값 1.10 s, 최대 38.0 s |

mp4 로 묶을 때: `ffmpeg -framerate 1 -pattern_type glob -i 'robot3/*.jpg' -c:v libx264 -pix_fmt yuv420p robot3_r15.mp4` (실시간 간격은 약 1.1 s 라 1 fps 가 거의 실속도).

## 공개 저장소 점검

- 블랙박스 1010장 중 20장 간격 표본 52장 + 점검 사진 3장 = 55장을 눈으로 검토: 사람 등장 28장(51 %), **얼굴이 또렷하게 식별되는 프레임 3장**(robot3 1792066219502 — 6명 정면, robot3 1792066724117, robot4 1792066330956), 경계 4장. 전체로 환산하면 식별 가능 얼굴 프레임이 수십 장이라 공개 저장소에는 올리지 않고 Victus 에 보관(scv_media/recordings/r15/). LAN 전달(scp) 또는 얼굴 블러 뒤 올릴 것.
- 점검 사진 3장: EXL-01 · EXT-01 사람 없음, RCK-01 은 우상단에 사람 몸통만(머리 잘림) → 올림.
- 목걸이 명찰 · 모니터 화면은 글자 판독 불가 수준.
비밀값(토큰 · 비번 · DSN) 없음 — 이미지 파일만.

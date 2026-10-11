-- =====================================================================
-- SCV 운영 DB 초기값 (2차)
--   01_schema.sql 다음에 실행. 여러 번 실행해도 값이 겹치지 않음(ON CONFLICT).
--   이미 있는 설정값은 덮어쓰지 않음 — 운영 중 admin이 바꾼 값을 보존.
-- =====================================================================

BEGIN;

-- 비밀번호 해시(bcrypt)용. postgres:16 공식 이미지에 포함된 확장
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- ---------------------------------------------------------------------
-- 로봇 2대 (ROS 이름공간 /robot3, /robot4)
--   역할(1차 순찰 / 재검증)은 순찰 시작 때 task를 만들며 정해지므로 여기에는 두지 않음
-- ---------------------------------------------------------------------
INSERT INTO robots (robot_id, robot_pc_host, rpi_host) VALUES
    ('robot3', NULL, NULL),
    ('robot4', NULL, NULL)
ON CONFLICT (robot_id) DO NOTHING;

-- ---------------------------------------------------------------------
-- 판정 기준값 (문서의 「제안」 값 — 첫 측정 뒤 server page 설정 탭에서 조정)
-- ---------------------------------------------------------------------
INSERT INTO settings (key, value, unit, source) VALUES
    -- 로봇이 실시간 판단에 쓰는 값 (순찰 시작 때 두 제어 PC로 전달)
    ('rssi_suspect_dbm',     '-72',  'dBm', 'BR-T04'),   -- 칸 중앙값이 이 값 미만이면 후보, 재검증도 같은 기준
    ('min_samples',          '3',    NULL,  'SR-06'),    -- 후보 판단 최소 표본 수 (보고서의 '낮음' 신뢰도 기준과 같음)
    ('precise_duration_s',   '30',   's',   'SR-08'),    -- 재검증 정지 측정 시간
    ('max_pose_retry',       '3',    NULL,  'SR-21'),    -- 설비 정면 자세 보정 최대 횟수
    ('battery_return_pct',   '15',   '%',   'E2'),       -- 이 값 이하이면 복귀(E2)
    -- 경로 생성 (inspection_manager)
    ('path_spacing_m',       '2.0',  'm',   'BR-T01'),   -- 순찰 왕복 경로 간격
    ('wall_margin_m',        '0.3',  'm',   'SR-05'),    -- 벽에서 띄우는 거리
    ('segment_points',       '20',   NULL,  'SR-05'),    -- SURVEY_SEGMENT 한 구간의 점 수
    -- 관제 PC
    ('status_timeout_s',     '30',   's',   'E3'),       -- 상태가 이 시간 동안 안 오면 E3 (Redis robot:{rid}:state TTL과 같음)
    -- 보고서 (실시간 판단에는 쓰지 않음)
    ('baseline_runs_min',    '3',    NULL,  'SR-07'),    -- 기준선을 만들 최소 정상 회차 수
    ('baseline_sigma',       '2',    NULL,  'SR-07'),    -- 기준선 이탈: 평균 − kσ
    ('baseline_drop_db',     '6',    'dB',  'SR-07'),    -- 기준선 이탈: 평균 − N dB (둘 중 하나로 통일)
    ('photo_retention_days', '30',   'day', 'BR-R05')    -- 정상 판정 사진 보관 기간
ON CONFLICT (key) DO NOTHING;

-- ---------------------------------------------------------------------
-- 초기 관리자 계정 (시연: admin 1 + viewer 1)
--   비밀번호는 bcrypt 해시로만 저장. 아래 초기 비밀번호는 첫 로그인 뒤 반드시 변경.
--   api(Python bcrypt 등)에서 그대로 검증 가능한 $2a$ 형식
-- ---------------------------------------------------------------------
INSERT INTO users (username, password_hash, role) VALUES
    ('admin',  crypt('admin1234!',  gen_salt('bf', 12)), 'admin'),
    ('viewer', crypt('viewer1234!', gen_salt('bf', 12)), 'viewer')
ON CONFLICT (username) DO NOTHING;

-- ---------------------------------------------------------------------
-- 측정 대상 AP — SSID를 실제 값으로 바꾼 뒤 주석을 풀어 실행
-- ---------------------------------------------------------------------
-- INSERT INTO aps (ssid, note) VALUES ('SCV_HOTSPOT_A', '핫스팟 A')
-- ON CONFLICT (ssid) DO NOTHING;

-- 지도(maps)·격자(grid_cells)·설비(facilities)는 SLAM 지도를 만든 뒤
-- server page 설정 탭(또는 api 등록 스크립트)에서 등록 — 예시는 03_verify.sql 참고

COMMIT;

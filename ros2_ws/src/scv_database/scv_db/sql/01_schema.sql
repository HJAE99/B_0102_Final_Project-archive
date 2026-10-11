-- =====================================================================
-- SCV 운영 DB 스키마 (2차)
--   기준: SCV_DB구조도_2차.drawio · DB시스템제안설명_2차.docx · DB시스템구현방안_2차.docx(통신 구조 안 C)
--   대상: PostgreSQL 16 (Docker 컨테이너 robotdb4_postgresql, DB robotdb4_postgresql)
--   테이블 20개 · 6개 도메인
--     ① 기준 정보   robots · users · settings · aps · maps · grid_cells · facilities
--     ② 순찰·작업   runs · tasks
--     ③ 통신 인프라 net_samples · precise_measures · grid_stats · baselines
--     ④ 물리 인프라 inspections · media
--     ⑤ 이상 후보   candidates · findings
--     ⑥ 이벤트·이력 robot_events · robot_status_log · reports
--   시각은 모두 TIMESTAMPTZ(UTC 저장, 화면에서 KST 표시), x·y는 map 좌표(m)
--   여러 번 실행해도 되도록 IF NOT EXISTS / CREATE OR REPLACE 사용
--   재전송 중복 방지 키(안 C 아웃박스): net_samples PK · robot_events.event_uid · inspections.inspection_uid ·
--     precise_measures (task_id, ap_id) · media.object_key · robot_status_log PK
--   이전 01_schema.sql로 만든 DB는 파일 끝의 「업그레이드」 구간이 새 키를 추가함
-- =====================================================================

BEGIN;

SET client_min_messages = warning;
SET TIME ZONE 'UTC';

-- ---------------------------------------------------------------------
-- ① 기준 정보
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS robots (
    robot_id        VARCHAR(16)  PRIMARY KEY
                    CONSTRAINT robots_robot_id_format CHECK (robot_id ~ '^[a-z][a-z0-9_]*$'),
    robot_pc_host   VARCHAR(64),
    rpi_host        VARCHAR(64),
    is_active       BOOLEAN      NOT NULL DEFAULT true,
    registered_at   TIMESTAMPTZ  NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS users (
    user_id         SERIAL       PRIMARY KEY,
    username        VARCHAR(32)  NOT NULL UNIQUE,
    password_hash   TEXT         NOT NULL,
    role            VARCHAR(8)   NOT NULL CHECK (role IN ('admin', 'viewer')),
    created_at      TIMESTAMPTZ  NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS settings (
    key             VARCHAR(64)  PRIMARY KEY,
    value           TEXT         NOT NULL,
    unit            VARCHAR(16),
    source          VARCHAR(32),
    updated_by      INT          REFERENCES users (user_id),
    updated_at      TIMESTAMPTZ  NOT NULL DEFAULT now()
);

CREATE TABLE IF NOT EXISTS aps (
    ap_id           SMALLSERIAL  PRIMARY KEY,
    ssid            VARCHAR(64)  NOT NULL UNIQUE,
    note            TEXT,
    is_active       BOOLEAN      NOT NULL DEFAULT true
);

CREATE TABLE IF NOT EXISTS maps (
    map_id          SERIAL       PRIMARY KEY,
    name            VARCHAR(64)  NOT NULL,
    robot_map_path  TEXT         NOT NULL UNIQUE,
    image_key       TEXT         NOT NULL,
    resolution_m    REAL         NOT NULL CHECK (resolution_m > 0),
    origin_x        REAL         NOT NULL,
    origin_y        REAL         NOT NULL,
    grid_size_m     REAL         NOT NULL DEFAULT 1.0 CHECK (grid_size_m > 0),
    is_active       BOOLEAN      NOT NULL DEFAULT false,
    created_at      TIMESTAMPTZ  NOT NULL DEFAULT now()
);
-- 현재 쓰는 지도는 하나만
CREATE UNIQUE INDEX IF NOT EXISTS maps_one_active ON maps ((true)) WHERE is_active;

CREATE TABLE IF NOT EXISTS grid_cells (
    cell_id         SERIAL       PRIMARY KEY,
    map_id          INT          NOT NULL REFERENCES maps (map_id),
    ix              INT          NOT NULL,
    iy              INT          NOT NULL,
    center_x        REAL         NOT NULL,
    center_y        REAL         NOT NULL,
    CONSTRAINT grid_cells_map_ix_iy_key UNIQUE (map_id, ix, iy)
);

CREATE TABLE IF NOT EXISTS facilities (
    facility_id     VARCHAR(32)  PRIMARY KEY,
    map_id          INT          NOT NULL REFERENCES maps (map_id),
    facility_type   VARCHAR(16)  NOT NULL CHECK (facility_type IN ('extinguisher', 'shutter', 'exit_light')),
    has_gauge       BOOLEAN      NOT NULL DEFAULT false,
    expected_x      REAL         NOT NULL,
    expected_y      REAL         NOT NULL,
    view_pose       JSONB        NOT NULL,
    is_active       BOOLEAN      NOT NULL DEFAULT true
);

-- ---------------------------------------------------------------------
-- ② 순찰·작업
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS runs (
    run_id          SERIAL       PRIMARY KEY,
    map_id          INT          NOT NULL REFERENCES maps (map_id),
    run_type        VARCHAR(12)  NOT NULL CHECK (run_type IN ('patrol', 'remeasure', 'demo')),
    status          VARCHAR(10)  NOT NULL DEFAULT 'running' CHECK (status IN ('running', 'done', 'aborted')),
    is_baseline_ok  BOOLEAN      NOT NULL DEFAULT false,
    started_by      INT          REFERENCES users (user_id),
    started_at      TIMESTAMPTZ  NOT NULL DEFAULT now(),
    ended_at        TIMESTAMPTZ,
    CONSTRAINT runs_ended_when_finished CHECK ((status = 'running') = (ended_at IS NULL)),
    CONSTRAINT runs_baseline_only_done  CHECK (NOT is_baseline_ok OR status = 'done')
);
-- 진행 중인 run은 하나만 (start_run 중복 방지)
CREATE UNIQUE INDEX IF NOT EXISTS runs_one_running ON runs ((true)) WHERE status = 'running';

CREATE TABLE IF NOT EXISTS tasks (
    task_id         VARCHAR(40)  PRIMARY KEY,
    run_id          INT          NOT NULL REFERENCES runs (run_id),
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    type            VARCHAR(16)  NOT NULL CHECK (type IN ('SURVEY_SEGMENT', 'FACILITY_CHECK', 'REINSPECTION', 'DOCK')),
    target_ref      VARCHAR(40),
    waypoints       JSONB        NOT NULL DEFAULT '[]'::jsonb,
    status          VARCHAR(10)  NOT NULL DEFAULT 'PENDING'
                    CHECK (status IN ('PENDING', 'ACTIVE', 'DONE', 'FAILED', 'CANCELED')),
    progress        REAL         NOT NULL DEFAULT 0 CHECK (progress BETWEEN 0 AND 1),
    result_ok       BOOLEAN,
    reason          TEXT,
    assigned_at     TIMESTAMPTZ,
    ended_at        TIMESTAMPTZ,
    CONSTRAINT tasks_target_required CHECK (type = 'DOCK' OR target_ref IS NOT NULL),
    CONSTRAINT tasks_ended_when_finished CHECK ((status IN ('DONE', 'FAILED', 'CANCELED')) = (ended_at IS NOT NULL))
);
CREATE INDEX IF NOT EXISTS tasks_run_robot_idx ON tasks (run_id, robot_id, task_id);
-- 로봇마다 ACTIVE task는 하나
CREATE UNIQUE INDEX IF NOT EXISTS tasks_one_active_per_robot ON tasks (robot_id) WHERE status = 'ACTIVE';

-- ---------------------------------------------------------------------
-- ③ 통신 인프라 · RSSI
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS net_samples (
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    run_id          INT          NOT NULL REFERENCES runs (run_id),
    seq             BIGINT       NOT NULL CHECK (seq >= 0),
    stamp           TIMESTAMPTZ  NOT NULL,
    x               REAL         NOT NULL,
    y               REAL         NOT NULL,
    yaw             REAL         NOT NULL,
    pose_ok         BOOLEAN      NOT NULL,
    ap_id           SMALLINT     NOT NULL REFERENCES aps (ap_id),
    rssi_dbm        SMALLINT     NOT NULL CHECK (rssi_dbm BETWEEN -127 AND 0),
    cell_id         INT          REFERENCES grid_cells (cell_id),
    received_at     TIMESTAMPTZ  NOT NULL DEFAULT now(),
    PRIMARY KEY (robot_id, run_id, seq)
);
CREATE INDEX IF NOT EXISTS net_samples_run_cell_idx   ON net_samples (run_id, cell_id);
CREATE INDEX IF NOT EXISTS net_samples_robot_stamp_idx ON net_samples (robot_id, stamp);

CREATE TABLE IF NOT EXISTS precise_measures (
    measure_id      BIGSERIAL    PRIMARY KEY,
    task_id         VARCHAR(40)  NOT NULL REFERENCES tasks (task_id),
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    ap_id           SMALLINT     NOT NULL REFERENCES aps (ap_id),
    cell_id         INT          NOT NULL REFERENCES grid_cells (cell_id),
    duration_s      SMALLINT     NOT NULL CHECK (duration_s > 0),
    n_samples       INT          NOT NULL CHECK (n_samples >= 0),
    rssi_median     REAL         NOT NULL,
    rssi_iqr        REAL         NOT NULL CHECK (rssi_iqr >= 0),
    measured_at     TIMESTAMPTZ  NOT NULL,
    CONSTRAINT precise_measures_task_ap_key UNIQUE (task_id, ap_id)
);
CREATE INDEX IF NOT EXISTS precise_measures_task_idx ON precise_measures (task_id);

CREATE TABLE IF NOT EXISTS grid_stats (
    run_id          INT          NOT NULL REFERENCES runs (run_id),
    cell_id         INT          NOT NULL REFERENCES grid_cells (cell_id),
    ap_id           SMALLINT     NOT NULL REFERENCES aps (ap_id),
    n               INT          NOT NULL CHECK (n >= 0),
    mean            REAL,
    median          REAL,
    var             REAL,
    min             SMALLINT,
    max             SMALLINT,
    confidence      VARCHAR(4)   NOT NULL CHECK (confidence IN ('low', 'ok')),
    interpolated    BOOLEAN      NOT NULL DEFAULT false,
    state           VARCHAR(16)  NOT NULL CHECK (state IN ('normal', 'coverage_suspect', 'unknown')),
    baseline_delta  REAL,
    computed_at     TIMESTAMPTZ  NOT NULL DEFAULT now(),
    PRIMARY KEY (run_id, cell_id, ap_id)
);

CREATE TABLE IF NOT EXISTS baselines (
    baseline_id     SERIAL       PRIMARY KEY,
    cell_id         INT          NOT NULL REFERENCES grid_cells (cell_id),
    ap_id           SMALLINT     NOT NULL REFERENCES aps (ap_id),
    version         INT          NOT NULL CHECK (version >= 1),
    run_ids         INT[]        NOT NULL,
    mean            REAL         NOT NULL,
    var             REAL         NOT NULL CHECK (var >= 0),
    n               INT          NOT NULL CHECK (n > 0),
    built_at        TIMESTAMPTZ  NOT NULL DEFAULT now(),
    is_current      BOOLEAN      NOT NULL DEFAULT true,
    CONSTRAINT baselines_cell_ap_version_key UNIQUE (cell_id, ap_id, version)
);
-- 칸·AP마다 현재 기준선은 하나
CREATE UNIQUE INDEX IF NOT EXISTS baselines_one_current ON baselines (cell_id, ap_id) WHERE is_current;

-- ---------------------------------------------------------------------
-- ④ 물리 인프라 · 설비 점검
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS inspections (
    inspection_id   BIGSERIAL    PRIMARY KEY,
    inspection_uid  UUID         NOT NULL
                    CONSTRAINT inspections_inspection_uid_key UNIQUE,
    task_id         VARCHAR(40)  NOT NULL REFERENCES tasks (task_id),
    run_id          INT          NOT NULL REFERENCES runs (run_id),
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    facility_id     VARCHAR(32)  NOT NULL REFERENCES facilities (facility_id),
    detail          VARCHAR(20)  NOT NULL
                    CHECK (detail IN ('NORMAL', 'NOT_DETECTED', 'OCCLUDED', 'POSITION_ANOMALY', 'ABNORMAL')),
    final           VARCHAR(12)  NOT NULL CHECK (final IN ('normal', 'abnormal', 'unconfirmed')),
    frontal         BOOLEAN      NOT NULL,
    pose_retry      SMALLINT     NOT NULL DEFAULT 0 CHECK (pose_retry BETWEEN 0 AND 3),
    gauge_state     VARCHAR(8)   CHECK (gauge_state IN ('green', 'abnormal')),
    confidence      REAL         CHECK (confidence BETWEEN 0 AND 1),
    bbox            JSONB,
    model_version   VARCHAR(32)  NOT NULL,
    inspected_at    TIMESTAMPTZ  NOT NULL,
    CONSTRAINT inspections_gauge_needs_frontal CHECK (gauge_state IS NULL OR frontal)
);
CREATE INDEX IF NOT EXISTS inspections_facility_time_idx ON inspections (facility_id, inspected_at);
CREATE INDEX IF NOT EXISTS inspections_task_idx ON inspections (task_id);

CREATE TABLE IF NOT EXISTS media (
    media_id        BIGSERIAL    PRIMARY KEY,
    inspection_id   BIGINT       NOT NULL REFERENCES inspections (inspection_id) ON DELETE CASCADE,
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    kind            VARCHAR(6)   NOT NULL CHECK (kind IN ('rgb', 'depth')),
    object_key      TEXT         NOT NULL UNIQUE,
    sha256          CHAR(64)     NOT NULL,
    size_bytes      INT          NOT NULL CHECK (size_bytes >= 0),
    captured_at     TIMESTAMPTZ  NOT NULL
);
CREATE INDEX IF NOT EXISTS media_inspection_idx ON media (inspection_id);

-- ---------------------------------------------------------------------
-- ⑤ 이상 후보 → 확정
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS candidates (
    candidate_id      VARCHAR(40)  PRIMARY KEY,
    run_id            INT          NOT NULL REFERENCES runs (run_id),
    source            VARCHAR(8)   NOT NULL CHECK (source IN ('NETWORK', 'VISION')),
    kind              VARCHAR(20)  NOT NULL
                      CHECK (kind IN ('NET_COVERAGE', 'VIS_NOT_DETECTED', 'VIS_OCCLUDED', 'VIS_POSITION', 'VIS_ABNORMAL')),
    ref               VARCHAR(40)  NOT NULL,
    target_x          REAL         NOT NULL,
    target_y          REAL         NOT NULL,
    score             REAL,
    detect_task_id    VARCHAR(40)  NOT NULL REFERENCES tasks (task_id),
    detected_by       VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    status            VARCHAR(12)  NOT NULL DEFAULT 'OPEN' CHECK (status IN ('OPEN', 'REINSPECTING', 'CLOSED')),
    reinspect_task_id VARCHAR(40)  REFERENCES tasks (task_id),
    outcome           VARCHAR(12)  CHECK (outcome IN ('CONFIRMED', 'CLEARED', 'UNCONFIRMED')),
    detected_at       TIMESTAMPTZ  NOT NULL,
    decided_at        TIMESTAMPTZ,
    CONSTRAINT candidates_run_ref_key UNIQUE (run_id, ref),
    CONSTRAINT candidates_source_kind CHECK ((source = 'NETWORK') = (kind = 'NET_COVERAGE')),
    CONSTRAINT candidates_reinspecting_has_task CHECK (status = 'OPEN' OR reinspect_task_id IS NOT NULL),
    CONSTRAINT candidates_closed_has_outcome CHECK ((status = 'CLOSED') = (outcome IS NOT NULL)
                                                   AND (outcome IS NULL) = (decided_at IS NULL))
);
CREATE INDEX IF NOT EXISTS candidates_open_idx ON candidates (status) WHERE status <> 'CLOSED';
CREATE UNIQUE INDEX IF NOT EXISTS candidates_reinspect_task_key ON candidates (reinspect_task_id) WHERE reinspect_task_id IS NOT NULL;

CREATE TABLE IF NOT EXISTS findings (
    finding_id      BIGSERIAL    PRIMARY KEY,
    candidate_id    VARCHAR(40)  NOT NULL UNIQUE REFERENCES candidates (candidate_id),
    task_id         VARCHAR(40)  NOT NULL REFERENCES tasks (task_id),
    category        VARCHAR(16)  NOT NULL,
    status          VARCHAR(8)   NOT NULL DEFAULT 'open' CHECK (status IN ('open', 'acked')),
    acked_by        INT          REFERENCES users (user_id),
    confirmed_at    TIMESTAMPTZ  NOT NULL DEFAULT now(),
    CONSTRAINT findings_acked_has_user CHECK (status = 'open' OR acked_by IS NOT NULL)
);
CREATE INDEX IF NOT EXISTS findings_open_idx ON findings (status) WHERE status = 'open';

-- ---------------------------------------------------------------------
-- ⑥ 이벤트·이력·보고
-- ---------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS robot_events (
    event_id        BIGSERIAL    PRIMARY KEY,
    event_uid       VARCHAR(64)  NOT NULL
                    CONSTRAINT robot_events_event_uid_key UNIQUE,
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    run_id          INT          REFERENCES runs (run_id),
    task_id         VARCHAR(40)  REFERENCES tasks (task_id),
    code            VARCHAR(16)  NOT NULL CHECK (code IN (
                        'E1', 'E2', 'E3', 'E4', 'E5', 'E6', 'E7', 'E8',
                        'LOCAL_ESTOP', 'MAP_MISMATCH', 'UNDOCK', 'DOCK',
                        'CANDIDATE', 'CONFIRMED', 'CLEARED', 'UNCONFIRMED')),
    source          VARCHAR(8)   NOT NULL DEFAULT 'robot' CHECK (source IN ('robot', 'control')),
    level           VARCHAR(8)   NOT NULL DEFAULT 'info' CHECK (level IN ('info', 'warn', 'error')),
    detail          JSONB,
    ts              TIMESTAMPTZ  NOT NULL,
    CONSTRAINT robot_events_anomaly_has_task CHECK (
        code NOT IN ('CANDIDATE', 'CONFIRMED', 'CLEARED', 'UNCONFIRMED') OR task_id IS NOT NULL),
    CONSTRAINT robot_events_control_codes CHECK (source = 'robot' OR code IN ('E3', 'E8'))
);
CREATE INDEX IF NOT EXISTS robot_events_ts_idx  ON robot_events (ts DESC);
CREATE INDEX IF NOT EXISTS robot_events_run_idx ON robot_events (run_id, ts);

CREATE TABLE IF NOT EXISTS robot_status_log (
    robot_id        VARCHAR(16)  NOT NULL REFERENCES robots (robot_id),
    ts              TIMESTAMPTZ  NOT NULL,
    run_id          INT          REFERENCES runs (run_id),
    task_id         VARCHAR(40)  REFERENCES tasks (task_id),
    x               REAL         NOT NULL,
    y               REAL         NOT NULL,
    yaw             REAL         NOT NULL,
    battery_pct     REAL         NOT NULL CHECK (battery_pct BETWEEN 0 AND 100),
    nav_state       VARCHAR(16)  NOT NULL,
    progress        REAL         CHECK (progress BETWEEN 0 AND 1),
    PRIMARY KEY (robot_id, ts)
);
CREATE INDEX IF NOT EXISTS robot_status_log_run_idx ON robot_status_log (run_id, ts);

CREATE TABLE IF NOT EXISTS reports (
    report_id       BIGSERIAL    PRIMARY KEY,
    run_id          INT          NOT NULL REFERENCES runs (run_id),
    kind            VARCHAR(16)  NOT NULL CHECK (kind IN ('network_report', 'checklist')),
    file_key        TEXT         NOT NULL,
    summary         JSONB,
    generated_at    TIMESTAMPTZ  NOT NULL DEFAULT now(),
    CONSTRAINT reports_run_kind_key UNIQUE (run_id, kind)
);

-- =====================================================================
-- 트리거 — 여러 테이블을 비교하는 규칙 (CHECK로는 표현 불가)
-- =====================================================================

-- [규칙 A] 후보를 발견한 task: 같은 run, 1차 task(SURVEY_SEGMENT·FACILITY_CHECK), 발견 로봇 = 그 task의 로봇
-- [규칙 B] 재검증 task: 같은 run, REINSPECTION, 재검증 로봇 ≠ 발견 로봇 (C4), task.target_ref = candidate_id
CREATE OR REPLACE FUNCTION scv_check_candidate() RETURNS trigger
LANGUAGE plpgsql AS $$
DECLARE
    t RECORD;
BEGIN
    SELECT run_id, robot_id, type INTO t FROM tasks WHERE task_id = NEW.detect_task_id;
    IF t.run_id <> NEW.run_id THEN
        RAISE EXCEPTION 'candidates %: detect_task_id %의 run(%)이 후보의 run(%)과 다름',
            NEW.candidate_id, NEW.detect_task_id, t.run_id, NEW.run_id;
    END IF;
    IF t.type NOT IN ('SURVEY_SEGMENT', 'FACILITY_CHECK') THEN
        RAISE EXCEPTION 'candidates %: detect_task_id %는 1차 task가 아님(type=%)',
            NEW.candidate_id, NEW.detect_task_id, t.type;
    END IF;
    IF t.robot_id <> NEW.detected_by THEN
        RAISE EXCEPTION 'candidates %: detected_by(%)가 발견 task의 로봇(%)과 다름',
            NEW.candidate_id, NEW.detected_by, t.robot_id;
    END IF;

    IF NEW.reinspect_task_id IS NOT NULL THEN
        SELECT run_id, robot_id, type, target_ref INTO t FROM tasks WHERE task_id = NEW.reinspect_task_id;
        IF t.type <> 'REINSPECTION' THEN
            RAISE EXCEPTION 'candidates %: reinspect_task_id %는 REINSPECTION이 아님(type=%)',
                NEW.candidate_id, NEW.reinspect_task_id, t.type;
        END IF;
        IF t.run_id <> NEW.run_id THEN
            RAISE EXCEPTION 'candidates %: 재검증 task의 run(%)이 후보의 run(%)과 다름',
                NEW.candidate_id, t.run_id, NEW.run_id;
        END IF;
        IF t.target_ref IS DISTINCT FROM NEW.candidate_id THEN
            RAISE EXCEPTION 'candidates %: 재검증 task %의 target_ref(%)가 candidate_id와 다름',
                NEW.candidate_id, NEW.reinspect_task_id, t.target_ref;
        END IF;
        IF t.robot_id = NEW.detected_by THEN
            RAISE EXCEPTION 'candidates %: 재검증 로봇(%)이 발견 로봇과 같음 — 다른 로봇이 재검증해야 함',
                NEW.candidate_id, t.robot_id;
        END IF;
    END IF;
    RETURN NEW;
END $$;

DROP TRIGGER IF EXISTS trg_candidates_check ON candidates;
CREATE TRIGGER trg_candidates_check
    BEFORE INSERT OR UPDATE ON candidates
    FOR EACH ROW EXECUTE FUNCTION scv_check_candidate();

-- [규칙 C] 재검증 없이 확정 0건 (BR-T06·R02):
--   findings.task_id = 그 후보의 reinspect_task_id 이고 후보 outcome = CONFIRMED 여야 INSERT/UPDATE 가능
CREATE OR REPLACE FUNCTION scv_check_finding() RETURNS trigger
LANGUAGE plpgsql AS $$
DECLARE
    c RECORD;
BEGIN
    SELECT reinspect_task_id, outcome INTO c FROM candidates WHERE candidate_id = NEW.candidate_id;
    IF c.outcome IS DISTINCT FROM 'CONFIRMED' THEN
        RAISE EXCEPTION 'findings: 후보 %의 결론이 CONFIRMED가 아님(outcome=%) — 재검증 없이 확정할 수 없음',
            NEW.candidate_id, c.outcome;
    END IF;
    IF c.reinspect_task_id IS DISTINCT FROM NEW.task_id THEN
        RAISE EXCEPTION 'findings: task_id %가 후보 %의 재검증 task(%)와 다름',
            NEW.task_id, NEW.candidate_id, c.reinspect_task_id;
    END IF;
    RETURN NEW;
END $$;

DROP TRIGGER IF EXISTS trg_findings_check ON findings;
CREATE TRIGGER trg_findings_check
    BEFORE INSERT OR UPDATE OF candidate_id, task_id ON findings
    FOR EACH ROW EXECUTE FUNCTION scv_check_finding();

-- 확정된 후보의 결론을 나중에 바꾸지 못하게 (findings와 어긋남 방지)
CREATE OR REPLACE FUNCTION scv_lock_confirmed_candidate() RETURNS trigger
LANGUAGE plpgsql AS $$
BEGIN
    IF OLD.outcome = 'CONFIRMED' AND NEW.outcome IS DISTINCT FROM OLD.outcome
       AND EXISTS (SELECT 1 FROM findings f WHERE f.candidate_id = OLD.candidate_id) THEN
        RAISE EXCEPTION 'candidates %: 이미 findings가 있는 확정 후보의 결론은 바꿀 수 없음', OLD.candidate_id;
    END IF;
    RETURN NEW;
END $$;

DROP TRIGGER IF EXISTS trg_candidates_lock_confirmed ON candidates;
CREATE TRIGGER trg_candidates_lock_confirmed
    BEFORE UPDATE OF outcome ON candidates
    FOR EACH ROW EXECUTE FUNCTION scv_lock_confirmed_candidate();

-- [규칙 D] 판정·측정·이벤트가 가리키는 task의 로봇·run이 맞는지
CREATE OR REPLACE FUNCTION scv_check_task_robot() RETURNS trigger
LANGUAGE plpgsql AS $$
DECLARE
    t RECORD;
BEGIN
    IF NEW.task_id IS NULL THEN
        RETURN NEW;
    END IF;
    SELECT run_id, robot_id, type INTO t FROM tasks WHERE task_id = NEW.task_id;
    IF t.robot_id <> NEW.robot_id THEN
        RAISE EXCEPTION '%: robot_id(%)가 task %의 로봇(%)과 다름', TG_TABLE_NAME, NEW.robot_id, NEW.task_id, t.robot_id;
    END IF;
    IF TG_TABLE_NAME = 'precise_measures' AND t.type <> 'REINSPECTION' THEN
        RAISE EXCEPTION 'precise_measures: task %는 REINSPECTION이 아님(type=%)', NEW.task_id, t.type;
    END IF;
    IF TG_TABLE_NAME = 'inspections' THEN
        IF t.type NOT IN ('FACILITY_CHECK', 'REINSPECTION') THEN
            RAISE EXCEPTION 'inspections: task %는 FACILITY_CHECK·REINSPECTION이 아님(type=%)', NEW.task_id, t.type;
        END IF;
        IF t.run_id <> NEW.run_id THEN
            RAISE EXCEPTION 'inspections: run_id(%)가 task %의 run(%)과 다름', NEW.run_id, NEW.task_id, t.run_id;
        END IF;
    END IF;
    RETURN NEW;
END $$;

DROP TRIGGER IF EXISTS trg_precise_measures_check ON precise_measures;
CREATE TRIGGER trg_precise_measures_check
    BEFORE INSERT OR UPDATE ON precise_measures
    FOR EACH ROW EXECUTE FUNCTION scv_check_task_robot();

DROP TRIGGER IF EXISTS trg_inspections_check ON inspections;
CREATE TRIGGER trg_inspections_check
    BEFORE INSERT OR UPDATE ON inspections
    FOR EACH ROW EXECUTE FUNCTION scv_check_task_robot();

DROP TRIGGER IF EXISTS trg_robot_events_check ON robot_events;
CREATE TRIGGER trg_robot_events_check
    BEFORE INSERT OR UPDATE ON robot_events
    FOR EACH ROW EXECUTE FUNCTION scv_check_task_robot();

-- settings.updated_at 자동 갱신
CREATE OR REPLACE FUNCTION scv_touch_updated_at() RETURNS trigger
LANGUAGE plpgsql AS $$
BEGIN
    NEW.updated_at := now();
    RETURN NEW;
END $$;

DROP TRIGGER IF EXISTS trg_settings_touch ON settings;
CREATE TRIGGER trg_settings_touch
    BEFORE UPDATE ON settings
    FOR EACH ROW EXECUTE FUNCTION scv_touch_updated_at();

-- =====================================================================
-- 실패 처리 도우미 — task 하나가 FAILED면 run aborted + 남은 task CANCELED (한 트랜잭션)
--   사용: SELECT scv_fail_task('r12-t003', 'BLOCKED');
-- =====================================================================
CREATE OR REPLACE FUNCTION scv_fail_task(p_task_id VARCHAR, p_reason TEXT) RETURNS INT
LANGUAGE plpgsql AS $$
DECLARE
    v_run INT;
    v_canceled INT;
BEGIN
    UPDATE tasks
       SET status = 'FAILED', result_ok = false, reason = p_reason, ended_at = now()
     WHERE task_id = p_task_id AND status IN ('PENDING', 'ACTIVE')
    RETURNING run_id INTO v_run;
    IF v_run IS NULL THEN
        RAISE EXCEPTION 'scv_fail_task: task %가 없거나 이미 끝남', p_task_id;
    END IF;

    UPDATE tasks
       SET status = 'CANCELED', reason = 'RUN_ABORTED:' || p_task_id, ended_at = now()
     WHERE run_id = v_run AND status IN ('PENDING', 'ACTIVE');
    GET DIAGNOSTICS v_canceled = ROW_COUNT;

    UPDATE runs SET status = 'aborted', is_baseline_ok = false, ended_at = now()
     WHERE run_id = v_run AND status = 'running';
    RETURN v_canceled;
END $$;

-- =====================================================================
-- 업그레이드 — 이전 01_schema.sql(중복 방지 키 추가 전)로 만든 DB에 새 키를 더함
--   새로 만든 DB에서는 이미 있으므로 아무것도 하지 않음
-- =====================================================================
ALTER TABLE robot_events ADD COLUMN IF NOT EXISTS event_uid VARCHAR(64);
UPDATE robot_events SET event_uid = 'legacy-' || event_id WHERE event_uid IS NULL;
ALTER TABLE robot_events ALTER COLUMN event_uid SET NOT NULL;

ALTER TABLE inspections ADD COLUMN IF NOT EXISTS inspection_uid UUID;
UPDATE inspections SET inspection_uid = gen_random_uuid() WHERE inspection_uid IS NULL;
ALTER TABLE inspections ALTER COLUMN inspection_uid SET NOT NULL;

DO $$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_constraint WHERE conname = 'robot_events_event_uid_key') THEN
        ALTER TABLE robot_events ADD CONSTRAINT robot_events_event_uid_key UNIQUE (event_uid);
    END IF;
    IF NOT EXISTS (SELECT 1 FROM pg_constraint WHERE conname = 'inspections_inspection_uid_key') THEN
        ALTER TABLE inspections ADD CONSTRAINT inspections_inspection_uid_key UNIQUE (inspection_uid);
    END IF;
    IF NOT EXISTS (SELECT 1 FROM pg_constraint WHERE conname = 'precise_measures_task_ap_key') THEN
        ALTER TABLE precise_measures ADD CONSTRAINT precise_measures_task_ap_key UNIQUE (task_id, ap_id);
    END IF;
END $$;

-- =====================================================================
-- 테이블·컬럼 설명 (psql \d+ 또는 DBeaver·pgAdmin에서 보임)
-- =====================================================================

COMMENT ON TABLE robots IS '로봇 — SCV에 등록된 로봇 목록(로봇마다 짝인 제어 PC 주소 포함). 다른 모든 운영 테이블이 robot_id로 이 표를 가리킵니다.';
COMMENT ON COLUMN robots.robot_id IS '로봇 식별자. ''robot3'', ''robot4''. ROS 이름공간은 ''/'' + robot_id로 만들어 쓰므로({ns} = ''/robot3'') 영문 소문자로 시작하고 하이픈을 쓰지 않음. NetSample.robot_id와 같은 값이어야 관제 PC api가 표본을 로봇에 연결';
COMMENT ON COLUMN robots.robot_pc_host IS '제어 PC 호스트명 또는 IP. client page 주소 안내, 장애 때 접속 확인용';
COMMENT ON COLUMN robots.rpi_host IS '로봇의 호스트명 또는 IP(컬럼 이름의 rpi = 로봇에 들어 있는 Raspberry Pi). 이 로봇의 Discovery Server 주소 확인용';
COMMENT ON COLUMN robots.is_active IS 'false면 배정 대상에서 빠짐. 이력이 참조하므로 행은 지우지 않음';
COMMENT ON COLUMN robots.registered_at IS '등록 시각';

COMMENT ON TABLE users IS 'server page 사용자 — server page 로그인 계정과 권한. 「누가 했는지」를 남기는 기준입니다.';
COMMENT ON COLUMN users.user_id IS '사용자 번호. 다른 표의 「누가」 컬럼이 참조';
COMMENT ON COLUMN users.username IS '로그인 ID';
COMMENT ON COLUMN users.password_hash IS 'bcrypt/argon2 해시. 평문 비밀번호는 저장하지 않음';
COMMENT ON COLUMN users.role IS 'admin = 순찰 시작·배정·정지·설정 변경, viewer = 조회만';
COMMENT ON COLUMN users.created_at IS '계정 생성 시각';

COMMENT ON TABLE settings IS '판정 기준값 (제안값 관리) — 원문 SR의 「제안」 수치를 코드 밖에 두는 키-값 표. 첫 측정 뒤 수치를 바꿔도 코드를 고치지 않아도 됩니다.';
COMMENT ON COLUMN settings.key IS '설정 이름. 예: rssi_suspect_dbm(후보·확정 기준 −72), min_samples(후보 판단 최소 표본 3), precise_duration_s(30), max_pose_retry(3), battery_return_pct(15), path_spacing_m(2.0), wall_margin_m(0.3). 판정에 쓰는 값은 순찰 시작 때 제어 PC로 전달';
COMMENT ON COLUMN settings.value IS '설정 값. 문자열로 저장하고 읽을 때 숫자로 변환';
COMMENT ON COLUMN settings.unit IS '단위. dBm, s, %, m 등. 단위 없는 값은 NULL';
COMMENT ON COLUMN settings.source IS '근거 문서 번호. 예: ''SR-12'', ''D6''';
COMMENT ON COLUMN settings.updated_by IS '마지막으로 바꾼 사람. 초기값(init 스크립트)은 NULL';
COMMENT ON COLUMN settings.updated_at IS '마지막 변경 시각';

COMMENT ON TABLE aps IS '측정 대상 AP (핫스팟) — 측정 대상 AP 목록. 시연에서는 휴대폰 핫스팟입니다.';
COMMENT ON COLUMN aps.ap_id IS 'AP 번호';
COMMENT ON COLUMN aps.ssid IS '로봇 rssi_scanner가 스캔 결과에서 고를 SSID. 로봇 rssi_scanner 설정과 같아야 함(run_config의 ap_ssid로 제어 PC에도 전달)';
COMMENT ON COLUMN aps.note IS '메모. 예: ''핫스팟 A — 사물함 뒤 설치''';
COMMENT ON COLUMN aps.is_active IS 'false면 측정 대상이 아님';

COMMENT ON TABLE maps IS '저장 지도 (SLAM 1회) — SLAM으로 한 번 만든 저장 지도의 버전 관리. 좌표가 들어간 모든 데이터는 이 지도 기준입니다.';
COMMENT ON COLUMN maps.map_id IS '지도 번호';
COMMENT ON COLUMN maps.name IS '지도 이름. 예: ''testbed_20261010'' (파일명과 맞추면 혼동이 적음)';
COMMENT ON COLUMN maps.robot_map_path IS '제어 PC에 있는 지도 yaml의 절대 경로. 예: ''/opt/scv/maps/testbed_20261010.yaml''. 작업에 실어 보냄. 파일명이 같으면 같은 지도로 간주하므로 지도를 다시 만들면 새 파일명·새 행으로 등록(덮어쓰기 금지)';
COMMENT ON COLUMN maps.image_key IS '관제 PC 표시용 지도 이미지의 scv_media 경로. 등록 때 pgm을 크기를 바꾸지 않고 PNG로 변환해 저장. server page 지도 배경·보고서 배경';
COMMENT ON COLUMN maps.resolution_m IS '지도 이미지 1픽셀의 실제 크기(m). yaml resolution 값이며, pgm → png 변환 시 크기를 바꾸지 않으므로 PNG에도 그대로 적용. 사용처는 origin_x와 같음';
COMMENT ON COLUMN maps.origin_x IS '지도 이미지 왼쪽 아래 모서리의 map 좌표 x(m). yaml origin[0]. origin[2](yaw)는 0이어야 하며 등록 때 확인. 사용처: ① server page에서 PNG를 m 좌표 위에 까는 범위 계산(실시간·과거 회차·로봇이 꺼져 있어도) ② 관리자가 지도에서 클릭한 픽셀 → m 변환(설비 위치·관측 자세 등록) ③ 보고서용 히트맵 이미지 합성 ④ 격자 칸 기준점(편의상 맞춤). 제어 PC의 좌표 계산에는 쓰지 않음(제어 PC map_server가 yaml로 처리)';
COMMENT ON COLUMN maps.origin_y IS '지도 이미지 왼쪽 아래 모서리의 map 좌표 y(m). yaml origin[1]. 이미지 픽셀 (col, row)은 위에서부터 세므로 y = origin_y + (이미지 높이 − row − 0.5) × resolution. 이미지 높이는 PNG를 불러올 때 알 수 있어 따로 저장하지 않음. 사용처는 origin_x와 같음';
COMMENT ON COLUMN maps.grid_size_m IS '격자 칸 크기(m). SR-06 제안 1 m, D6에서 확정';
COMMENT ON COLUMN maps.is_active IS '현재 쓰는 지도. 부분 유일 인덱스로 true는 하나만';
COMMENT ON COLUMN maps.created_at IS '등록 시각';

COMMENT ON TABLE grid_cells IS '격자 칸 (1 m) — 1 m 격자 칸 목록. 측정 표본·통계·기준선·후보·회피 구역이 모두 cell_id 하나로 이어지는 공통 키입니다.';
COMMENT ON COLUMN grid_cells.cell_id IS '칸 번호. 측정·통계·후보·회피가 함께 쓰는 키';
COMMENT ON COLUMN grid_cells.map_id IS '이 칸이 속한 지도';
COMMENT ON COLUMN grid_cells.ix IS 'x 방향 칸 번호 = floor((x − origin_x) / grid_size_m). 지도의 빈 공간(벽 여유 제외)에 걸치는 칸만 생성';
COMMENT ON COLUMN grid_cells.iy IS 'y 방향 칸 번호 = floor((y − origin_y) / grid_size_m)';
COMMENT ON COLUMN grid_cells.center_x IS '칸 중심의 map 좌표 x(m). 히트맵 그리기·재방문 지점';
COMMENT ON COLUMN grid_cells.center_y IS '칸 중심의 map 좌표 y(m)';

COMMENT ON TABLE facilities IS '점검 설비 (YAML 등록) — 점검 설비 등록부. 원문의 facilities YAML을 DB로 옮긴 것입니다.';
COMMENT ON COLUMN facilities.facility_id IS '설비 ID. 예: ''EXT-01''. Candidate.ref에 그대로 들어가는 값';
COMMENT ON COLUMN facilities.map_id IS '등록 기준 지도';
COMMENT ON COLUMN facilities.facility_type IS '소화기 / 방화셔터 하강구간 / 유도등';
COMMENT ON COLUMN facilities.has_gauge IS '소화기 압력계까지 판정할지(BR-T13)';
COMMENT ON COLUMN facilities.expected_x IS '등록 위치 x(m). 검출 위치와 비교해 위치 이상(POSITION_ANOMALY) 판단';
COMMENT ON COLUMN facilities.expected_y IS '등록 위치 y(m)';
COMMENT ON COLUMN facilities.view_pose IS '관측 자세 {x, y, yaw}. 설비 앞 1 m 안 정면. FACILITY_CHECK 작업의 waypoint로 보냄';
COMMENT ON COLUMN facilities.is_active IS '철거된 설비는 false';

COMMENT ON TABLE runs IS '순찰 1회 · 두 로봇 공동 — 순찰 1회 전체. 두 로봇이 함께 맡는 하나의 작업 단위이며 특정 로봇에 속하지 않습니다. 지도·칸 통계·후보·보고서 같은 결과는 이 단위로 모이고, 실제로 누가 무엇을 했는지는 그 아래 tasks에 나뉘어 기록됩니다(run 1 : task N). 두 로봇의 역할(1차 순찰 / 재검증)은 시스템에 고정되어 있어 run마다 저장하지 않습니다.';
COMMENT ON COLUMN runs.run_id IS '순찰 1회의 번호. 이 순찰의 모든 task와 결과 데이터가 이 값으로 묶임. run_config·execute_task로 제어 PC에 전달되고, 제어 PC가 붙인 NetSample.run_id로 돌아옴(로봇은 모름). 실패 후 다시 시작하면 새 run_id';
COMMENT ON COLUMN runs.map_id IS '이 회차가 쓴 지도';
COMMENT ON COLUMN runs.run_type IS '정기 순찰 / 배치 변경 뒤 재측정(UC-03) / 시연';
COMMENT ON COLUMN runs.status IS 'running = 진행 중, done = 두 로봇 모두 DOCK 완료, aborted = task 실패·비상정지·중단. aborted 회차는 기준선·보고서에서 제외하고 처음부터 새 run으로 다시 수행';
COMMENT ON COLUMN runs.is_baseline_ok IS 'done이고 확정 이상이 없으면 종료 때 true. 보고서의 기준선 재료로 사용';
COMMENT ON COLUMN runs.started_by IS 'server page에서 순찰을 시작한 사람';
COMMENT ON COLUMN runs.started_at IS '시작 시각(POST /api/runs로 run을 만든 시각)';
COMMENT ON COLUMN runs.ended_at IS '종료 시각. 진행 중이면 NULL';

COMMENT ON TABLE tasks IS '로봇별 수행 단위 · 계획분+추가분 — run 안에서 로봇 한 대가 한 번에 하나씩 수행하는 일의 조각. ExecuteTask 액션 1회와 1:1입니다. 순찰 시작 때 계획을 나눈 것(계획분)과 순찰 중 이상 후보가 나와 추가된 재검증·순찰 끝 복귀(추가분)가 쌓이므로, 한 run의 tasks 목록은 곧 「각 로봇이 실제로 맡아 수행한 일의 기록」입니다. task는 모두 inspection_manager가 만들고, 수행 중에는 바꾸지 않습니다. 인계·재배정은 없습니다.';
COMMENT ON COLUMN tasks.task_id IS 'task ID. ExecuteTask.task_id와 같음. 예: ''r12-t003'' = run 12의 3번째 task. 번호를 0으로 채워 만들므로 task_id 순서 = 만든 순서 = 실행 순서';
COMMENT ON COLUMN tasks.run_id IS '이 task가 속한 순찰. run 하나에 task 여러 개';
COMMENT ON COLUMN tasks.robot_id IS '수행할 로봇. 역할이 고정이라 만들 때 정해짐 — SURVEY_SEGMENT·FACILITY_CHECK는 1차 순찰 로봇, REINSPECTION은 재검증 로봇, DOCK은 각자';
COMMENT ON COLUMN tasks.type IS 'task 종류(ExecuteTask.type). SURVEY_SEGMENT·FACILITY_CHECK는 순찰 시작 때(계획분), REINSPECTION은 후보 이벤트가 올 때, DOCK은 순찰 끝에 만들어짐';
COMMENT ON COLUMN tasks.target_ref IS '대상 ID. SURVEY_SEGMENT는 경로 구간 ''seg:0-40'', FACILITY_CHECK는 facility_id, REINSPECTION은 candidate_id(재검증 로봇의 제어 PC가 어느 후보를 확인하는지 알 수 있도록 메시지에도 실어 보냄), DOCK은 NULL';
COMMENT ON COLUMN tasks.waypoints IS '거쳐 갈 지점 [{x, y, yaw}, …]. ExecuteTask.waypoints로 전송. 자동 생성한 순찰 경로는 여기에만 저장되므로 회차마다 실제로 돈 경로가 남음';
COMMENT ON COLUMN tasks.status IS 'PENDING = 대기, ACTIVE = 수행 중, DONE = 완료, FAILED = 실패(→ run aborted), CANCELED = run이 중단되어 실행하지 않음. 수행 중에는 바꾸지 않으며 인계·재배정은 없음';
COMMENT ON COLUMN tasks.progress IS '진행률 0~1. execute_task feedback의 progress';
COMMENT ON COLUMN tasks.result_ok IS 'execute_task result의 ok. 끝나기 전에는 NULL';
COMMENT ON COLUMN tasks.reason IS '실패·중단 사유. 예: BLOCKED, BATTERY_LOW, NO_RESPONSE, MAP_MISMATCH, ESTOP';
COMMENT ON COLUMN tasks.assigned_at IS '그 로봇의 제어 PC에 보낸 시각. PENDING이면 NULL';
COMMENT ON COLUMN tasks.ended_at IS '끝난 시각';

COMMENT ON TABLE net_samples IS 'RSSI 표본 (NetSample) — RSSI 원시 표본. 실시간 히트맵, 보고서의 칸 통계·기준선, 후보의 근거 데이터가 모두 여기서 나옵니다.';
COMMENT ON COLUMN net_samples.robot_id IS '측정한 로봇(NetSample.robot_id)';
COMMENT ON COLUMN net_samples.run_id IS '회차(NetSample.run_id)';
COMMENT ON COLUMN net_samples.seq IS '제어 PC가 로봇·회차마다 1씩 올리는 번호. 같은 번호가 다시 오면 무시, 빠진 번호는 유실';
COMMENT ON COLUMN net_samples.stamp IS '로봇 rssi_scanner가 측정한 시각(NetSample.stamp). 수신 시각이 아님';
COMMENT ON COLUMN net_samples.x IS '측정 시각의 로봇 위치 x(m). 제어 PC가 AMCL 위치를 붙임';
COMMENT ON COLUMN net_samples.y IS '측정 시각의 로봇 위치 y(m)';
COMMENT ON COLUMN net_samples.yaw IS '측정 시각의 로봇 방향(rad)';
COMMENT ON COLUMN net_samples.pose_ok IS '위치 추정이 믿을 만한지(AMCL 공분산 기준). false면 통계에서 제외(E5)';
COMMENT ON COLUMN net_samples.ap_id IS '측정 대상 AP. 관제 PC api가 NetSample.ap_ssid로 aps.ssid를 찾아 채움(등록되지 않은 SSID면 저장하지 않음)';
COMMENT ON COLUMN net_samples.rssi_dbm IS '수신 신호 세기(dBm). 대상 AP가 안 보이면 −100';
COMMENT ON COLUMN net_samples.cell_id IS 'x, y가 속한 격자 칸. pose_ok가 false이거나 격자 밖이면 NULL';
COMMENT ON COLUMN net_samples.received_at IS '관제 PC가 받은 시각. stamp와 크게 차이 나면 재연결 뒤 늦게 받은 표본';

COMMENT ON TABLE precise_measures IS '정지 정밀 측정 (30 s) — 재검증 로봇이 후보 칸에 멈춰 30초 동안 잰 RSSI 요약입니다. 네트워크 후보의 확정·해제 근거입니다.';
COMMENT ON COLUMN precise_measures.measure_id IS '측정 번호';
COMMENT ON COLUMN precise_measures.task_id IS '이 측정을 시킨 REINSPECTION task. 이 task의 target_ref가 확인 대상 후보. (task_id, ap_id)는 유일 — 아웃박스 재전송 중복 방지';
COMMENT ON COLUMN precise_measures.robot_id IS '측정한 로봇';
COMMENT ON COLUMN precise_measures.ap_id IS '측정 대상 AP';
COMMENT ON COLUMN precise_measures.cell_id IS '로봇이 멈춰 선 칸';
COMMENT ON COLUMN precise_measures.duration_s IS '실제 측정 시간(초). 요청값은 settings.precise_duration_s(30)';
COMMENT ON COLUMN precise_measures.n_samples IS '측정 동안 얻은 RSSI 개수(PreciseMeasure.n_samples)';
COMMENT ON COLUMN precise_measures.rssi_median IS 'RSSI 중앙값(dBm). 재검증 로봇의 제어 PC가 이 값이 settings.rssi_suspect_dbm(−72) 미만이면 CONFIRMED, 아니면 CLEARED로 판단';
COMMENT ON COLUMN precise_measures.rssi_iqr IS 'RSSI 사분위 범위(75% − 25%). 클수록 신호가 불안정';
COMMENT ON COLUMN precise_measures.measured_at IS '측정이 끝난 시각';

COMMENT ON TABLE grid_stats IS '회차별 칸 통계 (보고서용) — 회차·칸·AP별 통계와 사후 분석 결과입니다. 보고서용이며 실시간 후보를 만들지 않습니다.';
COMMENT ON COLUMN grid_stats.run_id IS '회차';
COMMENT ON COLUMN grid_stats.cell_id IS '칸';
COMMENT ON COLUMN grid_stats.ap_id IS 'AP';
COMMENT ON COLUMN grid_stats.n IS '통계에 쓴 표본 수(pose_ok = true만). 두 로봇 표본을 합친 값. 순찰 종료 후 보고서용으로 계산';
COMMENT ON COLUMN grid_stats.mean IS '평균 RSSI(dBm). 표본이 없고 보간도 안 했으면 NULL';
COMMENT ON COLUMN grid_stats.median IS '중앙값 RSSI(dBm)';
COMMENT ON COLUMN grid_stats.var IS '분산(dB²). 표본 2개 미만이면 NULL';
COMMENT ON COLUMN grid_stats.min IS '최솟값(dBm)';
COMMENT ON COLUMN grid_stats.max IS '최댓값(dBm)';
COMMENT ON COLUMN grid_stats.confidence IS 'n < settings.min_samples(3)이면 low';
COMMENT ON COLUMN grid_stats.interpolated IS '표본 없이 주변 칸으로 채운 값이면 true. 판정에 쓰지 않음(BR-R02)';
COMMENT ON COLUMN grid_stats.state IS '사후 분석 결과(SR-08). coverage_suspect = 커버리지 저하 의심, unknown = 판단 불가. 보고서에만 표시하며 후보·이벤트를 만들지 않음(실시간 후보는 제어 PC가 판단)';
COMMENT ON COLUMN grid_stats.baseline_delta IS 'mean − 현재 기준선 mean(dB). 기준선이 없으면 NULL. 보고서의 「기준선 대비 변화」 표시용';
COMMENT ON COLUMN grid_stats.computed_at IS '계산 시각. 순찰이 done으로 끝난 뒤 한 번 계산(순찰 중 히트맵은 Redis live:{run}:cells)';

COMMENT ON TABLE baselines IS '기준선 (보고서용) — 정상 회차 여러 번으로 만든 칸별 기준선입니다. 보고서용입니다.';
COMMENT ON COLUMN baselines.baseline_id IS '기준선 번호';
COMMENT ON COLUMN baselines.cell_id IS '칸';
COMMENT ON COLUMN baselines.ap_id IS 'AP';
COMMENT ON COLUMN baselines.version IS '같은 칸·AP 안에서 1씩 올라가는 버전';
COMMENT ON COLUMN baselines.run_ids IS '이 기준선을 만든 정상 회차 목록';
COMMENT ON COLUMN baselines.mean IS '구성 회차 전체 표본의 평균(dBm)';
COMMENT ON COLUMN baselines.var IS '분산(dB²). 「평균 − 2σ」 판정의 σ는 이 값의 제곱근';
COMMENT ON COLUMN baselines.n IS '구성 표본 수';
COMMENT ON COLUMN baselines.built_at IS '만든 시각';
COMMENT ON COLUMN baselines.is_current IS '보고서 비교에 쓰는 현재 기준선. 칸·AP마다 true는 하나';

COMMENT ON TABLE inspections IS '설비 판정 (InspectionResult) — 설비 점검 1회의 판정 결과입니다. 1차 점검(FACILITY_CHECK)과 재검증 근접 재촬영(REINSPECTION) 모두 여기에 남습니다.';
COMMENT ON COLUMN inspections.inspection_id IS '판정 번호';
COMMENT ON COLUMN inspections.inspection_uid IS '재전송 중복 방지 키. 제어 PC detection_alert가 판정마다 만드는 UUID(사진 파일 이름에도 사용). 같은 값이 다시 오면 저장하지 않음';
COMMENT ON COLUMN inspections.task_id IS '이 판정을 만든 task. FACILITY_CHECK(1차 점검) 또는 REINSPECTION(근접 재촬영)';
COMMENT ON COLUMN inspections.run_id IS '회차';
COMMENT ON COLUMN inspections.robot_id IS '판정한 로봇';
COMMENT ON COLUMN inspections.facility_id IS '대상 설비';
COMMENT ON COLUMN inspections.detail IS '세부 판정(SR-21). ABNORMAL은 압력 이상·소등·적치 등';
COMMENT ON COLUMN inspections.final IS '최종 판정. 정면 확보 실패 등은 unconfirmed';
COMMENT ON COLUMN inspections.frontal IS '정면 판별에 성공했는지';
COMMENT ON COLUMN inspections.pose_retry IS '자세 보정 재관측 횟수. 3회를 넘기면 미확인(E6)';
COMMENT ON COLUMN inspections.gauge_state IS '압력계 판정. has_gauge이고 frontal일 때만 채움(BR-T13)';
COMMENT ON COLUMN inspections.confidence IS 'YOLO 검출 신뢰도(0~1). 검출이 없으면 NULL';
COMMENT ON COLUMN inspections.bbox IS '검출 상자 [x1, y1, x2, y2](이미지 픽셀)';
COMMENT ON COLUMN inspections.model_version IS '판정에 쓴 YOLO 모델 버전. 예: ''yolov8n-scv-1.2''. 모델별 정확도 비교용';
COMMENT ON COLUMN inspections.inspected_at IS '판정 시각';

COMMENT ON TABLE media IS '증거 사진 (파일은 볼륨) — 판정에 쓴 사진의 메타데이터. 파일 자체는 scv_media 볼륨에 있습니다.';
COMMENT ON COLUMN media.media_id IS '사진 번호';
COMMENT ON COLUMN media.inspection_id IS '이 사진을 쓴 판정';
COMMENT ON COLUMN media.robot_id IS '촬영한 로봇';
COMMENT ON COLUMN media.kind IS '컬러 사진 또는 깊이 영상';
COMMENT ON COLUMN media.object_key IS 'scv_media 볼륨 안 파일 경로. 예: ''2026/10/10/r12/EXT-01_1.jpg''';
COMMENT ON COLUMN media.sha256 IS '파일 해시. 업로드가 깨지지 않았는지·중복 업로드인지 확인';
COMMENT ON COLUMN media.size_bytes IS '파일 크기(바이트). 보관 용량 관리';
COMMENT ON COLUMN media.captured_at IS '촬영 시각(제어 PC 기준)';

COMMENT ON TABLE candidates IS '이상 후보·재검증 결과 — 이상 후보와 그 재검증 결론을 한 행에 담습니다. 1차 순찰 로봇의 제어 PC가 당장 받은 값(자기 RSSI 표본, YOLO 판정)으로 후보를 내고, 재검증 로봇의 제어 PC가 결론을 냅니다. 기준선 이탈·두 로봇 병합 통계로는 후보를 만들지 않습니다(보고서에서 확인). 후보는 확정이 아니며 정상으로도 처리하지 않습니다(BR-R02).';
COMMENT ON COLUMN candidates.candidate_id IS '후보 ID. 1차 순찰 로봇의 제어 PC가 CANDIDATE 이벤트에 붙여 보내는 값. 예: ''r12-c01''';
COMMENT ON COLUMN candidates.run_id IS '후보가 나온 회차';
COMMENT ON COLUMN candidates.source IS '네트워크 측정에서 나왔는지, 설비 판정에서 나왔는지';
COMMENT ON COLUMN candidates.kind IS 'NET_COVERAGE(자기 측정 중앙값이 기준 미달), VIS_NOT_DETECTED, VIS_OCCLUDED, VIS_POSITION, VIS_ABNORMAL. 기준선 이탈·두 로봇 병합 통계로는 후보를 만들지 않음(보고서에서 확인)';
COMMENT ON COLUMN candidates.ref IS 'NETWORK면 cell_id, VISION이면 facility_id. 같은 run에서 같은 칸·설비는 후보 하나';
COMMENT ON COLUMN candidates.target_x IS '재방문할 지점 x(m). Candidate.target';
COMMENT ON COLUMN candidates.target_y IS '재방문할 지점 y(m)';
COMMENT ON COLUMN candidates.score IS 'NETWORK면 그 칸에서 잰 RSSI 중앙값(dBm), VISION이면 판정 신뢰도';
COMMENT ON COLUMN candidates.detect_task_id IS '후보를 발견한 1차 task(SURVEY_SEGMENT 또는 FACILITY_CHECK). 근거 데이터는 이 task·ref로 net_samples·inspections를 조회';
COMMENT ON COLUMN candidates.detected_by IS '후보를 낸 로봇(1차 순찰 로봇)';
COMMENT ON COLUMN candidates.status IS 'OPEN = 재검증 대기, REINSPECTING = 재검증 task 진행 중, CLOSED = 결론 남(outcome)';
COMMENT ON COLUMN candidates.reinspect_task_id IS '이 후보를 확인한 REINSPECTION task. 트리거로 type = REINSPECTION이고 그 task의 robot_id ≠ detected_by인지 검사';
COMMENT ON COLUMN candidates.outcome IS '재검증 로봇의 제어 PC가 보낸 결론. CONFIRMED = 확정(findings 생성), CLEARED = 재현 안 됨, UNCONFIRMED = 판단 불가(사람 확인). 결론 전에는 NULL';
COMMENT ON COLUMN candidates.detected_at IS '후보 발견 시각(CANDIDATE 이벤트 시각)';
COMMENT ON COLUMN candidates.decided_at IS '결론 시각(CONFIRMED·CLEARED·UNCONFIRMED 이벤트 시각)';

COMMENT ON TABLE findings IS '확정 이상 (알림 대상) — 확정된 이상 목록입니다. 시스템은 알리기만 하고 조치·해소는 담당자 몫이라 다루지 않습니다(BR-R01).';
COMMENT ON COLUMN findings.finding_id IS '확정 이상 번호';
COMMENT ON COLUMN findings.candidate_id IS '출발한 후보. 후보 하나에 확정 이상은 하나';
COMMENT ON COLUMN findings.task_id IS '확정을 지은 REINSPECTION task. 트리거로 candidates.reinspect_task_id와 같고 outcome = CONFIRMED인지 검사 → 재검증 없이 확정 0건(BR-T06)';
COMMENT ON COLUMN findings.category IS '이상 분류. 예: coverage, extinguisher, gauge, shutter, exit_light';
COMMENT ON COLUMN findings.status IS 'open = 확정됨, acked = 담당자가 봤음. 조치·해소 여부는 시스템이 다루지 않음';
COMMENT ON COLUMN findings.acked_by IS '확인(ack)한 사람';
COMMENT ON COLUMN findings.confirmed_at IS '확정 시각';

COMMENT ON TABLE robot_events IS '로봇 이벤트 (사건·후보·확정) — 제어 PC가 알리는 모든 이벤트(로봇 사건 포함)의 기록이자 유일한 알림 원본입니다. 로봇 사건(예외·정지·도킹)과 이상 알림(후보 발견, 확정·해제·미확인)이 함께 쌓이며, 사용자는 이 테이블을 보여 주는 이벤트 화면에서 확인합니다. 별도 알림 테이블·알림 화면은 없습니다.';
COMMENT ON COLUMN robot_events.event_id IS '이벤트 번호';
COMMENT ON COLUMN robot_events.event_uid IS '재전송 중복 방지 키. 제어 PC scv_agent가 아웃박스에 쓸 때 붙임(예: ''robot3-r12-e000123''), 관제 PC 판단(E3·E8)은 api가 ''control-<uuid>''로 붙임. 같은 값이 다시 오면 저장하지 않음(ON CONFLICT DO NOTHING)';
COMMENT ON COLUMN robot_events.robot_id IS '이벤트를 알린 로봇. 후보는 발견한 로봇, 확정·해제는 결론을 지은 로봇';
COMMENT ON COLUMN robot_events.run_id IS '회차. 순찰 밖(대기 중) 이벤트면 NULL';
COMMENT ON COLUMN robot_events.task_id IS '이벤트가 난 task. 후보·확정 이벤트는 반드시 채움. task 밖 사건(LOCAL_ESTOP 등)은 NULL';
COMMENT ON COLUMN robot_events.code IS '로봇 사건: E1~E8, LOCAL_ESTOP, MAP_MISMATCH, UNDOCK, DOCK / 이상: CANDIDATE, CONFIRMED, CLEARED, UNCONFIRMED';
COMMENT ON COLUMN robot_events.source IS 'robot = 제어 PC가 보고(사건·후보·확정), control = 관제 PC가 판단(E3 응답 없음, E8 DB 쓰기 실패)';
COMMENT ON COLUMN robot_events.level IS '중요도. 이벤트 화면 색. 예: CANDIDATE = warn, CONFIRMED = error';
COMMENT ON COLUMN robot_events.detail IS '상황 정보. 사건 예: {"battery": 14.5, "x": 3.1, "y": 7.4} / 후보 예: {"candidate_id": "r12-c01", "kind": "NET_COVERAGE", "ref": "cell:12,7", "score": -78, "target": [3.5, 7.5]}';
COMMENT ON COLUMN robot_events.ts IS '발생 시각(제어 PC 기준, 관제 PC 수신 시각 아님)';

COMMENT ON TABLE robot_status_log IS '상태 이력 (RobotStatus 1 Hz) — 1 Hz 상태 이력입니다. 최신값은 Redis에, 흐름은 여기에 남깁니다.';
COMMENT ON COLUMN robot_status_log.robot_id IS '로봇';
COMMENT ON COLUMN robot_status_log.ts IS 'status 메시지 시각. 1초 간격';
COMMENT ON COLUMN robot_status_log.run_id IS '진행 중 회차. 대기 중이면 NULL';
COMMENT ON COLUMN robot_status_log.task_id IS '수행 중 작업';
COMMENT ON COLUMN robot_status_log.x IS '위치 x(m)';
COMMENT ON COLUMN robot_status_log.y IS '위치 y(m)';
COMMENT ON COLUMN robot_status_log.yaw IS '방향(rad)';
COMMENT ON COLUMN robot_status_log.battery_pct IS '배터리 잔량(%). 15% 복귀(SR-10) 확인';
COMMENT ON COLUMN robot_status_log.nav_state IS 'IDLE, NAVIGATING, INSPECTING, MEASURING, DOCKING, DOCKED, STOPPED 등';
COMMENT ON COLUMN robot_status_log.progress IS '현재 작업 진행률 0~1. 작업이 없으면 NULL';

COMMENT ON TABLE reports IS '보고서·점검표 — 순찰 보고서와 점검표 파일 목록입니다. 칸 통계·두 로봇 병합·기준선 비교 같은 종합 분석은 여기서만 봅니다.';
COMMENT ON COLUMN reports.report_id IS '보고서 번호';
COMMENT ON COLUMN reports.run_id IS '대상 회차';
COMMENT ON COLUMN reports.kind IS '네트워크 순찰 보고서 / 설비 점검표';
COMMENT ON COLUMN reports.file_key IS 'scv_media 안 파일 경로(PDF 등)';
COMMENT ON COLUMN reports.summary IS '요약 수치. 예: {"suspect_cells": 12, "coverage_pct": 96.5, "findings": 2}';
COMMENT ON COLUMN reports.generated_at IS '생성 시각';

COMMIT;

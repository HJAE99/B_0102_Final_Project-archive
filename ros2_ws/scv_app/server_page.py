#!/usr/bin/env python3
"""
SCV server page — 관제 PC (3-2차)

  범위: PostgreSQL 기준 정보 7개 표(robots · users · settings · aps · maps · grid_cells · facilities)
  화면: /server/login → /server/monitor (현재 지도) · /server/master (정보 탭 — 세로 탭으로 표 선택 + 목록 창)
  제어 PC(client page) 연결: POST /api/client/login (계정 확인) · WebSocket /ws/agent (hello → 로봇 활성, status 1 Hz)
  실시간 상태(3-2차 — Redis 대신 이 프로세스 메모리): 로그인 세션(8 h) · 로봇별 최신 status
               1초마다 점검 — settings.status_timeout_s(30 s) 동안 status가 없으면 E3 기록 후 상태 삭제
  화면 실시간: WebSocket /ws/ui (모니터 — 로봇 위치·상태 1초마다)
  섹터(3-2차): 격자를 만들 때 칸을 섹터 1 · 2로 나눔(split_sectors — 임시 규칙, 바꿔 끼울 수 있음).
               모니터에 섹터를 색으로 표시, 순찰(run) 진행 중에는 그 지도의 격자 다시 생성·삭제와 현재 지도 바꾸기 금지
  실행: python3 server_page.py           (기본 http://0.0.0.0:8000)
  환경변수:
    SCV_DB_DSN     기본 "host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey"
    SCV_MEDIA_DIR  지도 PNG 등 파일 저장 위치 (기본 ./scv_media)
    SCV_HOST / SCV_PORT  기본 0.0.0.0 / 8000

  워커는 1개로만 실행 — 세션 · 로봇 상태 · 제어 PC WebSocket 연결을 이 프로세스 메모리에 두기 때문
  (메모리에는 다시 채울 수 있는 값만 둠. 기록은 모두 PostgreSQL)
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import re
import secrets
import threading
import uuid
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import numpy as np
import psycopg
import psycopg.errors
import uvicorn
import yaml
from datetime import datetime, timezone

from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from psycopg import sql
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb
from psycopg_pool import ConnectionPool
from starlette.concurrency import run_in_threadpool

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
BASE = Path(__file__).resolve().parent
STATIC_DIR = BASE / "static"
DSN = os.environ.get("SCV_DB_DSN",
                     "host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey")
MEDIA_DIR = Path(os.environ.get("SCV_MEDIA_DIR", BASE / "scv_media")).resolve()
HOST = os.environ.get("SCV_HOST", "0.0.0.0")
PORT = int(os.environ.get("SCV_PORT", "8000"))

SESSION_COOKIE = "scv_session"
SESSION_TTL_S = 8 * 3600                     # 로그인 세션 8 h (요청마다 연장)
PAGES = {"login", "monitor", "master"}

# 지도 빈 공간(흰 부분) 판정 — map_server(yaml) 기본 free 임계값 (제안설명서 4.2절)
FREE_THRESH = 0.25
UNKNOWN_PIXEL = 205          # map_saver가 미탐색 칸을 저장하는 값
SECTORS = (1, 2)
ROBOT_ID_RE = re.compile(r"^[a-z][a-z0-9_]*$")     # DB CHECK robots_robot_id_format과 같은 규칙
# 섹터 ↔ 로봇 기본 배정 (3차). 순찰 시작 때 바꿀 수 있고 run마다 runs.sector_robots에 저장(3-3차)
DEFAULT_SECTOR_ROBOTS = {"1": "robot3", "2": "robot4"}

pool = ConnectionPool(DSN, min_size=1, max_size=5, kwargs={"row_factory": dict_row}, open=False)
# 관제 PC 메모리 상태 (3-2차 — Redis 대신). 요청 처리 스레드 풀과 이벤트 루프가 함께 쓰므로 잠금 하나로 보호
MEM_LOCK = threading.Lock()
SESSIONS: dict[str, dict[str, Any]] = {}       # token → {user_id, username, role, created_at, expires(monotonic)}
ROBOT_STATE: dict[str, dict[str, Any]] = {}    # robot_id → {"fields": status 필드 + received_at, "rx": 받은 시각(monotonic)}


@asynccontextmanager
async def lifespan(_app: FastAPI):
    MEDIA_DIR.joinpath("maps").mkdir(parents=True, exist_ok=True)
    pool.open(wait=True, timeout=10)
    global MAIN_LOOP
    MAIN_LOOP = asyncio.get_running_loop()
    reset_reported_active()
    watchdog = asyncio.create_task(state_watchdog())
    yield
    watchdog.cancel()
    pool.close()


app = FastAPI(title="SCV server page", version="3-2", lifespan=lifespan)
MAIN_LOOP: asyncio.AbstractEventLoop | None = None       # 동기 API에서 제어 PC로 config를 밀어 보낼 때 씀

# ---------------------------------------------------------------------------
# 활성 여부 — robots · aps는 client(제어 PC)·로봇이 값을 보내 줄 때만 활성
#   server page에서는 바꾸지 못함. 아직 받은 값이 없으므로 시작할 때 모두 비활성으로 맞춤
#   (값을 받는 API는 client page·scv_agent 구현 차수에서 추가)
# ---------------------------------------------------------------------------
REPORTED_ACTIVE_TABLES = ("robots", "aps")


def reset_reported_active() -> None:
    with pool.connection() as conn:
        for t in REPORTED_ACTIVE_TABLES:
            conn.execute(sql.SQL("UPDATE {} SET is_active = false WHERE is_active").format(sql.Identifier(t)))

# ---------------------------------------------------------------------------
# 세션 — 메모리 SESSIONS (user_id · username · role · created_at), 8 h (요청마다 연장)
#   server_page를 다시 켜면 비므로 다시 로그인
# ---------------------------------------------------------------------------
def session_new(user: dict[str, Any]) -> str:
    token = secrets.token_urlsafe(32)
    with MEM_LOCK:
        SESSIONS[token] = {"user_id": int(user["user_id"]), "username": user["username"], "role": user["role"],
                           "created_at": datetime.now(timezone.utc).isoformat(),
                           "expires": time.monotonic() + SESSION_TTL_S}
    return token


def session_get(token: str | None) -> dict[str, Any] | None:
    if not token:
        return None
    with MEM_LOCK:
        s = SESSIONS.get(token)
        if not s:
            return None
        if s["expires"] < time.monotonic():
            SESSIONS.pop(token, None)
            return None
        s["expires"] = time.monotonic() + SESSION_TTL_S
        return {k: v for k, v in s.items() if k != "expires"}


def session_tokens_of(user_id: int) -> list[str]:
    with MEM_LOCK:
        return [t for t, s in SESSIONS.items() if s["user_id"] == user_id]


def session_drop(token: str):
    with MEM_LOCK:
        SESSIONS.pop(token, None)


def current_user(request: Request) -> dict[str, Any]:
    s = session_get(request.cookies.get(SESSION_COOKIE))
    if not s:
        raise HTTPException(401, "로그인이 필요합니다")
    return s


# ---------------------------------------------------------------------------
# DB 오류 → 사용자 메시지
# ---------------------------------------------------------------------------
def db_error(e: psycopg.Error, in_use_hint: str = "") -> HTTPException:
    state = e.sqlstate or ""
    detail = (e.diag.message_primary or str(e)).strip()
    if state == "23503":
        return HTTPException(409, "다른 표가 이 행을 사용 중이라 지울 수 없습니다. " + in_use_hint + "(" + detail + ")")
    if state == "23505":
        return HTTPException(409, "이미 있는 값입니다. (" + detail + ")")
    if state in ("23514", "23502"):
        return HTTPException(400, "입력값이 규칙에 맞지 않습니다. (" + detail + ")")
    if state.startswith("22"):
        return HTTPException(400, "입력 형식이 맞지 않습니다. (" + detail + ")")
    if state == "P0001":                     # 트리거 규칙 (예: 수퍼 계정 admin 보호)
        return HTTPException(400, detail)
    return HTTPException(500, "DB 오류: " + detail)


# ---------------------------------------------------------------------------
# 기준 정보 표 정의 — 화면(목록 창)도 이 정의로 그림
#   type: text · int · real · bool · select · pose · password · datetime
#   add: 행 추가 때 입력 · edit: 수정 가능 · ops: 이 표에서 허용하는 동작
#   order: 목록·요약 정렬 — 최신 순(시각 컬럼이 없으면 번호·ID 내림차순). 한 쪽에 PAGE_SIZE행
# ---------------------------------------------------------------------------
PAGE_SIZE = 10
SUPER_USER = "admin"         # 수퍼 계정 — 비밀번호만 변경, 삭제 불가 (DB CHECK·트리거와 같은 규칙)

def C(name, label, type_="text", add=False, edit=False, required=False, options=None, help_=None, default=None,
      hide=False):
    return {"name": name, "label": label, "type": type_, "add": add, "edit": edit,
            "required": required, "options": options, "help": help_, "default": default, "hide": hide}


FACILITY_TYPES = [{"value": v, "label": v} for v in ("extinguisher", "shutter", "exit_light")]

TABLES: dict[str, dict[str, Any]] = {
    "robots": {
        "title": "로봇", "desc": "SCV에 등록된 로봇과 짝인 제어 PC 주소. 활성은 client(제어 PC)가 값을 보내 줄 때만 — 받지 못하면 비활성",
        "pk": "robot_id", "ops": ["add", "edit", "delete"],
        "columns": [
            C("robot_id", "로봇 ID", add=True, required=True, help_="영문 소문자로 시작, 하이픈 없이 (예: robot3)"),
            C("robot_pc_host", "제어 PC 주소", add=True, edit=True),
            C("rpi_host", "로봇(RPi) 주소", add=True, edit=True),
            C("is_active", "활성 (client 보고)", "bool"),
            C("registered_at", "등록 시각", "datetime"),
        ],
        "list_sql": "SELECT * FROM robots",
        "order": "registered_at DESC, robot_id",
    },
    "users": {
        "title": "사용자 계정",
        "desc": "client page 로그인 계정(권한 user — 추가·수정·삭제) + 수퍼 계정 admin(server page·client page, 비밀번호만 변경)",
        "pk": "user_id", "ops": ["add", "edit", "delete"],
        "columns": [
            C("user_id", "번호", "int"),
            C("username", "아이디", add=True, edit=True, required=True),
            C("password", "비밀번호", "password", add=True, edit=True, required=True,
              help_="4자 이상 · bcrypt 해시로 저장. 수정 때는 바꿀 때만 입력"),
            C("role", "권한"),
            C("created_at", "생성 시각", "datetime"),
        ],
        "list_sql": "SELECT user_id, username, role, created_at FROM users",
        "order": "created_at DESC, user_id DESC",
    },
    "settings": {
        "title": "판정 기준값", "desc": "제안 수치 키-값. 바꾸면 변경자·시각이 남음",
        "pk": "key", "ops": ["add", "edit", "delete"],
        "columns": [
            C("key", "설정 이름", add=True, required=True),
            C("description", "설명", add=True, edit=True, help_="이 설정이 정하는 것 · 쓰는 곳"),
            C("value", "값", add=True, edit=True, required=True),
            C("unit", "단위", add=True, edit=True),
            C("source", "근거", add=True, edit=True),
            C("updated_by_name", "변경자"),
            C("updated_at", "변경 시각", "datetime"),
        ],
        "list_sql": ("SELECT s.key, s.description, s.value, s.unit, s.source, u.username AS updated_by_name, "
                     "s.updated_at FROM settings s LEFT JOIN users u ON u.user_id = s.updated_by"),
        "order": "updated_at DESC, key",
    },
    "aps": {
        "title": "측정 대상 AP", "desc": "지도마다 등록 — 목록 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. rssi_scanner가 측정할 핫스팟 SSID와 설치 위치. 활성은 client·로봇이 그 AP 값을 보내 줄 때만 — 받지 못하면 비활성",
        "pk": "ap_id", "ops": ["add", "edit", "delete"], "map_scoped": True,
        "columns": [
            C("ap_id", "번호", "int"),
            C("map_id", "지도", "int", hide=True),
            C("ssid", "SSID", add=True, edit=True, required=True),
            C("note", "메모", add=True, edit=True),
            C("x", "설치 x(m)", "real", add=True, edit=True, help_="x · y 함께 입력 또는 둘 다 비움"),
            C("y", "설치 y(m)", "real", add=True, edit=True),
            C("is_active", "활성 (client·로봇 보고)", "bool"),
        ],
        "list_sql": "SELECT * FROM aps",
        "order": "ap_id DESC",
    },
    "maps": {
        "title": "지도", "desc": "yaml + pgm 업로드로 등록(값은 파일에서 읽음). 수정은 로봇별 도크 접근 자세(도크 바로 앞에서 도크를 바라보는 자세)만. 지우면 그 지도의 격자 · AP · 설비도 함께 지움(순찰 기록이 있으면 거부)",
        "pk": "map_id", "ops": ["upload", "activate", "edit", "delete"],
        "columns": [
            C("map_id", "번호", "int"),
            C("name", "이름"),
            C("robot_map_path", "제어 PC 지도 경로"),
            C("image_key", "이미지", hide=True),
            C("resolution_m", "해상도(m/px)", "real"),
            C("origin_x", "원점 x(m)", "real"),
            C("origin_y", "원점 y(m)", "real"),
            C("grid_size_m", "격자(m)", "real"),
            C("dock_poses", "도크 접근 자세 (로봇별 x, y, yaw)", "docks", edit=True),
            C("cell_count", "격자 칸 수", "int"),
            C("is_active", "현재 지도", "bool"),
            C("created_at", "등록 시각", "datetime"),
        ],
        "list_sql": ("SELECT m.*, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS cell_count "
                     "FROM maps m"),
        "order": "created_at DESC, map_id DESC",
    },
    "grid_cells": {
        "title": "격자 칸", "desc": "현재 지도로 선택할 때 그 지도만 자동 생성 — 지도의 흰 부분이 걸치는 칸은 모두. 섹터 1 · 2로 나눔(ix 순 칸 수 절반). 손으로 추가·수정하지 않음",
        "pk": "cell_id", "ops": ["regenerate", "delete_map_cells"],
        "columns": [
            C("cell_id", "번호", "int"),
            C("map_id", "지도", "int"),
            C("ix", "ix", "int"),
            C("iy", "iy", "int"),
            C("center_x", "중심 x(m)", "real"),
            C("center_y", "중심 y(m)", "real"),
            C("sector", "섹터", "int"),
        ],
        "list_sql": None,       # 지도별로 조회 (아래 list_grid_cells)
        "order": "cell_id DESC",
    },
    "facilities": {
        "title": "점검 설비", "desc": "지도마다 등록 — 목록 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. 설비 ID는 지도 안에서 유일. 소화기·셔터·유도등 위치와 관측 자세",
        "pk": "facility_id", "ops": ["add", "edit", "delete"], "map_scoped": True,
        "columns": [
            C("facility_id", "설비 ID", add=True, required=True, help_="예: EXT-01 (지도 안에서 유일)"),
            C("map_id", "지도", "int", hide=True),
            C("facility_type", "종류", "select", add=True, edit=True, required=True, options=FACILITY_TYPES),
            C("has_gauge", "압력계", "bool", add=True, edit=True, default=False),
            C("expected_x", "위치 x(m)", "real", add=True, edit=True, required=True),
            C("expected_y", "위치 y(m)", "real", add=True, edit=True, required=True),
            C("view_pose", "관측 자세 (x, y, yaw)", "pose", add=True, edit=True, required=True),
            C("is_active", "활성", "bool", add=True, edit=True, default=True),
        ],
        "list_sql": "SELECT * FROM facilities",
        "order": "facility_id DESC",          # 등록 시각 컬럼이 없어 ID 내림차순
    },
}
ORDER = ["robots", "users", "settings", "aps", "maps", "grid_cells", "facilities"]


def table_meta(name: str) -> dict[str, Any]:
    meta = TABLES.get(name)
    if not meta:
        raise HTTPException(404, "없는 표입니다")
    return meta


def coerce(col: dict[str, Any], value: Any) -> Any:
    """화면에서 온 값을 컬럼 형식으로 바꿈. 빈 문자열은 NULL."""
    t = col["type"]
    if value is None or (isinstance(value, str) and value.strip() == ""):
        if col["required"]:
            raise HTTPException(400, f"「{col['label']}」을(를) 입력하세요")
        return None
    try:
        if t == "int" or (t == "select" and col["options"] == "maps"):
            return int(value)
        if t == "real":
            v = float(value)
            if not math.isfinite(v):
                raise ValueError
            return v
        if t == "bool":
            return value if isinstance(value, bool) else str(value).lower() in ("true", "1", "on", "yes")
        if t == "pose":
            pose = {k: float(value[k]) for k in ("x", "y", "yaw")}
            return Jsonb(pose)
        if t == "docks":                               # {robot_id: {x, y, yaw}} — x · y가 빈 로봇은 도크 없음
            out = {}
            for rid, p in dict(value).items():
                if not ROBOT_ID_RE.match(str(rid)):
                    raise ValueError
                if p is None or all(str(p.get(k, "")).strip() == "" for k in ("x", "y", "yaw")):
                    continue
                txt = {k: str(p.get(k, "") if p.get(k) is not None else "").strip() for k in ("x", "y", "yaw")}
                if not txt["x"] or not txt["y"]:           # x · y는 함께 (yaw만 비우면 0)
                    raise ValueError
                v = {"x": float(txt["x"]), "y": float(txt["y"]), "yaw": float(txt["yaw"] or 0.0)}
                if not all(math.isfinite(z) for z in v.values()):
                    raise ValueError
                out[str(rid)] = v
            return Jsonb(out)
    except (TypeError, ValueError, KeyError):
        raise HTTPException(400, f"「{col['label']}」 형식이 맞지 않습니다")
    return str(value).strip()


def select_options(conn) -> dict[str, list[dict[str, Any]]]:
    maps = conn.execute("SELECT map_id, name, is_active FROM maps ORDER BY map_id").fetchall()
    return {"maps": [{"value": m["map_id"], "label": f"{m['map_id']} · {m['name']}" + (" (현재)" if m["is_active"] else "")}
                     for m in maps]}


# ---------------------------------------------------------------------------
# 화면 (SPA) · 정적 파일
# ---------------------------------------------------------------------------
@app.get("/", include_in_schema=False)
def root():
    return RedirectResponse("/server/login")


@app.get("/server", include_in_schema=False)
def server_root():
    return RedirectResponse("/server/monitor")


@app.get("/server/{page}", include_in_schema=False)
def server_page(page: str):
    if page not in PAGES:
        raise HTTPException(404, "없는 화면입니다")
    return FileResponse(STATIC_DIR / "server" / "index.html")


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
MEDIA_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/media", StaticFiles(directory=MEDIA_DIR), name="media")


# ---------------------------------------------------------------------------
# 로그인
# ---------------------------------------------------------------------------
@app.post("/api/login")
async def login(request: Request):
    body = await request.json()
    username = str(body.get("username", "")).strip()
    password = str(body.get("password", ""))
    if not username or not password:
        raise HTTPException(400, "아이디와 비밀번호를 입력하세요")
    with pool.connection() as conn:
        user = conn.execute(
            "SELECT user_id, username, role FROM users "
            "WHERE username = %s AND password_hash = crypt(%s, password_hash)",
            (username, password)).fetchone()
    if not user:
        raise HTTPException(401, "아이디 또는 비밀번호가 맞지 않습니다")
    if user["role"] != "admin":
        raise HTTPException(403, "일반 계정은 client page용입니다. server page는 admin만 로그인할 수 있습니다")
    token = session_new(user)
    resp = JSONResponse({"username": user["username"]})
    resp.set_cookie(SESSION_COOKIE, token, httponly=True, samesite="lax", max_age=SESSION_TTL_S, path="/")
    return resp


@app.post("/api/logout")
def logout(request: Request):
    token = request.cookies.get(SESSION_COOKIE)
    if token:
        session_drop(token)
    resp = JSONResponse({"ok": True})
    resp.delete_cookie(SESSION_COOKIE, path="/")
    return resp


@app.get("/api/me")
def me(request: Request):
    """로그인 여부 확인 (로그인 전에도 200 — 화면이 401 오류를 띄우지 않게)."""
    try:
        u = current_user(request)
    except HTTPException:
        return {"user": None}
    return {"user": {"user_id": u["user_id"], "username": u["username"]}}


# ---------------------------------------------------------------------------
# 쪽 나누기 — 최신 순 PAGE_SIZE행씩
# ---------------------------------------------------------------------------
def paging(total: int, page: int | None) -> tuple[int, int, int]:
    """(쪽 번호, 전체 쪽 수, OFFSET). 범위를 넘으면 마지막 쪽으로."""
    pages = max(1, math.ceil(total / PAGE_SIZE))
    p = min(max(1, page or 1), pages)
    return p, pages, (p - 1) * PAGE_SIZE


def page_rows(conn, base_sql: str, order: str, page: int | None, params=()) -> dict[str, Any]:
    total = conn.execute(f"SELECT count(*) AS n FROM ({base_sql}) t", params).fetchone()["n"]
    p, pages, off = paging(total, page)
    rows = conn.execute(f"SELECT * FROM ({base_sql}) t ORDER BY {order} LIMIT {PAGE_SIZE} OFFSET {off}",
                        params).fetchall()
    return {"rows": rows, "total": total, "page": p, "pages": pages, "size": PAGE_SIZE}


# ---------------------------------------------------------------------------
# 정보 탭 — 표별 요약 (행을 나열하는 표는 최신 순 10개씩 쪽 나눔)
# ---------------------------------------------------------------------------
def summary_card(conn, t: str, page: int | None = None) -> dict[str, Any]:
    meta = TABLES[t]
    count = conn.execute(sql.SQL("SELECT count(*) AS n FROM {}").format(sql.Identifier(t))).fetchone()["n"]
    items: list[list[Any]] = []
    p, pages = 1, 1
    if t in ("aps", "facilities"):                    # 지도에 속한 표 — 현재 지도 기준
        cur = conn.execute("SELECT map_id, name FROM maps WHERE is_active").fetchone()
        if not cur:
            return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": 0,
                    "items": [["현재 지도", "없음"]], "page": 1, "pages": 1}
        count = conn.execute(sql.SQL("SELECT count(*) AS n FROM {} WHERE map_id = %s").format(sql.Identifier(t)),
                             (cur["map_id"],)).fetchone()["n"]
        others = conn.execute(sql.SQL("SELECT count(*) AS n FROM {} WHERE map_id <> %s").format(sql.Identifier(t)),
                              (cur["map_id"],)).fetchone()["n"]
        items.append(["현재 지도", cur["name"], f"다른 지도 {others}행"])
        if t == "aps":
            pg = page_rows(conn, "SELECT * FROM aps WHERE map_id = %s", meta["order"], page, (cur["map_id"],))
            p, pages = pg["page"], pg["pages"]
            for r in pg["rows"]:
                pos = f" · ({r['x']:g}, {r['y']:g})" if r["x"] is not None else " · 위치 없음"
                items.append([r["ssid"], ("활성" if r["is_active"] else "비활성") + pos])
        else:
            for r in conn.execute("SELECT facility_type, count(*) AS n, count(*) FILTER (WHERE is_active) AS a "
                                  "FROM facilities WHERE map_id = %s GROUP BY facility_type ORDER BY facility_type",
                                  (cur["map_id"],)):
                items.append([r["facility_type"], f"{r['n']}개 (활성 {r['a']})"])
        return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": count, "items": items,
                "page": p, "pages": pages}
    if t in ("robots", "users", "settings"):
        pg = page_rows(conn, meta["list_sql"], meta["order"], page)
        p, pages = pg["page"], pg["pages"]
        for r in pg["rows"]:
            if t == "robots":
                items.append([r["robot_id"], ("활성" if r["is_active"] else "비활성") +
                              (f" · {r['robot_pc_host']}" if r["robot_pc_host"] else "")])
            elif t == "users":
                items.append([r["username"], "admin (수퍼 계정)" if r["role"] == "admin" else r["role"]])
            elif t == "settings":
                items.append([r["key"], r["value"] + (f" {r['unit']}" if r["unit"] else ""), r["description"] or ""])
    elif t == "maps":
        cur = conn.execute("SELECT name, resolution_m FROM maps WHERE is_active").fetchone()
        items.append(["현재 지도", cur["name"] if cur else "없음"])
        if cur:
            items.append(["해상도", f"{cur['resolution_m']:g} m/px"])
    elif t == "grid_cells":
        cur = conn.execute("SELECT m.name, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS n "
                           "FROM maps m WHERE m.is_active").fetchone()
        items.append(["현재 지도 칸", f"{cur['n']}개 ({cur['name']})" if cur else "현재 지도 없음"])
        if cur:
            for r in conn.execute("SELECT g.sector, count(*) AS n, min(g.ix) AS a, max(g.ix) AS b FROM grid_cells g "
                                  "JOIN maps m USING (map_id) WHERE m.is_active GROUP BY g.sector ORDER BY g.sector"):
                rid = DEFAULT_SECTOR_ROBOTS.get(str(r["sector"]), "—")
                items.append([f"섹터 {r['sector']}", f"{r['n']}칸 · ix {r['a']}~{r['b']}", f"기본 배정 {rid}"])
        n_maps = conn.execute("SELECT count(DISTINCT map_id) AS n FROM grid_cells").fetchone()["n"]
        items.append(["격자가 있는 지도", f"{n_maps}개"])
    return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": count, "items": items,
            "page": p, "pages": pages}


@app.get("/api/master/summary")
def master_summary(request: Request):
    current_user(request)
    with pool.connection() as conn:
        return {"cards": [summary_card(conn, t) for t in ORDER]}


@app.get("/api/master/summary/{table}")
def master_summary_one(table: str, request: Request, page: int = 1):
    current_user(request)
    table_meta(table)
    with pool.connection() as conn:
        return summary_card(conn, table, page)


# ---------------------------------------------------------------------------
# master — 목록·추가·수정·삭제 (일반 표)
# ---------------------------------------------------------------------------
def public_meta(name: str, conn) -> dict[str, Any]:
    meta = table_meta(name)
    opts = select_options(conn)
    cols = []
    for c in meta["columns"]:
        c2 = dict(c)
        if c2["options"] == "maps":
            c2["options"] = opts["maps"]
        if c2["type"] == "docks":                      # 도크 입력 칸 = 등록된 로봇마다
            c2["options"] = [r["robot_id"] for r in conn.execute("SELECT robot_id FROM robots ORDER BY robot_id")]
        cols.append(c2)
    return {"table": name, "title": meta["title"], "desc": meta["desc"], "pk": meta["pk"],
            "ops": meta["ops"], "columns": cols, "map_scoped": bool(meta.get("map_scoped"))}


@app.get("/api/master/{table}")
def list_rows(table: str, request: Request, map_id: int | None = None, page: int = 1):
    current_user(request)
    with pool.connection() as conn:
        meta = public_meta(table, conn)
        if table == "grid_cells" or TABLES[table].get("map_scoped"):
            return list_by_map(conn, table, meta, map_id, page)
        pg = page_rows(conn, TABLES[table]["list_sql"], TABLES[table]["order"], page)
    if table == "users":                       # 수퍼 계정: 비밀번호만 수정, 삭제 버튼 없음
        for row in pg["rows"]:
            if row["username"] == SUPER_USER:
                row["_edit_only"] = ["password"]
                row["_no_delete"] = True
    return {"meta": meta, **pg}


def list_by_map(conn, table, meta, map_id, page):
    """지도에 속한 표(grid_cells · aps · facilities) — 고른 지도의 행만. 처음은 현재 지도 (3-2차)"""
    maps = conn.execute("SELECT map_id, name, is_active FROM maps ORDER BY map_id").fetchall()
    if map_id is None:
        act = next((m for m in maps if m["is_active"]), None)
        map_id = act["map_id"] if act else (maps[0]["map_id"] if maps else None)
    pg = {"rows": [], "total": 0, "page": 1, "pages": 1, "size": PAGE_SIZE}
    if map_id is not None:
        pg = page_rows(conn, sql.SQL("SELECT * FROM {} WHERE map_id = %s").format(sql.Identifier(table)).as_string(conn),
                       TABLES[table]["order"], page, (map_id,))
    run = conn.execute("SELECT run_id FROM runs WHERE status = 'running' AND map_id = %s", (map_id,)).fetchone() \
        if map_id is not None else None
    return {"meta": meta, **pg, "map_id": map_id, "running_run": run["run_id"] if run else None,
            "maps": [{"value": m["map_id"], "label": f"{m['map_id']} · {m['name']}" + (" (현재)" if m["is_active"] else "")}
                     for m in maps]}


@app.post("/api/master/{table}")
async def add_row(table: str, request: Request):
    user = current_user(request)
    meta = table_meta(table)
    if "add" not in meta["ops"]:
        raise HTTPException(405, "이 표는 행 추가를 지원하지 않습니다")
    body = await request.json()
    try:
        with pool.connection() as conn:
            if table == "users":
                username = str(body.get("username", "")).strip()
                password = str(body.get("password", ""))
                if not username:
                    raise HTTPException(400, "「아이디」를 입력하세요")
                if len(password) < 4:
                    raise HTTPException(400, "비밀번호는 4자 이상이어야 합니다")
                if username == SUPER_USER:
                    raise HTTPException(400, f"「{SUPER_USER}」은 수퍼 계정 이름이라 쓸 수 없습니다")
                conn.execute("INSERT INTO users (username, password_hash, role) "
                             "VALUES (%s, crypt(%s, gen_salt('bf', 12)), 'user')", (username, password))
                return {"ok": True}
            cols, vals = [], []
            for c in meta["columns"]:
                if not c["add"]:
                    continue
                v = body.get(c["name"])
                if v is None:
                    v = c["default"]
                cols.append(sql.Identifier(c["name"]))
                vals.append(coerce(c, v))
            if table == "settings":
                cols.append(sql.Identifier("updated_by"))
                vals.append(user["user_id"])
            if table in REPORTED_ACTIVE_TABLES:      # 보고를 받기 전에는 비활성
                cols.append(sql.Identifier("is_active"))
                vals.append(False)
            if meta.get("map_scoped"):               # 지도에 속한 표 — 목록에서 고른 지도에 추가
                try:
                    mid = int(body.get("map_id"))
                except (TypeError, ValueError):
                    raise HTTPException(400, "지도를 먼저 고르세요")
                get_map(conn, mid)
                ensure_no_running_run(conn, mid, {"aps": "AP를", "facilities": "설비를"}[table])
                cols.append(sql.Identifier("map_id"))
                vals.append(mid)
            q = sql.SQL("INSERT INTO {} ({}) VALUES ({})").format(
                sql.Identifier(table), sql.SQL(", ").join(cols), sql.SQL(", ").join(sql.Placeholder() * len(vals)))
            conn.execute(q, vals)
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True}


@app.patch("/api/master/{table}/{pk}")
async def update_row(table: str, pk: str, request: Request):
    user = current_user(request)
    meta = table_meta(table)
    if "edit" not in meta["ops"]:
        raise HTTPException(405, "이 표는 수정할 수 없습니다")
    body = await request.json()
    if table == "users":
        return update_user(pk_value(meta, pk), body)
    sets, vals = [], []
    for c in meta["columns"]:
        if c["edit"] and c["name"] in body:
            sets.append(sql.SQL("{} = {}").format(sql.Identifier(c["name"]), sql.Placeholder()))
            vals.append(coerce(c, body[c["name"]]))
    if not sets:
        raise HTTPException(400, "바꿀 값이 없습니다")
    if table == "settings":
        sets.append(sql.SQL("updated_by = {}").format(sql.Placeholder()))
        vals.append(user["user_id"])
    key = pk_value(meta, pk)
    where, wvals = sql.SQL("{} = {}").format(sql.Identifier(meta["pk"]), sql.Placeholder()), [key]
    try:
        with pool.connection() as conn:
            with conn.transaction():
                mid = row_map_id(conn, table, key, request)
                if mid is not None:
                    ensure_no_running_run(conn, mid, LOCK_WHAT[table])    # 순찰 중인 지도의 AP · 설비 · 도크는 잠금
                    if table == "facilities":           # 설비 키 = (map_id, facility_id)
                        where = sql.SQL("{} AND map_id = {}").format(where, sql.Placeholder())
                        wvals.append(mid)
                q = sql.SQL("UPDATE {} SET {} WHERE {}").format(sql.Identifier(table), sql.SQL(", ").join(sets), where)
                n = conn.execute(q, [*vals, *wvals]).rowcount
    except psycopg.Error as e:
        raise db_error(e)
    if n == 0:
        raise HTTPException(404, "행이 없습니다")
    if table == "maps":                                 # 도크가 바뀜 → 현재 지도면 제어 PC에 다시 보냄
        push_config()
    return {"ok": True}


LOCK_WHAT = {"aps": "AP를", "facilities": "설비를", "maps": "도크 접근 자세를"}


def row_map_id(conn, table: str, key, request: Request) -> int | None:
    """그 행이 속한 지도 번호 (잠금 확인용). maps는 자기 번호, 설비는 ?map_id= 필수, AP는 행에서 읽음"""
    if table == "maps":
        return int(key)
    if table == "facilities":
        try:
            return int(request.query_params["map_id"])
        except (KeyError, ValueError):
            raise HTTPException(400, "설비는 지도 번호(map_id)와 함께 지정해야 합니다")
    if table == "aps":
        r = conn.execute("SELECT map_id FROM aps WHERE ap_id = %s", (key,)).fetchone()
        if not r:
            raise HTTPException(404, "행이 없습니다")
        return r["map_id"]
    return None


def update_user(user_id: int, body: dict[str, Any]):
    """일반 계정: 아이디·비밀번호 / 수퍼 계정 admin: 비밀번호만. 비밀번호는 입력했을 때만 바꿈."""
    username = body.get("username")
    password = str(body.get("password") or "")
    try:
        with pool.connection() as conn:
            row = conn.execute("SELECT username FROM users WHERE user_id = %s", (user_id,)).fetchone()
            if not row:
                raise HTTPException(404, "행이 없습니다")
            sets, vals = [], []
            if username is not None and str(username).strip() != row["username"]:
                username = str(username).strip()
                if row["username"] == SUPER_USER:
                    raise HTTPException(400, "수퍼 계정 admin은 비밀번호만 바꿀 수 있습니다")
                if not username:
                    raise HTTPException(400, "「아이디」를 입력하세요")
                if username == SUPER_USER:
                    raise HTTPException(400, f"「{SUPER_USER}」은 수퍼 계정 이름이라 쓸 수 없습니다")
                sets.append("username = %s"); vals.append(username)
            if password:
                if len(password) < 4:
                    raise HTTPException(400, "비밀번호는 4자 이상이어야 합니다")
                sets.append("password_hash = crypt(%s, gen_salt('bf', 12))"); vals.append(password)
            if not sets:
                raise HTTPException(400, "바꿀 값이 없습니다 (비밀번호는 바꿀 때만 입력)")
            conn.execute(f"UPDATE users SET {', '.join(sets)} WHERE user_id = %s", (*vals, user_id))
    except psycopg.Error as e:
        raise db_error(e)
    if isinstance(username, str) and username:     # 로그인 중인 세션의 이름도 맞춤
        with MEM_LOCK:
            for s in SESSIONS.values():
                if s["user_id"] == user_id:
                    s["username"] = username
    return {"ok": True}


def pk_value(meta, pk: str):
    col = next(c for c in meta["columns"] if c["name"] == meta["pk"])
    if col["type"] == "int":
        try:
            return int(pk)
        except ValueError:
            raise HTTPException(400, "번호 형식이 맞지 않습니다")
    return pk


@app.delete("/api/master/{table}/{pk}")
def delete_row(table: str, pk: str, request: Request):
    user = current_user(request)
    meta = table_meta(table)
    if "delete" not in meta["ops"]:
        raise HTTPException(405, "이 표는 행 삭제를 지원하지 않습니다")
    key = pk_value(meta, pk)
    try:
        with pool.connection() as conn:
            if table == "users":
                row = conn.execute("SELECT username FROM users WHERE user_id = %s", (key,)).fetchone()
                if row and row["username"] == SUPER_USER:
                    raise HTTPException(400, "수퍼 계정 admin은 삭제할 수 없습니다")
            if table == "maps":
                try:
                    return delete_map(conn, key)
                except psycopg.errors.ForeignKeyViolation as e:
                    raise HTTPException(409, "사용 중 — 이 지도의 순찰 · 측정 · 판정 기록이 있어 지울 수 없습니다 ("
                                        + (e.diag.message_primary or "") + ")")
            mid = row_map_id(conn, table, key, request)
            q = sql.SQL("DELETE FROM {} WHERE {} = %s").format(sql.Identifier(table), sql.Identifier(meta["pk"]))
            params = [key]
            if mid is not None:
                ensure_no_running_run(conn, mid, LOCK_WHAT[table])
                if table == "facilities":
                    q = sql.SQL("{} AND map_id = %s").format(q)
                    params.append(mid)
            n = conn.execute(q, params).rowcount
    except psycopg.Error as e:
        raise db_error(e, "비활성(is_active = false)으로 바꾸세요. " if table in ("facilities", "aps") else "")
    if n == 0:
        raise HTTPException(404, "행이 없습니다")
    if table == "users":                         # 지운 계정의 세션도 지움
        for t in session_tokens_of(key):
            session_drop(t)
    return {"ok": True}


# ---------------------------------------------------------------------------
# 지도 — 업로드 · 현재 지도 선택 · 격자 생성 · 삭제
# ---------------------------------------------------------------------------
def slug(text: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_-]+", "_", text).strip("_")
    return s or "map"


@app.post("/api/maps/upload")
async def upload_map(request: Request,
                     name: str = Form(...), robot_map_path: str = Form(...), grid_size_m: float = Form(1.0),
                     yaml_file: UploadFile = File(...), pgm_file: UploadFile = File(...)):
    current_user(request)
    name, robot_map_path = name.strip(), robot_map_path.strip()
    if not name or not robot_map_path:
        raise HTTPException(400, "이름과 제어 PC 지도 경로를 입력하세요")
    if not (grid_size_m > 0):
        raise HTTPException(400, "격자 크기는 0보다 커야 합니다")
    try:
        meta = yaml.safe_load((await yaml_file.read()).decode("utf-8"))
        resolution = float(meta["resolution"])
        ox, oy, oyaw = (float(v) for v in meta["origin"][:3])
    except Exception:
        raise HTTPException(400, "yaml에서 resolution·origin을 읽지 못했습니다")
    if resolution <= 0:
        raise HTTPException(400, "yaml의 resolution이 0보다 커야 합니다")
    if abs(oyaw) > 1e-9:
        raise HTTPException(400, f"yaml origin의 yaw가 0이어야 합니다 (지금 {oyaw})")
    if int(meta.get("negate", 0) or 0) != 0:
        raise HTTPException(400, "negate = 1 지도는 지원하지 않습니다 (negate = 0으로 저장해 주세요)")
    try:
        img = Image.open(pgm_file.file)
        img.load()
        img = img.convert("L")
    except Exception:
        raise HTTPException(400, "pgm 이미지를 읽지 못했습니다")
    fname = f"{slug(name)}_{int(time.time())}.png"
    path = MEDIA_DIR / "maps" / fname
    img.save(path, format="PNG")                        # 크기는 바꾸지 않음
    try:
        with pool.connection() as conn:
            row = conn.execute(
                "INSERT INTO maps (name, robot_map_path, image_key, resolution_m, origin_x, origin_y, grid_size_m) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s) RETURNING map_id",
                (name, robot_map_path, f"maps/{fname}", resolution, ox, oy, grid_size_m)).fetchone()
    except psycopg.Error as e:
        path.unlink(missing_ok=True)
        raise db_error(e)
    return {"ok": True, "map_id": row["map_id"], "width": img.width, "height": img.height}


def split_sectors(cells) -> dict[tuple[int, int], int]:
    """칸 (ix, iy) 목록 → 섹터 번호 (3차 — 임시 규칙, 나중에 이 함수만 바꿔 끼움).
    ix 순으로 칸 수가 절반에 가장 가까운 열에서 나눔: 왼쪽(그 열까지) = 섹터 1, 오른쪽 = 섹터 2.
    같은 차이면 왼쪽 열에서 나눔. 열이 하나뿐이면 모두 섹터 1.
    (DB 01_schema.sql 업그레이드 구간 ⑥ · 더미 make_dummy.py도 같은 규칙)"""
    cells = [(int(a), int(b)) for a, b in cells]
    cols: dict[int, int] = {}
    for ix, _ in cells:
        cols[ix] = cols.get(ix, 0) + 1
    order = sorted(cols)
    total, left, split, best = len(cells), 0, None, None
    for ix in order[:-1]:
        left += cols[ix]
        diff = abs(2 * left - total)
        if best is None or diff < best:
            best, split = diff, ix
    return {c: (1 if split is None or c[0] <= split else 2) for c in cells}


def generate_cells(conn, m: dict[str, Any]) -> int:
    """지도 PNG의 흰 부분(빈 공간)이 조금이라도 걸치는 칸은 모두 grid_cells에 넣고 섹터를 정함.
    벽 여유(wall_margin_m)로 빼지 않음 — 벽 가까이 좁게 남은 흰 부분도 칸이 생김 (경로 생성에서만 벽 여유를 씀)."""
    png = MEDIA_DIR / m["image_key"]
    if not png.exists():
        raise HTTPException(500, f"지도 이미지 파일이 없습니다: {m['image_key']}")
    res, ox, oy, g = float(m["resolution_m"]), float(m["origin_x"]), float(m["origin_y"]), float(m["grid_size_m"])

    raw = np.asarray(Image.open(png).convert("L"))
    occ = 1.0 - raw.astype(np.float32) / 255.0             # negate = 0
    safe = (occ < FREE_THRESH) & (raw != UNKNOWN_PIXEL)    # 흰 부분. map_saver의 미탐색 205(회색)는 제외
    rows, cols = np.nonzero(safe)
    if rows.size == 0:
        return 0
    h = raw.shape[0]
    x = ox + (cols + 0.5) * res
    y = oy + (h - rows - 0.5) * res
    ix = np.floor((x - ox) / g).astype(np.int64)
    iy = np.floor((y - oy) / g).astype(np.int64)
    cells = np.unique(np.stack([ix, iy], axis=1), axis=0)
    sector = split_sectors(cells)
    with conn.cursor().copy("COPY grid_cells (map_id, ix, iy, center_x, center_y, sector) FROM STDIN") as cp:
        for cx, cy in cells:
            cp.write_row((m["map_id"], int(cx), int(cy), ox + (cx + 0.5) * g, oy + (cy + 0.5) * g,
                          sector[(int(cx), int(cy))]))
    return int(len(cells))


def get_map(conn, map_id: int) -> dict[str, Any]:
    m = conn.execute("SELECT * FROM maps WHERE map_id = %s", (map_id,)).fetchone()
    if not m:
        raise HTTPException(404, "지도가 없습니다")
    return m


@app.post("/api/maps/{map_id}/activate")
def activate_map(map_id: int, request: Request):
    """현재 지도로 선택. 그 지도에 격자가 없으면 이때 생성 (선택된 지도만)."""
    current_user(request)
    try:
        with pool.connection() as conn:
            with conn.transaction():
                m = get_map(conn, map_id)
                # 순찰(run) 진행 중에는 현재 지도를 다른 지도로 바꾸지 않음 (3-2차) — 같은 지도를 다시 고르는 것은 그대로
                r = conn.execute("SELECT run_id, map_id FROM runs WHERE status = 'running'").fetchone()
                if r and r["map_id"] != map_id:
                    raise HTTPException(409, f"순찰(run {r['run_id']})이 진행 중입니다. 순찰이 끝난 뒤 현재 지도를 바꾸세요")
                conn.execute("UPDATE maps SET is_active = false WHERE is_active AND map_id <> %s", (map_id,))
                conn.execute("UPDATE maps SET is_active = true WHERE map_id = %s", (map_id,))
                n = conn.execute("SELECT count(*) AS n FROM grid_cells WHERE map_id = %s", (map_id,)).fetchone()["n"]
                created = 0
                if n == 0:
                    created = generate_cells(conn, m)
                    n = created
    except psycopg.Error as e:
        raise db_error(e)
    push_config()                                      # 현재 지도가 바뀜 → 제어 PC에 도크 접근 자세 다시 보냄
    return {"ok": True, "cell_count": n, "created": created}


def ensure_no_running_run(conn, map_id: int, what: str = "격자를"):
    """순찰(run)이 진행 중인 지도는 격자를 바꾸지 않음 — 칸 번호·섹터가 순찰 도중 바뀌지 않도록 (3-2차)"""
    r = conn.execute("SELECT run_id FROM runs WHERE status = 'running' AND map_id = %s", (map_id,)).fetchone()
    if r:
        raise HTTPException(409, f"순찰(run {r['run_id']})이 진행 중인 지도입니다. 순찰이 끝난 뒤 {what} 바꾸세요")


@app.post("/api/maps/{map_id}/grid")
def regenerate_grid(map_id: int, request: Request):
    current_user(request)
    try:
        with pool.connection() as conn:
            with conn.transaction():
                m = get_map(conn, map_id)
                ensure_no_running_run(conn, map_id)
                conn.execute("DELETE FROM grid_cells WHERE map_id = %s", (map_id,))
                n = generate_cells(conn, m)
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True, "cell_count": n}


@app.delete("/api/maps/{map_id}/grid")
def delete_grid(map_id: int, request: Request):
    current_user(request)
    try:
        with pool.connection() as conn:
            get_map(conn, map_id)
            ensure_no_running_run(conn, map_id)
            n = conn.execute("DELETE FROM grid_cells WHERE map_id = %s", (map_id,)).rowcount
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True, "deleted": n}


def delete_map(conn, map_id: int):
    with conn.transaction():
        m = get_map(conn, map_id)
        # 그 지도에 속한 격자 칸 · AP · 설비를 함께 지움 — 순찰 · 측정 · 판정 기록이 참조하면 외래키로 거부(사용 중)
        for t in ("facilities", "aps", "grid_cells"):
            conn.execute(sql.SQL("DELETE FROM {} WHERE map_id = %s").format(sql.Identifier(t)), (map_id,))
        conn.execute("DELETE FROM maps WHERE map_id = %s", (map_id,))
    (MEDIA_DIR / m["image_key"]).unlink(missing_ok=True)
    return {"ok": True}


@app.get("/api/maps/active")
def active_map(request: Request):
    current_user(request)
    with pool.connection() as conn:
        m = conn.execute("SELECT m.*, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS cell_count "
                         "FROM maps m WHERE m.is_active").fetchone()
    if not m:
        return {"map": None}
    png = MEDIA_DIR / m["image_key"]
    if png.exists():
        with Image.open(png) as im:
            m["width"], m["height"] = im.size
    else:
        m["width"] = m["height"] = None
    m["image_url"] = "/media/" + m["image_key"]
    # 모니터 격자 표시용 — [cell_id, ix, iy, sector]. 칸 범위는 origin + ix·grid_size_m 로 화면에서 계산
    with pool.connection() as conn:
        m["cells"] = [[c["cell_id"], c["ix"], c["iy"], c["sector"]] for c in conn.execute(
            "SELECT cell_id, ix, iy, sector FROM grid_cells WHERE map_id = %s ORDER BY cell_id", (m["map_id"],))]
        run = conn.execute("SELECT run_id, sector_robots FROM runs WHERE status = 'running' AND map_id = %s",
                           (m["map_id"],)).fetchone()
    m["sectors"] = [{"sector": s_, "cells": sum(1 for c in m["cells"] if c[3] == s_),
                     "robot": DEFAULT_SECTOR_ROBOTS.get(str(s_))} for s_ in SECTORS]
    m["running_run"] = run["run_id"] if run else None          # 진행 중이면 격자 다시 생성 금지 (화면 안내용)
    # 지도 위 표시 (3차) — 도크(maps.dock_poses) · AP 설치 위치 · 점검 설비. 음영은 측정 결과로만(3-4차 히트맵)
    with pool.connection() as conn:
        m["aps"] = conn.execute("SELECT ap_id, ssid, x, y, is_active FROM aps WHERE map_id = %s AND x IS NOT NULL "
                                "ORDER BY ssid", (m["map_id"],)).fetchall()
        m["facilities"] = conn.execute(
            "SELECT facility_id, facility_type, has_gauge, expected_x, expected_y, view_pose, is_active "
            "FROM facilities WHERE map_id = %s ORDER BY facility_id", (m["map_id"],)).fetchall()
    return {"map": m}


# ---------------------------------------------------------------------------
# 제어 PC(client page · scv_agent) 연결
#   POST /api/client/login — client page 로그인 확인 (제어 PC는 DB에 접속하지 않으므로 관제 PC가 대신 확인)
#   WebSocket /ws/agent   — 첫 메시지 hello { robot_id, … } → robots.is_active = true, 연결이 끊기면 false
#   메시지 틀: { "type", "msg_id", "ts", "body" } (구현방안 6.2절)
# ---------------------------------------------------------------------------
@app.post("/api/client/login")
async def client_login(request: Request):
    body = await request.json()
    username = str(body.get("username", "")).strip()
    password = str(body.get("password", ""))
    if not username or not password:
        raise HTTPException(400, "아이디와 비밀번호를 입력하세요")

    def check():
        with pool.connection() as conn:
            return conn.execute(
                "SELECT user_id, username, role FROM users "
                "WHERE username = %s AND password_hash = crypt(%s, password_hash)",
                (username, password)).fetchone()

    user = await run_in_threadpool(check)
    if not user:                                       # admin · user 모두 client page에 들어갈 수 있음
        raise HTTPException(401, "아이디 또는 비밀번호가 맞지 않습니다")
    return {"user": user}


AGENTS: dict[str, WebSocket] = {}                      # robot_id → 지금 연결된 제어 PC
_msg_seq = 0


def envelope(type_: str, body: dict[str, Any]) -> dict[str, Any]:
    global _msg_seq
    _msg_seq += 1
    return {"type": type_, "msg_id": f"s-{_msg_seq:06d}",
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"), "body": body}


def agent_config(robot_id: str) -> dict[str, Any]:
    """제어 PC에 보내는 설정 (3-2차) — 현재 지도와 그 지도의 이 로봇 도크 접근 자세.
    client는 도크 자세를 받으면 로봇이 도킹 중(또는 위치 모름)일 때 {ns}/initialpose로 로봇 시작 위치를 지정"""
    with pool.connection() as conn:
        m = conn.execute("SELECT map_id, name, robot_map_path, dock_poses FROM maps WHERE is_active").fetchone()
    if not m:
        return {"map": None, "dock_pose": None}
    return {"map": {"map_id": m["map_id"], "name": m["name"], "robot_map_path": m["robot_map_path"]},
            "dock_pose": (m["dock_poses"] or {}).get(robot_id)}


async def _push_config_async():
    for rid, ws in list(AGENTS.items()):
        try:
            await ws.send_json(envelope("config", await run_in_threadpool(agent_config, rid)))
        except Exception as e:                          # 끊긴 연결은 ws_agent가 정리
            print(f"[agent] {rid} config 보내기 실패: {e}", flush=True)


def push_config():
    """연결된 모든 제어 PC에 config를 다시 보냄 (현재 지도 변경 · 도크 수정 때). 동기 API에서 불러도 됨"""
    if MAIN_LOOP is not None and AGENTS:
        asyncio.run_coroutine_threadsafe(_push_config_async(), MAIN_LOOP)


def set_robot_active(robot_id: str, active: bool) -> bool:
    """robots.is_active 변경. 등록된 로봇이면 True."""
    with pool.connection() as conn:
        return conn.execute("UPDATE robots SET is_active = %s WHERE robot_id = %s",
                            (active, robot_id)).rowcount == 1


@app.websocket("/ws/agent")
async def ws_agent(ws: WebSocket):
    await ws.accept()
    robot_id = None
    try:
        try:
            hello = await asyncio.wait_for(ws.receive_json(), timeout=10)
        except (asyncio.TimeoutError, json.JSONDecodeError, KeyError):
            await ws.close(code=4400, reason="first message must be hello")
            return
        body = hello.get("body") or {}
        if hello.get("type") != "hello" or not body.get("robot_id"):
            await ws.close(code=4400, reason="first message must be hello")
            return
        robot_id = str(body["robot_id"])
        if not await run_in_threadpool(set_robot_active, robot_id, True):
            await ws.send_json(envelope("error", {"reason": "UNKNOWN_ROBOT", "robot_id": robot_id}))
            await ws.close(code=4404, reason="unknown robot")
            robot_id = None
            return
        old = AGENTS.get(robot_id)
        AGENTS[robot_id] = ws
        if old is not None:                            # 같은 로봇의 이전 연결은 닫음 (재접속)
            try:
                await old.close(code=4409, reason="replaced by new connection")
            except Exception:
                pass
        print(f"[agent] {robot_id} 연결 · hello {body} → 활성", flush=True)
        await ws.send_json(envelope("welcome", {
            "server_time": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "run": None, "active_task_id": None}))
        await ws.send_json(envelope("config", await run_in_threadpool(agent_config, robot_id)))
        while True:
            msg = await ws.receive_json()
            kind = msg.get("type")
            if kind == "status":                       # 1 Hz — 메모리 최신값 + robot_status_log
                try:
                    await run_in_threadpool(store_status, robot_id, msg.get("body") or {}, msg.get("ts"))
                except (psycopg.Error, TypeError, ValueError) as e:
                    print(f"[agent] {robot_id} status 처리 실패: {e}", flush=True)
            else:
                print(f"[agent] {robot_id} ← {kind} (아직 처리하지 않는 메시지)", flush=True)
    except WebSocketDisconnect:
        pass
    finally:
        if robot_id and AGENTS.get(robot_id) is ws:
            AGENTS.pop(robot_id, None)
            await run_in_threadpool(set_robot_active, robot_id, False)
            print(f"[agent] {robot_id} 연결 끊김 → 비활성", flush=True)


# ---------------------------------------------------------------------------
# 로봇 상태 — 메모리 ROBOT_STATE (로봇별 최신 status), 보존 = settings.status_timeout_s(30 s)
#   state_watchdog가 1초마다 확인 → 그동안 status가 없으면 E3(응답 없음)를 robot_events에 기록하고 상태를 지움
# ---------------------------------------------------------------------------
STATE_FIELDS = ("ts", "run_id", "task_id", "nav_state", "progress", "x", "y", "yaw", "battery_pct", "is_docked", "estop")
LAST_E3: dict[str, str] = {}                           # robot_id → 마지막 E3 시각 (화면 표시용)
_ttl_cache = {"v": 30, "t": 0.0}


def status_timeout() -> int:
    if time.time() - _ttl_cache["t"] > 30:
        with pool.connection() as conn:
            row = conn.execute("SELECT value FROM settings WHERE key = 'status_timeout_s'").fetchone()
        try:
            _ttl_cache["v"] = max(1, int(float(row["value"]))) if row else 30
        except ValueError:
            _ttl_cache["v"] = 30
        _ttl_cache["t"] = time.time()
    return _ttl_cache["v"]


def _num(v):
    try:
        return None if v is None or v == "" else float(v)
    except (TypeError, ValueError):
        return None


def store_status(robot_id: str, body: dict[str, Any], msg_ts: str | None):
    ts = body.get("stamp") or msg_ts or datetime.now(timezone.utc).isoformat()
    vals = {"ts": ts, **{k: body.get(k) for k in STATE_FIELDS if k != "ts"}}
    fields = {k: "" if v is None else (str(v).lower() if isinstance(v, bool) else str(v))
              for k, v in {**vals, "received_at": datetime.now(timezone.utc).isoformat()}.items()}
    with MEM_LOCK:
        ROBOT_STATE[robot_id] = {"fields": fields, "rx": time.monotonic()}
    x, y, yaw, bat = _num(vals["x"]), _num(vals["y"]), _num(vals["yaw"]), _num(vals["battery_pct"])
    if None in (x, y, yaw, bat):                       # 위치·배터리를 모르면 이력은 남기지 않음 (메모리만)
        return
    try:
        with pool.connection() as conn:
            conn.execute(
                "INSERT INTO robot_status_log (robot_id, ts, run_id, task_id, x, y, yaw, battery_pct, nav_state, progress) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT (robot_id, ts) DO NOTHING",
                (robot_id, ts, vals["run_id"], vals["task_id"] or None, x, y, yaw, max(0.0, min(100.0, bat)),
                 str(vals["nav_state"] or "UNKNOWN")[:16], _num(vals["progress"])))
    except psycopg.Error as e:
        print(f"[status] {robot_id} robot_status_log 저장 실패: {e.diag.message_primary}", flush=True)


def record_e3(robot_id: str):
    ts = datetime.now(timezone.utc)
    detail = {"reason": "NO_STATUS", "timeout_s": status_timeout(), "agent_connected": robot_id in AGENTS}
    try:
        with pool.connection() as conn:
            conn.execute("INSERT INTO robot_events (event_uid, robot_id, code, source, level, detail, ts) "
                         "VALUES (%s, %s, 'E3', 'control', 'warn', %s, %s)",
                         (f"control-{uuid.uuid4()}", robot_id, Jsonb(detail), ts))
        LAST_E3[robot_id] = ts.isoformat()
        print(f"[E3] {robot_id} 응답 없음 — {detail['timeout_s']}초 동안 status 없음", flush=True)
    except psycopg.Error as e:
        print(f"[E3] {robot_id} 기록 실패: {e.diag.message_primary}", flush=True)


async def state_watchdog():
    """1초마다 — status가 settings.status_timeout_s 동안 없던 로봇은 E3 기록 후 상태를 지우고, 지난 세션을 정리."""
    while True:
        try:
            await asyncio.sleep(1.0)
            limit = await run_in_threadpool(status_timeout)
            now = time.monotonic()
            with MEM_LOCK:
                gone = [rid for rid, st in ROBOT_STATE.items() if now - st["rx"] > limit]
                for rid in gone:
                    ROBOT_STATE.pop(rid, None)
                for t in [t for t, s_ in SESSIONS.items() if s_["expires"] < now]:
                    SESSIONS.pop(t, None)
            for rid in gone:
                await run_in_threadpool(record_e3, rid)
        except asyncio.CancelledError:
            raise
        except Exception as e:                         # 점검은 멈추지 않음
            print(f"[watchdog] 점검 실패: {e}", flush=True)


def robots_snapshot() -> list[dict[str, Any]]:
    with pool.connection() as conn:
        robots = conn.execute("SELECT robot_id, is_active FROM robots ORDER BY robot_id").fetchall()
    limit = status_timeout()
    out = []
    for r in robots:
        with MEM_LOCK:
            mem = ROBOT_STATE.get(r["robot_id"])
            st = dict(mem["fields"]) if mem else None
            age = time.monotonic() - mem["rx"] if mem else 0.0
        state = None
        if st:
            state = {k: st.get(k) or None for k in (*STATE_FIELDS, "received_at")}
            for k in ("x", "y", "yaw", "battery_pct", "progress"):
                state[k] = _num(state[k])
            for k in ("is_docked", "estop"):
                state[k] = state[k] == "true"
            state["ttl_s"] = max(0, math.ceil(limit - age))       # E3까지 남은 초
            state["raw"] = dict(sorted(st.items()))           # 받은 status 그대로 (모니터에 전체 표시)
        out.append({"robot_id": r["robot_id"], "is_active": r["is_active"], "connected": r["robot_id"] in AGENTS,
                    "state": state, "last_e3": LAST_E3.get(r["robot_id"])})
    return out


@app.websocket("/ws/ui")
async def ws_ui(ws: WebSocket):
    """server page 모니터 — 1초마다 로봇 위치·상태를 보냄 (로그인 세션 쿠키로 확인)."""
    await ws.accept()
    if not await run_in_threadpool(session_get, ws.cookies.get(SESSION_COOKIE)):
        await ws.close(code=4401, reason="login required")
        return
    try:
        while True:
            snap = await run_in_threadpool(robots_snapshot)
            await ws.send_json({"type": "robots", "ts": datetime.now(timezone.utc).isoformat(),
                                "timeout_s": status_timeout(), "robots": snap})
            await asyncio.sleep(1.0)
    except (WebSocketDisconnect, RuntimeError):
        pass


@app.get("/api/agents")
def agents(request: Request):
    """지금 연결된 제어 PC 목록 (server page 확인용)."""
    current_user(request)
    return {"robots": sorted(AGENTS)}


if __name__ == "__main__":
    uvicorn.run(app, host=HOST, port=PORT, workers=1)

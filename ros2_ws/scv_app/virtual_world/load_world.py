#!/usr/bin/env python3
"""
가상 세계 지도 등록 (3-2차) — testbed 지도를 server page API로 등록하고 현재 지도로 지정

  관리자가 server page 화면에서 넣는 것과 같은 API를 씁니다(지도 등록 규칙 yaw = 0 · negate = 0, 격자 · 섹터 생성).
  AP · 설비 · 도크 접근 자세는 지도에 속한 정보라 server page 정보 탭에서 이 지도를 고르고 직접 입력합니다
  (README 0장의 testbed 입력값 표). world.yaml(실제 세계의 정답)은 등록하지 않습니다 — 가상 로봇만 읽음.
  시험 DB(robotdb4_test)에 쓰는 것을 권장.

  사용 (server_page.py가 실행 중일 때):
    python3 load_world.py                                   # http://127.0.0.1:8000, admin / admin
    python3 load_world.py --server http://<관제PC>:8000 --robot-map-path /home/rokey/…/virtual_world/testbed.yaml

  옵션
    --server          server page 주소 (기본 http://127.0.0.1:8000)
    --user / --password  관리자 계정 (기본 admin / admin)
    --map-yaml        지도 yaml (기본 이 스크립트 옆 testbed.yaml)
    --name            지도 이름 (기본 scv_virtual_testbed) · --grid 격자 크기 m (기본 1.0)
    --robot-map-path  제어 PC에 있는 testbed.yaml의 절대 경로 (기본: 이 PC에서의 절대 경로)
                      제어 PC가 다른 PC면 그 PC의 경로를 넣어야 함 (client가 지도를 불러올 때 씀)
  여러 번 실행해도 됨 — 이미 있는 지도(같은 robot_map_path)는 다시 올리지 않고 현재 지도로만 지정
"""
from __future__ import annotations

import argparse
import http.cookiejar
import json
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent


class Api:
    def __init__(self, base: str):
        self.base = base.rstrip("/")
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))

    def call(self, method: str, path: str, body=None, raw: bytes | None = None, ctype: str | None = None):
        data, headers = None, {}
        if body is not None:
            data, headers["Content-Type"] = json.dumps(body).encode(), "application/json"
        elif raw is not None:
            data, headers["Content-Type"] = raw, ctype
        req = urllib.request.Request(self.base + path, data=data, method=method, headers=headers)
        try:
            with self.opener.open(req, timeout=30) as r:
                return r.status, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read())
            except Exception:
                return e.code, None
        except urllib.error.URLError as e:
            sys.exit(f"server page에 접속하지 못했습니다 ({self.base}): {e.reason}")

    def upload(self, path: str, fields: dict[str, str], files: dict[str, Path]):
        b = uuid.uuid4().hex
        parts = []
        for k, v in fields.items():
            parts.append(f'--{b}\r\nContent-Disposition: form-data; name="{k}"\r\n\r\n{v}\r\n'.encode())
        for k, p in files.items():
            parts.append(f'--{b}\r\nContent-Disposition: form-data; name="{k}"; filename="{p.name}"\r\n'
                         f'Content-Type: application/octet-stream\r\n\r\n'.encode() + p.read_bytes() + b"\r\n")
        parts.append(f"--{b}--\r\n".encode())
        return self.call("POST", path, raw=b"".join(parts), ctype=f"multipart/form-data; boundary={b}")


def all_rows(api: Api, table: str) -> list[dict]:
    rows, page = [], 1
    while True:
        st, d = api.call("GET", f"/api/master/{table}?page={page}")
        if st != 200:
            sys.exit(f"{table} 목록 조회 실패: {d}")
        rows += d["rows"]
        if page >= d["pages"]:
            return rows
        page += 1


def main():
    ap = argparse.ArgumentParser(description="가상 세계 지도 등록")
    ap.add_argument("--server", default="http://127.0.0.1:8000")
    ap.add_argument("--user", default="admin")
    ap.add_argument("--password", default="admin")
    ap.add_argument("--map-yaml", default=str(HERE / "testbed.yaml"))
    ap.add_argument("--name", default="scv_virtual_testbed")
    ap.add_argument("--grid", type=float, default=1.0)
    ap.add_argument("--robot-map-path", default=None)
    a = ap.parse_args()

    yaml_path = Path(a.map_yaml).resolve()
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    pgm_path = (yaml_path.parent / meta["image"]).resolve()
    robot_map_path = a.robot_map_path or str(yaml_path)

    api = Api(a.server)
    st, d = api.call("POST", "/api/login", {"username": a.user, "password": a.password})
    if st != 200:
        sys.exit(f"로그인 실패: {d}")

    maps = [m for m in all_rows(api, "maps") if m["robot_map_path"] == robot_map_path]
    if maps:
        map_id = maps[0]["map_id"]
        print(f"지도: 이미 있음 — map_id {map_id} ({robot_map_path})")
    else:
        st, d = api.upload("/api/maps/upload",
                           {"name": a.name, "robot_map_path": robot_map_path, "grid_size_m": str(a.grid)},
                           {"yaml_file": yaml_path, "pgm_file": pgm_path})
        if st != 200:
            sys.exit(f"지도 등록 실패: {d}")
        map_id = d["map_id"]
        print(f"지도: 등록 — map_id {map_id} ({d['width']} × {d['height']} px, 제어 PC 경로 {robot_map_path})")
    st, d = api.call("POST", f"/api/maps/{map_id}/activate")
    if st != 200:
        sys.exit(f"현재 지도 지정 실패: {d}")
    print(f"      현재 지도로 지정 — 격자 {d['cell_count']}칸" + (f" (새로 {d['created']}칸 생성)" if d.get("created") else ""))
    api.call("POST", "/api/logout")
    print("완료 — 도크 접근 자세 · AP · 설비는 server page 정보 탭에서 이 지도를 고르고 입력하세요 (README 0장 표)")


if __name__ == "__main__":
    main()

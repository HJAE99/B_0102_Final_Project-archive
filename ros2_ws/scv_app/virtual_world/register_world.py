#!/usr/bin/env python3
"""
가상 세계 등록 (3-3차) — world.yaml의 map · register 구역을 server page API로 관제 PC(DB)에 등록

  관리자가 server page 화면에서 넣는 것과 같은 API를 씁니다(로그인 → 지도 등록 → 현재 지도 → 도크 → AP → 설비 → 측정 AP).
    1. 지도      map.yaml(+ pgm)을 지도 등록(yaw = 0 · negate = 0 규칙). 같은 robot_map_path 지도가 있으면 다시 올리지 않음
    2. 현재 지도 그 지도를 현재 지도로 — 격자가 없으면 이때 생성(섹터 1 · 2)
    3. 도크      register.dock_poses → 지도 › 도크 접근 자세 (다른 로봇의 도크는 그대로 둠)
    4. AP        register.aps → 측정 대상 AP (같은 SSID가 있으면 메모 · 설치 위치만 갱신)
    5. 설비      register.facilities → 점검 설비 (같은 설비 ID가 있으면 종류 · 압력계 · 위치 · 관측 자세 갱신)
    6. 측정 AP   register.aps 중 target: true — 활성 AP만 지정되므로 활성이 될 때까지 기다렸다가(기본 120초) 지정
                 AP 활성은 client가 그 SSID 값을 받을 때 server가 정함 — 기다리는 동안 다른 터미널에서 가상 로봇 · client를 켬
  truth 구역(실제 세계의 정답)은 읽지 않습니다 — 가상 로봇(robot3_virtual.py)만 읽음.
  여러 번 실행해도 결과가 같습니다. 순찰 중인 지도는 server page가 잠그므로 순찰이 끝난 뒤 실행하세요.

  사용 (server_page.py가 실행 중일 때, 시험 DB robotdb4_test 권장):
    python3 register_world.py                  # = --world testbed/world.yaml --robot-map-path <그 지도 yaml 절대 경로> --wait 120
    python3 register_world.py --no-target      # 등록만 (측정 AP는 지정하지 않음 · 기다리지 않음)
    python3 register_world.py --wait 300       # 측정 AP 활성을 더 오래 기다림 (0이면 한 번만 시도)
    python3 register_world.py --server http://<관제PC>:8000
    python3 register_world.py --world 다른세계/world.yaml
  측정 AP만 다시 지정하려면 그냥 다시 실행 — 등록은 여러 번 해도 같지만, 현재 지도 · 정보 탭에서 고친 도크 · AP · 설비 ·
  측정 AP가 world.yaml 값으로 돌아갑니다.

  옵션
    --world           world.yaml (기본: 이 스크립트 옆 testbed/world.yaml)
    --server          server page 주소 (기본 http://127.0.0.1:8000)
    --user / --password  관리자 계정 (기본 admin / admin)
    --robot-map-path  제어 PC에 있는 지도 yaml의 절대 경로 (기본: --world의 map.yaml을 이 PC에서 본 절대 경로)
                      제어 PC가 다른 PC면 그 PC의 경로 — client는 task의 지도 파일이 그 경로에 없으면 MAP_MISMATCH로 거부
    --prune           world.yaml에 없는 이 지도의 AP · 설비를 지움 (기록이 참조하면 server가 거부 → 그대로 둠)
    --wait 초         측정 AP가 비활성이면 그 시간까지 2초마다 다시 시도 (기본 120, 가상 로봇 + client가 켜져 있어야 활성)
    --no-target       측정 AP를 지정하지 않음 (기다리지도 않음) — 정보 탭에서 직접 지정
  종료 코드: 0 = 완료, 2 = 측정 AP를 지정하지 못함(시간 초과 · 중단), 그 밖 = 등록 실패
"""
from __future__ import annotations

import argparse
import http.cookiejar
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path

import yaml

DEFAULT_WORLD = Path(__file__).resolve().parent / "testbed" / "world.yaml"


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

    def must(self, what: str, method: str, path: str, body=None):
        st, d = self.call(method, path, body)
        if st != 200:
            sys.exit(f"{what} 실패 ({st}): {detail(d)}")
        return d

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


def detail(d) -> str:
    return d.get("detail") if isinstance(d, dict) and d.get("detail") else str(d)


def all_rows(api: Api, table: str, map_id: int | None = None) -> list[dict]:
    rows, page = [], 1
    while True:
        q = f"?page={page}" + (f"&map_id={map_id}" if map_id is not None else "")
        d = api.must(f"{table} 목록 조회", "GET", f"/api/master/{table}{q}")
        rows += d["rows"]
        if page >= d["pages"]:
            return rows
        page += 1


def register_map(api: Api, cfg: dict, yaml_path: Path, robot_map_path: str) -> int:
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    pgm_path = (yaml_path.parent / meta["image"]).resolve()
    maps = [m for m in all_rows(api, "maps") if m["robot_map_path"] == robot_map_path]
    if maps:
        map_id = maps[0]["map_id"]
        print(f"지도   이미 있음 — map_id {map_id} ({robot_map_path})")
    else:
        st, d = api.upload("/api/maps/upload",
                           {"name": cfg.get("name") or yaml_path.stem, "robot_map_path": robot_map_path,
                            "grid_size_m": str(cfg.get("grid_size_m", 1.0))},
                           {"yaml_file": yaml_path, "pgm_file": pgm_path})
        if st != 200:
            sys.exit(f"지도 등록 실패 ({st}): {detail(d)}")
        map_id = d["map_id"]
        print(f"지도   등록 — map_id {map_id} ({d['width']} × {d['height']} px, 제어 PC 경로 {robot_map_path})")
    d = api.must("현재 지도 지정", "POST", f"/api/maps/{map_id}/activate")
    print(f"       현재 지도로 지정 — 격자 {d['cell_count']}칸" + (f" (새로 {d['created']}칸 생성)" if d.get("created") else ""))
    return map_id


def register_docks(api: Api, map_id: int, docks: dict):
    if not docks:
        return
    cur = next((m for m in all_rows(api, "maps") if m["map_id"] == map_id), {}).get("dock_poses") or {}
    merged = {**cur, **{rid: {k: float(p[k]) for k in ("x", "y", "yaw")} for rid, p in docks.items()}}
    api.must("도크 접근 자세 저장", "PATCH", f"/api/master/maps/{map_id}", {"dock_poses": merged})
    txt = ", ".join("%s (%g, %g, %g)" % (r, float(p["x"]), float(p["y"]), float(p["yaw"])) for r, p in docks.items())
    print(f"도크   {txt}")


def register_aps(api: Api, map_id: int, aps: list[dict], prune: bool):
    have = {a["ssid"]: a for a in all_rows(api, "aps", map_id)}
    for a in aps:
        body = {"ssid": a["ssid"], "note": a.get("note"), "x": a.get("x"), "y": a.get("y")}
        if a["ssid"] in have:
            api.must(f"AP {a['ssid']} 갱신", "PATCH", f"/api/master/aps/{have[a['ssid']]['ap_id']}",
                     {k: body[k] for k in ("note", "x", "y")})
            print(f"AP     {a['ssid']} 갱신 ({a.get('x')}, {a.get('y')})")
        else:
            api.must(f"AP {a['ssid']} 추가", "POST", "/api/master/aps", {**body, "map_id": map_id})
            print(f"AP     {a['ssid']} 추가 ({a.get('x')}, {a.get('y')})")
    extra = [s for s in have if s not in {a["ssid"] for a in aps}]
    for s in extra:
        if prune:
            st, d = api.call("DELETE", f"/api/master/aps/{have[s]['ap_id']}")
            print(f"AP     {s} " + ("삭제" if st == 200 else f"삭제 못 함 ({detail(d)})"))
        else:
            print(f"AP     {s} — world.yaml에 없음 (그대로 둠, 지우려면 --prune)")


def register_facilities(api: Api, map_id: int, facs: list[dict], prune: bool):
    have = {f["facility_id"]: f for f in all_rows(api, "facilities", map_id)}
    for f in facs:
        body = {"facility_type": f["type"], "has_gauge": bool(f.get("gauge")), "expected_x": f["x"], "expected_y": f["y"],
                "view_pose": {k: float(f["view"][k]) for k in ("x", "y", "yaw")}, "is_active": f.get("active", True)}
        fid = str(f["id"])
        if fid in have:
            api.must(f"설비 {fid} 갱신", "PATCH",
                     f"/api/master/facilities/{urllib.parse.quote(fid)}?map_id={map_id}", body)
            print(f"설비   {fid} 갱신 ({f['type']})")
        else:
            api.must(f"설비 {fid} 추가", "POST", "/api/master/facilities", {**body, "facility_id": fid, "map_id": map_id})
            print(f"설비   {fid} 추가 ({f['type']})")
    extra = [s for s in have if s not in {str(f["id"]) for f in facs}]
    for s in extra:
        if prune:
            st, d = api.call("DELETE", f"/api/master/facilities/{urllib.parse.quote(s)}?map_id={map_id}")
            print(f"설비   {s} " + ("삭제" if st == 200 else f"삭제 못 함 ({detail(d)})"))
        else:
            print(f"설비   {s} — world.yaml에 없음 (그대로 둠, 지우려면 --prune)")


def set_target(api: Api, map_id: int, aps: list[dict], wait_s: float) -> bool:
    """측정 AP — 활성 AP만 지정됨. 비활성이면 wait_s 동안 2초마다 다시(10초마다 남은 시간 출력). 지정했으면 True"""
    tgt = [a["ssid"] for a in aps if a.get("target")]
    if not tgt:
        print("측정AP world.yaml에 target: true인 AP가 없음 — 정보 탭에서 지정하세요")
        return True
    if len(tgt) > 1:
        sys.exit(f"target: true인 AP가 둘 이상입니다 ({', '.join(tgt)}) — 지도마다 하나")
    t0 = time.monotonic()
    end, shown = t0 + wait_s, -1
    while True:
        a = next((x for x in all_rows(api, "aps", map_id) if x["ssid"] == tgt[0]), None)
        if a is None:
            sys.exit(f"측정 AP {tgt[0]}가 이 지도에 없습니다")
        if a["is_target"]:
            print(f"측정AP {tgt[0]} — 이미 지정됨")
            return True
        st, d = api.call("POST", f"/api/aps/{a['ap_id']}/target")
        if st == 200:
            print(f"측정AP {tgt[0]} 지정")
            return True
        now = time.monotonic()
        if now >= end:
            print(f"측정AP {tgt[0]} 지정 못 함 — {detail(d)}\n"
                  f"       가상 로봇(robot3_virtual.py)과 client를 켜고 client에 로그인한 뒤 "
                  f"register_world.py를 다시 실행하거나 정보 탭에서 지정하세요")
            return False
        if int(now - t0) // 10 != shown:                # 10초마다 한 줄
            shown = int(now - t0) // 10
            print(f"측정AP {tgt[0]} 활성 기다리는 중 — 남은 {int(end - now)}초 "
                  f"(다른 터미널에서 가상 로봇 · client를 켜고 client에 로그인)", flush=True)
        time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description="가상 세계 등록 (world.yaml map · register → server page)")
    ap.add_argument("--world", default=str(DEFAULT_WORLD), help="world.yaml (기본: 이 스크립트 옆 testbed/world.yaml)")
    ap.add_argument("--server", default="http://127.0.0.1:8000")
    ap.add_argument("--user", default="admin")
    ap.add_argument("--password", default="admin")
    ap.add_argument("--robot-map-path", default=None)
    ap.add_argument("--prune", action="store_true")
    ap.add_argument("--wait", type=float, default=120.0)
    ap.add_argument("--no-target", action="store_true")
    a = ap.parse_args()

    wpath = Path(a.world).resolve()
    if not wpath.exists():
        sys.exit(f"world 파일이 없습니다: {wpath}")
    w = yaml.safe_load(wpath.read_text(encoding="utf-8")) or {}
    cfg, reg = w.get("map") or {}, w.get("register") or {}
    if "yaml" not in cfg:
        sys.exit(f"{wpath}: map.yaml(지도 yaml 경로)이 없습니다")
    yaml_path = (wpath.parent / cfg["yaml"]).resolve()
    robot_map_path = a.robot_map_path or str(yaml_path)

    api = Api(a.server)
    api.must("로그인", "POST", "/api/login", {"username": a.user, "password": a.password})
    targeted = True
    try:
        map_id = register_map(api, cfg, yaml_path, robot_map_path)
        register_docks(api, map_id, reg.get("dock_poses") or {})
        register_aps(api, map_id, reg.get("aps") or [], a.prune)
        register_facilities(api, map_id, reg.get("facilities") or [], a.prune)
        if a.no_target:
            print("측정AP 지정 안 함 (--no-target) — 정보 탭에서 지정하세요")
        else:
            try:
                targeted = set_target(api, map_id, reg.get("aps") or [], a.wait)
            except KeyboardInterrupt:
                print("\n측정AP 지정 안 함 (중단) — register_world.py를 다시 실행하거나 정보 탭에서 지정하세요")
                targeted = False
    finally:
        api.call("POST", "/api/logout")
    print(f"완료 — map_id {map_id} (world {wpath}, 제어 PC 지도 경로 {robot_map_path})"
          + ("" if targeted else " · 측정 AP 미지정"))
    sys.exit(0 if targeted else 2)


if __name__ == "__main__":
    main()

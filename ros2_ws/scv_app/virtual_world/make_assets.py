#!/usr/bin/env python3
"""
가상 세계 사진 · 영상 만들기 (3-3차) — world.yaml의 truth가 가리키는 파일을 합성으로 만듦

  판정 사진  truth.facilities.<설비 ID>.photo   설비 종류(register) × 실제 상태(truth)로 그린 640 × 360 JPEG
             normal · abnormal(압력계 빨강 · 유도등 꺼짐 · 셔터 휨) · not_detected(빈 자리) · occluded(상자에 가림)
             · position(옆으로 옮겨짐 + 원래 자리 점선)
  순찰 영상  truth.camera.video                 지도(pgm)를 1인칭으로 그린 순찰 영상 (mp4, 640 × 360, 10 fps)
             가상 로봇이 움직일 때 차례로 내보냄(멈추면 그 장면) — 로봇 위치와 맞추지는 않음

  묶음에 이미 만들어 둔 파일이 있으므로 다시 만들 때만 실행합니다. 같은 이름의 실제 사진 · 영상으로 바꿔 끼워도 됩니다.
  필요 패키지: numpy · pillow · pyyaml (사진), opencv(python3-opencv — 영상)

  사용
    python3 make_assets.py testbed/world.yaml                  # 사진 + 영상
    python3 make_assets.py testbed/world.yaml --photos         # 사진만
    python3 make_assets.py testbed/world.yaml --video --seconds 60
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from world_model import World

W, H = 640, 360
FPS = 10
STATES = ("normal", "abnormal", "not_detected", "occluded", "position")


def font(size: int):
    try:
        return ImageFont.load_default(size=size)              # Pillow 10.1+
    except TypeError:                                          # pragma: no cover — 이전 Pillow
        return ImageFont.load_default()


# =====================================================================
# 판정 사진
# =====================================================================
def background() -> Image.Image:
    """벽(위 72 %) + 바닥 타일(원근)"""
    a = np.zeros((H, W, 3), np.uint8)
    wall_h = int(H * 0.72)
    g = np.linspace(0.9, 1.0, wall_h)[:, None, None]                                # 위가 조금 어두움
    a[:wall_h] = (np.array([226, 222, 210])[None, None, :] * g).astype(np.uint8)
    a[wall_h:] = (150, 152, 156)
    img = Image.fromarray(a)
    d = ImageDraw.Draw(img)
    d.rectangle([0, wall_h - 8, W, wall_h], fill=(120, 116, 108))                     # 걸레받이
    for i in range(-8, 9):                                                            # 바닥 타일 줄 (소실점)
        d.line([(W / 2 + i * 26, wall_h), (W / 2 + i * 150, H)], fill=(132, 134, 138), width=1)
    for yy in (wall_h + 10, wall_h + 24, wall_h + 44, wall_h + 72):
        d.line([(0, yy), (W, yy)], fill=(132, 134, 138), width=1)
    return img


def dashed_rect(d: ImageDraw.ImageDraw, box, color, dash=8):
    x0, y0, x1, y1 = box
    for x in range(int(x0), int(x1), dash * 2):
        d.line([(x, y0), (min(x + dash, x1), y0)], fill=color, width=2)
        d.line([(x, y1), (min(x + dash, x1), y1)], fill=color, width=2)
    for y in range(int(y0), int(y1), dash * 2):
        d.line([(x0, y), (x0, min(y + dash, y1))], fill=color, width=2)
        d.line([(x1, y), (x1, min(y + dash, y1))], fill=color, width=2)


def draw_extinguisher(d, cx, floor_y, gauge: bool, bad: bool):
    """소화기 — 빨간 몸통 · 손잡이 · 호스 · 압력계(초록 바늘 = 정상, 빨강 구역 바늘 = 이상) · 받침"""
    bw, bh = 70, 170
    x0, y0 = cx - bw // 2, floor_y - bh
    d.rounded_rectangle([x0 + 6, floor_y - 6, x0 + bw + 14, floor_y + 4], 4, fill=(110, 110, 110))   # 그림자 · 받침
    d.rounded_rectangle([x0, y0, x0 + bw, floor_y], 22, fill=(200, 28, 32), outline=(120, 10, 14), width=3)
    d.rectangle([x0 + 8, y0 + 60, x0 + bw - 8, y0 + 110], fill=(240, 236, 228))                    # 라벨
    for i in range(4):
        d.line([(x0 + 14, y0 + 70 + i * 10), (x0 + bw - 14, y0 + 70 + i * 10)], fill=(150, 30, 30), width=2)
    d.rectangle([cx - 10, y0 - 22, cx + 10, y0 + 4], fill=(60, 60, 64))                             # 밸브
    d.polygon([(cx - 10, y0 - 22), (cx + 34, y0 - 30), (cx + 34, y0 - 22), (cx - 10, y0 - 14)], fill=(40, 40, 44))
    d.line([(cx + 10, y0 - 10), (x0 + bw + 18, y0 + 30), (x0 + bw + 14, floor_y - 40)], fill=(25, 25, 25), width=6)  # 호스
    if gauge:                                                                                        # 압력계
        gx, gy, r = cx - 26, y0 - 4, 17
        d.ellipse([gx - r, gy - r, gx + r, gy + r], fill=(250, 250, 250), outline=(40, 40, 40), width=3)
        d.arc([gx - r + 4, gy - r + 4, gx + r - 4, gy + r - 4], 200, 250, fill=(220, 40, 40), width=4)
        d.arc([gx - r + 4, gy - r + 4, gx + r - 4, gy + r - 4], 255, 320, fill=(30, 170, 60), width=4)
        ang = math.radians(215 if bad else 290)
        d.line([(gx, gy), (gx + math.cos(ang) * (r - 5), gy + math.sin(ang) * (r - 5))],
               fill=(200, 20, 20) if bad else (20, 20, 20), width=3)
    return [x0 - 30, y0 - 34, x0 + bw + 26, floor_y + 4]


def draw_shutter(d, cx, floor_y, bad: bool):
    """방화 셔터 — 금속 틀 + 가로 판. 이상 = 아래가 휘어 틈이 벌어짐"""
    sw, sh = 360, 230
    x0, y0 = cx - sw // 2, floor_y - sh
    d.rectangle([x0 - 14, y0 - 22, x0 + sw + 14, floor_y], fill=(90, 94, 100))                       # 틀
    d.rectangle([x0 - 10, y0 - 22, x0 + sw + 10, y0], fill=(70, 74, 80))                             # 셔터 상자
    for i in range(0, sh, 14):
        top = y0 + i
        sag = 0
        if bad and i > sh - 70:
            sag = int((i - (sh - 70)) * 0.5)
        d.rectangle([x0, top, x0 + sw, top + 12], fill=(178, 184, 190))
        d.line([(x0, top + 12), (x0 + sw, top + 12)], fill=(120, 126, 132), width=2)
        if sag:
            d.polygon([(x0 + 120, top), (x0 + 240, top), (x0 + 200, top + 12 + sag), (x0 + 160, top + 12 + sag)],
                      fill=(60, 60, 60))
    d.rectangle([cx - 30, floor_y - 18, cx + 30, floor_y - 8], fill=(220, 180, 40))                  # 손잡이 띠
    return [x0 - 14, y0 - 22, x0 + sw + 14, floor_y]


def draw_exit_light(d, cx, wall_y, bad: bool):
    """유도등 — 초록 상자 + 흰 비상구 그림. 이상 = 꺼짐(어두움)"""
    bw, bh = 190, 80
    x0, y0 = cx - bw // 2, wall_y
    d.rectangle([x0 - 4, y0 - 4, x0 + bw + 4, y0 + bh + 4], fill=(60, 60, 60))
    body = (40, 60, 44) if bad else (24, 190, 88)
    d.rectangle([x0, y0, x0 + bw, y0 + bh], fill=body)
    fg = (110, 120, 112) if bad else (250, 255, 250)
    d.rectangle([x0 + bw - 62, y0 + 12, x0 + bw - 22, y0 + bh - 12], outline=fg, width=4)             # 문
    hx, hy = x0 + 62, y0 + 22                                                                        # 달리는 사람
    d.ellipse([hx - 8, hy - 8, hx + 8, hy + 8], fill=fg)
    d.line([(hx, hy + 8), (hx - 6, hy + 30)], fill=fg, width=6)
    d.line([(hx - 6, hy + 30), (hx - 24, hy + 44)], fill=fg, width=5)
    d.line([(hx - 6, hy + 30), (hx + 10, hy + 46)], fill=fg, width=5)
    d.line([(hx - 2, hy + 14), (hx + 22, hy + 22)], fill=fg, width=5)
    d.line([(hx - 2, hy + 14), (hx - 22, hy + 18)], fill=fg, width=5)
    d.polygon([(x0 + 100, y0 + 40), (x0 + 118, y0 + 30), (x0 + 118, y0 + 50)], fill=fg)             # 화살표
    if not bad:                                                                                       # 빛 번짐
        d.rectangle([x0 - 8, y0 + bh + 4, x0 + bw + 8, y0 + bh + 10], fill=(150, 225, 170))
    return [x0 - 4, y0 - 4, x0 + bw + 4, y0 + bh + 4]


def draw_facility(d, ftype, cx, gauge, bad):
    floor_y = int(H * 0.72) + 30
    if ftype == "extinguisher":
        return draw_extinguisher(d, cx, floor_y, gauge, bad)
    if ftype == "shutter":
        return draw_shutter(d, cx, floor_y, bad)
    return draw_exit_light(d, cx, 70, bad)


def draw_boxes(d, box):
    """가림 — 설비 앞에 쌓인 상자"""
    x0, y0, x1, y1 = box
    floor_y = int(H * 0.72) + 34
    w = max(150, (x1 - x0) * 0.9)
    cx = (x0 + x1) / 2
    for i, (bx, bw, bh) in enumerate([(cx - w / 2 - 20, w * 0.62, 150), (cx - w / 2 + w * 0.45, w * 0.6, 120),
                                       (cx - w / 2 + 10, w * 0.5, 110)]):
        by = floor_y - bh - (130 if i == 2 else 0)
        d.rectangle([bx, by, bx + bw, by + bh], fill=(176, 132, 82), outline=(110, 78, 44), width=3)
        d.line([(bx, by + bh * 0.45), (bx + bw, by + bh * 0.45)], fill=(206, 176, 120), width=8)       # 테이프


def make_photo(fid: str, ftype: str, gauge: bool, state: str, out: Path):
    img = background()
    d = ImageDraw.Draw(img)
    cx = W // 2
    if state == "not_detected":                        # 빈 자리 — 걸이 · 받침만
        floor_y = int(H * 0.72) + 30
        if ftype == "exit_light":
            d.rectangle([cx - 30, 70, cx + 30, 80], fill=(120, 120, 120))
        else:
            d.rectangle([cx - 40, floor_y - 6, cx + 40, floor_y + 4], fill=(120, 120, 120))
            d.rectangle([cx - 6, floor_y - 150, cx + 6, floor_y - 120], fill=(130, 130, 130))
        dashed_rect(d, [cx - 70, 50, cx + 70, floor_y], (200, 60, 60))
    elif state == "position":                          # 옆으로 옮겨짐 + 원래 자리 점선
        box = draw_facility(d, ftype, cx + 170, gauge, False)
        w_ = box[2] - box[0]
        dashed_rect(d, [cx - w_ / 2, box[1], cx + w_ / 2, box[3]], (230, 140, 20))
        d.line([(cx + w_ / 2 + 6, (box[1] + box[3]) / 2), (box[0] - 6, (box[1] + box[3]) / 2)], fill=(230, 140, 20), width=3)
    else:
        box = draw_facility(d, ftype, cx, gauge, state == "abnormal")
        if state == "occluded":
            draw_boxes(d, box)
    d.rectangle([0, H - 26, W, H], fill=(20, 20, 20))
    d.text((10, H - 21), f"SCV virtual camera   {fid}  {ftype}{' (gauge)' if gauge else ''}   truth: {state}",
           fill=(235, 235, 235), font=font(14))
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out, format="JPEG", quality=88)


def make_photos(w: World) -> int:
    n = 0
    for fid, t in (w.truth.get("facilities") or {}).items():
        reg = w.facility_reg(fid)
        out = w.file(t.get("photo"))
        if not out:
            print(f"  {fid}: photo 경로가 없어 건너뜀")
            continue
        if not reg:
            print(f"  {fid}: register.facilities에 없어 종류를 몰라 건너뜀")
            continue
        state = t.get("truth", "normal")
        if state not in STATES:
            sys.exit(f"{fid}: truth '{state}'는 {STATES} 중 하나여야 합니다")
        make_photo(fid, reg["type"], bool(reg.get("gauge")), state, out)
        print(f"  사진 {out.relative_to(w.base)} — {fid} {reg['type']} {state}")
        n += 1
    return n


# =====================================================================
# 순찰 영상 — 지도 1인칭 (레이캐스팅)
# =====================================================================
PATROL = [(1.0, 0.9), (1.0, 7.0), (3.0, 7.2), (5.0, 5.8), (5.0, 3.0), (5.2, 1.0), (4.6, 5.6), (7.0, 6.6),
          (7.2, 1.2), (11.0, 1.0), (11.2, 4.4), (10.0, 4.6), (10.0, 7.0), (8.6, 6.6), (10.0, 4.4), (7.6, 3.6),
          (5.6, 6.2), (1.6, 6.0), (1.0, 0.9)]                      # testbed 한 바퀴 (작은 방 문 · 칸막이 끝을 지남)


def patrol_poses(seconds: float, speed=0.6, turn=1.6):
    """경로점을 차례로 (제자리 회전 → 직진) — FPS마다 자세 (x, y, yaw)"""
    poses, x, y = [], *PATROL[0]
    yaw = math.atan2(PATROL[1][1] - y, PATROL[1][0] - x)
    dt = 1.0 / FPS
    i = 1
    while len(poses) < seconds * FPS:
        tx, ty = PATROL[i % len(PATROL)]
        head = math.atan2(ty - y, tx - x)
        err = math.atan2(math.sin(head - yaw), math.cos(head - yaw))
        if abs(err) > 0.03:
            yaw += math.copysign(min(abs(err), turn * dt), err)
        else:
            dist = math.hypot(tx - x, ty - y)
            step = min(dist, speed * dt)
            x, y = x + math.cos(head) * step, y + math.sin(head) * step
            if dist - step < 1e-3:
                i += 1
        poses.append((x, y, yaw))
    return poses


def render_view(w: World, x, y, yaw, rw=320, rh=180, fov=math.radians(72)):
    """레이캐스팅 — 열마다 벽까지 거리 → 벽 높이 · 밝기, 바닥은 1 m 타일"""
    ang = yaw + np.linspace(fov / 2, -fov / 2, rw)
    t = np.arange(0.05, 14.0, 0.01)
    xs = x + np.cos(ang)[:, None] * t[None, :]
    ys = y + np.sin(ang)[:, None] * t[None, :]
    rows = w.h - 1 - np.floor((ys - w.oy) / w.res).astype(int)
    cols = np.floor((xs - w.ox) / w.res).astype(int)
    inside = (rows >= 0) & (rows < w.h) & (cols >= 0) & (cols < w.w)
    hit = np.ones_like(inside)
    hit[inside] = w.wall[rows[inside], cols[inside]]
    k = hit.argmax(axis=1)
    dist = t[k] * np.cos(ang - yaw)                                  # 어안 보정
    hx, hy = xs[np.arange(rw), k], ys[np.arange(rw), k]
    # 벽 면 방향 — 마지막 한 걸음에서 행(y)이 바뀌었으면 가로 벽(y면), 열(x)만 바뀌었으면 세로 벽
    kp = np.maximum(k - 1, 0)
    ri = np.arange(rw)
    r0, c0, r1, c1 = rows[ri, kp], cols[ri, kp], rows[ri, k], cols[ri, k]
    both = (r0 != r1) & (c0 != c1)                                   # 대각선으로 넘어감 — 사이 칸 (r1, c0)이 벽이면 가로 벽
    mid = w.wall[np.clip(r1, 0, w.h - 1), np.clip(c0, 0, w.w - 1)]
    face_y = np.where(both, mid, r0 != r1)
    u = np.where(face_y, hx, hy)
    img = np.zeros((rh, rw, 3), np.float32)
    horizon = rh / 2
    yy = np.arange(rh)[:, None]
    # 천장 · 바닥
    img[: int(horizon)] = np.array([205, 208, 214])[None, None, :] * (0.75 + 0.25 * (1 - yy[: int(horizon)] / horizon))[:, :, None]
    fy = yy[int(horizon):] - horizon + 0.5
    fd = (0.5 * rh * 0.55) / fy                                       # 카메라 높이 0.55 · 화면 줄마다 바닥 거리
    fdx = x + np.cos(ang)[None, :] * fd / np.cos(ang - yaw)[None, :]
    fdy = y + np.sin(ang)[None, :] * fd / np.cos(ang - yaw)[None, :]
    tile = ((np.floor(fdx) + np.floor(fdy)) % 2)[:, :, None]                    # 1 m 바닥 타일 (옅은 체크)
    floor = np.where(tile > 0, np.array([150, 152, 156]), np.array([160, 162, 166])) * np.clip(1.2 - fd[:, :, None] / 12, 0.45, 1.0)
    img[int(horizon):] = floor
    # 벽 — 위 밝은 벽 · 아래 0.9 m 징두리(어두운 띠) · 1 m마다 벽판 이음(가까울수록 진하게, 멀면 사라짐)
    wh = np.clip(rh * 1.1 / np.maximum(dist, 0.05), 0, rh * 4)
    top, bot = horizon - wh / 2, horizon + wh / 2 * 0.95
    shade = np.clip(1.15 - dist / 11, 0.35, 1.0) * np.where(face_y, 1.0, 0.84)
    frac = np.abs(u - np.round(u))
    du = np.abs(np.gradient(u))                                       # 화면 한 열이 덮는 벽 길이 — 크면 이음이 깨져 보이므로 흐리게
    seam = np.clip(1 - frac / 0.025, 0, 1) * np.clip(1 - dist / 5, 0, 1) * np.clip(1 - du / 0.08, 0, 1)
    upper = np.array([226, 222, 210])[None, :] * (1 - 0.35 * seam[:, None])
    lower = np.array([168, 176, 184])[None, :] * (1 - 0.35 * seam[:, None])
    mask = (yy >= top[None, :]) & (yy <= bot[None, :])
    band = yy >= (bot - wh * 0.33)[None, :]
    wall = np.where(band[:, :, None], lower[None, :, :], upper[None, :, :]) * shade[None, :, None]
    img = np.where(mask[:, :, None], wall, img)
    skirt = (yy <= bot[None, :]) & (yy >= bot[None, :] - wh[None, :] * 0.05)       # 걸레받이
    img = np.where(skirt[:, :, None], (np.array([96, 92, 86]) * shade[:, None])[None, :, :], img)
    return np.clip(img, 0, 255).astype(np.uint8)


def make_video(w: World, out: Path, seconds: float) -> int:
    try:
        import cv2
    except ImportError:
        sys.exit("영상을 만들려면 OpenCV가 필요합니다 (sudo apt install python3-opencv 또는 pip install opencv-python)")
    out.parent.mkdir(parents=True, exist_ok=True)
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not vw.isOpened():
        sys.exit(f"영상 파일을 열지 못했습니다: {out}")
    poses = patrol_poses(seconds)
    for i, (x, y, yaw) in enumerate(poses):
        rgb = render_view(w, x, y, yaw)
        frame = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (W, H), interpolation=cv2.INTER_LINEAR)
        cv2.putText(frame, f"SCV virtual patrol  x {x:5.2f}  y {y:5.2f}", (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (40, 40, 40), 1, cv2.LINE_AA)
        vw.write(frame)
    vw.release()
    return len(poses)


def main():
    ap = argparse.ArgumentParser(description="가상 세계 사진 · 영상 만들기")
    ap.add_argument("world", help="world.yaml (예: testbed/world.yaml)")
    ap.add_argument("--photos", action="store_true", help="사진만")
    ap.add_argument("--video", action="store_true", help="영상만")
    ap.add_argument("--seconds", type=float, default=45.0, help="영상 길이(초, 기본 45 — 한 바퀴)")
    a = ap.parse_args()
    w = World(a.world)
    both = not a.photos and not a.video
    if a.photos or both:
        print(f"판정 사진 — {w.path}")
        print(f"  {make_photos(w)}장")
    if a.video or both:
        out = w.file((w.truth.get("camera") or {}).get("video"))
        if not out:
            sys.exit("truth.camera.video 경로가 없습니다")
        n = make_video(w, out, a.seconds)
        print(f"순찰 영상 {out.relative_to(w.base)} — {n}장 ({n / FPS:g}초, {out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()

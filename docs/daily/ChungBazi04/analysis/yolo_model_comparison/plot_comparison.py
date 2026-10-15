#!/usr/bin/env python3
"""Plot the three actual training runs without re-running inference.

Usage:
    python3 plot_comparison.py

Inputs: metrics_summary.json, the results.csv selected by each summary run_dir,
and complexity_evidence.json (fused checkpoint counts measured separately).
Class AP50 values are explicitly transcribed from saved test PR-curve legends
and have only three decimal places, unlike the exact aggregate JSON metrics.
All output figures use complete zero-based percentage or size scales.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
import numpy as np


OUT = Path(__file__).resolve().parent
FIGURES = OUT.parents[1] / "img"
EVIDENCE = OUT / "evidence"
SUMMARY = EVIDENCE / "metrics_summary.json"
FONT = Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
FONT_BOLD = Path("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf")
for path in (FONT, FONT_BOLD):
    if path.exists():
        font_manager.fontManager.addfont(str(path))
plt.rcParams.update({
    "font.family": "NanumGothic" if FONT.exists() else "DejaVu Sans",
    "font.size": 12,
    "axes.titlesize": 16,
    "axes.titleweight": "bold",
    "axes.labelsize": 12,
    "axes.unicode_minus": False,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.edgecolor": "#BCC6D1",
    "axes.labelcolor": "#334155",
    "xtick.color": "#334155",
    "ytick.color": "#334155",
    "text.color": "#172F45",
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "svg.fonttype": "path",
})
ORDER = ["yolov8n", "yolo11n", "yolo26n"]
LABELS = ["YOLOv8n", "YOLO11n", "YOLO26n"]
EXPECTED_RUNS = {"yolov8n": "yolov8n", "yolo11n": "yolo11n", "yolo26n": "yolo26n-2"}
COLORS = ["#247BA0", "#D97924", "#127C5B"]
rows = {row["model"]: row for row in json.loads(SUMMARY.read_text())}
data = [rows[name] for name in ORDER]
curves = []
for row in data:
    expected = EXPECTED_RUNS[row["model"]]
    if Path(row["run_dir"]).name != expected:
        raise ValueError(f"{row['model']} must use run {expected}: {row['run_dir']}")
    if Path(row["best_weights"]).parent.parent.name != expected:
        raise ValueError(f"{row['model']} must use the best checkpoint from {expected}")
    # Prefer frozen CSV evidence, preserving the selected execution identity.
    run_dir = EVIDENCE / "runs" / Path(row["run_dir"]).name
    with (run_dir / "results.csv").open(newline="") as f:
        curve = [{k.strip(): float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    curves.append(curve)


def header(fig, title, subtitle):
    """그림 위에 제목과 설명, 분석에 사용한 실행 이름을 표시한다."""
    fig.text(0.065, 0.947, title, fontsize=23, fontweight="bold", va="top")
    fig.text(0.065, 0.892, subtitle, fontsize=12, color="#536778", va="top")
    fig.text(0.96, 0.972, "YOLO26n 분석 실행: yolo26n-2", fontsize=10,
             color="#536778", ha="right", va="top")


def legend(fig, y=0.847):
    """세 모델의 색상과 이름을 그림의 범례에 표시한다."""
    fig.legend([Patch(facecolor=c) for c in COLORS], LABELS,
               loc="center", bbox_to_anchor=(0.5, y), ncol=3,
               frameon=False, handlelength=1.2, columnspacing=3.5)


def axis_style(ax, ymax=100):
    """비교가 과장되지 않도록 세로축을 0부터 그리고 눈금선을 표시한다."""
    ax.set_ylim(0, ymax)
    ax.set_axisbelow(True)
    ax.grid(axis="y", color="#E7EDF1", linewidth=0.8)
    ax.tick_params(length=0, pad=8)


def save(fig, stem):
    """완성한 그림을 작성자 img 폴더에 10월 14일 자료 이름으로 저장한다."""
    FIGURES.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURES / f"2026-10-14-yolo-{stem}.png", dpi=180)
    fig.savefig(FIGURES / f"2026-10-14-yolo-{stem}.svg")
    plt.close(fig)


# Figure 1: exact aggregate metrics from the separately evaluated best.pt.
fig, axes = plt.subplots(1, 2, figsize=(16, 9))
fig.subplots_adjust(left=0.065, right=0.96, top=0.74, bottom=0.24, wspace=0.22)
header(fig, "객체 탐지 성능 비교", "best.pt 재평가 결과 | 검증 71장 · 테스트 33장 | 5개 객체 클래스")
legend(fig, 0.827)
metric_keys = ["metrics/mAP50-95(B)", "metrics/mAP50(B)"]
width = 0.235
for ax, split, title in zip(axes, ["val", "test"], ["검증 세트 (Validation)", "테스트 세트 (Test)"]):
    for i, row in enumerate(data):
        values = [100 * row[split][key] for key in metric_keys]
        bars = ax.bar(np.arange(2) + (i - 1) * width, values, width,
                      color=COLORS[i], edgecolor="white", linewidth=0.7)
        ax.bar_label(bars, fmt="%.2f", padding=4, fontsize=11, fontweight="bold")
    axis_style(ax)
    ax.set_yticks(range(0, 101, 20))
    ax.set_xticks([0, 1], ["mAP50–95", "mAP50"])
    ax.set_ylabel("mAP (%)")
    ax.set_title(title, pad=22)
    ax.set_xlim(-0.55, 1.55)
fig.text(0.065, 0.149, "검증 mAP50–95: YOLO26n 89.26% (최고)   |   테스트 mAP50–95: YOLOv8n 90.32% (최고)",
         fontsize=14, fontweight="bold")
fig.text(0.065, 0.095, "출처: runs/fire_extinguisher/metrics_summary.json · YOLO26n은 yolo26n-2 실행 결과", fontsize=10, color="#536778")
fig.text(0.065, 0.065, "단일 학습·단일 분할의 기술 통계. 객체 탐지 mAP이며 점등 상태·압력 정상 여부·박스 돌출 판단 정확도를 직접 측정하지 않음.",
         fontsize=10, color="#536778")
save(fig, "01_detection_metrics")


# Figure 2: raw epochs, no smoothing and no extension of the stopped 26n run.
fig, axes = plt.subplots(1, 2, figsize=(16, 9))
fig.subplots_adjust(left=0.065, right=0.96, top=0.74, bottom=0.30, wspace=0.22)
header(fig, "학습 중 검증 지표의 변화", "results.csv의 epoch별 원시 값 | 보간·평활화 없음 | YOLO26n은 86 epoch에서 종료")
legend(fig, 0.827)
best_epochs = []
for i, curve in enumerate(curves):
    epochs = np.array([r["epoch"] for r in curve])
    for ax, key, title in zip(axes,
                             ["metrics/mAP50-95(B)", "metrics/recall(B)"],
                             ["검증 mAP50–95", "검증 Recall"]):
        values = 100 * np.array([r[key] for r in curve])
        ax.plot(epochs, values, color=COLORS[i], linewidth=1.7, alpha=0.92)
        ax.scatter(epochs[-1], values[-1], s=32, color=COLORS[i], zorder=4)
        axis_style(ax)
        ax.set_xlim(1, 100)
        ax.set_xticks([1, 20, 40, 60, 80, 100])
        ax.set_yticks(range(0, 101, 20))
        ax.set_xlabel("Epoch")
        ax.set_ylabel("%")
        ax.set_title(title, pad=22)
    scores = np.array([r["metrics/mAP50-95(B)"] for r in curve])
    best = int(np.argmax(scores))
    best_epochs.append((int(epochs[best]), 100 * scores[best]))
    axes[0].scatter(epochs[best], 100 * scores[best], marker="D", s=72,
                    color=COLORS[i], edgecolors="white", linewidth=1, zorder=5)
for i, (epoch, score) in enumerate(best_epochs):
    x = 0.065 + 0.31 * i
    fig.text(x, 0.203, LABELS[i], fontsize=13, fontweight="bold", color=COLORS[i])
    fig.text(x, 0.167, f"최고 mAP50–95: {score:.2f}% / epoch {epoch}", fontsize=12)
fig.text(0.065, 0.095, "◆: CSV에서 mAP50–95가 가장 높았던 epoch · ●: 마지막 epoch · 적은 epoch 수만으로 실제 학습·추론 시간 우위를 의미하지 않음.", fontsize=10, color="#536778")
fig.text(0.065, 0.065, "출처: summary에 지정된 각 실행의 results.csv. 학습 중 CSV 값과 best.pt 재평가 JSON 값은 소폭 다르므로 구분해 표시함.", fontsize=10, color="#536778")
save(fig, "02_training_curves")


# Figure 3: rounded AP values visible in the saved PR-curve legends.
ap50 = np.array([
    [.992, .995, .995, .995, .932],
    [.994, .995, .995, .995, .873],
    [.995, .995, .995, .995, .923],
]) * 100
fig, ax = plt.subplots(figsize=(16, 9))
fig.subplots_adjust(left=0.15, right=0.88, top=0.70, bottom=0.28)
header(fig, "테스트 세트의 클래스별 탐지 AP50", "저장된 test PR 곡선 범례에서 읽은 반올림 값 | 테스트 33장 | 색상 범위 0–100%")
im = ax.imshow(ap50, cmap="Blues", vmin=0, vmax=100, aspect="auto")
ax.set_xticks(range(5), ["박스\nbox", "비상구 표지\nexit_sign", "소화기\nfire_extinguisher", "압력계\npressure_gauge", "랙\nrack"])
ax.set_yticks(range(3), LABELS)
ax.tick_params(axis="both", length=0, pad=13, labelsize=13)
ax.set_xticks(np.arange(-.5, 5, 1), minor=True)
ax.set_yticks(np.arange(-.5, 3, 1), minor=True)
ax.grid(which="minor", color="white", linewidth=4)
ax.tick_params(which="minor", bottom=False, left=False)
for spine in ax.spines.values():
    spine.set_visible(False)
for i in range(3):
    for j in range(5):
        ax.text(j, i, f"{ap50[i, j]:.1f}", ha="center", va="center",
                fontsize=23, color="white", fontweight="bold")
cax = fig.add_axes([0.91, 0.28, 0.017, 0.42])
cbar = fig.colorbar(im, cax=cax)
cbar.set_label("AP50 (%)", labelpad=10)
cbar.set_ticks(range(0, 101, 20))
cbar.outline.set_visible(False)
fig.text(0.065, 0.158, "YOLO26n은 랙 AP50 92.3%로 YOLO11n(87.3%)보다 높고, YOLOv8n(93.2%)보다 낮음.", fontsize=14, fontweight="bold")
fig.text(0.065, 0.102, "출처: 각 모델의 저장된 test PR 곡선 범례 (원본 소수 3자리 → 여기서는 % 소수 1자리). 반올림된 값이므로 정밀한 동률 판단에 사용하지 않음.", fontsize=10, color="#536778")
fig.text(0.065, 0.067, "AP50은 객체 클래스 탐지 지표. 비상구 ON/OFF, 압력 정상/이상, 박스 돌출 여부의 판단 정확도는 별도 평가가 필요함.", fontsize=10, color="#536778")
save(fig, "03_test_class_ap50")


# Figure 4: actual fused checkpoint complexity, not a timing measurement.
complexity = json.loads((OUT / "complexity_evidence.json").read_text())
comp_by_run = {row["run"]: row for row in complexity}
comp = [comp_by_run[Path(row["run_dir"]).name] for row in data]
fig, axes = plt.subplots(1, 2, figsize=(16, 9))
fig.subplots_adjust(left=0.065, right=0.96, top=0.74, bottom=0.28, wspace=0.22)
header(fig, "실제 학습 체크포인트의 모델 크기와 연산량", "동일 5클래스 체크포인트, fuse 후 계산 | 입력 640 × 640 | FPS 실측 아님")
for ax, key, title, scale, upper, unit, fmt in [
    (axes[0], "fused_parameters", "파라미터 수", 1e6, 3.6, "백만 개 (M)", "%.3f"),
    (axes[1], "fused_gflops_640", "연산량", 1, 10, "GFLOPs", "%.2f"),
]:
    vals = np.array([r[key] for r in comp]) / scale
    bars = ax.bar(np.arange(3), vals, color=COLORS, width=0.58)
    ax.bar_label(bars, fmt=fmt, padding=8, fontsize=14, fontweight="bold")
    axis_style(ax, upper)
    ax.set_xticks(range(3), LABELS)
    ax.set_ylabel(unit)
    ax.set_title(title, pad=22)
    ax.set_xlim(-0.6, 2.6)
parameter_reduction = 100 * (1 - comp[2]["fused_parameters"] / comp[0]["fused_parameters"])
flops_reduction = 100 * (1 - comp[2]["fused_gflops_640"] / comp[0]["fused_gflops_640"])
fig.text(0.065, 0.177, f"YOLO26n: YOLOv8n 대비 파라미터 {parameter_reduction:.2f}% 감소 · 연산량 {flops_reduction:.2f}% 감소", fontsize=15, fontweight="bold")
fig.text(0.065, 0.115, "구현의 경량화 근거로 사용할 수 있음. 실제 지연시간·FPS·메모리 사용량은 배포 장치에서 별도 측정해야 함.", fontsize=11, color="#536778")
fig.text(0.065, 0.078, "출처: complexity_evidence.json · 실제 YOLO26n 체크포인트 end2end=False. 이 비교는 NMS-free 추론 성능을 입증하지 않음.", fontsize=10, color="#536778")
save(fig, "04_model_complexity")

print("Generated four figures in PNG and SVG:")
for path in sorted(FIGURES.glob("2026-10-14-yolo-*")):
    print(path)

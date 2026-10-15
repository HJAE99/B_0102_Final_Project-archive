#!/usr/bin/env python3
"""Reproduce CPU-only model-complexity measurements; never trains or evaluates data.

Run with the project's Python environment:
  /home/hv-05/venvs/rokey_venv/bin/python measure_complexity.py
  /home/hv-05/venvs/rokey_venv/bin/python measure_complexity.py --root /path/to/finalproj.v5

The printed GFLOPs are Ultralytics/THOP operation estimates at 640x640, not a
runtime benchmark. Fusion is performed only on an in-memory copy of each model;
original checkpoints are neither overwritten nor exported.
"""

import argparse
import json
import os
from pathlib import Path

# Disable CUDA before importing torch/Ultralytics.
os.environ["CUDA_VISIBLE_DEVICES"] = ""

import torch
import ultralytics
from ultralytics import YOLO
from ultralytics.utils.torch_utils import get_flops


def main():
    """지정한 학습 폴더의 세 모델을 CPU에서 비교하고 모델 규모와 클래스 수를 출력한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/home/hv-05/Downloads/finalproj.v5"))
    args = parser.parse_args()
    torch.set_num_threads(2)
    rows = []
    for name in ("yolov8n", "yolo11n", "yolo26n-2"):
        path = args.root / "runs" / "fire_extinguisher" / name / "weights" / "best.pt"
        model = YOLO(str(path)).model.float().cpu().eval()
        head = model.model[-1]
        row = {
            "run": name,
            "checkpoint": str(path.resolve()),
            "unfused_parameters": sum(p.numel() for p in model.parameters()),
            "unfused_one2one_head_parameters": sum(
                p.numel()
                for module in (
                    getattr(head, "one2one_cv2", None),
                    getattr(head, "one2one_cv3", None),
                )
                if module is not None
                for p in module.parameters()
            ),
            "end2end": model.end2end,
            "head_end2end_attribute": getattr(head, "_end2end", None),
            "reg_max": head.reg_max,
            "classes": head.nc,
        }
        model.fuse(verbose=False)
        row.update({
            "fused_parameters": sum(p.numel() for p in model.parameters()),
            "fused_gflops_640": get_flops(model, imgsz=640),
        })
        rows.append(row)

    label_counts = {}
    names = ["box", "exit_sign", "fire_extinguisher", "pressure_gauge", "rack"]
    for split in ("train", "valid", "test"):
        files = list((args.root / split / "labels").glob("*.txt"))
        counts = dict.fromkeys(names, 0)
        images = dict.fromkeys(names, 0)
        for path in files:
            present = set()
            for line in path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    name = names[int(line.split()[0])]
                    counts[name] += 1
                    present.add(name)
            for name in present:
                images[name] += 1
        label_counts[split] = {"label_files": len(files), "instances": counts, "images_with_class": images}
    print(json.dumps({
        "ultralytics_version": ultralytics.__version__,
        "torch_version": torch.__version__,
        "device": "cpu",
        "method": "FP32 model.float().cpu().eval(); model.fuse(); parameter numel sum; Ultralytics get_flops(imgsz=640)",
        "models": rows,
        "label_counts": label_counts,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

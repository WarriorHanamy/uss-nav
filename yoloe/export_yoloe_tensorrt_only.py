#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure YOLO/YOLOE TensorRT export script.

Removed:
- rospy / ROS topics / ROS messages
- cv_bridge / message_filters
- odom buffer / RGB-D sync
- inference loop / publishing / visualization
- mobileclip text feature runtime path

Kept:
- load YOLO/YOLOE model
- optionally set text prompt classes
- export TensorRT engine
"""

import argparse
from pathlib import Path

import torch
from ultralytics import YOLO


def read_prompt_classes(prompt_file: str):
    """Read one class name per line from prompt file."""
    path = Path(prompt_file)
    if not path.exists():
        raise FileNotFoundError(f"Prompt file not found: {path}")

    names = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not names:
        raise ValueError(f"Prompt file is empty: {path}")
    return names


def build_model(model_path: str, prompt_file: str | None = None, device: str = "cuda:0"):
    """
    Load YOLO/YOLOE model.

    If prompt_file is provided, set YOLOE text prompt classes before export.
    """
    model_path = str(model_path)

    if model_path.endswith(".engine"):
        raise ValueError(
            f"Input model is already a TensorRT engine: {model_path}\n"
            "Please use a .pt model as input for export."
        )

    model = YOLO(model_path)

    if device.startswith("cuda"):
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available, cannot export TensorRT engine on GPU.")
        model = model.cuda()

    if prompt_file:
        names = read_prompt_classes(prompt_file)
        print(f"[PROMPT] Loaded {len(names)} classes from {prompt_file}")
        # YOLOE text-prompt export path.
        model.set_classes(names, model.get_text_pe(names))

    return model


def warmup_model(model, imgsz=(480, 640), device="cuda:0", loops: int = 2):
    """Optional lightweight warmup before export."""
    import cv2
    import numpy as np

    h, w = imgsz
    dummy = np.zeros((h, w, 3), dtype=np.uint8)

    print(f"[WARMUP] loops={loops}, imgsz={imgsz}, device={device}")
    with torch.no_grad():
        for _ in range(loops):
            model.predict(
                dummy,
                conf=0.3,
                imgsz=imgsz,
                device=device,
                save=False,
                verbose=False,
            )


def export_engine(args):
    model = build_model(args.model, args.prompt_file, args.device)

    if args.warmup:
        warmup_model(model, imgsz=(args.height, args.width), device=args.device, loops=args.warmup_loops)

    print("[EXPORT] Start TensorRT export")
    print(f"[EXPORT] model      = {args.model}")
    print(f"[EXPORT] imgsz      = ({args.height}, {args.width})")
    print(f"[EXPORT] dynamic    = {args.dynamic}")
    print(f"[EXPORT] half       = {args.half}")
    print(f"[EXPORT] simplify   = {args.simplify}")
    print(f"[EXPORT] workspace  = {args.workspace}")

    export_kwargs = dict(
        format="engine",
        imgsz=(args.height, args.width),
        dynamic=args.dynamic,
        half=args.half,
        simplify=args.simplify,
    )

    if args.workspace is not None:
        export_kwargs["workspace"] = args.workspace

    engine_path = model.export(**export_kwargs)
    print(f"[EXPORT] Done: {engine_path}")
    return engine_path


def parse_args():
    parser = argparse.ArgumentParser(description="Export YOLO/YOLOE .pt model to TensorRT .engine")

    parser.add_argument(
        "--model",
        default="./yoloe-v8m-seg.pt",
        help="Input .pt model path, for example ./yoloe-v8m-seg.pt",
    )
    parser.add_argument(
        "--prompt-file",
        default="./prompt/prompt2.txt",
        help="Text prompt file. One class per line. Use empty string to disable.",
    )
    parser.add_argument(
        "--device",
        default="cuda:0",
        help="Export device, usually cuda:0 on Jetson",
    )

    parser.add_argument("--height", type=int, default=480, help="Export input height")
    parser.add_argument("--width", type=int, default=640, help="Export input width")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic shape. Default is static shape.")
    parser.add_argument("--no-half", dest="half", action="store_false", help="Disable FP16 export")
    parser.set_defaults(half=True)
    parser.add_argument("--no-simplify", dest="simplify", action="store_false", help="Disable ONNX simplify")
    parser.set_defaults(simplify=True)
    parser.add_argument("--workspace", type=float, default=None, help="TensorRT workspace size in GiB, optional")

    parser.add_argument("--warmup", action="store_true", help="Run lightweight warmup before export")
    parser.add_argument("--warmup-loops", type=int, default=2, help="Warmup loop count")

    args = parser.parse_args()

    if args.prompt_file == "":
        args.prompt_file = None

    return args


if __name__ == "__main__":
    export_engine(parse_args())

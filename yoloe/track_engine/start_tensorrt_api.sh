#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLOE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${YOLOE_ROOT}"

python track_engine/tensorrt-api.py \
  --pt-model "${YOLOE_ROOT}/yoloe-v8m-seg-test.pt" \
  --engine "${YOLOE_ROOT}/yoloe-v8m-seg-test.engine" \
  --classes "${YOLOE_ROOT}/prompt/prompt2.txt" \
  --tracker-dir "${YOLOE_ROOT}/ultralytics/cfg/trackers" \
  --host "127.0.0.1" \
  --port 2250 \
  --conf 0.1 \
  --iou 0.5 \
  --imgsz 480,640 \
  --engine-imgsz 480,640 \
  "$@"

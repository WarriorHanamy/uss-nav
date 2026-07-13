#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FAST_REID_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${FAST_REID_ROOT}"

python3 tools/deploy/onnx_export.py \
  --config-file configs/Market1501/bagtricks_R50.yml \
  --name market_bot_R50 \
  --output outputs/onnx_model \
  --batch-size 16 \
  --opts MODEL.WEIGHTS weights/market_bot_R50.pth MODEL.DEVICE cuda:0

export PYTHONPATH=$PWD:$PYTHONPATH
PYTHONPATH=$PWD:$PYTHONPATH python3 tools/deploy/trt_export.py \
  --name market_bot_R50 \
  --output outputs/trt_model \
  --mode fp16 \
  --batch-size 16 \
  --height 256 \
  --width 128 \
  --onnx-model outputs/onnx_model/market_bot_R50.onnx
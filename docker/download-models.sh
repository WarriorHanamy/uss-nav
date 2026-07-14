#!/usr/bin/env bash
# Download YOLOE and MobileCLIP model weights for the release image.
# Usage:
#   ./docker/download-models.sh [output_dir]
#   docker compose run --rm release -c "bash /download-models.sh /workspace/.pretrained"
#
# Default output directory: ./.pretrained (relative to project root)
set -euo pipefail

MODEL_DIR="${1:-$(dirname "$0")/../.pretrained}"
mkdir -p "$MODEL_DIR"

echo "Downloading model weights to: $(cd "$MODEL_DIR" && pwd)"

# YOLOE-11m segmentation (prompt + prompt-free)
# Source: https://huggingface.co/jameslahm/yoloe
if [ ! -f "$MODEL_DIR/yoloe-11m-seg.pt" ]; then
  echo "Downloading yoloe-11m-seg.pt..."
  wget -q --show-progress \
    https://huggingface.co/jameslahm/yoloe/resolve/main/yoloe-11m-seg.pt \
    -O "$MODEL_DIR/yoloe-11m-seg.pt"
else
  echo "yoloe-11m-seg.pt already exists, skipping."
fi

if [ ! -f "$MODEL_DIR/yoloe-11m-seg-pf.pt" ]; then
  echo "Downloading yoloe-11m-seg-pf.pt..."
  wget -q --show-progress \
    https://huggingface.co/jameslahm/yoloe/resolve/main/yoloe-11m-seg-pf.pt \
    -O "$MODEL_DIR/yoloe-11m-seg-pf.pt"
else
  echo "yoloe-11m-seg-pf.pt already exists, skipping."
fi

# MobileCLIP text encoder (blt variant)
# Source: https://docs-assets.developer.apple.com/ml-research/datasets/mobileclip/
if [ ! -f "$MODEL_DIR/mobileclip_blt.pt" ]; then
  echo "Downloading mobileclip_blt.pt..."
  wget -q --show-progress \
    https://docs-assets.developer.apple.com/ml-research/datasets/mobileclip/mobileclip_blt.pt \
    -O "$MODEL_DIR/mobileclip_blt.pt"
else
  echo "mobileclip_blt.pt already exists, skipping."
fi

echo ""
echo "Done. Files in $(cd "$MODEL_DIR" && pwd):"
ls -lh "$MODEL_DIR"/*.pt 2>/dev/null || echo "(no .pt files found)"

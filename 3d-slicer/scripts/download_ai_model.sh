#!/usr/bin/env bash
# Downloads the Obico pre-trained print failure detection model (MIT license).
# Model is ~25 MB — not committed to git, downloaded at setup time.

set -euo pipefail

DEST_DIR="$(dirname "$0")/../backend/monitoring/models"
mkdir -p "$DEST_DIR"

echo "Downloading Obico failure detection model..."

# Obico publishes model weights in their GitHub releases.
# The model is an EfficientDet variant exported to ONNX.
OBICO_MODEL_URL="https://github.com/TheSpaghettiDetective/obico-server/releases/download/1.9.0/model.onnx"

curl -fL --progress-bar -o "$DEST_DIR/failure_detector.onnx" "$OBICO_MODEL_URL"

echo ""
echo "Model downloaded to $DEST_DIR/failure_detector.onnx"
echo "The AI failure detector is now ready."

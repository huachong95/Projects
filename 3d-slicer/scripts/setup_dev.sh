#!/usr/bin/env bash
set -euo pipefail

echo "=== 3D Slicer Development Setup ==="
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$SCRIPT_DIR/.."

# Python backend
echo "1. Setting up Python backend..."
cd "$ROOT_DIR/backend"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
echo "   Backend dependencies installed."

# Three.js viewer assets
echo ""
echo "2. Downloading Three.js viewer assets..."
VIEWER_DIR="$ROOT_DIR/frontend/assets/viewer"
mkdir -p "$VIEWER_DIR"
THREE_VERSION="r160"
curl -fL -o "$VIEWER_DIR/three.min.js" \
  "https://cdnjs.cloudflare.com/ajax/libs/three.js/${THREE_VERSION}/three.min.js"
curl -fL -o "$VIEWER_DIR/STLLoader.js" \
  "https://raw.githubusercontent.com/mrdoob/three.js/${THREE_VERSION}/examples/js/loaders/STLLoader.js"
curl -fL -o "$VIEWER_DIR/OrbitControls.js" \
  "https://raw.githubusercontent.com/mrdoob/three.js/${THREE_VERSION}/examples/js/controls/OrbitControls.js"
echo "   Three.js assets downloaded."

# CuraEngine
echo ""
echo "3. Downloading CuraEngine..."
bash "$SCRIPT_DIR/download_curaengine.sh" || echo "   WARNING: CuraEngine download failed. Download manually."

# Definition files
echo ""
echo "4. Downloading CuraEngine definition files..."
bash "$SCRIPT_DIR/download_definitions.sh" || echo "   WARNING: Definition file download failed. Download manually."

# Flutter
echo ""
echo "5. Setting up Flutter app..."
cd "$ROOT_DIR/frontend"
if command -v flutter &> /dev/null; then
  # Generate platform directories if they don't exist
  if [ ! -d "windows" ]; then
    flutter create --platforms=windows,ios . --project-name slicer_app
    echo "   Flutter platform directories created."
  fi
  flutter pub get
  echo "   Flutter dependencies installed."
else
  echo "   Flutter not found. Install from https://flutter.dev/docs/get-started/install"
  echo "   Then run: cd frontend && flutter create --platforms=windows,ios . && flutter pub get"
fi

echo ""
echo "=== Setup complete ==="
echo ""
echo "To start the backend:"
echo "  cd backend && source .venv/bin/activate && python main.py"
echo ""
echo "To start the Flutter app (Windows):"
echo "  cd frontend && flutter run -d windows"
echo ""
echo "Definition files are downloaded automatically by this script."
echo "CuraEngine binary is downloaded automatically by this script."

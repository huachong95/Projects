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
# NOTE: these are already vendored in the repo (committed, not gitignored).
# This step only refreshes them. r128 is REQUIRED: three_viewer.html loads
# the libraries via plain <script> tags and uses the global THREE.STLLoader
# / THREE.OrbitControls namespace. That non-module build only exists in
# examples/js/ up to ~r147; r148+ moved to ES-module examples/jsm/ which
# does not attach to the global THREE and would break the viewer.
echo ""
echo "2. Refreshing Three.js viewer assets (r128)..."
VIEWER_DIR="$ROOT_DIR/frontend/assets/viewer"
mkdir -p "$VIEWER_DIR"
curl -fL -o "$VIEWER_DIR/three.min.js" \
  "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
curl -fL -o "$VIEWER_DIR/STLLoader.js" \
  "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"
curl -fL -o "$VIEWER_DIR/OrbitControls.js" \
  "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"
echo "   Three.js assets refreshed."

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
  # Generate platform directories if they don't exist.
  # Pick the right platform for the host OS — flutter create fails if asked
  # for a platform it cannot build on the current machine (e.g. windows on
  # Linux, ios on Linux/Windows).
  if [ ! -d "windows" ] && [ ! -d "linux" ] && [ ! -d "macos" ]; then
    case "$(uname -s)" in
      Darwin*) flutter create --platforms=macos . --project-name slicer_app ;;
      Linux*)  flutter create --platforms=linux  . --project-name slicer_app ;;
      *)       flutter create --platforms=windows . --project-name slicer_app ;;
    esac
    echo "   Flutter platform directories created."
  fi
  flutter pub get
  echo "   Flutter dependencies installed."
else
  echo "   Flutter not found. Install from https://flutter.dev/docs/get-started/install"
  echo "   Then run: cd frontend && flutter pub get"
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

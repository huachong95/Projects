#!/usr/bin/env bash
set -euo pipefail

echo "=== 3D Slicer Development Setup ==="
echo ""

# Python backend
echo "1. Setting up Python backend..."
cd "$(dirname "$0")/../backend"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
echo "   Backend dependencies installed."
cd ..

# CuraEngine
echo ""
echo "2. Downloading CuraEngine..."
bash scripts/download_curaengine.sh || echo "   WARNING: CuraEngine download failed. Download manually."

# Flutter
echo ""
echo "3. Checking Flutter..."
if command -v flutter &> /dev/null; then
  cd frontend
  flutter pub get
  echo "   Flutter dependencies installed."
  cd ..
else
  echo "   Flutter not found. Install from https://flutter.dev/docs/get-started/install"
fi

echo ""
echo "=== Setup complete ==="
echo ""
echo "To start the backend:"
echo "  cd backend && source .venv/bin/activate && python main.py"
echo ""
echo "To start the Flutter app (Windows):"
echo "  cd frontend && flutter run -d windows"

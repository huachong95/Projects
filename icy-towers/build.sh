#!/bin/bash
# Linux build / test script
set -e

echo "=== Icy Towers Build ==="

echo "[1/3] Installing dependencies..."
pip install pygame pyinstaller

echo "[2/3] Building executable..."
pyinstaller icy_towers.spec --clean

echo "[3/3] Done! Executable is in dist/"
echo "Run: ./dist/IcyTowers"

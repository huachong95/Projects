#!/usr/bin/env bash
# Downloads the CuraEngine binary for Linux.
# CuraEngine is AGPL-licensed — we download at runtime, never commit the binary.

set -euo pipefail

CURA_VERSION="5.7.2"
DEST_DIR="$(dirname "$0")/../backend/slicer/cura_engine/linux"
BINARY_URL="https://github.com/Ultimaker/CuraEngine/releases/download/${CURA_VERSION}/CuraEngine-${CURA_VERSION}-Linux-X64.AppImage"

mkdir -p "$DEST_DIR"

echo "Downloading CuraEngine ${CURA_VERSION} for Linux..."
curl -L -o "$DEST_DIR/CuraEngine" "$BINARY_URL"
chmod +x "$DEST_DIR/CuraEngine"

echo "CuraEngine downloaded to $DEST_DIR/CuraEngine"
echo ""
echo "NOTE: You also need the printer definition files."
echo "Download fdmprinter.def.json and fdmextruder.def.json from:"
echo "  https://github.com/Ultimaker/Cura/tree/${CURA_VERSION}/resources/definitions"
echo "and place them in: backend/slicer/cura_profiles/definitions/"

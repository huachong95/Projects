#!/usr/bin/env bash
# Downloads CuraEngine printer-definition JSON files needed for slicing.
# These files are MIT-licensed and come from the Ultimaker/Cura repository.

set -euo pipefail

CURA_VERSION="5.7.2"
DEST_DIR="$(dirname "$0")/../backend/slicer/cura_profiles/definitions"
BASE_URL="https://raw.githubusercontent.com/Ultimaker/Cura/${CURA_VERSION}/resources/definitions"

mkdir -p "$DEST_DIR"

echo "Downloading CuraEngine definition files (Cura ${CURA_VERSION})..."

curl -fL -o "$DEST_DIR/fdmprinter.def.json"  "$BASE_URL/fdmprinter.def.json"
curl -fL -o "$DEST_DIR/fdmextruder.def.json" "$BASE_URL/fdmextruder.def.json"

echo ""
echo "Definition files saved to: $DEST_DIR"
echo "  fdmprinter.def.json   — base machine definition"
echo "  fdmextruder.def.json  — extruder definition"

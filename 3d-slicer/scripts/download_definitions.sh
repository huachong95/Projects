#!/usr/bin/env bash
# Downloads CuraEngine printer-definition JSON files needed for slicing.
# These files are MIT-licensed and come from the Ultimaker/Cura repository.
# NOTE: The definitions are already vendored in the repo (committed). This
# script only needs to be run if you want to refresh them to a newer version.

set -euo pipefail

# Use the main branch — the 5.7.2 tag does not include resources/definitions/
# in the raw.githubusercontent path (it returns 404).
BASE_URL="https://raw.githubusercontent.com/Ultimaker/Cura/main/resources/definitions"
DEST_DIR="$(dirname "$0")/../backend/slicer/cura_profiles/definitions"

mkdir -p "$DEST_DIR"

echo "Downloading CuraEngine definition files from Cura main..."

curl -fL -o "$DEST_DIR/fdmprinter.def.json"  "$BASE_URL/fdmprinter.def.json"
curl -fL -o "$DEST_DIR/fdmextruder.def.json" "$BASE_URL/fdmextruder.def.json"

echo ""
echo "Definition files saved to: $DEST_DIR"
echo "  fdmprinter.def.json   — base machine definition"
echo "  fdmextruder.def.json  — extruder definition"

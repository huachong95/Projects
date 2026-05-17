@echo off
REM Downloads CuraEngine printer-definition JSON files needed for slicing.
REM These files are MIT-licensed and come from the Ultimaker/Cura repository.
REM NOTE: The definitions are already vendored in the repo. This script only
REM needs to be run if you want to refresh them to a newer version.
REM Use main branch — the 5.7.2 tag does not expose resources/definitions/ via
REM raw.githubusercontent (returns 404).

SET DEST_DIR=%~dp0..\backend\slicer\cura_profiles\definitions
SET BASE_URL=https://raw.githubusercontent.com/Ultimaker/Cura/main/resources/definitions

IF NOT EXIST "%DEST_DIR%" mkdir "%DEST_DIR%"

echo Downloading CuraEngine definition files from Cura main...

curl -fL -o "%DEST_DIR%\fdmprinter.def.json"  "%BASE_URL%/fdmprinter.def.json"
curl -fL -o "%DEST_DIR%\fdmextruder.def.json" "%BASE_URL%/fdmextruder.def.json"

echo.
echo Definition files saved to: %DEST_DIR%
echo   fdmprinter.def.json   — base machine definition
echo   fdmextruder.def.json  — extruder definition

@echo off
REM Downloads CuraEngine printer-definition JSON files needed for slicing.
REM These files are MIT-licensed and come from the Ultimaker/Cura repository.

SET CURA_VERSION=5.7.2
SET DEST_DIR=%~dp0..\backend\slicer\cura_profiles\definitions
SET BASE_URL=https://raw.githubusercontent.com/Ultimaker/Cura/%CURA_VERSION%/resources/definitions

IF NOT EXIST "%DEST_DIR%" mkdir "%DEST_DIR%"

echo Downloading CuraEngine definition files (Cura %CURA_VERSION%)...

curl -fL -o "%DEST_DIR%\fdmprinter.def.json"  "%BASE_URL%/fdmprinter.def.json"
curl -fL -o "%DEST_DIR%\fdmextruder.def.json" "%BASE_URL%/fdmextruder.def.json"

echo.
echo Definition files saved to: %DEST_DIR%
echo   fdmprinter.def.json   — base machine definition
echo   fdmextruder.def.json  — extruder definition

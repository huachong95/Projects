@echo off
REM Downloads the CuraEngine binary for Windows.
REM CuraEngine is AGPL-licensed — download at runtime, never commit the binary.

SET CURA_VERSION=5.7.2
SET DEST_DIR=%~dp0..\backend\slicer\cura_engine
SET BINARY_URL=https://github.com/Ultimaker/CuraEngine/releases/download/%CURA_VERSION%/CuraEngine-%CURA_VERSION%-Windows-X64.exe

IF NOT EXIST "%DEST_DIR%" mkdir "%DEST_DIR%"

echo Downloading CuraEngine %CURA_VERSION% for Windows...
curl -L -o "%DEST_DIR%\CuraEngine.exe" "%BINARY_URL%"

echo CuraEngine downloaded to %DEST_DIR%\CuraEngine.exe
echo.
echo Printer definition files (fdmprinter.def.json, fdmextruder.def.json)
echo are already included in the repository -- no separate download needed.

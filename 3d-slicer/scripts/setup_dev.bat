@echo off
REM 3D Slicer Development Setup for Windows

echo === 3D Slicer Development Setup ===

SET ROOT=%~dp0..

REM Python backend
echo 1. Setting up Python backend...
cd /d "%ROOT%\backend"
python -m venv .venv
call .venv\Scripts\activate.bat
pip install --upgrade pip
pip install -r requirements.txt
echo    Backend dependencies installed.

REM Three.js viewer assets
echo.
echo 2. Downloading Three.js viewer assets...
SET VIEWER_DIR=%ROOT%\frontend\assets\viewer
IF NOT EXIST "%VIEWER_DIR%" mkdir "%VIEWER_DIR%"
curl -fL -o "%VIEWER_DIR%\three.min.js" "https://cdnjs.cloudflare.com/ajax/libs/three.js/r160/three.min.js"
curl -fL -o "%VIEWER_DIR%\STLLoader.js" "https://raw.githubusercontent.com/mrdoob/three.js/r160/examples/js/loaders/STLLoader.js"
curl -fL -o "%VIEWER_DIR%\OrbitControls.js" "https://raw.githubusercontent.com/mrdoob/three.js/r160/examples/js/controls/OrbitControls.js"
echo    Three.js assets downloaded.

REM CuraEngine
echo.
echo 3. Downloading CuraEngine...
call "%~dp0download_curaengine.bat" || echo    WARNING: CuraEngine download failed. Download manually.

REM Flutter
echo.
echo 4. Setting up Flutter app...
cd /d "%ROOT%\frontend"
where flutter >nul 2>&1
IF %ERRORLEVEL% EQU 0 (
  IF NOT EXIST "windows" (
    flutter create --platforms=windows,ios . --project-name slicer_app
    echo    Flutter platform directories created.
  )
  flutter pub get
  echo    Flutter dependencies installed.
) ELSE (
  echo    Flutter not found. Install from https://flutter.dev/docs/get-started/install
  echo    Then run: flutter create --platforms=windows,ios . ^&^& flutter pub get
)

echo.
echo === Setup complete ===
echo.
echo To start the backend:
echo   cd backend ^&^& .venv\Scripts\activate ^&^& python main.py
echo.
echo To start the Flutter app:
echo   cd frontend ^&^& flutter run -d windows
echo.
echo IMPORTANT: Place fdmprinter.def.json and fdmextruder.def.json in:
echo   backend\slicer\cura_profiles\definitions\
echo   (Download from https://github.com/Ultimaker/Cura/tree/5.7.2/resources/definitions)

pause

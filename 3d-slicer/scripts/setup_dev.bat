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
echo 2. Refreshing Three.js viewer assets (r128)...
REM Already vendored in the repo. r128 is REQUIRED: the viewer loads these
REM via plain script tags and uses the global THREE.STLLoader /
REM THREE.OrbitControls namespace, which only exists in examples/js/ up to
REM ~r147; r148+ is ES-module-only and would break the viewer.
SET VIEWER_DIR=%ROOT%\frontend\assets\viewer
IF NOT EXIST "%VIEWER_DIR%" mkdir "%VIEWER_DIR%"
curl -fL -o "%VIEWER_DIR%\three.min.js" "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
curl -fL -o "%VIEWER_DIR%\STLLoader.js" "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"
curl -fL -o "%VIEWER_DIR%\OrbitControls.js" "https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"
echo    Three.js assets refreshed.

REM CuraEngine
echo.
echo 3. Downloading CuraEngine...
call "%~dp0download_curaengine.bat" || echo    WARNING: CuraEngine download failed. Download manually.

REM Definition files
echo.
echo 4. Downloading CuraEngine definition files...
call "%~dp0download_definitions.bat" || echo    WARNING: Definition file download failed. Download manually.

REM Flutter
echo.
echo 5. Setting up Flutter app...
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
echo Definition files are downloaded automatically by this script.
echo CuraEngine binary is downloaded automatically by this script.

pause

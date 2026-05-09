@echo off
echo =============================================
echo   Icy Towers - Windows Build Script
echo =============================================
echo.

echo [1/3] Installing Python dependencies...
pip install pygame pyinstaller
if %errorlevel% neq 0 (
    echo ERROR: pip install failed. Make sure Python is on your PATH.
    pause
    exit /b 1
)

echo.
echo [2/3] Building Windows executable...
pyinstaller icy_towers.spec --clean
if %errorlevel% neq 0 (
    echo ERROR: PyInstaller build failed.
    pause
    exit /b 1
)

echo.
echo [3/3] Done!
echo.
echo  IcyTowers.exe is in the dist\ folder.
echo.
pause

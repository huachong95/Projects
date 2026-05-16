@echo off
REM Downloads the Obico print failure detection model (MIT license).
SET DEST=%~dp0..\backend\monitoring\models
IF NOT EXIST "%DEST%" mkdir "%DEST%"
echo Downloading Obico failure detection model...
curl -fL --progress-bar -o "%DEST%\failure_detector.onnx" "https://github.com/TheSpaghettiDetective/obico-server/releases/download/1.9.0/model.onnx"
echo Model downloaded to %DEST%\failure_detector.onnx

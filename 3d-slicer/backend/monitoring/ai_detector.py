"""ONNX-based print failure detector using a model trained on the Obico dataset."""
import asyncio
import io
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image


@dataclass
class DetectionResult:
    failure_probability: float = 0.0
    is_alert: bool = False


class FailureDetector:
    INPUT_SIZE = (416, 416)
    MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def __init__(self, model_path: Path, alert_threshold: float = 0.75):
        self.alert_threshold = alert_threshold
        self._session = None
        self._input_name: Optional[str] = None
        self._load_model(model_path)

    def _load_model(self, model_path: Path) -> None:
        if not model_path.exists():
            return
        try:
            import onnxruntime as ort
            self._session = ort.InferenceSession(
                str(model_path),
                providers=["CPUExecutionProvider"],
            )
            self._input_name = self._session.get_inputs()[0].name
        except Exception:
            self._session = None

    @property
    def is_available(self) -> bool:
        return self._session is not None

    def analyze_frame(self, jpeg_bytes: bytes) -> DetectionResult:
        if not self._session:
            return DetectionResult()
        try:
            img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
            img = img.resize(self.INPUT_SIZE, Image.BILINEAR)
            arr = np.array(img, dtype=np.float32) / 255.0
            arr = (arr - self.MEAN) / self.STD
            arr = arr.transpose(2, 0, 1)[np.newaxis]  # [1, 3, H, W]

            outputs = self._session.run(None, {self._input_name: arr})
            prob = float(outputs[0].flatten()[0])
            return DetectionResult(
                failure_probability=prob,
                is_alert=prob >= self.alert_threshold,
            )
        except Exception:
            return DetectionResult()


class AIMonitor:
    def __init__(self, detector: FailureDetector, inference_interval: int = 5):
        self.detector = detector
        self.interval = inference_interval
        self._task: Optional[asyncio.Task] = None
        self._consecutive_alerts = 0
        self._alert_threshold = 3
        self._enabled = False

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False
        self._consecutive_alerts = 0

    async def run(self, camera_stream, on_detection, on_alert) -> None:
        self._task = asyncio.create_task(
            self._monitor_loop(camera_stream, on_detection, on_alert)
        )

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            self._task = None

    async def _monitor_loop(self, camera_stream, on_detection, on_alert) -> None:
        while True:
            await asyncio.sleep(self.interval)
            if not self._enabled or not self.detector.is_available:
                continue

            frame = camera_stream.get_latest_frame()
            if frame is None:
                continue

            result = await asyncio.get_event_loop().run_in_executor(
                None, self.detector.analyze_frame, frame
            )
            await on_detection(result)

            if result.is_alert:
                self._consecutive_alerts += 1
                if self._consecutive_alerts >= self._alert_threshold:
                    await on_alert(result)
                    self._consecutive_alerts = 0
            else:
                self._consecutive_alerts = 0

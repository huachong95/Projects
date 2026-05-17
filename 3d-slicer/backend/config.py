import sys
from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict


BASE_DIR = Path(__file__).parent

_CURA_EXE = "CuraEngine.exe" if sys.platform == "win32" else "CuraEngine"


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=BASE_DIR / ".env", extra="ignore")

    backend_host: str = "0.0.0.0"
    backend_port: int = 8000
    cors_origins: list[str] = ["*"]

    temp_dir: Path = BASE_DIR / "data" / "temp"
    frames_dir: Path = BASE_DIR / "data" / "timelapses"
    gcode_dir: Path = BASE_DIR / "data" / "gcode"

    cura_engine_path: Path = BASE_DIR / "slicer" / "cura_engine" / _CURA_EXE
    cura_definitions_dir: Path = BASE_DIR / "slicer" / "cura_profiles" / "definitions"
    cura_profiles_dir: Path = BASE_DIR / "slicer" / "cura_profiles"

    ai_model_path: Path = BASE_DIR / "monitoring" / "models" / "failure_detector.onnx"
    ai_alert_threshold: float = 0.75
    ai_alert_consecutive_frames: int = 3
    ai_inference_interval_seconds: int = 5

    prusalink_host: str = ""
    prusalink_key: str = ""

    firebase_credentials_path: Path = BASE_DIR / "firebase_credentials.json"

    def model_post_init(self, __context) -> None:
        for path in [self.temp_dir, self.frames_dir, self.gcode_dir]:
            path.mkdir(parents=True, exist_ok=True)


settings = Settings()

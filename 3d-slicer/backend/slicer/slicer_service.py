import asyncio
import json
import shutil
import uuid
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

from pydantic import BaseModel

from slicer.gcode_processor import GcodeMetadata, extract_metadata, insert_timelapse_hooks


class SliceState(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETE = "complete"
    FAILED = "failed"


class SliceJob(BaseModel):
    slice_job_id: str
    mesh_job_id: str
    state: SliceState = SliceState.PENDING
    progress_percent: float = 0.0
    error: Optional[str] = None
    gcode_path: Optional[str] = None
    processed_gcode_path: Optional[str] = None
    metadata: Optional[GcodeMetadata] = None


class SliceSettings(BaseModel):
    profile_name: str = "pla_standard"
    overrides: dict = {}


_PRUSA_MK4_DEFAULTS = {
    "machine_width": 250,
    "machine_depth": 210,
    "machine_height": 220,
    "machine_nozzle_size": 0.4,
    "machine_heated_bed": True,
}

_PROGRESS_PATTERN = "Progress:"


class SlicerService:
    def __init__(
        self,
        cura_engine_path: Path,
        cura_definitions_dir: Path,
        cura_profiles_dir: Path,
        gcode_dir: Path,
    ):
        self.cura_engine = cura_engine_path
        self.definitions_dir = cura_definitions_dir
        self.profiles_dir = cura_profiles_dir
        self.gcode_dir = gcode_dir
        self._jobs: dict[str, SliceJob] = {}

    def list_profiles(self) -> list[str]:
        return [
            p.stem
            for p in self.profiles_dir.glob("*.json")
            if not p.stem.startswith("base")
        ]

    def get_job(self, slice_job_id: str) -> Optional[SliceJob]:
        return self._jobs.get(slice_job_id)

    async def start_slice(
        self,
        stl_path: Path,
        mesh_job_id: str,
        settings: SliceSettings,
        progress_callback: Optional[Callable[[float], None]] = None,
        timelapse_hooks: bool = True,
    ) -> SliceJob:
        slice_job_id = str(uuid.uuid4())
        job = SliceJob(slice_job_id=slice_job_id, mesh_job_id=mesh_job_id)
        self._jobs[slice_job_id] = job

        asyncio.create_task(
            self._run_slice(job, stl_path, settings, progress_callback, timelapse_hooks)
        )
        return job

    async def _run_slice(
        self,
        job: SliceJob,
        stl_path: Path,
        settings: SliceSettings,
        progress_callback: Optional[Callable[[float], None]],
        timelapse_hooks: bool,
    ) -> None:
        job.state = SliceState.RUNNING
        out_dir = self.gcode_dir / job.slice_job_id
        out_dir.mkdir(parents=True, exist_ok=True)
        raw_gcode = out_dir / "raw.gcode"
        final_gcode = out_dir / "model.gcode"

        machine_def = self.definitions_dir / "fdmprinter.def.json"
        if not machine_def.exists():
            job.state = SliceState.FAILED
            job.error = f"CuraEngine definition not found: {machine_def}"
            return

        merged = {**_PRUSA_MK4_DEFAULTS, **self._load_profile(settings.profile_name), **settings.overrides}
        flags = self._build_flags(merged)

        cmd = [
            str(self.cura_engine),
            "slice",
            "-j", str(machine_def),
            "-l", str(stl_path),
            "-o", str(raw_gcode),
            *flags,
        ]

        try:
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )

            async for raw_line in process.stdout:
                line = raw_line.decode(errors="replace").strip()
                if _PROGRESS_PATTERN in line:
                    pct = self._parse_progress(line)
                    if pct is not None:
                        job.progress_percent = pct
                        if progress_callback:
                            await progress_callback(pct)

            await process.wait()

            if process.returncode != 0:
                job.state = SliceState.FAILED
                job.error = f"CuraEngine exited with code {process.returncode}"
                return

            if timelapse_hooks:
                insert_timelapse_hooks(raw_gcode, final_gcode)
            else:
                shutil.copy(raw_gcode, final_gcode)

            job.gcode_path = str(raw_gcode)
            job.processed_gcode_path = str(final_gcode)
            job.metadata = extract_metadata(final_gcode)
            job.state = SliceState.COMPLETE
            job.progress_percent = 100.0

        except FileNotFoundError:
            job.state = SliceState.FAILED
            job.error = f"CuraEngine binary not found at {self.cura_engine}. Run scripts/download_curaengine.sh."
        except Exception as e:
            job.state = SliceState.FAILED
            job.error = str(e)

    def _load_profile(self, profile_name: str) -> dict:
        base_path = self.profiles_dir / "base_config.json"
        base = json.loads(base_path.read_text()) if base_path.exists() else {}

        profile_path = self.profiles_dir / f"{profile_name}.json"
        profile = json.loads(profile_path.read_text()) if profile_path.exists() else {}

        return {**base, **profile}

    def _build_flags(self, settings: dict) -> list[str]:
        flags = []
        for key, value in settings.items():
            if isinstance(value, bool):
                value = "true" if value else "false"
            flags.extend(["-s", f"{key}={value}"])
        return flags

    def _parse_progress(self, line: str) -> Optional[float]:
        try:
            parts = line.split("Progress:")
            if len(parts) > 1:
                pct_str = parts[1].strip().split()[0].rstrip("%")
                return float(pct_str)
        except (ValueError, IndexError):
            pass
        return None

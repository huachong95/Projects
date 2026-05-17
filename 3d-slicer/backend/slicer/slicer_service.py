import asyncio
import json
import re
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


# Settings that belong at machine level (before -e0 in the CuraEngine command).
# Everything else is extruder-level (after -e0 -j fdmextruder.def.json).
_MACHINE_SETTINGS = frozenset({
    "machine_width", "machine_depth", "machine_height",
    "machine_heated_bed", "machine_start_gcode", "machine_end_gcode",
    "machine_name",
})

_PRUSA_MK4_DEFAULTS = {
    "machine_width": 250,
    "machine_depth": 210,
    "machine_height": 220,
    "machine_nozzle_size": 0.4,
    "machine_heated_bed": True,
}

# Matches any "NN.N%" pattern — used for CuraEngine 5.x progress lines like
# "Progress:inset+skin:50.0%:Mesh 1/1".
_PROGRESS_RE = re.compile(r"(\d+(?:\.\d+)?)%")


class SlicerService:
    def __init__(
        self,
        cura_engine_path: Path,
        cura_definitions_dir: Path,
        cura_profiles_dir: Path,
        gcode_dir: Path,
        timeout_seconds: int = 600,
    ):
        self.cura_engine = cura_engine_path
        self.definitions_dir = cura_definitions_dir
        self.profiles_dir = cura_profiles_dir
        self.gcode_dir = gcode_dir
        self.timeout_seconds = timeout_seconds
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
        completion_callback: Optional[Callable[["SliceJob"], None]] = None,
        timelapse_hooks: bool = True,
    ) -> SliceJob:
        slice_job_id = str(uuid.uuid4())
        job = SliceJob(slice_job_id=slice_job_id, mesh_job_id=mesh_job_id)
        self._jobs[slice_job_id] = job

        task = asyncio.create_task(
            self._run_slice(job, stl_path, settings, progress_callback, completion_callback, timelapse_hooks)
        )
        task.add_done_callback(lambda t: t.exception() if not t.cancelled() else None)
        return job

    async def _run_slice(
        self,
        job: SliceJob,
        stl_path: Path,
        settings: SliceSettings,
        progress_callback: Optional[Callable[[float], None]],
        completion_callback: Optional[Callable[["SliceJob"], None]],
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
            job.error = (
                f"CuraEngine definition not found: {machine_def}. "
                "Run scripts/download_definitions.bat to download it."
            )
            return

        merged = {
            **_PRUSA_MK4_DEFAULTS,
            **self._load_profile(settings.profile_name),
            **settings.overrides,
        }
        cmd = self._build_cmd(stl_path, raw_gcode, merged)

        try:
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )

            try:
                async with asyncio.timeout(self.timeout_seconds):
                    async for raw_line in process.stdout:
                        line = raw_line.decode(errors="replace").strip()
                        if "Progress:" in line:
                            pct = self._parse_progress(line)
                            if pct is not None:
                                job.progress_percent = pct
                                if progress_callback:
                                    await progress_callback(pct)
                    await process.wait()
            except asyncio.TimeoutError:
                process.kill()
                job.state = SliceState.FAILED
                job.error = f"Slicing timed out after {self.timeout_seconds}s"
                return

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

            if completion_callback:
                await completion_callback(job)

        except FileNotFoundError:
            job.state = SliceState.FAILED
            job.error = (
                f"CuraEngine binary not found at {self.cura_engine}. "
                "Run scripts/download_curaengine.bat to download it."
            )
        except Exception as e:
            job.state = SliceState.FAILED
            job.error = str(e)

    def _build_cmd(self, stl_path: Path, raw_gcode: Path, merged_settings: dict) -> list[str]:
        """Build the CuraEngine 5.x command.

        Structure:
          CuraEngine slice -v
            -j fdmprinter.def.json   (base machine def)
            -s machine_width=250 ... (machine-level overrides)
            -e0
            -j fdmextruder.def.json  (extruder def, if present)
            -s layer_height=0.2 ...  (extruder-level overrides)
            -l model.stl
            -o output.gcode
        """
        machine_def = self.definitions_dir / "fdmprinter.def.json"
        extruder_def = self.definitions_dir / "fdmextruder.def.json"

        machine_kvs = {k: v for k, v in merged_settings.items() if k in _MACHINE_SETTINGS}
        extruder_kvs = {k: v for k, v in merged_settings.items() if k not in _MACHINE_SETTINGS}

        cmd = [str(self.cura_engine), "slice", "-v", "-j", str(machine_def)]
        cmd += self._build_flags(machine_kvs)
        cmd += ["-e0"]
        if extruder_def.exists():
            cmd += ["-j", str(extruder_def)]
        cmd += self._build_flags(extruder_kvs)
        cmd += ["-l", str(stl_path), "-o", str(raw_gcode)]
        return cmd

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
            elif isinstance(value, str) and ("\n" in value or '"' in value or "'" in value):
                # Skip multi-line gcode strings (start/end gcode) — CuraEngine uses its own defaults.
                continue
            flags.extend(["-s", f"{key}={value}"])
        return flags

    def _parse_progress(self, line: str) -> Optional[float]:
        m = _PROGRESS_RE.search(line)
        if m:
            return min(float(m.group(1)), 100.0)
        return None

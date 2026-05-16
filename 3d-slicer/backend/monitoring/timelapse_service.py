"""Layer-synchronized timelapse capture and FFmpeg compilation."""
import asyncio
import uuid
from pathlib import Path
from typing import Optional


class TimelapseService:
    def __init__(self, frames_dir: Path):
        self.frames_dir = frames_dir
        self._active_job_id: Optional[str] = None

    def start_job(self) -> str:
        self._active_job_id = str(uuid.uuid4())
        job_dir = self._job_frames_dir(self._active_job_id)
        job_dir.mkdir(parents=True, exist_ok=True)
        return self._active_job_id

    def end_job(self) -> Optional[str]:
        job_id = self._active_job_id
        self._active_job_id = None
        return job_id

    def capture_frame(self, layer_num: int, jpeg_bytes: bytes) -> Optional[Path]:
        if not self._active_job_id or not jpeg_bytes:
            return None
        frame_path = self._job_frames_dir(self._active_job_id) / f"layer_{layer_num:05d}.jpg"
        frame_path.write_bytes(jpeg_bytes)
        return frame_path

    def frame_count(self, job_id: str) -> int:
        frames_dir = self._job_frames_dir(job_id)
        if not frames_dir.exists():
            return 0
        return len(list(frames_dir.glob("layer_*.jpg")))

    async def compile(self, job_id: str, fps: int = 24) -> Optional[Path]:
        frames_dir = self._job_frames_dir(job_id)
        if not frames_dir.exists() or not list(frames_dir.glob("layer_*.jpg")):
            return None

        output_path = self.frames_dir / job_id / "timelapse.mp4"
        cmd = [
            "ffmpeg", "-y",
            "-framerate", str(fps),
            "-pattern_type", "glob",
            "-i", str(frames_dir / "layer_*.jpg"),
            "-vf", "scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2",
            "-c:v", "libx264",
            "-preset", "slow",
            "-crf", "23",
            "-pix_fmt", "yuv420p",
            str(output_path),
        ]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        await proc.wait()
        return output_path if output_path.exists() else None

    def list_jobs(self) -> list[dict]:
        if not self.frames_dir.exists():
            return []
        jobs = []
        for job_dir in self.frames_dir.iterdir():
            if not job_dir.is_dir():
                continue
            mp4 = job_dir / "timelapse.mp4"
            jobs.append({
                "job_id": job_dir.name,
                "frame_count": self.frame_count(job_dir.name),
                "has_video": mp4.exists(),
                "video_size_mb": round(mp4.stat().st_size / 1_048_576, 1) if mp4.exists() else 0,
            })
        return sorted(jobs, key=lambda x: x["job_id"], reverse=True)

    def _job_frames_dir(self, job_id: str) -> Path:
        return self.frames_dir / job_id / "frames"

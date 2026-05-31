"""Print history with JSON persistence.

A record is opened when a print starts and closed when it finishes, is
cancelled or fails. The lifecycle is driven by the printer status poll loop
(see main.py) reacting to state transitions.
"""
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, Field

MAX_RECORDS = 200


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


class PrintRecord(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    filename: str
    status: str = "printing"  # printing | completed | cancelled | failed
    started_at: str = Field(default_factory=_now)
    completed_at: Optional[str] = None
    duration_seconds: int = 0
    progress_percent: float = 0.0
    filament_used_g: float = 0.0
    slice_job_id: Optional[str] = None


class PrintHistoryService:
    def __init__(self, path: Path):
        self.path = path
        self._records: list[PrintRecord] = []
        self._load()

    # ------------------------------------------------------------------ #
    # Persistence
    # ------------------------------------------------------------------ #
    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text())
            self._records = [PrintRecord(**item) for item in raw]
        except (json.JSONDecodeError, OSError, ValueError):
            self._records = []

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        trimmed = self._records[:MAX_RECORDS]
        self.path.write_text(json.dumps([r.model_dump() for r in trimmed], indent=2))

    # ------------------------------------------------------------------ #
    # Queries
    # ------------------------------------------------------------------ #
    def list(self) -> list[PrintRecord]:
        return list(self._records)

    def active(self) -> Optional[PrintRecord]:
        """The currently-open (printing) record, if any."""
        return next((r for r in self._records if r.status == "printing"), None)

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def start_record(
        self,
        filename: str,
        slice_job_id: Optional[str] = None,
        filament_used_g: float = 0.0,
    ) -> PrintRecord:
        # Newest first.
        record = PrintRecord(
            filename=filename or "Untitled print",
            slice_job_id=slice_job_id,
            filament_used_g=filament_used_g,
        )
        self._records.insert(0, record)
        del self._records[MAX_RECORDS:]
        self._save()
        return record

    def update_progress(self, progress_percent: float) -> None:
        active = self.active()
        if active and progress_percent > active.progress_percent:
            active.progress_percent = round(progress_percent, 1)
            # Light-touch save; progress changes often, so don't trim/dump on
            # every tick — only persist meaningful jumps.

    def complete_record(self, status: str, progress_percent: Optional[float] = None) -> Optional[PrintRecord]:
        active = self.active()
        if not active:
            return None
        active.status = status
        active.completed_at = _now()
        if progress_percent is not None:
            active.progress_percent = round(progress_percent, 1)
        try:
            started = datetime.fromisoformat(active.started_at)
            ended = datetime.fromisoformat(active.completed_at)
            active.duration_seconds = max(0, int((ended - started).total_seconds()))
        except ValueError:
            active.duration_seconds = 0
        self._save()
        return active

    def delete(self, record_id: str) -> bool:
        before = len(self._records)
        self._records = [r for r in self._records if r.id != record_id]
        removed = len(self._records) != before
        if removed:
            self._save()
        return removed

    def clear(self) -> None:
        self._records = []
        self._save()

"""Persistent print history — records every completed/failed/cancelled print."""
import json
import time
import uuid
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Optional


@dataclass
class PrintRecord:
    record_id: str
    filename: str
    slice_job_id: str
    started_at: float
    completed_at: Optional[float] = None
    duration_seconds: Optional[int] = None
    layer_count: int = 0
    filament_used_mm: float = 0.0
    filament_used_g: float = 0.0
    status: str = "running"  # running | success | failed | cancelled
    thumbnail_b64: Optional[str] = None


class PrintHistoryService:
    _MAX_RECORDS = 200

    def __init__(self, data_dir: Path):
        self._path = data_dir / "print_history.json"
        self._records: list[PrintRecord] = self._load()

    def _load(self) -> list[PrintRecord]:
        if not self._path.exists():
            return []
        try:
            return [PrintRecord(**r) for r in json.loads(self._path.read_text())]
        except Exception:
            return []

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(json.dumps([asdict(r) for r in self._records], indent=2))

    def start_record(self, filename: str, slice_job_id: str) -> PrintRecord:
        record = PrintRecord(
            record_id=str(uuid.uuid4()),
            filename=filename,
            slice_job_id=slice_job_id,
            started_at=time.time(),
            status="running",
        )
        self._records.insert(0, record)
        if len(self._records) > self._MAX_RECORDS:
            self._records = self._records[: self._MAX_RECORDS]
        self._save()
        return record

    def complete_record(
        self,
        record_id: str,
        status: str,
        layer_count: int = 0,
        filament_used_mm: float = 0.0,
        filament_used_g: float = 0.0,
    ) -> Optional[PrintRecord]:
        for r in self._records:
            if r.record_id == record_id:
                r.completed_at = time.time()
                r.duration_seconds = int(r.completed_at - r.started_at)
                r.status = status
                r.layer_count = layer_count
                r.filament_used_mm = filament_used_mm
                r.filament_used_g = filament_used_g
                self._save()
                return r
        return None

    def list_records(self) -> list[PrintRecord]:
        return self._records

    def get_record(self, record_id: str) -> Optional[PrintRecord]:
        return next((r for r in self._records if r.record_id == record_id), None)

    def delete_record(self, record_id: str) -> bool:
        before = len(self._records)
        self._records = [r for r in self._records if r.record_id != record_id]
        if len(self._records) < before:
            self._save()
            return True
        return False

    def clear(self) -> int:
        count = len(self._records)
        self._records = []
        self._save()
        return count

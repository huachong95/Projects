"""Filament spool inventory with JSON persistence.

Tracks the spools the user owns, how much is left on each, and which one is
currently loaded ("active"). Print completion deducts the estimated grams from
the active spool.
"""
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, Field


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


class Spool(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str
    material: str = "PLA"
    color: str = "#E85D04"  # hex
    diameter_mm: float = 1.75
    total_weight_g: float = 1000.0
    remaining_weight_g: float = 1000.0
    notes: str = ""
    is_active: bool = False
    created_at: str = Field(default_factory=_now)


class SpoolCreate(BaseModel):
    name: str
    material: str = "PLA"
    color: str = "#E85D04"
    diameter_mm: float = 1.75
    total_weight_g: float = 1000.0
    remaining_weight_g: Optional[float] = None
    notes: str = ""


class SpoolUpdate(BaseModel):
    name: Optional[str] = None
    material: Optional[str] = None
    color: Optional[str] = None
    diameter_mm: Optional[float] = None
    total_weight_g: Optional[float] = None
    remaining_weight_g: Optional[float] = None
    notes: Optional[str] = None


class FilamentService:
    def __init__(self, path: Path):
        self.path = path
        self._spools: list[Spool] = []
        self._load()

    # ------------------------------------------------------------------ #
    # Persistence
    # ------------------------------------------------------------------ #
    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text())
            self._spools = [Spool(**item) for item in raw]
        except (json.JSONDecodeError, OSError, ValueError):
            self._spools = []

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps([s.model_dump() for s in self._spools], indent=2))

    # ------------------------------------------------------------------ #
    # CRUD
    # ------------------------------------------------------------------ #
    def list(self) -> list[Spool]:
        return list(self._spools)

    def get(self, spool_id: str) -> Optional[Spool]:
        return next((s for s in self._spools if s.id == spool_id), None)

    def create(self, data: SpoolCreate) -> Spool:
        spool = Spool(
            name=data.name,
            material=data.material,
            color=data.color,
            diameter_mm=data.diameter_mm,
            total_weight_g=data.total_weight_g,
            remaining_weight_g=data.remaining_weight_g
            if data.remaining_weight_g is not None
            else data.total_weight_g,
            notes=data.notes,
            # First spool added becomes active by default.
            is_active=not self._spools,
        )
        self._spools.append(spool)
        self._save()
        return spool

    def update(self, spool_id: str, data: SpoolUpdate) -> Optional[Spool]:
        spool = self.get(spool_id)
        if not spool:
            return None
        patch = data.model_dump(exclude_none=True)
        updated = spool.model_copy(update=patch)
        self._replace(updated)
        self._save()
        return updated

    def delete(self, spool_id: str) -> bool:
        before = len(self._spools)
        self._spools = [s for s in self._spools if s.id != spool_id]
        removed = len(self._spools) != before
        if removed:
            self._save()
        return removed

    # ------------------------------------------------------------------ #
    # Usage / active spool
    # ------------------------------------------------------------------ #
    def set_active(self, spool_id: str) -> Optional[Spool]:
        target = self.get(spool_id)
        if not target:
            return None
        for s in self._spools:
            s.is_active = s.id == spool_id
        self._save()
        return self.get(spool_id)

    def get_active(self) -> Optional[Spool]:
        return next((s for s in self._spools if s.is_active), None)

    def deduct(self, spool_id: str, used_g: float) -> Optional[Spool]:
        spool = self.get(spool_id)
        if not spool:
            return None
        spool.remaining_weight_g = max(0.0, round(spool.remaining_weight_g - used_g, 2))
        self._replace(spool)
        self._save()
        return spool

    def deduct_from_active(self, used_g: float) -> Optional[Spool]:
        active = self.get_active()
        if not active or used_g <= 0:
            return None
        return self.deduct(active.id, used_g)

    def _replace(self, spool: Spool) -> None:
        for i, s in enumerate(self._spools):
            if s.id == spool.id:
                self._spools[i] = spool
                return

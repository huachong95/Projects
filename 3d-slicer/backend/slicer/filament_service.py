"""Filament spool manager — tracks spools, material, remaining weight."""
import json
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional


@dataclass
class FilamentSpool:
    spool_id: str
    name: str
    material: str       # PLA | PETG | ABS | ASA | TPU | PC | PA | PLA+ | …
    color_hex: str      # #RRGGBB
    vendor: str = ""
    diameter_mm: float = 1.75
    initial_weight_g: float = 1000.0
    used_weight_g: float = 0.0
    nozzle_temp_c: int = 215
    bed_temp_c: int = 60
    notes: str = ""


class FilamentService:
    def __init__(self, data_dir: Path):
        self._path = data_dir / "filament_spools.json"
        self._spools: list[FilamentSpool] = self._load()

    def _load(self) -> list[FilamentSpool]:
        if not self._path.exists():
            return []
        try:
            return [FilamentSpool(**s) for s in json.loads(self._path.read_text())]
        except Exception:
            return []

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(json.dumps([asdict(s) for s in self._spools], indent=2))

    def list_spools(self) -> list[FilamentSpool]:
        return self._spools

    def get_spool(self, spool_id: str) -> Optional[FilamentSpool]:
        return next((s for s in self._spools if s.spool_id == spool_id), None)

    def create_spool(
        self,
        name: str,
        material: str,
        color_hex: str,
        vendor: str = "",
        diameter_mm: float = 1.75,
        initial_weight_g: float = 1000.0,
        nozzle_temp_c: int = 215,
        bed_temp_c: int = 60,
        notes: str = "",
    ) -> FilamentSpool:
        spool = FilamentSpool(
            spool_id=str(uuid.uuid4()),
            name=name,
            material=material,
            color_hex=color_hex,
            vendor=vendor,
            diameter_mm=diameter_mm,
            initial_weight_g=initial_weight_g,
            nozzle_temp_c=nozzle_temp_c,
            bed_temp_c=bed_temp_c,
            notes=notes,
        )
        self._spools.insert(0, spool)
        self._save()
        return spool

    def update_spool(self, spool_id: str, **fields) -> Optional[FilamentSpool]:
        spool = self.get_spool(spool_id)
        if spool is None:
            return None
        allowed = {f for f in asdict(spool) if f != "spool_id"}
        for k, v in fields.items():
            if k in allowed:
                setattr(spool, k, v)
        self._save()
        return spool

    def deduct_filament(self, spool_id: str, used_g: float) -> Optional[FilamentSpool]:
        spool = self.get_spool(spool_id)
        if spool is None:
            return None
        spool.used_weight_g = round(spool.used_weight_g + used_g, 2)
        self._save()
        return spool

    def delete_spool(self, spool_id: str) -> bool:
        before = len(self._spools)
        self._spools = [s for s in self._spools if s.spool_id != spool_id]
        if len(self._spools) < before:
            self._save()
            return True
        return False

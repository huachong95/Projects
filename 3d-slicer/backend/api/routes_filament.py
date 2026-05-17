from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from slicer.filament_service import FilamentService

router = APIRouter()
_filament: Optional[FilamentService] = None


def init(filament_service: FilamentService) -> None:
    global _filament
    _filament = filament_service


class SpoolCreate(BaseModel):
    name: str
    material: str
    color_hex: str = Field("#FFFFFF", pattern=r"^#[0-9A-Fa-f]{6}$")
    vendor: str = ""
    diameter_mm: float = Field(1.75, gt=0, le=3)
    initial_weight_g: float = Field(1000.0, gt=0, le=10000)
    nozzle_temp_c: int = Field(215, ge=150, le=300)
    bed_temp_c: int = Field(60, ge=0, le=120)
    notes: str = ""


class SpoolUpdate(BaseModel):
    name: Optional[str] = None
    material: Optional[str] = None
    color_hex: Optional[str] = Field(None, pattern=r"^#[0-9A-Fa-f]{6}$")
    vendor: Optional[str] = None
    initial_weight_g: Optional[float] = Field(None, gt=0, le=10000)
    used_weight_g: Optional[float] = Field(None, ge=0)
    nozzle_temp_c: Optional[int] = Field(None, ge=150, le=300)
    bed_temp_c: Optional[int] = Field(None, ge=0, le=120)
    notes: Optional[str] = None


class DeductRequest(BaseModel):
    used_g: float = Field(..., gt=0)


def _spool_dict(s) -> dict:
    from dataclasses import asdict
    d = asdict(s)
    d["remaining_weight_g"] = round(max(0, s.initial_weight_g - s.used_weight_g), 2)
    d["remaining_percent"] = round(
        max(0, (s.initial_weight_g - s.used_weight_g) / s.initial_weight_g * 100)
        if s.initial_weight_g > 0 else 0,
        1,
    )
    return d


@router.get("")
async def list_spools():
    return {"spools": [_spool_dict(s) for s in _filament.list_spools()]}


@router.post("", status_code=201)
async def create_spool(req: SpoolCreate):
    spool = _filament.create_spool(**req.model_dump())
    return _spool_dict(spool)


@router.get("/{spool_id}")
async def get_spool(spool_id: str):
    spool = _filament.get_spool(spool_id)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return _spool_dict(spool)


@router.patch("/{spool_id}")
async def update_spool(spool_id: str, req: SpoolUpdate):
    updates = {k: v for k, v in req.model_dump().items() if v is not None}
    spool = _filament.update_spool(spool_id, **updates)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return _spool_dict(spool)


@router.post("/{spool_id}/deduct")
async def deduct_filament(spool_id: str, req: DeductRequest):
    spool = _filament.deduct_filament(spool_id, req.used_g)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return _spool_dict(spool)


@router.delete("/{spool_id}")
async def delete_spool(spool_id: str):
    if not _filament.delete_spool(spool_id):
        raise HTTPException(404, "Spool not found")
    return {"status": "deleted"}

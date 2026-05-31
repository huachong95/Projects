from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from slicer.filament_service import FilamentService, SpoolCreate, SpoolUpdate

router = APIRouter()
_filament: Optional[FilamentService] = None


def init(filament_service: FilamentService) -> None:
    global _filament
    _filament = filament_service


class DeductRequest(BaseModel):
    used_g: float


@router.get("")
async def list_spools():
    spools = _filament.list()
    active = _filament.get_active()
    return {
        "spools": [s.model_dump() for s in spools],
        "active_id": active.id if active else None,
    }


@router.post("")
async def create_spool(data: SpoolCreate):
    return _filament.create(data).model_dump()


@router.patch("/{spool_id}")
async def update_spool(spool_id: str, data: SpoolUpdate):
    spool = _filament.update(spool_id, data)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return spool.model_dump()


@router.delete("/{spool_id}")
async def delete_spool(spool_id: str):
    if not _filament.delete(spool_id):
        raise HTTPException(404, "Spool not found")
    return {"status": "deleted"}


@router.post("/{spool_id}/activate")
async def activate_spool(spool_id: str):
    spool = _filament.set_active(spool_id)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return spool.model_dump()


@router.post("/{spool_id}/deduct")
async def deduct_spool(spool_id: str, req: DeductRequest):
    spool = _filament.deduct(spool_id, req.used_g)
    if not spool:
        raise HTTPException(404, "Spool not found")
    return spool.model_dump()

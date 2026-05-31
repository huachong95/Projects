from typing import Optional

from fastapi import APIRouter, HTTPException

from monitoring.print_history import PrintHistoryService

router = APIRouter()
_history: Optional[PrintHistoryService] = None


def init(history_service: PrintHistoryService) -> None:
    global _history
    _history = history_service


@router.get("")
async def list_history():
    records = _history.list()
    return {
        "records": [r.model_dump() for r in records],
        "total": len(records),
    }


@router.delete("")
async def clear_history():
    _history.clear()
    return {"status": "cleared"}


@router.delete("/{record_id}")
async def delete_record(record_id: str):
    if not _history.delete(record_id):
        raise HTTPException(404, "Record not found")
    return {"status": "deleted"}

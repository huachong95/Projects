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
    records = _history.list_records()
    return {"records": [r.__dict__ for r in records], "total": len(records)}


@router.delete("")
async def clear_history():
    count = _history.clear()
    return {"deleted": count}


@router.get("/{record_id}")
async def get_record(record_id: str):
    record = _history.get_record(record_id)
    if not record:
        raise HTTPException(404, "Record not found")
    return record.__dict__


@router.delete("/{record_id}")
async def delete_record(record_id: str):
    if not _history.delete_record(record_id):
        raise HTTPException(404, "Record not found")
    return {"status": "deleted"}

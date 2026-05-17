from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from printer.printer_manager import ConnectionConfig, PrinterManager
from printer.prusalink_driver import discover_prusalink
from slicer.slicer_service import SlicerService, SliceState

router = APIRouter()
_printer: Optional[PrinterManager] = None
_slicer: Optional[SlicerService] = None


def init(printer_manager: PrinterManager, slicer_service: SlicerService) -> None:
    global _printer, _slicer
    _printer = printer_manager
    _slicer = slicer_service


class GcodeCommand(BaseModel):
    command: str


class PrintRequest(BaseModel):
    slice_job_id: str
    filename: str = "print.gcode"


@router.get("/discover")
async def discover():
    devices = await discover_prusalink(timeout=3.0)
    return {"devices": devices}


@router.post("/connect")
async def connect(config: ConnectionConfig):
    success = await _printer.connect(config)
    if not success:
        raise HTTPException(503, "Failed to connect to printer")
    return {"status": "connected"}


@router.delete("/connect")
async def disconnect():
    await _printer.disconnect()
    return {"status": "disconnected"}


@router.get("/status")
async def get_status():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    return await _printer.get_status()


@router.post("/print")
async def start_print(req: PrintRequest):
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    job = _slicer.get_job(req.slice_job_id)
    if not job:
        raise HTTPException(404, "Slice job not found")
    if job.state != SliceState.COMPLETE or not job.processed_gcode_path:
        raise HTTPException(422, "Slice job is not complete")
    success = await _printer.upload_and_print(job.processed_gcode_path, req.filename)
    if not success:
        raise HTTPException(503, "Failed to start print")
    return {"status": "printing"}


@router.post("/pause")
async def pause():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    await _printer.pause()
    return {"status": "paused"}


@router.post("/resume")
async def resume():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    await _printer.resume()
    return {"status": "resumed"}


@router.post("/cancel")
async def cancel():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    await _printer.cancel()
    return {"status": "cancelled"}


@router.post("/gcode")
async def send_gcode(cmd: GcodeCommand):
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    await _printer.send_gcode(cmd.command)
    return {"status": "sent"}


@router.get("/camera-url")
async def get_camera_url():
    url = _printer.get_camera_url()
    if not url:
        raise HTTPException(404, "No camera available")
    return {"url": url}

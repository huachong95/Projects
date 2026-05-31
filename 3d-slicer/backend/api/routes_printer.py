from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from printer.printer_manager import ConnectionConfig, PrinterManager
from printer.prusalink_driver import PrinterStatus, discover_prusalink
from slicer.slicer_service import SlicerService

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
    # Always return 200 with a status object. When disconnected we return a
    # default (connected=False) status so the UI can poll without error spam.
    if not _printer.is_connected:
        return PrinterStatus(connected=False)
    return await _printer.get_status()


@router.post("/print")
async def start_print(req: PrintRequest):
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    job = _slicer.get_job(req.slice_job_id)
    if not job:
        raise HTTPException(404, "Slice job not found")
    success = await _printer.upload_and_print(job.processed_gcode_path, req.filename)
    if not success:
        raise HTTPException(503, "Failed to start print")
    # Remember slice + filament estimate so the poll loop can log history and
    # deduct filament when the print finishes.
    filament_g = job.metadata.filament_used_g if job.metadata else 0.0
    _printer.set_pending_print(req.filename, req.slice_job_id, filament_g)
    return {"status": "printing"}


@router.post("/pause")
async def pause():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    if not await _printer.pause():
        raise HTTPException(409, "Could not pause — no active job or printer rejected the command")
    return {"status": "paused"}


@router.post("/resume")
async def resume():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    if not await _printer.resume():
        raise HTTPException(409, "Could not resume — no paused job or printer rejected the command")
    return {"status": "resumed"}


@router.post("/cancel")
async def cancel():
    if not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    if not await _printer.cancel():
        raise HTTPException(409, "Could not cancel — no active job or printer rejected the command")
    return {"status": "cancelled"}


@router.post("/gcode")
async def send_gcode(cmd: GcodeCommand):
    # PrusaLink does not expose an arbitrary G-code endpoint, so movement /
    # temperature / fan control cannot be sent this way. Report it clearly
    # instead of pretending it worked.
    raise HTTPException(
        501,
        "Sending raw G-code is not supported over PrusaLink. "
        "Control the printer from its touchscreen.",
    )


@router.get("/camera-url")
async def get_camera_url():
    if not _printer.camera_available:
        raise HTTPException(404, "No camera available")
    return {"url": "/api/monitoring/camera/snapshot"}

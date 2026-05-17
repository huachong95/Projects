from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

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


def _require_printer() -> PrinterManager:
    if not _printer or not _printer.is_connected:
        raise HTTPException(503, "Printer not connected")
    return _printer


class GcodeCommand(BaseModel):
    command: str


class PrintRequest(BaseModel):
    slice_job_id: str
    filename: str = "print.gcode"


class MoveRequest(BaseModel):
    axis: str = Field(..., pattern="^[xXyYzZeE]$")
    distance: float
    speed: int = 3000


class HomeRequest(BaseModel):
    axes: Optional[list[str]] = None  # None means home all


class TemperatureRequest(BaseModel):
    hotend: Optional[float] = Field(None, ge=0, le=300)
    bed: Optional[float] = Field(None, ge=0, le=120)


class FanRequest(BaseModel):
    speed_percent: int = Field(..., ge=0, le=100)


class ExtrudeRequest(BaseModel):
    distance_mm: float = Field(..., ge=-200, le=200)
    speed_mm_per_min: int = Field(300, ge=10, le=6000)


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
    _require_printer()
    return await _printer.get_status()


@router.get("/temp-history")
async def get_temp_history():
    _require_printer()
    return {"readings": [r.model_dump() for r in _printer.get_temp_history()]}


@router.post("/print")
async def start_print(req: PrintRequest):
    _require_printer()
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
    _require_printer()
    if not await _printer.pause():
        raise HTTPException(503, "Pause command rejected by printer")
    return {"status": "paused"}


@router.post("/resume")
async def resume():
    _require_printer()
    if not await _printer.resume():
        raise HTTPException(503, "Resume command rejected by printer")
    return {"status": "resumed"}


@router.post("/cancel")
async def cancel():
    _require_printer()
    if not await _printer.cancel():
        raise HTTPException(503, "Cancel command rejected by printer")
    return {"status": "cancelled"}


@router.post("/gcode")
async def send_gcode(cmd: GcodeCommand):
    printer = _require_printer()
    ok = await printer.send_gcode(cmd.command)
    if not ok:
        raise HTTPException(503, "G-code command rejected by printer")
    return {"status": "sent"}


@router.post("/move")
async def move_axis(req: MoveRequest):
    _require_printer()
    if not await _printer.move_axis(req.axis, req.distance, req.speed):
        raise HTTPException(503, "Move command failed")
    return {"status": "moved", "axis": req.axis, "distance": req.distance}


@router.post("/home")
async def home_axes(req: HomeRequest):
    _require_printer()
    if not await _printer.home(req.axes):
        raise HTTPException(503, "Home command failed")
    return {"status": "homing", "axes": req.axes or ["X", "Y", "Z"]}


@router.post("/temperature")
async def set_temperature(req: TemperatureRequest):
    _require_printer()
    if req.hotend is None and req.bed is None:
        raise HTTPException(400, "Specify at least one of hotend or bed temperature")
    if not await _printer.set_temperature(req.hotend, req.bed):
        raise HTTPException(503, "Temperature command failed")
    return {"status": "set"}


@router.post("/fan")
async def set_fan(req: FanRequest):
    _require_printer()
    if not await _printer.set_fan(req.speed_percent):
        raise HTTPException(503, "Fan command failed")
    return {"status": "set", "speed_percent": req.speed_percent}


@router.post("/extrude")
async def extrude(req: ExtrudeRequest):
    _require_printer()
    if not await _printer.extrude(req.distance_mm, req.speed_mm_per_min):
        raise HTTPException(503, "Extrude command failed")
    return {"status": "extruded", "distance_mm": req.distance_mm}


@router.get("/camera-url")
async def get_camera_url():
    url = _printer.get_camera_url()
    if not url:
        raise HTTPException(404, "No camera available")
    return {"url": url}

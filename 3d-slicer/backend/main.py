import asyncio
import sys
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from api import routes_filament, routes_history, routes_mesh, routes_printer, routes_slicer
from api.websockets import ws_manager
from config import settings
from monitoring.ai_detector import AIMonitor, FailureDetector
from monitoring.camera_stream import camera_stream
from monitoring.print_history import PrintHistoryService
from monitoring.timelapse_service import TimelapseService
from printer.printer_manager import PrinterManager
from printer.prusalink_driver import PrinterState
from slicer.filament_service import FilamentService
from slicer.mesh_service import MeshService
from slicer.slicer_service import SlicerService


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Verify CuraEngine binary exists on startup (warn, don't crash)
    if not settings.cura_engine_path.exists():
        print(
            f"WARNING: CuraEngine binary not found at {settings.cura_engine_path}.\n"
            "Run scripts/download_curaengine.sh to download it.",
            file=sys.stderr,
        )

    # Start print status polling loop
    app.state.poll_task = asyncio.create_task(
        _poll_printer_status(app.state.printer, app.state.history, app.state.filament)
    )

    yield

    # Cleanup
    app.state.poll_task.cancel()
    await camera_stream.stop()
    await app.state.printer.disconnect()


app = FastAPI(title="3D Slicer Backend", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Instantiate services
mesh_service = MeshService(settings.temp_dir)
slicer_service = SlicerService(
    cura_engine_path=settings.cura_engine_path,
    cura_definitions_dir=settings.cura_definitions_dir,
    cura_profiles_dir=settings.cura_profiles_dir,
    gcode_dir=settings.gcode_dir,
)
printer_manager = PrinterManager()
timelapse_service = TimelapseService(settings.frames_dir)
history_service = PrintHistoryService(settings.history_file)
filament_service = FilamentService(settings.filament_file)
failure_detector = FailureDetector(settings.ai_model_path, settings.ai_alert_threshold)
ai_monitor = AIMonitor(failure_detector, settings.ai_inference_interval_seconds)

# Store on app.state for cross-module access
app.state.mesh = mesh_service
app.state.slicer = slicer_service
app.state.printer = printer_manager
app.state.timelapse = timelapse_service
app.state.history = history_service
app.state.filament = filament_service
app.state.ai_monitor = ai_monitor

# Wire up routers
routes_mesh.init(mesh_service)
routes_slicer.init(slicer_service, settings.temp_dir)
routes_printer.init(printer_manager, slicer_service)
routes_history.init(history_service)
routes_filament.init(filament_service)

app.include_router(routes_mesh.router, prefix="/api/mesh", tags=["mesh"])
app.include_router(routes_slicer.router, prefix="/api/slice", tags=["slicer"])
app.include_router(routes_printer.router, prefix="/api/printer", tags=["printer"])
app.include_router(routes_history.router, prefix="/api/history", tags=["history"])
app.include_router(routes_filament.router, prefix="/api/filament", tags=["filament"])


@app.get("/health")
async def health():
    return {
        "status": "ok",
        "cura_engine_ready": settings.cura_engine_path.exists(),
        "ai_model_ready": failure_detector.is_available,
        "printer_connected": printer_manager.is_connected,
    }


@app.websocket("/ws/{channel}")
async def websocket_endpoint(websocket: WebSocket, channel: str):
    await ws_manager.connect(channel, websocket)
    try:
        while True:
            # Keep alive — client can send pings
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(channel, websocket)


@app.get("/api/monitoring/camera/snapshot")
async def camera_snapshot():
    from fastapi.responses import Response
    # Prefer the connected printer's camera (PrusaLink JPEG snapshot).
    if printer_manager.camera_available:
        frame = await printer_manager.get_snapshot()
        if frame:
            return Response(content=frame, media_type="image/jpeg")
    # Fall back to a manually-configured MJPEG stream (e.g. external webcam).
    frame = camera_stream.get_latest_frame()
    if frame is None:
        return Response(status_code=204)
    return Response(content=frame, media_type="image/jpeg")


@app.post("/api/monitoring/camera/connect")
async def connect_camera(body: dict):
    url = body.get("url")
    if not url:
        from fastapi import HTTPException
        raise HTTPException(400, "url required")
    camera_stream.set_url(url)
    await camera_stream.start()
    return {"status": "streaming", "url": url}


@app.post("/api/monitoring/ai/enable")
async def enable_ai():
    ai_monitor.enable()
    await ai_monitor.run(
        camera_stream,
        on_detection=lambda r: ws_manager.broadcast("ai_status", "detection", {
            "probability": r.failure_probability
        }),
        on_alert=lambda r: ws_manager.broadcast("ai_status", "alert", {
            "message": "Possible print failure detected",
            "probability": r.failure_probability,
        }),
    )
    return {"status": "enabled"}


@app.post("/api/monitoring/ai/disable")
async def disable_ai():
    ai_monitor.disable()
    await ai_monitor.stop()
    return {"status": "disabled"}


@app.get("/api/timelapse")
async def list_timelapses():
    return {"jobs": timelapse_service.list_jobs()}


@app.post("/api/timelapse/{job_id}/compile")
async def compile_timelapse(job_id: str):
    path = await timelapse_service.compile(job_id)
    if not path:
        from fastapi import HTTPException
        raise HTTPException(422, "No frames found or FFmpeg failed")
    return {"status": "compiled", "path": str(path)}


_DONE_STATES = {PrinterState.IDLE, PrinterState.READY, PrinterState.STOPPED}


async def _poll_printer_status(
    printer: PrinterManager,
    history: PrintHistoryService,
    filament: FilamentService,
) -> None:
    """Poll the printer, broadcast status, and drive the print-history /
    filament lifecycle off state transitions."""
    while True:
        await asyncio.sleep(2)
        if not printer.is_connected:
            continue
        status = await printer.get_status()
        await ws_manager.broadcast("print_status", "status", status.model_dump())

        try:
            _track_lifecycle(status, history, filament)
        except Exception as exc:  # never let bookkeeping kill the poll loop
            print(f"print-history tracking error: {exc}", file=sys.stderr)


def _track_lifecycle(status, history: PrintHistoryService, filament: FilamentService) -> None:
    state = status.state
    progress = status.progress_percent or 0.0
    active = history.active()

    if active is None:
        # Open a record when a print begins.
        if state in (PrinterState.PRINTING, PrinterState.PAUSED):
            pending = printer_manager.take_pending_print() or {}
            history.start_record(
                filename=status.filename or pending.get("filename") or "Print",
                slice_job_id=pending.get("slice_job_id"),
                filament_used_g=pending.get("filament_used_g", 0.0),
            )
        return

    # A record is open — decide whether it has ended.
    if state == PrinterState.FINISHED or (state in _DONE_STATES and progress >= 99):
        record = history.complete_record("completed", progress)
        if record and record.filament_used_g > 0:
            filament.deduct_from_active(record.filament_used_g)
    elif state == PrinterState.ERROR:
        history.complete_record("failed", progress)
    elif state in _DONE_STATES:
        history.complete_record("cancelled", progress)
    else:
        history.update_progress(progress)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=settings.backend_host, port=settings.backend_port, reload=False)

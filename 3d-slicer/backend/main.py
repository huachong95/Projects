import asyncio
import sys
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from api import routes_mesh, routes_printer, routes_slicer
from api.websockets import ws_manager
from config import settings
from monitoring.ai_detector import AIMonitor, FailureDetector
from monitoring.camera_stream import camera_stream
from monitoring.timelapse_service import TimelapseService
from printer.printer_manager import PrinterManager
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
    app.state.poll_task = asyncio.create_task(_poll_printer_status(app.state.printer))

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
    timeout_seconds=settings.slice_timeout_seconds,
)
printer_manager = PrinterManager()
timelapse_service = TimelapseService(settings.frames_dir)
failure_detector = FailureDetector(settings.ai_model_path, settings.ai_alert_threshold)
ai_monitor = AIMonitor(failure_detector, settings.ai_inference_interval_seconds)

# Store on app.state for cross-module access
app.state.mesh = mesh_service
app.state.slicer = slicer_service
app.state.printer = printer_manager
app.state.timelapse = timelapse_service
app.state.ai_monitor = ai_monitor

# Wire up routers
routes_mesh.init(mesh_service)
routes_slicer.init(slicer_service, settings.temp_dir)
routes_printer.init(printer_manager, slicer_service)

app.include_router(routes_mesh.router, prefix="/api/mesh", tags=["mesh"])
app.include_router(routes_slicer.router, prefix="/api/slice", tags=["slicer"])
app.include_router(routes_printer.router, prefix="/api/printer", tags=["printer"])


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


@app.get("/api/monitoring/ai/status")
async def ai_status():
    frame = camera_stream.get_latest_frame()
    result = failure_detector.analyze_frame(frame) if frame else None
    return {
        "enabled": ai_monitor._enabled,
        "model_ready": failure_detector.is_available,
        "failure_probability": result.failure_probability if result else 0.0,
        "is_alert": result.is_alert if result else False,
    }


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


async def _poll_printer_status(printer: PrinterManager) -> None:
    while True:
        await asyncio.sleep(2)
        if printer.is_connected:
            status = await printer.get_status()
            await ws_manager.broadcast("print_status", "status", status.model_dump())


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host=settings.backend_host, port=settings.backend_port, reload=False)

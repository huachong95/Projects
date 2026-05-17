from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from api.websockets import ws_manager
from slicer.gcode_processor import LayerData, extract_layer_data
from slicer.slicer_service import SliceJob, SliceSettings, SlicerService, SliceState

router = APIRouter()
_slicer: SlicerService = None
_temp_dir: Path = None


def init(slicer_service: SlicerService, temp_dir: Path) -> None:
    global _slicer, _temp_dir
    _slicer = slicer_service
    _temp_dir = temp_dir


class StartSliceRequest(BaseModel):
    mesh_job_id: str
    settings: SliceSettings = SliceSettings()
    timelapse_hooks: bool = True


@router.post("/start", response_model=SliceJob)
async def start_slice(req: StartSliceRequest):
    stl_path = _temp_dir / req.mesh_job_id / "model.stl"
    if not stl_path.exists():
        raise HTTPException(404, f"Mesh job {req.mesh_job_id} not found")

    async def on_progress(pct: float):
        await ws_manager.broadcast("slice_progress", "progress", {"percent": pct})

    async def on_complete(finished_job: SliceJob):
        await ws_manager.broadcast("slice_progress", "complete", {
            "slice_job_id": finished_job.slice_job_id,
            "layer_count": finished_job.metadata.layer_count if finished_job.metadata else 0,
        })

    job = await _slicer.start_slice(
        stl_path=stl_path,
        mesh_job_id=req.mesh_job_id,
        settings=req.settings,
        progress_callback=on_progress,
        completion_callback=on_complete,
        timelapse_hooks=req.timelapse_hooks,
    )
    return job


@router.get("/profiles")
async def list_profiles():
    return {"profiles": _slicer.list_profiles()}


@router.get("/{slice_job_id}/status", response_model=SliceJob)
async def get_slice_status(slice_job_id: str):
    job = _slicer.get_job(slice_job_id)
    if not job:
        raise HTTPException(404, "Slice job not found")
    return job


@router.get("/{slice_job_id}/gcode")
async def download_gcode(slice_job_id: str):
    job = _slicer.get_job(slice_job_id)
    if not job or job.state != SliceState.COMPLETE:
        raise HTTPException(404, "Slice job not complete")
    gcode_path = Path(job.processed_gcode_path)
    if not gcode_path.exists():
        raise HTTPException(404, "G-code file not found")
    return FileResponse(str(gcode_path), filename=f"print_{slice_job_id}.gcode", media_type="text/plain")


@router.get("/{slice_job_id}/metadata")
async def get_metadata(slice_job_id: str):
    job = _slicer.get_job(slice_job_id)
    if not job or job.state != SliceState.COMPLETE:
        raise HTTPException(404, "Slice job not complete")
    return job.metadata


@router.get("/{slice_job_id}/layer/{layer_index}")
async def get_layer(slice_job_id: str, layer_index: int):
    job = _slicer.get_job(slice_job_id)
    if not job or job.state != SliceState.COMPLETE:
        raise HTTPException(404, "Slice job not complete")
    gcode_path = Path(job.processed_gcode_path)
    layer_data = extract_layer_data(gcode_path, layer_index)
    if layer_data is None:
        raise HTTPException(404, f"Layer {layer_index} not found")
    return layer_data

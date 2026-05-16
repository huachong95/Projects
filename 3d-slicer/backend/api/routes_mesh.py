from fastapi import APIRouter, File, HTTPException, UploadFile
from fastapi.responses import Response

from slicer.mesh_service import MeshInfo, MeshService

router = APIRouter()
_mesh_service: MeshService = None


def init(mesh_service: MeshService) -> None:
    global _mesh_service
    _mesh_service = mesh_service


@router.post("/upload", response_model=MeshInfo)
async def upload_mesh(file: UploadFile = File(...)):
    if not file.filename:
        raise HTTPException(400, "No filename provided")
    suffix = file.filename.rsplit(".", 1)[-1].lower()
    if suffix not in {"stl", "3mf", "obj"}:
        raise HTTPException(400, f"Unsupported format: {suffix}. Use STL, 3MF, or OBJ.")
    contents = await file.read()
    try:
        return await _mesh_service.load_and_repair(contents, file.filename)
    except Exception as e:
        raise HTTPException(422, str(e))


@router.get("/{job_id}")
async def get_mesh_info(job_id: str):
    stl = _mesh_service.get_stl_bytes(job_id)
    if stl is None:
        raise HTTPException(404, "Job not found")
    return {"job_id": job_id, "has_stl": True}


@router.get("/{job_id}/download")
async def download_mesh(job_id: str):
    stl = _mesh_service.get_stl_bytes(job_id)
    if stl is None:
        raise HTTPException(404, "Job not found")
    return Response(content=stl, media_type="application/octet-stream")


@router.delete("/{job_id}")
async def delete_mesh(job_id: str):
    _mesh_service.cleanup(job_id)
    return {"status": "deleted"}

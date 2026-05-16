import uuid
from pathlib import Path
from typing import Optional
import trimesh
import numpy as np
from pydantic import BaseModel


class MeshInfo(BaseModel):
    job_id: str
    filename: str
    format: str
    triangle_count: int
    vertex_count: int
    is_watertight: bool
    was_repaired: bool
    dimensions_mm: dict[str, float]
    volume_cm3: float
    stl_path: str


class MeshService:
    def __init__(self, temp_dir: Path):
        self.temp_dir = temp_dir

    async def load_and_repair(self, file_bytes: bytes, filename: str) -> MeshInfo:
        job_id = str(uuid.uuid4())
        job_dir = self.temp_dir / job_id
        job_dir.mkdir(parents=True, exist_ok=True)

        suffix = Path(filename).suffix.lower()
        raw_path = job_dir / f"raw{suffix}"
        raw_path.write_bytes(file_bytes)

        mesh = self._load_mesh(raw_path, suffix)
        was_repaired = False

        if not mesh.is_watertight:
            try:
                trimesh.repair.fix_normals(mesh)
                trimesh.repair.fill_holes(mesh)
            except Exception:
                pass
            was_repaired = True

        stl_path = job_dir / "model.stl"
        mesh.export(str(stl_path))

        bounds = mesh.bounds
        dimensions = {
            "x": float(bounds[1][0] - bounds[0][0]),
            "y": float(bounds[1][1] - bounds[0][1]),
            "z": float(bounds[1][2] - bounds[0][2]),
        }

        volume_cm3 = float(abs(mesh.volume) / 1000.0) if mesh.is_volume else 0.0

        return MeshInfo(
            job_id=job_id,
            filename=filename,
            format=suffix.lstrip(".").upper(),
            triangle_count=len(mesh.faces),
            vertex_count=len(mesh.vertices),
            is_watertight=mesh.is_watertight,
            was_repaired=was_repaired,
            dimensions_mm=dimensions,
            volume_cm3=volume_cm3,
            stl_path=str(stl_path),
        )

    def _load_mesh(self, path: Path, suffix: str) -> trimesh.Trimesh:
        loaded = trimesh.load(str(path), force="mesh")
        if isinstance(loaded, trimesh.Scene):
            meshes = [g for g in loaded.geometry.values()]
            if not meshes:
                raise ValueError("No geometry found in file")
            loaded = trimesh.util.concatenate(meshes)
        return loaded

    def get_stl_bytes(self, job_id: str) -> Optional[bytes]:
        stl_path = self.temp_dir / job_id / "model.stl"
        if not stl_path.exists():
            return None
        return stl_path.read_bytes()

    def cleanup(self, job_id: str) -> None:
        import shutil
        job_dir = self.temp_dir / job_id
        if job_dir.exists():
            shutil.rmtree(job_dir)

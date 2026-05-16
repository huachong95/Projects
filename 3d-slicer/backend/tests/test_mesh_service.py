import asyncio
import io
import struct
from pathlib import Path

import numpy as np
import pytest

from slicer.mesh_service import MeshService


def _make_binary_stl(triangles: list) -> bytes:
    """Build a minimal binary STL from a list of (normal, v1, v2, v3) tuples."""
    header = b"\x00" * 80
    count = struct.pack("<I", len(triangles))
    body = b""
    for normal, v1, v2, v3 in triangles:
        body += struct.pack("<fff", *normal)
        body += struct.pack("<fff", *v1)
        body += struct.pack("<fff", *v2)
        body += struct.pack("<fff", *v3)
        body += struct.pack("<H", 0)  # attribute
    return header + count + body


def _unit_tetrahedron_stl() -> bytes:
    return _make_binary_stl([
        ((0, 0, -1), (0, 0, 0), (1, 0, 0), (0, 1, 0)),
        ((0, -1, 0), (0, 0, 0), (1, 0, 0), (1, 0, 1)),
        ((-1, 0, 0), (0, 0, 0), (0, 1, 0), (0, 0, 1)),
        ((1, 1, 1), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
    ])


@pytest.fixture
def tmp_dir(tmp_path):
    return tmp_path


@pytest.fixture
def service(tmp_dir):
    return MeshService(tmp_dir)


def test_load_binary_stl(service):
    stl_bytes = _unit_tetrahedron_stl()
    info = asyncio.run(service.load_and_repair(stl_bytes, "tetra.stl"))
    assert info.filename == "tetra.stl"
    assert info.format == "STL"
    assert info.triangle_count >= 4
    assert info.dimensions_mm["x"] > 0
    assert info.dimensions_mm["y"] > 0
    assert info.dimensions_mm["z"] > 0


def test_stl_bytes_accessible_after_load(service):
    stl_bytes = _unit_tetrahedron_stl()
    info = asyncio.run(service.load_and_repair(stl_bytes, "tetra.stl"))
    retrieved = service.get_stl_bytes(info.job_id)
    assert retrieved is not None
    assert len(retrieved) > 0


def test_cleanup_removes_files(service, tmp_dir):
    stl_bytes = _unit_tetrahedron_stl()
    info = asyncio.run(service.load_and_repair(stl_bytes, "tetra.stl"))
    job_dir = tmp_dir / info.job_id
    assert job_dir.exists()
    service.cleanup(info.job_id)
    assert not job_dir.exists()


def test_get_stl_bytes_missing_job(service):
    result = service.get_stl_bytes("nonexistent-job-id")
    assert result is None

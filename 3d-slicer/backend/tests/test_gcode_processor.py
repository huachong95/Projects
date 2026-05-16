from pathlib import Path
import textwrap

from slicer.gcode_processor import extract_metadata, insert_timelapse_hooks, extract_layer_data


SAMPLE_GCODE = textwrap.dedent("""\
    ;Flavor:Marlin
    ;TIME:3600
    ;Filament used: 1234.5mm
    ;Filament used: 3.1cm3
    ;LAYER:0
    G1 X10.0 Y10.0 Z0.2 F3000
    G1 X20.0 Y10.0 E1.0
    G1 X20.0 Y20.0 E2.0
    ;TYPE:WALL-OUTER
    G1 X30.0 Y30.0 E3.0
    ;LAYER:1
    G1 X0.0 Y0.0 Z0.4 F3000
    G1 X10.0 Y10.0 E4.0
    ;LAYER:2
    G1 X0.0 Y0.0 Z0.6
""")


def _write_gcode(tmp_path: Path, content: str) -> Path:
    p = tmp_path / "test.gcode"
    p.write_text(content)
    return p


def test_extract_metadata_layer_count(tmp_path):
    p = _write_gcode(tmp_path, SAMPLE_GCODE)
    meta = extract_metadata(p)
    assert meta.layer_count == 3


def test_extract_metadata_time(tmp_path):
    p = _write_gcode(tmp_path, SAMPLE_GCODE)
    meta = extract_metadata(p)
    assert meta.estimated_time_seconds == 3600


def test_extract_metadata_filament(tmp_path):
    p = _write_gcode(tmp_path, SAMPLE_GCODE)
    meta = extract_metadata(p)
    assert abs(meta.filament_used_mm - 1234.5) < 0.01
    assert abs(meta.filament_used_cm3 - 3.1) < 0.01


def test_timelapse_hooks_inserted(tmp_path):
    src = _write_gcode(tmp_path, SAMPLE_GCODE)
    dst = tmp_path / "hooked.gcode"
    insert_timelapse_hooks(src, dst)
    content = dst.read_text()
    assert content.count("HOST_ACTION:TIMELAPSE_CAPTURE") == 3


def test_extract_layer_data_perimeters(tmp_path):
    p = _write_gcode(tmp_path, SAMPLE_GCODE)
    layer = extract_layer_data(p, 0)
    assert layer is not None
    assert layer.layer_index == 0

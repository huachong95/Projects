import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class GcodeMetadata:
    layer_count: int = 0
    estimated_time_seconds: int = 0
    filament_used_mm: float = 0.0
    filament_used_cm3: float = 0.0
    filament_used_g: float = 0.0


@dataclass
class LayerData:
    layer_index: int
    z_height: float
    perimeter_lines: list[list[tuple[float, float]]] = field(default_factory=list)
    infill_lines: list[list[tuple[float, float]]] = field(default_factory=list)
    support_lines: list[list[tuple[float, float]]] = field(default_factory=list)
    travel_lines: list[list[tuple[float, float]]] = field(default_factory=list)


_TIME_RE = re.compile(r";TIME:(\d+)")
_LAYER_RE = re.compile(r";LAYER:(\d+)")
_LAYER_Z_RE = re.compile(r";LAYER_CHANGE")
_FILAMENT_RE = re.compile(r";Filament used: ([\d.]+)mm")
_FILAMENT_CM3_RE = re.compile(r";Filament used: ([\d.]+)cm3")
_G1_RE = re.compile(r"G[01]\s+(?:X([\d.-]+))?\s*(?:Y([\d.-]+))?\s*(?:Z([\d.-]+))?\s*(?:E([\d.-]+))?")


def extract_metadata(gcode_path: Path) -> GcodeMetadata:
    meta = GcodeMetadata()
    with open(gcode_path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if m := _TIME_RE.match(line):
                meta.estimated_time_seconds = int(m.group(1))
            elif m := _LAYER_RE.match(line):
                meta.layer_count = max(meta.layer_count, int(m.group(1)) + 1)
            elif m := _FILAMENT_RE.match(line):
                meta.filament_used_mm = float(m.group(1))
            elif m := _FILAMENT_CM3_RE.match(line):
                meta.filament_used_cm3 = float(m.group(1))
                meta.filament_used_g = meta.filament_used_cm3 * 1.24  # PLA density
    return meta


def insert_timelapse_hooks(gcode_path: Path, output_path: Path, park_x: float = 5.0, park_y: float = 5.0) -> None:
    """Insert G-code hooks after each layer change for synchronized timelapse capture."""
    hook = (
        f"G4 P500 ; dwell for vibration settle\n"
        f"M400 ; wait for moves\n"
        f"; HOST_ACTION:TIMELAPSE_CAPTURE\n"
        f"G1 X{park_x:.1f} Y{park_y:.1f} F9000 ; park nozzle\n"
    )

    with open(gcode_path, "r", errors="replace") as src, open(output_path, "w") as dst:
        for line in src:
            dst.write(line)
            if _LAYER_RE.match(line.strip()):
                dst.write(hook)


def extract_layer_data(gcode_path: Path, layer_index: int) -> Optional[LayerData]:
    """Extract 2D toolpath polylines for a specific layer (lazy, on-demand)."""
    target_layer = layer_index
    in_target = False
    current_z = 0.0
    layer_data = None

    current_x, current_y = 0.0, 0.0
    current_line: list[tuple[float, float]] = []
    current_type = "travel"

    type_map = {
        ";TYPE:WALL-OUTER": "perimeter",
        ";TYPE:WALL-INNER": "perimeter",
        ";TYPE:FILL": "infill",
        ";TYPE:SUPPORT": "support",
        ";TYPE:TRAVEL": "travel",
        ";TYPE:SKIRT": "travel",
    }

    def flush_line(data: LayerData, line_pts: list, ltype: str) -> list:
        if len(line_pts) >= 2:
            if ltype == "perimeter":
                data.perimeter_lines.append(line_pts[:])
            elif ltype == "infill":
                data.infill_lines.append(line_pts[:])
            elif ltype == "support":
                data.support_lines.append(line_pts[:])
            else:
                data.travel_lines.append(line_pts[:])
        return []

    with open(gcode_path, "r", errors="replace") as f:
        for line in f:
            stripped = line.strip()

            if m := _LAYER_RE.match(stripped):
                layer_num = int(m.group(1))
                if layer_num == target_layer:
                    in_target = True
                    layer_data = LayerData(layer_index=layer_num, z_height=current_z)
                    current_line = []
                elif in_target:
                    break

            if in_target:
                if stripped in type_map:
                    current_line = flush_line(layer_data, current_line, current_type)
                    current_type = type_map[stripped]

                if m := _G1_RE.match(stripped):
                    nx = float(m.group(1)) if m.group(1) else current_x
                    ny = float(m.group(2)) if m.group(2) else current_y
                    nz = float(m.group(3)) if m.group(3) else current_z
                    has_e = m.group(4) is not None

                    if nz != current_z:
                        current_z = nz
                        # Update z_height to the actual Z seen in this layer's moves.
                        # The ;LAYER: comment appears before the G1 Z move, so z_height
                        # is always 0.0 at LayerData creation time — fix it here.
                        if layer_data is not None:
                            layer_data.z_height = nz
                    if has_e:
                        if not current_line:
                            current_line.append((current_x, current_y))
                        current_line.append((nx, ny))
                    else:
                        current_line = flush_line(layer_data, current_line, current_type)
                        current_line = [(nx, ny)]
                        current_type = "travel"

                    current_x, current_y = nx, ny

        if layer_data:
            flush_line(layer_data, current_line, current_type)

    return layer_data

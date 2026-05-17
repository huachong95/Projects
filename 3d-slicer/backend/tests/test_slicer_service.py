import json
from pathlib import Path

import pytest

from slicer.slicer_service import (
    SlicerService,
    SliceSettings,
    SliceState,
    _MACHINE_SETTINGS,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def profiles_dir(tmp_path):
    d = tmp_path / "profiles"
    d.mkdir()
    (d / "base_config.json").write_text(json.dumps({
        "layer_height": 0.2,
        "infill_sparse_density": 15,
        "speed_print": 60,
    }))
    (d / "pla_standard.json").write_text(json.dumps({
        "material_print_temperature": 215,
        "material_bed_temperature": 60,
    }))
    (d / "pla_fine.json").write_text(json.dumps({
        "layer_height": 0.1,
        "material_print_temperature": 210,
    }))
    (d / "petg_standard.json").write_text(json.dumps({
        "material_print_temperature": 235,
        "material_bed_temperature": 85,
    }))
    (d / "prusa_mk4.json").write_text(json.dumps({
        "machine_name": "Prusa MK4",
        "machine_start_gcode": "G28\nG29",
        "machine_end_gcode": "M104 S0\nM140 S0",
    }))
    return d


@pytest.fixture
def definitions_dir(tmp_path):
    d = tmp_path / "definitions"
    d.mkdir()
    return d


@pytest.fixture
def slicer(tmp_path, profiles_dir, definitions_dir):
    return SlicerService(
        cura_engine_path=tmp_path / "CuraEngine",
        cura_definitions_dir=definitions_dir,
        cura_profiles_dir=profiles_dir,
        gcode_dir=tmp_path / "gcode",
    )


# ---------------------------------------------------------------------------
# list_profiles
# ---------------------------------------------------------------------------

def test_list_profiles_returns_all_four(slicer):
    assert set(slicer.list_profiles()) == {"pla_standard", "pla_fine", "petg_standard", "prusa_mk4"}


def test_list_profiles_excludes_base_config(slicer):
    assert "base_config" not in slicer.list_profiles()


# ---------------------------------------------------------------------------
# _load_profile
# ---------------------------------------------------------------------------

def test_load_profile_merges_base_defaults(slicer):
    profile = slicer._load_profile("pla_standard")
    assert profile["layer_height"] == 0.2          # from base
    assert profile["infill_sparse_density"] == 15   # from base
    assert profile["material_print_temperature"] == 215  # from pla_standard


def test_load_profile_overrides_base_value(slicer):
    profile = slicer._load_profile("pla_fine")
    assert profile["layer_height"] == 0.1  # pla_fine overrides base 0.2


def test_load_profile_unknown_name_returns_base_only(slicer):
    profile = slicer._load_profile("nonexistent")
    assert profile["layer_height"] == 0.2
    assert "material_print_temperature" not in profile


# ---------------------------------------------------------------------------
# _build_flags
# ---------------------------------------------------------------------------

def test_build_flags_produces_s_pairs(slicer):
    flags = slicer._build_flags({"layer_height": 0.2, "speed_print": 60})
    assert flags == ["-s", "layer_height=0.2", "-s", "speed_print=60"]


def test_build_flags_converts_bool_true(slicer):
    flags = slicer._build_flags({"machine_heated_bed": True})
    assert "machine_heated_bed=true" in flags


def test_build_flags_converts_bool_false(slicer):
    flags = slicer._build_flags({"retraction_enable": False})
    assert "retraction_enable=false" in flags


def test_build_flags_skips_multiline_strings(slicer):
    flags = slicer._build_flags({"machine_start_gcode": "G28\nG29", "layer_height": 0.2})
    assert "layer_height=0.2" in flags
    assert not any("machine_start_gcode" in f for f in flags)


def test_build_flags_empty_dict(slicer):
    assert slicer._build_flags({}) == []


# ---------------------------------------------------------------------------
# _parse_progress
# ---------------------------------------------------------------------------

def test_parse_progress_simple(slicer):
    assert slicer._parse_progress("Progress:50.0%") == 50.0


def test_parse_progress_cura5_format(slicer):
    # CuraEngine 5.x: "Progress:inset+skin:75.5%:Mesh 1/1"
    assert slicer._parse_progress("Progress:inset+skin:75.5%:Mesh 1/1") == 75.5


def test_parse_progress_integer_percent(slicer):
    assert slicer._parse_progress("Progress:100%") == 100.0


def test_parse_progress_caps_at_100(slicer):
    assert slicer._parse_progress("Progress:110%") == 100.0


def test_parse_progress_no_percent_returns_none(slicer):
    assert slicer._parse_progress("G1 X10.0 Y10.0") is None


def test_parse_progress_zero(slicer):
    assert slicer._parse_progress("Progress:0%") == 0.0


# ---------------------------------------------------------------------------
# _build_cmd
# ---------------------------------------------------------------------------

@pytest.fixture
def machine_def(definitions_dir):
    p = definitions_dir / "fdmprinter.def.json"
    p.write_text("{}")
    return p


def test_build_cmd_starts_with_curaengine_slice(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert cmd[0] == str(slicer.cura_engine)
    assert cmd[1] == "slice"


def test_build_cmd_includes_verbose_flag(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert "-v" in cmd


def test_build_cmd_includes_machine_def(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert str(machine_def) in cmd


def test_build_cmd_includes_e0(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert "-e0" in cmd


def test_build_cmd_machine_settings_before_e0(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(
        tmp_path / "m.stl", tmp_path / "out.gcode",
        {"machine_width": 250, "layer_height": 0.2},
    )
    e0_idx = cmd.index("-e0")
    mw_s_idx = next(i for i, v in enumerate(cmd) if v == "machine_width=250")
    assert mw_s_idx < e0_idx


def test_build_cmd_extruder_settings_after_e0(slicer, machine_def, tmp_path):
    cmd = slicer._build_cmd(
        tmp_path / "m.stl", tmp_path / "out.gcode",
        {"machine_width": 250, "layer_height": 0.2},
    )
    e0_idx = cmd.index("-e0")
    lh_s_idx = next(i for i, v in enumerate(cmd) if v == "layer_height=0.2")
    assert lh_s_idx > e0_idx


def test_build_cmd_includes_extruder_def_when_present(slicer, machine_def, definitions_dir, tmp_path):
    extruder_def = definitions_dir / "fdmextruder.def.json"
    extruder_def.write_text("{}")
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert str(extruder_def) in cmd
    # extruder def must come after -e0
    e0_idx = cmd.index("-e0")
    ed_idx = cmd.index(str(extruder_def))
    assert ed_idx > e0_idx


def test_build_cmd_omits_extruder_def_when_absent(slicer, machine_def, definitions_dir, tmp_path):
    # fdmextruder.def.json does not exist
    extruder_def = str(definitions_dir / "fdmextruder.def.json")
    cmd = slicer._build_cmd(tmp_path / "m.stl", tmp_path / "out.gcode", {})
    assert extruder_def not in cmd


def test_build_cmd_stl_and_output_at_end(slicer, machine_def, tmp_path):
    stl = tmp_path / "model.stl"
    out = tmp_path / "out.gcode"
    cmd = slicer._build_cmd(stl, out, {})
    # -l model.stl must appear before -o
    l_idx = cmd.index("-l")
    o_idx = cmd.index("-o")
    assert l_idx < o_idx
    assert cmd[l_idx + 1] == str(stl)
    assert cmd[o_idx + 1] == str(out)


# ---------------------------------------------------------------------------
# get_job
# ---------------------------------------------------------------------------

def test_get_job_returns_none_for_unknown(slicer):
    assert slicer.get_job("no-such-id") is None

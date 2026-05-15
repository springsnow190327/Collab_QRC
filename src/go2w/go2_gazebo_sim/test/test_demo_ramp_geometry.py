import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _floats(value):
    return [float(v) for v in value.split()]


def _top_edge_z(center_z, half_x, half_z, pitch_y, sign_x):
    local_x = sign_x * half_x
    local_z = half_z
    return center_z - math.sin(pitch_y) * local_x + math.cos(pitch_y) * local_z


def test_demo_ramp_top_surface_meets_ground_and_platform_without_lip():
    path = (
        Path(__file__).resolve().parents[1]
        / "mujoco"
        / "demo_ramp.xml"
    )
    root = ET.parse(path).getroot()
    ramp = root.find(".//geom[@name='ramp']")
    assert ramp is not None

    center_z = _floats(ramp.attrib["pos"])[2]
    half_x, _half_y, half_z = _floats(ramp.attrib["size"])
    pitch_y = _floats(ramp.attrib["euler"])[1]

    west_z = _top_edge_z(center_z, half_x, half_z, pitch_y, sign_x=-1.0)
    east_z = _top_edge_z(center_z, half_x, half_z, pitch_y, sign_x=1.0)

    assert abs(west_z - 0.0) <= 0.01
    assert abs(east_z - 1.0) <= 0.02
    assert ramp.attrib["condim"] == "6"
    assert float(_floats(ramp.attrib["friction"])[0]) >= 1.0

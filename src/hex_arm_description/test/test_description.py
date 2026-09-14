from pathlib import Path
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory


JOINTS = [f"joint_{index}" for index in range(1, 7)]


def _expanded(backend: str) -> ET.Element:
    source = Path(get_package_share_directory("hex_arm_description")) / "urdf" / "firefly_y6.urdf.xacro"
    result = subprocess.run(
        ["xacro", str(source), f"backend:={backend}", "controllers_file:=/tmp/controllers.yaml"],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _original() -> ET.Element:
    source = Path(get_package_share_directory("xpkg_urdf_firefly_y6")) / "urdf" / "xpkg_urdf_firefly_y6.urdf"
    return ET.parse(source).getroot()


def _attrs(element: ET.Element | None) -> dict[str, float | str] | None:
    if element is None:
        return None
    out: dict[str, float | str] = {}
    for key, value in sorted(element.attrib.items()):
        try:
            out[key] = float(value)
        except ValueError:
            out[key] = value
    return out


@pytest.mark.parametrize("backend", ["view", "mock", "gz", "real"])
def test_all_backends_expand_and_contain_exactly_six_actuated_joints(backend: str) -> None:
    robot = _expanded(backend)
    assert robot.attrib["name"] == "firefly_y6"
    assert [joint.attrib["name"] for joint in robot.findall("joint") if joint.attrib.get("type") != "fixed"] == JOINTS
    controls = robot.findall("ros2_control")
    assert len(controls) == (0 if backend == "view" else 1)


@pytest.mark.parametrize("backend", ["view", "mock", "gz", "real"])
def test_check_urdf_accepts_every_expanded_backend(backend: str) -> None:
    source = Path(get_package_share_directory("hex_arm_description")) / "urdf" / "firefly_y6.urdf.xacro"
    xml = subprocess.run(
        ["xacro", str(source), f"backend:={backend}", "controllers_file:=/tmp/controllers.yaml"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as model:
        model.write(xml)
        model.flush()
        subprocess.run(["check_urdf", model.name], check=True, capture_output=True, text=True)


def test_overlay_preserves_upstream_kinematic_and_dynamic_contract() -> None:
    original = _original()
    overlay = _expanded("real")
    for joint_name in JOINTS:
        lhs = original.find(f"joint[@name='{joint_name}']")
        rhs = overlay.find(f"joint[@name='{joint_name}']")
        assert lhs is not None and rhs is not None
        for tag in ("origin", "axis", "limit", "parent", "child"):
            assert _attrs(lhs.find(tag)) == _attrs(rhs.find(tag)), (joint_name, tag)

    for link in original.findall("link"):
        rhs = overlay.find(f"link[@name='{link.attrib['name']}']")
        assert rhs is not None
        for path in ("inertial/origin", "inertial/mass", "inertial/inertia", "visual/geometry/mesh", "collision/geometry/mesh"):
            assert _attrs(link.find(path)) == _attrs(rhs.find(path)), (link.attrib["name"], path)


def test_limits_are_finite_and_ordered() -> None:
    robot = _expanded("view")
    for name in JOINTS:
        limit = robot.find(f"joint[@name='{name}']/limit")
        assert limit is not None
        values = [float(limit.attrib[key]) for key in ("lower", "upper", "effort", "velocity")]
        assert all(math.isfinite(value) for value in values)
        assert values[0] < values[1]
        assert values[2] > 0.0 and values[3] > 0.0


def test_joint_3_zero_is_centered_at_the_previous_1_57_rad_pose() -> None:
    joint = _expanded("view").find("joint[@name='joint_3']")
    assert joint is not None
    origin = joint.find("origin")
    limit = joint.find("limit")
    assert origin is not None and limit is not None
    assert [float(value) for value in origin.attrib["rpy"].split()] == [0.0, 1.57, 0.0]
    assert (float(limit.attrib["lower"]), float(limit.attrib["upper"])) == (-1.57, 1.57)


def test_all_package_meshes_exist() -> None:
    share = Path(get_package_share_directory("xpkg_urdf_firefly_y6"))
    for mesh in _original().findall(".//mesh"):
        uri = mesh.attrib["filename"]
        prefix = "package://xpkg_urdf_firefly_y6/"
        assert uri.startswith(prefix)
        assert (share / uri.removeprefix(prefix)).is_file(), uri

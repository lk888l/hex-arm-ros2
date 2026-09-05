from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SRDF = PACKAGE_ROOT / "config" / "firefly_y6.srdf"


def test_full_range_sampling_finds_no_permanent_nonadjacent_overlap(tmp_path: Path) -> None:
    urdf = (
        Path(get_package_share_directory("xpkg_urdf_firefly_y6"))
        / "urdf"
        / "xpkg_urdf_firefly_y6.urdf"
    )
    output = tmp_path / "sampled.srdf"
    command = [
        "ros2",
        "run",
        "moveit_setup_assistant",
        "collisions_updater",
        "--urdf",
        str(urdf),
        "--srdf",
        str(SRDF),
        "--output",
        str(output),
        "--always",
        "--verbose",
        "--trials",
        "100000",
        "--min-collision-fraction",
        "0.95",
    ]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=30)
    report = completed.stdout + completed.stderr
    assert completed.returncode == 0, report
    assert "Total possible collisions : 21.000000" in report
    assert "Always in collision : 0" in report

    sampled = ET.parse(output).getroot()
    reasons = [item.attrib["reason"] for item in sampled.findall("disable_collisions")]
    assert reasons.count("Adjacent") == 6
    assert "Always" not in reasons
    # Without --keep, the updater independently rebuilds the matrix. Thus the
    # two plan-only field exceptions cannot masquerade as permanent overlap.
    assert "CommissioningFolded" not in reasons
    assert "PlanOnlySurveyedFold" not in reasons

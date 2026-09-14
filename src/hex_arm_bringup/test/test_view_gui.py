"""Exercise the actual Qt slider and Center button against the installed model."""
import os
from pathlib import Path
import subprocess

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from ament_index_python.packages import get_package_share_directory
from joint_state_publisher.joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui.joint_state_publisher_gui import JointStatePublisherGui
from python_qt_binding.QtWidgets import QApplication
import pytest
import rclpy


def test_j3_slider_and_center(tmp_path):
    model = Path(get_package_share_directory("hex_arm_description")) / "urdf/firefly_y6.urdf.xacro"
    urdf = tmp_path / "view.urdf"
    urdf.write_text(subprocess.check_output(["xacro", str(model), "backend:=view"], text=True))
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    publisher = JointStatePublisher(str(urdf))
    gui = JointStatePublisherGui("J3 regression", publisher)
    try:
        joint = publisher.free_joints["joint_3"]
        assert joint["position"] == pytest.approx(0.0)
        slider = gui.joint_map["joint_3"]["slider"]
        slider.setValue(gui.valueToSlider(0.7, joint))
        app.processEvents()
        assert joint["position"] == pytest.approx(0.7, abs=0.001)
        gui.ctr_button.click()
        app.processEvents()
        assert joint["position"] == pytest.approx(0.0)
        assert gui.joint_map["joint_3"]["display"].text() == "0.000"
    finally:
        gui.close()
        publisher.destroy_node()
        rclpy.shutdown()

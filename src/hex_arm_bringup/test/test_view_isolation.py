"""Two previews must not share state, model or TF with each other or / topics."""
import os
import signal
import subprocess
import time

import rclpy
from sensor_msgs.msg import JointState


def test_two_preview_sessions_are_isolated(tmp_path, monkeypatch):
    # Keep the deliberately conflicting publisher away from the deployment domain.
    monkeypatch.setenv("ROS_DOMAIN_ID", "197")
    # Opt into rendering on an available X server; default CI needs no OpenGL.
    use_rviz = os.environ.get("HEX_ARM_TEST_RVIZ") == "1"
    processes = []
    logs = []
    rclpy.init(args=[])
    node = rclpy.create_node("view_isolation_probe")
    try:
        for index in range(2):
            log = (tmp_path / f"view_{index}.log").open("w+")
            logs.append(log)
            processes.append(subprocess.Popen(
                ["ros2", "launch", "hex_arm_bringup", "view.launch.py",
                 f"use_rviz:={str(use_rviz).lower()}"],
                env={**os.environ, "QT_QPA_PLATFORM": "xcb" if use_rviz else "offscreen"},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            ))
        deadline = time.monotonic() + 20
        namespaces = set()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            namespaces = {ns for name, ns in node.get_node_names_and_namespaces()
                          if name == "robot_state_publisher" and ns.startswith("/hex_arm_view_")}
            if len(namespaces) == 2 and all(
                node.count_publishers(ns + "/joint_states") == 1 for ns in namespaces
            ):
                if not use_rviz or all(
                    any(s.node_name.startswith("rviz") for s in
                        node.get_subscriptions_info_by_topic(ns + "/robot_description"))
                    for ns in namespaces
                ):
                    break
        assert len(namespaces) == 2
        subscriptions = []
        received = {ns: [] for ns in namespaces}
        for ns in namespaces:
            for topic in ("joint_states", "robot_description", "tf", "tf_static"):
                assert node.count_publishers(ns + "/" + topic) == 1
            subscriptions.append(node.create_subscription(
                JointState, ns + "/joint_states", received[ns].append, 10))
        # An unrelated controller publishing on the global topic must not move either preview.
        interference = node.create_publisher(JointState, "/joint_states", 10)
        until = time.monotonic() + 2
        while time.monotonic() < until:
            interference.publish(JointState(name=["joint_3"], position=[1.2]))
            rclpy.spin_once(node, timeout_sec=0.05)
        for messages in received.values():
            assert len(messages) >= 5
            assert all(abs(msg.position[msg.name.index("joint_3")]) < 1e-6 for msg in messages)
        for ns in namespaces:
            subscribers = node.get_subscriptions_info_by_topic(ns + "/joint_states")
            assert any(s.node_name == "robot_state_publisher" and s.node_namespace == ns
                       for s in subscribers)
            for topic in (("robot_description", "tf", "tf_static") if use_rviz else ()):
                subscribers = node.get_subscriptions_info_by_topic(ns + "/" + topic)
                assert any(s.node_name.startswith(("rviz", "transform_listener"))
                           and s.node_namespace == ns for s in subscribers), (
                               ns, topic, [(s.node_name, s.node_namespace) for s in subscribers])
        assert all(p.poll() is None for p in processes)
    finally:
        # Signal whole test process groups and wait; no background GUI may survive the test.
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

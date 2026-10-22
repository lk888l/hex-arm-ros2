#!/usr/bin/env python3
"""Keep ROS alive until the instance stop owner has completed disable confirmation."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from hex_arm_msgs.action import StopArm
from hex_arm_msgs.msg import DriverState
from hex_arm_bringup.integration import add_instance_arguments, instance_from_args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_instance_arguments(parser)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    instance = instance_from_args(args)
    rclpy.init()
    node = Node("hex_arm_stop_request", namespace=instance.namespace)
    client = ActionClient(node, StopArm, instance.ros_name("hex_arm/stop"))
    driver = [None, 0., None]
    def on_driver(message):
        stamp = (message.stamp.sec, message.stamp.nanosec)
        if stamp != (0, 0) and (driver[2] is None or stamp > driver[2]):
            driver[:] = [message, time.monotonic(), stamp]
    node.create_subscription(DriverState, instance.ros_name("hex_arm/driver_state"), on_driver, 10)
    handle = None
    report = {"passed": False, "motion_completed": False, "disabled_confirmed": False,
              "instance": instance.as_dict()}
    def wait(future, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError("stop action did not finish before deadline")
        return future.result()
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("instance stop action unavailable")
        handle = wait(client.send_goal_async(StopArm.Goal(mode=StopArm.Goal.NORMAL)), 5.0)
        if not handle.accepted:
            # A completed standalone campaign may already have disabled the
            # hardware. Confirm that state through the owner, without motion.
            deadline = time.monotonic() + .3
            while driver[0] is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            state = driver[0]
            if not (state and time.monotonic() - driver[1] <= .2 and state.feedback_fresh
                    and state.mode == DriverState.MODE_DISABLED
                    and not state.fault_latched):
                raise RuntimeError("instance normal stop rejected")
            report['motion_skipped'] = 'already_disabled'
            handle = wait(client.send_goal_async(StopArm.Goal(mode=StopArm.Goal.IMMEDIATE)), 5.0)
            if not handle.accepted:
                raise RuntimeError("instance disable confirmation rejected")
        wrapped = wait(handle.get_result_async(), 125.0)
        result = wrapped.result
        report.update(passed=wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success,
                      motion_completed=result.motion_completed, disabled_confirmed=result.disabled_confirmed,
                      motion_report=result.report_file, message=result.message, status=wrapped.status)
        handle = None
        return 0 if report["passed"] else 1
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        if handle is not None and handle.accepted and rclpy.ok():
            try:
                wait(handle.cancel_goal_async(), 3.0)
            except Exception:
                pass
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

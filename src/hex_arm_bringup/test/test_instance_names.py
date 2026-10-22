"""Names and report identity must keep integrated arms isolated."""
import argparse

import pytest

from hex_arm_bringup.integration import (
    InstanceNames, add_instance_arguments, instance_from_args, verify_report_instance,
)


def test_defaults_match_existing_interfaces():
    instance = InstanceNames()
    assert instance.hardware_name == "FireflyY6System"
    assert instance.ros_name("hex_arm/internal/state") == "/hex_arm/internal/state"
    assert instance.controller_ros_name() == "/firefly_arm_controller"
    assert instance.manager_service("switch_controller") == "/controller_manager/switch_controller"


def test_shared_manager_action_namespace_is_independent_of_arm_namespace():
    left = InstanceNames(namespace="cell/left/", joint_prefix="left_",
                         controller_manager="/cell/control/manager", controller_name="left_arm")
    right = InstanceNames(namespace="cell/right", joint_prefix="right_",
                          controller_manager="/cell/control/manager", controller_name="right_arm")
    assert left.ros_name("hex_arm/driver_state") == "/cell/left/hex_arm/driver_state"
    assert left.ros_name("/cell/shared/topic") == "/cell/shared/topic"
    assert left.manager_service("switch_controller") == "/cell/control/manager/switch_controller"
    assert left.controller_ros_name() == "/cell/control/left_arm"
    assert right.controller_ros_name() == "/cell/control/right_arm"
    assert set(left.joint_names).isdisjoint(right.joint_names)


@pytest.mark.parametrize("values", [
    {"namespace": "bad//arm"}, {"namespace": "1robot"}, {"joint_prefix": "left/"},
    {"hardware_name": "bad name"}, {"controller_name": "a/b"},
    {"controller_manager": "/manager//child"}, {"planning_group": ""},
])
def test_invalid_instance_names_are_rejected_before_ros_creation(values):
    with pytest.raises(ValueError):
        InstanceNames(**values)


def test_cli_round_trip_and_report_binding():
    instance = InstanceNames(namespace="left", joint_prefix="left_", hardware_name="LeftArm",
                             controller_manager="/manager", controller_name="left_arm")
    parser = argparse.ArgumentParser()
    add_instance_arguments(parser)
    assert instance_from_args(parser.parse_args(instance.cli_args())) == instance
    verify_report_instance({"instance": instance.as_dict()}, instance)
    verify_report_instance({}, InstanceNames())
    with pytest.raises(RuntimeError, match="no instance identity"):
        verify_report_instance({}, instance)
    with pytest.raises(RuntimeError, match="different robot instance"):
        verify_report_instance({"instance": InstanceNames().as_dict()}, instance)

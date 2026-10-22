"""Instance naming shared by launch descriptions and commissioning clients.

Hardware profiles always retain their physical joint_1..joint_6 names. Prefixes
apply at the ROS boundary only, including collision-body names in the model.
"""
from dataclasses import asdict, dataclass, fields
import re


def _token(value, label):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", value):
        raise ValueError(f"{label} must be a valid ROS name token")
    return value


def _path(value, label):
    if not isinstance(value, str) or not value or "//" in value:
        raise ValueError(f"{label} must be a valid ROS path")
    for part in value.strip("/").split("/"):
        _token(part, label)
    return value.rstrip("/")


@dataclass(frozen=True)
class InstanceNames:
    namespace: str = ""
    joint_prefix: str = ""
    hardware_name: str = "FireflyY6System"
    controller_manager: str = "controller_manager"
    controller_name: str = "firefly_arm_controller"
    move_group_name: str = "move_group"
    planning_group: str = "arm"

    def __post_init__(self):
        namespace = self.namespace
        if namespace and namespace != "/":
            namespace = "/" + _path(namespace, "namespace").strip("/")
        else:
            namespace = ""
        object.__setattr__(self, "namespace", namespace)
        if not isinstance(self.joint_prefix, str):
            raise ValueError("joint_prefix must be a string")
        if self.joint_prefix:
            _token(self.joint_prefix + "joint_1", "joint_prefix")
        for name in ("hardware_name", "controller_name", "move_group_name", "planning_group"):
            _token(getattr(self, name), name)
        object.__setattr__(self, "controller_manager", _path(self.controller_manager, "controller_manager"))

    def ros_name(self, relative):
        """Resolve a relative interface, preserving an explicit absolute path."""
        relative = _path(relative, "ROS interface")
        return relative if relative.startswith("/") else f"{self.namespace}/{relative}"

    @property
    def joint_names(self):
        return [self.joint_prefix + f"joint_{index}" for index in range(1, 7)]

    def joint_name(self, physical):
        if physical not in [f"joint_{index}" for index in range(1, 7)]:
            raise ValueError(f"unknown physical joint: {physical}")
        return self.joint_prefix + physical

    def manager_service(self, service):
        return self.ros_name(self.controller_manager + "/" + service)

    def controller_ros_name(self, name=None):
        manager = self.ros_name(self.controller_manager)
        parent = manager.rsplit("/", 1)[0]
        name = self.controller_name if name is None else _token(name, "controller name")
        return parent + "/" + name

    def as_dict(self):
        return asdict(self)

    def cli_args(self):
        return [argument for key, value in self.as_dict().items()
                for argument in ("--" + key.replace("_", "-"), value)]


def add_instance_arguments(parser):
    defaults = InstanceNames()
    for field in fields(defaults):
        parser.add_argument("--" + field.name.replace("_", "-"),
                            default=getattr(defaults, field.name))


def instance_from_args(args):
    return InstanceNames(**{field.name: getattr(args, field.name, field.default)
                            for field in fields(InstanceNames)})


def verify_report_instance(report, instance):
    """Legacy reports are accepted only for the original, unprefixed instance."""
    recorded = report.get("instance")
    if recorded is None:
        if instance != InstanceNames():
            raise RuntimeError("startup report has no instance identity for this namespaced arm")
    elif recorded != instance.as_dict():
        raise RuntimeError("startup report belongs to a different robot instance")

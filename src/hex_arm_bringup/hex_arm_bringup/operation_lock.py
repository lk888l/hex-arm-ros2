"""Local-process exclusion for stop clients addressing the same ROS interface."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path

from .integration import add_instance_arguments, instance_from_args


def lock_path(instance, role):
    identity = os.environ.get("ROS_DOMAIN_ID", "0") + ":" + instance.ros_name("hex_arm/deactivate_hardware")
    digest = hashlib.sha256(identity.encode()).hexdigest()
    directory = Path("/tmp") / f"hex-arm-operation-locks-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    return directory / f"{digest}.{role}.lock"


class OperationLock:
    def __init__(self, instance, role="stop", inherit=False):
        self.fd = None
        path = lock_path(instance, role)
        inherited = os.environ.get("HEX_ARM_OPERATION_LOCK_FD") if inherit else None
        if inherited is not None:
            candidate = int(inherited)
            existing, expected = os.fstat(candidate), path.stat()
            if (existing.st_dev, existing.st_ino) != (expected.st_dev, expected.st_ino):
                raise RuntimeError("inherited stop lock belongs to another arm")
            self.fd = os.dup(candidate)
            return
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(descriptor)
            raise RuntimeError(f"another {role} operation already owns {instance.ros_name('hex_arm')}") from None
        self.fd = descriptor

    def close(self):
        if self.fd is not None:
            # Close, rather than LOCK_UN: a child can share this open-file
            # description. Ownership ends only after both processes release it.
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def run_stop_script(main):
    parser = argparse.ArgumentParser(add_help=False)
    add_instance_arguments(parser)
    args, _ = parser.parse_known_args()
    with OperationLock(instance_from_args(args), inherit=True):
        return main()

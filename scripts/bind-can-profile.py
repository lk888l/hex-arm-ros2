#!/usr/bin/env python3
"""Bind an existing arm profile to an explicitly selected SocketCAN interface.

Reads sysfs only. Never sends CAN frames, configures a link, changes joint
calibration, or overwrites an existing profile.
"""
import argparse
import copy
from pathlib import Path
import re
import sys


def adapter_binding(interface, sys_class_net=Path("/sys/class/net")):
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", interface):
        raise ValueError("invalid SocketCAN interface name")
    netdev = sys_class_net / interface
    if (netdev / "type").read_text().strip() != "280":
        raise ValueError(f"{interface} is not a CAN interface")
    device = (netdev / "device").resolve(strict=True)
    driver = (device / "driver").resolve(strict=True).name
    channel = int((netdev / "dev_port").read_text().strip(), 10)
    dev_id = int((netdev / "dev_id").read_text().strip(), 0)
    usb = next((p for p in (device, *device.parents)
                if (p / "idVendor").is_file() and (p / "idProduct").is_file()), None)
    if usb is None:
        raise ValueError("CAN interface has no USB adapter ancestor")
    vid = int((usb / "idVendor").read_text().strip(), 16)
    pid = int((usb / "idProduct").read_text().strip(), 16)
    serial = (usb / "serial").read_text().strip()
    if driver != "gs_usb" or (vid, pid) != (0x1209, 0x2323):
        raise ValueError("selected interface is not a supported HexMeow Quad gs_usb adapter")
    if channel not in range(4) or dev_id != channel:
        raise ValueError("USB channel/dev_id mismatch")
    if not re.fullmatch(r"[0-9a-fA-F]{32}", serial):
        raise ValueError("adapter must report a complete 32-digit USB serial")
    return dict(driver=driver, vendor_id=vid, product_id=pid, serial=serial, channel=channel)


def bind_profile(source, interface, binding):
    result = copy.deepcopy(source)
    if (not isinstance(result, dict) or result.get("schema_version") != 3
            or result.get("joint_coordinate_version") != 2):
        raise ValueError("requires a schema v3 profile with joint_coordinate_version 2")
    bus = result.get("bus")
    if not isinstance(bus, dict) or bus.get("transport") != "socket_can":
        raise ValueError("requires a socket_can profile")
    if not isinstance(bus.get("expected_link"), dict):
        raise ValueError("profile is missing expected_link timing requirements")
    bus.update(interface=interface, channel=binding["channel"],
               adapter_vid=binding["vendor_id"], adapter_pid=binding["product_id"])
    bus["expected_link"]["adapter"] = copy.deepcopy(binding)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        if bool(args.profile) != bool(args.output):
            raise ValueError("--profile and --output must be provided together")
        binding = adapter_binding(args.interface)
        if args.profile:
            import yaml
            if not args.output.name.endswith(".local.yaml"):
                raise ValueError("--output must end in .local.yaml")
            result = bind_profile(yaml.safe_load(args.profile.read_text()), args.interface, binding)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write("# CAN binding selected from sysfs; motor identities and calibration retained.\n")
                yaml.safe_dump(result, stream, sort_keys=False)
            print(f"Created {args.output}: {args.interface}, channel {binding['channel']}, "
                  f"serial {binding['serial']}. No motor I/O.")
        else:
            # Consumed by docker-dev.sh without eval.
            print(binding["serial"], binding["channel"])
    except (OSError, ValueError) as error:
        parser.exit(2, f"error: {error}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

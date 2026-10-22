#!/usr/bin/env python3
"""Compatibility entry point; implementation belongs to the ROS package."""
from pathlib import Path
import sys

_package = Path(__file__).resolve().parents[1] / "src" / "hex_arm_bringup"
sys.path.insert(0, str(_package))
__file__ = str(_package / "scripts" / Path(__file__).name)
exec(compile(Path(__file__).read_bytes(), __file__, "exec"), globals())

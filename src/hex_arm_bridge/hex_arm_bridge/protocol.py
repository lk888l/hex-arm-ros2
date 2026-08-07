from __future__ import annotations

from collections.abc import Iterable, Sequence
import math


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
SUPPORTED_API_MAJOR = 0


def reorder(values: Sequence[float], names: Sequence[str], expected: Sequence[str] = JOINT_NAMES) -> list[float]:
    """Return values in the canonical order, rejecting ambiguous or bad input."""
    if len(values) != len(names) or len(names) != len(expected) or len(set(names)) != len(names):
        raise ValueError("command must contain each of the six joint names exactly once")
    source = dict(zip(names, values, strict=True))
    try:
        ordered = [float(source[name]) for name in expected]
    except KeyError as error:
        raise ValueError(f"missing joint {error.args[0]}") from error
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("joint values must be finite")
    return ordered


def optional_vector(values: Iterable[float], size: int, default: float) -> list[float]:
    result = [float(value) for value in values]
    if not result:
        return [default] * size
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"vector must be empty or contain {size} finite values")
    return result


def require_api_major(actual: int, expected: int = SUPPORTED_API_MAJOR) -> None:
    if actual != expected:
        raise RuntimeError(f"robot_api major mismatch: controller={actual}, bridge={expected}")


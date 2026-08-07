import math

import pytest

from hex_arm_bridge.protocol import JOINT_NAMES, optional_vector, reorder, require_api_major


def test_reorder_accepts_permutation() -> None:
    names = ["joint_6", "joint_1", "joint_4", "joint_2", "joint_5", "joint_3"]
    values = [6.0, 1.0, 4.0, 2.0, 5.0, 3.0]
    assert reorder(values, names) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


@pytest.mark.parametrize(
    ("values", "names"),
    [
        ([0.0] * 5, JOINT_NAMES[:5]),
        ([0.0] * 6, ["joint_1"] * 6),
        ([0.0] * 5 + [math.nan], JOINT_NAMES),
    ],
)
def test_reorder_rejects_incomplete_ambiguous_or_nonfinite(values, names) -> None:
    with pytest.raises(ValueError):
        reorder(values, names)


def test_optional_vector_contract() -> None:
    assert optional_vector([], 3, 2.0) == [2.0, 2.0, 2.0]
    assert optional_vector([1.0, 2.0, 3.0], 3, 0.0) == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError):
        optional_vector([1.0], 3, 0.0)


def test_api_major_is_strict() -> None:
    require_api_major(0)
    with pytest.raises(RuntimeError):
        require_api_major(1)


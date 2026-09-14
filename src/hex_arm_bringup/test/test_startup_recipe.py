import copy

import pytest

from hex_arm_bringup.startup_recipe import load_recipe, ready_position, validate_recipe


def test_ready_pose_is_derived_from_the_single_ordered_recipe():
    recipe = load_recipe()
    assert [step["joint_index"] for step in recipe["steps"]] == [1, 3, 2]
    assert ready_position(recipe) == [0.0, -1.35, 1.43, -0.3, 0.0, 0.0]


@pytest.mark.parametrize("key,value", [
    ("schema_version", 2), ("schema_version", True), ("stopped_velocity_rad_s", 0),
    ("stopped_velocity_rad_s", float("nan")), ("folded_position_rad", [0] * 5),
])
def test_invalid_recipe_is_rejected(key, value):
    recipe = copy.deepcopy(load_recipe())
    recipe[key] = value
    with pytest.raises(ValueError):
        validate_recipe(recipe)


def test_unknown_fields_reordering_and_nonfinite_targets_are_rejected():
    for mutate in (
        lambda r: r.update(extra=True),
        lambda r: r["steps"].reverse(),
        lambda r: r["steps"][0].update(target_rad=float("inf")),
        lambda r: r["steps"][0].update(duration_sec=-1),
    ):
        recipe = copy.deepcopy(load_recipe())
        mutate(recipe)
        with pytest.raises(ValueError):
            validate_recipe(recipe)

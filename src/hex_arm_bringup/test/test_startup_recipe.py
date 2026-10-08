import copy
import math

import pytest

from hex_arm_bringup.startup_recipe import (
    JOINT_NAMES, load_recipe, ready_position, resolve_motion_limits,
    rest_to_rest_duration, validate_recipe,
)


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


def motion_fixture():
    profile = {"joints": [
        {"name": name, "limits": {"velocity_rad_s": .2 * math.tau,
                                  "acceleration_rad_s2": .2 * math.tau}}
        for name in JOINT_NAMES
    ]}
    planning = {name: {"velocity_rad_s": .2 * math.tau, "acceleration_rad_s2": .6}
                for name in JOINT_NAMES}
    return profile, planning


@pytest.mark.parametrize("distance", [.0003, .14, .22, .3, 2.7, 6.0])
def test_timed_quintic_obeys_normal_caps_over_the_entire_curve(distance):
    profile, planning = motion_fixture()
    rates = resolve_motion_limits(profile, planning)
    seconds = rest_to_rest_duration([0.] * 6, [0.] * 5 + [distance], rates)
    dt = seconds / 2000
    # Finite differences also cover peaks between trajectory endpoints.
    q = [distance * (10*u**3 - 15*u**4 + 6*u**5)
         for u in (sample / 2000 for sample in range(2001))]
    velocities = [(b-a) / dt for a, b in zip(q, q[1:])]
    accelerations = [(b-a) / dt for a, b in zip(velocities, velocities[1:])]
    assert max(abs(v) for v in velocities) <= planning['joint_6']['velocity_rad_s']
    assert max(abs(a) for a in accelerations) <= planning['joint_6']['acceleration_rad_s2']
    if distance >= .14:
        assert max(max(abs(v) for v in velocities) / planning['joint_6']['velocity_rad_s'],
                   max(abs(a) for a in accelerations) / planning['joint_6']['acceleration_rad_s2']) > .99


def test_startup_waypoints_use_deployment_acceleration_instead_of_fixed_slow_times():
    profile, planning = motion_fixture()
    rates = resolve_motion_limits(profile, planning)
    recipe = load_recipe()
    previous = recipe['folded_position_rad'].copy()
    durations = []
    for step in recipe['steps']:
        target = previous.copy(); target[step['joint_index']] = step['target_rad']
        durations.append(rest_to_rest_duration(previous, target, rates))
        previous = target
    assert durations == pytest.approx([1.455, 1.699, 1.161], abs=.001)
    assert sum(durations) < 5.


@pytest.mark.parametrize("fault", ["missing_joint", "missing_rate", "zero", "negative",
                                   "nan", "inf", "bool", "hardware_exceeded"])
def test_invalid_or_excessive_planning_rates_are_rejected(fault):
    profile, planning = motion_fixture()
    if fault == 'missing_joint': planning.pop('joint_6')
    elif fault == 'missing_rate': planning['joint_2'].pop('acceleration_rad_s2')
    else:
        planning['joint_2']['acceleration_rad_s2'] = {
            'zero': 0, 'negative': -1, 'nan': math.nan, 'inf': math.inf,
            'bool': True, 'hardware_exceeded': 2.,
        }[fault]
    with pytest.raises(ValueError):
        resolve_motion_limits(profile, planning)


def test_motion_duration_rejects_invalid_positions_and_excessive_time():
    profile, planning = motion_fixture()
    rates = resolve_motion_limits(profile, planning)
    for target in ([0.] * 5, [math.nan] * 6):
        with pytest.raises(ValueError):
            rest_to_rest_duration([0.] * 6, target, rates)
    rates['joint_6']['velocity_rad_s'] = .001
    with pytest.raises(ValueError, match='120 second'):
        rest_to_rest_duration([0.] * 6, [0.] * 5 + [1.], rates)

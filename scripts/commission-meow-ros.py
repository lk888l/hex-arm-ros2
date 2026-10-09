#!/usr/bin/env python3
"""Verify bounded Meow MoveIt round trips, or return a verified startup to folded.

Run against the active supervised launch that produced --startup-report.
This client never enables hardware. A failure cancels motion and requests disable.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time

import rclpy
import yaml

spec = importlib.util.spec_from_file_location(
    'shutdown_client', Path(__file__).with_name('commission-shutdown-ros.py'))
shutdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shutdown)
startup = shutdown.startup


def return_steps(profile, prior, digest):
    """Only reverse a successful, exact-profile, standard Meow startup."""
    if (profile.get('bus', {}).get('protocol') != 'meow'
            or not profile.get('validated') or not profile.get('calibrated')
            or prior.get('passed') is not True or prior.get('deactivated')
            or 'ready_hold' not in prior or prior.get('profile_sha256') != digest
            or not isinstance(prior.get('motion_limits'), dict)):
        raise RuntimeError('requires this active launch\'s successful, exact-profile Meow startup')
    records = prior.get('steps', [])
    if not records:
        raise RuntimeError('startup evidence contains no steps')
    expected = startup.startup_steps(
        profile, records[0]['initial'], [0.] * 6,
        align_folded=records[0]['step'] == 'align_folded',
        motion_limits=prior.get('motion_limits'))
    if len(records) != len(expected):
        raise RuntimeError('startup evidence does not match the standard sequence')
    for index, (record, (label, target, duration)) in enumerate(zip(records, expected)):
        if (record.get('status') != 4 or record.get('error_code') != 0
                or record.get('step') != label or record.get('duration_sec') != duration
                or record.get('target') != target):
            raise RuntimeError('startup evidence does not match the standard sequence')
        reference = (records[index-1]['target'] if index else
                     startup.folded_entry_command(profile, record['initial']))
        if record.get('command_reference', reference) != reference:
            raise RuntimeError('startup evidence contains a changed command reference')
    forward = records[-3:]
    rates = startup.resolve_motion_limits(profile, prior.get('motion_limits'))
    folded = forward[0].get('command_reference')
    if folded is None:
        folded = startup.folded_entry_command(profile, forward[0]['initial'])
    return [(f"return_{forward[i]['step']}",
             forward[i-1]['target'] if i else folded,
             startup.rest_to_rest_duration(
                 forward[i]['target'],
                 forward[i-1]['target'] if i else folded, rates))
            for i in (2, 1, 0)]


def warm_connections(node, check_live=True):
    # Endpoint discovery can block callbacks. Finish it before the monitored
    # phase, then drain queued messages and require fresh joint/driver feedback.
    for endpoint in (node.hardware, node.switch, node.set_hardware, node.validity):
        if not endpoint.wait_for_service(timeout_sec=5.):
            raise RuntimeError('required ROS service unavailable')
    for endpoint in (node.fjt, node.move_group, node.trajectory_executor):
        if not endpoint.wait_for_server(timeout_sec=5.):
            raise RuntimeError('required ROS action unavailable')
    deadline = time.monotonic() + 1.
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    if check_live:
        node.check_live()


def already_inactive(node, components):
    """Accept an explicit prior stop; the supervisor still verifies final disable."""
    driver = node.driver
    now = time.monotonic()
    return (
        any(c.name == 'FireflyY6System' and 'hex_arm_hardware' in c.plugin_name
            and c.state.id == 2 for c in components)
        and now - node.received_at <= 0.2 and now - node.driver_at <= 0.2
        and len(node.positions) == 6 and len(node.velocity) == 6
        and all(math.isfinite(v) for v in [*node.q(), *node.velocity.values()])
        and driver is not None and driver.mode == 1 and not driver.session_owned
        and driver.profile_valid and driver.calibrated and driver.all_motors_online
        and driver.feedback_fresh and not driver.fault_latched
    )


def campaign_goals(campaign, profile, digest):
    """Bind a finite list of absolute goals to this arm's exact test profile."""
    if (campaign.get('schema_version') != 1
            or campaign.get('profile_sha256') != digest):
        raise RuntimeError('campaign requires schema 1 and the exact profile SHA256')
    goals = campaign.get('goals', [])
    if not isinstance(goals, list) or not 1 <= len(goals) <= 100:
        raise RuntimeError('campaign requires 1..100 goals')
    checked = []
    for entry in goals:
        q, hold = entry.get('position_rad', []), entry.get('hold_sec', 1.)
        if (len(q) != 6 or any(isinstance(x, bool) or not isinstance(x, (int, float))
                              or not math.isfinite(x) for x in q)
                or isinstance(hold, bool) or not isinstance(hold, (int, float))
                or not math.isfinite(hold) or not 0.5 <= hold <= 60):
            raise RuntimeError('campaign requires finite six-axis goals and holds in [0.5,60] s')
        for x, joint in zip(q, profile['joints']):
            limits = joint['limits']
            if not limits['position_lower_rad'] <= x <= limits['position_upper_rad']:
                raise RuntimeError(f"campaign exceeds {joint['name']} profile limits")
        checked.append((str(entry.get('label', len(checked))), list(q), float(hold)))
    # Every campaign finishes at the commissioned entry to the folded return.
    if checked[-1][1] != startup.READY:
        checked.append(('return_ready', startup.READY.copy(), 1.))
    return checked


def planning_scaling(value):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 1:
        raise argparse.ArgumentTypeError("planning scaling must be finite and within (0,1]")
    return value


def run(node, reverse, test_moveit, report, goals=None, velocity_scaling=1.0, acceleration_scaling=1.0):
    warm_connections(node, check_live=False)
    components = node.list_hardware_components()
    if already_inactive(node, components):
        if goals or test_moveit:
            raise RuntimeError('campaign/tests require active real hardware')
        report.update(passed=True, deactivated=True, already_inactive=True)
        print('Real hardware already INACTIVE and driver DISABLED; no further motion', flush=True)
        return
    node.check_live()
    node.monitor_motion = True
    if not any(c.name == 'FireflyY6System' and 'hex_arm_hardware' in c.plugin_name
               and c.state.label == 'active' for c in components):
        raise RuntimeError('requires active real FireflyY6System')
    node.shutdown_path_tolerance_rad = (
        None if goals or test_moveit else shutdown.SHUTDOWN_PATH_TOLERANCE_RAD)
    report['shutdown_path_tolerance_rad'] = shutdown.SHUTDOWN_PATH_TOLERANCE_RAD
    node.wait_stationary()
    if any(abs(q-t) > startup.HOLD_POSITION_TOLERANCE_RAD
           for q, t in zip(node.q(), startup.READY)):
        report['return_to_ready'] = node.plan_and_execute(
            startup.READY, shutdown.SHUTDOWN_RETURN_VELOCITY_SCALING,
            shutdown.SHUTDOWN_RETURN_ACCELERATION_SCALING)
    node.wait_stationary(startup.READY)
    # Fixed startup/re-entry alone traverses the known folded contacts. Every
    # MoveIt plan and its execution retain the strict collision model.
    previous = startup.READY.copy()
    for label, target, _ in reverse:
        for k in range(81):
            q = [x+(y-x)*k/80 for x, y in zip(previous, target)]
            node.check_point(q)
            valid = node.is_valid(q, startup.FOLDED_CONTACTS)
            node.sequence_path_checks.append({'step': label, 'q': q, 'strict_valid': valid})
        previous = target
    if goals:
        # Reject every requested endpoint before the first campaign motion.
        # The full planned path is independently checked before each execution.
        for _, target, _ in goals:
            node.check_point(target)
            node.is_valid(target)
        for label, target, hold in goals:
            record = {'label': label, 'target': target, 'passed': False}
            report['moveit'].append(record)
            record.update(node.plan_and_execute(target, velocity_scaling, acceleration_scaling))
            node.wait_stationary(target)
            errors, velocity = node.spin_hold(target, hold, len(node.samples))
            record.update(passed=True, hold_sec=hold, max_error_rad=errors,
                          max_velocity_rad_s=velocity)
            print(json.dumps(record), flush=True)
    elif test_moveit:
        for axis, delta in enumerate([-.02, .015, -.015, -.015, -.02, .015]):
            target = startup.READY.copy()
            target[axis] += delta
            for goal in (target, startup.READY):
                record = node.plan_and_execute(goal, velocity_scaling, acceleration_scaling)
                node.wait_stationary(goal)
                report['moveit'].append(record)
                print(json.dumps(record), flush=True)
        errors, velocity = node.spin_hold(startup.READY, 10., len(node.samples))
        report['ready_hold'] = {'max_error_rad': errors, 'max_velocity_rad_s': velocity}
    node.shutdown_path_tolerance_rad = shutdown.SHUTDOWN_PATH_TOLERANCE_RAD
    for label, target, duration in reverse:
        node.direct_step(target, duration, label, retime=True)
        node.is_valid(node.q(), startup.FOLDED_CONTACTS)
    node.wait_stationary(reverse[-1][1])
    report['folded_q'] = node.q()
    node.monitor_motion = False
    node.deactivate()
    report.update(passed=True, deactivated=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--startup-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-motion', action='store_true', required=True)
    parser.add_argument('--velocity-scaling', type=planning_scaling, default=1.0,
                        help='Campaign/test MoveIt velocity fraction of effective launch limits (default: 1)')
    parser.add_argument('--acceleration-scaling', type=planning_scaling, default=1.0,
                        help='Campaign/test MoveIt acceleration fraction of effective launch limits (default: 1)')
    parser.add_argument('--record-commands', action='store_true',
                        help='Record the ROS command stream for tracking diagnostics')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--test-moveit', action='store_true',
                        help='Test each joint out/back before controlled folded return')
    mode.add_argument('--campaign', type=Path,
                      help='Exact-profile JSON goals, strict MoveIt execution, then folded return')
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    prior = json.loads(args.startup_report.read_text())
    digest = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    reverse = return_steps(profile, prior, digest)
    goals = campaign_goals(json.loads(args.campaign.read_text()), profile, digest) if args.campaign else None
    rclpy.init()
    node = shutdown.StopProbe(profile, record_commands=args.record_commands,
                              motion_limits=prior.get('motion_limits'))
    report = {'passed': False, 'profile_sha256': digest,
              'startup_report': str(args.startup_report), 'moveit': [],
              'motion_limits': node.motion_limits,
              'velocity_scaling': args.velocity_scaling, 'acceleration_scaling': args.acceleration_scaling,
              'hold_position_tolerance_rad': startup.HOLD_POSITION_TOLERANCE_RAD,
              'hold_velocity_peak_rad_s': startup.HOLD_VELOCITY_PEAK_RAD_S}
    if args.campaign:
        report.update(campaign=str(args.campaign),
                      campaign_sha256=hashlib.sha256(args.campaign.read_bytes()).hexdigest(),
                      requested_goals=goals)
    try:
        run(node, reverse, args.test_moveit, report, goals, args.velocity_scaling, args.acceleration_scaling)
    except BaseException as error:
        report['error'] = str(error)
        report['failed_plan'] = getattr(node, 'last_plan_diagnostics', None)
        report['failure_feedback_age_s'] = {
            'joint': time.monotonic()-node.received_at, 'driver': time.monotonic()-node.driver_at}
        node.monitor_motion = False
        if node.active_goal is not None:
            try:
                node.wait(node.active_goal.cancel_goal_async(), 3.)
            except Exception as cancel_error:
                report['cancel_error'] = str(cancel_error)
        try:
            node.deactivate()
            report['deactivated'] = True
        except Exception as stop_error:
            report['deactivation_error'] = str(stop_error)
        raise
    finally:
        report.update(samples=node.samples, steps=node.steps, action_feedback=node.action_feedback,
                      sequence_path_checks=node.sequence_path_checks)
        if args.record_commands:
            report['commands'] = node.commands
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

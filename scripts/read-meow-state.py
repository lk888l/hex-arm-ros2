#!/usr/bin/env python3
"""Read Meow identity, disabled state and encoder samples over SocketCAN.

Run with all controller owners stopped. Sends expedited SDO uploads only;
never writes motor objects, enables drives, or changes local calibration.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import struct
import time

import yaml


IDENTITY_KEYS = ("vendor_id", "product_code", "revision", "serial_number")


def upload(bus, node, index, subindex=0, timeout=0.5):
    if not 1 <= node <= 127 or not 0 < timeout <= 5:
        raise ValueError("invalid SDO node or timeout")
    request = struct.pack('<BHB4x', 0x40, index, subindex)
    bus.send(struct.pack('=IB3x8s', 0x600 + node, 8, request))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus.settimeout(max(0.001, deadline - time.monotonic()))
        frame = bus.recv(72)
        if len(frame) < 16:
            continue
        can_id, length = struct.unpack_from('=IB', frame)
        if can_id != 0x580 + node or length != 8:
            continue
        data = frame[8:16]
        if data[1:4] != request[1:4]:
            continue
        if data[0] == 0x80:
            raise RuntimeError(f'node {node} SDO {index:04x}:{subindex:02x} abort {data[4:8].hex()}')
        if data[0] not in (0x43, 0x47, 0x4b, 0x4f):
            raise RuntimeError(f'unsupported SDO response {data.hex()}')
        size = 4 - ((data[0] >> 2) & 3)
        return data[4:4 + size]
    raise TimeoutError(f'node {node} SDO {index:04x}:{subindex:02x} timed out')


def read_value(bus, node, index, subindex, fmt):
    data = upload(bus, node, index, subindex)
    if len(data) != struct.calcsize(fmt):
        raise RuntimeError(f"node {node} SDO {index:04x}:{subindex:02x} unexpected size {len(data)}")
    return struct.unpack(fmt, data)[0]


def observe(bus, node, samples, factory_calibration=False):
    uint = lambda index, sub=0, fmt="<I": read_value(bus, node, index, sub, fmt)
    identity = {key: uint(0x1018, sub) for sub, key in enumerate(IDENTITY_KEYS, 1)}
    record = dict(node=node, identity=identity, heartbeat_consumer=uint(0x1016, 1),
                  modes=[], error_codes=[], samples_rev=[])
    if factory_calibration:
        record['factory_highest_subindex'] = uint(0x4001, fmt="<B")
        record['factory_words_u32'] = [uint(0x4001, sub) for sub in range(1, 8)]
    for sample in range(samples + 1):
        record['modes'].append(uint(0x4402, fmt="<B"))
        record['error_codes'].append(uint(0x453f, fmt="<H"))
        if sample < samples:
            record['samples_rev'].append(read_value(bus, node, 0x4564, 0, "<i") / 2**24)
        if sample < samples - 1:
            time.sleep(0.07)
    if identity != {key: uint(0x1018, sub) for sub, key in enumerate(IDENTITY_KEYS, 1)}:
        raise RuntimeError(f'node {node} identity changed during capture')
    record.update(mode=record['modes'][0], final_mode=record['modes'][-1],
                  error=record['error_codes'][0], error_code=record['error_codes'][0],
                  final_heartbeat_consumer=uint(0x1016, 1),
                  disabled=all(mode == 0 for mode in record['modes']),
                  position_rev=sorted(record['samples_rev'])[samples // 2],
                  span_rad=(max(record['samples_rev'])-min(record['samples_rev']))*math.tau)
    return record


def verify_disabled(report):
    nodes = report.get('nodes', [])
    if (report.get('protocol') != 'meow' or report.get('passed') is not True
            or report.get('errors') or len(nodes) != 6
            or len({node.get('node') for node in nodes}) != 6):
        raise RuntimeError('independent six-axis Meow state/identity read failed')
    for node in nodes:
        samples = node.get('samples_rev', [])
        modes, errors = node.get('modes', []), node.get('error_codes', [])
        if (not node.get('disabled') or len(samples) < 2
                or len(modes) != len(samples) + 1 or any(mode != 0 for mode in modes)
                or len(errors) != len(samples) + 1 or any(errors)
                or node.get('heartbeat_consumer') != 0 or node.get('final_heartbeat_consumer') != 0):
            raise RuntimeError(f"node {node.get('node')}: disable/heartbeat release unconfirmed")
        if any(isinstance(raw, bool) or not isinstance(raw, (int, float))
               or not math.isfinite(raw) or not -128 <= raw < 128 for raw in samples):
            raise RuntimeError('invalid Meow Q8.24 encoder samples')
        span = (max(samples) - min(samples)) * math.tau
        if span > 0.001 or node.get('span_rad') != span:
            raise RuntimeError(f"node {node.get('node')}: arm is not stationary")
        if node.get('position_rev') != sorted(samples)[len(samples) // 2]:
            raise RuntimeError('encoder median does not match samples')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=7)
    parser.add_argument("--factory-calibration", action="store_true",
                        help="Also save raw 0x4001 factory words; validity remains the driver decoder's responsibility")
    args = parser.parse_args(argv)
    if not 2 <= args.samples <= 100:
        parser.error('--samples must be within 2..100')
    if args.output.exists() or args.output.is_symlink() or args.output.resolve() == args.profile.resolve():
        parser.error('output must be new and must not replace the profile')
    profile = yaml.safe_load(args.profile.read_text())
    if (profile['bus'].get('protocol', 'meow') != 'meow'
            or profile['bus'].get('transport', 'socket_can') != 'socket_can'
            or args.interface != profile['bus']['interface']):
        raise RuntimeError('requires a Meow profile bound to the selected interface')
    report = {'time': datetime.now(timezone.utc).isoformat(), 'interface': args.interface,
              'protocol': 'meow', 'profile': str(args.profile),
              'profile_sha256': hashlib.sha256(args.profile.read_bytes()).hexdigest(),
              'nodes': [], 'errors': [], 'passed': False}
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            mask = socket.CAN_SFF_MASK | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
            bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                           b''.join(struct.pack('=II', 0x580 + j['node_id'], mask) for j in profile['joints']))
            bus.bind((args.interface,))
            for joint in profile['joints']:
                node = joint['node_id']
                try:
                    record = observe(bus, node, args.samples, args.factory_calibration)
                    record['q_using_previous_zero_rad'] = (joint['direction']*math.tau*record['position_rev']
                                                          + joint['zero_offset_rad'])
                    report['nodes'].append(record)
                    if record['identity'] != {k: joint['identity'][k] for k in IDENTITY_KEYS}:
                        raise RuntimeError(f'node {node} identity mismatch')
                except (OSError, RuntimeError, ValueError) as error:
                    report['errors'].append(str(error))
        report['passed'] = not report['errors']
        verify_disabled(report)
    except (OSError, RuntimeError, ValueError) as error:
        report['errors'].append(str(error))
        report['passed'] = False
    with args.output.open('x') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Read Meow identity, disabled state and encoder samples over SocketCAN.

Run with all controller owners stopped. Sends expedited SDO uploads only;
never writes motor objects, enables drives, or changes local calibration.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import struct
import time

import yaml


def upload(bus, node, index, subindex=0):
    request = struct.pack('<BHB4x', 0x40, index, subindex)
    bus.send(struct.pack('=IB3x8s', 0x600 + node, 8, request))
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        bus.settimeout(max(0.001, deadline - time.monotonic()))
        frame = bus.recv(72)
        can_id, length = struct.unpack_from('=IB', frame)
        if can_id != 0x580 + node or length < 8:
            continue
        data = frame[8:16]
        if data[1:4] != request[1:4]:
            continue
        if data[0] == 0x80:
            raise RuntimeError(f'node {node} SDO {index:04x}:{subindex:02x} abort {data[4:8].hex()}')
        if data[0] & 0xe3 != 0x43:
            raise RuntimeError(f'unsupported SDO response {data.hex()}')
        size = 4 - ((data[0] >> 2) & 3)
        return data[4:4 + size]
    raise TimeoutError(f'node {node} SDO {index:04x}:{subindex:02x} timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    if profile['bus']['protocol'] != 'meow' or args.interface != profile['bus']['interface']:
        raise RuntimeError('requires a Meow profile bound to the selected interface')
    report = {'time': datetime.now(timezone.utc).isoformat(), 'interface': args.interface,
              'profile': str(args.profile), 'nodes': []}
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        bus.bind((args.interface,))
        for joint in profile['joints']:
            node = joint['node_id']
            uint = lambda index, sub=0: int.from_bytes(upload(bus, node, index, sub), 'little')
            identity = [uint(0x1018, i) for i in range(1, 5)]
            expected = joint['identity']
            if identity != [expected[k] for k in ('vendor_id','product_code','revision','serial_number')]:
                raise RuntimeError(f'node {node} identity mismatch: {identity}')
            record = {'node': node, 'identity': identity, 'mode': uint(0x4402),
                      'error': uint(0x453f), 'heartbeat_consumer': uint(0x1016, 1),
                      'samples_rev': []}
            for i in range(7):
                record['samples_rev'].append(int.from_bytes(upload(bus,node,0x4564), 'little', signed=True) / 2**24)
                if i < 6:
                    time.sleep(0.07)
            raw = sorted(record['samples_rev'])[3]
            record.update(position_rev=raw, span_rad=(max(record['samples_rev'])-min(record['samples_rev']))*math.tau,
                          q_using_previous_zero_rad=joint['direction']*math.tau*raw+joint['zero_offset_rad'])
            report['nodes'].append(record)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()

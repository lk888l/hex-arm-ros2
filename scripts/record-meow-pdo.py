#!/usr/bin/env python3
"""Passively record Meow MIT target/feedback PDOs alongside a supervised trial.

No CAN transmission or ROS participant is created. Each JSONL sample contains
[host_monotonic_seconds, CAN_ID, payload_hex]; the first/last lines are metadata.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text())
    if profile['bus']['protocol'] != 'meow' or args.interface != profile['bus']['interface']:
        parser.error('requires a Meow profile bound to the selected interface')
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    nodes = [j['node_id'] for j in profile['joints']]
    filters = [base + node for base in (0x180, 0x200, 0x280) for node in nodes]
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        # Match standard data frames; CAN_ERR_FLAG in a filter mask selects errors.
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       b''.join(struct.pack('=II', can_id, 0xC00007FF) for can_id in filters))
        bus.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        bus.bind((args.interface,))
        bus.settimeout(0.2)
        with args.output.open('x') as output:
            output.write(json.dumps({'time': datetime.now(timezone.utc).isoformat(),
                'interface': args.interface, 'profile': str(args.profile),
                'joints': [{k: j[k] for k in ('name', 'node_id', 'direction', 'zero_offset_rad')}
                           for j in profile['joints']]}) + '\n')
            output.flush()
            deadline = time.monotonic() + args.seconds
            count = 0
            print(f'Passive CAN recording ready: {args.output}', flush=True)
            try:
                while time.monotonic() < deadline:
                    try:
                        frame = bus.recv(72)
                    except TimeoutError:
                        continue
                    can_id, length = struct.unpack_from('=IB', frame)
                    output.write(json.dumps([time.monotonic(), can_id,
                        frame[8:8 + length].hex()], separators=(',', ':')) + '\n')
                    count += 1
            finally:
                output.write(json.dumps({'frames': count, 'ended_at': time.monotonic()}) + '\n')
                print(f'Passive CAN recording stopped: {count} frames', flush=True)


if __name__ == '__main__':
    main()

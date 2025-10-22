"""
CAN Node ID scanner for RobStride RS02 using Private protocol GET_ID (type 0x00).

Sends GET_ID to motor IDs in range 0x00..0x7F and listens for GET_ID responses.
Prints any discovered motor CAN IDs and their 64-bit unique IDs.

Usage notes:
- Requires python-can installed and a SocketCAN interface (e.g., can0) up at 1Mbps.
- Run as root or ensure the user has access to CAN device.
- For testing, use vcan0 instead of can0.
"""

import time
from typing import List, Tuple

import can

from robstride.protocol.private import PrivateProtocolHandler
from robstride.models import MotorStatus


def scan_can_ids(can_interface: str = "can0", master_id: int = 0xFD, ids: range = range(0x00, 0x80), timeout_s: float = 0.5) -> List[Tuple[int, int]]:
    """Scan IDs and return list of tuples (motor_can_id, unique_uid).

    Args:
        can_interface: SocketCAN interface name
        master_id: Host/master CAN ID to use when building requests
        ids: Iterable of motor IDs to probe
        timeout_s: Per-ID wait timeout in seconds
    """
    discovered = []

    try:
        bus = can.interface.Bus(channel=can_interface, bustype="socketcan", bitrate=1_000_000)
    except Exception as exc:
        raise SystemExit(f"Failed to open CAN interface '{can_interface}': {exc}") from exc

    try:
        handler = None
        print(f"Scanning {can_interface} for motor IDs {hex(ids.start)}..{hex(ids.stop-1)} using master ID 0x{master_id:02X}")

        for candidate in ids:
            handler = PrivateProtocolHandler(candidate, bus, master_id)

            # Send GET_ID
            if not handler.send_get_id():
                print(f"Warning: failed to send GET_ID to 0x{candidate:02X}")
                continue

            # Listen for response for a short window
            deadline = time.monotonic() + timeout_s
            status = MotorStatus()

            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.05)
                if msg is None:
                    continue

                processed = handler.process_message(msg, status, None)
                if not processed:
                    continue

                if status.device_uid is not None:
                    discovered.append((status.device_id, status.device_uid))
                    print(f"Found motor: CAN ID 0x{status.device_id:02X}, UID 0x{status.device_uid:016X}")
                    break

        if not discovered:
            print("No motors discovered on the bus. Check power, wiring and CAN interface settings.")

        return discovered

    finally:
        bus.shutdown()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Scan CAN bus for RobStride RS02 motor IDs using Private protocol GET_ID")
    parser.add_argument('--interface', '-i', default='can0', help='SocketCAN interface (default: can0)')
    parser.add_argument('--master', '-m', type=lambda x: int(x, 0), default=0xFD, help='Master/host ID to use (default: 0xFD)')
    parser.add_argument('--timeout', '-t', type=float, default=0.5, help='Per-ID response timeout seconds (default: 0.5)')
    parser.add_argument('--start', type=lambda x: int(x, 0), default=0x00, help='Start CAN ID (inclusive)')
    parser.add_argument('--end', type=lambda x: int(x, 0), default=0x7F, help='End CAN ID (inclusive)')

    args = parser.parse_args()

    ids_range = range(args.start, args.end + 1)
    found = scan_can_ids(can_interface=args.interface, master_id=args.master, ids=ids_range, timeout_s=args.timeout)

    print('\nScan complete. Summary:')
    for cid, uid in found:
        print(f" - CAN ID 0x{cid:02X}  UID 0x{uid:016X}")
